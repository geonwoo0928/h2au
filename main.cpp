#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>
#include <opencv2/opencv.hpp>

#include "Camera.h"
#include "GpsController.h"
#include "I2cDevice.h"
#include "ImuController.h"
#include "MotorController.h"
#include "PwmController.h"
#include "RcCarDataManager.h"
#include "SeoilCoordController.h"
#include "ServoController.h"
#include "TerminalInput.h"

struct ControlCommand
{
    double speed = 0.0;
    double steeringAngle = 0.0;
    double cameraPan = 0.0;
    double cameraTilt = 0.0;
};

int main()
{
    try
    {
        I2cDevice pwmI2c("/dev/i2c-1", 0x14);
        I2cDevice imuI2c("/dev/i2c-1", 0x28);

        PwmController pwm(pwmI2c);
        ServoController servo(pwm);
        MotorController motor(pwm);
        ImuController imu(imuI2c);

        RcCarDataManager dataManager;
        SeoilCoordController coordController;
        GpsController gps("/dev/serial0", dataManager);

        imu.initialize();

        servo.setCalibration(0, {-90.0, 90.0, 0.0, false, 500.0, 2500.0});
        servo.setCalibration(1, {-45.0, 45.0, 0.0, false, 500.0, 2500.0});
        servo.setCalibration(2, {-30.0, 30.0, 0.0, false, 500.0, 2500.0});

        servo.center(0);
        servo.center(1);
        servo.center(2);
        motor.stop();

        TerminalInput keyboard;
        Camera camera(640, 480, 30);

        std::atomic<bool> running{true};

        ControlCommand command;

        auto path = dataManager.getRcCarPath();
        cv::Mat satelliteImg = coordController.drawPathOnSatelliteImg(path);
        std::mutex mapMutex;

        std::thread gpsThread;

        auto requestStop = [&]()
        {
            running.store(false);
            gps.stopThread();
        };

        auto joinThreads = [&]()
        {
            if (gpsThread.joinable())
                gpsThread.join();
        };

        try
        {
            gpsThread = std::thread([&]()
                                    {
                try
                {
                    gps.runGpsThread(
                        coordController,
                        [&]()
                        {
                            auto path = dataManager.getRcCarPath();
                            cv::Mat newMap = coordController.drawPathOnSatelliteImg(path);

                            {
                                std::lock_guard<std::mutex> lock(mapMutex);
                                satelliteImg = newMap;
                            }
                        }
                    );
                }
                catch (const std::exception &e)
                {
                    std::cerr << "[GPS THREAD ERROR] " << e.what() << '\n';
                    requestStop();
                }
                catch (...)
                {
                    std::cerr << "[GPS THREAD ERROR] Unknown error\n";
                    requestStop();
                } });

            std::cout
                << "==========================================\n"
                << " [조작 가이드]\n"
                << " W/S : 전진/후진\n"
                << " A/D : 좌/우 조향\n"
                << " I/K : 카메라 위/아래\n"
                << " J/L : 카메라 좌/우\n"
                << " R   : 카메라 리셋\n"
                << " Space : 정지\n"
                << " Q/ESC : 종료\n"
                << "==========================================\n";

            cv::Mat frame;
            constexpr double speedSetting = 40.0;
            auto lastPrint = std::chrono::steady_clock::now();

            while (running.load())
            {
                ImuData imuData = imu.read();
                dataManager.updateYaw(imuData.heading);

                if (!camera.read(frame) || frame.empty())
                {
                    std::cerr << "[ERROR] Failed to read camera frame.\n";
                    requestStop();
                    break;
                }

                cv::imshow("Robot Camera", frame);

                cv::Mat mapToShow;

                {
                    std::lock_guard<std::mutex> lock(mapMutex);

                    if (!satelliteImg.empty())
                        mapToShow = satelliteImg.clone();
                }

                if (!mapToShow.empty())
                    cv::imshow("Satellite Map", mapToShow);

                auto now = std::chrono::steady_clock::now();

                int cvKey = cv::waitKey(1);

                if (cvKey == 27)
                {
                    requestStop();
                    break;
                }

                int key = keyboard.readKey();

                if (key < 0)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    continue;
                }

                switch (key)
                {
                case 'w':
                case 'W':
                    command.speed = speedSetting;
                    motor.drive(command.speed);
                    break;

                case 's':
                case 'S':
                    command.speed = -speedSetting;
                    motor.drive(command.speed);
                    break;

                case ' ':
                    command.speed = 0.0;
                    motor.stop();
                    break;

                case 'a':
                case 'A':
                    command.steeringAngle -= 5.0;

                    if (command.steeringAngle < -30.0)
                        command.steeringAngle = -30.0;

                    servo.setAngle(2, command.steeringAngle);
                    break;

                case 'd':
                case 'D':
                    command.steeringAngle += 5.0;

                    if (command.steeringAngle > 30.0)
                        command.steeringAngle = 30.0;

                    servo.setAngle(2, command.steeringAngle);
                    break;

                case 'i':
                case 'I':
                    command.cameraTilt += 5.0;

                    if (command.cameraTilt > 45.0)
                        command.cameraTilt = 45.0;

                    servo.setAngle(1, command.cameraTilt);
                    break;

                case 'k':
                case 'K':
                    command.cameraTilt -= 5.0;

                    if (command.cameraTilt < -45.0)
                        command.cameraTilt = -45.0;

                    servo.setAngle(1, command.cameraTilt);
                    break;

                case 'j':
                case 'J':
                    command.cameraPan -= 5.0;

                    if (command.cameraPan < -90.0)
                        command.cameraPan = -90.0;

                    servo.setAngle(0, command.cameraPan);
                    break;

                case 'l':
                case 'L':
                    command.cameraPan += 5.0;

                    if (command.cameraPan > 90.0)
                        command.cameraPan = 90.0;

                    servo.setAngle(0, command.cameraPan);
                    break;

                case 'r':
                case 'R':
                    command.cameraPan = 0.0;
                    command.cameraTilt = 0.0;

                    servo.center(0);
                    servo.center(1);
                    break;

                case 'q':
                case 'Q':
                    requestStop();
                    break;
                }
            }

            requestStop();
            joinThreads();

            try
            {
                motor.stop();
            }
            catch (...)
            {
            }

            try
            {
                servo.center(0);
                servo.center(1);
                servo.center(2);
            }
            catch (...)
            {
            }
        }
        catch (...)
        {
            requestStop();
            joinThreads();
            throw;
        }

        cv::destroyAllWindows();
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }
    catch (...)
    {
        std::cerr << "Unknown error\n";
        return 1;
    }

    return 0;
}