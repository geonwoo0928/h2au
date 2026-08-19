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
#include "PersonDetector.h"

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
        // ================= HARDWARE / CONTROLLER =================

        I2cDevice pwmI2c("/dev/i2c-1", 0x14);
        I2cDevice imuI2c("/dev/i2c-1", 0x28);

        PwmController pwm(pwmI2c);
        ServoController servo(pwm);
        MotorController motor(pwm);
        ImuController imu(imuI2c);

        RcCarDataManager dataManager;
        SeoilCoordController coordController;

        GpsController gps("172.20.10.1", 11123, dataManager);

        // ================= IMU 초기화 =================

        imu.initialize();

        // ================= SERVO 초기화 =================

        servo.setCalibration(
            0,
            {-90.0, 90.0, 0.0, false, 500.0, 2500.0});

        servo.setCalibration(
            1,
            {-45.0, 45.0, 0.0, false, 500.0, 2500.0});

        servo.setCalibration(
            2,
            {-30.0, 30.0, 0.0, false, 500.0, 2500.0});

        servo.setAngle(0, -30);
        servo.setAngle(1, -30);
        servo.center(2);

        motor.stop();

        // ================= INPUT / CAMERA =================

        TerminalInput keyboard;
        Camera camera(640, 480, 30);

        // ================= AI MODEL =================

        detection::PersonDetector personDetector(
            "detection/models/person_detector_script_11_lite.onnx", 320, 240, 0.25f, 0.3f);

        if (!personDetector.isLoaded())
        {
            std::cerr
                << "[WARNING] PersonDetector model load failed.\n";
        }

        // ================= PROGRAM STATE =================

        std::atomic<bool> running{true};

        ControlCommand command;

        // ================= MAP 공유 데이터 =================

        auto path =
            dataManager.getRcCarPath();

        cv::Mat satelliteImg =
            coordController.drawPathOnSatelliteImg(path);

        std::mutex mapMutex;

        // ================= AI 공유 데이터 =================

        // Main Thread → AI Thread
        cv::Mat aiInputFrame;
        std::mutex aiInputMutex;

        // AI Thread → Main Thread
        cv::Mat aiOutputFrame;
        std::mutex aiOutputMutex;

        // 새로운 Camera frame이 준비됐는지 표시
        std::atomic<bool> aiFrameReady{false};

        // ================= THREAD =================

        std::thread gpsThread;
        std::thread aiThread;

        // ================= 종료 함수 =================

        auto requestStop = [&]()
        {
            running.store(false);
            gps.stopThread();
        };

        auto joinThreads = [&]()
        {
            if (gpsThread.joinable())
                gpsThread.join();

            if (aiThread.joinable())
                aiThread.join();
        };

        try
        {
            // ==================================================
            // GPS THREAD
            // ==================================================

            gpsThread = std::thread([&]()
                                    {
                try
                {
                    gps.runGpsThread(coordController, [&]()
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
                    std::cerr << "[GPS THREAD ERROR] "<< e.what() << '\n';
                    requestStop();
                }
                catch (...)
                {
                    std::cerr << "[GPS THREAD ERROR] Unknown error\n";
                    requestStop();
                } });

            // ==================================================
            // AI THREAD
            // ==================================================

            aiThread = std::thread([&]()
                                   {
    try
    {
        std::cerr << "[AI] thread started\n";

        while (running.load())
        {
            // 새로운 Camera frame이 없으면 잠깐 대기
            if (!aiFrameReady.exchange(false))
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }

            std::cerr << "[AI] new frame received\n";

            cv::Mat inferenceFrame;

            // Main Thread가 전달한 최신 frame 복사
            {
                std::lock_guard<std::mutex> lock(aiInputMutex);

                if (aiInputFrame.empty())
                {
                    std::cerr << "[AI] input frame is empty\n";
                    continue;
                }

                inferenceFrame = aiInputFrame.clone();
            }

            std::cerr << "[AI] frame cloned\n";

            // 모델이 정상적으로 로드되지 않았다면 추론하지 않음
            if (!personDetector.isLoaded())
            {
                std::cerr << "[AI] model is not loaded\n";
                continue;
            }

            std::cerr << "[AI] detect start\n";

            // =========================================
            // AI 추론
            // =========================================

            auto boxes = personDetector.detect(inferenceFrame, true);

            std::cerr << "[AI] detect finished, boxes=" << boxes.size() << '\n';

            // =========================================
            // 탐지 결과
            // =========================================

            if (!boxes.empty())
            {
                auto feet = detection::PersonDetector::getFootPoints(boxes);

                std::cerr << "[AI] foot points finished, count=" << feet.size() << '\n';
            }

            // =========================================
            // AI 결과 → Main Thread
            // =========================================

            std::cerr << "[AI] output frame lock start\n";

            {
                std::lock_guard<std::mutex> lock(aiOutputMutex);
                aiOutputFrame = inferenceFrame.clone();
            }

            std::cerr << "[AI] output frame updated\n";
        }

        std::cerr << "[AI] thread loop ended\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << "[AI THREAD ERROR] " << e.what() << '\n';

        // 테스트 중에는 프로그램 전체 종료시키지 않음
        // requestStop();
    }
    catch (...)
    {
        std::cerr << "[AI THREAD ERROR] Unknown error\n";

        // requestStop();
    } });
            // ================= 조작 가이드 =================
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

            // ==================================================
            // MAIN THREAD
            // Camera + Keyboard + GUI + IMU
            // ==================================================

            cv::Mat frame;

            constexpr double speedSetting = 40.0;

            auto lastPrint = std::chrono::steady_clock::now();

            while (running.load())
            {
                // ================= IMU =================
                ImuData imuData = imu.read();
                dataManager.updateYaw(imuData.heading);

                // ================= CAMERA =================
                if (!camera.read(frame) || frame.empty())
                {
                    std::cerr << "[ERROR] Failed to read camera frame.\n";
                    requestStop();
                    break;
                }

                // =========================================
                // Main Thread → AI Thread
                // 최신 Camera frame 전달
                // =========================================

                {
                    std::lock_guard<std::mutex> lock(aiInputMutex);

                    aiInputFrame = frame.clone();
                }
                aiFrameReady.store(true);
                // =========================================
                // AI Thread → Main Thread
                // Bounding Box 결과 frame 가져오기
                // =========================================

                cv::Mat frameToShow;
                {
                    std::lock_guard<std::mutex> lock(aiOutputMutex);

                    if (!aiOutputFrame.empty())
                    {
                        frameToShow = aiOutputFrame.clone();
                    }
                }
                // 아직 AI가 첫 추론을 끝내지 않았다면
                // 원본 Camera frame 출력
                if (frameToShow.empty())
                {
                    frameToShow = frame.clone();
                }
                cv::imshow("Robot Camera", frameToShow);
                // ================= MAP =================
                cv::Mat mapToShow;
                {
                    std::lock_guard<std::mutex> lock(mapMutex);
                    if (!satelliteImg.empty())
                    {
                        mapToShow = satelliteImg.clone();
                    }
                }

                if (!mapToShow.empty())
                {
                    cv::imshow("Satellite Map", mapToShow);
                }
                // ================= OpenCV KEY =================
                int cvKey = cv::waitKey(1);

                if (cvKey == 27)
                {
                    requestStop();
                    break;
                }

                // ================= KEYBOARD =================

                int key = keyboard.readKey();

                if (key < 0)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    continue;
                }

                switch (key)
                {
                    // ================= DRIVE =================
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

                    // ================= STEERING =================
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

                    // ================= CAMERA TILT =================
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

                    // ================= CAMERA PAN =================

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

                    // ================= CAMERA RESET =================
                case 'r':
                case 'R':
                    command.cameraPan = 0.0;
                    command.cameraTilt = 0.0;
                    servo.center(0);
                    servo.center(1);
                    break;
                    // ================= EXIT =================
                case 'q':
                case 'Q':
                    requestStop();
                    break;
                }
            }

            // ================= 종료 =================
            requestStop();
            joinThreads();
            // ================= Hardware 안전 정지 =================
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