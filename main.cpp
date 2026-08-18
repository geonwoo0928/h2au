#include <chrono>
#include <thread>
#include <deque>
#include <iostream>

#include <opencv2/opencv.hpp>

#include "Camera.h"
#include "I2cDevice.h"
#include "MotorController.h"
#include "PwmController.h"
#include "ServoController.h"
#include "TerminalInput.h"
#include "SeoilCoordController.h"
#include "GpsController.h"
#include "RcCarDataManager.h"
#include "PersonDetector.h"

int main()
{
    try
    {
        I2cDevice i2c("/dev/i2c-1", 0x14);

        PwmController pwm(i2c);
        ServoController servo(pwm);
        MotorController motor(pwm);

        RcCarDataManager dataManager;
        SeoilCoordController coordController;
        GpsController gpsController("/dev/serial0", dataManager);

        std::thread gpsThread(&GpsController::runGpsThread, &gpsController, std::ref(coordController));

        // 채널 0: 카메라 Pan
        // 채널 1: 카메라 Tilt
        // 채널 2: Steering
        servo.setCalibration(
            0,
            {-90.0, 90.0, 0.0, false, 500.0, 2500.0}
        );

        servo.setCalibration(
            1,
            {-45.0, 45.0, 0.0, false, 500.0, 2500.0}
        );

        servo.setCalibration(
            2,
            {-30.0, 30.0, 0.0, false, 500.0, 2500.0}
        );

        servo.center(0);
        servo.center(1);
        servo.center(2);

        motor.stop();

        TerminalInput keyboard;
        Camera camera(640, 480, 30);

        // 모델 로드 실패해도 앱 전체를 죽이지 않음 - 주행/서보 제어는
        // 탐지 없이도 계속 동작해야 하므로 isLoaded()만 확인하고 넘어간다.
        detection::PersonDetector personDetector("detection/models/person_detector_script_11_lite.onnx", 320, 240, 0.25f, 0.3f);
        if (!personDetector.isLoaded())
        {
            std::cerr << "[경고] PersonDetector 모델 로드 실패 - 보행자 탐지 없이 계속 진행합니다.\n";
        }

        double speedSetting = 30.0;         // 기본 속도
        double driveCommand = 0.0;          // 현재 주행 명령값
        double steeringAngle = 0.0;         // 현재 조향 각도
        double cameraPan = 0.0;             // 카메라 좌우 각도
        double cameraTilt = 0.0;            // 카메라 상하 각도

        cv::Mat frame;

        std::cout
            << "==========================================\n"
            << " [조작 가이드 (터미널 클릭 후 입력)]\n"
            << " 주행 : W(전진), S(후진), A(좌), D(우), Space(정지)\n"
            << " 머리 : I(위), K(아래), J(왼쪽), L(오른쪽), R(리셋)\n"
            << " 종료 : Q 또는 카메라 창에서 ESC\n"
            << "==========================================\n";

        bool isRunning = true;
        while (isRunning)
        {
            bool ok = camera.read(frame);

            if (ok == false || frame.empty())
            {
                std::cerr
                    << "[ERROR] Failed to read camera frame.\n";

                break;
            }

            // 탐지 결과를 화면에 박스로 그린다 (위성지도 투영은 카메라가 고정이
            // 아니라 아직 캘리브레이션 방식이 정해지지 않아 보류 중).
            if (personDetector.isLoaded())
            {
                personDetector.detect(frame, /*drawBoxes=*/true);
            }

            cv::imshow("Robot Camera", frame);

            std::deque<RcCarPosition> path = dataManager.getRcCarPath();
            cv::Mat satImg = coordController.drawPathOnSatelliteImg(path);
            cv::imshow("Seoil Satellite Img", satImg);

            int cvKey = cv::waitKey(1);

            if (cvKey == 27)
                break;

            int key = keyboard.readKey();

            if (key < 0)
            {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(10)
                );

                continue;
            }

            switch (key)
            {
            case 'w':
            case 'W':
                driveCommand = speedSetting;
                motor.drive(driveCommand);
                break;

            case 's':
            case 'S':
                driveCommand = -speedSetting;
                motor.drive(driveCommand);
                break;

            case ' ':
                driveCommand = 0.0;
                motor.stop();
                break;

            case 'a':
            case 'A':
                steeringAngle -= 5.0;

                if (steeringAngle < -30.0)
                    steeringAngle = -30.0;

                servo.setAngle(2, steeringAngle);
                break;

            case 'd':
            case 'D':
                steeringAngle += 5.0;

                if (steeringAngle > 30.0)
                    steeringAngle = 30.0;

                servo.setAngle(2, steeringAngle);
                break;

            case 'i':
            case 'I':
                cameraTilt += 5.0;

                if (cameraTilt > 45.0)
                    cameraTilt = 45.0;

                servo.setAngle(1, cameraTilt);
                break;

            case 'k':
            case 'K':
                cameraTilt -= 5.0;

                if (cameraTilt < -45.0)
                    cameraTilt = -45.0;

                servo.setAngle(1, cameraTilt);
                break;

            case 'j':
            case 'J':
                cameraPan -= 5.0;

                if (cameraPan < -90.0)
                    cameraPan = -90.0;

                servo.setAngle(0, cameraPan);
                break;

            case 'l':
            case 'L':
                cameraPan += 5.0;

                if (cameraPan > 90.0)
                    cameraPan = 90.0;

                servo.setAngle(0, cameraPan);
                break;

            case 'r':
            case 'R':
                cameraPan = 0.0;
                cameraTilt = 0.0;

                servo.setAngle(0, cameraPan);
                servo.setAngle(1, cameraTilt);
                break;

            case 'q':
            case 'Q':
                isRunning = false;
                break;
            }
        }

        motor.stop();
        servo.center(0);
        servo.center(1);
        servo.center(2);
        cv::destroyAllWindows();

        gpsController.stopThread();
        if(gpsThread.joinable())
            gpsThread.join();

        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr
            << "Error: "
            << e.what()
            << '\n';

        return 1;
    }
    return 0;
}
