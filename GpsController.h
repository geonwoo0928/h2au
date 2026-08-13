#pragma once

#include <iostream>
#include <string>
#include <sstream>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <stdexcept>
#include <mutex>
#include <deque>
#include <opencv2/opencv.hpp>

class SeoilCoordController;

struct RcCarPosition {
    double lat;
    double lon;
    float pixelX;
    float pixelY;
};

class GpsController {
public:
    GpsController(const char* serialPort);

    ~GpsController();

    // 백그라운드 스레드에서 주기적으로 GPS 센서 데이터를 수신, 파싱 및 좌표 변환을 수행하는 메인 루프 함수
    void runGpsThread(const SeoilCoordController& coordController); 

    // GPS 수신 스레드의 안전한 종료를 요청하는 플래그 설정 함수
    void stopThread();

    // 위성 지도에 궤적(선)을 그리기 위해 저장된 RC카의 과거 이동 경로(deque)를 반환하는 함수
    std::deque<RcCarPosition> getRcCarPath();

private:

    // 시리얼 포트로부터 버퍼를 읽어와 개행 문자(\n) 기준 완결된 NMEA($GPGGA) 문장을 추출하는 함수
    bool getGpsData(double& lat, double& lon);
    
    // 완성된 NMEA 문장($GPGGA)에서 콤마(,) 구분자를 기준으로 위도와 경도 원본 문자열을 추출하는 함수
    bool parseGpsData(const std::string& nmeaLine, double& lat, double& lon); // gps 데이터에서 위도, 경도만 가져올수있도록 파싱하는 함수

    // NMEA 규격의 도·분(DDMM.MMMM) 형식 문자열을 십진 도(Decimal Degrees) 형식으로 변환하는 함수
    bool convertToDegree(const std::string& degreeText, double& degrees); // gps 데이터에서 위도, 경도로 변환하는 함수

    // 수신된 최신 RC카 위치(위경도 및 픽셀 좌표)를 궤적 저장소(deque)에 추가하는 함수
    void addRcCarPath(const RcCarPosition& pos); // rc카 경로에 데이터 추가하는 함수

    int uartFilestream_;

    std::string rxBuffer_;

    std::deque<RcCarPosition> rcCarPath_;

    int pathMaxSize_ = 13;

    std::mutex gpsMutex_;

    bool isThreadRun_ = true;
};