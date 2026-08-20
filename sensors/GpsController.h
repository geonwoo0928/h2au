#pragma once

#include <atomic>
#include <functional>
#include <string>

class SeoilCoordController;
class RcCarDataManager;

class GpsController
{
public:
    GpsController(const char *serverIp, int port, RcCarDataManager &dataManager);
    ~GpsController();

    // 백그라운드 스레드에서 주기적으로 GPS 센서 데이터를 수신, 파싱 및 좌표 변환을 수행하는 메인 루프 함수
    void runGpsThread(SeoilCoordController &coordController);

    // GPS 수신 스레드의 안전한 종료를 요청하는 플래그 설정 함수
    void stopThread();

private:
    bool getGpsData(double &lat, double &lon);
    bool parseGpsData(const std::string &nmeaLine, double &lat, double &lon);
    bool convertToDegree(const std::string &degreeText, double &degrees);

    int socketFd_ = -1;
    std::string rxBuffer_;
    std::atomic<bool> isThreadRun_{true};
    RcCarDataManager &dataManager_;
};