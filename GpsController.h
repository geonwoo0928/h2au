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

    void runGpsThread(const SeoilCoordController &coordController, const std::function<void()> &onGpsUpdated);
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