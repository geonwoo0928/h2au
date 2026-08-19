#include "GpsController.h"
#include "SeoilCoordController.h"
#include "RcCarDataManager.h"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

GpsController::GpsController(const char *serverIp, int port, RcCarDataManager &dataManager)
    : dataManager_(dataManager)
{
    socketFd_ = socket(AF_INET, SOCK_STREAM, 0);

    if (socketFd_ < 0)
        throw std::runtime_error("Failed to create GPS TCP socket");

    sockaddr_in serverAddr{};
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(static_cast<uint16_t>(port));

    if (inet_pton(AF_INET, serverIp, &serverAddr.sin_addr) != 1)
    {
        close(socketFd_);
        socketFd_ = -1;
        throw std::runtime_error("Invalid GPS server IP address");
    }

    if (connect(socketFd_, reinterpret_cast<sockaddr *>(&serverAddr), sizeof(serverAddr)) < 0)
    {
        close(socketFd_);
        socketFd_ = -1;
        throw std::runtime_error(std::string("Failed to connect to iPhone GPS: ") + std::strerror(errno));
    }

    // recv()가 무한정 blocking되지 않도록 100ms timeout
    timeval timeout{};
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000;

    if (setsockopt(socketFd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0)
    {
        close(socketFd_);
        socketFd_ = -1;
        throw std::runtime_error("Failed to set GPS socket timeout");
    }
}

GpsController::~GpsController()
{
    if (socketFd_ >= 0)
        close(socketFd_);
}

void GpsController::runGpsThread(const SeoilCoordController &coordController, const std::function<void()> &onGpsUpdated)
{
    while (isThreadRun_)
    {
        double lat = 0.0;
        double lon = 0.0;

        if (!getGpsData(lat, lon))
            continue;

        cv::Point2f pixel = coordController.getRcCarPixel(lat, lon);
        dataManager_.updateGps(lat, lon, pixel.x, pixel.y);

        onGpsUpdated();
    }
}

void GpsController::stopThread()
{
    isThreadRun_ = false;
}

bool GpsController::getGpsData(double &lat, double &lon)
{
    char buffer[256];

    while (isThreadRun_)
    {
        size_t pos = rxBuffer_.find('\n');

        if (pos == std::string::npos)
        {
            ssize_t rxLength = recv(socketFd_, buffer, sizeof(buffer) - 1, 0);

            if (rxLength > 0)
            {
                rxBuffer_.append(buffer, static_cast<size_t>(rxLength));
                continue;
            }

            if (rxLength == 0)
                throw std::runtime_error("iPhone GPS connection closed");

            if (errno == EAGAIN || errno == EWOULDBLOCK)
                return false;

            if (errno == EINTR)
                continue;

            throw std::runtime_error(std::string("GPS TCP receive failed: ") + std::strerror(errno));
        }

        std::string nmeaLine = rxBuffer_.substr(0, pos);
        rxBuffer_.erase(0, pos + 1);

        if (!nmeaLine.empty() && nmeaLine.back() == '\r')
            nmeaLine.pop_back();

        // GPS2IP에서 보내는 RMC 사용
        // 일부 장치는 GPRMC 대신 GNRMC를 사용할 수 있어서 둘 다 허용
        if (nmeaLine.rfind("$GPRMC", 0) != 0 && nmeaLine.rfind("$GNRMC", 0) != 0)
            continue;

        if (parseGpsData(nmeaLine, lat, lon))
            return true;
    }

    return false;
}

bool GpsController::parseGpsData(const std::string &nmeaLine, double &lat, double &lon)
{
    std::stringstream stream(nmeaLine);
    std::string token;

    std::string status;
    std::string latStr;
    std::string latDirection;
    std::string lonStr;
    std::string lonDirection;

    int index = 0;

    while (std::getline(stream, token, ','))
    {
        if (index == 2)
            status = token;
        else if (index == 3)
            latStr = token;
        else if (index == 4)
            latDirection = token;
        else if (index == 5)
            lonStr = token;
        else if (index == 6)
            lonDirection = token;

        index++;
    }

    // RMC status:
    // A = valid
    // V = invalid
    if (status != "A")
        return false;

    if (latStr.empty() || lonStr.empty())
        return false;

    if (!convertToDegree(latStr, lat) || !convertToDegree(lonStr, lon))
        return false;

    if (latDirection == "S")
        lat = -lat;

    if (lonDirection == "W")
        lon = -lon;

    return true;
}

bool GpsController::convertToDegree(const std::string &degreeText, double &degrees)
{
    try
    {
        double rawDegree = std::stod(degreeText);

        int convertedDegrees = static_cast<int>(rawDegree / 100.0);
        double minutes = rawDegree - (convertedDegrees * 100.0);

        degrees = convertedDegrees + (minutes / 60.0);

        return true;
    }
    catch (...)
    {
        return false;
    }
}