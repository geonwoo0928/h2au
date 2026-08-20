#pragma once

#include <deque>
#include <mutex>

struct RcCarPosition
{
    double lat = 0.0;
    double lon = 0.0;
    double yaw = 0.0;
    float pixelX = 0.0f;
    float pixelY = 0.0f;
};

class RcCarDataManager
{
public:
    // 위경도, 픽셀 좌표를 currentPos_에 대입 및 rcCarPath_에 추가하는 함수
    void updateGps(double lat, double lon, float pixelX, float pixelY);

    // 방위각을 currentPos_에 대입하는 함수
    void updateYaw(double yaw);

    // RC카 위치(위경도, 픽셀 좌표, 방위각)를 반환하는 함수
    std::deque<RcCarPosition> getRcCarPath();

    // 현재 RC카 위치 정보를 반환하는 함수
    RcCarPosition getCurrentPos();

private:
    std::mutex mutex_;
    RcCarPosition currentPos_;
    std::deque<RcCarPosition> rcCarPath_;
    const int MAXSIZE_ = 100;
};