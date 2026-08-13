#include "RcCarDataManager.h"

void RcCarDataManager::updateGps(double lat, double lon, float pixelX, float pixelY) {
    std::lock_guard<std::mutex> lock(mutex_);

    currentPos_.lat = lat;
    currentPos_.lon = lon;
    currentPos_.pixelX = pixelX;
    currentPos_.pixelY = pixelY;

    rcCarPath_.emplace_back(currentPos_);
    if(rcCarPath_.size() > MAXSIZE_)
        rcCarPath_.pop_front();

        return;
}

void RcCarDataManager::updateYaw(double yaw) {
    std::lock_guard<std::mutex> lock(mutex_);

    currentPos_.yaw = yaw;

    return;
}

std::deque<RcCarPosition> RcCarDataManager::getRcCarPath() {
    std::lock_guard<std::mutex> lock(mutex_);

    return rcCarPath_;
}

RcCarPosition RcCarDataManager::getCurrentPos() {
    std::lock_guard<std::mutex> lock(mutex_);

    return currentPos_;
}