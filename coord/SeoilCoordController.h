#pragma once

#include <iostream>
#include <vector>
#include <deque>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#include "RcCarDataManager.h"
#include "PersonLocationCalculator.h"

class SeoilCoordController
{
public:
    SeoilCoordController();

    // rc카 위치, 객체 위치를 그리며 위성지도 이미지 반환
    cv::Mat getSatImg();

    // rc카 위도, 경도 값을 이미지 픽셀 값으로 변환하는 함수
    cv::Point2f getPixelFromLatLon(double lat, double lon) const;

    // rc카 경로 최신화하는 함수
    void updatePath(const std::deque<RcCarPosition>& path);

    // 객체 위치 최신화하는 함수
    void updatePeople(const std::vector<cv::Point>& locaiton);

    // rc카 위치가 지도 밖으로 나갔는지 검증하는 함수
    bool validateGpsData(float lat, float lon) const;
    
private:
    cv::Mat satImg_;

    cv::Mat h_gps_to_pixel_;

    std::deque<RcCarPosition> currentPath_;

    std::vector<cv::Point> currentPeople_;

    std::mutex mapMutex_;

    const std::vector<cv::Point2f> sat_pixel_points_ = {
        cv::Point2f(0.0f, 0.0f),        // 1. 좌상단
        cv::Point2f(930.0f, 0.0f),      // 2. 우상단
        cv::Point2f(930.0f, 1216.0f),    // 3. 우하단
        cv::Point2f(0.0f, 1216.0f),      // 4. 좌하단 
    };

    const std::vector<cv::Point2f> sat_gps_points_ = {
        cv::Point2f(127.0976250f, 37.5871470f), // 1. 좌상단 
        cv::Point2f(127.0982868f, 37.5870565f), // 2. 우상단 
        cv::Point2f(127.0981418f, 37.5863835f), // 3. 우하단 
        cv::Point2f(127.0974974f, 37.5864552f), // 4. 좌하단
    };
};