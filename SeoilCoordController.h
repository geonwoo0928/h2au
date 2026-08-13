#pragma once

#include <iostream>
#include <vector>
#include <deque>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#include "RcCarDataManager.h"

class SeoilCoordController {
public:
    SeoilCoordController();

    // rc카 위도, 경도 값을 이미지 픽셀 값으로 변환하는 함수
    cv::Point2f getRcCarPixel(double lat, double lon) const;

    // rc카 경로 데이터를 이용해 위성 사진에 경로(선)을 그리는 함수
    cv::Mat drawPathOnSatelliteImg(const std::deque<RcCarPosition>& path);
    
private:
    cv::Mat satImg_;

    cv::Mat h_gps_to_pixel_;
};