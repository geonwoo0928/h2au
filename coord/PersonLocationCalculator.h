#pragma once

#include <cmath>
#include <vector>
#include <opencv2/opencv.hpp>

#include "PersonDetector.h"

class SeoilCoordController;

class PersonLocationCalculator
{
public:
    // 객체의 위도 경도 계산
    std::vector<cv::Point> calculate(std::vector<detection::FootPoint> footPoints, double rcLat, double rcLon, float imuHeading, const SeoilCoordController& controller);

private:
    const float H_FOV = 53.0f;
    const float V_FOV = 41.0f;    
    const float CAMERA_HEIGHT = 0.06f;
    const float TILT_ANGLE = 2.0f;
    const int FRAME_WIDTH_ = 640;
    const int FRAME_HEIGHT_ = 480;
    std::vector<cv::Point> locations_;

    struct ObjectMetric
    {
        float distance;
        float relAngleDeg;
    };

    // rc카로부터 객체의 거리, 상대각도 계산
    ObjectMetric calculateObjectMetrics(float pixelX, float pixelY) const;

    // 객체의 절대방위각 계산
    float getAbsoluteBearing(float imuHeading, float relAngleDeg) const;
};