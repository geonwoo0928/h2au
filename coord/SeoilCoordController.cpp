#include "SeoilCoordController.h"

SeoilCoordController::SeoilCoordController()
{
    satImg_ = cv::imread("../sat_img.png");
    if (satImg_.empty())
        throw std::runtime_error("위성 지도 이미지를 불러오지 못했습니다.");

    h_gps_to_pixel_ = cv::findHomography(sat_gps_points_, sat_pixel_points_);
    if (h_gps_to_pixel_.empty())
        throw std::runtime_error("호모그래피 행렬 계산을 실패하였습니다.");
}

cv::Mat SeoilCoordController::getSatImg()
{
    std::lock_guard<std::mutex> lock(mapMutex_);

    cv::Mat displayImg = satImg_.clone();

    if (!currentPath_.empty())
    {
        std::vector<cv::Point> pixelPoints;
        pixelPoints.reserve(currentPath_.size());
        
        for (const RcCarPosition& pos : currentPath_)
            pixelPoints.push_back(cv::Point(cvRound(pos.pixelX), cvRound(pos.pixelY)));
        
        if (pixelPoints.size() >= 2)
        {
            cv::polylines(displayImg, pixelPoints, false, cv::Scalar(0, 0, 0), 5, cv::LINE_AA);
            cv::polylines(displayImg, pixelPoints, false, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
        }
        
        cv::circle(displayImg, pixelPoints.back(), 10, cv::Scalar(255, 255, 255), -1, cv::LINE_AA); 
        cv::circle(displayImg, pixelPoints.back(), 6, cv::Scalar(0, 255, 0), -1, cv::LINE_AA);
    }

    if (!currentPeople_.empty())
    {
        for (const cv::Point& p : currentPeople_)
        {
            cv::circle(displayImg, p, 12, cv::Scalar(255, 255, 255), -1, cv::LINE_AA); 
            cv::circle(displayImg, p, 7, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
        }
    }

    return displayImg;
}

cv::Point2f SeoilCoordController::getPixelFromLatLon(double lat, double lon) const
{
    std::vector<cv::Point2f> src = { cv::Point2f(static_cast<float>(lon), static_cast<float>(lat)) };
    std::vector<cv::Point2f> dst;
    cv::perspectiveTransform(src, dst, h_gps_to_pixel_);

    return dst[0];
}

void SeoilCoordController::updatePath(const std::deque<RcCarPosition>& path)
{
    std::lock_guard<std::mutex> lock(mapMutex_);
    currentPath_ = path;
}

void SeoilCoordController::updatePeople(const std::vector<cv::Point>& location)
{
    std::lock_guard<std::mutex> lock(mapMutex_);
    currentPeople_ = location;
}

bool SeoilCoordController::validateGpsData(float lat, float lon) const
{
    cv::Point2f target = {lon, lat};

    return cv::pointPolygonTest(sat_gps_points_, target, false) >= 0;
}