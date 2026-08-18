#include "SeoilCoordController.h"

SeoilCoordController::SeoilCoordController() {
    satImg_ = cv::imread("sat_img.png");
    if (satImg_.empty()) {
        throw std::runtime_error("위성 지도 이미지를 불러오지 못했습니다.");
    }

    h_gps_to_pixel_ = cv::findHomography(sat_gps_points_, sat_pixel_points_);
    if(h_gps_to_pixel_.empty()) {
        throw std::runtime_error("호모그래피 행렬 계산을 실패하였습니다.");
    }
}

cv::Point2f SeoilCoordController::getRcCarPixel(double lat, double lon) const {
    std::vector<cv::Point2f> src = { cv::Point2f(static_cast<float>(lon), static_cast<float>(lat)) };
    std::vector<cv::Point2f> dst;
    cv::perspectiveTransform(src, dst, h_gps_to_pixel_);

    return dst[0];
}

cv::Mat SeoilCoordController::drawPathOnSatelliteImg(const std::deque<RcCarPosition>& path) {
    cv::Mat sat_img = satImg_.clone();

    if(path.empty()) return sat_img;

    std::vector<cv::Point> pixelPoints;
    pixelPoints.reserve(path.size());

    for(const RcCarPosition pos : path)
        pixelPoints.push_back(cv::Point(static_cast<int>(pos.pixelX), static_cast<int>(pos.pixelY)));
    
    if(pixelPoints.size() >= 2) {
        cv::polylines(sat_img, pixelPoints, false, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    }

    cv::circle(sat_img, pixelPoints.back(), 6, cv::Scalar(0, 255, 0), -1);

    return sat_img;
}

bool SeoilCoordController::validateGpsData(float lat, float lon) const{
    cv::Point2f target = {lon, lat};

    return cv::pointPolygonTest(sat_gps_points_, target, false) >= 0;
}