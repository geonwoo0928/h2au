#include "PersonLocationCalculator.h"
#include "SeoilCoordController.h"

PersonLocationCalculator::ObjectMetric PersonLocationCalculator::calculateObjectMetrics(float pixelX, float pixelY) const
{
    float cx = FRAME_WIDTH_ / 2.0f;
    float cy = FRAME_HEIGHT_ / 2.0f;
    
    float dx = pixelX - cx;
    float rel_angle_deg = dx * (H_FOV / FRAME_WIDTH_); // rc카로부터 객체의 상대각도
    
    float dy = pixelY - cy;
    float pixel_v_deg = dy * (V_FOV / FRAME_HEIGHT_);
    float total_v_angle_deg = TILT_ANGLE + pixel_v_deg; // 지상으로부터 카메라가 total_v_angle_deg만큼 숙이면 객체 밑 부분이 보인다
    
    if (total_v_angle_deg <= 0.0f)
        return {-1.0f, rel_angle_deg};
    
    float total_v_angle_rad = total_v_angle_deg * M_PI / 180.0f; // 라디안으로 변환
    float distance = CAMERA_HEIGHT / std::tan(total_v_angle_rad); // 삼각비 tan(total_v_angle_rad) 는 (rc카로부터 객체의 거리) / (카메라 높이) 랑 똑같음
    
    return {distance, rel_angle_deg};
}

float PersonLocationCalculator::getAbsoluteBearing(float imuHeading, float relAngleDeg) const
{
    float absBearing = imuHeading + relAngleDeg;
    if (absBearing < 0.0f)
        absBearing += 360.0f;
    if (absBearing >= 360.0f)
        absBearing -= 360.0f;
    return absBearing;
}

std::vector<cv::Point> PersonLocationCalculator::calculate(std::vector<detection::FootPoint> footPoints, double rcLat, double rcLon, float imuHeading, const SeoilCoordController& controller)
{
    if (!locations_.empty())
        locations_.clear();

    for (detection::FootPoint point : footPoints)
    {
        ObjectMetric metrics = calculateObjectMetrics(point.x, point.y);
        if (metrics.distance < 0.0f)
            continue;

        float bearing = getAbsoluteBearing(imuHeading, metrics.relAngleDeg);

        const double metersPerLat = 111320.0; // 1도 = 111320m
        float bearingRad = bearing * M_PI / 180.0f; // 라디안 -> 도 로 변경
        double latRad = rcLat * M_PI / 180.0f;

        double northMeters = metrics.distance * std::cos(bearingRad); // cos(객체의 방위각) == (rc카 기준 객체와 떨어진 세로 길이) / (떨어진 대각선 길이)
        double eastMeters = metrics.distance * std::sin(bearingRad); // sin(객체의 방위각) == (rc카 기준 객체와 떨어진 가로 길이) / (떨어진 대각선 길이)

        double deltaLat = northMeters / metersPerLat;
        double deltaLon = eastMeters / (metersPerLat * std::cos(latRad));

        double lat = rcLat + deltaLat;
        double lon = rcLon + deltaLon;

        cv::Point2f pixel = controller.getPixelFromLatLon(lat, lon);

        locations_.push_back(cv::Point(cvRound(pixel.x), cvRound(pixel.y)));
    }

    return locations_;
}