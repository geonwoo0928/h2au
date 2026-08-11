#pragma once

#include <opencv2/opencv.hpp>

#include <string>

class Camera
{
public:
    Camera(int width = 640, int height = 480, int fps = 30);
    ~Camera();

    bool read(cv::Mat& frame);

private:
    std::string makePipeline(int width, int height, int fps) const;

    cv::VideoCapture capture_;
};