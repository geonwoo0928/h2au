#include "Camera.h"

#include <iostream>
#include <sstream>
#include <stdexcept>

Camera::Camera(int width, int height, int fps)
{
    capture_.open(makePipeline(width, height, fps), cv::CAP_GSTREAMER);

    if (!capture_.isOpened())
    {
        std::cerr << "GStreamer camera open failed. Trying V4L2.\n";
        capture_.open(0, cv::CAP_V4L2);
    }

    if (!capture_.isOpened())
        throw std::runtime_error("Failed to open camera");
}

Camera::~Camera()
{
    if (capture_.isOpened())
        capture_.release();
}

bool Camera::read(cv::Mat& frame)
{
    return capture_.read(frame);
}

std::string Camera::makePipeline(int width, int height, int fps) const
{
    std::ostringstream pipeline;

    pipeline
        << "libcamerasrc ! "
        << "video/x-raw,width=" << width
        << ",height=" << height
        << ",format=NV12,framerate=" << fps
        << "/1 ! "
        << "videoconvert ! "
        << "video/x-raw,format=BGR ! "
        << "queue max-size-buffers=1 "
        << "leaky=downstream ! "
        << "appsink drop=true "
        << "max-buffers=1 "
        << "sync=false";

    return pipeline.str();
}