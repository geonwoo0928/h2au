#include "ServoController.h"

#include "PwmController.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

ServoController::ServoController(PwmController& pwm)
    : pwm_(pwm)
{
    pwm_.configureServoTimer();
}

void ServoController::setCalibration(int channel, const ServoCalibration& calibration)
{
    checkChannel(channel);
    calibration_.at(channel) = calibration;
}

void ServoController::setAngle(int channel, double angle)
{
    checkChannel(channel);

    const auto& c = calibration_.at(channel);

    angle = std::clamp(angle, c.minAngle, c.maxAngle);

    double physicalAngle = c.reversed ? -angle : angle;
    physicalAngle += c.centerOffset;
    physicalAngle = std::clamp(physicalAngle, -90.0, 90.0);

    double pulseUs = c.minPulseUs + (physicalAngle + 90.0) * (c.maxPulseUs - c.minPulseUs) / 180.0;
    uint16_t pwmValue = static_cast<uint16_t>(std::round(pulseUs * 4095.0 / 20000.0));

    pwm_.setChannelValue(channel, pwmValue);
}

void ServoController::center(int channel)
{
    setAngle(channel, 0.0);
}

void ServoController::checkChannel(int channel)
{
    if (channel < 0 || channel > 2)
        throw std::out_of_range("Servo channel must be 0~2");
}