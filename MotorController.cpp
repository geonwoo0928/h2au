#include "MotorController.h"

#include "PwmController.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>

MotorController::MotorController(PwmController& pwm)
    : pwm_(pwm),
      motor1Direction_(23),
      motor2Direction_(24)
{
    pwm_.configureMotorTimer();

    stop();
}

MotorController::~MotorController()
{
    try
    {
        stop();
    }
    catch (...)
    {
    }
}

void MotorController::setReversed(
    int motor,
    bool reversed
)
{
    if (motor == 1)
    {
        motor1Reversed_ = reversed;
    }
    else if (motor == 2)
    {
        motor2Reversed_ = reversed;
    }
    else
    {
        throw std::out_of_range(
            "Motor number must be 1 or 2"
        );
    }
}

void MotorController::setSpeed(
    int motor,
    double speedPercent
)
{
    if (motor != 1 && motor != 2)
    {
        throw std::out_of_range(
            "Motor number must be 1 or 2"
        );
    }

    speedPercent = std::clamp(
        speedPercent,
        -100.0,
        100.0
    );

    int pwmChannel = motor == 1 ? 13 : 12;

    GpioOutput& direction =
        motor == 1
        ? motor1Direction_
        : motor2Direction_;

    bool reversed =
        motor == 1
        ? motor1Reversed_
        : motor2Reversed_;

    if (std::abs(speedPercent) < 0.001)
    {
        pwm_.setDutyPercent(pwmChannel, 0.0);
        return;
    }

    bool forward = speedPercent > 0.0;

    if (reversed)
        forward = !forward;

    pwm_.setDutyPercent(pwmChannel, 0.0);

    std::this_thread::sleep_for(
        std::chrono::milliseconds(10)
    );

    direction.set(forward);

    pwm_.setDutyPercent(
        pwmChannel,
        std::abs(speedPercent)
    );
}

void MotorController::drive(double speedPercent)
{
    setSpeed(1, speedPercent);
    setSpeed(2, speedPercent);
}

void MotorController::stop()
{
    pwm_.setDutyPercent(13, 0.0);
    pwm_.setDutyPercent(12, 0.0);

    motor1Direction_.set(false);
    motor2Direction_.set(false);
}