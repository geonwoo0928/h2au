#pragma once

#include "GpioOutput.h"

class PwmController;

class MotorController
{
public:
    explicit MotorController(PwmController& pwm);
    ~MotorController();

    void setReversed(int motor, bool reversed);
    void setSpeed(int motor, double speedPercent);
    void drive(double speedPercent);
    void stop();

private:
    PwmController& pwm_;

    GpioOutput motor1Direction_;
    GpioOutput motor2Direction_;

    bool motor1Reversed_ = false;
    bool motor2Reversed_ = true;
};