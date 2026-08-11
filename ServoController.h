#pragma once

#include <array>

class PwmController;

struct ServoCalibration
{
    double minAngle = -90.0;            //최소 각도
    double maxAngle = 90.0;             //최대 각도
    double centerOffset = 0.0;          //중앙 보정값
    bool reversed = false;              //방향 반전
    double minPulseUs = 500.0;          //최소 펄스
    double maxPulseUs = 2500.0;         //최대 펄스
};

class ServoController
{
public:
    explicit ServoController(PwmController& pwm);

    void setCalibration(
        int channel,
        const ServoCalibration& calibration
    );

    void setAngle(int channel, double angle);
    void center(int channel);

private:
    static void checkChannel(int channel);

    PwmController& pwm_;

    std::array<ServoCalibration, 3> calibration_{};
};