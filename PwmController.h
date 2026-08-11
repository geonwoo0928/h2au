#pragma once

#include <array>
#include <cstdint>

class I2cDevice;

class PwmController
{
public:
    explicit PwmController(I2cDevice& i2c);

    void configureServoTimer();
    void configureMotorTimer();

    void setChannelValue(int channel, uint16_t value);
    void setDutyPercent(int channel, double percent);

private:
    void setTimer(
        int timer,
        uint16_t prescaler,
        uint16_t period
    );

    int getTimer(int channel) const;

    I2cDevice& i2c_;
    std::array<uint16_t, 7> periods_{};
};