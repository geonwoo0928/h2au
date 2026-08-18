#pragma once

#include "I2cDevice.h"

struct ImuData
{
    double heading; // 좌우 회전 방향(Yaw)
    double roll;    // 좌우 기울기
    double pitch;   // 앞뒤 기울기
};

class ImuController
{
public:
    explicit ImuController(I2cDevice &i2c);

    void initialize();
    ImuData read();

private:
    I2cDevice &i2c_;

    static int16_t combineBytes(uint8_t low, uint8_t high);
};