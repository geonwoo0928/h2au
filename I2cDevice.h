#pragma once

#include <cstdint>
#include <string>

class I2cDevice
{
public:
    I2cDevice(
        const std::string& device = "/dev/i2c-1",
        uint8_t address = 0x14
    );

    ~I2cDevice();

    I2cDevice(const I2cDevice&) = delete;
    I2cDevice& operator=(const I2cDevice&) = delete;

    void writeRegister16(uint8_t reg, uint16_t value);

private:
    int fd_ = -1;
};