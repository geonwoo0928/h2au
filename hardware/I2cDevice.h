#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

class I2cDevice
{
public:
    I2cDevice(const std::string &device, uint8_t address);
    ~I2cDevice();

    I2cDevice(const I2cDevice &) = delete;
    I2cDevice &operator=(const I2cDevice &) = delete;

    void writeRegister16(uint8_t reg, uint16_t value);
    void writeRegister8(uint8_t reg, uint8_t value);
    uint8_t readRegister8(uint8_t reg);
    void readRegisters(uint8_t startReg, uint8_t *buffer, std::size_t length);

private:
    int fd_ = -1;
    uint8_t address_ = 0;
};