#include "I2cDevice.h"

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>

I2cDevice::I2cDevice(const std::string &device, uint8_t address)
    : address_(address)
{
    fd_ = open(device.c_str(), O_RDWR | O_CLOEXEC);

    if (fd_ < 0)
        throw std::runtime_error("Failed to open I2C device");

    if (ioctl(fd_, I2C_SLAVE, address) < 0)
    {
        close(fd_);
        fd_ = -1;

        throw std::runtime_error("Failed to select I2C slave");
    }
}

I2cDevice::~I2cDevice()
{
    if (fd_ >= 0)
        close(fd_);
}

void I2cDevice::writeRegister16(uint8_t reg, uint16_t value)
{
    uint8_t data[3];

    data[0] = reg;
    data[1] = static_cast<uint8_t>((value >> 8) & 0xff);
    data[2] = static_cast<uint8_t>(value & 0xff);

    if (write(fd_, data, sizeof(data)) != static_cast<ssize_t>(sizeof(data)))
    {
        throw std::runtime_error(
            std::string("I2C write failed: ") + std::strerror(errno));
    }
}

void I2cDevice::writeRegister8(uint8_t reg, uint8_t value)
{
    uint8_t data[2];

    data[0] = reg;
    data[1] = value;

    if (write(fd_, data, sizeof(data)) != static_cast<ssize_t>(sizeof(data)))
    {
        throw std::runtime_error(
            std::string("I2C write failed: ") + std::strerror(errno));
    }
}

uint8_t I2cDevice::readRegister8(uint8_t reg)
{
    uint8_t value = 0;

    readRegisters(reg, &value, 1);

    return value;
}

void I2cDevice::readRegisters(
    uint8_t startReg,
    uint8_t *buffer,
    std::size_t length)
{
    if (buffer == nullptr || length == 0)
        throw std::invalid_argument(
            "I2C read buffer must be non-null and length > 0");

    i2c_msg messages[2]{};

    messages[0].addr = address_;
    messages[0].flags = 0;
    messages[0].len = 1;
    messages[0].buf = &startReg;

    messages[1].addr = address_;
    messages[1].flags = I2C_M_RD;
    messages[1].len = static_cast<__u16>(length);
    messages[1].buf = buffer;

    i2c_rdwr_ioctl_data transaction{};

    transaction.msgs = messages;
    transaction.nmsgs = 2;

    if (ioctl(fd_, I2C_RDWR, &transaction) < 0)
    {
        throw std::runtime_error(
            std::string("I2C combined read failed: ") + std::strerror(errno));
    }
}