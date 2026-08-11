#include "I2cDevice.h"

#include <cerrno>
#include <cstring>
#include <stdexcept>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

I2cDevice::I2cDevice(
    const std::string& device,
    uint8_t address
)
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

    if (write(fd_, data, sizeof(data))
        != static_cast<ssize_t>(sizeof(data)))
    {
        throw std::runtime_error(
            std::string("I2C write failed: ") +
            std::strerror(errno)
        );
    }
}