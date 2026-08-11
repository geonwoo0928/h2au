#include "GpioOutput.h"

#include <cstring>
#include <stdexcept>

#include <fcntl.h>
#include <linux/gpio.h>
#include <sys/ioctl.h>
#include <unistd.h>

GpioOutput::GpioOutput(
    unsigned int offset,
    const std::string& consumer
)
{
    chipFd_ = open("/dev/gpiochip0", O_RDONLY | O_CLOEXEC);

    if (chipFd_ < 0)
        throw std::runtime_error("Failed to open GPIO chip");

    gpio_v2_line_request request{};

    request.offsets[0] = offset;
    request.num_lines = 1;
    request.config.flags = GPIO_V2_LINE_FLAG_OUTPUT;

    strncpy(
        request.consumer,
        consumer.c_str(),
        sizeof(request.consumer) - 1
    );

    request.consumer[sizeof(request.consumer) - 1] = '\0';

    if (ioctl(chipFd_, GPIO_V2_GET_LINE_IOCTL, &request) < 0)
    {
        close(chipFd_);
        chipFd_ = -1;

        throw std::runtime_error("GPIO request failed");
    }

    lineFd_ = request.fd;

    set(false);
}

GpioOutput::~GpioOutput()
{
    if (lineFd_ >= 0)
        close(lineFd_);

    if (chipFd_ >= 0)
        close(chipFd_);
}

void GpioOutput::set(bool value)
{
    gpio_v2_line_values values{};

    values.mask = 1;
    values.bits = value ? 1 : 0;

    if (ioctl(lineFd_, GPIO_V2_LINE_SET_VALUES_IOCTL, &values) < 0)
        throw std::runtime_error("GPIO set failed");
}