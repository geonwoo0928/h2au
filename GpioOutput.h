#pragma once

#include <string>

class GpioOutput
{
public:
    GpioOutput(
        unsigned int offset,
        const std::string& consumer = "robot-hat-control"
    );

    ~GpioOutput();

    GpioOutput(const GpioOutput&) = delete;
    GpioOutput& operator=(const GpioOutput&) = delete;

    void set(bool value);

private:
    int chipFd_ = -1;
    int lineFd_ = -1;
};