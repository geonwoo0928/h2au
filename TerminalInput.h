#pragma once

#include <termios.h>

class TerminalInput
{
public:
    TerminalInput();
    ~TerminalInput();

    TerminalInput(const TerminalInput&) = delete;
    TerminalInput& operator=(const TerminalInput&) = delete;

    int readKey(int timeoutMs = 0);

private:
    termios original_{};

    bool active_ = false;
};