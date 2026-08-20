#include "TerminalInput.h"

#include <cerrno>
#include <stdexcept>

#include <poll.h>
#include <unistd.h>

TerminalInput::TerminalInput()
{
    if (!isatty(STDIN_FILENO))
        throw std::runtime_error("Keyboard input requires terminal");

    if (tcgetattr(STDIN_FILENO, &original_) < 0)
        throw std::runtime_error("tcgetattr failed");

    termios raw = original_;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;

    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) < 0)
        throw std::runtime_error("tcsetattr failed");

    active_ = true;
}

TerminalInput::~TerminalInput()
{
    if (active_)
        tcsetattr(STDIN_FILENO, TCSANOW, &original_);
}

int TerminalInput::readKey(int timeoutMs)
{
    pollfd descriptor{};

    descriptor.fd = STDIN_FILENO;
    descriptor.events = POLLIN;

    int result = poll(&descriptor, 1, timeoutMs);

    if (result < 0)
    {
        if (errno == EINTR)
            return -1;

        throw std::runtime_error("poll failed");
    }

    if (result == 0 || !(descriptor.revents & POLLIN))
        return -1;

    unsigned char key = 0;

    if (read(STDIN_FILENO, &key, 1) == 1)
        return static_cast<int>(key);

    return -1;
}