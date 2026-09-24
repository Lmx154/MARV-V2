#include "transport.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

namespace marv::ground {
namespace {

class Udp final : public Transport {
public:
    explicit Udp(int fd) : fd_(fd) {}
    ~Udp() override { ::close(fd_); }

    bool send(const std::uint8_t* p, std::size_t n) override {
        const ssize_t k = ::send(fd_, p, n, 0);
        // Nobody listening yet (the bridge not started) is not an error: the next send tries again.
        return k == static_cast<ssize_t>(n) || (k < 0 && errno == ECONNREFUSED);
    }

    long recv(std::uint8_t* p, std::size_t cap, int timeout_ms) override {
        pollfd pfd{fd_, POLLIN, 0};
        const int r = ::poll(&pfd, 1, timeout_ms);
        if (r < 0) return errno == EINTR ? 0 : -1;
        if (r == 0) return 0;
        const ssize_t k = ::recv(fd_, p, cap, MSG_DONTWAIT);
        if (k < 0) return (errno == EAGAIN || errno == ECONNREFUSED || errno == EINTR) ? 0 : -1;
        return static_cast<long>(k);
    }

private:
    int fd_;
};

class Serial final : public Transport {
public:
    explicit Serial(int fd) : fd_(fd) {}
    ~Serial() override { ::close(fd_); }

    bool send(const std::uint8_t* p, std::size_t n) override {
        while (n > 0) {
            const ssize_t k = ::write(fd_, p, n);
            if (k < 0 && errno == EINTR) continue;
            if (k <= 0) return false;
            p += k;
            n -= static_cast<std::size_t>(k);
        }
        return true;
    }

    long recv(std::uint8_t* p, std::size_t cap, int timeout_ms) override {
        pollfd pfd{fd_, POLLIN, 0};
        const int r = ::poll(&pfd, 1, timeout_ms);
        if (r < 0) return errno == EINTR ? 0 : -1;
        if (r == 0) return 0;
        if (!(pfd.revents & POLLIN)) return -1;  // hung up: the device went away
        const ssize_t k = ::read(fd_, p, cap);
        if (k < 0) return (errno == EAGAIN || errno == EINTR) ? 0 : -1;
        return static_cast<long>(k);
    }

private:
    int fd_;
};

}  // namespace

std::unique_ptr<Transport> open_udp(const char* host_port) {
    const std::string s = host_port;
    const auto colon = s.rfind(':');
    sockaddr_in to{};
    to.sin_family = AF_INET;
    if (colon == std::string::npos || inet_pton(AF_INET, s.substr(0, colon).c_str(), &to.sin_addr) != 1) {
        std::fprintf(stderr, "marv_ground: bad address %s, want a.b.c.d:port\n", host_port);
        return nullptr;
    }
    to.sin_port = htons(static_cast<std::uint16_t>(std::atoi(s.c_str() + colon + 1)));
    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0 || ::connect(fd, reinterpret_cast<const sockaddr*>(&to), sizeof(to)) != 0) {
        std::fprintf(stderr, "marv_ground: udp %s: %s\n", host_port, std::strerror(errno));
        if (fd >= 0) ::close(fd);
        return nullptr;
    }
    return std::make_unique<Udp>(fd);
}

std::unique_ptr<Transport> open_serial(const char* path) {
    const int fd = ::open(path, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        std::fprintf(stderr, "marv_ground: cannot open %s: %s\n", path, std::strerror(errno));
        return nullptr;
    }
    termios tio{};
    if (::tcgetattr(fd, &tio) != 0) {
        std::fprintf(stderr, "marv_ground: %s is not a tty: %s\n", path, std::strerror(errno));
        ::close(fd);
        return nullptr;
    }
    ::cfmakeraw(&tio);
    tio.c_cflag &= ~static_cast<tcflag_t>(CSTOPB | PARENB | CRTSCTS);
    tio.c_cflag |= CS8 | CLOCAL | CREAD;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    ::cfsetspeed(&tio, B115200);  // USB CDC: nominal, the link runs at USB speed regardless
    if (::tcsetattr(fd, TCSANOW, &tio) != 0) {
        std::fprintf(stderr, "marv_ground: cannot configure %s: %s\n", path, std::strerror(errno));
        ::close(fd);
        return nullptr;
    }
    ::tcflush(fd, TCIOFLUSH);
    return std::make_unique<Serial>(fd);
}

}  // namespace marv::ground
