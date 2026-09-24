#include "endpoint.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <vector>

#include <marv/fsw/fsw.hpp>
#include <marv/link/protocol.hpp>

namespace marv::bridge {
namespace {

class Serial final : public Endpoint {
public:
    explicit Serial(int fd) : fd_(fd) {}
    ~Serial() override { ::close(fd_); }

    bool write(const std::uint8_t* p, std::size_t n) override {
        while (n > 0) {
            const ssize_t k = ::write(fd_, p, n);
            if (k < 0 && errno == EINTR) continue;
            if (k <= 0) return false;
            p += k;
            n -= static_cast<std::size_t>(k);
        }
        return true;
    }

    long read(std::uint8_t* p, std::size_t cap, int timeout_ms) override {
        pollfd pfd{fd_, POLLIN, 0};
        const int r = ::poll(&pfd, 1, timeout_ms);
        if (r < 0) return errno == EINTR ? 0 : -1;
        if (r == 0) return 0;
        const ssize_t k = ::read(fd_, p, cap);
        return k < 0 ? -1 : static_cast<long>(k);
    }

private:
    int fd_;
};

class Sitl final : public Endpoint {
public:
    explicit Sitl(float motor_cmd) : fsw_(motor_cmd) {}

    bool write(const std::uint8_t* p, std::size_t n) override {
        for (std::size_t i = 0; i < n; ++i) {
            SensorBus bus;
            if (!decoder_.push(p[i]) || !decoder_.packet().as(bus)) continue;
            std::uint8_t frame[link::kMaxFrame];
            const std::size_t len = link::encode(fsw_.step(bus), frame);
            out_.insert(out_.end(), frame, frame + len);
        }
        return true;
    }

    long read(std::uint8_t* p, std::size_t cap, int) override {
        const std::size_t n = out_.size() < cap ? out_.size() : cap;
        std::memcpy(p, out_.data(), n);
        out_.erase(out_.begin(), out_.begin() + static_cast<long>(n));
        return static_cast<long>(n);
    }

private:
    Fsw fsw_;
    link::Decoder decoder_;
    std::vector<std::uint8_t> out_;
};

}  // namespace

std::unique_ptr<Endpoint> open_serial(const char* path) {
    const int fd = ::open(path, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        std::fprintf(stderr, "marv_bridge: cannot open %s: %s\n", path, std::strerror(errno));
        return nullptr;
    }
    termios tio{};
    if (::tcgetattr(fd, &tio) != 0) {
        std::fprintf(stderr, "marv_bridge: %s is not a tty: %s\n", path, std::strerror(errno));
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
        std::fprintf(stderr, "marv_bridge: cannot configure %s: %s\n", path, std::strerror(errno));
        ::close(fd);
        return nullptr;
    }
    ::tcflush(fd, TCIOFLUSH);
    return std::make_unique<Serial>(fd);
}

std::unique_ptr<Endpoint> make_sitl(float motor_cmd) { return std::make_unique<Sitl>(motor_cmd); }

}  // namespace marv::bridge
