#include "endpoint.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include <marv/fsw/fsw.hpp>
#include <marv/link/protocol.hpp>

#include "../../firmware/src/dispatch.hpp"

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

// The SITL platform of fw::Node. Its flash is a file beside the bridge binary, marv_setup.bin, holding the same record
// bytes as the Pico's flash sector (dispatch.hpp); kReboot cannot restart the process, so it returns and the node
// powers on in place.
class SitlPlatform {
public:
    explicit SitlPlatform(std::vector<std::uint8_t>& out) : out_(out) {}

    void send(const std::uint8_t* p, std::size_t n) { out_.insert(out_.end(), p, p + n); }

    std::size_t read_record(std::uint8_t* p, std::size_t cap) {
        std::FILE* f = std::fopen(path().c_str(), "rb");
        if (!f) return 0;
        const std::size_t n = std::fread(p, 1, cap, f);
        std::fclose(f);
        return n;
    }

    void write_record(const std::uint8_t* p, std::size_t n) {
        std::FILE* f = std::fopen(path().c_str(), "wb");
        if (!f || std::fwrite(p, 1, n, f) != n)
            std::fprintf(stderr, "marv_bridge: cannot store the setup in %s\n", path().c_str());
        if (f) std::fclose(f);
    }

    void reboot() {}

private:
    static std::string path() {
        char exe[4096];
        const ssize_t n = ::readlink("/proc/self/exe", exe, sizeof(exe) - 1);
        std::string dir = n > 0 ? std::string(exe, static_cast<std::size_t>(n)) : std::string(".");
        const std::size_t slash = dir.rfind('/');
        dir = slash == std::string::npos ? std::string(".") : dir.substr(0, slash);
        return dir + "/marv_setup.bin";
    }

    std::vector<std::uint8_t>& out_;
};

class Sitl final : public Endpoint {
public:
    Sitl() : platform_(out_), node_(platform_) {}

    bool write(const std::uint8_t* p, std::size_t n) override {
        for (std::size_t i = 0; i < n; ++i)
            if (decoder_.push(p[i])) node_.dispatch(decoder_.packet());
        return true;
    }

    long read(std::uint8_t* p, std::size_t cap, int) override {
        const std::size_t n = out_.size() < cap ? out_.size() : cap;
        std::memcpy(p, out_.data(), n);
        out_.erase(out_.begin(), out_.begin() + static_cast<long>(n));
        return static_cast<long>(n);
    }

private:
    std::vector<std::uint8_t> out_;
    SitlPlatform platform_;
    fw::Node<SitlPlatform> node_;
    link::Decoder decoder_;
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

std::unique_ptr<Endpoint> make_sitl() { return std::make_unique<Sitl>(); }

}  // namespace marv::bridge
