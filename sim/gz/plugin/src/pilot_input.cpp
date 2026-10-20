// The pilot input's socket and datagram decoder (pilot_input.hpp has the datagram and the staging rule).
#include "pilot_input.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <limits>

#include "marv/prim/constants.hpp"

namespace marv::gz {

static_assert(kPilotChannels == prim::kCrsfRcChannels);
static_assert((1U << prim::kCrsfRcChannelBits) - 1U == kPilotRawMax);

namespace {

constexpr std::array<std::uint8_t, 4> kMagic{'M', 'P', 'L', 'T'};
constexpr unsigned kByteBits = 8;

std::uint16_t u16_at(const std::uint8_t* p) {
  return static_cast<std::uint16_t>(p[0] | (static_cast<unsigned>(p[1]) << kByteBits));
}

}  // namespace

std::optional<PilotFrame> decode_pilot_datagram(const std::uint8_t* data, std::size_t size) {
  constexpr std::size_t kVersionAt = kMagic.size();
  constexpr std::size_t kLengthAt = kVersionAt + sizeof(std::uint16_t);
  if (size != kPilotDatagramBytes || std::memcmp(data, kMagic.data(), kMagic.size()) != 0 ||
      u16_at(data + kVersionAt) != kPilotVersion || u16_at(data + kLengthAt) != kPilotDatagramBytes) {
    return std::nullopt;
  }
  PilotFrame f;
  for (std::size_t i = 0; i < kPilotChannels; ++i) {
    f.raw[i] = u16_at(data + kPilotHeaderBytes + (i * sizeof(std::uint16_t)));
    if (f.raw[i] > kPilotRawMax) {
      return std::nullopt;
    }
  }
  return f;
}

PilotInput::~PilotInput() {
  if (fd_ >= 0) {
    ::close(fd_);
  }
}

std::string PilotInput::open(std::uint32_t port) {
  if (port < 1 || port > std::numeric_limits<std::uint16_t>::max()) {
    return "<pilot_udp_port> is not a UDP port 1..65535";
  }
  fd_ = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (fd_ < 0) {
    return std::string("cannot open the pilot socket: ") + std::strerror(errno);
  }
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<std::uint16_t>(port));
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (::bind(fd_, reinterpret_cast<const sockaddr*>(&addr), sizeof addr) != 0) {
    const std::string why = std::strerror(errno);
    ::close(fd_);
    fd_ = -1;
    return "cannot bind the pilot socket to 127.0.0.1:" + std::to_string(port) + ": " + why;
  }
  return {};
}

std::optional<PilotFrame> PilotInput::poll() {
  std::optional<PilotFrame> newest;
  if (fd_ < 0) {
    return newest;
  }
  // One byte more than a datagram, so that a longer one is seen as such and dropped.
  std::array<std::uint8_t, kPilotDatagramBytes + 1> buf{};
  for (;;) {
    const ssize_t n = ::recv(fd_, buf.data(), buf.size(), 0);
    if (n < 0) {
      break;  // EAGAIN: drained (another error leaves the input silent: the failsafe acts)
    }
    const std::optional<PilotFrame> f = decode_pilot_datagram(buf.data(), static_cast<std::size_t>(n));
    if (f) {
      ++received_;
      newest = f;
    } else {
      ++dropped_;
    }
  }
  return newest;
}

}  // namespace marv::gz
