#include "marv/hal_sim/link_udp.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "marv/prim/constants.hpp"

namespace marv::hal_sim {
namespace {

struct Link {
  int fd = -1;
  bool have_peer = false;
  sockaddr_in peer{};
  // The datagram being handed out: bytes [next, size) are still to be read.
  std::array<std::uint8_t, prim::kUdpIpv4PayloadMaxBytes> datagram{};
  std::size_t size = 0;
  std::size_t next = 0;
};

Link g_link;

// Reads the next datagram into g_link.datagram; false if none waits (or the socket is closed). An empty datagram only makes
// its sender the peer.
[[nodiscard]] bool fetch() noexcept {
  while (g_link.fd >= 0) {
    sockaddr_in from{};
    socklen_t from_len = sizeof(from);
    const ssize_t n = ::recvfrom(g_link.fd, g_link.datagram.data(), g_link.datagram.size(), MSG_DONTWAIT,
                                 reinterpret_cast<sockaddr*>(&from), &from_len);
    if (n < 0) {
      return false;  // EAGAIN: nothing waits; any other error: the datagram is lost, as on a lossy link
    }
    if (from_len == sizeof(from) && from.sin_family == AF_INET) {
      g_link.peer = from;
      g_link.have_peer = true;
    }
    if (n > 0) {
      g_link.size = static_cast<std::size_t>(n);
      g_link.next = 0;
      return true;
    }
  }
  return false;
}

}  // namespace

bool link_udp_open(std::uint16_t port) noexcept {
  if (port == 0 || g_link.fd >= 0) {
    return false;
  }
  const int fd = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (fd < 0) {
    return false;
  }
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (::bind(fd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) != 0) {
    static_cast<void>(::close(fd));
    return false;
  }
  g_link.fd = fd;
  g_link.have_peer = false;
  g_link.size = 0;
  g_link.next = 0;
  return true;
}

void link_udp_close() noexcept {
  if (g_link.fd >= 0) {
    static_cast<void>(::close(g_link.fd));
  }
  g_link.fd = -1;
  g_link.have_peer = false;
  g_link.size = 0;
  g_link.next = 0;
}

std::size_t link_udp_receive(std::span<std::uint8_t> out) noexcept {
  if (out.empty() || (g_link.next == g_link.size && !fetch())) {
    return 0;
  }
  const std::size_t n = std::min(out.size(), g_link.size - g_link.next);
  std::copy_n(g_link.datagram.begin() + static_cast<std::ptrdiff_t>(g_link.next), n, out.begin());
  g_link.next += n;
  return n;
}

void link_udp_send(std::span<const std::uint8_t> frame) noexcept {
  if (g_link.fd < 0 || !g_link.have_peer) {
    return;
  }
  static_cast<void>(::sendto(g_link.fd, frame.data(), frame.size(), MSG_DONTWAIT,
                             reinterpret_cast<const sockaddr*>(&g_link.peer), sizeof(g_link.peer)));
}

}  // namespace marv::hal_sim
