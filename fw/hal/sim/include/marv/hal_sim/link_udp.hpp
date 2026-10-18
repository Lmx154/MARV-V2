// hal_sim's ground-link transport (decision 0053 Q4; ground-segment unit 6): one non-blocking UDP socket bound to
// 127.0.0.1:<port>. The ground station's helper (tools/gs) sends to that port; the sender of the latest datagram, an empty one
// included, is the peer every frame goes to. Without an open socket nothing is received or sent. Called from the SIL's link
// service, between tick calls; never from a tick.
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

namespace marv::hal_sim {

// Binds the socket to 127.0.0.1:port. False, with no socket open, if the port is 0, a socket is already open, or the socket
// cannot be created or bound.
[[nodiscard]] bool link_udp_open(std::uint16_t port) noexcept;

// Closes the socket; nothing is received or sent from then on.
void link_udp_close() noexcept;

// Copies waiting bytes into `out` and returns how many; 0 when no datagram waits. A datagram longer than `out` is handed out
// over successive calls, in order.
[[nodiscard]] std::size_t link_udp_receive(std::span<std::uint8_t> out) noexcept;

// Sends `frame` as one datagram to the peer. Dropped when no peer is known yet or the socket would block.
void link_udp_send(std::span<const std::uint8_t> frame) noexcept;

}  // namespace marv::hal_sim
