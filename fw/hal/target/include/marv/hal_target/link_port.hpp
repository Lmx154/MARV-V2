// The ground link's transport on the FC3v2 flight image (decision 0053 Q4; ground-segment unit 12b): the link service's
// Transport (marv/link/service.hpp) over the link CDC (usb.hpp), shaped like hal_sim's UDP transport
// (marv/hal_sim/link_udp.hpp). The CDC takes a few bytes per call, so frames wait whole in a queue that link_port_pump moves
// to it; a frame that does not fit is dropped whole, never cut. While no host has the link port open the queue is emptied and
// frames are dropped, so a host that opens it later reads no stale frames. Background loop only, never blocking; never from
// the tick. Not compiled into the HIL images (they have no link port).
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

namespace marv::hal_target {

// Copies received link bytes into `out`; returns how many (0: none).
[[nodiscard]] std::size_t link_port_receive(std::span<std::uint8_t> out) noexcept;

// Queues one frame whole, or drops it (no host has the port open, or the queue has no room for it).
void link_port_send(std::span<const std::uint8_t> frame) noexcept;

// Bytes link_port_send takes now without dropping a frame; 0 while no host has the port open.
[[nodiscard]] std::size_t link_port_writable() noexcept;

// Moves queued bytes to the link CDC as far as it takes them; empties the queue while no host has the port open.
void link_port_pump() noexcept;

}  // namespace marv::hal_target
