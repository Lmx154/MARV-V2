// The ground link's transport on the FC3v2 flight image (marv/hal_target/link_port.hpp): a queue of whole frames in front of
// the link CDC's byte transport (usb.hpp).
#include "marv/hal_target/link_port.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "marv/hal_target/usb.hpp"
#include "marv/link/messages.hpp"

#ifndef MARV_FC3V2_LINK_QUEUE_FRAMES
#error "MARV_FC3V2_LINK_QUEUE_FRAMES is set by fw/hal/target/CMakeLists.txt"
#endif

namespace marv::hal_target {
namespace {

// The queue holds MARV_FC3V2_LINK_QUEUE_FRAMES frames of the longest length (PLACEHOLDER, fw/hal/target/CMakeLists.txt).
constexpr std::size_t kQueueBytes = std::size_t{MARV_FC3V2_LINK_QUEUE_FRAMES} * link::kFrameBytesMax;
static_assert(kQueueBytes >= 2 * link::kFrameBytesMax,
              "a listing waits for room for one parameter's two frames (fw/link/src/service.cpp), so the queue must hold them");

// Bytes [g_head, g_tail) of g_queue wait to be sent.
std::array<std::uint8_t, kQueueBytes> g_queue{};
std::size_t g_head = 0;
std::size_t g_tail = 0;

void clear() noexcept {
  g_head = 0;
  g_tail = 0;
}

}  // namespace

std::size_t link_port_receive(std::span<std::uint8_t> out) noexcept { return link_usb_receive(out); }

void link_port_send(std::span<const std::uint8_t> frame) noexcept {
  if (frame.empty() || frame.size() > link_port_writable()) {
    return;
  }
  if (g_tail + frame.size() > g_queue.size()) {
    std::copy(g_queue.begin() + static_cast<std::ptrdiff_t>(g_head), g_queue.begin() + static_cast<std::ptrdiff_t>(g_tail),
              g_queue.begin());
    g_tail -= g_head;
    g_head = 0;
  }
  std::copy(frame.begin(), frame.end(), g_queue.begin() + static_cast<std::ptrdiff_t>(g_tail));
  g_tail += frame.size();
}

std::size_t link_port_writable() noexcept { return link_usb_connected() ? g_queue.size() - (g_tail - g_head) : 0; }

void link_port_pump() noexcept {
  if (!link_usb_connected()) {
    clear();
    return;
  }
  while (g_head < g_tail) {
    const std::size_t n = link_usb_send(std::span<const std::uint8_t>{g_queue.data() + g_head, g_tail - g_head});
    if (n == 0) {
      break;
    }
    g_head += n;
  }
  if (g_head == g_tail) {
    clear();
  }
}

}  // namespace marv::hal_target
