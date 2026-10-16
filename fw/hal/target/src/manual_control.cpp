#include "marv/hal_target/manual_control.hpp"

#include <array>
#include <atomic>
#include <cstddef>

#include "marv/hal/hal.hpp"

namespace marv {
namespace hal_target {
namespace {

// Receive path to tick: two slots and the index of the one the reader uses. The writer fills the other slot and
// publishes it by flipping the index; it never writes the slot the index names.
std::array<ManualControl, 2> g_slot{};
std::atomic<std::size_t> g_active{0};

}  // namespace

void manual_control_publish(const ManualControl& m) noexcept {
  const std::size_t next = 1 - g_active.load(std::memory_order_relaxed);
  g_slot[next] = m;
  g_active.store(next, std::memory_order_release);
}

}  // namespace hal_target

ManualControl hal_manual_control() noexcept {
  return hal_target::g_slot[hal_target::g_active.load(std::memory_order_acquire)];
}

}  // namespace marv
