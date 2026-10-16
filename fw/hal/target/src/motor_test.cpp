#include "marv/hal_target/motor_test.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "marv/hal/hal.hpp"

namespace marv {
namespace hal_target {
namespace {

// Console to tick: two slots and the index of the one the reader uses. The writer (background loop) keeps `g_next`, the
// slot it fills, and publishes it by flipping the index; it never writes the slot the index names.
std::array<MotorTestPoll, 2> g_slot{};
std::atomic<std::size_t> g_active{0};

void publish(const MotorTestPoll& p) noexcept {
  const std::size_t next = 1 - g_active.load(std::memory_order_relaxed);
  g_slot[next] = p;
  g_active.store(next, std::memory_order_release);
}

// Tick to console: g_seq is odd while the tick writes g_status.
std::atomic<std::uint32_t> g_seq{0};
MotorTestStatus g_status{};

}  // namespace

void motor_test_post_cancel(MotorTestCancel reason) noexcept {
  MotorTestPoll p = g_slot[g_active.load(std::memory_order_relaxed)];
  ++p.cancel_count;
  p.cancel_reason = reason;
  publish(p);
}

void motor_test_post_request(const MotorTestRequest& request) noexcept {
  MotorTestPoll p = g_slot[g_active.load(std::memory_order_relaxed)];
  ++p.request_count;
  p.request_cancel_count = p.cancel_count;
  p.request = request;
  publish(p);
}

MotorTestStatus motor_test_status() noexcept {
  MotorTestStatus s{};
  for (;;) {
    const std::uint32_t q = g_seq.load(std::memory_order_acquire);
    if ((q & 1U) != 0) {
      continue;
    }
    s = g_status;
    std::atomic_thread_fence(std::memory_order_acquire);
    if (g_seq.load(std::memory_order_relaxed) == q) {
      return s;
    }
  }
}

}  // namespace hal_target

MotorTestPoll hal_motor_test() noexcept {
  return hal_target::g_slot[hal_target::g_active.load(std::memory_order_acquire)];
}

void hal_motor_test_report(const MotorTestEvent& event) noexcept {
  using hal_target::g_seq;
  using hal_target::g_status;
  const std::uint32_t q = g_seq.load(std::memory_order_relaxed);
  g_seq.store(q + 1, std::memory_order_relaxed);
  std::atomic_thread_fence(std::memory_order_release);
  g_status.last = event;
  ++g_status.events;
  g_seq.store(q + 2, std::memory_order_release);
}

}  // namespace marv
