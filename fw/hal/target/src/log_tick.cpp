// The onboard logger's tick side (marv/hal_target/log_tick.hpp).
#include "marv/hal_target/log_tick.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "marv/hal/hal.hpp"
#include "marv/log/records.hpp"

#ifndef MARV_FC3V2_LOG_RING_BYTES
#error "MARV_FC3V2_LOG_RING_BYTES is set by fw/hal/target/CMakeLists.txt"
#endif

namespace marv::hal_target {

static_assert(MARV_FC3V2_LOG_RING_BYTES % log::kCellBytes == 0, "the ring is a whole number of cells");

// External linkage and this name: the image's post-link check (tools/ci/check_log_ring.py, invariant L-5) reads its size.
std::array<log::Cell, MARV_FC3V2_LOG_RING_BYTES / log::kCellBytes> g_log_ring_cells{};

namespace {

log::Ring g_ring{g_log_ring_cells};
std::atomic<ArmStateSource> g_arm_source{nullptr};
// Tick context only: the frame count of the last ManualControl pushed.
std::uint32_t g_frames_logged = 0;

}  // namespace

void log_tick_set_arm_source(ArmStateSource source) noexcept { g_arm_source.store(source, std::memory_order_release); }

void log_tick_sample(const ImuSample& s) noexcept { static_cast<void>(g_ring.push_imu(s)); }

void log_tick_rate(std::span<const DshotValue> motor) noexcept {
  log::RateTickRecord r;
  r.now_us = hal_time_us();
  const ArmStateSource source = g_arm_source.load(std::memory_order_acquire);
  r.arm_state = source != nullptr ? source() : 0;
  r.gate_state = kGateUnknown;
  for (std::size_t i = 0; i < r.dshot.size() && i < motor.size(); ++i) {
    r.dshot[i] = motor[i];
  }
  static_cast<void>(g_ring.push_rate_tick(r));
}

void log_tick_manual(const ManualControl& m) noexcept {
  if (m.frames == g_frames_logged) {
    return;
  }
  g_frames_logged = m.frames;
  static_cast<void>(g_ring.push_manual(m));
}

log::Ring& log_ring() noexcept { return g_ring; }

}  // namespace marv::hal_target
