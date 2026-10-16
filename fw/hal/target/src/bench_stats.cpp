#include "marv/hal_target/bench_stats.hpp"

#if defined(MARV_FC3V2_BENCH_STATS)

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include <hardware/irq.h>
#include <hardware/regs/m33.h>
#include <hardware/structs/m33.h>
#include <pico.h>

#include "marv/hal/hal.hpp"
#include "marv/hal_target/lsm6dsv_decode.hpp"
#include "marv/types/imu_sample.hpp"

namespace marv::hal_target::bench {
namespace {

[[nodiscard]] constexpr std::size_t index(Timed t) noexcept { return static_cast<std::size_t>(t); }

[[nodiscard]] std::uint32_t cycles() noexcept { return m33_hw->dwt_cyccnt; }

void record(Timing& t, std::uint32_t c) noexcept {
  ++t.count;
  t.max = std::max(t.max, c);
  t.window_max = std::max(t.window_max, c);
  ++t.buckets[bucket_of(c)];
}

void record(Spacing& s, std::uint32_t d) noexcept {
  ++s.count;
  s.sum += d;
  if (s.min == 0 || d < s.min) {
    s.min = d;
  }
  s.max = std::max(s.max, d);
}

// Round to the nearest integer; |v| is at most the int16 range, so the float is exact.
[[nodiscard]] std::int64_t to_lsb(float v) noexcept {
  return static_cast<std::int64_t>(v >= 0.0F ? v + 0.5F : v - 0.5F);
}

// Written by the IMU interrupt (all but timing[UsbIrq]) while g_imu_seq is odd, and by the USBCTRL handlers
// (timing[UsbIrq]) while g_usb_seq is odd.
Snapshot g_data{};
std::atomic<std::uint32_t> g_imu_seq{0};
std::atomic<std::uint32_t> g_usb_seq{0};
std::atomic<bool> g_imu_window_reset{false};
std::atomic<bool> g_usb_window_reset{false};

// The IMU interrupt's own state.
std::uint32_t g_entry = 0;
std::uint32_t g_mark = 0;
Timed g_stage = Timed::Read;
std::array<std::uint32_t, kIrqStages> g_cycles{};
std::uint32_t g_ran = 0;  // bit i: stage i ran in this entry
std::uint32_t g_prev_entry = 0;
bool g_prev_single = false;
std::uint32_t g_account = 0;
bool g_have_account = false;
float g_gyro_lsb_per_si = 0.0F;
float g_accel_lsb_per_si = 0.0F;

// The USBCTRL chain, preempted by the IMU interrupt: the cycles of the IMU entries that began inside it are subtracted.
std::uint32_t g_usb_entry = 0;
std::atomic<bool> g_usb_active{false};
std::atomic<std::uint32_t> g_usb_preempted{0};

void usb_begin() {
  g_usb_entry = cycles();
  g_usb_preempted.store(0, std::memory_order_relaxed);
  g_usb_active.store(true, std::memory_order_relaxed);
}

void usb_end() {
  g_usb_active.store(false, std::memory_order_relaxed);
  const std::uint32_t net = cycles() - g_usb_entry - g_usb_preempted.load(std::memory_order_relaxed);
  const std::uint32_t q = g_usb_seq.load(std::memory_order_relaxed);
  g_usb_seq.store(q + 1, std::memory_order_relaxed);
  std::atomic_thread_fence(std::memory_order_release);
  Timing& t = g_data.timing[index(Timed::UsbIrq)];
  if (g_usb_window_reset.exchange(false, std::memory_order_relaxed)) {
    t.window_max = 0;
  }
  record(t, net);
  g_usb_seq.store(q + 2, std::memory_order_release);
}

}  // namespace

void start() noexcept {
  m33_hw->demcr = m33_hw->demcr | M33_DEMCR_TRCENA_BITS;
  if ((m33_hw->dwt_ctrl & M33_DWT_CTRL_NOCYCCNT_BITS) != 0) {
    hal_panic("bench stats: DWT_CTRL.NOCYCCNT is set, this core has no cycle counter");
  }
  m33_hw->dwt_cyccnt = 0;
  m33_hw->dwt_ctrl = m33_hw->dwt_ctrl | M33_DWT_CTRL_CYCCNTENA_BITS;
  const Lsm6dsvScales s = lsm6dsv_scales();
  g_gyro_lsb_per_si = 1.0F / s.gyro_rad_s_per_lsb;
  g_accel_lsb_per_si = 1.0F / s.accel_m_s2_per_lsb;
  irq_add_shared_handler(USBCTRL_IRQ, &usb_begin, PICO_SHARED_IRQ_HANDLER_HIGHEST_ORDER_PRIORITY);
  irq_add_shared_handler(USBCTRL_IRQ, &usb_end, PICO_SHARED_IRQ_HANDLER_LOWEST_ORDER_PRIORITY);
}

void irq_begin() noexcept {
  g_entry = cycles();
  g_mark = g_entry;
  g_stage = Timed::Read;
  g_cycles.fill(0);
  g_ran = 1U << index(Timed::Read);
  const std::uint32_t q = g_imu_seq.load(std::memory_order_relaxed);
  g_imu_seq.store(q + 1, std::memory_order_relaxed);
  std::atomic_thread_fence(std::memory_order_release);
}

void stage(Timed next) noexcept {
  const std::uint32_t now = cycles();
  g_cycles[index(g_stage)] += now - g_mark;
  g_mark = now;
  g_stage = next;
  g_ran |= 1U << index(next);
}

void still_sample(const ImuSample& s) noexcept {
  const std::uint32_t both = imu_flag(ImuFlag::GyroValid) | imu_flag(ImuFlag::AccelValid);
  if ((s.flags & both) != both) {
    return;
  }
  ++g_data.still_samples;
  for (std::size_t a = 0; a < prim::kSpatialDim; ++a) {
    const std::int64_t g = to_lsb(s.gyro_rad_s[a] * g_gyro_lsb_per_si);
    const std::int64_t x = to_lsb(s.accel_m_s2[a] * g_accel_lsb_per_si);
    g_data.gyro_sum[a] += g;
    g_data.gyro_sq[a] += g * g;
    g_data.accel_sum[a] += x;
    g_data.accel_sq[a] += x * x;
  }
}

void irq_end(std::size_t samples) noexcept {
  const std::uint32_t end = cycles();
  g_cycles[index(g_stage)] += end - g_mark;
  if (g_imu_window_reset.exchange(false, std::memory_order_relaxed)) {
    for (std::size_t i = 0; i < kTimed; ++i) {
      if (i != index(Timed::UsbIrq)) {
        g_data.timing[i].window_max = 0;
      }
    }
    g_data.window_spacing = Spacing{};
  }
  for (std::size_t i = 0; i < kIrqStages; ++i) {
    if ((g_ran & (1U << i)) != 0) {
      record(g_data.timing[i], g_cycles[i]);
    }
  }
  record(g_data.timing[index(Timed::Irq)], end - g_entry);
  if (g_have_account) {
    record(g_data.timing[index(Timed::Account)], g_account);
  }
  if (samples == 1 && g_prev_single) {
    record(g_data.spacing, g_entry - g_prev_entry);
    record(g_data.window_spacing, g_entry - g_prev_entry);
  }
  g_prev_single = samples == 1;
  g_prev_entry = g_entry;
  g_data.max_batch = std::max(g_data.max_batch, static_cast<std::uint32_t>(samples));
  const bool in_usb = g_usb_active.load(std::memory_order_relaxed);
  if (in_usb) {
    ++g_data.usb_preemptions;
  }
  g_imu_seq.store(g_imu_seq.load(std::memory_order_relaxed) + 1, std::memory_order_release);
  const std::uint32_t done = cycles();
  g_account = done - end;
  g_have_account = true;
  if (in_usb) {
    g_usb_preempted.store(g_usb_preempted.load(std::memory_order_relaxed) + (done - g_entry),
                          std::memory_order_relaxed);
  }
}

const Snapshot& snapshot() noexcept {
  static Snapshot s{};
  for (;;) {
    const std::uint32_t a = g_imu_seq.load(std::memory_order_acquire);
    const std::uint32_t b = g_usb_seq.load(std::memory_order_acquire);
    if (((a | b) & 1U) != 0) {
      continue;
    }
    s = g_data;
    std::atomic_thread_fence(std::memory_order_acquire);
    if (g_imu_seq.load(std::memory_order_relaxed) == a && g_usb_seq.load(std::memory_order_relaxed) == b) {
      break;
    }
  }
  g_imu_window_reset.store(true, std::memory_order_relaxed);
  g_usb_window_reset.store(true, std::memory_order_relaxed);
  return s;
}

}  // namespace marv::hal_target::bench

#endif
