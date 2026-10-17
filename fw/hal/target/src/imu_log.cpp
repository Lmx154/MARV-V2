// The bench IMU log's ring and USB writer (fw/hal/target/include/marv/hal_target/imu_log.hpp, format in imu_log_format.hpp).
//
// Ring size. When the host stops reading, the SDK's stdio driver waits PICO_STDIO_USB_STDOUT_TIMEOUT_US in its write before
// it gives up (stdio_usb.c, stdio_usb_out_chars), so the background loop can be held that long inside one write. The ring
// holds that long at the nominal ODR kOdrHz, ceil(timeout x ODR), rounded up to a power of two so the free-running indices
// wrap correctly and index with a mask. The measured part ODR is below nominal (7500.3 Hz), so the ring holds a little
// more than that time. A stall longer than the timeout loses data in the SDK driver itself (the host sees a sequence gap or
// a CRC failure) and the ring then overflows; both are counted. RingHighWater in the status record is the measured need.
//
// Frame size. One data record holds the samples that arrive in one stdio USB background task interval,
// PICO_STDIO_USB_TASK_INTERVAL_US (the SDK's period for servicing the device), ceil(ODR x interval), so a record never
// waits longer than the device is serviced anyway and the 16 bytes of framing stay a small part of it.
#include <array>
#include <atomic>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <span>

#include <pico/stdio/driver.h>
#include <pico/stdio_usb.h>
#include <parts/lsm6dsv.hpp>

#include "marv/hal/hal.hpp"
#include "marv/hal_target/imu.hpp"
#include "marv/hal_target/imu_log.hpp"
#include "marv/hal_target/imu_log_format.hpp"
#include "marv/prim/constants.hpp"

namespace marv::hal_target {
namespace {

namespace fmt = imu_log;

constexpr std::uint64_t kUsPerS = static_cast<std::uint64_t>(prim::kMicrosecondsPerSecond);
constexpr std::size_t kStallSamples =
    static_cast<std::size_t>((std::uint64_t{PICO_STDIO_USB_STDOUT_TIMEOUT_US} * lsm6dsv::kOdrHz + kUsPerS - 1) / kUsPerS);
constexpr std::size_t kRingSamples = std::bit_ceil(kStallSamples);
constexpr std::size_t kFrameSamples =
    static_cast<std::size_t>((std::uint64_t{PICO_STDIO_USB_TASK_INTERVAL_US} * lsm6dsv::kOdrHz + kUsPerS - 1) / kUsPerS);
static_assert(kRingSamples >= kStallSamples && kFrameSamples >= 1 && kFrameSamples <= kRingSamples);

constexpr TimeUs kStatusPeriodUs = static_cast<TimeUs>(prim::kMicrosecondsPerSecond);

std::array<ImuSample, kRingSamples> g_ring{};
// Free-running; the producer writes g_head, the consumer g_tail. Occupancy g_head - g_tail never exceeds kRingSamples.
std::atomic<std::uint32_t> g_head{0};
std::atomic<std::uint32_t> g_tail{0};
std::atomic<std::uint32_t> g_overflows{0};
std::atomic<std::uint32_t> g_high_water{0};

std::array<std::uint8_t, fmt::data_record_bytes(kFrameSamples)> g_frame{};
std::uint32_t g_seq = 0;
TimeUs g_status_due = 0;

void send(std::span<const std::uint8_t> bytes) noexcept {
  stdio_usb.out_chars(reinterpret_cast<const char*>(bytes.data()), static_cast<int>(bytes.size()));
}

void send_status() noexcept {
  const ImuStats s = imu_stats();
  std::array<std::uint32_t, fmt::kStatusFields> f{};
  const auto at = [&f](fmt::StatusField k) -> std::uint32_t& { return f[static_cast<std::size_t>(k)]; };
  at(fmt::StatusField::RingOverflows) = g_overflows.load(std::memory_order_relaxed);
  at(fmt::StatusField::RingHighWater) = g_high_water.load(std::memory_order_relaxed);
  at(fmt::StatusField::DeadlineMisses) = hal_imu_deadline_misses();
  at(fmt::StatusField::FifoOverflows) = s.fifo_overflows;
  at(fmt::StatusField::TimingFaults) = s.rate.timing_faults;
  at(fmt::StatusField::StampClamps) = s.stamp_clamps;
  at(fmt::StatusField::OrderFaults) = s.order_faults;
  at(fmt::StatusField::SpiErrors) = s.spi_errors;
  at(fmt::StatusField::Delivered) = static_cast<std::uint32_t>(s.samples);
  at(fmt::StatusField::Logged) = g_tail.load(std::memory_order_relaxed);
  std::array<std::uint8_t, fmt::kStatusRecordBytes> buf{};
  fmt::RecordWriter w{buf};
  w.begin(fmt::kStatusMagic, g_seq++, static_cast<std::uint32_t>(f.size()));
  for (const std::uint32_t v : f) {
    w.put_u32(v);
  }
  send(std::span{buf}.first(w.end()));
}

}  // namespace

void imu_log_push(const ImuSample& s) noexcept {
  const std::uint32_t head = g_head.load(std::memory_order_relaxed);
  const std::uint32_t used = head - g_tail.load(std::memory_order_acquire);
  if (used >= kRingSamples) {
    g_overflows.store(g_overflows.load(std::memory_order_relaxed) + 1, std::memory_order_relaxed);
    return;
  }
  g_ring[head & (kRingSamples - 1)] = s;
  g_head.store(head + 1, std::memory_order_release);
  if (used + 1 > g_high_water.load(std::memory_order_relaxed)) {
    g_high_water.store(used + 1, std::memory_order_relaxed);
  }
}

void imu_log_service() noexcept {
  if (!stdio_usb_connected()) {
    g_tail.store(g_head.load(std::memory_order_acquire), std::memory_order_release);
    return;
  }
  for (;;) {
    const std::uint32_t tail = g_tail.load(std::memory_order_relaxed);
    if (g_head.load(std::memory_order_acquire) - tail < kFrameSamples) {
      break;
    }
    fmt::RecordWriter w{g_frame};
    w.begin(fmt::kDataMagic, g_seq++, static_cast<std::uint32_t>(kFrameSamples));
    for (std::size_t i = 0; i < kFrameSamples; ++i) {
      w.put_sample(g_ring[(tail + i) & (kRingSamples - 1)]);
    }
    g_tail.store(tail + static_cast<std::uint32_t>(kFrameSamples), std::memory_order_release);
    send(std::span{g_frame}.first(w.end()));
  }
  const TimeUs now = hal_time_us();
  if (now >= g_status_due) {
    g_status_due = now + kStatusPeriodUs;
    send_status();
  }
}

}  // namespace marv::hal_target
