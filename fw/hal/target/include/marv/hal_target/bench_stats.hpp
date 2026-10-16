#pragma once

// The FC3v2 bench statistics (quad spec §4.0 M1: "EMB-3 holds on measured stage times", "still-bench IMU samples sit
// inside the profile's bounds"; M1 packet P12a). Recorded only in the bench image (MARV_FC3V2_BENCH_STATS,
// fw/hal/target/CMakeLists.txt); in every other image the calls below are empty and nothing is recorded.
//
// Clock: the Cortex-M33 DWT cycle counter CYCCNT, one count per processor clock (clk_sys), 32 bits, at 0xE0001004; it
// counts while DEMCR.TRCENA (DEMCR 0xE000EDFC, bit 24) and DWT_CTRL.CYCCNTENA (DWT_CTRL 0xE0001000, bit 0) are set (Arm
// ARMv8-M Architecture Reference Manual DDI0553, DWT_CTRL, DWT_CYCCNT and DEMCR; CMSIS core_cm33.h in the pinned SDK:
// DWT_BASE 0xE0001000, DCB_BASE 0xE000EDF0 with DEMCR at offset 0xC, DWT_CTRL_CYCCNTENA_Msk, DCB_DEMCR_TRCENA_Msk). The
// code reaches them through the pinned SDK's m33_hw and M33_* definitions (hardware/structs/m33.h,
// hardware/regs/m33.h), the same addresses and bits. That header gives DWT_CTRL.NOCYCCNT a reset value of 1; the board
// reads NOCYCCNT 0 and a counting CYCCNT (probe read, 2026-10-16). start() panics if NOCYCCNT is set.
//
// What is timed, in cycles. The IMU interrupt (lsm6dsv.cpp) in stages, each the sum of its segments within one entry:
// Read (FIFO status, temperature and FIFO words over SPI), Decode (assembly, stamping and the driver's bookkeeping
// around the sink calls), Filter, Rate, Mixer, Gate (the composition's hal_tick_stage marks), Dshot
// (hal_actuators_write), Publish (the counter seqlock) and Bench (the still-bench sums). Irq: the entry from its first
// statement to its last mark. Account: the recording after the last mark, recorded at the next entry. UsbIrq: the
// USBCTRL interrupt's handler chain, from a handler at its head to one just before the stdio handler (which only pends
// the stdio worker interrupt), net of the IMU interrupts that preempted it. Each keeps a count, its largest value since
// start and since the previous snapshot, and a histogram (bucket_of). Spacing: the time between the entries of
// consecutive IMU interrupts that each delivered one sample; its spread about its mean is the variation of the
// interrupt's start (masked windows, the entry path).
//
// Still bench: per FRD axis, the count, sum and sum of squares of the gyroscope and accelerometer values of the
// delivered samples in LSB (value over the datasheet sensitivity, lsm6dsv_scales), samples with both sensors valid.

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "marv/prim/constants.hpp"
#include "marv/types/imu_sample.hpp"

namespace marv::hal_target::bench {

enum class Timed : std::uint8_t { Read, Decode, Filter, Rate, Mixer, Gate, Dshot, Publish, Bench, Irq, Account, UsbIrq };
inline constexpr std::size_t kTimed = static_cast<std::size_t>(Timed::UsbIrq) + 1;
// The stages inside one IMU interrupt entry: Read .. Bench.
inline constexpr std::size_t kIrqStages = static_cast<std::size_t>(Timed::Bench) + 1;

// Histogram: values below 2^(kSubBits+1) have a bucket each; above, every octave has 2^kSubBits buckets, the value's
// leading kSubBits+1 bits. Bucket b >= 2^(kSubBits+1) holds [t << s, (t+1) << s) with s = b / 2^kSubBits - 1 and
// t = b % 2^kSubBits + 2^kSubBits.
inline constexpr unsigned kSubBits = 2;
inline constexpr std::size_t kSub = std::size_t{1} << kSubBits;
inline constexpr std::size_t kBuckets =
    (static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::digits) - kSubBits + 1) * kSub;

[[nodiscard]] constexpr std::size_t bucket_of(std::uint32_t cycles) noexcept {
  const auto width = static_cast<unsigned>(std::bit_width(cycles));
  if (width <= kSubBits + 1) {
    return cycles;
  }
  const unsigned shift = width - (kSubBits + 1);
  return shift * kSub + (cycles >> shift);
}

struct Timing {
  std::uint64_t count = 0;
  std::uint32_t max = 0;         // largest since start
  std::uint32_t window_max = 0;  // largest since the previous snapshot
  std::array<std::uint32_t, kBuckets> buckets{};
};

struct Spacing {
  std::uint64_t count = 0;
  std::uint64_t sum = 0;
  std::uint32_t min = 0;  // 0: none yet
  std::uint32_t max = 0;
};

struct Snapshot {
  std::array<Timing, kTimed> timing{};
  Spacing spacing{};         // since start
  Spacing window_spacing{};  // since the previous snapshot
  std::uint32_t max_batch = 0;        // most samples one IMU interrupt delivered
  std::uint32_t usb_preemptions = 0;  // IMU interrupts that began while the USBCTRL handler chain ran
  std::uint64_t still_samples = 0;
  std::array<std::int64_t, prim::kSpatialDim> gyro_sum{};
  std::array<std::int64_t, prim::kSpatialDim> gyro_sq{};
  std::array<std::int64_t, prim::kSpatialDim> accel_sum{};
  std::array<std::int64_t, prim::kSpatialDim> accel_sq{};
};

#if defined(MARV_FC3V2_BENCH_STATS)
inline constexpr bool kEnabled = true;

// Starts CYCCNT and adds the USBCTRL timing handlers. Call once from main, after stdio_init_all and before imu_start.
void start() noexcept;
// The IMU interrupt: irq_begin first, stage at each boundary (Read runs from irq_begin), still_sample per delivered
// sample, irq_end last with the samples delivered.
void irq_begin() noexcept;
void stage(Timed next) noexcept;
void still_sample(const ImuSample& s) noexcept;
void irq_end(std::size_t samples) noexcept;
// Background loop only: a consistent copy (seqlock reads; never blocks the interrupts) in static storage, valid until
// the next call, which starts a new window.
[[nodiscard]] const Snapshot& snapshot() noexcept;
#else
inline constexpr bool kEnabled = false;

inline void start() noexcept {}
inline void irq_begin() noexcept {}
inline void stage(Timed) noexcept {}
inline void still_sample(const ImuSample&) noexcept {}
inline void irq_end(std::size_t) noexcept {}
#endif

}  // namespace marv::hal_target::bench
