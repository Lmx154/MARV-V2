#pragma once

// The FC3v2 IMU (LSM6DSV on SPI1) and the target tick loop (quad spec §4.0 M1, packet P9; fw/hal/target/src/lsm6dsv.cpp).
// Single core: the LSM6DSV's FIFO watermark interrupt (INT1, level high, the highest IRQ priority) runs the tick. Each
// handler entry reads the FIFO, assembles and stamps the samples it holds (lsm6dsv_decode.hpp) and passes them to the
// sink in order, one call per sample. The counters are published to the background loop through a seqlock.

#include <cstdint>

#include "marv/types/imu_sample.hpp"
#include "marv/types/time.hpp"

namespace marv::hal_target {

using ImuSink = void (*)(const ImuSample&) noexcept;

// The composition's rate-group counters (rate executions, rate-loop timing faults), read in the interrupt after each
// batch's samples, so the published counters are one consistent snapshot. Provided by the image (main.cpp).
struct RateCounts {
  std::uint64_t executions = 0;
  std::uint32_t timing_faults = 0;
};
using RateCountsSource = RateCounts (*)() noexcept;

struct ImuStats {
  std::uint64_t irqs = 0;            // INT1 handler entries
  std::uint64_t words = 0;           // FIFO words read
  std::uint64_t samples = 0;         // samples assembled and stamped
  std::uint64_t ticks = 0;           // sink calls returned
  std::uint32_t fifo_overflows = 0;  // reads of FIFO_STATUS2 with FIFO_OVR_LATCHED set
  std::uint32_t spi_errors = 0;      // FIFO words whose tag is neither gyroscope nor accelerometer (a corrupt read)
  std::uint32_t partial = 0;         // samples missing the gyroscope or the accelerometer word (GyroValid or AccelValid clear)
  std::uint32_t stamp_clamps = 0;    // stamps raised to keep t_us strictly increasing (ImuStamper)
  std::uint32_t order_faults = 0;    // samples whose t_us did not exceed the previous one (expected 0: the check)
  std::uint32_t empty_irqs = 0;      // handler entries that found no FIFO word
  std::uint64_t actuator_writes = 0; // hal_actuators_write calls
  std::uint64_t actuator_drops = 0;  // motor frames dropped because the previous one had not left (motor_output.hpp)
  RateCounts rate{};                 // the composition's counters (RateCountsSource)
  TimeUs spacing_min_us = 0;         // smallest and largest t_us spacing since the previous imu_stats() (0: none)
  TimeUs spacing_max_us = 0;
  ImuSample last{};                  // the newest sample
};

struct ImuBoot {
  std::uint32_t spi_hz = 0;     // the SPI clock set
  std::int8_t freq_fine = 0;    // INTERNAL_FREQ_FINE (4Fh): the part's ODR trim, steps of 0.13 % (DS13476 §9.49)
  double odr_hz_by_trim = 0.0;  // 7680 Hz x (1 + 0.0013 x freq_fine), §9.49 (the stamping rule keeps the nominal T)
};

// The tick period the rate design was checked at, tick_period_num_us / tick_period_den microseconds (the parameters).
struct TickPeriod {
  std::int32_t num = 0;
  std::int32_t den = 0;
};

// Configures SPI1 and the LSM6DSV (WHO_AM_I check, software reset, ±2000 dps, ±16 g, 7.68 kHz high-performance, FIFO
// continuous with both sensors batched at 7.68 kHz, watermark one sample on INT1), then enables the interrupt; from
// then on every sample goes to `sink`, in the interrupt, and `rate_counts` is read after each batch. hal_panic on any
// failure, and when the period `design_period` is not exactly the one this part's INTERNAL_FREQ_FINE gives (DS13476
// §9.49, tick_period_from_trim): the rate design was then checked at the wrong period (decision 0014, Amendment
// 2026-10-16). Call once, from main, after composition::init.
ImuBoot imu_start(ImuSink sink, RateCountsSource rate_counts, TickPeriod design_period) noexcept;

// The hal_actuators_write calls so far (hal_target.cpp). Called only from the tick's counter publication.
[[nodiscard]] std::uint64_t actuator_writes() noexcept;

// A consistent snapshot of the counters (a seqlock read; never blocks the interrupt). Starts a new spacing window.
[[nodiscard]] ImuStats imu_stats() noexcept;

}  // namespace marv::hal_target
