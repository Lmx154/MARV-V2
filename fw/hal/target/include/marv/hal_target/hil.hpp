#pragma once

// The bench HIL image's link and tick (HIL stage S3a, decision 0046 items 3-5; fw/hal/target/src/hil.cpp, format in
// hil_format.hpp). Built only into the image with MARV_FC3V2_HIL (fw/hal/target/CMakeLists.txt): quad_fc runs on the
// FC3v2 from the IMU samples and ManualControl frames a host sends over USB, and its DShot values go back over USB. No IMU,
// receiver or motor output is started: no DShot program is loaded and no motor pad is driven.

#include <cstdint>
#include <span>

#include "marv/hal_target/imu.hpp"
#include "marv/types/actuator.hpp"
#include "marv/types/time.hpp"

namespace marv::hal_target {

// The composition's state for the status record, read from the background loop (functions callable from any context).
struct HilCompositionStatus {
  std::uint32_t arming_state = 0;  // the ArmState code (arming.hpp)
  std::uint32_t attitude_faults = 0;
  std::uint32_t attitude_timing_faults = 0;
};
using HilCompositionStatusSource = HilCompositionStatus (*)() noexcept;

struct HilConfig {
  ImuSink sink = nullptr;                          // called once per sample in the tick interrupt, as the IMU driver's sink
  RateCountsSource rate_counts = nullptr;          // read in the tick interrupt after each record's ticks
  HilCompositionStatusSource composition_status = nullptr;
  std::uint64_t param_schema_hash = 0;             // reported in the hello record
  std::uint32_t param_count = 0;
  std::int32_t tick_period_num_us = 0;
  std::int32_t tick_period_den = 0;
  std::int32_t rate_loop_divisor = 0;
  TimeUs deadline_us = 0;                          // D_HIL (hil.cpp)
  const char* version = "";
};

// Starts the cycle counter and the tick interrupt. hal_panic if a function is missing or D_HIL is not a positive 32-bit
// microsecond count. Call once, from main, after composition::init.
void hil_start(const HilConfig& config) noexcept;

// A host opened the port: the receive buffer and both sequence counters restart and the hello record is sent.
void hil_connect() noexcept;

// Background loop, as often as it can run: reads and parses what USB holds (ticking each sample record), checks the
// deadline, sends the DShot reports and, once per second while `connected`, the status record.
void hil_service(bool connected) noexcept;

// hal_actuators_write's body in this image (hal_target.cpp), in the tick: queues the values with the tick number for the
// background loop to send.
void hil_actuators_write(std::span<const DshotValue> motor) noexcept;

}  // namespace marv::hal_target
