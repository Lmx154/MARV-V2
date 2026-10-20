#pragma once

// The onboard logger's tick side on the FC3v2 (unit U2 LOG-TARGET; serena memory quad-m4/architect-onboard-logger-2026-10-18
// item 1): the SPSC ring (marv/log/ring.hpp) and the hooks that push into it from the tick context, the IMU interrupt, in tick
// order: the sample (the image's sample sink calls log_tick_sample before composition::tick), then that tick's pull of a new
// receiver frame (hal_manual_control) and its actuator write (hal_actuators_write). The composition is not changed and reads
// nothing written here (invariant L-2). A push is wait-free and bounded and calls no FatFs or SD function (invariant L-1);
// when the ring is full the newest record is dropped and counted (ring.hpp). Two images implement the hooks: the bench log image
// (MARV_FC3V2_LOG_BENCH, fw/hal/target/CMakeLists.txt) on its own ring of MARV_FC3V2_LOG_RING_BYTES bytes (src/log_tick.cpp,
// with log_ring), and the flight image with the logger (MARV_FC3V2_FLIGHT_LOG) through the arena logger's published ring
// pointer (src/flight_log.cpp, marv/hal_target/flight_log.hpp; log_ring is not defined there).
//
// The rate-tick record's arm state is the composition's ArmState code as the image's arm-state source reads it in the write
// (composition::arming_status, which the rate tick publishes before its write). Its gate state is kGateUnknown: the
// composition does not publish its motor-output gate, and the hook does not derive it.
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>

#include "marv/log/ring.hpp"
#include "marv/types/actuator.hpp"
#include "marv/types/imu_sample.hpp"
#include "marv/types/manual_control.hpp"

namespace marv::hal_target {

inline constexpr std::uint8_t kGateUnknown = std::numeric_limits<std::uint8_t>::max();

// The arm state code of the latest rate tick; called in the tick. Before one is set the record carries 0.
using ArmStateSource = std::uint8_t (*)() noexcept;
void log_tick_set_arm_source(ArmStateSource source) noexcept;

// Tick context only.
void log_tick_sample(const ImuSample& s) noexcept;
void log_tick_rate(std::span<const DshotValue> motor) noexcept;
void log_tick_manual(const ManualControl& m) noexcept;

// The consumer's end of the ring (the background loop).
[[nodiscard]] log::Ring& log_ring() noexcept;

}  // namespace marv::hal_target
