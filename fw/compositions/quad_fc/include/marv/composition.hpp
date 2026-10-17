// The quad flight composition: what a composition tells its harness about itself.
#pragma once

#include <cstddef>
#include <cstdint>

#include "marv/arming/arming.hpp"
#include "marv/types/actuator.hpp"

namespace marv::composition {

inline constexpr std::size_t kMotors = kQuadXMotors;
inline constexpr std::size_t kServos = 0;
inline constexpr const char* kName = "quad_fc";

// The rate group's counters since init: rate executions and the rate loop's timing faults (decision 0034). Call from the
// tick's context (on the target, the IMU interrupt).
struct RateCounts {
  std::uint64_t executions = 0;
  std::uint32_t timing_faults = 0;
};
[[nodiscard]] RateCounts rate_counts() noexcept;

// The arming machine's state after the latest rate tick: Armed, ArmedHold and ArmedFailsafeCentred when armed, else the
// cause of the latest disarm or the first refusing condition (arming.hpp); Boot before the first rate tick. A single
// atomic read, callable from any context (the target's background loop prints it).
[[nodiscard]] arming::ArmState arming_status() noexcept;

// The attitude estimate as of the latest rate tick: its tilt, the angle between body down and NED down (rad, [0, pi]), and
// its validity (estimator::Mahony::valid). The tilt is 0 until the estimator is initialised. Callable from any context
// (the target's background loop prints it); the tilt is from two values published one after the other, so a read that
// overlaps a rate tick can mix two ticks.
struct AttitudeStatus {
  float tilt_rad = 0.0F;
  bool valid = false;
};
[[nodiscard]] AttitudeStatus attitude_status() noexcept;

}  // namespace marv::composition
