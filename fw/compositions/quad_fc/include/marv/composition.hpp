// The quad flight composition: what a composition tells its harness about itself.
#pragma once

#include <cstddef>
#include <cstdint>

#include "marv/arming/arming.hpp"
#include "marv/params/param_types.hpp"
#include "marv/types/actuator.hpp"

namespace marv::composition {

inline constexpr std::size_t kMotors = kQuadXMotors;
inline constexpr std::size_t kServos = 0;
inline constexpr const char* kName = "quad_fc";

// The outcome of validate: `reason` is null if the table is accepted, else the text init panics with.
struct Verdict {
  const char* reason = nullptr;
  [[nodiscard]] bool ok() const noexcept { return reason == nullptr; }
};

// Whether `table` (a candidate of the quad_fc parameter set's kParamCount records) is a configuration the board boots with:
// it runs, in the order init and the target's start-ups run them, every check that ends in a hal_panic on a parameter value
// (the rate, mixer, gyro chain, attitude and arming configurations, the composition's own limits and channel numbers, and
// the motor_pad permutation and the receiver channel map of the target's start-up), and returns the first that fails.
// Reads only `table`; never calls hal_panic or a HAL function, and changes nothing. init calls it on the runtime
// table and panics with its reason, so init panics iff validate rejects the runtime table (invariant V-1).
[[nodiscard]] Verdict validate(ParamTable table) noexcept;

// The rate group's counters since init: rate executions and the rate loop's timing faults (decision 0034). Call from the
// tick's context (on the target, the IMU interrupt).
struct RateCounts {
  std::uint64_t executions = 0;
  std::uint32_t timing_faults = 0;
};
[[nodiscard]] RateCounts rate_counts() noexcept;

// The attitude law's counters since init (angle mode, decision 0043): its faults and, of them, its spacing faults. Atomic
// reads of values published after each execution, callable from any context (the target's background loop prints them).
struct AttitudeCounts {
  std::uint32_t faults = 0;
  std::uint32_t timing_faults = 0;
};
[[nodiscard]] AttitudeCounts attitude_counts() noexcept;

// The arming machine's state after the latest rate tick: Armed, ArmedHold and ArmedFailsafeCentred when armed, else the
// cause of the latest disarm or the first refusing condition (arming.hpp); Boot before the first rate tick. A single
// atomic read, callable from any context (the target's background loop prints it).
[[nodiscard]] arming::ArmState arming_status() noexcept;

// The status of the config this boot uses (decision 0053): Ok for the factory table, else why a stored config cannot be used
// (arming.hpp ConfigStatus); anything but Ok refuses arming with ArmState::ConfigRefused. Fixed at init: set_config_status
// is for the harness to call before init (the default is Ok; a call after init takes effect at the next init, which also
// keeps the last status given).
void set_config_status(arming::ConfigStatus status) noexcept;
[[nodiscard]] arming::ConfigStatus config_status() noexcept;

// The maintenance grant (decision 0053, ground spec GS-4): request_maintenance asks the rate tick for it, from any context
// (the target's background loop); a single atomic store. The next rate tick that starts disarmed grants it: from then on
// arming is refused (ArmState::Maintenance) until the next init; there is no un-grant. A request that tick finds the
// vehicle armed is dropped, not granted and changes nothing, and the requester asks again. maintenance_granted is a single
// atomic read of the grant as of the latest rate tick (false until a tick answers); true also means the vehicle is and
// stays disarmed. Neither can disarm an armed vehicle.
void request_maintenance() noexcept;
[[nodiscard]] bool maintenance_granted() noexcept;

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
