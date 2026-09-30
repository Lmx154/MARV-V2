// The L5 attitude-scripted composition: what a composition tells the SIL entry about itself, and the attitude hook that a
// TRUTH_STATE SIL library calls before each tick (decision 0006 section B). The composition latches the state handed
// over and, at each attitude execution, panics unless the latched stamp equals the sample stamp.
#pragma once

#include <cstddef>

#include "marv/types/actuator.hpp"
#include "marv/types/attitude_state.hpp"

namespace marv::composition {

inline constexpr std::size_t kMotors = kQuadXMotors;
inline constexpr std::size_t kServos = 0;
inline constexpr const char* kName = "l5_attitude_scripted";

// Latches a. The stamp a.t_us is the sample stamp of the tick the state belongs to.
void attitude_input(const AttitudeState<float>& a) noexcept;

}  // namespace marv::composition
