// The L11 vertical-scripted composition: what a composition tells the SIL entry about itself, the attitude hook that a
// TRUTH_STATE SIL library calls before each tick (decision 0006 section B) and the vertical hook that a TRUTH_VERTICAL
// SIL library calls before each tick (decision 0020 section 1, item c). The composition latches each state handed over
// and, at each execution of the group that reads it, panics unless the latched stamp equals the sample stamp.
#pragma once

#include <cstddef>

#include "marv/types/actuator.hpp"
#include "marv/types/attitude_state.hpp"
#include "marv/types/vertical_state.hpp"

namespace marv::composition {

inline constexpr std::size_t kMotors = kQuadXMotors;
inline constexpr std::size_t kServos = 0;
inline constexpr const char* kName = "l11_vertical_scripted";

// Latches a. The stamp a.t_us is the sample stamp of the tick the state belongs to.
void attitude_input(const AttitudeState<float>& a) noexcept;

// Latches v. The stamp v.t_us is the sample stamp of the tick the state belongs to.
void vertical_input(const VerticalState<float>& v) noexcept;

}  // namespace marv::composition
