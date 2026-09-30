// Test-only composition for the truth-state SIL tests: what a composition tells the SIL entry about itself, and the
// attitude hook that a TRUTH_STATE library needs (decision 0006 section B). A composition without attitude_input
// cannot be built into a TRUTH_STATE library.
#pragma once

#include <cstddef>

#include "marv/types/actuator.hpp"
#include "marv/types/attitude_state.hpp"
#include "marv/types/imu_sample.hpp"

namespace marv::composition {

inline constexpr std::size_t kMotors = kQuadXMotors;
inline constexpr std::size_t kServos = 0;
inline constexpr const char* kName = "truth-state-test-composition";

void init() noexcept;
void tick(const ImuSample& s) noexcept;
void attitude_input(const AttitudeState<float>& a) noexcept;

}  // namespace marv::composition
