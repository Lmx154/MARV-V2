// Test-only composition for the truth-vertical SIL tests: what a composition tells the SIL entry about itself, and the
// vertical hook that a TRUTH_VERTICAL library needs (decision 0020 section 1, item c). A composition without
// vertical_input cannot be built into a TRUTH_VERTICAL library.
#pragma once

#include <cstddef>

#include "marv/types/actuator.hpp"
#include "marv/types/imu_sample.hpp"
#include "marv/types/vertical_state.hpp"

namespace marv::composition {

inline constexpr std::size_t kMotors = kQuadXMotors;
inline constexpr std::size_t kServos = 0;
inline constexpr const char* kName = "truth-vertical-test-composition";

void init() noexcept;
void tick(const ImuSample& s) noexcept;
void vertical_input(const VerticalState<float>& v) noexcept;

}  // namespace marv::composition
