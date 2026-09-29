// Test-only composition for the SIL library tests: what a composition tells the SIL entry about itself.
#pragma once

#include <cstddef>

#include "marv/types/actuator.hpp"

namespace marv::composition {

inline constexpr std::size_t kMotors = kQuadXMotors;
inline constexpr std::size_t kServos = 0;
inline constexpr const char* kName = "sil-test-composition";

}  // namespace marv::composition
