// The fixed L0 test composition: what a composition tells the SIL entry about itself.
#pragma once

#include <cstddef>

#include "marv/types/actuator.hpp"

namespace marv::composition {

inline constexpr std::size_t kMotors = kQuadXMotors;
inline constexpr std::size_t kServos = 0;
inline constexpr const char* kName = "l0";

}  // namespace marv::composition
