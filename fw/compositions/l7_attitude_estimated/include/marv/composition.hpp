// The L7 attitude-estimated composition: what a composition tells the SIL entry about itself. It has no attitude hook: the
// attitude state is the estimator's, from the IMU sample alone (decision 0042 item 4), so no TRUTH_STATE library is built.
#pragma once

#include <cstddef>

#include "marv/types/actuator.hpp"

namespace marv::composition {

inline constexpr std::size_t kMotors = kQuadXMotors;
inline constexpr std::size_t kServos = 0;
inline constexpr const char* kName = "l7_attitude_estimated";

}  // namespace marv::composition
