// The quad flight composition: what a composition tells its harness about itself.
#pragma once

#include <cstddef>
#include <cstdint>

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

}  // namespace marv::composition
