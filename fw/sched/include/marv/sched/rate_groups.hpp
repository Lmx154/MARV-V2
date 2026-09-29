#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "marv/hal/hal.hpp"
#include "marv/types/time.hpp"

namespace marv::sched {

// Rate-group scheduler (core EMB-2). The tick is one primary-IMU sample; group i runs at the tick rate divided by
// divisors[i]. `due(n)` is a pure function of n and the divisors: no state, no accumulation, so any tick can be
// evaluated, including the first tick after a jump, and a counter can never drift or overflow.
//
// Dispatch contract: the composition calls due(n) once per tick and runs the groups whose bit is set, with plain
// `if`s in a fixed order, lowest index first. Bit i of the mask is group i. Tick 0 fires every group. There are no
// callbacks, function pointers or virtual calls here (core section 9).
template <std::size_t N>
class RateGroups {
  static_assert(N >= 1, "at least one rate group");
  static_assert(N <= std::numeric_limits<std::uint32_t>::digits, "one mask bit per group");

 public:
  // Every divisor must be >= 1, else returns false and leaves the object unchanged (uninitialised if it was).
  [[nodiscard]] bool init(const std::array<std::uint32_t, N>& divisors) noexcept {
    for (const std::uint32_t d : divisors) {
      if (d < 1) {
        return false;
      }
    }
    divisors_ = divisors;
    ready_ = true;
    return true;
  }

  // Bit i set iff n is an exact multiple of divisors[i]. hal_panic before a successful init.
  [[nodiscard]] std::uint32_t due(Tick n) const noexcept {
    if (!ready_) {
      hal_panic("RateGroups::due before init");
    }
    std::uint32_t mask = 0;
    for (std::size_t i = 0; i < N; ++i) {
      if (n % divisors_[i] == 0) {
        mask |= std::uint32_t{1} << i;
      }
    }
    return mask;
  }

  [[nodiscard]] std::uint32_t divisor(std::size_t i) const noexcept { return divisors_[i]; }

 private:
  std::array<std::uint32_t, N> divisors_{};
  bool ready_ = false;
};

}  // namespace marv::sched
