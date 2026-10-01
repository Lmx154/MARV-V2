#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "marv/types/actuator.hpp"
#include "marv/types/time.hpp"

namespace marv {

// One valid bit per motor, in logical-motor order: bit i is motor i + 1.
enum class RotorSpeedFlag : std::uint8_t { M1Valid = 0, M2Valid, M3Valid, M4Valid, Count };

static_assert(static_cast<std::size_t>(RotorSpeedFlag::Count) == kQuadXMotors, "one valid bit per motor");

[[nodiscard]] constexpr std::uint32_t rotor_speed_flag(RotorSpeedFlag f) noexcept {
  return std::uint32_t{1} << static_cast<unsigned>(f);
}

// The valid bit of the motor with array index `i` (logical number i + 1). pre: i < kQuadXMotors.
[[nodiscard]] constexpr std::uint32_t rotor_speed_valid_bit(std::size_t i) noexcept {
  return std::uint32_t{1} << static_cast<unsigned>(i);
}

inline constexpr std::uint32_t kRotorSpeedFlagsDefined = rotor_speed_flag(RotorSpeedFlag::Count) - 1;

// Rotor mechanical speed per motor, SI. If a motor's valid bit is clear its speed is exactly 0; bits outside
// kRotorSpeedFlagsDefined are 0.
struct RotorSpeedSample {
  TimeUs t_us;                                  // acquisition stamp, hal_time_us() timebase
  std::array<float, kQuadXMotors> omega_rad_s;  // [motor_index(m)], rotor mechanical speed, rad/s
  std::uint32_t flags;                          // rotor_speed_flag(...) bits
};

static_assert(std::is_trivially_copyable_v<RotorSpeedSample> && std::is_standard_layout_v<RotorSpeedSample>);

}  // namespace marv
