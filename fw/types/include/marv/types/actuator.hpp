#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <type_traits>

#include "marv/prim/constants.hpp"

namespace marv {

// Logical quad-X motor numbers, viewed from above (core 3).
enum class Motor : std::uint8_t { M1FrontRight = 1, M2RearLeft, M3FrontLeft, M4RearRight };

inline constexpr std::size_t kQuadXMotors = static_cast<std::size_t>(Motor::M4RearRight);

[[nodiscard]] constexpr std::size_t motor_index(Motor m) noexcept { return static_cast<std::size_t>(m) - 1; }

// pre: i < kQuadXMotors
[[nodiscard]] constexpr Motor motor_from_index(std::size_t i) noexcept { return static_cast<Motor>(i + 1); }

// Holds only {0 = stop} union [kDshotThrottleMin, kDshotThrottleMax].
class DshotValue {
 public:
  constexpr DshotValue() noexcept = default;  // stop

  [[nodiscard]] static constexpr DshotValue stop() noexcept { return {}; }

  [[nodiscard]] static constexpr bool is_valid_raw(std::uint16_t r) noexcept {
    return r == 0 || (r >= prim::kDshotThrottleMin && r <= prim::kDshotThrottleMax);
  }

  [[nodiscard]] static constexpr std::optional<DshotValue> from_raw(std::uint16_t r) noexcept {
    return is_valid_raw(r) ? std::optional<DshotValue>{DshotValue{r}} : std::nullopt;
  }

  [[nodiscard]] constexpr std::uint16_t raw() const noexcept { return raw_; }

  friend constexpr bool operator==(DshotValue, DshotValue) noexcept = default;

 private:
  constexpr explicit DshotValue(std::uint16_t r) noexcept : raw_{r} {}

  std::uint16_t raw_ = 0;
};

struct ServoUs {
  std::uint16_t us = 0;  // pulse width, us; 0 = no pulse
};

template <std::size_t NMotors, std::size_t NServos>
struct ActuatorOutput {
  std::array<DshotValue, NMotors> motor{};  // [motor_index(m)], default = stop
  std::array<ServoUs, NServos> servo{};     // default = no pulse
};

static_assert(std::is_trivially_copyable_v<DshotValue> && std::is_standard_layout_v<DshotValue>);
static_assert(std::is_trivially_copyable_v<ServoUs> && std::is_standard_layout_v<ServoUs>);

}  // namespace marv
