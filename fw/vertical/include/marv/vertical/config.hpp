#pragma once

// Configuration of the L11 vertical loop (decision 0020 section 1). A plain struct with validate; the library reads no
// parameters (the product parameter entries and the composition arrive with later stages).

#include <cmath>
#include <cstdint>

namespace marv::vertical {

template <class T>
struct VerticalConfig {
  T kp_z{};           // k_z, 1/s: height error to climb-rate setpoint
  T kp_v{};           // k_v, N*s/m: vertical-velocity proportional gain
  T ki_v{};           // k_i, N/m: vertical-velocity integral gain
  T period{};         // design period T of a vertical execution, s
  T hover_thrust{};   // T_h, N: the hover feed-forward and the invalid-state output
};

enum class ConfigError : std::uint8_t { None, NonFinite, NegativeGain, Period, HoverThrust, IntegralWithoutProportional };

// The first violated rule, else None. Written with negated comparisons, so a NaN is rejected.
//   all values finite; k_z >= 0, k_v >= 0, k_i >= 0; T > 0; T_h > 0; not (k_v == 0 and k_i != 0): an integrator with no
//   proportional term.
template <class T>
[[nodiscard]] ConfigError validate(const VerticalConfig<T>& c) noexcept {
  using std::isfinite;
  if (!(isfinite(c.kp_z) && isfinite(c.kp_v) && isfinite(c.ki_v) && isfinite(c.period) && isfinite(c.hover_thrust))) {
    return ConfigError::NonFinite;
  }
  if (!(c.kp_z >= T(0)) || !(c.kp_v >= T(0)) || !(c.ki_v >= T(0))) {
    return ConfigError::NegativeGain;
  }
  if (!(c.period > T(0))) {
    return ConfigError::Period;
  }
  if (!(c.hover_thrust > T(0))) {
    return ConfigError::HoverThrust;
  }
  if (c.kp_v == T(0) && c.ki_v != T(0)) {
    return ConfigError::IntegralWithoutProportional;
  }
  return ConfigError::None;
}

extern template ConfigError validate<float>(const VerticalConfig<float>&) noexcept;

}  // namespace marv::vertical
