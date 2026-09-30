#pragma once

// Configuration of the L5 attitude law and angle mode (decision 0006 C and D). A plain struct with validate.
// The parameter-reading pair (from_params, load_config) is not here: the parameter ids arrive with the generator, and
// the firmware-facing init will be require_valid(from_params()). The config is taken by value in init().

#include <cmath>
#include <cstddef>
#include <cstdint>

#include "marv/prim/constants.hpp"
#include "marv/prim/vec.hpp"

namespace marv::attitude {

// Indices of the rate-setpoint Vec3 and of rate_max.
enum RateAxis : std::size_t { kRollAxis, kPitchAxis, kYawAxis };

inline constexpr std::size_t kRateAxes = prim::kSpatialDim;

template <class T>
struct AttitudeConfig {
  T kp{};                    // att_kp k, 1/s
  T yaw_weight{};            // att_yaw_weight w, 1, in (0, 1]
  T period{};                // attitude execution period N*T (att_loop_ratio * rate period), s
  T tilt_max{};              // angle_tilt_max, rad
  T yaw_deadband{};          // yaw_deadband d, stick fraction, in [0, 1)
  T yaw_alpha_min{};         // att_yaw_alpha_min, rad/s^2: the smallest yaw angular acceleration over the band box
  T yaw_t_cross{};           // att_yaw_t_cross, s: the latest linear-regime yaw-rate zero crossing over the band box
  prim::Vec3<T> rate_max{};  // rate_max_{roll,pitch,yaw}, rad/s
};

enum class ConfigError : std::uint8_t {
  None,
  NonFinite,
  Gain,
  YawWeight,
  Period,
  TiltMax,
  Deadband,
  AlphaMin,
  TCross,
  RateMax
};

// The first violated rule, else None. Written with negated comparisons, so a NaN is rejected.
//   k >= 0; 0 < w <= 1; T > 0; 0 < theta_max <= pi; 0 <= d < 1; alpha_min > 0; t_cross > 0; rate_max > 0 on every axis.
template <class T>
[[nodiscard]] ConfigError validate(const AttitudeConfig<T>& c) noexcept {
  using std::isfinite;
  bool finite = isfinite(c.kp) && isfinite(c.yaw_weight) && isfinite(c.period) && isfinite(c.tilt_max) &&
                isfinite(c.yaw_deadband) && isfinite(c.yaw_alpha_min) && isfinite(c.yaw_t_cross);
  for (std::size_t a = 0; a < kRateAxes; ++a) {
    finite = finite && isfinite(c.rate_max[a]);
  }
  if (!finite) {
    return ConfigError::NonFinite;
  }
  if (!(c.kp >= T(0))) {
    return ConfigError::Gain;
  }
  if (!(c.yaw_weight > T(0)) || !(c.yaw_weight <= T(1))) {
    return ConfigError::YawWeight;
  }
  if (!(c.period > T(0))) {
    return ConfigError::Period;
  }
  if (!(c.tilt_max > T(0)) || !(c.tilt_max <= static_cast<T>(prim::kPi))) {
    return ConfigError::TiltMax;
  }
  if (!(c.yaw_deadband >= T(0)) || !(c.yaw_deadband < T(1))) {
    return ConfigError::Deadband;
  }
  if (!(c.yaw_alpha_min > T(0))) {
    return ConfigError::AlphaMin;
  }
  if (!(c.yaw_t_cross > T(0))) {
    return ConfigError::TCross;
  }
  for (std::size_t a = 0; a < kRateAxes; ++a) {
    if (!(c.rate_max[a] > T(0))) {
      return ConfigError::RateMax;
    }
  }
  return ConfigError::None;
}

// hal_panic naming the violated rule unless validate(c) == None.
void require_valid(const AttitudeConfig<float>& c) noexcept;

extern template ConfigError validate<float>(const AttitudeConfig<float>&) noexcept;

}  // namespace marv::attitude
