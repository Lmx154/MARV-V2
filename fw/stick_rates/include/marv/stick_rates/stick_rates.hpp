#pragma once

// QF-1, the rates curve: a stick deflection s in [-1, 1] (FRD sense, core 3) to a body-rate setpoint in rad/s, per axis.
// A pure function of the stick and three numbers per axis; it reads no parameter (the composition builds the
// configuration) and holds no state.
//
// The curve is the "Actual" rates model of Betaflight 4.5 (centre sensitivity, maximum rate, expo), written here from its
// mathematics. With a = |s|, centre sensitivity c (rad/s), maximum rate m (rad/s) and expo e in [0, 1]:
//
//   g(a) = (1 - e) a^2 + e a^6
//   r(s) = sign(s) [ c a + (m - c) g(a) ]
//
// A baseline linear response c s, plus a ramp of the extra rate m - c that reaches its full size at full stick; the expo
// e moves the ramp between a^2 and a^6 (PR 9495, "a true inflexion point curve adjustment"; "changing expo will not
// change your centre sensitivity or your max rate"). Consequences used by the tests: r(0) = 0; r(+-1) = +-m; r is odd;
// r'(0) = c, because g'(0) = 0; r is non-decreasing on [0, 1] when 0 <= e <= 1 and c <= m (r'(a) = c + (m - c) g'(a),
// g'(a) = 2 (1 - e) a + 6 e a^5 >= 0).
//
// Source of the model: Betaflight 4.5.2, commit 024f8e13d4e642eb6a380308685b9ea3aa3ef1a2, src/main/fc/rc.c:209-217
// (applyActualRates; its expo is a percentage, e = rcExpo / 100, and its rates are in deg/s, here rad/s) and PR 9495
// (https://github.com/betaflight/betaflight/pull/9495) for the meaning of the three values. Only the curve is taken: not
// Betaflight's rate limit, its stick smoothing or its other rates types. Where the maximum rate is below the centre
// sensitivity Betaflight flattens the ramp to zero; here that configuration is refused by validate.
// r is computed as c (a - g) + m g, the same function, so that full stick gives m exactly: at a = 1, g is exactly 1 and
// a - g exactly 0.

#include <cmath>
#include <cstddef>
#include <cstdint>

#include "marv/prim/vec.hpp"

namespace marv::stick_rates {

// Indices of the rate-setpoint Vec3, the same as the attitude library's.
enum Axis : std::size_t { kRollAxis, kPitchAxis, kYawAxis };

template <class T>
struct RateCurve {
  T centre{};  // centre sensitivity c, rad/s: the slope of r at the centre, as if it continued linearly to full stick
  T max{};     // maximum rate m, rad/s: |r| at full stick
  T expo{};    // expo e, 1, in [0, 1]
};

template <class T>
struct RatesConfig {
  RateCurve<T> roll{};
  RateCurve<T> pitch{};
  RateCurve<T> yaw{};
};

enum class ConfigError : std::uint8_t { None, NonFinite, Centre, Max, Expo };

// The first violated rule, else None. Written with negated comparisons, so a NaN is rejected.
//   0 < c <= m; 0 <= e <= 1.
template <class T>
[[nodiscard]] ConfigError validate(const RateCurve<T>& c) noexcept {
  using std::isfinite;
  if (!(isfinite(c.centre) && isfinite(c.max) && isfinite(c.expo))) {
    return ConfigError::NonFinite;
  }
  if (!(c.centre > T(0))) {
    return ConfigError::Centre;
  }
  if (!(c.max >= c.centre)) {
    return ConfigError::Max;
  }
  if (!(c.expo >= T(0)) || !(c.expo <= T(1))) {
    return ConfigError::Expo;
  }
  return ConfigError::None;
}

template <class T>
[[nodiscard]] ConfigError validate(const RatesConfig<T>& c) noexcept {
  const ConfigError e = validate(c.roll);
  if (e != ConfigError::None) {
    return e;
  }
  const ConfigError p = validate(c.pitch);
  if (p != ConfigError::None) {
    return p;
  }
  return validate(c.yaw);
}

// The rate setpoint r(s), rad/s. A stick beyond [-1, 1] is held at full stick; a non-finite stick gives 0. The curve is
// assumed valid (validate).
template <class T>
[[nodiscard]] T rate_setpoint(const RateCurve<T>& curve, T stick) noexcept {
  using std::copysign;
  using std::fabs;
  using std::isfinite;
  if (!isfinite(stick)) {
    return T(0);
  }
  T a = fabs(stick);
  if (a > T(1)) {
    a = T(1);
  }
  const T a2 = a * a;
  const T a4 = a2 * a2;
  const T a6 = a4 * a2;
  const T g = a2 + curve.expo * (a6 - a2);
  return copysign(curve.centre * (a - g) + curve.max * g, stick);
}

template <class T>
[[nodiscard]] prim::Vec3<T> rate_setpoint(const RatesConfig<T>& cfg, const prim::Vec3<T>& stick) noexcept {
  return prim::Vec3<T>(rate_setpoint(cfg.roll, stick[kRollAxis]), rate_setpoint(cfg.pitch, stick[kPitchAxis]),
                       rate_setpoint(cfg.yaw, stick[kYawAxis]));
}

extern template struct RateCurve<float>;
extern template struct RatesConfig<float>;
extern template ConfigError validate<float>(const RateCurve<float>&) noexcept;
extern template ConfigError validate<float>(const RatesConfig<float>&) noexcept;
extern template float rate_setpoint<float>(const RateCurve<float>&, float) noexcept;
extern template prim::Vec3<float> rate_setpoint<float>(const RatesConfig<float>&, const prim::Vec3<float>&) noexcept;

}  // namespace marv::stick_rates
