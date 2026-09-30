#pragma once

// The L3 mixer / allocation (quad spec 4 L3, decision 0004 items 2 to 5).
// Opening: in a thrust (N) and a torque (N*m, about the FRD CM); out DShot 48 to 2047 per motor (logical motor
// order), three per-axis saturation flags (roll, pitch, yaw: achieved < requested) and the achieved collective
// thrust (N); `mix` is that opening. Allocation is in per-motor thrust and does not iterate; the priority is
// roll/pitch > yaw > collective thrust (0004 item 3).

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "marv/prim/constants.hpp"
#include "marv/prim/mat.hpp"
#include "marv/prim/vec.hpp"
#include "marv/types/actuator.hpp"

namespace marv::mixer {

inline constexpr std::size_t kMotors = kQuadXMotors;
// Wrench axes: one thrust and kSpatialDim torques.
inline constexpr std::size_t kAxes = 1 + prim::kSpatialDim;

// Column order of the mixer M (f_i = sum over axes of M[i, axis] * u_axis) and row order of the effectiveness B.
enum Axis : std::size_t { kThrust, kRoll, kPitch, kYaw };

static_assert(kYaw + 1 == kAxes && kMotors == kAxes, "M = B^-1 is square");

template <class T>
struct MixerConfig {
  prim::Mat<T, kMotors, kAxes> m{};  // row i = logical motor i + 1
  T thrust_coeff{};                  // k, N/(rad/s)^2
  T omega_idle{};                    // rad/s
  T omega_min{};                     // ESC-map endpoints, rad/s (DShot kDshotThrottleMin, kDshotThrottleMax)
  T omega_max{};
};

enum class ConfigError : std::uint8_t {
  None,
  NonFinite,
  ThrustCoeff,
  IdleNegative,
  IdleBelowMin,
  MinNotBelowMax,
  IdleNotBelowMax,
  ThrustColumn,
  ZeroTorqueInfeasible
};

template <class T>
struct Request {
  T thrust{};              // N
  prim::Vec3<T> torque{};  // N*m, FRD: [0] roll, [1] pitch, [2] yaw
};

struct SaturationFlags {
  bool roll = false;
  bool pitch = false;
  bool yaw = false;
};

template <class T>
struct Allocation {
  std::array<T, kMotors> f{};  // per-motor thrust, N, each in [f_min, f_max]
  T achieved_thrust{};         // N, c of 0004 item 5 (before DShot quantisation)
  prim::Vec3<T> achieved_torque{};
  SaturationFlags flags{};
};

template <class T>
struct MixerOutput {
  std::array<DshotValue, kMotors> dshot{};
  SaturationFlags flags{};
  T achieved_thrust{};
};

template <class T>
[[nodiscard]] T f_min(const MixerConfig<T>& c) noexcept {
  return c.thrust_coeff * (c.omega_idle * c.omega_idle);
}

template <class T>
[[nodiscard]] T f_max(const MixerConfig<T>& c) noexcept {
  return c.thrust_coeff * (c.omega_max * c.omega_max);
}

// The first violated rule, else None. Written with negated comparisons, so a NaN is rejected.
template <class T>
[[nodiscard]] ConfigError validate(const MixerConfig<T>& c) noexcept {
  using std::isfinite;
  bool finite = isfinite(c.thrust_coeff) && isfinite(c.omega_idle) && isfinite(c.omega_min) &&
                isfinite(c.omega_max);
  for (const T& v : c.m.e) {
    finite = finite && isfinite(v);
  }
  if (!finite) {
    return ConfigError::NonFinite;
  }
  if (!(c.thrust_coeff > T(0))) {
    return ConfigError::ThrustCoeff;
  }
  if (!(c.omega_idle >= T(0))) {
    return ConfigError::IdleNegative;
  }
  if (!(c.omega_idle >= c.omega_min)) {
    return ConfigError::IdleBelowMin;
  }
  if (!(c.omega_min < c.omega_max)) {
    return ConfigError::MinNotBelowMax;
  }
  if (!(c.omega_idle < c.omega_max)) {
    return ConfigError::IdleNotBelowMax;
  }
  for (std::size_t i = 0; i < kMotors; ++i) {
    if (!(c.m(i, kThrust) > T(0))) {
      return ConfigError::ThrustColumn;
    }
  }
  // Zero torque is achievable iff max_i f_min/M[i,thrust] <= min_j f_max/M[j,thrust], i.e. for every pair.
  const T lo = f_min(c);
  const T hi = f_max(c);
  for (std::size_t i = 0; i < kMotors; ++i) {
    for (std::size_t j = 0; j < kMotors; ++j) {
      if (!(lo * c.m(j, kThrust) <= hi * c.m(i, kThrust))) {
        return ConfigError::ZeroTorqueInfeasible;
      }
    }
  }
  return ConfigError::None;
}

// pre: validate(c) == None. Total for any request, including NaN and infinities: every f_i is in [f_min, f_max]
// (a NaN goes to f_min); nothing more is specified for a non-finite request.
//
// f = s * (M_roll tx + M_pitch ty) + t * M_yaw tz + c * M_thrust. With a_i = M[i,roll] tx + M[i,pitch] ty,
// y_i = M[i,yaw] tz and m_i = M[i,thrust] > 0, some c keeps every f_i in [f_min, f_max] for (s, t) iff, for every
// ordered pair i != j,
//   s A_ij + t Y_ij <= R_ij,  A_ij = a_j m_i - a_i m_j,  Y_ij = y_j m_i - y_i m_j,  R_ij = f_max m_i - f_min m_j.
// (R_ij >= 0 is the validated zero-torque feasibility.) s is the largest value in [0, 1] with t = 0
// (s = min(1, min over A_ij > 0 of R_ij / A_ij)); s < 1 is case a and then t = 0. Otherwise t is the largest value in
// [0, 1] with s = 1 (t = min(1, min over Y_ij > 0 of (R_ij - A_ij) / Y_ij)), case c. c is the thrust request clamped
// to [max_i (f_min - p_i)/m_i, min_i (f_max - p_i)/m_i], p_i = s a_i + t y_i, case b.
template <class T>
[[nodiscard]] Allocation<T> allocate(const MixerConfig<T>& cfg, const Request<T>& req) noexcept {
  const T lo = f_min(cfg);
  const T hi = f_max(cfg);
  const T tx = req.torque[0];
  const T ty = req.torque[1];
  const T tz = req.torque[2];

  std::array<T, kMotors> a{};
  std::array<T, kMotors> y{};
  std::array<T, kMotors> m{};
  for (std::size_t i = 0; i < kMotors; ++i) {
    m[i] = cfg.m(i, kThrust);
    a[i] = cfg.m(i, kRoll) * tx + cfg.m(i, kPitch) * ty;
    y[i] = cfg.m(i, kYaw) * tz;
  }

  T s = T(1);
  for (std::size_t i = 0; i < kMotors; ++i) {
    for (std::size_t j = 0; j < kMotors; ++j) {
      if (i == j) {
        continue;
      }
      const T pair_a = a[j] * m[i] - a[i] * m[j];
      if (pair_a > T(0)) {
        const T r = (hi * m[i] - lo * m[j]) / pair_a;
        if (r < s) {
          s = r;
        }
      }
    }
  }

  T t = T(0);
  if (s >= T(1)) {
    t = T(1);
    for (std::size_t i = 0; i < kMotors; ++i) {
      for (std::size_t j = 0; j < kMotors; ++j) {
        if (i == j) {
          continue;
        }
        const T pair_y = y[j] * m[i] - y[i] * m[j];
        if (pair_y > T(0)) {
          T room = (hi * m[i] - lo * m[j]) - (a[j] * m[i] - a[i] * m[j]);
          if (room < T(0)) {
            room = T(0);
          }
          const T r = room / pair_y;
          if (r < t) {
            t = r;
          }
        }
      }
    }
  }

  std::array<T, kMotors> p{};
  for (std::size_t i = 0; i < kMotors; ++i) {
    p[i] = s * a[i] + t * y[i];
  }
  T c_lo = (lo - p[0]) / m[0];
  T c_hi = (hi - p[0]) / m[0];
  for (std::size_t i = 1; i < kMotors; ++i) {
    const T l = (lo - p[i]) / m[i];
    const T h = (hi - p[i]) / m[i];
    if (l > c_lo) {
      c_lo = l;
    }
    if (h < c_hi) {
      c_hi = h;
    }
  }
  T c = req.thrust;
  if (c > c_hi) {
    c = c_hi;
  }
  if (c < c_lo) {
    c = c_lo;
  }

  Allocation<T> out;
  for (std::size_t i = 0; i < kMotors; ++i) {
    T f = p[i] + c * m[i];
    if (!(f >= lo)) {
      f = lo;
    } else if (f > hi) {
      f = hi;
    }
    out.f[i] = f;
  }
  out.achieved_thrust = c;
  out.achieved_torque = prim::Vec3<T>(s * tx, s * ty, t * tz);
  out.flags.roll = s < T(1) && tx != T(0);
  out.flags.pitch = s < T(1) && ty != T(0);
  out.flags.yaw = t < T(1) && tz != T(0);
  return out;
}

// omega = sqrt(f / k); DShot = the inverse of the linear-in-omega ESC map (omega_min at kDshotThrottleMin, omega_max
// at kDshotThrottleMax), rounded to nearest, then clamped to [ceil(D(omega_idle)), kDshotThrottleMax]. pre:
// validate(cfg) == None.
template <class T>
[[nodiscard]] std::array<DshotValue, kMotors> thrust_to_dshot(const MixerConfig<T>& cfg,
                                                              const std::array<T, kMotors>& f) noexcept {
  using std::ceil;
  using std::round;
  using std::sqrt;
  const T d_min = static_cast<T>(prim::kDshotThrottleMin);
  const T d_max = static_cast<T>(prim::kDshotThrottleMax);
  const T omega_span = cfg.omega_max - cfg.omega_min;
  const T dshot_span = d_max - d_min;
  const T d_lo = ceil(d_min + (cfg.omega_idle - cfg.omega_min) / omega_span * dshot_span);

  std::array<DshotValue, kMotors> out{};
  for (std::size_t i = 0; i < kMotors; ++i) {
    const T omega = sqrt(f[i] / cfg.thrust_coeff);
    T d = round(d_min + (omega - cfg.omega_min) / omega_span * dshot_span);
    if (!(d >= d_lo)) {
      d = d_lo;
    } else if (d > d_max) {
      d = d_max;
    }
    out[i] = DshotValue::from_raw(static_cast<std::uint16_t>(d)).value_or(DshotValue::stop());
  }
  return out;
}

// The L3 opening: allocate, then thrust_to_dshot. pre: validate(cfg) == None.
template <class T>
[[nodiscard]] MixerOutput<T> mix(const MixerConfig<T>& cfg, const Request<T>& req) noexcept {
  const Allocation<T> al = allocate(cfg, req);
  MixerOutput<T> out;
  out.dshot = thrust_to_dshot(cfg, al.f);
  out.flags = al.flags;
  out.achieved_thrust = al.achieved_thrust;
  return out;
}

// The product parameter set's mixer: idle_speed, mixer_m<i>_*, rotor_thrust_coeff, rotor_speed_min/max. Not
// validated. pre: params_init succeeded.
[[nodiscard]] MixerConfig<float> from_params() noexcept;

// hal_panic naming the violated rule unless validate(c) == None.
void require_valid(const MixerConfig<float>& c) noexcept;

// from_params, then require_valid: the firmware-facing init.
[[nodiscard]] MixerConfig<float> load_config() noexcept;

extern template Allocation<float> allocate<float>(const MixerConfig<float>&, const Request<float>&) noexcept;
extern template std::array<DshotValue, kMotors> thrust_to_dshot<float>(
    const MixerConfig<float>&, const std::array<float, kMotors>&) noexcept;
extern template MixerOutput<float> mix<float>(const MixerConfig<float>&, const Request<float>&) noexcept;
extern template ConfigError validate<float>(const MixerConfig<float>&) noexcept;

}  // namespace marv::mixer
