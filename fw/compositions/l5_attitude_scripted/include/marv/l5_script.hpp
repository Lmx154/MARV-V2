// The scripted inputs of the l5_attitude_scripted composition, as pure functions of the sample stamp: a piecewise-constant
// stick script, an exponential-sweep chirp added to the rate setpoint, and a constant yaw torque disturbance switched on at
// a stamp. No parameters are read here; the composition fills the structs from its scenario parameters and validates them
// (validate) at init.
//
// Sticks: segment k applies for t >= t_k until t_(k+1); before t_1 and with no segment the sticks are zero. A stick is the
// shaped value in [-1, 1] that angle mode takes (decision 0006 D, "Stick source at L5"): positive s_r rolls right side
// down, positive s_p pitches nose up, positive s_psi yaws nose right.
// Chirp: with L = ln(w_hi / w_lo), D = dur, tau = t - t0 in s, phi(tau) = w_lo D / L (exp(L tau / D) - 1) and
// d = A sin(phi) for 0 <= tau < D, else 0, in rad/s on the chirp axis (decision 0006 F: added to the attitude output, the
// rate setpoint, before the rate loop). The instantaneous frequency dphi/dtau = w_lo exp(L tau / D) runs from w_lo to w_hi.
// Disturbance: a constant torque on the yaw axis for t >= t0, N*m (decision 0006 F, the yaw-lock fallback case).
#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "marv/prim/constants.hpp"
#include "marv/prim/vec.hpp"
#include "marv/types/time.hpp"

namespace marv::composition {

template <class T>
struct StickSegment {
  std::int32_t t_us = 0;  // first stamp the segment applies to, us
  prim::Vec3<T> stick{};  // shaped sticks [s_r, s_p, s_psi], each in [-1, 1]
};

// K is the capacity: the used segments are the first `count`.
template <class T, std::size_t K>
struct StickScript {
  std::int32_t count = 0;
  std::array<StickSegment<T>, K> segment{};
};

enum class ChirpAxis : std::int32_t { None, Roll, Pitch, Yaw };

template <class T>
struct Chirp {
  ChirpAxis axis = ChirpAxis::None;
  T amp{};                  // A, rad/s
  T w_lo{};                 // rad/s
  T w_hi{};                 // rad/s
  std::int32_t t0_us = 0;   // sweep start, us
  std::int32_t dur_us = 0;  // sweep duration D, us
};

template <class T>
struct Disturbance {
  T yaw_nm{};              // constant yaw torque, N*m
  std::int32_t t0_us = 0;  // first stamp it acts at, us
};

enum class ScriptError : std::uint8_t {
  None,
  SegmentCount,
  SegmentTime,
  SegmentStick,
  ChirpAxis,
  ChirpAmplitude,
  ChirpBand,
  ChirpDuration,
  ChirpStart,
  DisturbanceTorque,
  DisturbanceStart
};

// The first violated rule, else None. Segments: 0 <= count <= K, stamps >= 0 and strictly increasing, sticks finite and
// in [-1, 1]. Chirp: axis in None..Yaw, A finite and >= 0; when the axis is not None also w_lo > 0, w_hi > w_lo (finite),
// dur > 0 and t0 >= 0. Disturbance: torque finite and t0 >= 0. Negated comparisons, so a NaN is rejected.
template <class T, std::size_t K>
[[nodiscard]] ScriptError validate(const StickScript<T, K>& s) noexcept {
  using std::abs;
  using std::isfinite;
  if (s.count < 0 || static_cast<std::size_t>(s.count) > K) {
    return ScriptError::SegmentCount;
  }
  for (std::size_t k = 0; k < static_cast<std::size_t>(s.count); ++k) {
    if (s.segment[k].t_us < 0 || (k > 0 && !(s.segment[k].t_us > s.segment[k - 1].t_us))) {
      return ScriptError::SegmentTime;
    }
    for (std::size_t a = 0; a < prim::kSpatialDim; ++a) {
      const T v = s.segment[k].stick[a];
      if (!isfinite(v) || !(abs(v) <= T(1))) {
        return ScriptError::SegmentStick;
      }
    }
  }
  return ScriptError::None;
}

template <class T>
[[nodiscard]] ScriptError validate(const Chirp<T>& c) noexcept {
  using std::isfinite;
  if (c.axis < ChirpAxis::None || c.axis > ChirpAxis::Yaw) {
    return ScriptError::ChirpAxis;
  }
  if (!isfinite(c.amp) || !(c.amp >= T(0))) {
    return ScriptError::ChirpAmplitude;
  }
  if (c.axis == ChirpAxis::None) {
    return ScriptError::None;
  }
  if (!isfinite(c.w_lo) || !isfinite(c.w_hi) || !(c.w_lo > T(0)) || !(c.w_hi > c.w_lo)) {
    return ScriptError::ChirpBand;
  }
  if (!(c.dur_us > 0)) {
    return ScriptError::ChirpDuration;
  }
  if (!(c.t0_us >= 0)) {
    return ScriptError::ChirpStart;
  }
  return ScriptError::None;
}

template <class T>
[[nodiscard]] ScriptError validate(const Disturbance<T>& d) noexcept {
  using std::isfinite;
  if (!isfinite(d.yaw_nm)) {
    return ScriptError::DisturbanceTorque;
  }
  if (!(d.t0_us >= 0)) {
    return ScriptError::DisturbanceStart;
  }
  return ScriptError::None;
}

// The sticks at stamp t_us. pre: validate(s) == None.
template <class T, std::size_t K>
[[nodiscard]] prim::Vec3<T> sticks_at(const StickScript<T, K>& s, TimeUs t_us) noexcept {
  prim::Vec3<T> sticks;
  for (std::size_t k = 0; k < static_cast<std::size_t>(s.count) && k < K; ++k) {
    if (t_us >= static_cast<TimeUs>(s.segment[k].t_us)) {
      sticks = s.segment[k].stick;
    }
  }
  return sticks;
}

// The chirp value d, rad/s, at stamp t_us; 0 for axis None and outside [t0, t0 + D). pre: validate(c) == None.
// expm1 and log1p keep the relative error of phi and L independent of L (small L would cancel in exp and log).
template <class T>
[[nodiscard]] T chirp_value(const Chirp<T>& c, TimeUs t_us) noexcept {
  using std::expm1;
  using std::log1p;
  using std::sin;
  if (c.axis == ChirpAxis::None || t_us < static_cast<TimeUs>(c.t0_us)) {
    return T(0);
  }
  const TimeUs tau_us = t_us - static_cast<TimeUs>(c.t0_us);
  const auto dur_us = static_cast<TimeUs>(c.dur_us);
  if (tau_us >= dur_us) {
    return T(0);
  }
  const T sweep = log1p((c.w_hi - c.w_lo) / c.w_lo);  // L
  const T x = sweep * (static_cast<T>(tau_us) / static_cast<T>(dur_us));
  const T dur_s = static_cast<T>(dur_us) / static_cast<T>(prim::kMicrosecondsPerSecond);
  const T phi = (c.w_lo * dur_s / sweep) * expm1(x);
  return c.amp * sin(phi);
}

// The chirp as a rate setpoint addition: d on the chirp axis, 0 elsewhere.
template <class T>
[[nodiscard]] prim::Vec3<T> chirp_rate(const Chirp<T>& c, TimeUs t_us) noexcept {
  prim::Vec3<T> d;
  if (c.axis != ChirpAxis::None) {
    d[static_cast<std::size_t>(c.axis) - 1] = chirp_value(c, t_us);
  }
  return d;
}

// The disturbance as a torque request addition: yaw_nm on the yaw axis from t0, 0 before and on the other axes.
template <class T>
[[nodiscard]] prim::Vec3<T> disturbance_torque(const Disturbance<T>& d, TimeUs t_us) noexcept {
  prim::Vec3<T> tau;
  if (t_us >= static_cast<TimeUs>(d.t0_us)) {
    tau[prim::kSpatialDim - 1] = d.yaw_nm;
  }
  return tau;
}

}  // namespace marv::composition
