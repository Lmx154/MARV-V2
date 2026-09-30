#pragma once

// The L5 angle-mode setpoint generator (decision 0006 D). Opening: the sticks (s_r, s_p, s_psi) and the measured
// AttitudeState; out the setpoint quaternion q_sp and the yaw-rate command (world down, rad/s) for AttitudeLaw.
//
//   tilt     t = theta_max (s_r, s_p) / max(1, |(s_r, s_p)|); q_xy(t) = [cos(|t|/2), sin(|t|/2) t/|t|, 0]
//   heading  psi_m = 2 atan2(q.z, q.w) with (q.w, q.z) signed so that the result lies in (-pi, pi]
//   yaw-rate mode (|s_psi| > d): the heading setpoint tracks the current heading, yaw_rate_cmd =
//            sign(s_psi) (|s_psi| - d) / (1 - d) rate_max_yaw;  q_sp = q^ (x) q_xy(conj(q^) (x) q_z(psi_m) (x) q_xy(t))
//   braking  the first execution with |s_psi| <= d after yaw-rate mode records sigma_r = sign(psi_dot_m),
//            omega_r = |psi_dot_m|, t_r; the setpoint is yaw-rate mode's with yaw_rate_cmd = 0. The heading locks at the
//            first braking execution with sigma_r psi_dot_m <= 0 (the crossing) or with dt >= t_cross and
//            dt alpha_min >= omega_r (the fallback, at max(t_cross, omega_r / alpha_min)); psi_lock = psi_m there.
//   locked   q_sp = q_z(psi_lock) (x) q_xy(t), yaw_rate_cmd = 0. Nothing is re-armed until |s_psi| > d (guard a).
// psi_dot_m is the world-down component of the measured body rate, rotate(q^, omega_frd).z.

#include <cmath>
#include <cstdint>

#include "marv/attitude/config.hpp"
#include "marv/attitude/tilt_yaw.hpp"
#include "marv/prim/constants.hpp"
#include "marv/prim/quat.hpp"
#include "marv/prim/vec.hpp"
#include "marv/types/attitude_state.hpp"

namespace marv::attitude {

template <class T>
struct AngleSticks {
  T roll{};   // s_r in [-1, 1]; positive rolls right side down
  T pitch{};  // s_p in [-1, 1]; positive pitches nose up
  T yaw{};    // s_psi in [-1, 1]; positive yaws nose right
};

// q_sp is the zero quaternion when !valid, which AttitudeLaw::execute takes as an invalid setpoint (zero norm), so
// passing the output straight through gives the fault path of decision 0006 C.
template <class T>
struct AngleOutput {
  prim::Quat<T> q_sp{T(0), T(0), T(0), T(0)};
  T yaw_rate_cmd{};  // rad/s about world down
  bool valid = false;
};

enum class AnglePhase : std::uint8_t { Uninit, YawRate, Braking, Locked };

template <class T>
class AngleMode {
 public:
  // pre: validate(cfg) == None. Drops the braking state and the lock.
  void init(AttitudeConfig<T> cfg) noexcept {
    cfg_ = cfg;
    drop();
  }

  // The next valid execution re-initialises.
  void reset() noexcept { drop(); }

  [[nodiscard]] AnglePhase phase() const noexcept { return phase_; }
  [[nodiscard]] T heading_lock() const noexcept { return psi_lock_; }

  [[nodiscard]] AngleOutput<T> execute(const AngleSticks<T>& s, const AttitudeState<T>& a) noexcept {
    using std::abs;
    using std::copysign;
    using std::isfinite;
    bool ok = a.valid && is_finite(a.q) && isfinite(a.omega_frd[0]) && isfinite(a.omega_frd[1]) &&
              isfinite(a.omega_frd[2]) && isfinite(s.roll) && isfinite(s.pitch) && isfinite(s.yaw);
    ok = ok && abs(s.roll) <= T(1) && abs(s.pitch) <= T(1) && abs(s.yaw) <= T(1);
    T norm_q = T(0);
    if (ok) {
      norm_q = a.q.norm();
      ok = isfinite(norm_q) && norm_q > T(0);
    }
    if (!ok) {
      drop();
      return AngleOutput<T>{};
    }
    const prim::Quat<T> q_hat = a.q.normalized();
    const T psi_m = heading(a.q);
    const T rate_m = q_hat.rotate(a.omega_frd)[kYawAxis];
    if (!isfinite(rate_m)) {
      drop();
      return AngleOutput<T>{};
    }

    const bool active = abs(s.yaw) > cfg_.yaw_deadband;
    if (phase_ == AnglePhase::Uninit) {
      if (active) {
        phase_ = AnglePhase::YawRate;
      } else {
        phase_ = AnglePhase::Locked;
        psi_lock_ = psi_m;
      }
    } else if (active) {
      phase_ = AnglePhase::YawRate;
    } else if (phase_ == AnglePhase::YawRate) {
      phase_ = AnglePhase::Braking;
      sigma_ = rate_m > T(0) ? T(1) : (rate_m < T(0) ? -T(1) : T(0));
      omega_r_ = abs(rate_m);
      t_r_us_ = a.t_us;
    }
    if (phase_ == AnglePhase::Braking) {
      const T dt = static_cast<T>(a.t_us - t_r_us_) / static_cast<T>(prim::kMicrosecondsPerSecond);
      const bool crossing = sigma_ * rate_m <= T(0);
      const bool fallback = dt >= cfg_.yaw_t_cross && dt * cfg_.yaw_alpha_min >= omega_r_;
      if (crossing || fallback) {
        phase_ = AnglePhase::Locked;
        psi_lock_ = psi_m;
      }
    }

    const prim::Quat<T> q_xy_t = tilt_quat(s.roll, s.pitch);
    AngleOutput<T> out;
    out.valid = true;
    if (phase_ == AnglePhase::Locked) {
      out.q_sp = (heading_quat(psi_lock_) * q_xy_t).canonical();
      return out;
    }
    const prim::Quat<T> q_des = heading_quat(psi_m) * q_xy_t;
    const TiltYawSplit<T> split = split_tilt_yaw((q_hat.conjugate() * q_des).canonical());
    out.q_sp = (q_hat * split.q_xy).canonical();
    if (phase_ == AnglePhase::YawRate) {
      const T mag = (abs(s.yaw) - cfg_.yaw_deadband) / (T(1) - cfg_.yaw_deadband);
      out.yaw_rate_cmd = copysign(mag, s.yaw) * cfg_.rate_max[kYawAxis];
    }
    return out;
  }

 private:
  // q_xy(t) for t = theta_max (s_r, s_p) / max(1, |(s_r, s_p)|): the angle is theta_max min(1, n), n = |(s_r, s_p)|,
  // and the axis is (s_r, s_p) / n; the identity when n == 0.
  [[nodiscard]] prim::Quat<T> tilt_quat(T s_r, T s_p) const noexcept {
    using std::cos;
    using std::sin;
    using std::sqrt;
    const T n = sqrt(s_r * s_r + s_p * s_p);
    if (!(n > T(0))) {
      return prim::Quat<T>::identity();
    }
    const T half = T(0.5) * cfg_.tilt_max * (n > T(1) ? T(1) : n);
    const T sh = sin(half);
    return prim::Quat<T>(cos(half), sh * (s_r / n), sh * (s_p / n), T(0));
  }

  // q_z(psi): the rotation by psi about world down.
  [[nodiscard]] static prim::Quat<T> heading_quat(T psi) noexcept {
    return prim::Quat<T>::from_axis_angle(prim::Vec3<T>(T(0), T(0), T(1)), psi);
  }

  // psi_m = 2 atan2(q.z, q.w), with (w, z) negated when w < 0 or (w == 0 and z < 0), so psi_m is in (-pi, pi]; 0 on the
  // singular set q.w = q.z = 0. The world-first split q = q_z(psi) (x) q_xy.
  [[nodiscard]] static T heading(const prim::Quat<T>& q) noexcept {
    using std::atan2;
    if (q.w == T(0) && q.z == T(0)) {
      return T(0);
    }
    const bool flip = q.w < T(0) || (q.w == T(0) && q.z < T(0));
    return T(2) * (flip ? atan2(-q.z, -q.w) : atan2(q.z, q.w));
  }

  void drop() noexcept {
    phase_ = AnglePhase::Uninit;
    sigma_ = T(0);
    omega_r_ = T(0);
    t_r_us_ = 0;
    psi_lock_ = T(0);
  }

  AttitudeConfig<T> cfg_{};
  AnglePhase phase_ = AnglePhase::Uninit;
  T sigma_{};
  T omega_r_{};
  TimeUs t_r_us_ = 0;
  T psi_lock_{};
};

extern template class AngleMode<float>;

}  // namespace marv::attitude
