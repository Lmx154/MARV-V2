#pragma once

// The L5 attitude law (decision 0006 C): a tilt-prioritised quaternion P law with PX4's yaw-weight recomposition and
// yaw-gain compensation (PX4 AttitudeControl.cpp at commit 3b828e157a6d9b7d811c80c73d95fd39fca5e181, BSD-3), in a closed
// form of the same rotations. Opening: in the measured AttitudeState, the setpoint quaternion and the yaw-rate command
// (world down, rad/s); out a body-rate setpoint (FRD) for RateLoop::execute_bypass.
//
// Per execution, with q^ and q_sp^ the normalised inputs, k the gain and w the yaw weight:
//   q_e = canonical(conj(q^) (x) q_sp^)                        shortest rotation, in current body axes
//   q_e = q_xy (x) q_z,  rho = sqrt(w_e^2 + z_e^2)             tilt_yaw.hpp; psi = yaw angle of q_z
//   q_e,w = (rho c, c a + s b, c b - s a, rho s)               c = cos(w psi / 2), s = sin(w psi / 2)
//   eq = 2 imag(canonical(q_e,w))
//   r = (k eq_x, k eq_y, (k / w) eq_z) + rotate(conj(q^), (0, 0, yaw_rate_cmd))
// then the roll-pitch pair is scaled by one factor so both bounds hold, and yaw is clamped.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "marv/attitude/config.hpp"
#include "marv/attitude/tilt_yaw.hpp"
#include "marv/prim/constants.hpp"
#include "marv/prim/quat.hpp"
#include "marv/prim/vec.hpp"
#include "marv/types/attitude_state.hpp"
#include "marv/types/time.hpp"

namespace marv::attitude {

// The largest accepted |dt_n - T_a| in microseconds (the L4 tolerance: the hal_sim truncation bound of a stamp).
inline constexpr std::uint64_t kPeriodToleranceUs = 1;

template <class T>
struct AttitudeOutput {
  prim::Vec3<T> rate_setpoint{};  // rad/s, FRD
  bool fault_active = false;
  bool fault_latched = false;
  std::uint32_t fault_count = 0;
};

namespace detail {
// hal_panic: the execution spacing differs from the design period by more than kPeriodToleranceUs.
[[noreturn]] void panic_period() noexcept;
}  // namespace detail

template <class T>
class AttitudeLaw {
 public:
  // pre: validate(cfg) == None. Clears the period history, the fault latch and the fault counter.
  void init(AttitudeConfig<T> cfg) noexcept {
    using std::lround;
    cfg_ = cfg;
    k_yaw_ = cfg.kp / cfg.yaw_weight;
    // Twice the period in us (half-us resolution), so a period such as 312.5 us is checked exactly.
    two_period_us_ =
        static_cast<std::uint64_t>(lround(2 * cfg.period * static_cast<T>(prim::kMicrosecondsPerSecond)));
    have_t_ = false;
    latched_ = false;
    count_ = 0;
  }

  // The next execution is a first execution: no spacing check. Fault latch and counter are kept.
  void reset() noexcept { have_t_ = false; }

  // One attitude execution. Panics when the spacing to the previous execution differs from the period by more than
  // kPeriodToleranceUs. An invalid input (a.valid false, a non-finite or zero-norm quaternion, a non-finite yaw-rate
  // command) or a non-finite output returns a zero rate setpoint and sets fault_active for this execution.
  [[nodiscard]] AttitudeOutput<T> execute(const AttitudeState<T>& a, const prim::Quat<T>& q_sp,
                                          T yaw_rate_cmd) noexcept {
    using std::abs;
    using std::cos;
    using std::isfinite;
    using std::sin;
    if (have_t_) {
      const std::uint64_t dt_us = a.t_us - t_us_;
      const std::uint64_t two_tol = 2 * kPeriodToleranceUs;
      // A spacing too large to double (a stamp going backwards wraps to one) is outside the bound anyway.
      const bool no_double = dt_us > std::numeric_limits<std::uint64_t>::max() / 2;
      const std::uint64_t two_dt = no_double ? 0 : 2 * dt_us;
      if (no_double || two_dt > two_period_us_ + two_tol || two_dt + two_tol < two_period_us_) {
        detail::panic_period();
      }
    }
    t_us_ = a.t_us;
    have_t_ = true;

    bool ok = a.valid && is_finite(a.q) && is_finite(q_sp) && isfinite(yaw_rate_cmd);
    if (!ok) {
      return fault();
    }
    const T norm_q = a.q.norm();
    const T norm_sp = q_sp.norm();
    ok = isfinite(norm_q) && norm_q > T(0) && isfinite(norm_sp) && norm_sp > T(0);
    if (!ok) {
      return fault();
    }
    const prim::Quat<T> q_hat = a.q.normalized();
    const prim::Quat<T> q_sp_hat = q_sp.normalized();

    const prim::Quat<T> q_e = (q_hat.conjugate() * q_sp_hat).canonical();
    const TiltYawSplit<T> split = split_tilt_yaw(q_e);
    prim::Quat<T> q_ew = q_e;
    if (!split.singular) {
      const T half = cfg_.yaw_weight * split.psi * T(0.5);
      const T c = cos(half);
      const T s = sin(half);
      q_ew = prim::Quat<T>(split.rho * c, c * split.a + s * split.b, c * split.b - s * split.a, split.rho * s);
    }
    const prim::Quat<T> q_c = q_ew.canonical();
    const prim::Vec3<T> ff = q_hat.conjugate().rotate(prim::Vec3<T>(T(0), T(0), yaw_rate_cmd));
    prim::Vec3<T> r(cfg_.kp * (T(2) * q_c.x) + ff[kRollAxis], cfg_.kp * (T(2) * q_c.y) + ff[kPitchAxis],
                    k_yaw_ * (T(2) * q_c.z) + ff[kYawAxis]);
    for (std::size_t i = 0; i < kRateAxes; ++i) {
      if (!isfinite(r[i])) {
        return fault();
      }
    }
    return AttitudeOutput<T>{clamp(r), false, latched_, count_};
  }

  [[nodiscard]] bool fault_latched() const noexcept { return latched_; }
  [[nodiscard]] std::uint32_t fault_count() const noexcept { return count_; }

 private:
  // The roll-pitch pair is scaled by the one factor s <= 1 that brings both components inside their bounds (no
  // division unless a bound is exceeded), which keeps the tilt direction; a final min keeps the bound exactly against
  // the rounding of the scaled product. Yaw is clamped to its bound.
  [[nodiscard]] prim::Vec3<T> clamp(const prim::Vec3<T>& r) const noexcept {
    using std::abs;
    using std::copysign;
    const T ax = abs(r[kRollAxis]);
    const T ay = abs(r[kPitchAxis]);
    const T bx = cfg_.rate_max[kRollAxis];
    const T by = cfg_.rate_max[kPitchAxis];
    const T sx = ax > bx ? bx / ax : T(1);
    const T sy = ay > by ? by / ay : T(1);
    const T s = sx < sy ? sx : sy;
    const T cx = abs(r[kRollAxis] * s);
    const T cy = abs(r[kPitchAxis] * s);
    const T bz = cfg_.rate_max[kYawAxis];
    const T az = abs(r[kYawAxis]);
    return prim::Vec3<T>(copysign(cx > bx ? bx : cx, r[kRollAxis]), copysign(cy > by ? by : cy, r[kPitchAxis]),
                         copysign(az > bz ? bz : az, r[kYawAxis]));
  }

  [[nodiscard]] AttitudeOutput<T> fault() noexcept {
    latched_ = true;
    if (count_ != std::numeric_limits<std::uint32_t>::max()) {
      ++count_;
    }
    return AttitudeOutput<T>{prim::Vec3<T>(), true, true, count_};
  }

  AttitudeConfig<T> cfg_{};
  T k_yaw_{};  // k / w, computed once at init (PX4 setProportionalGain)
  std::uint64_t two_period_us_ = 0;
  TimeUs t_us_ = 0;
  bool have_t_ = false;
  bool latched_ = false;
  std::uint32_t count_ = 0;
};

extern template class AttitudeLaw<float>;

}  // namespace marv::attitude
