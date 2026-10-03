#pragma once

// The L4 rate loop (quad spec 4 L4): a per-axis parallel PID with a first-order reference-model prefilter (QF-2),
// D on the measurement, forward-Euler integration deferred one execution and clamping anti-windup driven by the L3
// allocation. Opening: in a gyro sample (rad/s, FRD) and a body-rate setpoint (rad/s); out a torque request
// (N*m, FRD). Configuration is a plain struct with validate; from_params and load_config read the product parameters.
//
// Per axis a, execution n at sample stamp t_n, dt_n = t_n - t_(n-1) converted once to seconds:
//   r_n = r_(n-1) + (1 - exp(-dt_n / tau_ref)) (sp_n - r_(n-1))     seeded r := y at the first execution after
//   e_n = r_n - y_n                                                 init or reset
//   I_n = I_(n-1) + (freeze_(n-1) ? 0 : ki e_(n-1) dt_n)
//   D_n = -kd (y_n - y_(n-1)) / dt_n                                D = 0 at that first execution
//   u_n = kp e_n + I_n + D_n
// D low-pass (quad spec D2, decision 0014): with T_f > 0 the derivative above is the input of a first-order low-pass,
//   Df_n = Df_(n-1) + (1 - exp(-dt_n / T_f)) (D_n - Df_(n-1)),   Df := 0 at the first execution, u_n uses Df_n,
// the discretisation of the prefilter. T_f = 0 runs the expression above unchanged (not an alpha computed to 1).
// Feed-forward (quad spec F1-F3, decision 0014), when some inertia J_a > 0: with w = y_n and g(w) = w x (J w),
//   u_n += g(w_n) + tau_m gdot_n,  gdot_n = gdot_(n-1) + (1 - exp(-dt_n / T_ff)) ((g_n - g_(n-1)) / dt_n - gdot_(n-1)),
// gdot := 0 at the first execution; T_ff = 0 takes gdot_n = (g_n - g_(n-1)) / dt_n. g is the gyroscopic torque of
// Euler's equation for a rigid body in principal axes, J w' + w x (J w) = tau (H. Goldstein, C. Poole and J. Safko,
// Classical Mechanics, 3rd ed., section 5.5, with the body axes the principal axes): the torque that holds w
// constant. The motor torque follows the request through a first-order lag, tau_m u' + u = u_req, so u_req = u + tau_m u'
// delivers u (feed-forward by inverting the first-order actuator lag; the identity is derived here, in this line);
// here u = g and u' = gdot. All J_a = 0 skips the term: u is exactly the PID law.
// freeze_n is recorded by record_allocation from execution n's allocation and gates the increment applied at
// execution n + 1, so the update has no lag.

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "marv/mixer/mixer.hpp"
#include "marv/prim/constants.hpp"
#include "marv/prim/mat.hpp"
#include "marv/prim/vec.hpp"
#include "marv/types/imu_sample.hpp"
#include "marv/types/time.hpp"

namespace marv::rate {

using mixer::kAxes;
using mixer::kMotors;

// Torque axes: indices of the Vec3 torque and of the per-axis gains.
enum TorqueAxis : std::size_t { kRollAxis, kPitchAxis, kYawAxis };

inline constexpr std::size_t kTorqueAxes = prim::kSpatialDim;

// The largest accepted |dt_n - T| in microseconds: the hal_sim truncation bound of a sample stamp.
inline constexpr std::uint64_t kPeriodToleranceUs = 1;

template <class T>
struct RateConfig {
  prim::Vec3<T> kp{};       // N*m*s/rad, [roll, pitch, yaw]
  prim::Vec3<T> ki{};       // N*m/rad
  prim::Vec3<T> kd{};       // N*m*s^2/rad
  prim::Vec3<T> tau_ref{};  // reference-model time constant, s
  T period{};               // design period T of a rate-loop execution, s
  prim::Vec3<T> d_filter_tau{};  // D low-pass time constant T_f, s, [roll, pitch, yaw]; 0 = unfiltered D
  prim::Vec3<T> inertia{};       // J, kg*m^2, principal moments [roll, pitch, yaw]; all 0 = no feed-forward
  T motor_tau{};                 // motor time constant tau_m, s
  T ff_filter_tau{};             // T_ff, the filter of the feed-forward derivative, s; 0 = unfiltered
  // Effectiveness B of decision 0004 item 2: column i = [1, -y_i, x_i, s_i c_q], index = logical motor - 1.
  std::array<T, kMotors> rotor_x{};   // m, FRD
  std::array<T, kMotors> rotor_y{};   // m, FRD
  std::array<T, kMotors> yaw_sign{};  // s_i
  T torque_ratio{};                   // c_q, N*m per N
};

enum class ConfigError : std::uint8_t { None, NonFinite, NegativeGain, TauRef, Period, DFilter, Feedforward };

// The first violated rule, else None. Written with negated comparisons, so a NaN is rejected.
template <class T>
[[nodiscard]] ConfigError validate(const RateConfig<T>& c) noexcept {
  using std::isfinite;
  bool finite = isfinite(c.period) && isfinite(c.torque_ratio) && isfinite(c.motor_tau) && isfinite(c.ff_filter_tau);
  for (std::size_t a = 0; a < kTorqueAxes; ++a) {
    finite = finite && isfinite(c.kp[a]) && isfinite(c.ki[a]) && isfinite(c.kd[a]) && isfinite(c.tau_ref[a]) &&
             isfinite(c.d_filter_tau[a]) && isfinite(c.inertia[a]);
  }
  for (std::size_t i = 0; i < kMotors; ++i) {
    finite = finite && isfinite(c.rotor_x[i]) && isfinite(c.rotor_y[i]) && isfinite(c.yaw_sign[i]);
  }
  if (!finite) {
    return ConfigError::NonFinite;
  }
  for (std::size_t a = 0; a < kTorqueAxes; ++a) {
    if (!(c.kp[a] >= T(0)) || !(c.ki[a] >= T(0)) || !(c.kd[a] >= T(0))) {
      return ConfigError::NegativeGain;
    }
  }
  for (std::size_t a = 0; a < kTorqueAxes; ++a) {
    if (!(c.tau_ref[a] > T(0))) {
      return ConfigError::TauRef;
    }
  }
  if (!(c.period > T(0))) {
    return ConfigError::Period;
  }
  for (std::size_t a = 0; a < kTorqueAxes; ++a) {
    if (!(c.d_filter_tau[a] >= T(0))) {
      return ConfigError::DFilter;
    }
  }
  bool ff_ok = c.motor_tau >= T(0) && c.ff_filter_tau >= T(0);
  for (std::size_t a = 0; a < kTorqueAxes; ++a) {
    ff_ok = ff_ok && c.inertia[a] >= T(0);
  }
  if (!ff_ok) {
    return ConfigError::Feedforward;
  }
  return ConfigError::None;
}

// B-hat, rows [thrust, roll, pitch, yaw], column = motor index.
template <class T>
[[nodiscard]] prim::Mat<T, kAxes, kMotors> effectiveness(const RateConfig<T>& c) noexcept {
  prim::Mat<T, kAxes, kMotors> b;
  for (std::size_t i = 0; i < kMotors; ++i) {
    b(mixer::kThrust, i) = T(1);
    b(mixer::kRoll, i) = -c.rotor_y[i];
    b(mixer::kPitch, i) = c.rotor_x[i];
    b(mixer::kYaw, i) = c.yaw_sign[i] * c.torque_ratio;
  }
  return b;
}

template <class T>
struct RateOutput {
  prim::Vec3<T> torque{};  // N*m, FRD
  bool fault_active = false;
  bool fault_latched = false;
  std::uint32_t fault_count = 0;
};

namespace detail {
// hal_panic: the execution spacing differs from the design period by more than kPeriodToleranceUs.
[[noreturn]] void panic_period() noexcept;
}  // namespace detail

template <class T>
class RateLoop {
 public:
  // pre: validate(cfg) == None and mixer::validate(mixer_cfg) == None. Clears all state, the fault latch and the
  // fault counter.
  void init(const RateConfig<T>& cfg, const mixer::MixerConfig<T>& mixer_cfg) noexcept {
    using std::abs;
    using std::lround;
    cfg_ = cfg;
    const prim::Mat<T, kAxes, kMotors> b = effectiveness(cfg);
    for (std::size_t r = 0; r < kAxes; ++r) {
      for (std::size_t c = 0; c < kAxes; ++c) {
        T sum = T(0);
        for (std::size_t k = 0; k < kMotors; ++k) {
          sum += abs(b(r, k)) * abs(mixer_cfg.m(k, c));
        }
        abs_bm_(r, c) = sum;
      }
    }
    // Twice the period in µs (half-µs resolution), so a period such as 312.5 µs (two 156.25 µs ticks) is checked
    // exactly: |dt − T| > tol  ⇔  |2·dt − 2·T| > 2·tol, with 2·dt and 2·T integers.
    two_period_us_ =
        static_cast<std::uint64_t>(lround(2 * cfg.period * static_cast<T>(prim::kMicrosecondsPerSecond)));
    ff_on_ = cfg.inertia[kRollAxis] > T(0) || cfg.inertia[kPitchAxis] > T(0) || cfg.inertia[kYawAxis] > T(0);
    reset_state();
    have_t_ = false;
    latched_ = false;
    count_ = 0;
  }

  // The next execution is a first execution: it seeds the prefilter from the gyro and applies no dt check.
  // Fault latch and counter are kept.
  void reset() noexcept {
    reset_state();
    have_t_ = false;
  }

  // One rate-loop execution. Panics when the spacing to the previous execution differs from the design period by
  // more than kPeriodToleranceUs. A failed check (gyro not valid or not finite, setpoint or torque not finite)
  // resets the loop state, returns zero torque and sets fault_active for this execution.
  [[nodiscard]] RateOutput<T> execute(const ImuSample& imu, const prim::Vec3<T>& setpoint) noexcept {
    return step<true>(imu, setpoint);
  }

  // The same execution with the reference-model prefilter bypassed: r_n = reference, so the PID law runs on the
  // given reference directly (angle mode feeds it the attitude loop's rate request). Seeding, the dt check, the
  // fault handling and the allocation record are those of execute. After a bypassed execution r_ holds the applied
  // reference, so a following execute continues its prefilter from it.
  [[nodiscard]] RateOutput<T> execute_bypass(const ImuSample& imu, const prim::Vec3<T>& reference) noexcept {
    return step<false>(imu, reference);
  }

  // Records the freeze that gates the next execution's integrator increment. `requested` is the torque passed to
  // the mixer, `al` the mixer's allocation of it. For each axis
  //   freeze = flag  and  |req - ach| > b  and  e_n (req - ach) > 0,
  //   b = (gamma_n + eps) (|B-hat| |M-hat| |v|)_axis, v = [achieved_thrust, achieved_torque], n = kMotors, eps = the
  //   scalar epsilon (the rounding bound of decision 0004 item 2), so a flag set with achieved == requested
  //   (rounding in the allocation) does not freeze.
  void record_allocation(const prim::Vec3<T>& requested, const mixer::Allocation<T>& al) noexcept {
    using std::abs;
    const std::array<T, kAxes> v{al.achieved_thrust, al.achieved_torque[0], al.achieved_torque[1],
                                 al.achieved_torque[2]};
    const std::array<bool, kTorqueAxes> flag{al.flags.roll, al.flags.pitch, al.flags.yaw};
    const T eps = std::numeric_limits<T>::epsilon();
    const T n_eps = static_cast<T>(kMotors) * eps;
    const T gamma = n_eps / (T(1) - n_eps);
    for (std::size_t a = 0; a < kTorqueAxes; ++a) {
      T sum = T(0);
      for (std::size_t k = 0; k < kAxes; ++k) {
        sum += abs_bm_(mixer::kRoll + a, k) * abs(v[k]);
      }
      const T bound = (gamma + eps) * sum;
      const T diff = requested[a] - al.achieved_torque[a];
      const bool same_sign = (e_[a] > T(0) && diff > T(0)) || (e_[a] < T(0) && diff < T(0));
      freeze_[a] = flag[a] && abs(diff) > bound && same_sign;
    }
  }

  [[nodiscard]] const prim::Vec3<T>& integrator() const noexcept { return integral_; }
  // True iff the next execution is a first execution (after init, reset or a fault): it seeds from its gyro.
  [[nodiscard]] bool seeding() const noexcept { return seed_; }
  [[nodiscard]] bool fault_latched() const noexcept { return latched_; }
  [[nodiscard]] std::uint32_t fault_count() const noexcept { return count_; }

 private:
  template <bool kPrefilter>
  [[nodiscard]] RateOutput<T> step(const ImuSample& imu, const prim::Vec3<T>& setpoint) noexcept {
    using std::exp;
    using std::isfinite;
    std::uint64_t dt_us = 0;
    if (have_t_) {
      dt_us = imu.t_us - t_us_;
      const std::uint64_t two_tol = 2 * kPeriodToleranceUs;
      // A spacing too large to double (a stamp going backwards wraps to one) is outside the bound anyway.
      const bool no_double = dt_us > std::numeric_limits<std::uint64_t>::max() / 2;
      const std::uint64_t two_dt = no_double ? 0 : 2 * dt_us;
      if (no_double || two_dt > two_period_us_ + two_tol || two_dt + two_tol < two_period_us_) {
        detail::panic_period();
      }
    }
    t_us_ = imu.t_us;
    have_t_ = true;

    prim::Vec3<T> y;
    bool ok = (imu.flags & imu_flag(ImuFlag::GyroValid)) != 0;
    for (std::size_t a = 0; a < kTorqueAxes; ++a) {
      y[a] = static_cast<T>(imu.gyro_rad_s[a]);
      ok = ok && isfinite(y[a]) && isfinite(setpoint[a]);
    }
    if (!ok) {
      return fault();
    }

    prim::Vec3<T> u;
    if (seed_) {
      r_ = y;
      e_ = prim::Vec3<T>();
      integral_ = prim::Vec3<T>();
      if (ff_on_) {
        g_prev_ = y.cross(inertia_times(y));
      }
      seed_ = false;
    } else {
      const T dt = static_cast<T>(dt_us) / static_cast<T>(prim::kMicrosecondsPerSecond);
      for (std::size_t a = 0; a < kTorqueAxes; ++a) {
        if constexpr (kPrefilter) {
          const T alpha = T(1) - exp(-dt / cfg_.tau_ref[a]);
          r_[a] = r_[a] + alpha * (setpoint[a] - r_[a]);
        } else {
          r_[a] = setpoint[a];
        }
        const T e = r_[a] - y[a];
        if (!freeze_[a]) {
          integral_[a] = integral_[a] + cfg_.ki[a] * e_[a] * dt;
        }
        T d = -cfg_.kd[a] * (y[a] - y_prev_[a]) / dt;
        if (cfg_.d_filter_tau[a] > T(0)) {
          const T alpha_d = T(1) - exp(-dt / cfg_.d_filter_tau[a]);
          d_f_[a] = d_f_[a] + alpha_d * (d - d_f_[a]);
          d = d_f_[a];
        }
        u[a] = cfg_.kp[a] * e + integral_[a] + d;
        e_[a] = e;
      }
      if (ff_on_) {
        const prim::Vec3<T> g = y.cross(inertia_times(y));
        for (std::size_t a = 0; a < kTorqueAxes; ++a) {
          const T raw = (g[a] - g_prev_[a]) / dt;
          if (cfg_.ff_filter_tau > T(0)) {
            const T alpha_ff = T(1) - exp(-dt / cfg_.ff_filter_tau);
            gdot_f_[a] = gdot_f_[a] + alpha_ff * (raw - gdot_f_[a]);
          } else {
            gdot_f_[a] = raw;
          }
          u[a] = u[a] + (g[a] + cfg_.motor_tau * gdot_f_[a]);
        }
        g_prev_ = g;
      }
      for (std::size_t a = 0; a < kTorqueAxes; ++a) {
        if (!isfinite(u[a])) {
          return fault();
        }
      }
    }
    y_prev_ = y;
    freeze_ = {};
    return RateOutput<T>{u, false, latched_, count_};
  }

  // J w, J diagonal.
  [[nodiscard]] prim::Vec3<T> inertia_times(const prim::Vec3<T>& w) const noexcept {
    return prim::Vec3<T>(cfg_.inertia[kRollAxis] * w[kRollAxis], cfg_.inertia[kPitchAxis] * w[kPitchAxis],
                         cfg_.inertia[kYawAxis] * w[kYawAxis]);
  }

  void reset_state() noexcept {
    d_f_ = prim::Vec3<T>();
    g_prev_ = prim::Vec3<T>();
    gdot_f_ = prim::Vec3<T>();
    r_ = prim::Vec3<T>();
    e_ = prim::Vec3<T>();
    y_prev_ = prim::Vec3<T>();
    integral_ = prim::Vec3<T>();
    freeze_ = {};
    seed_ = true;
  }

  [[nodiscard]] RateOutput<T> fault() noexcept {
    reset_state();
    latched_ = true;
    if (count_ != std::numeric_limits<std::uint32_t>::max()) {
      ++count_;
    }
    return RateOutput<T>{prim::Vec3<T>(), true, true, count_};
  }

  RateConfig<T> cfg_{};
  prim::Mat<T, kAxes, kAxes> abs_bm_{};  // |B-hat| |M-hat|
  std::uint64_t two_period_us_ = 0;
  prim::Vec3<T> r_{};
  prim::Vec3<T> e_{};
  prim::Vec3<T> y_prev_{};
  prim::Vec3<T> integral_{};
  prim::Vec3<T> d_f_{};      // the D low-pass state Df
  prim::Vec3<T> g_prev_{};   // w x (J w) at the previous execution
  prim::Vec3<T> gdot_f_{};   // the filtered derivative of w x (J w)
  bool ff_on_ = false;
  std::array<bool, kTorqueAxes> freeze_{};
  TimeUs t_us_ = 0;
  bool have_t_ = false;
  bool seed_ = true;
  bool latched_ = false;
  std::uint32_t count_ = 0;
};

// The product parameter set's rate configuration: rate_{kp,ki,kd,tau_ref}_<axis>, the period
// T = rate_loop_divisor * tick_period_num_us / tick_period_den microseconds in seconds, rotor_position_m<i>_{x,y},
// rotor_yaw_sign_m<i> and rotor_torque_ratio. With rate_ff_enable == 1 also inertia_{xx,yy,zz}, motor_tau and
// rate_ff_filter_tau (the feed-forward); any other value leaves them 0. Not validated. pre: params_init succeeded.
[[nodiscard]] RateConfig<float> from_params() noexcept;

// hal_panic naming the violated rule unless validate(c) == None.
void require_valid(const RateConfig<float>& c) noexcept;

// from_params, then require_valid: the firmware-facing init.
[[nodiscard]] RateConfig<float> load_config() noexcept;

extern template class RateLoop<float>;
extern template ConfigError validate<float>(const RateConfig<float>&) noexcept;

}  // namespace marv::rate
