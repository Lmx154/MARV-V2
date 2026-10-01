#pragma once

// The gyro chain (quad spec 4 L6 stage (b), QF-7; decision 0013). Per IMU tick and per gyro axis: 12 notches (4 motors
// x the harmonics 1x, 2x, 3x of each rotor's frequency, in the fixed order motor 1 harmonics 1..3, motor 2 ...), then a
// second-order Butterworth low-pass. Every stage is a biquad in direct form I; the notch coefficients are shared by the
// three axes.
//
// Direct form I, evaluated left to right (the order is part of the contract, so a float result is reproducible):
//   y_n = b0 x_n + b1 x_(n-1) + b2 x_(n-2) - a1 y_(n-1) - a2 y_(n-2)          (a0 = 1)
//
// Low-pass: the analogue Butterworth H(s) = 1 / (s^2 + sqrt(2) s + 1), s normalised to the cutoff, through the bilinear
// transform s -> (1/K)(1 - z^-1)/(1 + z^-1) with the prewarped K = tan(pi f_c / f_s) (A. V. Oppenheim and R. W.
// Schafer, Discrete-Time Signal Processing, 3rd ed., chapter 7: the bilinear transformation and its frequency
// warping). With norm = 1 / (1 + sqrt(2) K + K^2):
//   b0 = K^2 norm, b1 = 2 b0, b2 = b0, a1 = 2 (K^2 - 1) norm, a2 = (1 - sqrt(2) K + K^2) norm.
//
// Notch at f0 with quality Q: the analogue H(s) = (s^2 + 1) / (s^2 + s/Q + 1), s normalised to f0, through the same
// transform prewarped at f0. In the form of the Audio EQ Cookbook (R. Bristow-Johnson, "Cookbook formulae for audio
// equalizer biquad filter coefficients", the "notch" filter; its bandwidth is prewarped by the bilinear transform at
// w0), with w0 = 2 pi f0 / f_s and alpha = sin(w0) / (2 Q):
//   b0 = 1, b1 = -2 cos(w0), b2 = 1, a0 = 1 + alpha, a1 = -2 cos(w0), a2 = 1 - alpha, all divided by a0.
// This equals the K form (K = tan(w0 / 2): norm = 1 / (1 + K/Q + K^2), b0 = (1 + K^2) norm, b1 = 2 (K^2 - 1) norm,
// b2 = b0, a1 = b1, a2 = (1 - K/Q + K^2) norm) in exact arithmetic; the tests use the K form as the independent
// formula. The zero sits exactly at f0.
//
// Notch tracking: the caller (the rate-loop group) calls update_notches(rotor speeds) when the rate loop executes, so
// the coefficients are recomputed at the rate-loop rate and held, unchanged, for the ticks until the next call (the
// divisor of the configuration is the caller's schedule). For motor i and harmonic h: f0 = h omega_i / (2 pi), evaluated
// in T as (h * omega_i) / (2 * pi), pi cast to T from the constants header (a notch test reproduces it). A notch
// is ACTIVE only if motor i's valid bit is set, omega_i is finite, omega_i >= omega_th and f0 < f_s / 2 (strict).
// Otherwise it is BYPASSED at that update: its coefficients are the identity (b0 = 1, the rest 0), so its output is
// exactly its input and its direct-form-I history keeps running with input = output. No stale coefficient is held: a
// call decides every notch from the sample it is given alone. A motor with at least one bypassed notch sets its bit in
// bypass_flags() (bit i = motor i + 1, rewritten at every update) and increments its saturating counter.
//
// Non-finite gyro input (decision 0014, the 0013 known limit): an axis whose input value is NaN or infinite outputs
// the last output of that axis (0 before any), its filter states are NOT updated, its bit is set in input_fault_flags()
// (bit a = axis a: roll, pitch, yaw) and its saturating counter incremented; the other axes are untouched. At the next
// finite sample of that axis its states are reset to the steady state of that sample and the bit is cleared. With
// seed_first_sample set the same reset also seeds the first finite sample after init, so a constant input gives no
// start-up transient (the D term of the rate loop would see it as a kick); unset (the default), that first sample
// starts from the zero states of init, as the chain did before decision 0014. Steady state of a constant input x: every stage is a unity-DC-gain section, so its
// output is x and the direct-form-I history is x_(n-1) = x_(n-2) = y_(n-1) = y_(n-2) = x for the notch and the low-pass
// alike. DC gain (b0 + b1 + b2) / (1 + a1 + a2): the notch has b0 + b1 + b2 = (2 - 2 cos w0) / a0 = 1 + a1 + a2 (and the
// identity stage has gain 1), the Butterworth low-pass has b0 + b1 + b2 = 4 K^2 norm = 1 + a1 + a2. The float
// coefficients meet this to their rounding, so the seeded chain starts within that error of the input, not exactly on it.

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "marv/prim/constants.hpp"
#include "marv/prim/vec.hpp"
#include "marv/types/actuator.hpp"
#include "marv/types/rotor_speed_sample.hpp"

namespace marv::gyro_chain {

// The harmonics of each rotor's frequency that the chain notches: 1x, 2x, 3x. Rule: quad spec QF-7 "at the harmonics
// the B1 vibration model includes (1x, 2x, 3x)"; decision 0009 owner decision 4. The multiplier of harmonic index h
// is h + 1.
enum class Harmonic : std::uint8_t { First, Second, Third, Count };

inline constexpr std::size_t kHarmonics = static_cast<std::size_t>(Harmonic::Count);
// Quad-X: four motors (core section 3), logical number i + 1 at index i.
inline constexpr std::size_t kMotors = kQuadXMotors;
inline constexpr std::size_t kNotches = kMotors * kHarmonics;
inline constexpr std::size_t kAxes = prim::kSpatialDim;

// Notch n of an axis is motor n / kHarmonics, harmonic n % kHarmonics.
[[nodiscard]] constexpr std::size_t notch_index(std::size_t motor, std::size_t harmonic) noexcept {
  return motor * kHarmonics + harmonic;
}

template <class T>
struct BiquadCoeffs {
  T b0{1};
  T b1{0};
  T b2{0};
  T a1{0};
  T a2{0};
};

// The identity stage: y_n = x_n exactly for every finite x.
template <class T>
[[nodiscard]] constexpr BiquadCoeffs<T> identity_coeffs() noexcept {
  return BiquadCoeffs<T>{};
}

// Second-order Butterworth low-pass at f_c (Hz) for a sample period of `period` seconds (formulas above).
// pre: 0 < f_c and 2 f_c period < 1.
template <class T>
[[nodiscard]] BiquadCoeffs<T> lowpass_coeffs(T f_c, T period) noexcept {
  using std::sqrt;
  using std::tan;
  const T pi = static_cast<T>(prim::kPi);
  const T k = tan(pi * f_c * period);
  const T k2 = k * k;
  const T rk = sqrt(T(2)) * k;
  const T norm = T(1) / (T(1) + rk + k2);
  BiquadCoeffs<T> c;
  c.b0 = k2 * norm;
  c.b1 = T(2) * c.b0;
  c.b2 = c.b0;
  c.a1 = T(2) * (k2 - T(1)) * norm;
  c.a2 = (T(1) - rk + k2) * norm;
  return c;
}

// Notch at f0 (Hz) with quality q for a sample period of `period` seconds (formulas above).
// pre: 0 < f0, 2 f0 period < 1, q > 0.
template <class T>
[[nodiscard]] BiquadCoeffs<T> notch_coeffs(T f0, T q, T period) noexcept {
  using std::cos;
  using std::sin;
  const T two_pi = T(2) * static_cast<T>(prim::kPi);
  const T w0 = two_pi * f0 * period;
  const T alpha = sin(w0) / (T(2) * q);
  const T cw = cos(w0);
  const T norm = T(1) / (T(1) + alpha);
  BiquadCoeffs<T> c;
  c.b0 = norm;
  c.b1 = -(T(2) * cw) * norm;
  c.b2 = norm;
  c.a1 = c.b1;
  c.a2 = (T(1) - alpha) * norm;
  return c;
}

template <class T>
struct GyroChainConfig {
  T period{};                                // nominal tick period, s (the primary-IMU sample period)
  std::uint32_t rate_divisor = 0;            // rate-loop executions every rate_divisor ticks; the caller's schedule
  T cutoff_hz{};                             // low-pass cutoff f_c, Hz
  std::array<T, kHarmonics> notch_q{};       // notch quality per harmonic index (1x, 2x, 3x)
  T omega_threshold_rad_s{};                 // omega_th: a notch is active only at omega >= omega_th, rad/s
  bool seed_first_sample = false;            // seed the states at the first finite sample after init (file comment)
};

enum class ConfigError : std::uint8_t { None, NonFinite, Period, Divisor, Cutoff, NotchQ, Threshold };

// The first violated rule, else None. Written with negated comparisons, so a NaN is rejected. Rules: period > 0;
// divisor >= 2 (the low-pass rule puts f_r / 2 above the cutoff, so a divisor of 1 has no rate-loop Nyquist to
// attenuate at); 0 < cutoff < f_s / 2; every Q > 0; omega_th > 0.
template <class T>
[[nodiscard]] ConfigError validate(const GyroChainConfig<T>& c) noexcept {
  using std::isfinite;
  bool finite = isfinite(c.period) && isfinite(c.cutoff_hz) && isfinite(c.omega_threshold_rad_s);
  for (std::size_t h = 0; h < kHarmonics; ++h) {
    finite = finite && isfinite(c.notch_q[h]);
  }
  if (!finite) {
    return ConfigError::NonFinite;
  }
  if (!(c.period > T(0))) {
    return ConfigError::Period;
  }
  if (c.rate_divisor < 2) {
    return ConfigError::Divisor;
  }
  if (!(c.cutoff_hz > T(0)) || !(T(2) * c.cutoff_hz * c.period < T(1))) {
    return ConfigError::Cutoff;
  }
  for (std::size_t h = 0; h < kHarmonics; ++h) {
    if (!(c.notch_q[h] > T(0))) {
      return ConfigError::NotchQ;
    }
  }
  if (!(c.omega_threshold_rad_s > T(0))) {
    return ConfigError::Threshold;
  }
  return ConfigError::None;
}

template <class T>
class GyroChain {
 public:
  // pre: validate(cfg) == None. Clears every history, the bypass flags and counters; the low-pass is set, every notch
  // is the identity until the first update_notches.
  void init(const GyroChainConfig<T>& cfg) noexcept {
    cfg_ = cfg;
    lowpass_ = lowpass_coeffs<T>(cfg.cutoff_hz, cfg.period);
    two_pi_ = T(2) * static_cast<T>(prim::kPi);
    half_fs_ = T(1) / (T(2) * cfg.period);
    for (BiquadCoeffs<T>& c : notch_) {
      c = identity_coeffs<T>();
    }
    state_ = {};
    bypass_flags_ = 0;
    bypass_count_ = {};
    seeded_ = {};
    last_out_ = prim::Vec3<T>();
    fault_flags_ = 0;
    fault_count_ = {};
  }

  // Recomputes the 12 notch coefficients from the latest rotor speeds (rule in the file comment). Call it when the
  // rate loop executes. pre: init.
  void update_notches(const RotorSpeedSample& rotors) noexcept {
    using std::isfinite;
    bypass_flags_ = 0;
    for (std::size_t m = 0; m < kMotors; ++m) {
      const T omega = static_cast<T>(rotors.omega_rad_s[m]);
      const bool usable = (rotors.flags & rotor_speed_valid_bit(m)) != 0 && isfinite(omega) &&
                          omega >= cfg_.omega_threshold_rad_s;
      bool any_bypassed = false;
      for (std::size_t h = 0; h < kHarmonics; ++h) {
        const T f0 = static_cast<T>(h + 1) * omega / two_pi_;
        if (usable && f0 < half_fs_) {
          notch_[notch_index(m, h)] = notch_coeffs<T>(f0, cfg_.notch_q[h], cfg_.period);
        } else {
          notch_[notch_index(m, h)] = identity_coeffs<T>();
          any_bypassed = true;
        }
      }
      if (any_bypassed) {
        bypass_flags_ |= rotor_speed_valid_bit(m);
        if (bypass_count_[m] != std::numeric_limits<std::uint32_t>::max()) {
          ++bypass_count_[m];
        }
      }
    }
  }

  // One tick: gyro rates (FRD, rad/s) through the notches and the low-pass. A non-finite axis value holds that axis
  // (rules in the file comment).
  [[nodiscard]] prim::Vec3<T> filter(const prim::Vec3<T>& gyro_rad_s) noexcept {
    using std::isfinite;
    prim::Vec3<T> out;
    for (std::size_t a = 0; a < kAxes; ++a) {
      const std::uint32_t bit = std::uint32_t{1} << a;
      T v = gyro_rad_s[a];
      if (!isfinite(v)) {
        out[a] = last_out_[a];
        fault_flags_ |= bit;
        if (fault_count_[a] != std::numeric_limits<std::uint32_t>::max()) {
          ++fault_count_[a];
        }
        continue;
      }
      if ((cfg_.seed_first_sample && !seeded_[a]) || (fault_flags_ & bit) != 0) {
        seed(state_[a], v);
        fault_flags_ &= ~bit;
      }
      seeded_[a] = true;
      for (std::size_t n = 0; n < kNotches; ++n) {
        v = step(notch_[n], state_[a][n], v);
      }
      out[a] = step(lowpass_, state_[a][kNotches], v);
      last_out_[a] = out[a];
    }
    return out;
  }

  // Bit i (rotor_speed_valid_bit(i)) is set iff motor i + 1 had a bypassed notch at the latest update_notches.
  [[nodiscard]] std::uint32_t bypass_flags() const noexcept { return bypass_flags_; }
  // Number of updates with a bypassed notch of motor index i, saturating at the maximum of uint32_t.
  [[nodiscard]] std::uint32_t bypass_count(std::size_t motor_index) const noexcept { return bypass_count_[motor_index]; }
  // Bit a is set from the tick axis a (0 roll, 1 pitch, 2 yaw) had a non-finite input until its next finite sample.
  [[nodiscard]] std::uint32_t input_fault_flags() const noexcept { return fault_flags_; }
  // Number of non-finite samples of axis `axis`, saturating at the maximum of uint32_t.
  [[nodiscard]] std::uint32_t input_fault_count(std::size_t axis) const noexcept { return fault_count_[axis]; }

 private:
  struct State {
    T x1{0};
    T x2{0};
    T y1{0};
    T y2{0};
  };

  // The steady state of a constant input x for every stage of an axis (rule in the file comment).
  static void seed(std::array<State, kNotches + 1>& axis, T x) noexcept {
    for (State& s : axis) {
      s = State{x, x, x, x};
    }
  }

  [[nodiscard]] static T step(const BiquadCoeffs<T>& c, State& s, T x) noexcept {
    const T y = c.b0 * x + c.b1 * s.x1 + c.b2 * s.x2 - c.a1 * s.y1 - c.a2 * s.y2;
    s.x2 = s.x1;
    s.x1 = x;
    s.y2 = s.y1;
    s.y1 = y;
    return y;
  }

  GyroChainConfig<T> cfg_{};
  BiquadCoeffs<T> lowpass_{};
  std::array<BiquadCoeffs<T>, kNotches> notch_{};
  std::array<std::array<State, kNotches + 1>, kAxes> state_{};  // per axis: the notches, then the low-pass
  T two_pi_{};
  T half_fs_{};
  std::uint32_t bypass_flags_ = 0;
  std::array<std::uint32_t, kMotors> bypass_count_{};
  std::array<bool, kAxes> seeded_{};
  prim::Vec3<T> last_out_{};
  std::uint32_t fault_flags_ = 0;
  std::array<std::uint32_t, kAxes> fault_count_{};
};

extern template class GyroChain<float>;
extern template ConfigError validate<float>(const GyroChainConfig<float>&) noexcept;
extern template BiquadCoeffs<float> lowpass_coeffs<float>(float, float) noexcept;
extern template BiquadCoeffs<float> notch_coeffs<float>(float, float, float) noexcept;

}  // namespace marv::gyro_chain
