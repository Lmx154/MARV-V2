// Shared helpers of the L6 stage (b) gyro-chain tests (decision 0013). Every number is a labelled test value with its
// reason, a cited constant, or derived on the line. Nothing here is a product value except the tick period, which is
// read from the product parameter set.
#pragma once

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include <marv/gyro_chain/gyro_chain.hpp>
#include <marv/params/param.hpp>
#include <marv/prim/constants.hpp>
#include <marv/types/rotor_speed_sample.hpp>

namespace marv::gyro_chain::test {

using Cd = std::complex<double>;
using Vec3f = prim::Vec3<float>;

constexpr double kPi = prim::kPi;
// Unit roundoff of binary32 with round to nearest: u = 2^-24. Half an ulp of x is at most u |x|.
constexpr double kU = 0x1p-24;

// ---- Labelled test values -------------------------------------------------------------------------------------
// Decision 0013 owner decision 3: gyro_chain_attenuation_min a_min = 0.1 (the register's value, restated as a test value).
constexpr double kAMin = 0.1;
// Rate-loop divisor 2 (design/scenario_values.yaml rate_loop_divisor, restated as a test value).
constexpr std::uint32_t kDivisor = 2;
// Relative frequency error epsilon of decision 0013: the telemetry resolution 2^-8 plus the odr_error of 65 ppm (the
// board default profile), with the ESC clock error taken as 0 (the architect's scratch case).
constexpr double kTelemetryStep = 0x1p-8;
constexpr double kEps = kTelemetryStep + 65.0e-6;
// Notch qualities per harmonic: the architect's scratch values of decision 0013 (ESC error 0). The test shows they
// meet a_min at f0 (1 +- kEps) for the test frequencies below.
constexpr std::array<float, kHarmonics> kScratchQ{12.2F, 11.1F, 9.3F};
// Activation floor omega_th: a test value, below every test rotor speed (smallest 2 pi 170 = 1068 rad/s) and well
// above the 0 of an invalid motor.
constexpr float kOmegaThreshold = 100.0F;
// Rotor frequencies f_1 of the four motors, Hz: a test value. Motor 1 is the one the depth checks use, at 400 Hz (2513
// rad/s, inside the range of a 5-inch rotor: the 175 Hz of decision 0013 is the hover and 2800 rad/s = 446 Hz the
// NeuroBEM maximum). The others are spread so that no two harmonics of the 12 lie within a few percent of each other
// (400 800 1200, 310 620 930, 230 460 690, 170 340 510): every harmonic below f_s / 2 = 3200 Hz.
constexpr std::array<double, kMotors> kRotorHz{400.0, 310.0, 230.0, 170.0};
// The 14 rad/s crossover band of the L4 design (decision 0005) and the tone nearest it on the DTFT grid.
constexpr double kCrossoverRadS = 14.0;
// DTFT window length N: 2^17 samples. Every test tone is a bin f = f_s k / N, so the window holds a whole number of
// periods (k) exactly.
constexpr std::size_t kWindow = std::size_t{1} << 17;
// Residual-transient allowance of the settling time (the tail of the impulse response after it).
constexpr double kTransientTol = 1.0e-6;
// Length of the double impulse responses used for the l1 norms and the tail: far beyond n_settle (checked in the test
// that the last samples are below 1e-30).
constexpr std::size_t kImpulseLen = 40000;
// Slack added to the signal-amplitude bounds in the rounding analysis: the accumulated rounding error (checked
// below 1e-3) can enlarge a signal by at most that.
constexpr double kAmplitudeSlack = 1.0e-3;
// Libm accuracy assumed for sinf, cosf, tanf and used in the coefficient bound: 2 ulp. INFERRED (glibc's documented
// float maxima are 1 to 2 ulp); coeff_test.cpp checks the assumption at every argument the tests use.
constexpr double kLibmUlps = 2.0;
// The coefficient bound is first order in u; the neglected terms are O(u^2) ~ 1e-14 relative.

// ---- Tick period from the product parameters --------------------------------------------------------------------
inline bool init_params() {
  static const bool ok = params_init(param_defaults());
  return ok;
}

// T = tick_period_num_us / tick_period_den us in s, in float as the generator builds it (rate::from_params: float
// quotient of microseconds, then / 10^6).
inline float tick_period_s() {
  return static_cast<float>(param_value<ParamId::tick_period_num_us>()) /
         static_cast<float>(param_value<ParamId::tick_period_den>()) /
         static_cast<float>(prim::kMicrosecondsPerSecond);
}

// ---- Fixture ----------------------------------------------------------------------------------------------------
struct Fixture {
  float period = 0;         // s
  double fs = 0;            // Hz, 1 / period of the float value
  float cutoff = 0;         // Hz
  GyroChainConfig<float> config;
};

// f_c from the rule of decision 0013: the digital gain at the rate loop's Nyquist f_r / 2 equals a_min, i.e.
// tan(pi f_c / f_s) = tan(pi f_r / (2 f_s)) / (1 / a_min^2 - 1)^(1/4), f_r = f_s / divisor.
inline double cutoff_by_rule(double fs, std::uint32_t divisor, double a_min) {
  const double fr = fs / static_cast<double>(divisor);
  const double k = std::tan(kPi * fr / (2.0 * fs)) / std::pow(1.0 / (a_min * a_min) - 1.0, 0.25);
  return fs / kPi * std::atan(k);
}

inline Fixture make_fixture() {
  EXPECT_TRUE(init_params());
  Fixture f;
  f.period = tick_period_s();
  f.fs = 1.0 / static_cast<double>(f.period);
  f.cutoff = static_cast<float>(cutoff_by_rule(f.fs, kDivisor, kAMin));
  f.config.period = f.period;
  f.config.rate_divisor = kDivisor;
  f.config.cutoff_hz = f.cutoff;
  f.config.notch_q = kScratchQ;
  f.config.omega_threshold_rad_s = kOmegaThreshold;
  return f;
}

// omega = 2 pi f for the rotor frequencies of kRotorHz, rad/s (float, as the telemetry delivers it).
inline std::array<float, kMotors> rotor_omegas() {
  std::array<float, kMotors> w{};
  for (std::size_t i = 0; i < kMotors; ++i) {
    w[i] = static_cast<float>(2.0 * kPi * kRotorHz[i]);
  }
  return w;
}

inline RotorSpeedSample make_sample(const std::array<float, kMotors>& omega,
                                    std::uint32_t flags = kRotorSpeedFlagsDefined) {
  RotorSpeedSample s{};
  s.omega_rad_s = omega;
  s.flags = flags;
  return s;
}

// ---- The independent double design -----------------------------------------------------------------------------
struct Stage {
  double b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
};

inline Stage to_stage(const BiquadCoeffs<float>& c) {
  return Stage{c.b0, c.b1, c.b2, c.a1, c.a2};
}

// The K form (kept independent of the RBJ form the firmware uses): K = tan(pi f0 T), norm = 1 / (1 + K/Q + K^2),
// b0 = (1 + K^2) norm, b1 = 2 (K^2 - 1) norm, b2 = b0, a1 = b1, a2 = (1 - K/Q + K^2) norm.
inline Stage notch_design(double f0, double q, double period) {
  const double k = std::tan(kPi * f0 * period);
  const double norm = 1.0 / (1.0 + k / q + k * k);
  Stage s;
  s.b0 = (1.0 + k * k) * norm;
  s.b1 = 2.0 * (k * k - 1.0) * norm;
  s.b2 = s.b0;
  s.a1 = s.b1;
  s.a2 = (1.0 - k / q + k * k) * norm;
  return s;
}

// K = tan(pi f_c T), norm = 1 / (1 + sqrt(2) K + K^2), b0 = K^2 norm, b1 = 2 b0, b2 = b0, a1 = 2 (K^2 - 1) norm,
// a2 = (1 - sqrt(2) K + K^2) norm.
inline Stage lowpass_design(double fc, double period) {
  const double k = std::tan(kPi * fc * period);
  const double norm = 1.0 / (1.0 + std::sqrt(2.0) * k + k * k);
  Stage s;
  s.b0 = k * k * norm;
  s.b1 = 2.0 * s.b0;
  s.b2 = s.b0;
  s.a1 = 2.0 * (k * k - 1.0) * norm;
  s.a2 = (1.0 - std::sqrt(2.0) * k + k * k) * norm;
  return s;
}

// H(e^{jw}) = (b0 + b1 z^-1 + b2 z^-2) / (1 + a1 z^-1 + a2 z^-2), z^-1 = e^{-jw}.
inline Cd stage_response(const Stage& s, double w) {
  const Cd z1 = std::polar(1.0, -w);
  const Cd z2 = z1 * z1;
  return (s.b0 + s.b1 * z1 + s.b2 * z2) / (1.0 + s.a1 * z1 + s.a2 * z2);
}

inline Cd chain_response(const std::vector<Stage>& st, double w) {
  Cd h = 1.0;
  for (const Stage& s : st) {
    h *= stage_response(s, w);
  }
  return h;
}

// Digital frequency of a tone of f Hz.
inline double digital_w(double f, double fs) { return 2.0 * kPi * f / fs; }

// ---- Running forward-error analysis of the float coefficient code ---------------------------------------------
// Err carries the real-valued reference v (evaluated in double, whose own 1e-16 rounding is far below every bound)
// and a bound e on |float result - v|. Each float operation adds u |v| (half an ulp of the result); a libm call adds
// kLibmUlps ulp of its result (an ulp is at most 2 u |v|) and the propagated argument error. First order in u.
struct Err {
  double v;
  double e;
};

inline Err exact(double v) { return Err{v, 0.0}; }
inline Err rounded(double v) { return Err{v, kU * std::fabs(v)}; }  // an inexact constant: one rounding of the value
inline Err scale2(Err a) { return Err{2.0 * a.v, 2.0 * a.e}; }       // multiplication by 2 is exact
inline Err neg(Err a) { return Err{-a.v, a.e}; }
inline Err add(Err a, Err b) {
  const double v = a.v + b.v;
  return Err{v, a.e + b.e + kU * std::fabs(v)};
}
inline Err sub(Err a, Err b) {
  const double v = a.v - b.v;
  return Err{v, a.e + b.e + kU * std::fabs(v)};
}
inline Err mul(Err a, Err b) {
  const double v = a.v * b.v;
  return Err{v, std::fabs(a.v) * b.e + std::fabs(b.v) * a.e + a.e * b.e + kU * std::fabs(v)};
}
inline Err div(Err a, Err b) {
  const double v = a.v / b.v;
  return Err{v, (a.e + std::fabs(v) * b.e) / (std::fabs(b.v) - b.e) + kU * std::fabs(v)};
}
inline Err sin_e(Err a) {
  const double v = std::sin(a.v);
  return Err{v, std::fabs(std::cos(a.v)) * a.e + kLibmUlps * 2.0 * kU * std::fabs(v)};
}
inline Err cos_e(Err a) {
  const double v = std::cos(a.v);
  return Err{v, std::fabs(std::sin(a.v)) * a.e + kLibmUlps * 2.0 * kU * std::fabs(v)};
}
inline Err tan_e(Err a) {
  const double v = std::tan(a.v);
  return Err{v, (1.0 + v * v) * a.e + kLibmUlps * 2.0 * kU * std::fabs(v)};
}

// A stage's reference coefficients and the bound on each float coefficient's distance from them.
struct StageErr {
  Stage v;
  Stage e;
};

// The sequence of operations of lowpass_coeffs<float>.
inline StageErr lowpass_err(float fc, float period) {
  const Err pi = rounded(kPi);
  const Err k = tan_e(mul(mul(pi, exact(fc)), exact(period)));
  const Err k2 = mul(k, k);
  const Err rk = mul(rounded(std::sqrt(2.0)), k);
  const Err norm = div(exact(1.0), add(add(exact(1.0), rk), k2));
  const Err b0 = mul(k2, norm);
  const Err b1 = scale2(b0);
  const Err a1 = mul(scale2(sub(k2, exact(1.0))), norm);
  const Err a2 = mul(add(sub(exact(1.0), rk), k2), norm);
  return StageErr{Stage{b0.v, b1.v, b0.v, a1.v, a2.v}, Stage{b0.e, b1.e, b0.e, a1.e, a2.e}};
}

// The sequence of operations of notch_coeffs<float>, for a reference notch frequency f0 (Hz) that the float code
// receives with the relative error f0_rel_err (0 when f0 is the exact float input of the function).
inline StageErr notch_err(double f0, float q, float period, double f0_rel_err) {
  const Err two_pi = scale2(rounded(kPi));
  const Err f = Err{f0, f0_rel_err * f0};
  const Err w0 = mul(mul(two_pi, f), exact(period));
  const Err alpha = div(sin_e(w0), scale2(exact(q)));
  const Err cw = cos_e(w0);
  const Err norm = div(exact(1.0), add(exact(1.0), alpha));
  const Err b1 = mul(neg(scale2(cw)), norm);
  const Err a2 = mul(sub(exact(1.0), alpha), norm);
  return StageErr{Stage{norm.v, b1.v, norm.v, b1.v, a2.v}, Stage{norm.e, b1.e, norm.e, b1.e, a2.e}};
}

// ---- l1 rounding bound, settling time and coefficient sensitivity -------------------------------------------
inline std::vector<double> run_stage(const Stage& s, const std::vector<double>& x) {
  std::vector<double> y(x.size(), 0.0);
  double x1 = 0, x2 = 0, y1 = 0, y2 = 0;
  for (std::size_t n = 0; n < x.size(); ++n) {
    const double v = s.b0 * x[n] + s.b1 * x1 + s.b2 * x2 - s.a1 * y1 - s.a2 * y2;
    x2 = x1;
    x1 = x[n];
    y2 = y1;
    y1 = v;
    y[n] = v;
  }
  return y;
}

inline double l1(const std::vector<double>& h) {
  double s = 0;
  for (const double v : h) {
    s += std::fabs(v);
  }
  return s;
}

inline std::vector<double> unit_impulse() {
  std::vector<double> d(kImpulseLen, 0.0);
  d[0] = 1.0;
  return d;
}

// The smallest n whose impulse-response tail sum_{m > n} |h_m| is at most `tol`; h must have decayed (checked by the
// caller). The tail decays like r_max^n, r_max the largest pole radius.
inline std::size_t settle_samples(const std::vector<double>& h, double tol) {
  double tail = 0;
  std::size_t n = h.size();
  while (n > 0 && tail + std::fabs(h[n - 1]) <= tol) {
    tail += std::fabs(h[n - 1]);
    --n;
  }
  return n;
}

inline double tail_after(const std::vector<double>& h, std::size_t n) {
  double tail = 0;
  for (std::size_t m = n + 1; m < h.size(); ++m) {
    tail += std::fabs(h[m]);
  }
  return tail;
}

// The largest pole radius of a stage list: sqrt(a2) for a complex pair (a2 > 0 and a1^2 < 4 a2).
inline double max_pole_radius(const std::vector<Stage>& st) {
  double r = 0;
  for (const Stage& s : st) {
    const double disc = s.a1 * s.a1 - 4.0 * s.a2;
    const double rad = disc < 0 ? std::sqrt(s.a2) : (std::fabs(s.a1) + std::sqrt(disc)) / 2.0;
    r = std::max(r, rad);
  }
  return r;
}

// First-order bound on |H_float(e^{jw}) - H_design(e^{jw})| when each coefficient is within its bound: the sum over
// the coefficients of the larger change of H for a perturbation of that coefficient by +- its bound.
inline double coefficient_effect(const std::vector<StageErr>& se, double w) {
  std::vector<Stage> base;
  for (const StageErr& s : se) {
    base.push_back(s.v);
  }
  const Cd h0 = chain_response(base, w);
  double total = 0;
  for (std::size_t k = 0; k < se.size(); ++k) {
    for (int c = 0; c < 5; ++c) {
      double worst = 0;
      for (const double sgn : {-1.0, 1.0}) {
        std::vector<Stage> p = base;
        double* const f[5] = {&p[k].b0, &p[k].b1, &p[k].b2, &p[k].a1, &p[k].a2};
        const double* const e[5] = {&se[k].e.b0, &se[k].e.b1, &se[k].e.b2, &se[k].e.a1, &se[k].e.a2};
        *f[c] += sgn * *e[c];
        worst = std::max(worst, std::abs(chain_response(p, w) - h0));
      }
      total += worst;
    }
  }
  return total;
}

// What a chain is designed to be and how far the float chain may be from it.
//
// Rounding bound (the l1 precedent of decision 0005, refined to the measured quantity). Direct form I, evaluated left to right,
// makes stage k's output
//   y_n = fl(b0 x_n + b1 x_(n-1) + b2 x_(n-2) - a1 y_(n-1) - a2 y_(n-2))
// with a local error delta_n at most gamma_5 (|b0| |x_n| + |b1| |x_(n-1)| + |b2| |x_(n-2)| + |a1| |y_(n-1)| + |a2| |y_(n-2)|),
// gamma_5 = 5u / (1 - 5u): a dot product of five products summed left to right meets at most five roundings per term
// (Higham, Accuracy and Stability of Numerical Algorithms, 2nd ed., sections 3.1 and 3.5). Stage k's error enters its
// own recursion (1/A_k) and then the later stages, so it reaches the output as e = g_k * delta with the impulse response of
// G_k = (1/A_k) H_(k+1) ... H_K, and the input's own rounding (at most u, the cosine is rounded to float) as
// h * delta_0. The test measures H_hat = (2/N) sum_{n in window} y_n e^{-jwn} (window n_s .. n_s + N - 1), so the error of a
// stage reaches H_hat as E = (2/N) sum_m delta_m e^{-jwm} S_m with S_m the window part of sum_j g_j e^{-jwj}. For m in
// the window and N - 1 - (m - n_s) >= j the sum is G_k(e^{jw}) up to the impulse-response tail beyond that j; for m before
// the window it is at most the tail from n_s - m. Summing the tails over m gives at most M1 = sum_j j |g_j|, hence
//   |E| <= 2 delta_late |G_k(e^{jw})| + (2 / N) (delta_late + delta_early) M1_k,
// delta_late for the signals after the settling time (|x| <= 1 + transient tail, each stage's |H| <= 1 so the steady
// signals are at most the input's amplitude) and delta_early before it (the signals bounded by the l1 norm of the
// cascade up to that stage). The same applies to the input's rounding with G = H. Every |H_k| <= 1: the notch
// |H|^2 = (1 - x^2)^2 / ((1 - x^2)^2 + x^2 / Q^2) and the Butterworth low-pass are at most 1 (checked on a grid in
// response_test.cpp).
struct Plan {
  std::vector<StageErr> se;
  std::vector<Stage> design;
  std::size_t n_settle = 0;
  double tail = 0;
  double r_max = 0;
  std::vector<double> delta_late;
  std::vector<double> delta_early;
  std::vector<double> m1;  // M1 of G_k, then of the whole chain last
};

inline double first_moment(const std::vector<double>& g) {
  double s = 0;
  for (std::size_t j = 0; j < g.size(); ++j) {
    s += static_cast<double>(j) * std::fabs(g[j]);
  }
  return s;
}

inline Plan make_plan(const std::vector<StageErr>& se) {
  Plan p;
  p.se = se;
  for (const StageErr& s : se) {
    p.design.push_back(s.v);
  }
  const std::size_t n_st = p.design.size();
  const double gamma5 = 5.0 * kU / (1.0 - 5.0 * kU);
  std::vector<std::vector<double>> cum(n_st + 1);
  cum[0] = unit_impulse();
  for (std::size_t k = 0; k < n_st; ++k) {
    cum[k + 1] = run_stage(p.design[k], cum[k]);
  }
  p.n_settle = settle_samples(cum[n_st], kTransientTol);
  p.tail = tail_after(cum[n_st], p.n_settle);
  p.r_max = max_pole_radius(p.design);
  for (std::size_t k = 0; k < n_st; ++k) {
    const Stage& s = p.design[k];
    const double sum_b = std::fabs(s.b0) + std::fabs(s.b1) + std::fabs(s.b2);
    const double sum_a = std::fabs(s.a1) + std::fabs(s.a2);
    const double late_in = 1.0 + tail_after(cum[k], p.n_settle) + kAmplitudeSlack;
    const double late_out = 1.0 + tail_after(cum[k + 1], p.n_settle) + kAmplitudeSlack;
    const double early_in = l1(cum[k]) + kAmplitudeSlack;
    const double early_out = l1(cum[k + 1]) + kAmplitudeSlack;
    p.delta_late.push_back(gamma5 * (sum_b * late_in + sum_a * late_out));
    p.delta_early.push_back(gamma5 * (sum_b * early_in + sum_a * early_out));
    Stage rec;
    rec.a1 = s.a1;
    rec.a2 = s.a2;
    std::vector<double> g = run_stage(rec, unit_impulse());
    for (std::size_t j = k + 1; j < n_st; ++j) {
      g = run_stage(p.design[j], g);
    }
    p.m1.push_back(first_moment(g));
  }
  p.m1.push_back(first_moment(cum[n_st]));
  return p;
}

// Slack on the rounding bound: it is evaluated on the design coefficients; the float coefficients differ by relative
// <= 1e-5 (coeff_test.cpp), which moves |G_k| and M1_k by far less than 10 %.
constexpr double kBoundSlack = 1.1;

// The rounding part of the tolerance at the digital frequency w (formula in the comment of Plan).
inline double rounding_bound(const Plan& p, double w) {
  const double n = static_cast<double>(kWindow);
  const Cd z1 = std::polar(1.0, -w);
  const Cd z2 = z1 * z1;
  std::vector<Cd> h(p.design.size());
  for (std::size_t k = 0; k < h.size(); ++k) {
    h[k] = stage_response(p.design[k], w);
  }
  Cd total = 1.0;
  for (const Cd& v : h) {
    total *= v;
  }
  double b = 2.0 * kU * std::abs(total) + (2.0 / n) * (2.0 * kU) * p.m1.back();
  for (std::size_t k = 0; k < h.size(); ++k) {
    Cd g = 1.0 / (1.0 + p.design[k].a1 * z1 + p.design[k].a2 * z2);
    for (std::size_t j = k + 1; j < h.size(); ++j) {
      g *= h[j];
    }
    b += 2.0 * p.delta_late[k] * std::abs(g) + (2.0 / n) * (p.delta_late[k] + p.delta_early[k]) * p.m1[k];
  }
  return kBoundSlack * b;
}

// Relative error of f0 = (h omega) / (2 pi) evaluated in float as update_notches does: float(h) is exact, the product
// h omega, the constant 2 pi (float(pi) doubled) and the division each round once: 3 u.
constexpr double kF0RelErr = 3.0 * kU;

// The float f0 of notch (motor, harmonic) as update_notches computes it: (h * omega) / (2 * float(pi)).
inline float chain_f0(std::size_t harmonic, float omega) {
  const float two_pi = 2.0F * static_cast<float>(kPi);
  return static_cast<float>(harmonic + 1) * omega / two_pi;
}

// The plan of a chain whose notch (m, h) is at multiple (h + 1 + harmonic_shift) of the given speeds (rad/s), with the
// motors in `on` active, then the low-pass. Fixture Q and cutoff.
inline Plan plan_for(const Fixture& fx, const std::array<float, kMotors>& omega,
                     const std::array<bool, kMotors>& on, std::size_t harmonic_shift = 0) {
  std::vector<StageErr> se;
  for (std::size_t m = 0; m < kMotors; ++m) {
    if (!on[m]) {
      continue;
    }
    for (std::size_t h = 0; h < kHarmonics; ++h) {
      const double f0 = static_cast<double>(h + 1 + harmonic_shift) * static_cast<double>(omega[m]) / (2.0 * kPi);
      se.push_back(notch_err(f0, fx.config.notch_q[h], fx.period, kF0RelErr));
    }
  }
  se.push_back(lowpass_err(fx.cutoff, fx.period));
  return make_plan(se);
}

// ---- A test-local cascade, the explicit reference ------------------------------------------------------------
class RefChain {
 public:
  explicit RefChain(std::vector<BiquadCoeffs<float>> stages) : c_(std::move(stages)) {
    for (auto& axis : s_) {
      axis.assign(c_.size(), State{});
    }
  }
  Vec3f filter(const Vec3f& in) {
    Vec3f out;
    for (std::size_t a = 0; a < kAxes; ++a) {
      float v = in[a];
      for (std::size_t k = 0; k < c_.size(); ++k) {
        State& s = s_[a][k];
        const BiquadCoeffs<float>& c = c_[k];
        const float y = c.b0 * v + c.b1 * s.x1 + c.b2 * s.x2 - c.a1 * s.y1 - c.a2 * s.y2;
        s.x2 = s.x1;
        s.x1 = v;
        s.y2 = s.y1;
        s.y1 = y;
        v = y;
      }
      out[a] = v;
    }
    return out;
  }

 private:
  struct State {
    float x1 = 0, x2 = 0, y1 = 0, y2 = 0;
  };
  std::vector<BiquadCoeffs<float>> c_;
  std::array<std::vector<State>, kAxes> s_;
};

// The reference chain's stages: notch (m, h) for the (m, h) with include[notch_index] set, at multiple h + 1 + shift of
// omega[m], then the low-pass.
inline std::vector<BiquadCoeffs<float>> ref_stages(const Fixture& fx, const std::array<float, kMotors>& omega,
                                                   const std::array<bool, kNotches>& include,
                                                   std::size_t harmonic_shift = 0) {
  std::vector<BiquadCoeffs<float>> st;
  for (std::size_t m = 0; m < kMotors; ++m) {
    for (std::size_t h = 0; h < kHarmonics; ++h) {
      if (include[notch_index(m, h)]) {
        st.push_back(notch_coeffs<float>(chain_f0(h + harmonic_shift, omega[m]), fx.config.notch_q[h], fx.period));
      }
    }
  }
  st.push_back(lowpass_coeffs<float>(fx.cutoff, fx.period));
  return st;
}

// ---- DTFT measurement ----------------------------------------------------------------------------------------
inline double bin_hz(double fs, std::size_t k) { return fs * static_cast<double>(k) / static_cast<double>(kWindow); }

inline std::size_t nearest_bin(double fs, double f) {
  return static_cast<std::size_t>(std::llround(f * static_cast<double>(kWindow) / fs));
}

// Drives x_n = cos(w n), w = 2 pi k / N, into all three axes (float), discards the first n_settle samples, then
// returns H_hat = (2 / N) sum_{n = n_settle}^{n_settle + N - 1} y_n e^{-j w n} of axis 0. Since the window holds exactly k
// periods and the demodulated image term sum e^{-2jwn} vanishes, H_hat = H(e^{jw}) up to the transient, the rounding and
// the input's rounding. The three axes must be bit-identical (coefficients are shared; the inputs are equal).
template <class Filter>
Cd measure(Filter&& step, std::size_t k, std::size_t n_settle) {
  Cd acc = 0.0;
  bool axes_equal = true;
  const std::size_t total = n_settle + kWindow;
  for (std::size_t n = 0; n < total; ++n) {
    const double ph = 2.0 * kPi * static_cast<double>((k * n) % kWindow) / static_cast<double>(kWindow);
    const float x = static_cast<float>(std::cos(ph));
    const Vec3f y = step(Vec3f(x, x, x));
    axes_equal = axes_equal && std::memcmp(&y[0], &y[1], sizeof(float)) == 0 &&
                 std::memcmp(&y[0], &y[2], sizeof(float)) == 0;
    if (n >= n_settle) {
      acc += static_cast<double>(y[0]) * std::polar(1.0, -ph);
    }
  }
  EXPECT_TRUE(axes_equal);
  return acc * (2.0 / static_cast<double>(kWindow));
}

// Total tolerance of the measured against the design response at w: coefficient effect + rounding bound + transient
// (the transient's DTFT, normalised by 2 / N, is at most 2 tail).
inline double tolerance(const Plan& p, double w) { return coefficient_effect(p.se, w) + rounding_bound(p, w) + 2.0 * p.tail; }

inline bool bits_equal(const Vec3f& a, const Vec3f& b) {
  return std::memcmp(a.e, b.e, sizeof(a.e)) == 0;
}

}  // namespace marv::gyro_chain::test
