// L6 stage (b) T1, lines 2 and 4 (response part): the float chain's steady-state response against the exact design
// response |H(e^{jwT})| of the double design at the notch centres, at f0 (1 +- eps), in the crossover band and at the
// rate loop's Nyquist f_r / 2.
//
// Measurement: a unit cosine on a DTFT bin (whole periods in the window, support.hpp measure), after a settling time.
// Settling time: the smallest n whose impulse-response tail sum_{m > n} |h_m| of the design chain is at most
// kTransientTol; the tail decays like r_max^n for the largest pole radius r_max (the slowest pole, the sharpest notch at
// the lowest frequency), and that tail bounds the transient of the zero-state start for |x| <= 1.
// Tolerance of measured against design: coefficient effect (first order, from the coefficient bounds of coeff_test.cpp
// and the 3u error of f0 = h omega / 2 pi) + the l1 rounding bound of decision 0005's precedent (l1 of the error
// transfer times the local rounding, here with the direct-form-I local bound gamma_5) + the transient tail. The leakage of
// the DTFT is zero by construction (bins).
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include "support.hpp"

namespace {

using namespace marv;
using namespace marv::gyro_chain;
using namespace marv::gyro_chain::test;

using Step = std::function<Vec3f(const Vec3f&)>;
using Factory = std::function<Step()>;

Factory chain_factory(const GyroChainConfig<float>& cfg, const std::array<float, kMotors>& omega,
                      std::uint32_t flags = kRotorSpeedFlagsDefined) {
  return [cfg, omega, flags]() -> Step {
    auto chain = std::make_shared<GyroChain<float>>();
    chain->init(cfg);
    chain->update_notches(make_sample(omega, flags));
    return [chain](const Vec3f& v) { return chain->filter(v); };
  };
}

// The largest Q with |H| <= a_min at f0 (1 +- eps) for the notch (s^2 + 1) / (s^2 + s/Q + 1) prewarped at f0:
// with x = tan(pi f0 (1 +- eps) / f_s) / tan(pi f0 / f_s), |H|^2 = (1 - x^2)^2 / ((1 - x^2)^2 + x^2 / Q^2), so
// Q = x / (|1 - x^2| sqrt(1 / a_min^2 - 1)); the smaller of the two sides.
double q_max(double f0, double fs) {
  const double k0 = std::tan(kPi * f0 / fs);
  double q = std::numeric_limits<double>::infinity();
  for (const double s : {-1.0, 1.0}) {
    const double x = std::tan(kPi * f0 * (1.0 + s * kEps) / fs) / k0;
    q = std::min(q, x / (std::fabs(1.0 - x * x) * std::sqrt(1.0 / (kAMin * kAMin) - 1.0)));
  }
  return q;
}

struct Depth {
  bool ok = true;        // the notch alone: every edge tone's notch response at most a_min (+ tol)
  bool chain_ok = true;  // the whole chain: every edge tone at most a_min (+ tol)
  bool agree = true;     // every tone within tol of the design response
  double worst = 0;      // largest notch-only |H| at an edge tone
  double worst_chain = 0;
  double worst_dev_over_tol = 0;  // largest |measured - design| / tolerance
};

// The edge tones f0 (1 +- eps) of motor 1's harmonics, each on the bin nearer f0 (so the gain there is at most the gain at
// the exact edge, a gain that is monotone in |f - f0|). The notch's own response is the measured chain response divided by
// the design response of the other 12 stages (the low-pass and the other notches, |H_rest| >= 0.2 at these tones), so the
// low-pass's extra attenuation at 2x and 3x does not hide a notch that is too wide; the tolerance divides likewise.
Depth edge_check(const Factory& make, const Plan& plan, const Fixture& fx) {
  Depth d;
  for (std::size_t h = 0; h < kHarmonics; ++h) {
    const double f0 = static_cast<double>(h + 1) * kRotorHz[0];
    for (const double s : {-1.0, 1.0}) {
      const double fe = f0 * (1.0 + s * kEps);
      const double fb = bin_hz(fx.fs, 1);
      const std::size_t k = static_cast<std::size_t>(s > 0 ? std::floor(fe / fb) : std::ceil(fe / fb));
      const double w = digital_w(bin_hz(fx.fs, k), fx.fs);
      const Cd hm = measure(make(), k, plan.n_settle);
      const Cd hd = chain_response(plan.design, w);
      const double tol = tolerance(plan, w);
      std::vector<Stage> rest = plan.design;
      rest.erase(rest.begin() + static_cast<std::ptrdiff_t>(notch_index(0, h)));
      const double rest_gain = std::abs(chain_response(rest, w));
      d.agree = d.agree && std::abs(hm - hd) <= tol;
      d.worst_dev_over_tol = std::max(d.worst_dev_over_tol, std::abs(hm - hd) / tol);
      d.chain_ok = d.chain_ok && std::abs(hm) <= kAMin + tol;
      d.ok = d.ok && std::abs(hm) / rest_gain <= kAMin + tol / rest_gain;
      d.worst = std::max(d.worst, std::abs(hm) / rest_gain);
      d.worst_chain = std::max(d.worst_chain, std::abs(hm));
    }
  }
  return d;
}

const std::array<bool, kMotors> kAllOn{true, true, true, true};

TEST(L6GyroChainResponse, SettlingTimeFollowsTheSlowestPole) {
  const Fixture fx = make_fixture();
  const Plan plan = plan_for(fx, rotor_omegas(), kAllOn);
  EXPECT_LT(plan.r_max, 1.0);
  EXPECT_LE(plan.tail, kTransientTol);
  EXPECT_GT(plan.n_settle, 0U);
  EXPECT_LT(plan.n_settle, kImpulseLen / 4);
  // The design impulse response has decayed to nothing at the end of the record the tail is summed over.
  std::vector<double> h = unit_impulse();
  for (const Stage& st : plan.design) {
    h = run_stage(st, h);
  }
  EXPECT_LT(std::fabs(h.back()), 1.0e-30);
  // Every stage has |H| <= 1 (the amplitude bound of the rounding analysis), on a grid of 4096 digital frequencies in
  // [0, pi] and at DC, where a notch and the low-pass are exactly 1.
  for (const Stage& st : plan.design) {
    EXPECT_NEAR(std::abs(stage_response(st, 0.0)), 1.0, 1.0e-12);
    for (std::size_t i = 0; i <= 4096; ++i) {
      EXPECT_LE(std::abs(stage_response(st, kPi * static_cast<double>(i) / 4096.0)), 1.0 + 1.0e-9);
    }
  }
  // The tail is geometric in the slowest pole: the settling time is within a factor 2 of ln(tol) / ln(r_max).
  const double n_pole = std::log(kTransientTol) / std::log(plan.r_max);
  EXPECT_GE(static_cast<double>(plan.n_settle), n_pole / 2.0);
  EXPECT_LE(static_cast<double>(plan.n_settle), n_pole * 2.0);
  // The whole tolerance is small against the thresholds it is added to (a_min = 0.1) at the tones used.
  const double w_probe = digital_w(bin_hz(fx.fs, nearest_bin(fx.fs, kRotorHz[0])), fx.fs);
  EXPECT_LT(tolerance(plan, w_probe), 1.0e-3);
  RecordProperty("n_settle", static_cast<int>(plan.n_settle));
  RecordProperty("r_max", std::to_string(plan.r_max));
  RecordProperty("tolerance_at_400Hz", std::to_string(tolerance(plan, w_probe)));
  RecordProperty("rounding_at_400Hz", std::to_string(rounding_bound(plan, w_probe)));
  RecordProperty("coefficient_effect_at_400Hz", std::to_string(coefficient_effect(plan.se, w_probe)));
}

TEST(L6GyroChainResponse, NotchDepthAtTheCentres) {
  const Fixture fx = make_fixture();
  const auto omega = rotor_omegas();
  const Plan plan = plan_for(fx, omega, kAllOn);
  const Factory make = chain_factory(fx.config, omega);
  for (std::size_t m = 0; m < kMotors; ++m) {
    for (std::size_t h = 0; h < kHarmonics; ++h) {
      const double f0 = static_cast<double>(h + 1) * kRotorHz[m];
      const std::size_t k = nearest_bin(fx.fs, f0);
      const double w = digital_w(bin_hz(fx.fs, k), fx.fs);
      SCOPED_TRACE(testing::Message() << "motor " << m + 1 << " harmonic " << h + 1 << " f " << bin_hz(fx.fs, k));
      const Cd hm = measure(make(), k, plan.n_settle);
      const Cd hd = chain_response(plan.design, w);
      EXPECT_LE(std::abs(hm - hd), tolerance(plan, w));
      // The design has its zero within one bin of the tone: at most a_min / 10 there (deep notch).
      EXPECT_LE(std::abs(hd), kAMin / 10.0);
    }
  }
  // Motor 1's tones are exact bins (400, 800, 1200 Hz = f_s k / N): there the design is a true zero.
  for (std::size_t h = 0; h < kHarmonics; ++h) {
    const std::size_t k = nearest_bin(fx.fs, static_cast<double>(h + 1) * kRotorHz[0]);
    const double w = digital_w(bin_hz(fx.fs, k), fx.fs);
    const Cd hm = measure(make(), k, plan.n_settle);
    EXPECT_LE(std::abs(hm), tolerance(plan, w)) << h + 1;
    EXPECT_LE(std::abs(chain_response(plan.design, w)), 1.0e-6);
  }
}

TEST(L6GyroChainResponse, ScratchQMeetsAMinAtF0PlusMinusEps) {
  const Fixture fx = make_fixture();
  const auto omega = rotor_omegas();
  for (std::size_t h = 0; h < kHarmonics; ++h) {
    EXPECT_LE(static_cast<double>(kScratchQ[h]), q_max(static_cast<double>(h + 1) * kRotorHz[0], fx.fs)) << h + 1;
  }
  const Plan plan = plan_for(fx, omega, kAllOn);
  // The exact design at the exact edge frequencies (not bins): at most a_min.
  for (std::size_t h = 0; h < kHarmonics; ++h) {
    for (const double s : {-1.0, 1.0}) {
      const double f = static_cast<double>(h + 1) * kRotorHz[0] * (1.0 + s * kEps);
      EXPECT_LE(std::abs(chain_response(plan.design, digital_w(f, fx.fs))), kAMin) << h + 1 << " " << s;
    }
  }
  const Depth d = edge_check(chain_factory(fx.config, omega), plan, fx);
  EXPECT_TRUE(d.agree);
  EXPECT_TRUE(d.ok);
  EXPECT_TRUE(d.chain_ok);
  RecordProperty("worst_edge_notch_gain", std::to_string(d.worst));
  RecordProperty("worst_edge_chain_gain", std::to_string(d.worst_chain));
  RecordProperty("worst_deviation_over_tolerance", std::to_string(d.worst_dev_over_tol));
}

TEST(L6GyroChainResponse, CrossoverBandAndRateLoopNyquist) {
  const Fixture fx = make_fixture();
  const auto omega = rotor_omegas();
  const Plan plan = plan_for(fx, omega, kAllOn);
  const Factory make = chain_factory(fx.config, omega);
  // 14 rad/s on the nearest bin, and f_r / 2 = f_s / 4 (an exact bin).
  const double crossover_hz = kCrossoverRadS / (2.0 * kPi);
  const std::array<std::size_t, 2> bins{nearest_bin(fx.fs, crossover_hz),
                                        nearest_bin(fx.fs, fx.fs / static_cast<double>(kDivisor) / 2.0)};
  for (const std::size_t k : bins) {
    const double w = digital_w(bin_hz(fx.fs, k), fx.fs);
    SCOPED_TRACE(bin_hz(fx.fs, k));
    const Cd hm = measure(make(), k, plan.n_settle);
    const Cd hd = chain_response(plan.design, w);
    EXPECT_LE(std::abs(hm - hd), tolerance(plan, w));
    // Phase against the design as well, where the gain is large enough for a phase to mean something.
    if (std::abs(hd) > 0.5) {
      EXPECT_LE(std::abs(std::arg(hm / hd)), tolerance(plan, w) / std::abs(hd));
    }
  }
  // At f_r / 2 the rule's low-pass alone is a_min and the notches only lower it: the chain's design gain is at most a_min.
  const double w_nyq = digital_w(bin_hz(fx.fs, bins[1]), fx.fs);
  EXPECT_LE(std::abs(chain_response(plan.design, w_nyq)), kAMin * (1.0 + 1.0e-6));
  // At 14 rad/s the chain passes the tone: gain within 1e-3 of 1 (twelve notches of Q >= 9.3 and the low-pass: the
  // design gain, not a measurement).
  const double w_x = digital_w(bin_hz(fx.fs, bins[0]), fx.fs);
  EXPECT_NEAR(std::abs(chain_response(plan.design, w_x)), 1.0, 1.0e-3);
}

// A notch must not hold its coefficients after its motor's sample turns bad: a good update then a NaN update, with no
// input in between, is the chain with motor 1's notches absent.
TEST(L6GyroChainResponse, NoStaleCoefficientAfterABadSample) {
  const Fixture fx = make_fixture();
  const auto omega = rotor_omegas();
  const std::array<bool, kMotors> m1_off{false, true, true, true};
  const Plan plan = plan_for(fx, omega, m1_off);
  const Factory make = [&]() -> Step {
    auto chain = std::make_shared<GyroChain<float>>();
    chain->init(fx.config);
    chain->update_notches(make_sample(omega));
    auto bad = omega;
    bad[0] = std::numeric_limits<float>::quiet_NaN();
    chain->update_notches(make_sample(bad));
    return [chain](const Vec3f& v) { return chain->filter(v); };
  };
  const std::size_t k = nearest_bin(fx.fs, kRotorHz[0]);
  const double w = digital_w(bin_hz(fx.fs, k), fx.fs);
  const Cd hm = measure(make(), k, plan.n_settle);
  EXPECT_LE(std::abs(hm - chain_response(plan.design, w)), tolerance(plan, w));
  // Motor 1's 400 Hz is passed: the other motors' notches are far from it.
  EXPECT_GT(std::abs(hm), 0.5);
}

// ---- Negative controls: each must fail the edge-depth check -------------------------------------------------
Fixture with_q(const Fixture& fx, double scale) {
  Fixture g = fx;
  for (std::size_t h = 0; h < kHarmonics; ++h) {
    g.config.notch_q[h] = static_cast<float>(scale * q_max(static_cast<double>(h + 1) * kRotorHz[0], fx.fs));
  }
  return g;
}

// Positive twin of the Q control: the Q that just meets a_min at f0 (1 +- eps) passes, so the control's failure is the
// 1.1 and not the check.
TEST(L6GyroChainResponse, QThatJustMeetsAMinPassesTheEdgeCheck) {
  const Fixture fx = with_q(make_fixture(), 1.0);
  const auto omega = rotor_omegas();
  const Depth d = edge_check(chain_factory(fx.config, omega), plan_for(fx, omega, kAllOn), fx);
  EXPECT_TRUE(d.agree);
  EXPECT_TRUE(d.ok);
  EXPECT_LE(d.worst, kAMin + 1.0e-3);
}

TEST(L6GyroChainResponse, ControlQTimesOnePointOneFailsTheEdgeCheck) {
  const Fixture fx = with_q(make_fixture(), 1.1);
  const auto omega = rotor_omegas();
  const Depth d = edge_check(chain_factory(fx.config, omega), plan_for(fx, omega, kAllOn), fx);
  EXPECT_TRUE(d.agree);  // the chain is what it was designed to be: only its depth requirement is broken
  EXPECT_FALSE(d.ok);
  EXPECT_GT(d.worst, kAMin);
}

TEST(L6GyroChainResponse, ControlF0OffByOneTelemetryStepFailsTheEdgeCheck) {
  const Fixture fx = make_fixture();
  auto omega = rotor_omegas();
  omega[0] *= static_cast<float>(1.0 + kTelemetryStep);
  const Depth d = edge_check(chain_factory(fx.config, omega), plan_for(fx, omega, kAllOn), fx);
  EXPECT_FALSE(d.ok);
  EXPECT_GT(d.worst, kAMin);
}

TEST(L6GyroChainResponse, ControlHarmonicIndexPlusOneFailsTheEdgeCheck) {
  const Fixture fx = make_fixture();
  const auto omega = rotor_omegas();
  std::array<bool, kNotches> all{};
  all.fill(true);
  const Factory make = [&]() -> Step {
    auto ref = std::make_shared<RefChain>(ref_stages(fx, omega, all, 1));
    return [ref](const Vec3f& v) { return ref->filter(v); };
  };
  const Depth d = edge_check(make, plan_for(fx, omega, kAllOn, 1), fx);
  EXPECT_FALSE(d.ok);
  EXPECT_GT(d.worst, kAMin);
}

TEST(L6GyroChainResponse, ControlRevolutionsPerSecondInPlaceOfRadiansPerSecondFailsTheEdgeCheck) {
  const Fixture fx = make_fixture();
  std::array<float, kMotors> rev{};
  for (std::size_t m = 0; m < kMotors; ++m) {
    rev[m] = static_cast<float>(kRotorHz[m]);  // the same rotors, in rev/s
  }
  const Depth d = edge_check(chain_factory(fx.config, rev), plan_for(fx, rev, kAllOn), fx);
  EXPECT_FALSE(d.ok);
  EXPECT_GT(d.worst, kAMin);
}

}  // namespace
