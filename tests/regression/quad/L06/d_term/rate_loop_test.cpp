// L6 stage (c), commit 1, T1 (decision 0014): the rate loop's D low-pass (quad spec D2) and the lag-compensated
// omega x J omega feed-forward (F1-F3). Both are inert by default: with T_f = 0 and every J = 0 the loop is today's law
// bit for bit (the first two tests), and the frozen L4 and L5 suites stay as they were.
//
// Numbers here are one of: derived (the rule is stated), a cited constant (the float unit roundoff), or a "scenario test
// value" named with its reason. Nothing is a product value.
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

#include <marv/mixer/mixer.hpp>
#include <marv/params/param.hpp>
#include <marv/prim/constants.hpp>
#include <marv/prim/vec.hpp>
#include <marv/rate/rate_loop.hpp>
#include <marv/types/imu_sample.hpp>

// init_params(): the one params_init of this executable (it may run once; chain_guard_test.cpp uses it too).
#include "../gyro_chain/support.hpp"

namespace {

using namespace marv;
using marv::rate::RateConfig;
using marv::rate::RateLoop;
using V3 = prim::Vec3<float>;

constexpr std::size_t kA = rate::kTorqueAxes;
// Unit roundoff of binary32 with round to nearest: u = 2^-24. Half an ulp of x is at most u |x|.
constexpr double kU = 0x1p-24;
// Libm accuracy assumed for expf: 2 ulp, an ulp being at most 2 u |v|. INFERRED (glibc documents 1 to 2 ulp).
constexpr double kExpErr = 4.0 * kU;

// Scenario test value: the design period, 1 ms: a whole number of microseconds, so every stamp is exact.
constexpr float kPeriodS = 0.001F;
constexpr std::uint64_t kPeriodUs = 1000;
constexpr std::uint64_t kT0Us = 5000;
// dt as the loop forms it: float(dt_us) / float(1e6), one rounding.
constexpr float kDt = static_cast<float>(kPeriodUs) / static_cast<float>(prim::kMicrosecondsPerSecond);
constexpr double kDtD = static_cast<double>(kDt);

// Product-independent geometry and mixer: the product parameter set, as the L4 suite does.
struct Fixture {
  mixer::MixerConfig<float> mixer;
  RateConfig<float> geometry;
};

const Fixture* fixture() {
  static const Fixture* built = []() -> const Fixture* {
    if (!gyro_chain::test::init_params()) {
      return nullptr;
    }
    static Fixture f;
    f.mixer = mixer::from_params();
    f.geometry = rate::from_params();
    f.geometry.period = kPeriodS;
    return &f;
  }();
  return built;
}

#define REQUIRE_FIXTURE()                     \
  const Fixture* const fp = fixture();        \
  ASSERT_NE(fp, nullptr) << "params_init";    \
  const Fixture& fx = *fp

// Scenario test values for the PID part of the scripted-input comparison: distinct per axis so an axis mix-up shows.
// Not product gains.
RateConfig<float> pid_config(const Fixture& fx) {
  RateConfig<float> c = fx.geometry;
  const std::array<float, kA> kp{2.0F, 3.0F, 5.0F};
  const std::array<float, kA> ki{100.0F, 80.0F, 60.0F};
  const std::array<float, kA> kd{0.004F, 0.006F, 0.008F};
  const std::array<float, kA> tau{0.01F, 0.02F, 0.03F};
  for (std::size_t a = 0; a < kA; ++a) {
    c.kp[a] = kp[a];
    c.ki[a] = ki[a];
    c.kd[a] = kd[a];
    c.tau_ref[a] = tau[a];
  }
  return c;
}

// A loop with its stamp clock: the first execution at kT0Us, then every kPeriodUs.
struct Rig {
  RateLoop<float> loop;
  std::uint64_t t = kT0Us;
  bool first = true;

  void init(const Fixture& fx, const RateConfig<float>& cfg) {
    loop.init(cfg, fx.mixer);
    t = kT0Us;
    first = true;
  }
  void restart() {
    loop.reset();
    t = kT0Us;
    first = true;
  }
  V3 step(const V3& y, const V3& sp) {
    if (!first) {
      t += kPeriodUs;
    }
    first = false;
    ImuSample s{};
    s.t_us = t;
    s.gyro_rad_s = y;
    s.flags = imu_flag(ImuFlag::GyroValid);
    const rate::RateOutput<float> o = loop.execute(s, sp);
    EXPECT_FALSE(o.fault_active);
    return o.torque;
  }
};

bool bits_equal(const V3& a, const V3& b) { return std::memcmp(a.e, b.e, sizeof(a.e)) == 0; }

// ---- Today's law, a test-local copy of the L4 expression (rate_loop.hpp before decision 0014) ---------------------
struct TodaysLaw {
  RateConfig<float> cfg;
  V3 r, e, y_prev, integral;
  bool seed = true;

  explicit TodaysLaw(const RateConfig<float>& c) : cfg(c) {}

  V3 step(const V3& y, const V3& sp) {
    V3 u;
    if (seed) {
      r = y;
      e = V3();
      integral = V3();
      seed = false;
    } else {
      for (std::size_t a = 0; a < kA; ++a) {
        const float alpha = 1.0F - std::exp(-kDt / cfg.tau_ref[a]);
        r[a] = r[a] + alpha * (sp[a] - r[a]);
        const float ea = r[a] - y[a];
        integral[a] = integral[a] + cfg.ki[a] * e[a] * kDt;
        const float d = -cfg.kd[a] * (y[a] - y_prev[a]) / kDt;
        u[a] = cfg.kp[a] * ea + integral[a] + d;
        e[a] = ea;
      }
    }
    y_prev = y;
    return u;
  }
};

constexpr std::size_t kScript = 3000;  // scenario test value: ticks of the scripted input (3 s)

// A deterministic input: tones at scenario test frequencies (rad/tick) and amplitudes (rad/s), so y, sp and their
// derivatives are all nonzero on every axis.
V3 script_y(std::size_t n) {
  const double t = static_cast<double>(n);
  return V3(static_cast<float>(3.0 * std::sin(0.011 * t) + 0.5 * std::sin(0.31 * t)),
            static_cast<float>(-2.0 * std::sin(0.017 * t + 0.4) + 0.25 * std::sin(0.77 * t)),
            static_cast<float>(1.5 * std::sin(0.007 * t + 1.0) + 0.1 * std::sin(1.3 * t)));
}

V3 script_sp(std::size_t n) {
  const double t = static_cast<double>(n);
  return V3(static_cast<float>(4.0 * std::sin(0.013 * t + 0.2)), static_cast<float>(-3.0 * std::sin(0.009 * t)),
            static_cast<float>(2.0 * std::sin(0.021 * t + 0.9)));
}

bool matches_todays_law(const Fixture& fx, const RateConfig<float>& cfg) {
  Rig rig;
  rig.init(fx, cfg);
  TodaysLaw law(cfg);
  bool same = true;
  for (std::size_t n = 0; n < kScript; ++n) {
    same = same && bits_equal(rig.step(script_y(n), script_sp(n)), law.step(script_y(n), script_sp(n)));
  }
  return same;
}

TEST(L6DTerm, DefaultsAreOffAndTheLoopIsTodaysLawBitForBit) {
  REQUIRE_FIXTURE();
  const RateConfig<float> cfg = pid_config(fx);
  EXPECT_EQ(cfg.d_filter_tau[0], 0.0F);
  EXPECT_EQ(cfg.inertia[0], 0.0F);
  EXPECT_EQ(cfg.motor_tau, 0.0F);
  EXPECT_EQ(cfg.ff_filter_tau, 0.0F);
  EXPECT_TRUE(matches_todays_law(fx, cfg));
}

// All J = 0 skips the feed-forward whatever tau_m and T_ff are: the output is the PID law's exactly.
TEST(L6DTerm, ZeroInertiaIsBitIdenticalWhateverTheLagAndFilterAre) {
  REQUIRE_FIXTURE();
  RateConfig<float> cfg = pid_config(fx);
  cfg.motor_tau = 0.05F;      // scenario test value, a nonzero tau_m
  cfg.ff_filter_tau = 0.01F;  // scenario test value, a nonzero T_ff
  EXPECT_TRUE(matches_todays_law(fx, cfg));
}

// The control of the two checks above: the same comparison fails as soon as the law is not today's (a D filter on, a
// feed-forward on), so "bit-identical" is a statement the check can break.
TEST(L6DTerm, ControlFilterOrFeedforwardOnBreaksTheBitIdentity) {
  REQUIRE_FIXTURE();
  RateConfig<float> d_on = pid_config(fx);
  d_on.d_filter_tau[1] = 0.01F;
  EXPECT_FALSE(matches_todays_law(fx, d_on));
  RateConfig<float> ff_on = pid_config(fx);
  ff_on.inertia = V3(0.01F, 0.02F, 0.03F);
  EXPECT_FALSE(matches_todays_law(fx, ff_on));
}

// ---- D low-pass: the step response of the discrete first-order filter ---------------------------------------------
// Input y_n = n q on every axis, q = 2^-10 rad/s per tick (a scenario test value; a power of two, so every y_n and every
// difference is exact in float). The raw derivative term is then the constant D0 = -kd q / dt from n = 1 on (n = 0 is
// the seeding execution, D = 0). With kp = ki = 0 the output is the D term alone: u_n = Df_n with Df_0 = 0 and
//   Df_n = Df_(n-1) + alpha (D0 - Df_(n-1))  =>  Df_n = D0 (1 - (1 - alpha)^n),  alpha = 1 - exp(-dt / T_f),
// and T_f = 0 gives u_n = D0 for n >= 1.
// Tolerance, derived. D0 in float has three roundings (the product, the division, and dt itself): 3u |D0|. alpha in
// float: the quotient dt / T_f (u x), expf (kExpErr), the subtraction (u alpha): delta_alpha = (x + 4 + alpha) u; Df_n
// depends on alpha with slope D0 n (1 - alpha)^(n-1). Each filter step adds at most 3u |D0| (the difference, the
// product, the sum), and the recursion contracts by (1 - alpha), so the sum of the rounding is at most 3u |D0| / alpha.
constexpr float kDq = 0x1p-10F;
constexpr float kKdTest = 0.004F;  // scenario test value, the D gain of this test
constexpr std::size_t kStepTicks = 400;

double d0_exact() { return -static_cast<double>(kKdTest) * static_cast<double>(kDq) / kDtD; }

double closed_form(double tf, std::size_t n) {
  if (n == 0) {
    return 0.0;
  }
  if (tf == 0.0) {
    return d0_exact();
  }
  const double alpha = 1.0 - std::exp(-kDtD / tf);
  return d0_exact() * (1.0 - std::pow(1.0 - alpha, static_cast<double>(n)));
}

double tolerance(double tf, std::size_t n) {
  const double d0 = std::fabs(d0_exact());
  if (n == 0) {
    return 0.0;
  }
  if (tf == 0.0) {
    return 3.0 * kU * d0;
  }
  const double x = kDtD / tf;
  const double alpha = 1.0 - std::exp(-x);
  const double d_alpha = (x + 4.0 + alpha) * kU;
  return d0 * (static_cast<double>(n) * std::pow(1.0 - alpha, static_cast<double>(n) - 1.0) * d_alpha +
               3.0 * kU / alpha + 3.0 * kU);
}

RateConfig<float> d_only_config(const Fixture& fx, const std::array<float, kA>& tf) {
  RateConfig<float> c = fx.geometry;
  for (std::size_t a = 0; a < kA; ++a) {
    c.kp[a] = 0.0F;
    c.ki[a] = 0.0F;
    c.kd[a] = kKdTest;
    c.tau_ref[a] = 0.01F;  // scenario test value, irrelevant with kp = ki = 0
    c.d_filter_tau[a] = tf[a];
  }
  return c;
}

// Largest |u_n - closed form(T_ref)| / tolerance(T_ref) over the run and the axes, the filter configured with `tf` and
// the closed form evaluated at `tf_ref` (equal unless a control).
double worst_ratio(const Fixture& fx, const std::array<float, kA>& tf, const std::array<double, kA>& tf_ref) {
  Rig rig;
  rig.init(fx, d_only_config(fx, tf));
  double worst = 0;
  for (std::size_t n = 0; n < kStepTicks; ++n) {
    const float y = static_cast<float>(n) * kDq;
    const V3 u = rig.step(V3(y, y, y), V3());
    for (std::size_t a = 0; a < kA; ++a) {
      const double err = std::fabs(static_cast<double>(u[a]) - closed_form(tf_ref[a], n));
      const double tol = tolerance(tf_ref[a], n);
      worst = std::max(worst, tol > 0.0 ? err / tol : (err == 0.0 ? 0.0 : std::numeric_limits<double>::infinity()));
    }
  }
  return worst;
}

// One axis unfiltered (yaw, T_f = 0) beside two filtered axes: the per-axis constants are used per axis. The two
// filtered time constants are scenario test values, 20 and 40 ms.
constexpr std::array<float, kA> kTf{0.02F, 0.04F, 0.0F};

TEST(L6DTerm, DFilterStepResponseMatchesTheDiscreteClosedForm) {
  REQUIRE_FIXTURE();
  const double w = worst_ratio(fx, kTf, {0.02, 0.04, 0.0});
  EXPECT_LE(w, 1.0);
}

TEST(L6DTerm, DFilterUnfilteredAxisIsTheFullStepAtOnce) {
  REQUIRE_FIXTURE();
  Rig rig;
  rig.init(fx, d_only_config(fx, kTf));
  const V3 u0 = rig.step(V3(), V3());
  EXPECT_EQ(u0[0], 0.0F);
  const V3 u1 = rig.step(V3(kDq, kDq, kDq), V3());
  // Yaw (T_f = 0): the whole D0 on the first step. Roll and pitch: alpha D0, strictly between 0 and D0.
  EXPECT_EQ(u1[2], -kKdTest * kDq / kDt);
  EXPECT_LT(std::fabs(u1[0]), std::fabs(u1[2]));
  EXPECT_GT(std::fabs(u1[0]), 0.0F);
  EXPECT_LT(std::fabs(u1[1]), std::fabs(u1[0]));  // the longer time constant lags more
}

// Negative controls: the closed form of T_f x 1.1 (and /1.1) is not met by the loop configured with T_f.
TEST(L6DTerm, ControlTimeConstantOffByTenPercentFailsTheClosedForm) {
  REQUIRE_FIXTURE();
  EXPECT_GT(worst_ratio(fx, kTf, {0.02 * 1.1, 0.04 * 1.1, 0.0}), 1.0);
  EXPECT_GT(worst_ratio(fx, kTf, {0.02 / 1.1, 0.04 / 1.1, 0.0}), 1.0);
  // One axis at a time: each axis's constant is checked.
  EXPECT_GT(worst_ratio(fx, kTf, {0.02 * 1.1, 0.04, 0.0}), 1.0);
  EXPECT_GT(worst_ratio(fx, kTf, {0.02, 0.04 * 1.1, 0.0}), 1.0);
  // The unfiltered law's closed form against the filtered loop fails too (the filter does something).
  EXPECT_GT(worst_ratio(fx, kTf, {0.0, 0.0, 0.0}), 1.0);
}

// The filter state is reset with the others: after reset() the run repeats bit for bit.
TEST(L6DTerm, ResetClearsTheFilterStates) {
  REQUIRE_FIXTURE();
  RateConfig<float> cfg = pid_config(fx);
  cfg.d_filter_tau = V3(0.02F, 0.04F, 0.06F);
  cfg.inertia = V3(0.03F, 0.02F, 0.05F);
  cfg.motor_tau = 0.03F;
  cfg.ff_filter_tau = 0.01F;
  Rig rig;
  rig.init(fx, cfg);
  std::vector<V3> first;
  for (std::size_t n = 0; n < 200; ++n) {
    first.push_back(rig.step(script_y(n), script_sp(n)));
  }
  rig.restart();
  for (std::size_t n = 0; n < 200; ++n) {
    EXPECT_TRUE(bits_equal(rig.step(script_y(n), script_sp(n)), first[n])) << n;
  }
}

// ---- Feed-forward: known answer, lag term, controls -----------------------------------------------------------
// Known answer. omega = (1, -2, 3) rad/s, J = (1/2, 1/4, 1/8) kg m^2 (scenario test values: dyadic, so every product and
// sum below is exact in float). J omega = (1/2, -1/2, 3/8); omega x (J omega) =
//   (w_y (J w)_z - w_z (J w)_y, w_z (J w)_x - w_x (J w)_z, w_x (J w)_y - w_y (J w)_x)
//   = (-2 3/8 - 3 (-1/2), 3 1/2 - 1 3/8, 1 (-1/2) - (-2) 1/2) = (3/4, 9/8, 1/2).
RateConfig<float> ff_only_config(const Fixture& fx, const V3& inertia, float tau_m, float t_ff) {
  RateConfig<float> c = fx.geometry;
  for (std::size_t a = 0; a < kA; ++a) {
    c.kp[a] = 0.0F;
    c.ki[a] = 0.0F;
    c.kd[a] = 0.0F;
    c.tau_ref[a] = 0.01F;  // scenario test value, irrelevant with every gain 0
  }
  c.inertia = inertia;
  c.motor_tau = tau_m;
  c.ff_filter_tau = t_ff;
  return c;
}

TEST(L6DTerm, FeedforwardKnownAnswerAtConstantRate) {
  REQUIRE_FIXTURE();
  const V3 omega(1.0F, -2.0F, 3.0F);
  const V3 expected(0.75F, 1.125F, 0.5F);
  for (const float tau_m : {0.0F, 0.05F}) {  // at constant omega the lag term is 0 whatever tau_m is
    for (const float t_ff : {0.0F, 0.01F}) {
      Rig rig;
      rig.init(fx, ff_only_config(fx, V3(0.5F, 0.25F, 0.125F), tau_m, t_ff));
      EXPECT_TRUE(bits_equal(rig.step(omega, V3()), V3())) << "seeding execution outputs 0";
      for (std::size_t n = 0; n < 5; ++n) {
        EXPECT_TRUE(bits_equal(rig.step(omega, V3()), expected)) << tau_m << " " << t_ff << " " << n;
      }
    }
  }
}

TEST(L6DTerm, ControlKnownAnswerBreaksWithPermutedOrFlippedInertia) {
  REQUIRE_FIXTURE();
  const V3 omega(1.0F, -2.0F, 3.0F);
  const V3 expected(0.75F, 1.125F, 0.5F);
  const std::array<V3, 3> permuted{V3(0.25F, 0.125F, 0.5F), V3(0.125F, 0.5F, 0.25F), V3(0.25F, 0.5F, 0.125F)};
  for (const V3& j : permuted) {
    Rig rig;
    rig.init(fx, ff_only_config(fx, j, 0.0F, 0.0F));
    (void)rig.step(omega, V3());
    EXPECT_FALSE(bits_equal(rig.step(omega, V3()), expected));
  }
  // A sign flip of the term: the negated expectation is not met (and the term is not zero).
  Rig rig;
  rig.init(fx, ff_only_config(fx, V3(0.5F, 0.25F, 0.125F), 0.0F, 0.0F));
  (void)rig.step(omega, V3());
  const V3 u = rig.step(omega, V3());
  EXPECT_FALSE(bits_equal(u, V3(-0.75F, -1.125F, -0.5F)));
}

// The lag term against a double reference. Ramp: omega_n = omega0 + n s, s = (1, -2, 3) 2^-6 rad/s per tick (a
// scenario test value; multiples of 2^-6, so exact in float), i.e. an angular acceleration of about 16 to 47 rad/s^2 at
// dt = 1 ms. J = (0.03, 0.02, 0.05) kg m^2, tau_m = 0.03 s: scenario test values, distinct entries so a permutation
// changes the term.
constexpr std::size_t kRamp = 200;
const V3 kOmega0(4.0F, -2.0F, 3.0F);
const V3 kSlope(0x1p-6F, -0x1p-5F, 3.0F * 0x1p-6F);
const V3 kJ(0.03F, 0.02F, 0.05F);
constexpr float kTauM = 0.03F;

V3 ramp(std::size_t n) {
  const float k = static_cast<float>(n);
  return V3(kOmega0[0] + k * kSlope[0], kOmega0[1] + k * kSlope[1], kOmega0[2] + k * kSlope[2]);
}

struct Ref {
  std::vector<std::array<double, kA>> u;
  std::vector<double> tol;  // derived, per tick, the largest over the axes
};

std::array<double, kA> cross_jw(const std::array<double, kA>& w, const std::array<double, kA>& j) {
  const double jw0 = j[0] * w[0];
  const double jw1 = j[1] * w[1];
  const double jw2 = j[2] * w[2];
  return {w[1] * jw2 - w[2] * jw1, w[2] * jw0 - w[0] * jw2, w[0] * jw1 - w[1] * jw0};
}

// The law in double: u_0 = 0; g_n = w_n x (J w_n); raw_n = (g_n - g_(n-1)) / dt; gdot_n the first-order low-pass of raw
// with alpha = 1 - exp(-dt / T_ff) (T_ff = 0: raw), gdot_0 = 0; u_n = g_n + tau_m gdot_n.
// Tolerance (derived, first order in u). g in float: (J w)_c one rounding, each of the two products 2 u |p| (its own
// rounding and the rounding in J w), the subtraction u |g|: |dg| <= 6 u P, P = J_max w_max^2 an upper bound of a
// product; take 7 u P (the neglected second-order terms are below one more u P). raw: (dg_n + dg_(n-1)) / dt, the
// subtraction and the division u |raw| each: draw = 2 dg / dt + 2 u |raw|_max. The filter adds per step at most
// 3 u |raw|_max (difference, product, sum) and contracts by (1 - alpha): the error of gdot is at most
// draw + 3 u |raw|_max / alpha (T_ff = 0: draw). alpha's own error enters through (raw - gdot), at most
// delta_alpha |raw|_max, delta_alpha = (x + 4 + alpha) u as in the D test. The output: u = u + (g + tau_m gdot) has
// three roundings, 3 u (|g| + tau_m |gdot|_max): |du| <= dg + tau_m dgdot + that.
Ref reference(const std::array<double, kA>& j, double tau_m, double t_ff, double sign) {
  Ref r;
  std::array<double, kA> g_prev{};
  std::array<double, kA> gdot{};
  double w_max = 0;
  double raw_max = 0;
  double g_max = 0;
  std::vector<std::array<double, kA>> gs;
  std::vector<std::array<double, kA>> raws;
  for (std::size_t n = 0; n < kRamp; ++n) {
    const V3 wf = ramp(n);
    const std::array<double, kA> w{wf[0], wf[1], wf[2]};
    std::array<double, kA> g = cross_jw(w, j);
    std::array<double, kA> uu{};
    std::array<double, kA> raw{};
    if (n > 0) {
      for (std::size_t a = 0; a < kA; ++a) {
        raw[a] = (g[a] - g_prev[a]) / kDtD;
        gdot[a] = t_ff > 0.0 ? gdot[a] + (1.0 - std::exp(-kDtD / t_ff)) * (raw[a] - gdot[a]) : raw[a];
        uu[a] = sign * (g[a] + tau_m * gdot[a]);
      }
    }
    for (std::size_t a = 0; a < kA; ++a) {
      w_max = std::max(w_max, std::fabs(w[a]));
      raw_max = std::max(raw_max, std::fabs(raw[a]));
      g_max = std::max(g_max, std::fabs(g[a]));
    }
    r.u.push_back(uu);
    g_prev = g;
  }
  const double j_max = std::max({j[0], j[1], j[2]});
  const double dg = 7.0 * kU * j_max * w_max * w_max;
  const double draw = 2.0 * dg / kDtD + 2.0 * kU * raw_max;
  double dgdot = draw;
  if (t_ff > 0.0) {
    const double x = kDtD / t_ff;
    const double alpha = 1.0 - std::exp(-x);
    dgdot = draw + 3.0 * kU * raw_max / alpha + (x + 4.0 + alpha) * kU * raw_max;
  }
  const double tol = dg + tau_m * dgdot + 3.0 * kU * (g_max + tau_m * (raw_max + dgdot));
  r.tol.assign(kRamp, tol);
  r.tol[0] = 0.0;
  return r;
}

double worst_vs_reference(const Fixture& fx, const V3& j_cfg, float tau_cfg, float t_ff, const Ref& ref) {
  Rig rig;
  rig.init(fx, ff_only_config(fx, j_cfg, tau_cfg, t_ff));
  double worst = 0;
  for (std::size_t n = 0; n < kRamp; ++n) {
    const V3 u = rig.step(ramp(n), V3());
    for (std::size_t a = 0; a < kA; ++a) {
      const double err = std::fabs(static_cast<double>(u[a]) - ref.u[n][a]);
      worst = std::max(worst, ref.tol[n] > 0.0 ? err / ref.tol[n]
                                               : (err == 0.0 ? 0.0 : std::numeric_limits<double>::infinity()));
    }
  }
  return worst;
}

std::array<double, kA> as_double(const V3& v) { return {v[0], v[1], v[2]}; }

// T_ff scenario test values: 0 (unfiltered) and 10 ms.
constexpr std::array<float, 2> kTffs{0.0F, 0.01F};

TEST(L6DTerm, FeedforwardLagTermMatchesTheDoubleReferenceOnARamp) {
  REQUIRE_FIXTURE();
  for (const float t_ff : kTffs) {
    SCOPED_TRACE(t_ff);
    const Ref ref = reference(as_double(kJ), static_cast<double>(kTauM), static_cast<double>(t_ff), 1.0);
    EXPECT_LE(worst_vs_reference(fx, kJ, kTauM, t_ff, ref), 1.0);
    // The lag term is a real part of the output: the plain term (tau_m = 0) is far outside the tolerance.
    EXPECT_GT(worst_vs_reference(fx, kJ, 0.0F, t_ff, ref), 1.0);
  }
}

TEST(L6DTerm, ControlPermutedAxesFlippedSignAndTauTimesOnePointOneFailTheReference) {
  REQUIRE_FIXTURE();
  for (const float t_ff : kTffs) {
    SCOPED_TRACE(t_ff);
    const Ref ref = reference(as_double(kJ), static_cast<double>(kTauM), static_cast<double>(t_ff), 1.0);
    // The three non-trivial cyclic and swap permutations of the inertia axes (the reference keeps the true J).
    EXPECT_GT(worst_vs_reference(fx, V3(kJ[1], kJ[2], kJ[0]), kTauM, t_ff, ref), 1.0);
    EXPECT_GT(worst_vs_reference(fx, V3(kJ[2], kJ[0], kJ[1]), kTauM, t_ff, ref), 1.0);
    EXPECT_GT(worst_vs_reference(fx, V3(kJ[1], kJ[0], kJ[2]), kTauM, t_ff, ref), 1.0);
    // Sign flipped: the reference with the term negated.
    const Ref flipped = reference(as_double(kJ), static_cast<double>(kTauM), static_cast<double>(t_ff), -1.0);
    EXPECT_GT(worst_vs_reference(fx, kJ, kTauM, t_ff, flipped), 1.0);
    // tau_m x 1.1 (and / 1.1).
    EXPECT_GT(worst_vs_reference(fx, kJ, kTauM * 1.1F, t_ff, ref), 1.0);
    EXPECT_GT(worst_vs_reference(fx, kJ, kTauM / 1.1F, t_ff, ref), 1.0);
  }
}

// T_ff x 1.1 (filtered case) is also outside the reference: the filter constant is checked.
TEST(L6DTerm, ControlFeedforwardFilterConstantOffByTenPercentFailsTheReference) {
  REQUIRE_FIXTURE();
  const Ref ref = reference(as_double(kJ), static_cast<double>(kTauM), 0.01, 1.0);
  EXPECT_LE(worst_vs_reference(fx, kJ, kTauM, 0.01F, ref), 1.0);
  EXPECT_GT(worst_vs_reference(fx, kJ, kTauM, 0.011F, ref), 1.0);
  EXPECT_GT(worst_vs_reference(fx, kJ, kTauM, 0.009F, ref), 1.0);
}

// ---- validate -----------------------------------------------------------------------------------------------
TEST(L6DTerm, ValidateAcceptsZeroAndRejectsNegativeAndNonFinite) {
  REQUIRE_FIXTURE();
  RateConfig<float> ok = pid_config(fx);
  ok.d_filter_tau = V3(0.0F, 0.02F, 0.04F);
  ok.inertia = V3(0.0F, 0.02F, 0.04F);
  ok.motor_tau = 0.0F;
  ok.ff_filter_tau = 0.0F;
  ASSERT_EQ(rate::validate(ok), rate::ConfigError::None);
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  const auto with = [&](auto mutate) {
    RateConfig<float> c = ok;
    mutate(c);
    return rate::validate(c);
  };
  for (std::size_t a = 0; a < kA; ++a) {
    EXPECT_EQ(with([&](auto& c) { c.d_filter_tau[a] = -0.01F; }), rate::ConfigError::DFilter) << a;
    EXPECT_EQ(with([&](auto& c) { c.d_filter_tau[a] = nan; }), rate::ConfigError::NonFinite) << a;
    EXPECT_EQ(with([&](auto& c) { c.d_filter_tau[a] = inf; }), rate::ConfigError::NonFinite) << a;
    EXPECT_EQ(with([&](auto& c) { c.inertia[a] = -0.01F; }), rate::ConfigError::Feedforward) << a;
    EXPECT_EQ(with([&](auto& c) { c.inertia[a] = nan; }), rate::ConfigError::NonFinite) << a;
    EXPECT_EQ(with([&](auto& c) { c.inertia[a] = inf; }), rate::ConfigError::NonFinite) << a;
  }
  EXPECT_EQ(with([&](auto& c) { c.motor_tau = -0.01F; }), rate::ConfigError::Feedforward);
  EXPECT_EQ(with([&](auto& c) { c.motor_tau = nan; }), rate::ConfigError::NonFinite);
  EXPECT_EQ(with([&](auto& c) { c.motor_tau = inf; }), rate::ConfigError::NonFinite);
  EXPECT_EQ(with([&](auto& c) { c.ff_filter_tau = -0.01F; }), rate::ConfigError::Feedforward);
  EXPECT_EQ(with([&](auto& c) { c.ff_filter_tau = nan; }), rate::ConfigError::NonFinite);
  EXPECT_EQ(with([&](auto& c) { c.ff_filter_tau = inf; }), rate::ConfigError::NonFinite);
  // From the product parameter set the new fields are 0 (not yet parameters): the product loop is today's.
  EXPECT_EQ(rate::from_params().d_filter_tau[0], 0.0F);
  EXPECT_EQ(rate::from_params().inertia[2], 0.0F);
}

}  // namespace
