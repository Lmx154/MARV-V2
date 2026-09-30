// The L4 rate loop (quad spec 4 L4): the law against hand-computed values and a double-precision oracle with dt jitter,
// the reference-model prefilter and its bumpless seeding, D = 0 at a first execution, clamping anti-windup (each of
// its three conditions, the rounding case with synthetic and with real L3 allocations), the fault handling, the dt
// check and the configuration validation.
//
// Numbers in this file are one of: derived (the rule is stated), or a "scenario test value" chosen for the scenario
// and named with its reason. The card supplies the mixer and the rotor geometry; nothing here tunes to it.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <random>
#include <vector>

#include <marv/mixer/mixer.hpp>
#include <marv/params/param.hpp>
#include <marv/prim/constants.hpp>
#include <marv/prim/mat.hpp>
#include <marv/prim/vec.hpp>
#include <marv/rate/rate_loop.hpp>
#include <marv/types/imu_sample.hpp>

namespace {

using namespace marv;
using marv::mixer::Allocation;
using marv::mixer::kMotors;
using marv::mixer::MixerConfig;
using marv::mixer::Request;
using marv::rate::RateConfig;
using marv::rate::RateLoop;
using marv::rate::RateOutput;

constexpr std::size_t kN = kMotors;
constexpr std::size_t kA = rate::kTorqueAxes;
constexpr float kEps = std::numeric_limits<float>::epsilon();
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();

// Scenario test value: the design period T. One millisecond, a round value with an exactly representable microsecond
// count, and it makes dt = T a whole number of microseconds so the +-1 us jitter cases are exact.
constexpr float kPeriodS = 0.001F;
constexpr std::uint64_t kPeriodUs = 1000;
// Scenario test value: the first sample stamp, nonzero so a stamp of 0 is not mistaken for "no stamp".
constexpr std::uint64_t kT0Us = 5000;

// Scenario test value: comparison tolerance of a float result against an exact value, kTol (1 + |expected|). About
// ten float operations and one exp() at one ulp each stay below 10 eps; 256 eps leaves a factor of 25.
constexpr double kTol = 256.0 * static_cast<double>(kEps);

bool near(double actual, double expected) {
  return std::abs(actual - expected) <= kTol * (1.0 + std::abs(expected));
}

// ---- the card ------------------------------------------------------------------------------------------------

struct Card {
  MixerConfig<float> mixer;
  RateConfig<float> geometry;  // rotor geometry only; the gains are set per test
  double lo = 0;
  double hi = 0;
  double thrust_mid = 0;   // collective at which every motor is centred
  double pitch_scale = 0;  // torque whose largest |M[i, pitch]| * torque equals half the per-motor thrust spread
};

const Card* card() {
  static const Card* built = []() -> const Card* {
    if (!params_init(param_defaults())) {
      return nullptr;
    }
    static Card k;
    k.mixer = mixer::from_params();
    const std::array<ParamId, kN> xs{ParamId::rotor_position_m1_x, ParamId::rotor_position_m2_x,
                                     ParamId::rotor_position_m3_x, ParamId::rotor_position_m4_x};
    const std::array<ParamId, kN> ys{ParamId::rotor_position_m1_y, ParamId::rotor_position_m2_y,
                                     ParamId::rotor_position_m3_y, ParamId::rotor_position_m4_y};
    const std::array<ParamId, kN> ss{ParamId::rotor_yaw_sign_m1, ParamId::rotor_yaw_sign_m2,
                                     ParamId::rotor_yaw_sign_m3, ParamId::rotor_yaw_sign_m4};
    for (std::size_t i = 0; i < kN; ++i) {
      k.geometry.rotor_x[i] = param_get(xs[i]).value.f32;
      k.geometry.rotor_y[i] = param_get(ys[i]).value.f32;
      k.geometry.yaw_sign[i] = static_cast<float>(param_get(ss[i]).value.i32);
    }
    k.geometry.torque_ratio = param_value<ParamId::rotor_torque_ratio>();
    k.lo = static_cast<double>(mixer::f_min(k.mixer));
    k.hi = static_cast<double>(mixer::f_max(k.mixer));
    k.thrust_mid = static_cast<double>(kN) * (k.lo + k.hi) / 2;
    double worst = 0;
    for (std::size_t i = 0; i < kN; ++i) {
      worst = std::max(worst, std::abs(static_cast<double>(k.mixer.m(i, mixer::kPitch))));
    }
    k.pitch_scale = (k.hi - k.lo) / (2 * worst);
    return &k;
  }();
  return built;
}

#define REQUIRE_CARD()                     \
  const Card* const cp = card();           \
  ASSERT_NE(cp, nullptr) << "params_init"; \
  const Card& c = *cp

using V3 = prim::Vec3<float>;

V3 v3(double x, double y, double z) {
  return V3(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
}

V3 all(double v) { return v3(v, v, v); }

struct Gains {
  double kp = 0;
  double ki = 0;
  double kd = 0;
  double tau = 0;  // reference-model time constant, s
};

RateConfig<float> config(const Card& c, const std::array<Gains, kA>& g) {
  RateConfig<float> r = c.geometry;
  for (std::size_t a = 0; a < kA; ++a) {
    r.kp[a] = static_cast<float>(g[a].kp);
    r.ki[a] = static_cast<float>(g[a].ki);
    r.kd[a] = static_cast<float>(g[a].kd);
    r.tau_ref[a] = static_cast<float>(g[a].tau);
  }
  r.period = kPeriodS;
  return r;
}

RateConfig<float> config(const Card& c, const Gains& g) { return config(c, std::array<Gains, kA>{g, g, g}); }

// tau = T / ln 2 makes the prefilter factor 1 - exp(-T / tau) exactly one half.
double tau_half() { return static_cast<double>(kPeriodS) / std::log(2.0); }

// The hand-table gains, identical on all three axes: with dt = 1 ms, D = -kd dy / dt = -4 dy.
// Scenario test values: small integers that keep the hand arithmetic exact.
constexpr double kHandKp = 2;
constexpr double kHandKi = 100;
constexpr double kHandKd = 0.004;

Gains hand_gains() { return Gains{kHandKp, kHandKi, kHandKd, tau_half()}; }

// A loop plus its sample clock. step() advances the stamp by T + jitter, except for the first execution after init.
struct Rig {
  RateLoop<float> loop;
  std::uint64_t t = kT0Us;
  bool first = true;

  void init(const Card& c, const RateConfig<float>& cfg) {
    loop.init(cfg, c.mixer);
    t = kT0Us;
    first = true;
  }

  static ImuSample sample(std::uint64_t t_us, const V3& gyro, std::uint32_t flags) {
    ImuSample s{};
    s.t_us = t_us;
    s.gyro_rad_s = gyro;
    s.flags = flags;
    return s;
  }

  RateOutput<float> step(const V3& y, const V3& sp, std::int64_t jitter_us = 0,
                         std::uint32_t flags = imu_flag(ImuFlag::GyroValid)) {
    if (!first) {
      t = static_cast<std::uint64_t>(static_cast<std::int64_t>(t) + static_cast<std::int64_t>(kPeriodUs) + jitter_us);
    }
    first = false;
    return loop.execute(sample(t, y, flags), sp);
  }
};

// ---- 1. the law ----------------------------------------------------------------------------------------------

struct HandStep {
  double y;
  double sp;
  double u;
};

// Hand-computed with dt = T exactly, alpha = 1/2, kp = 2, ki = 100, kd = 0.004 (see the module header for the law):
//   n=0 seed: r=0, u=0.
//   n=1 y=1 sp=8: r=4 e=3 I=0 (e0=0) D=-4(1-0)=-4                u = 6 + 0 - 4      = 2
//   n=2 y=2 sp=8: r=6 e=4 I=0+100*3*0.001=0.3 D=-4(2-1)=-4       u = 8 + 0.3 - 4    = 4.3
//   n=3 y=2 sp=8: r=7 e=5 I=0.3+100*4*0.001=0.7 D=0              u = 10 + 0.7       = 10.7
//   n=4 y=4 sp=0: r=3.5 e=-0.5 I=0.7+100*5*0.001=1.2 D=-4*2=-8   u = -1 + 1.2 - 8   = -7.8
constexpr std::array<HandStep, 5> kHand{{{0, 0, 0}, {1, 8, 2}, {2, 8, 4.3}, {2, 8, 10.7}, {4, 0, -7.8}}};

// Runs the hand sequence; axis 0 as tabulated, axis 1 scaled by 2, axis 2 scaled by -1 (the law is linear in y and
// sp with equal gains and a seeded prefilter). Returns the outputs.
std::vector<V3> run_hand(const Card& c, const RateConfig<float>& cfg) {
  Rig rig;
  rig.init(c, cfg);
  std::vector<V3> out;
  for (const HandStep& s : kHand) {
    out.push_back(rig.step(v3(s.y, 2 * s.y, -s.y), v3(s.sp, 2 * s.sp, -s.sp)).torque);
  }
  return out;
}

bool hand_matches(const std::vector<V3>& got, double scale_error = 0) {
  bool ok = true;
  for (std::size_t n = 0; n < kHand.size(); ++n) {
    const double want = kHand[n].u + scale_error;
    ok = ok && near(static_cast<double>(got[n][0]), want) && near(static_cast<double>(got[n][1]), 2 * want) &&
         near(static_cast<double>(got[n][2]), -want);
  }
  return ok;
}

TEST(L4RateLaw, MatchesTheHandComputedTableOnAllThreeAxes) {
  REQUIRE_CARD();
  const std::vector<V3> got = run_hand(c, config(c, hand_gains()));
  EXPECT_TRUE(hand_matches(got));
  EXPECT_EQ(got[0][0], 0.0F);  // the seed execution
}

TEST(L4RateLaw, NegativeControlsPlantedChangesBreakTheHandTable) {
  REQUIRE_CARD();
  EXPECT_TRUE(hand_matches(run_hand(c, config(c, hand_gains()))));  // the baseline passes
  const auto broken = [&](auto mutate) {
    Gains g = hand_gains();
    mutate(g);
    return hand_matches(run_hand(c, config(c, g)));
  };
  EXPECT_FALSE(broken([](Gains& g) { g.kd = 0; })) << "the D path is exercised";
  EXPECT_FALSE(broken([](Gains& g) { g.kd = 2 * g.kd; }));
  EXPECT_FALSE(broken([](Gains& g) { g.ki = 0; })) << "the I path is exercised";
  EXPECT_FALSE(broken([](Gains& g) { g.kp = 2 * g.kp; }));
  EXPECT_FALSE(broken([](Gains& g) { g.tau = 2 * g.tau; })) << "the prefilter is exercised";
  EXPECT_FALSE(hand_matches(run_hand(*cp, config(*cp, hand_gains())), 10 * kTol)) << "a perturbed expectation";
}

// The double-precision oracle of the law (module header), with dt taken from the stamps.
struct OracleStep {
  std::array<double, kA> u{};
  std::array<double, kA> scale{};  // sum of the magnitudes of the terms, for the rounding bound
};

enum class OracleMode { Exact, NominalDt, NoD };

std::vector<OracleStep> run_oracle(const RateConfig<float>& cfg, const std::vector<std::array<double, 2 * kA>>& in,
                                   const std::vector<std::int64_t>& jitter, OracleMode mode) {
  std::array<double, kA> r{};
  std::array<double, kA> e{};
  std::array<double, kA> integral{};
  std::array<double, kA> y_prev{};
  std::vector<OracleStep> out;
  for (std::size_t n = 0; n < in.size(); ++n) {
    OracleStep s;
    const double dt_exact = static_cast<double>(static_cast<std::int64_t>(kPeriodUs) + jitter[n]) / 1.0e6;
    const double dt = mode == OracleMode::NominalDt ? static_cast<double>(kPeriodUs) / 1.0e6 : dt_exact;
    for (std::size_t a = 0; a < kA; ++a) {
      const double y = in[n][a];
      const double sp = in[n][kA + a];
      if (n == 0) {
        r[a] = y;
        e[a] = 0;
        integral[a] = 0;
        s.u[a] = 0;
      } else {
        const double alpha = 1.0 - std::exp(-dt / static_cast<double>(cfg.tau_ref[a]));
        r[a] = r[a] + alpha * (sp - r[a]);
        const double err = r[a] - y;
        integral[a] += static_cast<double>(cfg.ki[a]) * e[a] * dt;
        const double d = mode == OracleMode::NoD ? 0.0 : -static_cast<double>(cfg.kd[a]) * (y - y_prev[a]) / dt;
        s.u[a] = static_cast<double>(cfg.kp[a]) * err + integral[a] + d;
        s.scale[a] = static_cast<double>(cfg.kp[a]) * (std::abs(err) + std::abs(y)) + std::abs(integral[a]) +
                     std::abs(d);
        e[a] = err;
      }
      y_prev[a] = y;
    }
    out.push_back(s);
  }
  return out;
}

// Scenario test value: the rounding bound of a float run against the oracle, 1024 eps (about 1.2e-4) of the summed
// term magnitudes. Ten float operations, a cancellation in r - y and about two hundred integrator additions.
constexpr double kOracleBound = 1024.0 * static_cast<double>(kEps);

// Largest |u - oracle| / (kOracleBound * scale); at most 1 passes.
double worst_ratio(const std::vector<V3>& got, const std::vector<OracleStep>& want) {
  double worst = 0;
  for (std::size_t n = 0; n < got.size(); ++n) {
    for (std::size_t a = 0; a < kA; ++a) {
      const double err = std::abs(static_cast<double>(got[n][a]) - want[n].u[a]);
      const double bound = kOracleBound * want[n].scale[a];
      worst = std::max(worst, bound > 0 ? err / bound : err);
    }
  }
  return worst;
}

TEST(L4RateLaw, MatchesADoublePrecisionOracleWithDtJitterOfPlusMinusOneMicrosecond) {
  REQUIRE_CARD();
  // Scenario test values: distinct gains per axis so an axis mix-up shows; kd large enough that D is a large part
  // of u, so a 1 us change of dt (1e-3 relative) is visible in u; tau_ref of 5, 10 and 20 periods.
  const std::array<Gains, kA> g{Gains{2, 50, 0.2, 5 * static_cast<double>(kPeriodS)},
                                Gains{3, 60, 0.25, 10 * static_cast<double>(kPeriodS)},
                                Gains{4, 70, 0.3, 20 * static_cast<double>(kPeriodS)}};
  const RateConfig<float> cfg = config(c, g);

  // Scenario test value: 200 executions of a bounded random walk of the gyro (+-0.1 rad/s per step) and setpoints in
  // +-3 rad/s; a fixed seed. The jitter cycles -1, 0, +1 and is random in between (both bounds are reached).
  constexpr std::size_t kSteps = 200;
  std::mt19937 rng(20240930U);
  std::uniform_real_distribution<double> step_dist(-0.1, 0.1);
  std::uniform_real_distribution<double> sp_dist(-3.0, 3.0);
  std::uniform_int_distribution<int> jitter_dist(-1, 1);
  std::array<double, kA> y{};
  std::vector<std::array<double, 2 * kA>> in;
  std::vector<std::int64_t> jitter;
  Rig rig;
  rig.init(c, cfg);
  std::vector<V3> got;
  bool saw_minus = false;
  bool saw_plus = false;
  for (std::size_t n = 0; n < kSteps; ++n) {
    std::array<double, 2 * kA> row{};
    for (std::size_t a = 0; a < kA; ++a) {
      y[a] += step_dist(rng);
      // Round to float first: the oracle sees exactly the values the loop sees.
      row[a] = static_cast<double>(static_cast<float>(y[a]));
      row[kA + a] = static_cast<double>(static_cast<float>(sp_dist(rng)));
    }
    const std::int64_t j = n == 0 ? 0 : (n < 4 ? static_cast<std::int64_t>(n) - 2 : jitter_dist(rng));
    saw_minus = saw_minus || j == -1;
    saw_plus = saw_plus || j == 1;
    in.push_back(row);
    jitter.push_back(j);
    got.push_back(rig.step(v3(row[0], row[1], row[2]), v3(row[kA], row[kA + 1], row[kA + 2]), j).torque);
  }
  ASSERT_TRUE(saw_minus && saw_plus);

  EXPECT_LE(worst_ratio(got, run_oracle(cfg, in, jitter, OracleMode::Exact)), 1.0);
  // Negative controls: an oracle with a nominal dt (the jitter ignored) and one without D must fail the same bound.
  EXPECT_GT(worst_ratio(got, run_oracle(cfg, in, jitter, OracleMode::NominalDt)), 1.0);
  EXPECT_GT(worst_ratio(got, run_oracle(cfg, in, jitter, OracleMode::NoD)), 1.0);
}

// ---- 2. the prefilter and its seeding --------------------------------------------------------------------------

TEST(L4RatePrefilter, FollowsTheFirstOrderStepResponsePerAxis) {
  REQUIRE_CARD();
  // kp = 1 and no I or D with a zero gyro: u = e = r. tau_ref per axis 1, 2 and 4 periods (scenario test values).
  const std::array<double, kA> tau{static_cast<double>(kPeriodS), 2 * static_cast<double>(kPeriodS),
                                   4 * static_cast<double>(kPeriodS)};
  const RateConfig<float> cfg = config(c, std::array<Gains, kA>{Gains{1, 0, 0, tau[0]}, Gains{1, 0, 0, tau[1]},
                                                                  Gains{1, 0, 0, tau[2]}});
  // Scenario test value: a setpoint step of 8 rad/s from a seeded zero, held for six executions.
  constexpr double kStep = 8;
  constexpr int kExec = 6;
  Rig rig;
  rig.init(c, cfg);
  EXPECT_EQ(rig.step(all(0), all(kStep)).torque[0], 0.0F);
  std::array<double, kA> worst_wrong{};
  for (int n = 1; n <= kExec; ++n) {
    const V3 u = rig.step(all(0), all(kStep)).torque;
    for (std::size_t a = 0; a < kA; ++a) {
      // Closed form: r_n = sp (1 - exp(-n T / tau)).
      const double want = kStep * (1.0 - std::exp(-n * static_cast<double>(kPeriodS) / tau[a]));
      EXPECT_TRUE(near(static_cast<double>(u[a]), want)) << "axis " << a << " n " << n;
      // Negative control: the response of a filter with twice the time constant is not the actual response.
      const double wrong = kStep * (1.0 - std::exp(-n * static_cast<double>(kPeriodS) / (2 * tau[a])));
      worst_wrong[a] = std::max(worst_wrong[a], std::abs(static_cast<double>(u[a]) - wrong));
    }
  }
  for (std::size_t a = 0; a < kA; ++a) {
    EXPECT_GT(worst_wrong[a], kTol * (1.0 + kStep)) << "axis " << a;
  }
}

TEST(L4RatePrefilter, IsSeededFromTheFirstGyroValueAtInitAndAfterAReset) {
  REQUIRE_CARD();
  const RateConfig<float> cfg = config(c, hand_gains());  // alpha = 1/2, kp = 2: u_1 = 2 * (sp - y) / 2 = sp - y
  Rig rig;
  rig.init(c, cfg);
  // Scenario test values: a gyro far from zero and a setpoint far from the gyro on every axis, so an unseeded
  // (zero-started) filter would step at once.
  const V3 y1 = v3(5, -3, 1);
  const V3 sp1 = all(100);
  const V3 u0 = rig.step(y1, sp1).torque;
  for (std::size_t a = 0; a < kA; ++a) {
    EXPECT_EQ(u0[a], 0.0F) << "no kick at init, axis " << a;
  }
  const V3 u1 = rig.step(y1, sp1).torque;
  for (std::size_t a = 0; a < kA; ++a) {
    EXPECT_TRUE(near(static_cast<double>(u1[a]), static_cast<double>(sp1[a] - y1[a]))) << "axis " << a;
    // Negative control: an unseeded filter (r_0 = 0) would give 2 (sp/2 - y) at n = 1.
    const double unseeded = 2 * (static_cast<double>(sp1[a]) / 2 - static_cast<double>(y1[a]));
    EXPECT_GT(std::abs(static_cast<double>(u1[a]) - unseeded), kTol * (1.0 + std::abs(unseeded)));
  }

  // After reset(): re-seeded from the next gyro value; a gap in the stamps is not a dt violation.
  const V3 y2 = v3(-1, 2, 4);
  rig.loop.reset();
  rig.t += 10 * kPeriodUs;  // scenario test value: a ten-period pause before the reset execution
  const V3 v0 = rig.step(y2, all(0)).torque;
  for (std::size_t a = 0; a < kA; ++a) {
    EXPECT_EQ(v0[a], 0.0F) << "no kick after reset, axis " << a;
  }
  const V3 v1 = rig.step(y2, all(0)).torque;  // r = y/2 * ... : r_1 = y + (0 - y)/2 = y/2, u = 2 (y/2 - y) = -y
  for (std::size_t a = 0; a < kA; ++a) {
    EXPECT_TRUE(near(static_cast<double>(v1[a]), -static_cast<double>(y2[a]))) << "axis " << a;
  }
}

// ---- 3. D = 0 at a first execution ---------------------------------------------------------------------------------

TEST(L4RateDerivative, IsZeroAtTheFirstExecutionAfterInitAndAfterAFaultReset) {
  REQUIRE_CARD();
  const RateConfig<float> cfg = config(c, hand_gains());  // D = -4 dy at dt = 1 ms
  Rig rig;
  rig.init(c, cfg);
  // A nonzero gyro at the first execution: a D taken against an initial y_prev of 0 would be -4 * 3 = -12.
  const V3 first = rig.step(all(3), all(3)).torque;
  for (std::size_t a = 0; a < kA; ++a) {
    EXPECT_EQ(first[a], 0.0F) << "axis " << a;
  }

  // The D path is live: y 1 -> 2 with sp = 1: r = 1, e = -1, D = -4, u = 2 (-1) - 4 = -6.
  rig.init(c, cfg);
  (void)rig.step(all(1), all(1));
  const V3 live = rig.step(all(2), all(1)).torque;
  EXPECT_TRUE(near(static_cast<double>(live[0]), -6.0));

  // A fault (NaN setpoint), then y = 7 with sp = 7: u = 0 exactly; a D against the pre-fault y = 2 would be -20 and
  // against 0 would be -28. The next execution, y = 9 with sp = 7: r = 7, e = -2, I = 0 (e_prev = 0), D = -8, u = -12.
  const RateOutput<float> fault = rig.step(all(2), v3(kNaN, 1, 1));
  ASSERT_TRUE(fault.fault_active);
  const V3 after = rig.step(all(7), all(7)).torque;
  for (std::size_t a = 0; a < kA; ++a) {
    EXPECT_EQ(after[a], 0.0F) << "axis " << a;
  }
  const V3 next = rig.step(all(9), all(7)).torque;
  for (std::size_t a = 0; a < kA; ++a) {
    EXPECT_TRUE(near(static_cast<double>(next[a]), -12.0)) << "axis " << a;
  }
}

// ---- 4. anti-windup ------------------------------------------------------------------------------------------------

// The independent bound b_a = (gamma_n + eps) (|B||M||v|)_a in double, from the card and the allocated vector v.
double bound(const Card& c, std::size_t axis, double thrust, const std::array<double, kA>& torque) {
  const RateConfig<float>& g = c.geometry;
  const prim::Mat<float, mixer::kAxes, kN> b = rate::effectiveness(g);
  const double eps = static_cast<double>(kEps);
  const double n_eps = static_cast<double>(kN) * eps;
  const double gamma = n_eps / (1 - n_eps);
  const std::array<double, mixer::kAxes> v{thrust, torque[0], torque[1], torque[2]};
  double sum = 0;
  for (std::size_t k = 0; k < mixer::kAxes; ++k) {
    for (std::size_t j = 0; j < kN; ++j) {
      sum += std::abs(static_cast<double>(b(mixer::kRoll + axis, j))) *
             std::abs(static_cast<double>(c.mixer.m(j, k))) * std::abs(v[k]);
    }
  }
  return (gamma + eps) * sum;
}

// The integrator increment ki e1 dt of the hand config: sp = +-8 from a seeded zero gyro, alpha = 1/2: r = +-4, e = +-4,
// increment = +-100 * 4 * 0.001 = +-0.4.
constexpr double kIncrement = kHandKi * 4 * 0.001;

// Runs: seed, execution 1 (sp = 8 * sign on every axis), record_allocation(request, al), execution 2. Returns the
// integrator after execution 2 (the increment applied at execution 2, since the integrator was 0 after execution 1),
// and after a third execution when `third` is given.
std::array<double, kA> run_freeze(const Card& c, double sign, const V3& request, const Allocation<float>& al,
                                  std::array<double, kA>* third = nullptr) {
  Rig rig;
  rig.init(c, config(c, hand_gains()));
  (void)rig.step(all(0), all(0));
  (void)rig.step(all(0), all(8 * sign));
  rig.loop.record_allocation(request, al);
  (void)rig.step(all(0), all(8 * sign));
  std::array<double, kA> out{};
  for (std::size_t a = 0; a < kA; ++a) {
    out[a] = static_cast<double>(rig.loop.integrator()[a]);
  }
  if (third != nullptr) {
    (void)rig.step(all(0), all(8 * sign));
    for (std::size_t a = 0; a < kA; ++a) {
      (*third)[a] = static_cast<double>(rig.loop.integrator()[a]);
    }
  }
  return out;
}

struct FreezeCase {
  const char* name;
  bool flag;
  double diff_over_bound;  // (req - ach) / b
  double sign;             // sign of e_1
  bool freeze;             // the rule: flag and |diff| > b and e diff > 0
};

// Scenario test values: twice the bound for "well past it", half the bound for "within it".
constexpr std::array<FreezeCase, 8> kFreezeCases{{
    {"all three hold, e > 0", true, 2, 1, true},
    {"all three hold, e < 0", true, -2, -1, true},
    {"flag absent", false, 2, 1, false},
    {"|req - ach| within the bound", true, 0.5, 1, false},
    {"e opposes req - ach (e > 0, diff < 0)", true, -2, 1, false},
    {"e opposes req - ach (e < 0, diff > 0)", true, 2, -1, false},
    {"rounding: flag set, achieved == requested", true, 0, 1, false},
    {"rounding: flag set, achieved == requested, e < 0", true, 0, -1, false},
}};

// Runs one case on one axis; returns whether the integrator froze on that axis (increment 0) and checks that it
// otherwise integrated the full increment, and that the other axes (flag clear) integrated.
bool observed_freeze(const Card& c, const FreezeCase& fc, std::size_t axis) {
  const double thrust = c.thrust_mid;
  const std::array<double, kA> zero_torque{};
  const double b = bound(c, axis, thrust, zero_torque);
  EXPECT_GT(b, 0.0);
  Allocation<float> al;
  al.achieved_thrust = static_cast<float>(thrust);
  V3 req{};
  req[axis] = static_cast<float>(fc.diff_over_bound * b);  // achieved torque is 0 on every axis: diff = req
  (axis == 0 ? al.flags.roll : axis == 1 ? al.flags.pitch : al.flags.yaw) = fc.flag;
  std::array<double, kA> third{};
  const std::array<double, kA> after2 = run_freeze(c, fc.sign, req, al, &third);
  const double full = fc.sign * kIncrement;
  for (std::size_t a = 0; a < kA; ++a) {
    if (a != axis) {
      EXPECT_TRUE(near(after2[a], full)) << fc.name << " other axis " << a;
    }
  }
  const bool froze = std::abs(after2[axis]) <= kTol;
  if (froze) {
    EXPECT_EQ(after2[axis], 0.0) << fc.name;
  } else {
    EXPECT_TRUE(near(after2[axis], full)) << fc.name << " axis " << axis;
  }
  // A freeze gates one increment only: with no new record, execution 3 integrates the full increment again, on the
  // e_2 of the prefilter (r_2 = +-6, e_2 = +-6, increment +-0.6).
  const double full3 = fc.sign * kHandKi * 6 * 0.001;
  EXPECT_TRUE(near(third[axis] - after2[axis], full3)) << fc.name << " axis " << axis << " third";
  return froze;
}

TEST(L4RateAntiWindup, FreezesOnlyWhenTheFlagTheMagnitudeAndTheSignAllHold) {
  REQUIRE_CARD();
  for (std::size_t axis = 0; axis < kA; ++axis) {
    for (const FreezeCase& fc : kFreezeCases) {
      EXPECT_EQ(observed_freeze(c, fc, axis), fc.freeze) << fc.name << " axis " << axis;
    }
  }
}

TEST(L4RateAntiWindup, NegativeControlEachConditionDroppedIsCaughtByTheCaseTable) {
  REQUIRE_CARD();
  // A rule with one condition dropped disagrees with the observed behaviour on at least one case.
  const auto rule = [](const FreezeCase& fc, bool use_flag, bool use_mag, bool use_sign) {
    const bool sign_ok = fc.sign * fc.diff_over_bound > 0;
    return (!use_flag || fc.flag) && (!use_mag || std::abs(fc.diff_over_bound) > 1) && (!use_sign || sign_ok);
  };
  std::array<bool, 4> caught{};  // full rule, no flag, no magnitude, no sign
  for (const FreezeCase& fc : kFreezeCases) {
    const bool observed = observed_freeze(c, fc, 0);
    caught[0] = caught[0] || rule(fc, true, true, true) != observed;
    caught[1] = caught[1] || rule(fc, false, true, true) != observed;
    caught[2] = caught[2] || rule(fc, true, false, true) != observed;
    caught[3] = caught[3] || rule(fc, true, true, false) != observed;
  }
  EXPECT_FALSE(caught[0]) << "the full rule agrees on every case";
  EXPECT_TRUE(caught[1]);
  EXPECT_TRUE(caught[2]);
  EXPECT_TRUE(caught[3]);
}

TEST(L4RateAntiWindup, AFreezeOnOneAxisLeavesTheOthersIntegrating) {
  REQUIRE_CARD();
  const double thrust = c.thrust_mid;
  const double b_roll = bound(c, 0, thrust, {});
  const double b_yaw = bound(c, 2, thrust, {});
  Allocation<float> al;
  al.achieved_thrust = static_cast<float>(thrust);
  al.flags.roll = true;
  al.flags.yaw = true;
  V3 req{};
  req[0] = static_cast<float>(2 * b_roll);
  req[2] = static_cast<float>(-2 * b_yaw);  // opposes e: yaw integrates
  const std::array<double, kA> after = run_freeze(c, 1.0, req, al);
  EXPECT_EQ(after[0], 0.0);
  EXPECT_TRUE(near(after[1], kIncrement));
  EXPECT_TRUE(near(after[2], kIncrement));
}

TEST(L4RateAntiWindup, RealAllocationWithASubnormalRollAndASaturatingPitchFreezesPitchOnly) {
  REQUIRE_CARD();
  const float tiny = std::numeric_limits<float>::denorm_min();
  // Scenario test value: a pitch request of ten axis scales, past the per-motor thrust spread for any collective.
  const float big = static_cast<float>(10 * c.pitch_scale);
  Request<float> rq;
  rq.thrust = static_cast<float>(c.thrust_mid);
  rq.torque = V3(tiny, big, 0.0F);
  const Allocation<float> al = mixer::allocate(c.mixer, rq);
  ASSERT_TRUE(al.flags.roll) << "roll is flagged although its request is subnormal";
  ASSERT_TRUE(al.flags.pitch);
  ASSERT_FALSE(al.flags.yaw);
  // Roll rounds: |tx - achieved| is at most one subnormal step, far below the bound; pitch is really cut.
  EXPECT_LE(std::abs(tiny - al.achieved_torque[0]), tiny);
  EXPECT_GT(big - al.achieved_torque[1], 0.0F);
  const double b_roll = bound(c, 0, static_cast<double>(al.achieved_thrust),
                              {static_cast<double>(al.achieved_torque[0]), static_cast<double>(al.achieved_torque[1]),
                               static_cast<double>(al.achieved_torque[2])});
  ASSERT_GT(b_roll, static_cast<double>(tiny));

  const std::array<double, kA> after = run_freeze(c, 1.0, rq.torque, al);
  EXPECT_TRUE(near(after[0], kIncrement)) << "the rounding case integrates";
  EXPECT_EQ(after[1], 0.0) << "the real saturation freezes";
  EXPECT_TRUE(near(after[2], kIncrement));

  // Negative control: a rule without the magnitude condition (flag and sign only) would have frozen roll.
  EXPECT_GT(tiny - al.achieved_torque[0], 0.0F);
}

TEST(L4RateAntiWindup, TheBoundComesFromTheAllocatedVectorSoAZeroVectorHasNone) {
  REQUIRE_CARD();
  // v = 0 (nothing allocated) gives b = 0, so any nonzero difference with the flag and the sign freezes; with the
  // card's collective allocated the same difference is within the bound and integrates.
  const std::array<double, kA> zero{};
  EXPECT_EQ(bound(c, 0, 0.0, zero), 0.0);
  EXPECT_GT(bound(c, 0, c.thrust_mid, zero), 0.0);
  const float tiny = std::numeric_limits<float>::denorm_min();
  V3 req{};
  req[0] = tiny;
  Allocation<float> nothing;
  nothing.flags.roll = true;
  EXPECT_EQ(run_freeze(c, 1.0, req, nothing)[0], 0.0);
  Allocation<float> collective = nothing;
  collective.achieved_thrust = static_cast<float>(c.thrust_mid);
  EXPECT_TRUE(near(run_freeze(c, 1.0, req, collective)[0], kIncrement));
}

TEST(L4RateEffectiveness, TheMixerTimesTheRateConfigEffectivenessIsTheIdentityWithinTheBound) {
  REQUIRE_CARD();
  const double eps = static_cast<double>(kEps);
  const double n_eps = static_cast<double>(kN) * eps;
  const double gamma = n_eps / (1 - n_eps);
  const auto worst = [&](const prim::Mat<float, mixer::kAxes, kN>& b) {
    const prim::Mat<float, kN, kN> product = c.mixer.m * b;
    double w = 0;
    for (std::size_t i = 0; i < kN; ++i) {
      for (std::size_t j = 0; j < kN; ++j) {
        double abs_sum = 0;
        for (std::size_t k = 0; k < kN; ++k) {
          abs_sum += std::abs(static_cast<double>(c.mixer.m(i, k))) * std::abs(static_cast<double>(b(k, j)));
        }
        const double err = std::abs(static_cast<double>(product(i, j)) - (i == j ? 1.0 : 0.0));
        w = std::max(w, err / ((gamma + eps) * abs_sum));
      }
    }
    return w;
  };
  prim::Mat<float, mixer::kAxes, kN> b = rate::effectiveness(c.geometry);
  EXPECT_LE(worst(b), 1.0);
  // Negative control: B[roll, 0] moved so that element (0, 0) of M B moves by 1000 times its bound.
  const double m_roll = std::abs(static_cast<double>(c.mixer.m(0, mixer::kRoll)));
  ASSERT_GT(m_roll, 0.0);
  double abs_sum = 0;
  for (std::size_t k = 0; k < kN; ++k) {
    abs_sum += std::abs(static_cast<double>(c.mixer.m(0, k))) * std::abs(static_cast<double>(b(k, 0)));
  }
  b(mixer::kRoll, 0) += static_cast<float>(1000.0 * (gamma + eps) * abs_sum / m_roll);
  EXPECT_GT(worst(b), 1.0);
}

// ---- 5. faults -------------------------------------------------------------------------------------------------

struct FaultCase {
  const char* name;
  V3 gyro;
  V3 setpoint;
  std::uint32_t flags;
};

constexpr std::uint32_t kValid = imu_flag(ImuFlag::GyroValid);

std::vector<FaultCase> fault_cases() {
  std::vector<FaultCase> v;
  const V3 y = all(1);
  const V3 sp = all(2);
  for (std::size_t a = 0; a < kA; ++a) {
    V3 g = y;
    g[a] = kNaN;
    v.push_back({"gyro NaN", g, sp, kValid});
    g[a] = kInf;
    v.push_back({"gyro +inf", g, sp, kValid});
    g[a] = -kInf;
    v.push_back({"gyro -inf", g, sp, kValid});
    V3 s = sp;
    s[a] = kNaN;
    v.push_back({"setpoint NaN", y, s, kValid});
    s[a] = kInf;
    v.push_back({"setpoint +inf", y, s, kValid});
    s[a] = -kInf;
    v.push_back({"setpoint -inf", y, s, kValid});
  }
  v.push_back({"GyroValid clear with a finite zero gyro", all(0), sp, 0});
  v.push_back({"GyroValid clear with a finite gyro", y, sp, imu_flag(ImuFlag::AccelValid)});
  return v;
}

// Builds a nonzero integrator, plants `fc` and checks the fault, the reset and the resumption.
void check_fault(const Card& c, const RateConfig<float>& cfg, const FaultCase& fc) {
  Rig rig;
  rig.init(c, cfg);
  (void)rig.step(all(0), all(0));
  RateOutput<float> healthy;
  for (int n = 0; n < 3; ++n) {
    healthy = rig.step(all(0.5), all(2));
  }
  ASSERT_FALSE(healthy.fault_active) << fc.name;
  ASSERT_GT(rig.loop.integrator()[0], 0.0F) << fc.name << ": the integrator holds something to reset";
  ASSERT_GT(healthy.torque[0], 0.0F);

  const RateOutput<float> bad = rig.step(fc.gyro, fc.setpoint, 0, fc.flags);
  EXPECT_TRUE(bad.fault_active) << fc.name;
  EXPECT_TRUE(bad.fault_latched) << fc.name;
  EXPECT_EQ(bad.fault_count, 1U) << fc.name;
  for (std::size_t a = 0; a < kA; ++a) {
    EXPECT_EQ(bad.torque[a], 0.0F) << fc.name << " axis " << a;
    EXPECT_EQ(rig.loop.integrator()[a], 0.0F) << fc.name << " axis " << a;
  }

  // The next valid execution resumes from the reset state: identical to a fresh loop given the same two samples.
  const V3 y_a = v3(0.7, -0.4, 0.2);
  const V3 y_b = v3(0.9, -0.1, 0.5);
  const RateOutput<float> r0 = rig.step(y_a, all(2));
  EXPECT_FALSE(r0.fault_active) << fc.name;
  EXPECT_TRUE(r0.fault_latched) << fc.name << ": the latch is sticky";
  EXPECT_EQ(r0.fault_count, 1U) << fc.name;
  for (std::size_t a = 0; a < kA; ++a) {
    EXPECT_EQ(r0.torque[a], 0.0F) << fc.name << ": seeded, so no output step";
  }
  const RateOutput<float> r1 = rig.step(y_b, all(2));
  Rig fresh;
  fresh.init(c, cfg);
  (void)fresh.step(y_a, all(2));
  const RateOutput<float> f1 = fresh.step(y_b, all(2));
  EXPECT_FALSE(r1.fault_active) << fc.name;
  EXPECT_TRUE(r1.fault_latched) << fc.name;
  for (std::size_t a = 0; a < kA; ++a) {
    EXPECT_EQ(r1.torque[a], f1.torque[a]) << fc.name << " axis " << a;
  }
  EXPECT_FALSE(f1.fault_latched) << "a fresh loop is not latched";
  EXPECT_EQ(f1.fault_count, 0U);
  EXPECT_NE(r1.torque[0], 0.0F);

  // A second fault increments the counter; init clears the latch and the counter.
  const RateOutput<float> bad2 = rig.step(fc.gyro, fc.setpoint, 0, fc.flags);
  EXPECT_TRUE(bad2.fault_active) << fc.name;
  EXPECT_EQ(bad2.fault_count, 2U) << fc.name;
  rig.init(c, cfg);
  const RateOutput<float> after_init = rig.step(y_a, all(2));
  EXPECT_FALSE(after_init.fault_latched) << fc.name;
  EXPECT_EQ(after_init.fault_count, 0U) << fc.name;
}

TEST(L4RateFaults, PlantedBadInputsResetTheLoopZeroTheTorqueAndLatch) {
  REQUIRE_CARD();
  const RateConfig<float> cfg = config(c, hand_gains());
  for (const FaultCase& fc : fault_cases()) {
    check_fault(c, cfg, fc);
  }
}

TEST(L4RateFaults, ANonFiniteOwnOutputFaultsTheWholeVector) {
  REQUIRE_CARD();
  // Scenario test value: a roll kp of 3e38, finite but within a factor of 1.1 of the float maximum, so that kp * e
  // overflows to +inf for any error of about 1 rad/s or more. The other axes are ordinary.
  std::array<Gains, kA> g{hand_gains(), hand_gains(), hand_gains()};
  g[0].kp = 3.0e38;
  const RateConfig<float> cfg = config(c, g);
  ASSERT_EQ(rate::validate(cfg), rate::ConfigError::None);
  Rig rig;
  rig.init(c, cfg);
  (void)rig.step(all(0), all(0));
  // sp = 8, alpha = 1/2: e = 4 on every axis; roll kp e = 1.2e39 -> inf; pitch and yaw are finite.
  const RateOutput<float> bad = rig.step(all(0), all(8));
  EXPECT_TRUE(bad.fault_active);
  EXPECT_TRUE(bad.fault_latched);
  EXPECT_EQ(bad.fault_count, 1U);
  for (std::size_t a = 0; a < kA; ++a) {
    EXPECT_EQ(bad.torque[a], 0.0F) << "axis " << a;
    EXPECT_EQ(rig.loop.integrator()[a], 0.0F) << "axis " << a;
  }
  // The next valid execution (sp = y so that e stays 0) resumes and clears fault_active.
  const RateOutput<float> next = rig.step(all(0), all(0));
  EXPECT_FALSE(next.fault_active);
  EXPECT_TRUE(next.fault_latched);
  EXPECT_EQ(next.fault_count, 1U);
  for (std::size_t a = 0; a < kA; ++a) {
    EXPECT_EQ(next.torque[a], 0.0F);
  }
  const RateOutput<float> after = rig.step(all(0), all(0));
  EXPECT_FALSE(after.fault_active);
  EXPECT_EQ(after.torque[0], 0.0F);
}

TEST(L4RateFaults, NegativeControlHealthyInputsNeverFault) {
  REQUIRE_CARD();
  Rig rig;
  rig.init(c, config(c, hand_gains()));
  RateOutput<float> o;
  for (const HandStep& s : kHand) {
    o = rig.step(all(s.y), all(s.sp));
    EXPECT_FALSE(o.fault_active);
  }
  EXPECT_FALSE(o.fault_latched);
  EXPECT_EQ(o.fault_count, 0U);
  EXPECT_NE(o.torque[0], 0.0F);
}

// ---- 6. the dt check ---------------------------------------------------------------------------------------------

TEST(L4RateDtDeathTest, SpacingOutsideOneMicrosecondOfTheDesignPeriodPanics) {
  REQUIRE_CARD();
  const RateConfig<float> cfg = config(c, hand_gains());
  const auto run = [&](std::int64_t jitter) {
    Rig rig;
    rig.init(c, cfg);
    (void)rig.step(all(0), all(0));
    (void)rig.step(all(0), all(0), jitter);
  };
  // Inside the bound: no panic (negative control for the death cases).
  run(-1);
  run(0);
  run(1);
  EXPECT_DEATH(run(2), "more than 1 us");
  EXPECT_DEATH(run(-2), "more than 1 us");
  EXPECT_DEATH(run(-static_cast<std::int64_t>(kPeriodUs)), "more than 1 us");  // a repeated stamp
  EXPECT_DEATH(run(static_cast<std::int64_t>(kPeriodUs)), "more than 1 us");   // a dropped sample
  EXPECT_DEATH(run(-static_cast<std::int64_t>(kPeriodUs) - 1), "more than 1 us");  // stamp going backwards
}

// The flight period: two 156.25 µs ticks (the scenario register's 625/4 µs tick and divisor 2), T = 312.5 µs, with
// the hal_sim stamps floor(N · 625/4) µs, whose two-tick spacing alternates 312 / 313 µs (quad §4 L0).
TEST(L4RateDtDeathTest, HalfMicrosecondPeriodIsCheckedExactly) {
  REQUIRE_CARD();
  constexpr std::uint64_t kNum = 625;  // scenario test value: the tick numerator (µs), design/scenario_values.yaml
  constexpr std::uint64_t kDen = 4;    // scenario test value: the tick denominator, design/scenario_values.yaml
  constexpr std::uint64_t kDiv = 2;    // scenario test value: rate_loop_divisor, design/scenario_values.yaml
  RateConfig<float> cfg = config(c, hand_gains());
  cfg.period = static_cast<float>(static_cast<double>(kDiv * kNum) / static_cast<double>(kDen) / 1.0e6);
  const auto stamp = [&](std::uint64_t tick) { return tick * kNum / kDen; };
  // Real stamps over many executions: both spacings (312 and 313 µs, |dt − T| = 0.5 µs) pass.
  {
    RateLoop<float> loop;
    loop.init(cfg, c.mixer);
    bool saw312 = false;
    bool saw313 = false;
    for (std::uint64_t n = 0; n < 64; ++n) {  // scenario test value: 64 executions cover both spacings many times
      if (n > 0) {
        const std::uint64_t d = stamp(n * kDiv) - stamp((n - 1) * kDiv);
        saw312 = saw312 || d == 312;
        saw313 = saw313 || d == 313;
      }
      (void)loop.execute(Rig::sample(stamp(n * kDiv), all(0), imu_flag(ImuFlag::GyroValid)), all(0));
    }
    EXPECT_TRUE(saw312 && saw313);
  }
  // A spacing 1.5 µs from T (311 or 314 µs) is outside the bound and panics, whichever way T rounds to whole µs;
  // an integer-µs check with T rounded to 313 (or 312) accepts one of them (negative control for the rounding).
  const auto run = [&](std::uint64_t dt_us) {
    RateLoop<float> loop;
    loop.init(cfg, c.mixer);
    (void)loop.execute(Rig::sample(kT0Us, all(0), imu_flag(ImuFlag::GyroValid)), all(0));
    (void)loop.execute(Rig::sample(kT0Us + dt_us, all(0), imu_flag(ImuFlag::GyroValid)), all(0));
  };
  run(312);
  run(313);
  EXPECT_DEATH(run(311), "more than 1 us");
  EXPECT_DEATH(run(314), "more than 1 us");
}

TEST(L4RateDtDeathTest, TheCheckContinuesThroughFaultExecutions) {
  REQUIRE_CARD();
  const RateConfig<float> cfg = config(c, hand_gains());
  EXPECT_DEATH(
      {
        Rig rig;
        rig.init(c, cfg);
        (void)rig.step(all(0), all(0));
        (void)rig.step(all(0), v3(kNaN, 0, 0));
        (void)rig.step(all(0), all(0), 2);
      },
      "more than 1 us");
}

// ---- 7. configuration validation -----------------------------------------------------------------------------------

TEST(L4RateConfig, EveryInvalidConfigIsRejectedWithItsReason) {
  REQUIRE_CARD();
  const RateConfig<float> ok = config(c, hand_gains());
  EXPECT_EQ(rate::validate(ok), rate::ConfigError::None);  // the baseline is valid, so the mutations do the rejecting
  RateConfig<float> zero = ok;
  zero.kp = all(0);
  zero.ki = all(0);
  zero.kd = all(0);
  EXPECT_EQ(rate::validate(zero), rate::ConfigError::None) << "zero gains are allowed";

  struct Case {
    const char* name;
    RateConfig<float> cfg;
    rate::ConfigError want;
  };
  std::vector<Case> cases;
  auto add = [&](const char* name, rate::ConfigError want, auto mutate) {
    RateConfig<float> m = ok;
    mutate(m);
    cases.push_back({name, m, want});
  };
  using rate::ConfigError;
  for (std::size_t a = 0; a < kA; ++a) {
    add("kp NaN", ConfigError::NonFinite, [&](auto& m) { m.kp[a] = kNaN; });
    add("ki inf", ConfigError::NonFinite, [&](auto& m) { m.ki[a] = kInf; });
    add("kd -inf", ConfigError::NonFinite, [&](auto& m) { m.kd[a] = -kInf; });
    add("tau_ref NaN", ConfigError::NonFinite, [&](auto& m) { m.tau_ref[a] = kNaN; });
    add("kp < 0", ConfigError::NegativeGain, [&](auto& m) { m.kp[a] = -m.kp[a]; });
    add("ki < 0", ConfigError::NegativeGain, [&](auto& m) { m.ki[a] = -m.ki[a]; });
    add("kd < 0", ConfigError::NegativeGain, [&](auto& m) { m.kd[a] = -m.kd[a]; });
    add("tau_ref = 0", ConfigError::TauRef, [&](auto& m) { m.tau_ref[a] = 0; });
    add("tau_ref < 0", ConfigError::TauRef, [&](auto& m) { m.tau_ref[a] = -m.tau_ref[a]; });
  }
  add("T = 0", ConfigError::Period, [](auto& m) { m.period = 0; });
  add("T < 0", ConfigError::Period, [](auto& m) { m.period = -m.period; });
  add("T NaN", ConfigError::NonFinite, [&](auto& m) { m.period = kNaN; });
  add("T inf", ConfigError::NonFinite, [&](auto& m) { m.period = kInf; });
  add("rotor_x NaN", ConfigError::NonFinite, [&](auto& m) { m.rotor_x[0] = kNaN; });
  add("rotor_y inf", ConfigError::NonFinite, [&](auto& m) { m.rotor_y[1] = kInf; });
  add("yaw_sign NaN", ConfigError::NonFinite, [&](auto& m) { m.yaw_sign[2] = kNaN; });
  add("torque_ratio NaN", ConfigError::NonFinite, [&](auto& m) { m.torque_ratio = kNaN; });
  for (const Case& k : cases) {
    EXPECT_EQ(rate::validate(k.cfg), k.want) << k.name;
  }
}

TEST(L4RateConfigDeathTest, InvalidConfigPanicsNamingTheRule) {
  REQUIRE_CARD();
  const RateConfig<float> ok = config(c, hand_gains());
  rate::require_valid(ok);
  RateConfig<float> bad = ok;
  bad.kp[1] = -1;
  EXPECT_DEATH(rate::require_valid(bad), "a gain kp, ki or kd is negative");
  bad = ok;
  bad.tau_ref[2] = 0;
  EXPECT_DEATH(rate::require_valid(bad), "tau_ref is not positive");
  bad = ok;
  bad.period = 0;
  EXPECT_DEATH(rate::require_valid(bad), "design period is not positive");
  bad = ok;
  bad.kd[0] = kNaN;
  EXPECT_DEATH(rate::require_valid(bad), "not finite");
}

}  // namespace
