// L6 stage (c), commit 1, T1 (decision 0014): the gyro chain's NaN guard and first-sample seeding.
//   - Seeding: with seed_first_sample a constant input gives the input back from tick 0 (no start-up transient, so the
//     rate loop's D term sees no kick); the zero-state chain does not (control).
//   - Guard: a non-finite value on one axis holds that axis's output and states, sets its flag and counter, and leaves the
//     other axes bit-identical to a chain that never saw it; at the next finite sample the axis restarts from that
//     sample's steady state.
//   - Controls: an unguarded cascade is poisoned by the NaN for the rest of the run; a cascade that holds its states
//     across the gap but does not reseed starts the recovery from the old states (a transient).
// The notch coefficients, fixture and the independent double design come from the L6 gyro-chain helpers (support.hpp,
// read-only use). Every other number is derived or labelled.
#include <algorithm>
#include <limits>
#include <vector>

#include "../gyro_chain/support.hpp"

namespace {

using namespace marv;
using namespace marv::gyro_chain;
using namespace marv::gyro_chain::test;

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();

GyroChainConfig<float> seeded_config(const Fixture& fx, bool seed) {
  GyroChainConfig<float> c = fx.config;
  c.seed_first_sample = seed;
  return c;
}

// Drives update_notches every kDivisor ticks (the rate-loop group's schedule) with healthy speeds.
struct Rig {
  GyroChain<float> chain;
  std::array<float, kMotors> omega = rotor_omegas();
  std::size_t tick = 0;
  bool notches = true;

  void init(const GyroChainConfig<float>& cfg) {
    chain.init(cfg);
    tick = 0;
  }
  Vec3f step(const Vec3f& in) {
    if (notches && tick % kDivisor == 0) {
      chain.update_notches(make_sample(omega));
    }
    ++tick;
    return chain.filter(in);
  }
};

// A deterministic finite three-axis input: sums of tones at scenario test frequencies (rad/tick) and amplitudes (rad/s).
Vec3f varied(std::size_t n) {
  const double t = static_cast<double>(n);
  return Vec3f(static_cast<float>(1.0 + std::sin(0.3927 * t) + 0.5 * std::sin(0.0021 * t)),
               static_cast<float>(0.25 + std::sin(0.3043 * t + 0.5) + 0.5 * std::sin(0.7854 * t)),
               static_cast<float>(-1.0 + std::sin(1.1781 * t + 1.0) + 0.25 * std::sin(0.00137 * t)));
}

// Scenario test value: a constant rate on each axis, distinct and nonzero (rad/s).
const Vec3f kConst(1.5F, -2.25F, 0.75F);

// ---- The size of "equals the input" for a constant input ------------------------------------------------------
// Every stage is a unity-DC-gain section in exact arithmetic (rule in gyro_chain.hpp). The float coefficients give stage
// k the DC gain G_k = (b0 + b1 + b2) / (1 + a1 + a2), not exactly 1. With the states seeded to the constant c, the error
// e = y - c of stage k obeys e_n + a1 e_(n-1) + a2 e_(n-2) = c (b0 + b1 + b2 - 1 - a1 - a2) from zero states, i.e. the step
// response of 1 / A_k, scaled so that it settles at c (G_k - 1); a lightly damped second-order section overshoots a step
// to at most 1 + exp(-pi zeta / sqrt(1 - zeta^2)) < 2 times its final value, so the error of stage k is at most
// 2 |c| |G_k - 1| (and passes later stages, each of gain at most 1 on the slow signal). The rounding of the recursion adds
// at most the local rounding bound of support.hpp (gamma_5 times the sum of the absolute terms) times the l1 norm of the
// impulse response from that stage to the output. The tolerance is the sum, relative to |c|.
double seed_tolerance(const std::vector<BiquadCoeffs<float>>& stages) {
  std::vector<Stage> st;
  for (const BiquadCoeffs<float>& c : stages) {
    st.push_back(to_stage(c));
  }
  const double gamma5 = 5.0 * kU / (1.0 - 5.0 * kU);
  double total = 0;
  for (std::size_t k = 0; k < st.size(); ++k) {
    const Stage& s = st[k];
    total += 2.0 * std::fabs((s.b0 + s.b1 + s.b2) / (1.0 + s.a1 + s.a2) - 1.0);
    Stage rec;
    rec.a1 = s.a1;
    rec.a2 = s.a2;
    std::vector<double> g = run_stage(rec, unit_impulse());
    for (std::size_t j = k + 1; j < st.size(); ++j) {
      g = run_stage(st[j], g);
    }
    const double terms = std::fabs(s.b0) + std::fabs(s.b1) + std::fabs(s.b2) + std::fabs(s.a1) + std::fabs(s.a2);
    total += gamma5 * terms * (1.0 + kAmplitudeSlack) * l1(g);
  }
  return total;
}

std::vector<BiquadCoeffs<float>> chain_stages(const Fixture& fx, bool notches) {
  std::array<bool, kNotches> include{};
  include.fill(notches);
  return ref_stages(fx, rotor_omegas(), include);
}

double worst_relative_error(Rig& rig, std::size_t ticks, const Vec3f& c) {
  double worst = 0;
  for (std::size_t n = 0; n < ticks; ++n) {
    const Vec3f y = rig.step(c);
    for (std::size_t a = 0; a < kAxes; ++a) {
      worst = std::max(worst, std::fabs(static_cast<double>(y[a]) - static_cast<double>(c[a])) /
                                  std::fabs(static_cast<double>(c[a])));
    }
  }
  return worst;
}

constexpr std::size_t kHold = 4000;  // test value: ticks of a constant input, ~ 27 sharpest-notch time constants (147 samples)

TEST(L6ChainGuard, SeededConstantInputGivesTheInputFromTickZero) {
  const Fixture fx = make_fixture();
  for (const bool notches : {true, false}) {
    SCOPED_TRACE(notches);
    const double tol = seed_tolerance(chain_stages(fx, notches));
    EXPECT_LT(tol, 1.0e-2);  // not vacuous: a hundredth of |c|, far below the zero-state error of the control (order |c|)
    Rig rig;
    rig.notches = notches;
    rig.init(seeded_config(fx, true));
    const double worst = worst_relative_error(rig, kHold, kConst);
    EXPECT_LE(worst, tol);
  }
}

// Control: without seeding the first tick is the zero-state start: the error is of order |c| and exceeds the tolerance.
TEST(L6ChainGuard, ControlUnseededChainHasAStartUpTransient) {
  const Fixture fx = make_fixture();
  const double tol = seed_tolerance(chain_stages(fx, true));
  Rig rig;
  rig.init(seeded_config(fx, false));
  const Vec3f y0 = rig.step(kConst);
  for (std::size_t a = 0; a < kAxes; ++a) {
    EXPECT_GT(std::fabs(static_cast<double>(y0[a] - kConst[a])) / std::fabs(static_cast<double>(kConst[a])), 100.0 * tol)
        << a;
  }
}

// Seeding is per axis and only at the first finite sample: a NaN at tick 0 on one axis defers that axis's seed.
TEST(L6ChainGuard, FirstFiniteSampleSeedsEachAxisSeparately) {
  const Fixture fx = make_fixture();
  const double tol = seed_tolerance(chain_stages(fx, true));
  Rig rig;
  rig.init(seeded_config(fx, true));
  const Vec3f y0 = rig.step(Vec3f(kNaN, kConst[1], kInf));
  const float zero = 0.0F;
  EXPECT_EQ(std::memcmp(&y0[0], &zero, sizeof(float)), 0);  // +0 before any valid output
  EXPECT_EQ(std::memcmp(&y0[2], &zero, sizeof(float)), 0);
  EXPECT_EQ(rig.chain.input_fault_flags(), 0b101U);
  EXPECT_EQ(rig.chain.input_fault_count(0), 1U);
  EXPECT_EQ(rig.chain.input_fault_count(1), 0U);
  EXPECT_EQ(rig.chain.input_fault_count(2), 1U);
  for (std::size_t n = 0; n < 50; ++n) {
    const Vec3f y = rig.step(kConst);
    for (std::size_t a = 0; a < kAxes; ++a) {
      EXPECT_LE(std::fabs(static_cast<double>(y[a] - kConst[a])), tol * std::fabs(static_cast<double>(kConst[a])))
          << a << " " << n;
    }
  }
  EXPECT_EQ(rig.chain.input_fault_flags(), 0U);
  EXPECT_EQ(rig.chain.input_fault_count(0), 1U);
}

// ---- The guard -----------------------------------------------------------------------------------------------
constexpr std::size_t kWarm = 600;   // test value: ticks of varied input before the fault
constexpr std::size_t kGap = 7;      // test value: ticks of non-finite input on the faulted axis
constexpr std::size_t kAfter = 2000; // test value: ticks after the fault

// Pitch (axis 1) is non-finite for ticks [kWarm, kWarm + kGap): NaN, +inf, -inf, NaN, ...
float bad_value(std::size_t i) {
  switch (i % 3) {
    case 0:
      return kNaN;
    case 1:
      return kInf;
    default:
      return -kInf;
  }
}

TEST(L6ChainGuard, NonFiniteAxisHoldsFlagsAndCountsAndTheOtherAxesAreUntouched) {
  const Fixture fx = make_fixture();
  Rig faulted;
  Rig clean;
  faulted.init(seeded_config(fx, true));
  clean.init(seeded_config(fx, true));
  Vec3f last_good;
  bool others_identical = true;
  for (std::size_t n = 0; n < kWarm + kGap + kAfter; ++n) {
    Vec3f in = varied(n);
    const bool bad = n >= kWarm && n < kWarm + kGap;
    if (bad) {
      in[1] = bad_value(n - kWarm);
    }
    const Vec3f y = faulted.step(in);
    const Vec3f ref = clean.step(varied(n));
    others_identical = others_identical && std::memcmp(&y[0], &ref[0], sizeof(float)) == 0 &&
                       std::memcmp(&y[2], &ref[2], sizeof(float)) == 0;
    if (n == kWarm - 1) {
      last_good = y;
    }
    if (bad) {
      EXPECT_EQ(std::memcmp(&y[1], &last_good[1], sizeof(float)), 0) << n;  // held, bit for bit
      EXPECT_EQ(faulted.chain.input_fault_flags(), 2U) << n;
      EXPECT_EQ(faulted.chain.input_fault_count(1), n - kWarm + 1) << n;
    } else {
      EXPECT_TRUE(std::isfinite(y[1])) << n;
      EXPECT_EQ(faulted.chain.input_fault_flags(), 0U) << n;
    }
    EXPECT_EQ(faulted.chain.input_fault_count(0), 0U);
    EXPECT_EQ(faulted.chain.input_fault_count(2), 0U);
  }
  EXPECT_TRUE(others_identical);
  EXPECT_EQ(faulted.chain.input_fault_count(1), kGap);  // kept after recovery
  EXPECT_EQ(clean.chain.input_fault_count(1), 0U);
}

// The saturation at the maximum of uint32_t is not driven (2^32 ticks); the counter grows by one per bad sample.
TEST(L6ChainGuard, CounterGrowsByOnePerNonFiniteSample) {
  const Fixture fx = make_fixture();
  Rig rig;
  rig.init(seeded_config(fx, true));
  std::uint32_t prev = 0;
  for (std::size_t n = 0; n < 10000; ++n) {
    (void)rig.step(Vec3f(kNaN, 0.0F, 0.0F));
    EXPECT_EQ(rig.chain.input_fault_count(0), prev + 1U);
    prev = rig.chain.input_fault_count(0);
  }
}

// After the gap, pitch gets a constant: the first finite sample reseeds it, so it is within the constant-input tolerance
// of that constant from that very sample on. Yaw and roll are not reseeded and still carry the varied input's transient.
TEST(L6ChainGuard, RecoveryReseedsToTheSteadyStateOfTheFirstFiniteSample) {
  const Fixture fx = make_fixture();
  const double tol = seed_tolerance(chain_stages(fx, true));
  Rig rig;
  rig.init(seeded_config(fx, true));
  for (std::size_t n = 0; n < kWarm; ++n) {
    (void)rig.step(varied(n));
  }
  for (std::size_t n = 0; n < kGap; ++n) {
    (void)rig.step(Vec3f(varied(kWarm + n)[0], bad_value(n), varied(kWarm + n)[2]));
  }
  const float c = kConst[1];
  for (std::size_t n = 0; n < kAfter; ++n) {
    const Vec3f y = rig.step(Vec3f(varied(kWarm + kGap + n)[0], c, varied(kWarm + kGap + n)[2]));
    EXPECT_LE(std::fabs(static_cast<double>(y[1] - c)), tol * std::fabs(static_cast<double>(c))) << n;
  }
}

// ---- Negative controls -----------------------------------------------------------------------------------------
// A test-local chain that holds the output and states over a non-finite sample but never reseeds (the guard without its
// restart): after the gap pitch resumes from the states of before the gap.
class HoldOnly {
 public:
  explicit HoldOnly(std::vector<BiquadCoeffs<float>> stages) : c_(std::move(stages)), s_(c_.size()) {}
  float filter(float x) {
    if (!std::isfinite(x)) {
      return last_;
    }
    float v = x;
    for (std::size_t k = 0; k < c_.size(); ++k) {
      State& s = s_[k];
      const BiquadCoeffs<float>& c = c_[k];
      const float y = c.b0 * v + c.b1 * s.x1 + c.b2 * s.x2 - c.a1 * s.y1 - c.a2 * s.y2;
      s.x2 = s.x1;
      s.x1 = v;
      s.y2 = s.y1;
      s.y1 = y;
      v = y;
    }
    last_ = v;
    return v;
  }

 private:
  struct State {
    float x1 = 0, x2 = 0, y1 = 0, y2 = 0;
  };
  std::vector<BiquadCoeffs<float>> c_;
  std::vector<State> s_;
  float last_ = 0;
};

TEST(L6ChainGuard, ControlGuardWithoutReseedHasARecoveryTransient) {
  const Fixture fx = make_fixture();
  const double tol = seed_tolerance(chain_stages(fx, true));
  HoldOnly hold(chain_stages(fx, true));
  const float c = kConst[1];
  for (std::size_t n = 0; n < kWarm; ++n) {
    (void)hold.filter(varied(n)[1]);
  }
  for (std::size_t n = 0; n < kGap; ++n) {
    (void)hold.filter(bad_value(n));
  }
  double worst = 0;
  for (std::size_t n = 0; n < kAfter; ++n) {
    worst = std::max(worst, std::fabs(static_cast<double>(hold.filter(c) - c)) / std::fabs(static_cast<double>(c)));
  }
  EXPECT_GT(worst, 100.0 * tol);
  EXPECT_GT(worst, 1.0e-2);
}

TEST(L6ChainGuard, ControlUnguardedCascadeIsPoisonedByTheNaNForTheRestOfTheRun) {
  const Fixture fx = make_fixture();
  RefChain unguarded(chain_stages(fx, true));
  Rig guarded;
  guarded.init(seeded_config(fx, true));
  std::size_t poisoned = 0;
  std::size_t finite_guarded = 0;
  for (std::size_t n = 0; n < kWarm + kGap + kAfter; ++n) {
    Vec3f in = varied(n);
    if (n >= kWarm && n < kWarm + kGap) {
      in[1] = bad_value(n - kWarm);
    }
    const Vec3f u = unguarded.filter(in);
    const Vec3f g = guarded.step(in);
    if (n >= kWarm + kGap && !std::isfinite(u[1])) {
      ++poisoned;
    }
    if (n >= kWarm && std::isfinite(g[1])) {
      ++finite_guarded;
    }
  }
  EXPECT_EQ(poisoned, kAfter);  // every sample after the gap is still non-finite
  EXPECT_EQ(finite_guarded, kGap + kAfter);
}

}  // namespace
