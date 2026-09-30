// The L5 attitude law (quad spec 4 L5, decision 0006 C), T1.
//   structure     pure tilt, pure yaw, the feed-forward, q_e against -q_e, scale invariance
//   Jacobian      the small-angle Jacobian is diag(k, k, k) for the configured w and for w' = 1
//                 (control: an uncompensated law has a yaw entry w k)
//   w             large combined errors depend on w (control: w' = w gives equal outputs)
//   PX4           agreement with a double re-implementation of AttitudeControl::update (controls: the oracle without
//                 the yaw-gain compensation, and with w' = 1, disagree)
//   singularity   rho = 0 exactly gives 2k (x, y, 0); a near-singular sweep stays finite and bounded; 180 degrees in pure
//                 roll does not chatter
//   clamp         both the direction and the bound (control: a per-axis clamp changes the direction)
//   faults, configuration check, period check
// The from_params / load_config wiring test of decision 0006 C is not here: those functions arrive with the parameter
// ids (packet P2 scope).
//
// Numbers in this file are one of: derived (the rule is stated), or a "scenario test value" with its reason.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <random>
#include <vector>

#include <marv/attitude/attitude_law.hpp>
#include <marv/attitude/config.hpp>
#include <marv/prim/constants.hpp>
#include <marv/prim/quat.hpp>
#include <marv/prim/vec.hpp>
#include <marv/types/attitude_state.hpp>

#include "px4_oracle.hpp"

namespace {

using namespace marv;
using attitude::AttitudeConfig;
using attitude::AttitudeLaw;
using attitude::AttitudeOutput;
using attitude::ConfigError;
using marv::l5_attitude_oracle::Qd;
using Q = prim::Quat<float>;
using V3 = prim::Vec3<float>;

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();
constexpr float kMaxFloat = std::numeric_limits<float>::max();
constexpr double kEps = std::numeric_limits<float>::epsilon();
constexpr double kUnitRoundoff = kEps / 2;
constexpr double kPiD = prim::kPi;

double d(float x) { return static_cast<double>(x); }
float f(double x) { return static_cast<float>(x); }
Qd to_d(const Q& q) { return Qd{d(q.w), d(q.x), d(q.y), d(q.z)}; }
Q to_f(const Qd& q) { return Q(f(q.w), f(q.x), f(q.y), f(q.z)); }

// The rotation by `angle` about the unit axis (ax, ay, az).
Qd about(double ax, double ay, double az, double angle) {
  const double s = std::sin(angle / 2);
  return Qd{std::cos(angle / 2), ax * s, ay * s, az * s};
}

// ---- the derived rounding bound ---------------------------------------------------------------------------------

// Forward error of the float closed form against the exact law, in rad/s, for a setpoint error whose reduced-attitude
// magnitude is rho = sqrt(w_e^2 + z_e^2) and a yaw-rate command of magnitude ff. u = epsilon / 2 is the unit roundoff.
//   q^, q_sp^ (norm: 4 products, 3 sums, sqrt, reciprocal, product): 6u relative per component each.
//   q_e (16 products, 12 sums of terms with sum of |terms| <= 2, plus the 12u inherited): 32u absolute per component.
//   rho, a numerator: 50u and 70u absolute; a = numerator / rho: 120u / rho. psi: 100u / rho (atan2 and the quotients).
//   c, s of w psi / 2 (w <= 1): 50u / rho plus 2u of libm. c a + s b: 200u / rho. Times 2 k.
//   The yaw entry carries the factor 1 / w, but its error terms carry w as well (s = sin(w psi / 2)): it stays of
//   order k u, with no 1 / rho.
//   The sum is about 400u / rho per unit k; rounded up to the power of two 512u. The feed-forward rotation (a cross
//   product, a scale, two more products and sums) adds 32u |ff|.
constexpr double kRoundingUnits = 512;
constexpr double kFfRoundingUnits = 32;
// Error of a quantity that the exact law preserves up to the normalisations (a norm of (x, y), the magnitude of a
// unit-vector product): the 6u + 6u of the two normalisations, the 32u of q_e, a rotation by (c, s) and the products
// of the clamp, 64u after rounding up.
constexpr double kNormUnits = 64;

double rounding_bound(double k, double rho, double ff) {
  return kRoundingUnits * kUnitRoundoff * k * (1 + 1 / rho) + kFfRoundingUnits * kUnitRoundoff * ff;
}

// ---- the scenario configuration ---------------------------------------------------------------------------------

// Scenario test value: the attitude period, one millisecond: a whole number of microseconds, so the +-1 us cases are
// exact. (The half-microsecond period is tested separately.)
constexpr std::uint64_t kPeriodUs = 1000;
// Scenario test value: the first stamp, nonzero so a stamp of 0 is not mistaken for "no stamp".
constexpr std::uint64_t kT0Us = 5000;
// Scenario test values: the gain and the yaw weight, of the order the design rule gives (scratch values k = 3.09 1/s,
// w = 0.143), not round, so a mix-up of k and k / w shows. Any positive k and w in (0, 1] work; the outputs scale
// linearly in k.
constexpr float kGain = 3.25F;
constexpr double kK = static_cast<double>(kGain);
constexpr float kYawWeight = 0.15F;
// Scenario test values: rate limits distinct per axis, so an axis mix-up in the clamp shows; large enough not to clamp
// the structural cases (the largest unclamped output there is below 14: 2 k (1 + ...) in roll and pitch, and
// 2 k / w sin(w pi / 2) in yaw, plus a feed-forward below 3).
constexpr float kRateMaxRoll = 100;
constexpr float kRateMaxPitch = 110;
constexpr float kRateMaxYaw = 120;
// Scenario test values for the angle-mode entries the law does not read (they must only pass validate()).
constexpr float kTiltMax = 1;
constexpr float kDeadband = 0;
constexpr float kAlphaMin = 1;
constexpr float kTCross = 1;
// Scenario test value: the seed of every random stream, so a failure reproduces.
constexpr std::uint64_t kSeed = 20260930;
// Scenario test value: the number of random samples per property, enough for the extremes of the rotation group to
// appear and cheap to run.
constexpr int kSamples = 400;

AttitudeConfig<float> test_config() {
  AttitudeConfig<float> c;
  c.kp = kGain;
  c.yaw_weight = kYawWeight;
  c.period = static_cast<float>(kPeriodUs) / static_cast<float>(prim::kMicrosecondsPerSecond);
  c.tilt_max = kTiltMax;
  c.yaw_deadband = kDeadband;
  c.yaw_alpha_min = kAlphaMin;
  c.yaw_t_cross = kTCross;
  c.rate_max = V3(kRateMaxRoll, kRateMaxPitch, kRateMaxYaw);
  return c;
}

// ---- random streams ---------------------------------------------------------------------------------------------

class Rng {
 public:
  explicit Rng(std::uint64_t seed) : g_(seed) {}

  // Uniform in [0, 1) with all the digits of a double.
  double uniform() {
    constexpr int kDigits = std::numeric_limits<double>::digits;
    return std::ldexp(static_cast<double>(g_() >> (64 - kDigits)), -kDigits);
  }

  double gauss() {
    const double u1 = 1 - uniform();
    const double u2 = uniform();
    return std::sqrt(-2 * std::log(u1)) * std::cos(2 * kPiD * u2);
  }

  // Uniform on the rotation group (four Gaussians, normalised).
  Qd unit() { return l5_attitude_oracle::normalized(Qd{gauss(), gauss(), gauss(), gauss()}); }

 private:
  std::mt19937_64 g_;
};

// ---- the rig ----------------------------------------------------------------------------------------------------

struct Rig {
  AttitudeLaw<float> law;
  std::uint64_t t = kT0Us;

  explicit Rig(const AttitudeConfig<float>& c = test_config()) { law.init(c); }

  AttitudeOutput<float> run(const Q& q, const Q& q_sp, float cmd = 0) {
    const AttitudeState<float> a{t, q, V3(), true};
    t += kPeriodUs;
    return law.execute(a, q_sp, cmd);
  }
};

AttitudeOutput<float> once(const AttitudeConfig<float>& c, const Q& q, const Q& q_sp, float cmd = 0) {
  Rig rig(c);
  return rig.run(q, q_sp, cmd);
}

AttitudeOutput<float> once(const Q& q, const Q& q_sp, float cmd = 0) { return once(test_config(), q, q_sp, cmd); }

// rho of the error the law sees: q_e = canonical(q^* (x) q_sp^), from the float inputs, in double.
double rho_of(const Q& q, const Q& q_sp) {
  using namespace marv::l5_attitude_oracle;
  const Qd e = canonical(mul(inv(normalized(to_d(q))), normalized(to_d(q_sp))));
  return std::hypot(e.w, e.z);
}

double max_abs_diff(const V3& a, const std::array<double, 3>& b) {
  double m = 0;
  for (std::size_t i = 0; i < 3; ++i) {
    m = std::max(m, std::fabs(d(a[i]) - b[i]));
  }
  return m;
}

bool bits_equal(const V3& a, const V3& b) {
  for (std::size_t i = 0; i < 3; ++i) {
    if (std::memcmp(&a[i], &b[i], sizeof(float)) != 0) {
      return false;
    }
  }
  return true;
}

// ---- structure --------------------------------------------------------------------------------------------------

// Scenario test values: tilt angles (up to near 180 degrees), tilt axis azimuths and yaw angles that cover both signs
// and the large-angle region.
constexpr std::array<double, 4> kTilts{0.4, 1.3, 2.4, 3.0};
constexpr std::array<double, 5> kAzimuths{0.0, 0.7, 2.0, -2.6, 1.5707963267948966};
constexpr std::array<double, 6> kYaws{0.3, 1.1, 2.0, 3.0, -2.2, -3.1};

TEST(L5Attitude, PureTiltGivesNoYawAndAnAxisPerpendicularToBodyZ) {
  using namespace marv::l5_attitude_oracle;
  Rng rng(kSeed);
  const double k = kGain;
  for (int n = 0; n < kSamples / 8; ++n) {
    const Qd q = to_d(to_f(rng.unit()));
    for (const double theta : kTilts) {
      for (const double phi : kAzimuths) {
        const Q qf = to_f(q);
        const Q qsp = to_f(mul(q, about(std::cos(phi), std::sin(phi), 0, theta)));
        const AttitudeOutput<float> out = once(qf, qsp);
        const double mag = 2 * k * std::sin(theta / 2);
        const std::array<double, 3> expect{mag * std::cos(phi), mag * std::sin(phi), 0};
        EXPECT_LE(max_abs_diff(out.rate_setpoint, expect), rounding_bound(k, std::cos(theta / 2), 0))
            << "n " << n << " theta " << theta << " phi " << phi;
        EXPECT_FALSE(out.fault_active);
      }
    }
  }
}

TEST(L5Attitude, PureYawGivesNoTiltAndTheWeightedYawCommand) {
  using namespace marv::l5_attitude_oracle;
  Rng rng(kSeed);
  const double k = kGain;
  const double w = kYawWeight;
  for (int n = 0; n < kSamples / 8; ++n) {
    const Qd q = to_d(to_f(rng.unit()));
    for (const double psi : kYaws) {
      const Q qf = to_f(q);
      const Q qsp = to_f(mul(q, about(0, 0, 1, psi)));
      const AttitudeOutput<float> out = once(qf, qsp);
      const std::array<double, 3> expect{0, 0, 2 * k / w * std::sin(w * psi / 2)};
      EXPECT_LE(max_abs_diff(out.rate_setpoint, expect), rounding_bound(k, 1, 0)) << "n " << n << " psi " << psi;
    }
  }
}

TEST(L5Attitude, TheYawRateCommandIsFedForwardInBodyAxes) {
  using namespace marv::l5_attitude_oracle;
  // Scenario test value: a world-down yaw-rate command, inside every rate limit.
  constexpr float kCmd = 2.5F;
  // Level and aligned: the command is the yaw rate, exactly.
  const AttitudeOutput<float> level = once(Q::identity(), Q::identity(), kCmd);
  EXPECT_EQ(level.rate_setpoint[0], 0.0F);
  EXPECT_EQ(level.rate_setpoint[1], 0.0F);
  EXPECT_EQ(level.rate_setpoint[2], kCmd);
  // Tilted and on target: rotate(q^*, (0, 0, cmd)).
  Rng rng(kSeed);
  for (int n = 0; n < kSamples; ++n) {
    const Q q = to_f(rng.unit());
    const Qd qd = normalized(to_d(q));
    const std::array<double, 3> zb = dcm_z(inv(qd));
    const std::array<double, 3> expect{zb[0] * d(kCmd), zb[1] * d(kCmd), zb[2] * d(kCmd)};
    EXPECT_LE(max_abs_diff(once(q, q, kCmd).rate_setpoint, expect), rounding_bound(kGain, 1, d(kCmd))) << n;
  }
}

TEST(L5Attitude, TheErrorQuaternionAndItsNegativeGiveIdenticalOutputs) {
  using namespace marv::l5_attitude_oracle;
  Rng rng(kSeed);
  std::size_t differing_control = 0;
  for (int n = 0; n < kSamples; ++n) {
    const Q q = to_f(rng.unit());
    const Q qsp = to_f(rng.unit());
    const Q qn(-q.w, -q.x, -q.y, -q.z);
    const Q qspn(-qsp.w, -qsp.x, -qsp.y, -qsp.z);
    const V3 base = once(q, qsp, 1.0F).rate_setpoint;
    // Negation is exact in binary floating point, every operation is sign-symmetric and canonical restores the sign.
    EXPECT_TRUE(bits_equal(base, once(q, qspn, 1.0F).rate_setpoint)) << "-q_sp " << n;
    EXPECT_TRUE(bits_equal(base, once(qn, qsp, 1.0F).rate_setpoint)) << "-q " << n;
    EXPECT_TRUE(bits_equal(base, once(qn, qspn, 1.0F).rate_setpoint)) << "-q, -q_sp " << n;
    // Control: a different rotation (one component of q_sp moved by ten ulps... of the next float, ten steps) must
    // change the output, so the equality above can fail.
    Q moved = qsp;
    for (int s = 0; s < 10; ++s) {
      moved.x = std::nextafter(moved.x, kInf);
    }
    if (!bits_equal(base, once(q, moved, 1.0F).rate_setpoint)) {
      ++differing_control;
    }
  }
  EXPECT_GT(differing_control, static_cast<std::size_t>(kSamples) / 2);
}

TEST(L5Attitude, ScalingTheQuaternionsByAPositiveFactorLeavesTheOutputUnchanged) {
  using namespace marv::l5_attitude_oracle;
  Rng rng(kSeed);
  // Scenario test values: an exact power-of-two scale (bit-equal) and a factor of three (equal within the rounding).
  constexpr float kPow2 = 4.0F;
  constexpr float kThree = 3.0F;
  for (int n = 0; n < kSamples; ++n) {
    const Q q = to_f(rng.unit());
    const Q qsp = to_f(rng.unit());
    const auto scaled = [](const Q& a, float s) { return Q(a.w * s, a.x * s, a.y * s, a.z * s); };
    const V3 base = once(q, qsp).rate_setpoint;
    EXPECT_TRUE(bits_equal(base, once(scaled(q, kPow2), scaled(qsp, 1 / kPow2)).rate_setpoint)) << n;
    const double bound = 2 * rounding_bound(kGain, rho_of(q, qsp), 0);
    EXPECT_LE(max_abs_diff(once(scaled(q, kThree), scaled(qsp, kThree)).rate_setpoint,
                           {d(base[0]), d(base[1]), d(base[2])}),
              bound)
        << n;
  }
}

// ---- the Jacobian and the weight --------------------------------------------------------------------------------

TEST(L5Attitude, TheSmallAngleJacobianIsDiagKForTheConfiguredWeightAndForUnity) {
  using namespace marv::l5_attitude_oracle;
  const double k = kGain;
  // Central differences of step h: rounding error B / h (B the rounding bound at rho = 1) against truncation error
  // k h^2 / 24 (the third derivative of 2 k / w sin(w theta / 2) is at most k / 4 in magnitude). h balances the first
  // against k h^2: h = cbrt(512 u).
  const double h = std::cbrt(kRoundingUnits * kUnitRoundoff);
  const double tol = rounding_bound(k, 1, 0) / h + k * h * h;
  Rng rng(kSeed);
  const std::array<float, 2> weights{kYawWeight, 1.0F};
  for (const float w : weights) {
    AttitudeConfig<float> cfg = test_config();
    cfg.yaw_weight = w;
    for (int n = 0; n < kSamples / 8; ++n) {
      const Q qf = to_f(rng.unit());
      const Qd q = to_d(qf);
      double j[3][3];
      for (std::size_t col = 0; col < 3; ++col) {
        const double ax = col == 0 ? 1 : 0;
        const double ay = col == 1 ? 1 : 0;
        const double az = col == 2 ? 1 : 0;
        const V3 plus = once(cfg, qf, to_f(mul(q, about(ax, ay, az, h)))).rate_setpoint;
        const V3 minus = once(cfg, qf, to_f(mul(q, about(ax, ay, az, -h)))).rate_setpoint;
        for (std::size_t row = 0; row < 3; ++row) {
          j[row][col] = (d(plus[row]) - d(minus[row])) / (2 * h);
        }
      }
      for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t col = 0; col < 3; ++col) {
          EXPECT_NEAR(j[row][col], row == col ? k : 0, tol) << "w " << d(w) << " n " << n << " J(" << row << "," << col << ")";
        }
      }
      if (w == kYawWeight) {
        // Control: an uncompensated law (yaw gain k instead of k / w) has the yaw entry w k; the check above must fail it.
        EXPECT_GT(std::fabs(d(w) * j[2][2] - k), tol) << n;
      }
    }
  }
}

TEST(L5Attitude, TheWeightChangesLargeCombinedErrors) {
  using namespace marv::l5_attitude_oracle;
  // Scenario test values: a large tilt (1.5 rad) combined with a large yaw (2.5 rad), so w psi is far from small.
  constexpr double kLargeTilt = 1.5;
  constexpr double kLargeYaw = 2.5;
  const Q qsp = to_f(mul(about(1, 0, 0, kLargeTilt), about(0, 0, 1, kLargeYaw)));
  const auto max_diff = [&](float w_a, float w_b) {
    AttitudeConfig<float> a = test_config();
    AttitudeConfig<float> b = test_config();
    a.yaw_weight = w_a;
    b.yaw_weight = w_b;
    const V3 ra = once(a, Q::identity(), qsp).rate_setpoint;
    const V3 rb = once(b, Q::identity(), qsp).rate_setpoint;
    double m = 0;
    for (std::size_t i = 0; i < 3; ++i) {
      m = std::max(m, std::fabs(d(ra[i]) - d(rb[i])));
    }
    return m;
  };
  const double bound = 2 * rounding_bound(kGain, rho_of(Q::identity(), qsp), 0);
  EXPECT_GT(max_diff(kYawWeight, 1.0F), bound);
  // Control: w' = w gives equal outputs, so the "differs" assertion fails on it.
  EXPECT_FALSE(max_diff(kYawWeight, kYawWeight) > bound);
  EXPECT_EQ(max_diff(kYawWeight, kYawWeight), 0.0);
}

// ---- agreement with PX4 -----------------------------------------------------------------------------------------

TEST(L5Attitude, AgreesWithTheDoubleReimplementationOfPx4WithinTheDerivedBound) {
  using namespace marv::l5_attitude_oracle;
  // Scenario test values: the number of samples and the largest feed-forward magnitude (inside every rate limit).
  constexpr int kPx4Samples = 4000;
  constexpr double kFfMax = 3;
  Rng rng(kSeed);
  int skipped = 0;
  int compared = 0;
  int no_compensation_disagrees = 0;
  int unit_weight_disagrees = 0;
  double max_ratio = 0;
  for (int n = 0; n < kPx4Samples; ++n) {
    const Q qf = to_f(rng.unit());
    const Q qsf = to_f(rng.unit());
    const float cmd = f(kFfMax * (2 * rng.uniform() - 1));
    const Px4Result ref = px4_update(to_d(qf), to_d(qsf), d(kGain), d(kYawWeight), true, d(cmd));
    if (ref.near_opposite) {
      ++skipped;
      continue;
    }
    ++compared;
    const V3 out = once(qf, qsf, cmd).rate_setpoint;
    const double bound = rounding_bound(kGain, rho_of(qf, qsf), std::fabs(d(cmd)));
    const double err = max_abs_diff(out, ref.rate);
    max_ratio = std::max(max_ratio, err / bound);
    EXPECT_LE(err, bound) << "sample " << n;
    // Controls: the oracle without PX4's yaw-gain compensation, and with the unit weight, must disagree with the
    // closed form beyond the bound; otherwise the agreement above could not fail.
    const Px4Result nc = px4_update(to_d(qf), to_d(qsf), d(kGain), d(kYawWeight), false, d(cmd));
    const Px4Result w1 = px4_update(to_d(qf), to_d(qsf), d(kGain), 1.0, true, d(cmd));
    no_compensation_disagrees += max_abs_diff(out, nc.rate) > bound ? 1 : 0;
    unit_weight_disagrees += max_abs_diff(out, w1.rate) > bound ? 1 : 0;
  }
  RecordProperty("max_error_over_bound", std::to_string(max_ratio));
  EXPECT_LT(skipped, kPx4Samples / 50);
  EXPECT_GT(compared, 0);
  EXPECT_GT(no_compensation_disagrees, compared * 9 / 10);
  EXPECT_GT(unit_weight_disagrees, compared / 4);
}

// ---- the singular set --------------------------------------------------------------------------------------------

TEST(L5Attitude, AtRhoZeroExactlyTheOutputIsTwoKXYZero) {
  struct Case {
    Q q;
    Q q_sp;
    float ex;
    float ey;
  };
  // q_e = q^* (x) q_sp^ is (0, ex, ey, 0) exactly in each case (permutations and sign flips of exact values).
  const std::array<Case, 5> cases{{
      {Q(1, 0, 0, 0), Q(0, 1, 0, 0), 1, 0},
      {Q(1, 0, 0, 0), Q(0, 0, 1, 0), 0, 1},
      {Q(0, 0, 0, 1), Q(0, 0, 1, 0), 1, 0},
      {Q(0, 0, 0, 1), Q(0, 1, 0, 0), 0, 1},
      {Q(0, 1, 0, 0), Q(1, 0, 0, 0), 1, 0},
  }};
  for (std::size_t i = 0; i < cases.size(); ++i) {
    const AttitudeOutput<float> out = once(cases[i].q, cases[i].q_sp);
    EXPECT_FALSE(out.fault_active) << i;
    EXPECT_EQ(out.rate_setpoint[0], 2 * kGain * cases[i].ex) << i;
    EXPECT_EQ(out.rate_setpoint[1], 2 * kGain * cases[i].ey) << i;
    EXPECT_EQ(out.rate_setpoint[2], 0.0F) << i;
  }
  // A diagonal axis: the magnitude is 2 k to the normalisation rounding, and the yaw command is exactly zero.
  const float c = f(std::sqrt(0.5));
  const AttitudeOutput<float> diag = once(Q::identity(), Q(0, c, c, 0));
  EXPECT_FALSE(diag.fault_active);
  const double mag = std::hypot(d(diag.rate_setpoint[0]), d(diag.rate_setpoint[1]));
  EXPECT_NEAR(mag, 2.0 * kK, 2 * kK * kNormUnits * kUnitRoundoff);
  EXPECT_EQ(diag.rate_setpoint[2], 0.0F);
  // Continuity: just off the singular set (z = 0, rho small and positive) the output is the same 2 k (x, y, 0).
  // Scenario test values: rho = 2^-20 and 2^-10 (float-exact, far above the rounding of q_e).
  for (const int e : {20, 10}) {
    const float rho = std::ldexp(1.0F, -e);
    const float s = f(std::sqrt(1 - d(rho) * d(rho)));
    const AttitudeOutput<float> near = once(Q::identity(), Q(rho, 0.6F * s, 0.8F * s, 0));
    EXPECT_NEAR(d(near.rate_setpoint[0]), 2 * d(kGain) * 0.6, 2 * kK * kNormUnits * kUnitRoundoff) << e;
    EXPECT_NEAR(d(near.rate_setpoint[1]), 2 * d(kGain) * 0.8, 2 * kK * kNormUnits * kUnitRoundoff) << e;
    EXPECT_EQ(near.rate_setpoint[2], 0.0F) << e;
  }
}

TEST(L5Attitude, ANearSingularSweepStaysFiniteAndBounded) {
  using namespace marv::l5_attitude_oracle;
  // rho = 2^-e for every e down to the smallest subnormal and the exact zero: the exponent range is derived from the
  // float format (the smallest subnormal is 2^(min_exponent - digits); 2^(min_exponent - digits - 1) rounds to zero).
  constexpr int kMaxExp = -std::numeric_limits<float>::min_exponent + std::numeric_limits<float>::digits + 1;
  // Below this rho the products of the split no longer keep all their digits (a subnormal times a unit-size number),
  // so the magnitude is checked only at and above it: min_normal 2^digits.
  const double full_precision_rho = std::ldexp(d(std::numeric_limits<float>::min()), std::numeric_limits<float>::digits);
  // Scenario test values: angles that put (w, z) and (x, y) in every quadrant.
  constexpr std::array<double, 6> kBetas{0.0, 0.5, 1.2, 1.5707963267948966, -0.9, 2.5};
  constexpr std::array<double, 4> kGammas{0.0, 0.7, 2.0, -2.6};
  const std::array<Q, 2> bases{Q(1, 0, 0, 0), Q(0, 0, 0, 1)};
  const AttitudeConfig<float> cfg = test_config();
  std::size_t checked_magnitude = 0;
  for (const Q& q : bases) {
    for (int e = 0; e <= kMaxExp; ++e) {
      const double rho = std::ldexp(1.0, -e);
      const double s = std::sqrt(1 - rho * rho);
      for (const double beta : kBetas) {
        for (const double gamma : kGammas) {
          const Q qsp(f(rho * std::cos(beta)), f(s * std::cos(gamma)), f(s * std::sin(gamma)), f(rho * std::sin(beta)));
          const AttitudeOutput<float> out = once(cfg, q, qsp);
          ASSERT_FALSE(out.fault_active) << "e " << e;
          for (std::size_t i = 0; i < 3; ++i) {
            ASSERT_TRUE(std::isfinite(out.rate_setpoint[i])) << "e " << e << " axis " << i;
          }
          EXPECT_LE(std::fabs(out.rate_setpoint[0]), cfg.rate_max[0]);
          EXPECT_LE(std::fabs(out.rate_setpoint[1]), cfg.rate_max[1]);
          EXPECT_LE(std::fabs(out.rate_setpoint[2]), cfg.rate_max[2]);
          const double rho_e = rho_of(q, qsp);
          if (rho_e >= full_precision_rho) {
            const Qd qe = canonical(mul(inv(normalized(to_d(q))), normalized(to_d(qsp))));
            const double want = 2 * kK * std::hypot(qe.x, qe.y);
            EXPECT_NEAR(std::hypot(d(out.rate_setpoint[0]), d(out.rate_setpoint[1])), want,
                        2 * kK * kNormUnits * kUnitRoundoff)
                << "e " << e;
            ++checked_magnitude;
          }
        }
      }
    }
  }
  EXPECT_GT(checked_magnitude, std::size_t{100});
  // The exact zero itself is in the sweep at the largest exponent for beta = 0 and z = 0.
  const AttitudeOutput<float> zero = once(cfg, Q::identity(), Q(0, 1, 0, 0));
  EXPECT_EQ(zero.rate_setpoint[0], 2 * kGain);
}

TEST(L5Attitude, APureRollThroughOneHundredEightyDegreesDoesNotChatter) {
  // Scenario test values: a sweep of 101 roll angles in steps of 1e-3 rad around pi.
  constexpr int kHalf = 50;
  constexpr double kStep = 1e-3;
  int sign_changes = 0;
  double previous = 0;
  for (int j = -kHalf; j <= kHalf; ++j) {
    const double theta = kPiD + j * kStep;
    const Q qsp = to_f(about(1, 0, 0, theta));
    const AttitudeOutput<float> out = once(Q::identity(), qsp);
    ASSERT_FALSE(out.fault_active);
    EXPECT_EQ(out.rate_setpoint[2], 0.0F) << j;  // psi = 0 on both sides
    EXPECT_EQ(out.rate_setpoint[1], 0.0F) << j;
    // The magnitude stays 2 k sin(theta / 2), continuous through pi (it is 2 k there).
    EXPECT_NEAR(std::fabs(d(out.rate_setpoint[0])), 2 * kK * std::sin(theta / 2), 2 * kK * kNormUnits * kUnitRoundoff)
        << j;
    if (j > -kHalf && previous * d(out.rate_setpoint[0]) < 0) {
      ++sign_changes;
    }
    previous = d(out.rate_setpoint[0]);
  }
  // The shortest rotation reverses its sense exactly once, where the roll passes pi.
  EXPECT_EQ(sign_changes, 1);
}

// ---- the clamp ---------------------------------------------------------------------------------------------------

TEST(L5Attitude, TheClampKeepsTheTiltDirectionAndTheBound) {
  using namespace marv::l5_attitude_oracle;
  // Scenario test values: rate limits below the unclamped outputs, distinct per axis.
  constexpr float kClampRoll = 2.0F;
  constexpr float kClampPitch = 1.2F;
  constexpr float kClampYaw = 1.0F;
  AttitudeConfig<float> tight = test_config();
  tight.rate_max = V3(kClampRoll, kClampPitch, kClampYaw);
  // Scenario test values: tilt azimuths (an axis with pitch binding, one with roll binding, a negative one) of 2.5 rad.
  constexpr std::array<std::array<double, 2>, 4> kAxes{{{0.6, 0.8}, {1.0, 0.0}, {-0.8, 0.6}, {0.3, -0.954}}};
  constexpr double kClampTilt = 2.5;
  // The scale and the product round twice: a direction error of 16u.
  const double direction_tol = 16 * kUnitRoundoff;
  int both_exceed = 0;
  for (const auto& ax : kAxes) {
    const double norm = std::hypot(ax[0], ax[1]);
    const Q qsp = to_f(about(ax[0] / norm, ax[1] / norm, 0, kClampTilt));
    const V3 free = once(test_config(), Q::identity(), qsp).rate_setpoint;
    const V3 clamped = once(tight, Q::identity(), qsp).rate_setpoint;
    EXPECT_LE(std::fabs(clamped[0]), kClampRoll);
    EXPECT_LE(std::fabs(clamped[1]), kClampPitch);
    // One bound is reached.
    EXPECT_TRUE(std::fabs(clamped[0]) == kClampRoll || std::fabs(clamped[1]) == kClampPitch);
    // The direction of the (roll, pitch) pair is kept: the normalised cross product vanishes.
    const double cross = d(free[0]) * d(clamped[1]) - d(free[1]) * d(clamped[0]);
    const double norms = std::hypot(d(free[0]), d(free[1])) * std::hypot(d(clamped[0]), d(clamped[1]));
    EXPECT_LE(std::fabs(cross), direction_tol * norms);
    // Control: PX4's per-axis clamp of the same vector changes the direction beyond that tolerance when both axes
    // exceed or one does; it does not reproduce the scaled pair, so the direction test above could fail.
    const double px = std::copysign(std::min(std::fabs(d(free[0])), d(kClampRoll)), d(free[0]));
    const double py = std::copysign(std::min(std::fabs(d(free[1])), d(kClampPitch)), d(free[1]));
    const double cross_per_axis = d(free[0]) * py - d(free[1]) * px;
    const double norms_per_axis = std::hypot(d(free[0]), d(free[1])) * std::hypot(px, py);
    if (std::fabs(d(free[0])) > d(kClampRoll) && std::fabs(d(free[1])) > d(kClampPitch)) {
      EXPECT_GT(std::fabs(cross_per_axis), direction_tol * norms_per_axis);
      ++both_exceed;
    }
  }
  EXPECT_GE(both_exceed, 2);
  // Yaw alone is clamped to its own bound, in both signs.
  for (const double psi : {2.0, -2.0}) {
    const Q qsp = to_f(about(0, 0, 1, psi));
    const V3 free = once(test_config(), Q::identity(), qsp).rate_setpoint;
    const V3 clamped = once(tight, Q::identity(), qsp).rate_setpoint;
    EXPECT_GT(std::fabs(free[2]), kClampYaw);
    EXPECT_EQ(clamped[2], std::copysign(kClampYaw, free[2]));
  }
  // A feed-forward that saturates: the bound holds on the sum, in body axes.
  const Q tilted = to_f(about(1, 0, 0, 1.0));
  const V3 ff = once(tight, tilted, tilted, 5.0F).rate_setpoint;
  EXPECT_LE(std::fabs(ff[0]), kClampRoll);
  EXPECT_LE(std::fabs(ff[1]), kClampPitch);
  EXPECT_LE(std::fabs(ff[2]), kClampYaw);
  // Inside the bounds the clamp leaves the output bit-for-bit unchanged.
  const Q small = to_f(about(1, 0, 0, 0.1));
  EXPECT_TRUE(bits_equal(once(tight, Q::identity(), small).rate_setpoint, once(test_config(), Q::identity(), small).rate_setpoint));
}

// ---- faults ------------------------------------------------------------------------------------------------------

struct FaultInput {
  Q q;
  Q q_sp;
  float cmd;
  bool valid;
};

FaultInput valid_input() {
  // Scenario test values: a tilted measured attitude, a setpoint off it and a yaw-rate command, all finite and valid.
  return FaultInput{to_f(about(0.6, 0.0, 0.8, 0.7)), to_f(about(0.0, 1.0, 0.0, 0.4)), 1.0F, true};
}

AttitudeOutput<float> run_input(Rig& rig, const FaultInput& in) {
  const AttitudeState<float> a{rig.t, in.q, V3(), in.valid};
  rig.t += kPeriodUs;
  return rig.law.execute(a, in.q_sp, in.cmd);
}

TEST(L5Attitude, EachFaultInputFailsAndItsValidTwinPasses) {
  struct Case {
    const char* name;
    FaultInput bad;
    FaultInput twin;
  };
  const FaultInput ok = valid_input();
  const auto with = [&](auto&& edit) {
    FaultInput in = ok;
    edit(in);
    return in;
  };
  // The overflow case: a quaternion tilted by 180 degrees about x, so that the rotation of a huge feed-forward doubles it
  // past the float range. Its twin has an ordinary feed-forward.
  const Q tilted(0, 1, 0, 0);
  const std::vector<Case> cases{
      {"valid flag clear", with([](FaultInput& i) { i.valid = false; }), ok},
      {"q NaN", with([](FaultInput& i) { i.q.x = kNaN; }), with([](FaultInput& i) { i.q.x = 0.0F; })},
      {"q infinite", with([](FaultInput& i) { i.q.w = kInf; }), ok},
      {"q zero", with([](FaultInput& i) { i.q = Q(0, 0, 0, 0); }), with([](FaultInput& i) { i.q = Q(1, 0, 0, 0); })},
      {"q norm overflows", with([](FaultInput& i) { i.q = Q(kMaxFloat, kMaxFloat, 0, 0); }),
       with([](FaultInput& i) { i.q = Q(1, 0, 0, 0); })},
      {"q_sp NaN", with([](FaultInput& i) { i.q_sp.z = kNaN; }), with([](FaultInput& i) { i.q_sp.z = 0.0F; })},
      {"q_sp zero", with([](FaultInput& i) { i.q_sp = Q(0, 0, 0, 0); }),
       with([](FaultInput& i) { i.q_sp = Q(1, 0, 0, 0); })},
      {"yaw-rate command NaN", with([](FaultInput& i) { i.cmd = kNaN; }), ok},
      {"yaw-rate command infinite", with([](FaultInput& i) { i.cmd = -kInf; }), ok},
      {"output overflows", FaultInput{tilted, tilted, kMaxFloat, true}, FaultInput{tilted, tilted, 1.0F, true}},
  };
  for (const Case& c : cases) {
    // Control: the valid twin passes, from a fresh law.
    Rig twin_rig;
    const AttitudeOutput<float> twin = run_input(twin_rig, c.twin);
    EXPECT_FALSE(twin.fault_active) << c.name;
    EXPECT_FALSE(twin.fault_latched) << c.name;
    EXPECT_EQ(twin.fault_count, 0U) << c.name;

    Rig rig;
    const AttitudeOutput<float> bad = run_input(rig, c.bad);
    EXPECT_TRUE(bad.fault_active) << c.name;
    EXPECT_TRUE(bad.fault_latched) << c.name;
    EXPECT_EQ(bad.fault_count, 1U) << c.name;
    EXPECT_EQ(bad.rate_setpoint[0], 0.0F) << c.name;
    EXPECT_EQ(bad.rate_setpoint[1], 0.0F) << c.name;
    EXPECT_EQ(bad.rate_setpoint[2], 0.0F) << c.name;
    // The next valid execution runs normally; the latch and the count stay until init.
    const AttitudeOutput<float> after = run_input(rig, ok);
    EXPECT_FALSE(after.fault_active) << c.name;
    EXPECT_TRUE(after.fault_latched) << c.name;
    EXPECT_EQ(after.fault_count, 1U) << c.name;
    EXPECT_TRUE(rig.law.fault_latched());
    EXPECT_EQ(rig.law.fault_count(), 1U);
    // Another fault counts again.
    EXPECT_EQ(run_input(rig, c.bad).fault_count, 2U) << c.name;
    rig.law.init(test_config());
    EXPECT_FALSE(rig.law.fault_latched()) << c.name;
    EXPECT_EQ(rig.law.fault_count(), 0U) << c.name;
  }
}

// ---- the configuration check -------------------------------------------------------------------------------------

TEST(L5AttitudeConfig, ValidateNamesTheFirstViolatedRule) {
  EXPECT_EQ(attitude::validate(test_config()), ConfigError::None);
  const auto with = [](auto&& edit) {
    AttitudeConfig<float> c = test_config();
    edit(c);
    return attitude::validate(c);
  };
  EXPECT_EQ(with([](AttitudeConfig<float>& c) { c.kp = kNaN; }), ConfigError::NonFinite);
  EXPECT_EQ(with([](AttitudeConfig<float>& c) { c.yaw_weight = kNaN; }), ConfigError::NonFinite);
  EXPECT_EQ(with([](AttitudeConfig<float>& c) { c.rate_max[1] = kInf; }), ConfigError::NonFinite);
  EXPECT_EQ(with([](AttitudeConfig<float>& c) { c.kp = -1.0F; }), ConfigError::Gain);
  EXPECT_EQ(with([](AttitudeConfig<float>& c) { c.kp = 0.0F; }), ConfigError::None);
  EXPECT_EQ(with([](AttitudeConfig<float>& c) { c.yaw_weight = 0.0F; }), ConfigError::YawWeight);
  EXPECT_EQ(with([](AttitudeConfig<float>& c) { c.yaw_weight = -0.5F; }), ConfigError::YawWeight);
  EXPECT_EQ(with([](AttitudeConfig<float>& c) { c.yaw_weight = std::nextafter(1.0F, 2.0F); }), ConfigError::YawWeight);
  EXPECT_EQ(with([](AttitudeConfig<float>& c) { c.yaw_weight = 1.0F; }), ConfigError::None);
  EXPECT_EQ(with([](AttitudeConfig<float>& c) { c.period = 0.0F; }), ConfigError::Period);
  EXPECT_EQ(with([](AttitudeConfig<float>& c) { c.tilt_max = 0.0F; }), ConfigError::TiltMax);
  EXPECT_EQ(with([](AttitudeConfig<float>& c) { c.tilt_max = 4.0F; }), ConfigError::TiltMax);  // above pi
  EXPECT_EQ(with([](AttitudeConfig<float>& c) { c.yaw_deadband = -0.1F; }), ConfigError::Deadband);
  EXPECT_EQ(with([](AttitudeConfig<float>& c) { c.yaw_deadband = 1.0F; }), ConfigError::Deadband);
  EXPECT_EQ(with([](AttitudeConfig<float>& c) { c.yaw_alpha_min = 0.0F; }), ConfigError::AlphaMin);
  EXPECT_EQ(with([](AttitudeConfig<float>& c) { c.yaw_t_cross = 0.0F; }), ConfigError::TCross);
  EXPECT_EQ(with([](AttitudeConfig<float>& c) { c.rate_max[2] = 0.0F; }), ConfigError::RateMax);
}

TEST(L5AttitudeConfig, RequireValidPanicsOnAnInvalidWeightAndPassesAValidOne) {
  const auto with_weight = [](float w) {
    AttitudeConfig<float> c = test_config();
    c.yaw_weight = w;
    return c;
  };
  attitude::require_valid(test_config());  // the valid twin returns
  attitude::require_valid(with_weight(1.0F));
  EXPECT_DEATH(attitude::require_valid(with_weight(0.0F)), "yaw weight");
  EXPECT_DEATH(attitude::require_valid(with_weight(std::nextafter(1.0F, 2.0F))), "yaw weight");
  EXPECT_DEATH(attitude::require_valid(with_weight(kNaN)), "not finite");
}

// ---- the period check --------------------------------------------------------------------------------------------

// Runs the law with stamps t0, t0 + s1, ... for the given spacings (us), on valid input.
void run_spacings(const AttitudeConfig<float>& cfg, const std::vector<std::int64_t>& spacings) {
  AttitudeLaw<float> law;
  law.init(cfg);
  std::uint64_t t = kT0Us;
  const AttitudeState<float> a0{t, Q::identity(), V3(), true};
  (void)law.execute(a0, Q::identity(), 0.0F);
  for (const std::int64_t s : spacings) {
    t = static_cast<std::uint64_t>(static_cast<std::int64_t>(t) + s);
    const AttitudeState<float> a{t, Q::identity(), V3(), true};
    (void)law.execute(a, Q::identity(), 0.0F);
  }
}

TEST(L5AttitudePeriod, TheSpacingIsCheckedToPlusOrMinusOneMicrosecond) {
  const AttitudeConfig<float> cfg = test_config();
  const std::int64_t period = static_cast<std::int64_t>(kPeriodUs);
  run_spacings(cfg, {period, period - 1, period + 1, period});  // within the tolerance: no panic (control)
  EXPECT_DEATH(run_spacings(cfg, {period + 2}), "more than 1 us");
  EXPECT_DEATH(run_spacings(cfg, {period - 2}), "more than 1 us");
  EXPECT_DEATH(run_spacings(cfg, {-period}), "more than 1 us");  // a stamp going backwards
  EXPECT_DEATH(run_spacings(cfg, {0}), "more than 1 us");        // a repeated stamp
  EXPECT_DEATH(run_spacings(cfg, {2 * period}), "more than 1 us");  // a dropped execution
  EXPECT_DEATH(run_spacings(cfg, {period, period, period + 2}), "more than 1 us");
}

TEST(L5AttitudePeriod, AHalfMicrosecondPeriodIsCheckedExactly) {
  // Scenario test value: T_a = 312.5 us (one 3.2 kHz rate period), whose stamps alternate 312 and 313 us.
  AttitudeConfig<float> cfg = test_config();
  cfg.period = f(312.5e-6);
  run_spacings(cfg, {312, 313, 312, 313});
  EXPECT_DEATH(run_spacings(cfg, {311}), "more than 1 us");
  EXPECT_DEATH(run_spacings(cfg, {314}), "more than 1 us");
}

TEST(L5AttitudePeriod, NothingIsCheckedAtTheFirstExecutionAfterInitOrReset) {
  const AttitudeConfig<float> cfg = test_config();
  AttitudeLaw<float> law;
  law.init(cfg);
  const AttitudeState<float> first{kT0Us, Q::identity(), V3(), true};
  EXPECT_FALSE(law.execute(first, Q::identity(), 0.0F).fault_active);
  // A jump of a million periods after reset is the first execution again: no check.
  law.reset();
  const AttitudeState<float> jump{kT0Us + 1000000 * kPeriodUs, Q::identity(), V3(), true};
  EXPECT_FALSE(law.execute(jump, Q::identity(), 0.0F).fault_active);
  // And the spacing is checked from there (control: the same jump without a reset panics).
  const AttitudeState<float> jump2{jump.t_us + 1000000 * kPeriodUs, Q::identity(), V3(), true};
  EXPECT_DEATH((void)law.execute(jump2, Q::identity(), 0.0F), "more than 1 us");
}

TEST(L5AttitudePeriod, TheSpacingAfterAFaultIsStillChecked) {
  const AttitudeConfig<float> cfg = test_config();
  AttitudeLaw<float> law;
  law.init(cfg);
  const AttitudeState<float> bad{kT0Us, Q::identity(), V3(), false};
  EXPECT_TRUE(law.execute(bad, Q::identity(), 0.0F).fault_active);
  const AttitudeState<float> late{kT0Us + kPeriodUs + 5, Q::identity(), V3(), true};
  EXPECT_DEATH((void)law.execute(late, Q::identity(), 0.0F), "more than 1 us");
}

}  // namespace
