// The L3 mixer (quad spec 4 L3, decision 0004 items 2 to 5) against the product parameter set: identity with the
// card's effectiveness matrix within the 0004 item 2 bound, the sign contract, the three desaturation cases against
// an independent double-precision bisection oracle, the idle floor, a seeded property sweep and config validation.
//
// Test magnitudes are derived from the card: the per-motor thrust spread S = f_max - f_min, and for each axis the
// torque whose largest per-motor deviation is S/2 (the "axis scale"; on a symmetric card exactly the pure-axis
// authority at a centred collective). Fractions and multiples of those scales are labelled scenario test values.
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
#include <marv/types/actuator.hpp>

namespace {

using namespace marv;
using namespace marv::mixer;

constexpr std::size_t kN = kMotors;
constexpr float kEps = std::numeric_limits<float>::epsilon();

using FMat = prim::Mat<float, kN, kN>;
using Dshots = std::array<DshotValue, kN>;

// ---- the card ------------------------------------------------------------------------------------------------

struct Card {
  MixerConfig<float> cfg;
  FMat b;                               // B-hat, rows [thrust, roll, pitch, yaw], column = motor index
  std::array<int, kN> yaw_sign{};       // rotor_yaw_sign_m<i>
  double lo = 0;                        // f_min, f_max, S from the float config
  double hi = 0;
  double spread = 0;
  double roll_scale = 0;                // torque whose largest |M[i,axis]| * torque equals S/2
  double pitch_scale = 0;
  double yaw_scale = 0;
  double thrust_mid = 0;                // collective at which every motor is centred
  double thrust_top = 0;                // sum of f_max
  double d_idle = 0;                    // ceil(D(omega_idle)), double
};

double esc_map(const MixerConfig<float>& c, double omega) {
  const double d_min = static_cast<double>(prim::kDshotThrottleMin);
  const double d_max = static_cast<double>(prim::kDshotThrottleMax);
  return d_min + (omega - static_cast<double>(c.omega_min)) /
                     (static_cast<double>(c.omega_max) - static_cast<double>(c.omega_min)) * (d_max - d_min);
}

double axis_scale(const MixerConfig<float>& c, Axis a, double spread) {
  double worst = 0;
  for (std::size_t i = 0; i < kN; ++i) {
    worst = std::max(worst, std::abs(static_cast<double>(c.m(i, a))));
  }
  return spread / (2 * worst);
}

const Card* card() {
  static const Card* built = []() -> const Card* {
    if (!params_init(param_defaults())) {
      return nullptr;
    }
    static Card k;
    k.cfg = from_params();
    const std::array<ParamId, kN> xs{ParamId::rotor_position_m1_x, ParamId::rotor_position_m2_x,
                                     ParamId::rotor_position_m3_x, ParamId::rotor_position_m4_x};
    const std::array<ParamId, kN> ys{ParamId::rotor_position_m1_y, ParamId::rotor_position_m2_y,
                                     ParamId::rotor_position_m3_y, ParamId::rotor_position_m4_y};
    const std::array<ParamId, kN> ss{ParamId::rotor_yaw_sign_m1, ParamId::rotor_yaw_sign_m2,
                                     ParamId::rotor_yaw_sign_m3, ParamId::rotor_yaw_sign_m4};
    const float cq = param_value<ParamId::rotor_torque_ratio>();
    for (std::size_t i = 0; i < kN; ++i) {
      k.yaw_sign[i] = static_cast<int>(param_get(ss[i]).value.i32);
      k.b(kThrust, i) = 1.0F;
      k.b(kRoll, i) = -param_get(ys[i]).value.f32;
      k.b(kPitch, i) = param_get(xs[i]).value.f32;
      k.b(kYaw, i) = static_cast<float>(k.yaw_sign[i]) * cq;
    }
    k.lo = static_cast<double>(f_min(k.cfg));
    k.hi = static_cast<double>(f_max(k.cfg));
    k.spread = k.hi - k.lo;
    k.roll_scale = axis_scale(k.cfg, kRoll, k.spread);
    k.pitch_scale = axis_scale(k.cfg, kPitch, k.spread);
    k.yaw_scale = axis_scale(k.cfg, kYaw, k.spread);
    k.thrust_mid = static_cast<double>(kN) * (k.lo + k.hi) / 2;
    k.thrust_top = static_cast<double>(kN) * k.hi;
    k.d_idle = std::ceil(esc_map(k.cfg, static_cast<double>(k.cfg.omega_idle)));
    return &k;
  }();
  return built;
}

#define REQUIRE_CARD()                       \
  const Card* const cp = card();             \
  ASSERT_NE(cp, nullptr) << "params_init";   \
  const Card& c = *cp

Request<float> request(double thrust, double roll, double pitch, double yaw) {
  Request<float> r;
  r.thrust = static_cast<float>(thrust);
  r.torque = prim::Vec3<float>(static_cast<float>(roll), static_cast<float>(pitch), static_cast<float>(yaw));
  return r;
}

// ---- 1. identity ---------------------------------------------------------------------------------------------

struct IdentityResult {
  bool ok = true;
  double worst_ratio = 0;  // max over elements of |fl(M B) - I| / bound
};

// Decision 0004 item 2: |fl(M-hat B-hat) - I| <= (gamma_n + eps) |M||B| per element, n = motor count, eps = float
// epsilon, |M||B| in double.
IdentityResult identity_check(const FMat& m, const FMat& b) {
  const double eps = static_cast<double>(kEps);
  const double n = static_cast<double>(kN);
  const double gamma = n * eps / (1 - n * eps);
  const FMat product = m * b;
  IdentityResult out;
  for (std::size_t i = 0; i < kN; ++i) {
    for (std::size_t j = 0; j < kN; ++j) {
      double abs_sum = 0;
      for (std::size_t k = 0; k < kN; ++k) {
        abs_sum += std::abs(static_cast<double>(m(i, k))) * std::abs(static_cast<double>(b(k, j)));
      }
      const double bound = (gamma + eps) * abs_sum;
      const double err = std::abs(static_cast<double>(product(i, j)) - (i == j ? 1.0 : 0.0));
      out.worst_ratio = std::max(out.worst_ratio, err / bound);
      if (!(err <= bound)) {
        out.ok = false;
      }
    }
  }
  return out;
}

TEST(L3MixerIdentity, CardMixerTimesEffectivenessIsIdentityWithinBound) {
  REQUIRE_CARD();
  const IdentityResult r = identity_check(c.cfg.m, c.b);
  EXPECT_TRUE(r.ok) << "worst error / bound = " << r.worst_ratio;
  EXPECT_LE(r.worst_ratio, 1.0);
}

TEST(L3MixerIdentity, NegativeControlPerturbedEntryIsRejected) {
  REQUIRE_CARD();
  // Perturb M[0, thrust] by 16 times the largest per-element bound; the (0, j) product elements move by that times
  // B[thrust, j] = 1, so they exceed the bound.
  const double eps = static_cast<double>(kEps);
  const double n = static_cast<double>(kN);
  const double gamma = n * eps / (1 - n * eps);
  double worst_abs_sum = 0;
  for (std::size_t i = 0; i < kN; ++i) {
    for (std::size_t j = 0; j < kN; ++j) {
      double s = 0;
      for (std::size_t k = 0; k < kN; ++k) {
        s += std::abs(static_cast<double>(c.cfg.m(i, k))) * std::abs(static_cast<double>(c.b(k, j)));
      }
      worst_abs_sum = std::max(worst_abs_sum, s);
    }
  }
  const double kMultiple = 16;  // scenario test value: well beyond the bound
  FMat bad = c.cfg.m;
  bad(0, kThrust) += static_cast<float>(kMultiple * (gamma + eps) * worst_abs_sum);
  const IdentityResult r = identity_check(bad, c.b);
  EXPECT_FALSE(r.ok);
  EXPECT_GT(r.worst_ratio, 1.0);
}

// ---- oracle: independent bisection in double -----------------------------------------------------------------

struct Oracle {
  double s = 0;
  double t = 0;
};

// Some collective c keeps every motor in [lo, hi] for the roll/pitch scale s and the yaw scale t.
bool feasible(const Card& c, const std::array<double, kN>& a, const std::array<double, kN>& y, double s, double t) {
  double c_lo = -std::numeric_limits<double>::infinity();
  double c_hi = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < kN; ++i) {
    const double p = s * a[i] + t * y[i];
    const double m = static_cast<double>(c.cfg.m(i, kThrust));
    c_lo = std::max(c_lo, (c.lo - p) / m);
    c_hi = std::min(c_hi, (c.hi - p) / m);
  }
  return c_lo <= c_hi;
}

Oracle oracle(const Card& c, double roll, double pitch, double yaw) {
  std::array<double, kN> a{};
  std::array<double, kN> y{};
  for (std::size_t i = 0; i < kN; ++i) {
    a[i] = static_cast<double>(c.cfg.m(i, kRoll)) * roll + static_cast<double>(c.cfg.m(i, kPitch)) * pitch;
    y[i] = static_cast<double>(c.cfg.m(i, kYaw)) * yaw;
  }
  const int kBisections = 200;  // scenario test value: far past double resolution
  Oracle o;
  if (feasible(c, a, y, 1, 0)) {
    o.s = 1;
    if (feasible(c, a, y, 1, 1)) {
      o.t = 1;
    } else {
      double good = 0;
      double bad = 1;
      for (int k = 0; k < kBisections; ++k) {
        const double mid = (good + bad) / 2;
        (feasible(c, a, y, 1, mid) ? good : bad) = mid;
      }
      o.t = good;
    }
  } else {
    double good = 0;
    double bad = 1;
    for (int k = 0; k < kBisections; ++k) {
      const double mid = (good + bad) / 2;
      (feasible(c, a, y, mid, 0) ? good : bad) = mid;
    }
    o.s = good;
  }
  return o;
}

constexpr double kOracleTol = 1e-4;  // scenario test value: float allocation against the double oracle

// B-hat * f against (thrust, roll, pitch, yaw): the forward map of the plant recovers the achieved wrench.
void expect_wrench(const Card& c, const Allocation<float>& al) {
  const float eps16 = 16 * kEps;  // scenario test value: slack over the float rounding of B f
  const std::array<float, kAxes> want{al.achieved_thrust, al.achieved_torque[0], al.achieved_torque[1],
                                      al.achieved_torque[2]};
  for (std::size_t r = 0; r < kAxes; ++r) {
    float got = 0;
    float scale = 0;
    for (std::size_t i = 0; i < kN; ++i) {
      got += c.b(r, i) * al.f[i];
      scale += std::abs(c.b(r, i) * al.f[i]);
    }
    EXPECT_NEAR(got, want[r], eps16 * scale) << "axis " << r;
  }
}

// ---- 2. sign contract ----------------------------------------------------------------------------------------

TEST(L3MixerSign, UnsaturatedRollPitchYawRaiseTheContractedMotors) {
  REQUIRE_CARD();
  const double fraction = 0.2;  // scenario test value: an unsaturated fraction of the axis scale
  const Request<float> zero = request(c.thrust_mid, 0, 0, 0);
  const Allocation<float> base = allocate(c.cfg, zero);
  const Dshots base_dshot = thrust_to_dshot(c.cfg, base.f);

  struct Case {
    const char* name;
    Request<float> req;
    std::array<bool, kN> raised;  // logical motor m is at index m - 1
  };
  std::array<bool, kN> yaw_raised{};
  for (std::size_t i = 0; i < kN; ++i) {
    yaw_raised[i] = c.yaw_sign[i] == 1;
  }
  const std::array<Case, 3> cases{{
      {"+roll raises motors 2, 3", request(c.thrust_mid, fraction * c.roll_scale, 0, 0), {false, true, true, false}},
      {"+pitch raises motors 1, 3", request(c.thrust_mid, 0, fraction * c.pitch_scale, 0), {true, false, true, false}},
      {"+yaw raises the yaw_sign = +1 motors", request(c.thrust_mid, 0, 0, fraction * c.yaw_scale), yaw_raised},
  }};
  for (const Case& k : cases) {
    const Allocation<float> al = allocate(c.cfg, k.req);
    const Dshots d = thrust_to_dshot(c.cfg, al.f);
    EXPECT_FALSE(al.flags.roll || al.flags.pitch || al.flags.yaw) << k.name;
    for (std::size_t i = 0; i < kN; ++i) {
      if (k.raised[i]) {
        EXPECT_GT(al.f[i], base.f[i]) << k.name << " f, motor " << i + 1;
        EXPECT_GT(d[i].raw(), base_dshot[i].raw()) << k.name << " DShot, motor " << i + 1;
      } else {
        EXPECT_LT(al.f[i], base.f[i]) << k.name << " f, motor " << i + 1;
        EXPECT_LT(d[i].raw(), base_dshot[i].raw()) << k.name << " DShot, motor " << i + 1;
      }
    }
    expect_wrench(c, al);
  }
}

// ---- 3. case a: roll/pitch beyond the spread -----------------------------------------------------------------

TEST(L3MixerCaseA, RollPitchBeyondSpreadIsScaledTogetherAndYawIsDropped) {
  REQUIRE_CARD();
  const double roll = 1.0 * c.roll_scale;    // scenario test value: each axis alone is at full authority, both
  const double pitch = 1.0 * c.pitch_scale;  // together exceed the spread
  for (const double yaw : {0.1 * c.yaw_scale, -0.1 * c.yaw_scale}) {  // scenario test value: a small yaw request of
                                                                      // either sign, which must be dropped
    const Oracle want = oracle(c, roll, pitch, yaw);
    ASSERT_GT(want.s, 0.0);
    ASSERT_LT(want.s, 1.0) << "scenario premise: roll/pitch beyond the spread";

    const Request<float> req = request(c.thrust_mid, roll, pitch, yaw);
    const Allocation<float> al = allocate(c.cfg, req);
    const float s = al.achieved_torque[0] / req.torque[0];
    EXPECT_NEAR(static_cast<double>(s), want.s, kOracleTol);
    EXPECT_LT(s, 1.0F);
    EXPECT_FLOAT_EQ(al.achieved_torque[1] / req.torque[1], s);  // roll:pitch ratio preserved
    EXPECT_EQ(al.achieved_torque[2], 0.0F);
    EXPECT_TRUE(al.flags.roll);
    EXPECT_TRUE(al.flags.pitch);
    EXPECT_TRUE(al.flags.yaw);
    double f_lowest = c.hi;
    double f_highest = c.lo;
    for (const float f : al.f) {
      f_lowest = std::min(f_lowest, static_cast<double>(f));
      f_highest = std::max(f_highest, static_cast<double>(f));
    }
    EXPECT_NEAR(f_lowest, c.lo, 1e-5 * c.spread);  // scenario test value: s is the largest, the spread is used up
    EXPECT_NEAR(f_highest, c.hi, 1e-5 * c.spread);
    expect_wrench(c, al);
  }

  const Allocation<float> no_yaw = allocate(c.cfg, request(c.thrust_mid, roll, pitch, 0));
  EXPECT_TRUE(no_yaw.flags.roll);
  EXPECT_TRUE(no_yaw.flags.pitch);
  EXPECT_FALSE(no_yaw.flags.yaw);
}

// ---- 4. case b: air mode -------------------------------------------------------------------------------------

TEST(L3MixerCaseB, RollPitchKeepFullAuthorityAtZeroAndTopThrust) {
  REQUIRE_CARD();
  const double fraction = 0.4;  // scenario test value: roll/pitch that fit the spread only with the collective moved
  const double roll = fraction * c.roll_scale;
  const double pitch = fraction * c.pitch_scale;
  ASSERT_EQ(oracle(c, roll, pitch, 0).s, 1.0) << "scenario premise: roll/pitch fit the spread";

  for (const double thrust : {0.0, c.thrust_top}) {
    const Request<float> req = request(thrust, roll, pitch, 0);
    const Allocation<float> al = allocate(c.cfg, req);
    EXPECT_EQ(al.achieved_torque[0], req.torque[0]) << "thrust " << thrust;
    EXPECT_EQ(al.achieved_torque[1], req.torque[1]) << "thrust " << thrust;
    EXPECT_FALSE(al.flags.roll || al.flags.pitch || al.flags.yaw) << "thrust " << thrust;
    EXPECT_NE(al.achieved_thrust, req.thrust) << "thrust " << thrust;
    double f_lowest = c.hi;
    double f_highest = c.lo;
    for (const float f : al.f) {
      f_lowest = std::min(f_lowest, static_cast<double>(f));
      f_highest = std::max(f_highest, static_cast<double>(f));
    }
    if (thrust == 0.0) {
      EXPECT_GT(al.achieved_thrust, req.thrust);
      EXPECT_NEAR(f_lowest, c.lo, 1e-5 * c.spread);  // collective raised just enough
    } else {
      EXPECT_LT(al.achieved_thrust, req.thrust);
      EXPECT_NEAR(f_highest, c.hi, 1e-5 * c.spread);  // collective lowered just enough
    }
    expect_wrench(c, al);
  }
}

// ---- 5. case c: yaw ------------------------------------------------------------------------------------------

TEST(L3MixerCaseC, PureYawAtZeroThrustShiftsTheCollectiveUp) {
  REQUIRE_CARD();
  const double yaw = 0.8 * c.yaw_scale;  // scenario test value: fits the spread only with the collective raised
  ASSERT_EQ(oracle(c, 0, 0, yaw).t, 1.0) << "scenario premise";
  const Request<float> req = request(0, 0, 0, yaw);
  const Allocation<float> al = allocate(c.cfg, req);
  EXPECT_EQ(al.achieved_torque[2], req.torque[2]);
  EXPECT_FALSE(al.flags.roll || al.flags.pitch || al.flags.yaw);
  EXPECT_GT(al.achieved_thrust, 0.0F);
  expect_wrench(c, al);
}

TEST(L3MixerCaseC, YawBeyondHeadroomIsClippedAlone) {
  REQUIRE_CARD();
  const double fraction = 0.4;  // scenario test value: feasible roll/pitch, as in case b
  const double roll = fraction * c.roll_scale;
  const double pitch = fraction * c.pitch_scale;
  const double yaw = 1.0 * c.yaw_scale;  // scenario test value: beyond the headroom left by that roll/pitch
  const Oracle want = oracle(c, roll, pitch, yaw);
  ASSERT_EQ(want.s, 1.0) << "scenario premise: roll/pitch feasible";
  ASSERT_GT(want.t, 0.0);
  ASSERT_LT(want.t, 1.0) << "scenario premise: yaw beyond the headroom";

  const Request<float> req = request(0, roll, pitch, yaw);
  const Allocation<float> al = allocate(c.cfg, req);
  EXPECT_EQ(al.achieved_torque[0], req.torque[0]);
  EXPECT_EQ(al.achieved_torque[1], req.torque[1]);
  EXPECT_NEAR(static_cast<double>(al.achieved_torque[2] / req.torque[2]), want.t, kOracleTol);
  EXPECT_FALSE(al.flags.roll);
  EXPECT_FALSE(al.flags.pitch);
  EXPECT_TRUE(al.flags.yaw);
  expect_wrench(c, al);

  const Request<float> alone = request(c.thrust_mid, 0, 0, 1.5 * c.yaw_scale);  // scenario test value
  const Oracle alone_want = oracle(c, 0, 0, static_cast<double>(alone.torque[2]));
  const Allocation<float> al2 = allocate(c.cfg, alone);
  EXPECT_NEAR(static_cast<double>(al2.achieved_torque[2] / alone.torque[2]), alone_want.t, kOracleTol);
  EXPECT_TRUE(al2.flags.yaw);
  EXPECT_FALSE(al2.flags.roll || al2.flags.pitch);
}

// ---- 6. idle -------------------------------------------------------------------------------------------------

TEST(L3MixerIdle, ZeroRequestIsIdleOnEveryMotor) {
  REQUIRE_CARD();
  ASSERT_EQ(c.cfg.omega_idle, c.cfg.omega_min) << "the idle stand-in is the ESC-map minimum (0004 item 1)";
  const std::uint16_t idle = static_cast<std::uint16_t>(c.d_idle);
  EXPECT_EQ(idle, prim::kDshotThrottleMin);
  for (const double thrust : {0.0, -c.thrust_top}) {  // a zero request, and a thrust request below zero
    const MixerOutput<float> out = mix(c.cfg, request(thrust, 0, 0, 0));
    for (std::size_t i = 0; i < kN; ++i) {
      EXPECT_EQ(out.dshot[i].raw(), idle) << "motor " << i + 1 << " thrust " << thrust;
    }
    EXPECT_FALSE(out.flags.roll || out.flags.pitch || out.flags.yaw);
  }
}

TEST(L3MixerDshot, InverseEscMapRoundTripsEveryCommand) {
  REQUIRE_CARD();
  // The ESC map is a labelled scenario value (0004 item 5): omega(D) linear between the two endpoints.
  const std::uint16_t idle = static_cast<std::uint16_t>(c.d_idle);
  for (std::uint16_t d = idle; d <= prim::kDshotThrottleMax; ++d) {
    const double omega = static_cast<double>(c.cfg.omega_min) +
                         (static_cast<double>(c.cfg.omega_max) - static_cast<double>(c.cfg.omega_min)) *
                             (static_cast<double>(d) - prim::kDshotThrottleMin) /
                             (static_cast<double>(prim::kDshotThrottleMax) - prim::kDshotThrottleMin);
    const float f = static_cast<float>(static_cast<double>(c.cfg.thrust_coeff) * omega * omega);
    std::array<float, kN> fs{};
    fs.fill(f);
    for (const DshotValue v : thrust_to_dshot(c.cfg, fs)) {
      EXPECT_EQ(v.raw(), d);
    }
  }
}

// ---- 7. property sweep ---------------------------------------------------------------------------------------

constexpr std::uint64_t kSweepSeed = 20260930;  // scenario test value: the seed of the sweep
constexpr int kSweepCount = 20000;              // scenario test value: samples
constexpr unsigned kNonFiniteOneIn = 32;        // scenario test value: a component is NaN or +-inf one time in this
constexpr std::array<double, 4> kSweepSpans{0.25, 1.0, 4.0, 100.0};  // scenario test values: multiples of the
                                                                      // axis scale, up to far beyond the envelope

// nullptr when the sample satisfies every property, else the violated one.
const char* property_violation(const Card& c, const Request<float>& req, const Allocation<float>& al,
                               const Dshots& dshot) {
  const float lo = f_min(c.cfg);
  const float hi = f_max(c.cfg);
  for (const float f : al.f) {
    if (!(f >= lo && f <= hi)) {
      return "a motor thrust is outside [f_min, f_max]";
    }
  }
  const std::uint16_t d_lo = static_cast<std::uint16_t>(c.d_idle);
  for (const DshotValue v : dshot) {
    if (!(v.raw() >= d_lo && v.raw() <= prim::kDshotThrottleMax)) {
      return "a DShot value is outside [idle, 2047]";
    }
  }
  const std::array<bool, 3> flags{al.flags.roll, al.flags.pitch, al.flags.yaw};
  for (std::size_t k = 0; k < flags.size(); ++k) {
    const float r = req.torque[k];
    if (!std::isfinite(r)) {
      continue;
    }
    const float a = al.achieved_torque[k];
    if (!(std::abs(a) <= std::abs(r))) {
      return "achieved torque magnitude exceeds the request";
    }
    if (std::abs(a) < std::abs(r) && !flags[k]) {
      return "achieved < requested without the flag";
    }
    if (flags[k] && r == 0.0F) {
      return "flag set for a zero request";
    }
    if (!flags[k] && std::abs(a) != std::abs(r)) {
      return "no flag, but achieved differs from requested";
    }
  }
  return nullptr;
}

double draw_component(std::mt19937_64& g, double scale) {
  const double u = std::ldexp(static_cast<double>(g() >> 11), -53);  // [0, 1), 53 bits: portable, unlike distributions
  const double span = kSweepSpans[g() % kSweepSpans.size()];
  if (g() % kNonFiniteOneIn == 0) {
    switch (g() % 3) {
      case 0:
        return std::numeric_limits<double>::quiet_NaN();
      case 1:
        return std::numeric_limits<double>::infinity();
      default:
        return -std::numeric_limits<double>::infinity();
    }
  }
  return (2 * u - 1) * span * scale;
}

TEST(L3MixerProperty, RandomizedSweepStaysInsideTheEnvelope) {
  REQUIRE_CARD();
  std::mt19937_64 g(kSweepSeed);
  int unsaturated = 0;
  int case_a = 0;
  int case_c = 0;
  for (int n = 0; n < kSweepCount; ++n) {
    const Request<float> req = request(draw_component(g, c.thrust_top), draw_component(g, c.roll_scale),
                                       draw_component(g, c.pitch_scale), draw_component(g, c.yaw_scale));
    const Allocation<float> al = allocate(c.cfg, req);
    const Dshots d = thrust_to_dshot(c.cfg, al.f);
    const char* why = property_violation(c, req, al, d);
    ASSERT_EQ(why, nullptr) << "sample " << n << ": " << why << " thrust " << req.thrust << " torque "
                            << req.torque[0] << ", " << req.torque[1] << ", " << req.torque[2];
    const MixerOutput<float> out = mix(c.cfg, req);
    ASSERT_TRUE(out.dshot == d) << "sample " << n;
    ASSERT_TRUE(out.achieved_thrust == al.achieved_thrust || std::isnan(al.achieved_thrust));
    ASSERT_EQ(out.flags.roll, al.flags.roll);
    ASSERT_EQ(out.flags.pitch, al.flags.pitch);
    ASSERT_EQ(out.flags.yaw, al.flags.yaw);
    if (std::isfinite(req.torque[0]) && std::isfinite(req.torque[1]) && std::isfinite(req.torque[2])) {
      if (al.flags.roll || al.flags.pitch) {
        ++case_a;
      } else if (al.flags.yaw) {
        ++case_c;
      } else {
        ++unsaturated;
      }
    }
  }
  // The sweep must actually visit every regime, or the properties above hold vacuously.
  EXPECT_GT(unsaturated, 0);
  EXPECT_GT(case_a, 0);
  EXPECT_GT(case_c, 0);
}

TEST(L3MixerProperty, NegativeControlCheckerRejectsPlantedViolations) {
  REQUIRE_CARD();
  const double fraction = 0.3;  // scenario test value: an unsaturated roll request, the clean base for the plants
  const Request<float> req = request(c.thrust_mid, fraction * c.roll_scale, 0, 0);
  const Allocation<float> good = allocate(c.cfg, req);
  const Dshots good_d = thrust_to_dshot(c.cfg, good.f);
  ASSERT_EQ(property_violation(c, req, good, good_d), nullptr);

  Allocation<float> bad = good;
  bad.f[0] = f_max(c.cfg) * 2;  // a motor above f_max
  EXPECT_NE(property_violation(c, req, bad, good_d), nullptr);
  bad = good;
  bad.f[1] = f_min(c.cfg) / 2;  // a motor below f_min
  EXPECT_NE(property_violation(c, req, bad, good_d), nullptr);
  bad = good;
  bad.f[2] = std::numeric_limits<float>::quiet_NaN();
  EXPECT_NE(property_violation(c, req, bad, good_d), nullptr);
  Dshots low = good_d;
  low[3] = DshotValue::stop();  // 0: below idle
  EXPECT_NE(property_violation(c, req, good, low), nullptr);
  bad = good;
  bad.achieved_torque[0] = req.torque[0] * 2;  // achieved beyond the request
  EXPECT_NE(property_violation(c, req, bad, good_d), nullptr);
  bad = good;
  bad.flags.pitch = true;  // flag on an axis with a zero request
  EXPECT_NE(property_violation(c, req, bad, good_d), nullptr);
  bad = good;
  bad.achieved_torque[0] = req.torque[0] / 2;  // shortfall without the flag
  EXPECT_NE(property_violation(c, req, bad, good_d), nullptr);
}

// ---- 8. config validation ------------------------------------------------------------------------------------

TEST(L3MixerConfig, TheCardConfigIsValid) {
  REQUIRE_CARD();
  EXPECT_EQ(validate(c.cfg), ConfigError::None);
}

TEST(L3MixerConfig, EveryInvalidConfigIsRejectedWithItsReason) {
  REQUIRE_CARD();
  const MixerConfig<float> ok = c.cfg;
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();

  struct Case {
    const char* name;
    MixerConfig<float> cfg;
    ConfigError want;
  };
  std::vector<Case> cases;
  auto add = [&](const char* name, ConfigError want, auto mutate) {
    MixerConfig<float> m = ok;
    mutate(m);
    cases.push_back({name, m, want});
  };
  add("k = 0", ConfigError::ThrustCoeff, [](auto& m) { m.thrust_coeff = 0; });
  add("k < 0", ConfigError::ThrustCoeff, [](auto& m) { m.thrust_coeff = -m.thrust_coeff; });
  add("k NaN", ConfigError::NonFinite, [&](auto& m) { m.thrust_coeff = nan; });
  add("omega_idle NaN", ConfigError::NonFinite, [&](auto& m) { m.omega_idle = nan; });
  add("omega_max inf", ConfigError::NonFinite, [&](auto& m) { m.omega_max = inf; });
  add("omega_idle < 0", ConfigError::IdleNegative, [](auto& m) { m.omega_idle = -m.omega_idle; });
  add("omega_idle < omega_min", ConfigError::IdleBelowMin,
      [](auto& m) { m.omega_idle = std::nextafter(m.omega_min, 0.0F); });
  add("omega_min = omega_max", ConfigError::MinNotBelowMax, [](auto& m) {
    m.omega_min = m.omega_max;
    m.omega_idle = m.omega_max;
  });
  add("omega_min > omega_max", ConfigError::MinNotBelowMax, [](auto& m) {
    m.omega_min = m.omega_max + 1;
    m.omega_idle = m.omega_min;
  });
  add("omega_idle = omega_max", ConfigError::IdleNotBelowMax, [](auto& m) { m.omega_idle = m.omega_max; });
  add("omega_idle > omega_max", ConfigError::IdleNotBelowMax, [](auto& m) { m.omega_idle = 2 * m.omega_max; });
  for (std::size_t i = 0; i < kN; ++i) {
    add("M[i, thrust] = 0", ConfigError::ThrustColumn, [i](auto& m) { m.m(i, kThrust) = 0; });
    add("M[i, thrust] < 0", ConfigError::ThrustColumn, [i](auto& m) { m.m(i, kThrust) = -m.m(i, kThrust); });
    add("M[i, yaw] NaN", ConfigError::NonFinite, [&, i](auto& m) { m.m(i, kYaw) = nan; });
    // One motor's thrust weight more than f_max/f_min times another's: no collective puts both inside the limits.
    add("zero torque infeasible", ConfigError::ZeroTorqueInfeasible, [&, i](auto& m) {
      m.m(i, kThrust) = m.m(i, kThrust) * 2 * (f_max(m) / f_min(m));
    });
  }
  for (const Case& k : cases) {
    EXPECT_EQ(validate(k.cfg), k.want) << k.name;
  }
}

TEST(L3MixerConfigDeathTest, InvalidConfigPanicsAndTheCardConfigLoads) {
  REQUIRE_CARD();
  MixerConfig<float> bad = c.cfg;
  bad.thrust_coeff = 0;
  EXPECT_DEATH(require_valid(bad), "rotor_thrust_coeff is not positive");
  bad = c.cfg;
  bad.m(0, kThrust) = 0;
  EXPECT_DEATH(require_valid(bad), "thrust-column entry");
  const MixerConfig<float> loaded = load_config();
  EXPECT_EQ(validate(loaded), ConfigError::None);
}

}  // namespace
