// The scripted inputs of the l4_rate_scripted composition (marv/l4_script.hpp): the piecewise-constant setpoint, the
// exponential-sweep chirp and their validation. The chirp is checked against its closed form evaluated in double at the
// sample stamps within a derived float bound, with a negative control.
//
// Chirp float bound (basic operation rounds by at most u = eps/2, a libm call by at most 2u, where eps is the float
// epsilon), following the evaluation order of chirp_value: L = log1p((w_hi - w_lo) / w_lo) has relative error
// <= 2u + 2u = 4u; x = L (tau_us / dur_us) <= 4u + 4u = 8u (two integer conversions, one division, one product);
// expm1(x) <= kappa 8u + 2u with kappa = x e^x / (e^x - 1) <= 1 + L for 0 <= x <= L; w_lo dur_s / L <= 8u (dur_s: 2u,
// the product: u, the division: u + the 4u of L); phi = product <= u more. Total relative error of phi
// <= 11u + 8(1 + L) u <= 10 eps (1 + L). sin is 1-Lipschitz, sin and the product with A add <= eps, so
//   |d_float - d_double| <= A (10 eps (1 + L) |phi| + 2 eps)
// with phi and L from the double closed form (the parameters are the exact float values).
#include <gtest/gtest.h>

#include <marv/l4_script.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace {

using namespace marv;
using namespace marv::composition;

constexpr std::size_t kCapacity = 8;  // scenario test value: any capacity >= 3 serves the schedule tests
using Script = SetpointScript<float, kCapacity>;

Script three_segments() {
  Script s;
  s.count = 3;
  // scenario test values: stamps in us and distinct per-axis rates in rad/s, so a mixed-up axis or segment shows
  s.segment[0] = {1000, prim::Vec3<float>(1.0F, 2.0F, 3.0F)};
  s.segment[1] = {2000, prim::Vec3<float>(-1.0F, 0.0F, 4.0F)};
  s.segment[2] = {5000, prim::Vec3<float>(0.5F, -2.0F, 0.0F)};
  // an unused fourth segment must never apply
  s.segment[3] = {6000, prim::Vec3<float>(99.0F, 99.0F, 99.0F)};
  return s;
}

void expect_rate(const Script& s, TimeUs t, float roll, float pitch, float yaw) {
  const prim::Vec3<float> sp = setpoint_at(s, t);
  EXPECT_EQ(sp[0], roll) << "t = " << t;
  EXPECT_EQ(sp[1], pitch) << "t = " << t;
  EXPECT_EQ(sp[2], yaw) << "t = " << t;
}

TEST(L4ScriptSetpoint, ZeroSegmentsIsAZeroSetpointAtEveryStamp) {
  const Script s;
  for (const TimeUs t : {TimeUs{0}, TimeUs{1}, TimeUs{1000000}, std::numeric_limits<TimeUs>::max()}) {
    expect_rate(s, t, 0.0F, 0.0F, 0.0F);
  }
}

TEST(L4ScriptSetpoint, CountZeroIgnoresPopulatedSegments) {
  Script s = three_segments();
  s.count = 0;
  expect_rate(s, 3000, 0.0F, 0.0F, 0.0F);
}

TEST(L4ScriptSetpoint, BeforeTheFirstSegmentTheSetpointIsZero) {
  const Script s = three_segments();
  expect_rate(s, 0, 0.0F, 0.0F, 0.0F);
  expect_rate(s, 999, 0.0F, 0.0F, 0.0F);
}

TEST(L4ScriptSetpoint, ASegmentAppliesFromItsStampExactly) {
  const Script s = three_segments();
  expect_rate(s, 1000, 1.0F, 2.0F, 3.0F);
  expect_rate(s, 2000, -1.0F, 0.0F, 4.0F);
  expect_rate(s, 5000, 0.5F, -2.0F, 0.0F);
  expect_rate(s, 1999, 1.0F, 2.0F, 3.0F);
  expect_rate(s, 4999, -1.0F, 0.0F, 4.0F);
}

TEST(L4ScriptSetpoint, BetweenSegmentsTheEarlierOneHolds) {
  const Script s = three_segments();
  expect_rate(s, 1500, 1.0F, 2.0F, 3.0F);
  expect_rate(s, 3500, -1.0F, 0.0F, 4.0F);
}

TEST(L4ScriptSetpoint, AfterTheLastUsedSegmentItHoldsForever) {
  const Script s = three_segments();
  expect_rate(s, 6000, 0.5F, -2.0F, 0.0F);
  expect_rate(s, 60000000, 0.5F, -2.0F, 0.0F);
  expect_rate(s, std::numeric_limits<TimeUs>::max(), 0.5F, -2.0F, 0.0F);
}

TEST(L4ScriptSetpoint, ASegmentAtStampZeroAppliesFromTheFirstTick) {
  Script s;
  s.count = 1;
  s.segment[0] = {0, prim::Vec3<float>(7.0F, 0.0F, 0.0F)};
  expect_rate(s, 0, 7.0F, 0.0F, 0.0F);
}

TEST(L4ScriptValidate, AcceptsTheSchedulesAboveAndTheDefaults) {
  EXPECT_EQ(validate(three_segments()), ScriptError::None);
  EXPECT_EQ(validate(Script{}), ScriptError::None);
  EXPECT_EQ(validate(Chirp<float>{}), ScriptError::None);
}

TEST(L4ScriptValidate, RejectsEachSegmentRule) {
  Script s = three_segments();
  s.count = -1;
  EXPECT_EQ(validate(s), ScriptError::SegmentCount);
  s.count = static_cast<std::int32_t>(kCapacity) + 1;
  EXPECT_EQ(validate(s), ScriptError::SegmentCount);
  s = three_segments();
  s.segment[1].t_us = s.segment[0].t_us;  // not strictly increasing
  EXPECT_EQ(validate(s), ScriptError::SegmentTime);
  s = three_segments();
  s.segment[2].t_us = s.segment[1].t_us - 1;
  EXPECT_EQ(validate(s), ScriptError::SegmentTime);
  s = three_segments();
  s.segment[0].t_us = -1;
  EXPECT_EQ(validate(s), ScriptError::SegmentTime);
  s = three_segments();
  s.segment[1].rate[2] = std::numeric_limits<float>::quiet_NaN();
  EXPECT_EQ(validate(s), ScriptError::SegmentRate);
  s = three_segments();
  s.segment[0].rate[0] = std::numeric_limits<float>::infinity();
  EXPECT_EQ(validate(s), ScriptError::SegmentRate);
}

TEST(L4ScriptValidate, UnusedSegmentsAreNotChecked) {
  Script s = three_segments();
  s.segment[3].t_us = -5;
  s.segment[4].rate[1] = std::numeric_limits<float>::quiet_NaN();
  EXPECT_EQ(validate(s), ScriptError::None);
}

// scenario test values: a 5..500 rad/s sweep of 2 s starting at 0.1 s, amplitude 0.01 N*m
Chirp<float> test_chirp() {
  Chirp<float> c;
  c.axis = ChirpAxis::Pitch;
  c.amp = 0.01F;
  c.w_lo = 5.0F;
  c.w_hi = 500.0F;
  c.t0_us = 100000;
  c.dur_us = 2000000;
  return c;
}

TEST(L4ScriptValidate, RejectsEachChirpRule) {
  Chirp<float> c = test_chirp();
  EXPECT_EQ(validate(c), ScriptError::None);
  c.axis = static_cast<ChirpAxis>(4);
  EXPECT_EQ(validate(c), ScriptError::ChirpAxis);
  c.axis = static_cast<ChirpAxis>(-1);
  EXPECT_EQ(validate(c), ScriptError::ChirpAxis);
  c = test_chirp();
  c.amp = -0.01F;
  EXPECT_EQ(validate(c), ScriptError::ChirpAmplitude);
  c.amp = std::numeric_limits<float>::quiet_NaN();
  EXPECT_EQ(validate(c), ScriptError::ChirpAmplitude);
  c = test_chirp();
  c.w_lo = 0.0F;
  EXPECT_EQ(validate(c), ScriptError::ChirpBand);
  c = test_chirp();
  c.w_hi = c.w_lo;
  EXPECT_EQ(validate(c), ScriptError::ChirpBand);
  c = test_chirp();
  c.w_hi = std::numeric_limits<float>::infinity();
  EXPECT_EQ(validate(c), ScriptError::ChirpBand);
  c = test_chirp();
  c.dur_us = 0;
  EXPECT_EQ(validate(c), ScriptError::ChirpDuration);
  c = test_chirp();
  c.t0_us = -1;
  EXPECT_EQ(validate(c), ScriptError::ChirpStart);
}

TEST(L4ScriptValidate, TheChirpBandIsNotCheckedWhenTheAxisIsNone) {
  Chirp<float> c;
  c.axis = ChirpAxis::None;
  c.amp = 0.5F;
  EXPECT_EQ(validate(c), ScriptError::None);
}

// The closed form of the header comment in double.
double closed_form(const Chirp<float>& c, TimeUs t_us, double* phi_out) {
  const double w_lo = c.w_lo;
  const double w_hi = c.w_hi;
  const double dur = static_cast<double>(c.dur_us) / prim::kMicrosecondsPerSecond;
  const double tau = (static_cast<double>(t_us) - static_cast<double>(c.t0_us)) / prim::kMicrosecondsPerSecond;
  const double sweep = std::log(w_hi / w_lo);
  const double phi = w_lo * dur / sweep * (std::exp(sweep * tau / dur) - 1.0);
  *phi_out = phi;
  return static_cast<double>(c.amp) * std::sin(phi);
}

double bound(const Chirp<float>& c, double phi) {
  const double eps = std::numeric_limits<float>::epsilon();
  const double sweep = std::log(static_cast<double>(c.w_hi) / static_cast<double>(c.w_lo));
  const double ten = 10.0;  // scenario test value: the 10 of the derivation in the file header
  return static_cast<double>(c.amp) * (ten * eps * (1.0 + sweep) * std::abs(phi) + 2.0 * eps);
}

// Sample stamps of the 6.4 kHz tick (625/4 us) from before the sweep to after it.
constexpr std::uint64_t kNum = 625;
constexpr std::uint64_t kDen = 4;
TimeUs stamp(std::uint64_t n) { return n * kNum / kDen; }

struct Scan {
  double worst_margin = -std::numeric_limits<double>::infinity();  // max over samples of |error| - bound
  double max_abs = 0.0;
  std::size_t inside = 0;
};

// Scans the float chirp `actual` against the closed form of `reference`.
Scan scan(const Chirp<float>& actual, const Chirp<float>& reference) {
  Scan r;
  const std::uint64_t last = (static_cast<std::uint64_t>(reference.t0_us) + static_cast<std::uint64_t>(reference.dur_us)) *
                                 kDen / kNum +
                             100;
  for (std::uint64_t n = 0; n <= last; ++n) {
    const TimeUs t = stamp(n);
    const double got = chirp_value(actual, t);
    if (t >= static_cast<TimeUs>(reference.t0_us) &&
        t < static_cast<TimeUs>(reference.t0_us) + static_cast<TimeUs>(reference.dur_us)) {
      double phi = 0.0;
      const double want = closed_form(reference, t, &phi);
      r.worst_margin = std::max(r.worst_margin, std::abs(got - want) - bound(reference, phi));
      r.max_abs = std::max(r.max_abs, std::abs(got));
      ++r.inside;
    }
  }
  return r;
}

TEST(L4ScriptChirp, MatchesTheClosedFormInDoubleWithinTheDerivedBound) {
  const Chirp<float> c = test_chirp();
  const Scan r = scan(c, c);
  EXPECT_GT(r.inside, std::size_t{10000});
  EXPECT_LE(r.worst_margin, 0.0);
  // the check has power: the chirp actually swings through most of its amplitude
  EXPECT_GT(r.max_abs, 0.9 * static_cast<double>(c.amp));
}

TEST(L4ScriptChirp, NegativeControlAWrongUpperFrequencyBreaksTheCheck) {
  const Chirp<float> c = test_chirp();
  Chirp<float> wrong = c;
  wrong.w_hi = c.w_hi * 1.05F;  // scenario test value: 5 % high
  const Scan r = scan(wrong, c);
  EXPECT_GT(r.worst_margin, 0.0);
}

TEST(L4ScriptChirp, NegativeControlAWrongDurationBreaksTheCheck) {
  const Chirp<float> c = test_chirp();
  Chirp<float> wrong = c;
  wrong.dur_us = c.dur_us + c.dur_us / 20;  // scenario test value: 5 % long
  const Scan r = scan(wrong, c);
  EXPECT_GT(r.worst_margin, 0.0);
}

TEST(L4ScriptChirp, IsZeroOutsideTheSweepWindowAndAtItsEdgesAsDefined) {
  const Chirp<float> c = test_chirp();
  const auto t0 = static_cast<TimeUs>(c.t0_us);
  const auto dur = static_cast<TimeUs>(c.dur_us);
  EXPECT_EQ(chirp_value(c, 0), 0.0F);
  EXPECT_EQ(chirp_value(c, t0 - 1), 0.0F);
  EXPECT_EQ(chirp_value(c, t0), 0.0F);  // phi(0) = 0
  EXPECT_EQ(chirp_value(c, t0 + dur), 0.0F);
  EXPECT_EQ(chirp_value(c, t0 + dur + 1), 0.0F);
  EXPECT_EQ(chirp_value(c, std::numeric_limits<TimeUs>::max()), 0.0F);
  EXPECT_NE(chirp_value(c, t0 + dur - 1000), 0.0F);
}

TEST(L4ScriptChirp, AxisNoneIsZeroEverywhereAndTheTorqueLandsOnTheChirpAxisOnly) {
  Chirp<float> c = test_chirp();
  const TimeUs t = static_cast<TimeUs>(c.t0_us) + 12345;
  ASSERT_NE(chirp_value(c, t), 0.0F);
  const float d = chirp_value(c, t);
  for (const ChirpAxis a : {ChirpAxis::Roll, ChirpAxis::Pitch, ChirpAxis::Yaw}) {
    c.axis = a;
    const prim::Vec3<float> v = chirp_torque(c, t);
    const std::size_t index = static_cast<std::size_t>(a) - 1;
    for (std::size_t i = 0; i < prim::kSpatialDim; ++i) {
      EXPECT_EQ(v[i], i == index ? d : 0.0F) << "axis " << static_cast<int>(a) << " component " << i;
    }
  }
  c.axis = ChirpAxis::None;
  EXPECT_EQ(chirp_value(c, t), 0.0F);
  const prim::Vec3<float> none = chirp_torque(c, t);
  EXPECT_EQ(none[0], 0.0F);
  EXPECT_EQ(none[1], 0.0F);
  EXPECT_EQ(none[2], 0.0F);
}

}  // namespace
