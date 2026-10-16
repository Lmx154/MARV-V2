// T1 for the QF-1 rates curve (fw/stick_rates): r(s) = sign(s) [c a + (m - c) g(a)], a = |s|, g = (1 - e) a^2 + e a^6.
// The mid-stick known answers are computed here in double from the Betaflight 4.5 Actual-rates form
// r = s c + (m - c) a (e s^5 + (1 - e) s) (src/main/fc/rc.c:209-217 at 024f8e13, written as mathematics), a different
// arrangement from the implementation's c (a - g) + m g, and by hand for the exact-in-binary cases below.
#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>

#include "marv/prim/constants.hpp"
#include "marv/stick_rates/stick_rates.hpp"

namespace {

using marv::stick_rates::ConfigError;
using marv::stick_rates::RateCurve;
using marv::stick_rates::RatesConfig;
using marv::stick_rates::rate_setpoint;
using marv::stick_rates::validate;

// Test values (labelled scenario values of this test, not flight numbers): centre 1 rad/s, maximum 11 rad/s.
constexpr float kCentre = 1.0F;
constexpr float kMax = 11.0F;

RateCurve<float> curve(float centre, float max, float expo) { return RateCurve<float>{centre, max, expo}; }

// The published form, in double.
double reference(double c, double m, double e, double s) {
  const double a = std::fabs(s);
  return s * c + (m - c) * a * (e * std::pow(s, 5) + (1.0 - e) * s);
}

// The defaults of quad_fc_register.yaml and design/scenario_values.yaml, rebuilt from 70 and 670 deg/s by the rule
// deg x pi / 180.
RateCurve<float> defaults() {
  return curve(static_cast<float>(70.0 * marv::prim::kPi / marv::prim::kDegreesPerHalfTurn),
               static_cast<float>(670.0 * marv::prim::kPi / marv::prim::kDegreesPerHalfTurn), 0.0F);
}

TEST(StickRates, ZeroStickIsZero) {
  for (const float e : {0.0F, 0.5F, 1.0F}) {
    EXPECT_EQ(rate_setpoint(curve(kCentre, kMax, e), 0.0F), 0.0F);
  }
  EXPECT_EQ(rate_setpoint(defaults(), 0.0F), 0.0F);
}

TEST(StickRates, FullStickIsTheMaximumRateExactly) {
  for (const float e : {0.0F, 0.25F, 0.5F, 0.7F, 1.0F}) {
    const RateCurve<float> c = curve(kCentre, kMax, e);
    EXPECT_EQ(rate_setpoint(c, 1.0F), kMax);
    EXPECT_EQ(rate_setpoint(c, -1.0F), -kMax);
  }
  const RateCurve<float> d = defaults();
  EXPECT_EQ(rate_setpoint(d, 1.0F), d.max);
  EXPECT_EQ(rate_setpoint(d, -1.0F), -d.max);
  EXPECT_EQ(validate(d), ConfigError::None);
}

TEST(StickRates, MidStickKnownAnswersByHand) {
  // c = 1, m = 11, so m - c = 10. s = 0.5: a^2 = 1/4, a^6 = 1/64; all values are exact in binary.
  // e = 0:   g = 1/4;                     r = 1/2 + 10/4                  = 3.
  // e = 1:   g = 1/64;                    r = 1/2 + 10/64                 = 0.65625.
  // e = 1/2: g = 1/8 + 1/128 = 17/128;    r = 1/2 + 170/128 = 1/2 + 1.328125 = 1.828125.
  EXPECT_EQ(rate_setpoint(curve(kCentre, kMax, 0.0F), 0.5F), 3.0F);
  EXPECT_EQ(rate_setpoint(curve(kCentre, kMax, 1.0F), 0.5F), 0.65625F);
  EXPECT_EQ(rate_setpoint(curve(kCentre, kMax, 0.5F), 0.5F), 1.828125F);
  EXPECT_EQ(rate_setpoint(curve(kCentre, kMax, 0.5F), -0.5F), -1.828125F);
}

TEST(StickRates, MidStickAgainstThePublishedFormWithTheDefaults) {
  // The 70 / 670 deg/s defaults in rad/s, expo 0 then 0.5, at s = 0.3 and 0.8, against the published form in double.
  const RateCurve<float> d = defaults();
  for (const float e : {0.0F, 0.5F}) {
    const RateCurve<float> c = curve(d.centre, d.max, e);
    for (const float s : {0.3F, 0.8F, -0.3F, -0.8F}) {
      const double want = reference(static_cast<double>(c.centre), static_cast<double>(c.max), static_cast<double>(e),
                                    static_cast<double>(s));
      EXPECT_NEAR(static_cast<double>(rate_setpoint(c, s)), want, 1.0e-5) << "e " << e << " s " << s;
    }
  }
  // By hand, e = 0, s = 0.8: g = 0.64, r = 0.8 c + (m - c) 0.64 with c = 70 pi/180, m = 670 pi/180
  //   = (pi/180) (56 + 600 x 0.64) = (pi/180) 440 = 7.679448708775051 rad/s.
  EXPECT_NEAR(rate_setpoint(d, 0.8F), 7.679448708775051F, 1.0e-5F);
}

TEST(StickRates, OddSymmetry) {
  for (const float e : {0.0F, 0.5F, 1.0F}) {
    const RateCurve<float> c = curve(kCentre, kMax, e);
    for (int i = 0; i <= 100; ++i) {
      const float s = static_cast<float>(i) / 100.0F;
      EXPECT_EQ(rate_setpoint(c, -s), -rate_setpoint(c, s)) << "e " << e << " s " << s;
    }
  }
}

TEST(StickRates, MonotonicOnTheWholeRange) {
  const int n = 2000;
  for (const float e : {0.0F, 0.5F, 1.0F}) {
    const RateCurve<float> c = curve(kCentre, kMax, e);
    float previous = rate_setpoint(c, -1.0F);
    for (int i = -n + 1; i <= n; ++i) {
      const float r = rate_setpoint(c, static_cast<float>(i) / static_cast<float>(n));
      EXPECT_GE(r, previous) << "e " << e << " i " << i;
      previous = r;
    }
  }
  const RateCurve<float> d = defaults();
  float previous = rate_setpoint(d, -1.0F);
  for (int i = -n + 1; i <= n; ++i) {
    const float r = rate_setpoint(d, static_cast<float>(i) / static_cast<float>(n));
    EXPECT_GE(r, previous) << "defaults i " << i;
    previous = r;
  }
}

TEST(StickRates, CentreSlopeIsTheCentreSensitivity) {
  // r(h) = c h + (m - c) g(h) with 0 <= g(h) <= h^2, so |r(h) / h - c| <= (m - c) h, plus float rounding.
  const float h = 1.0F / 1024.0F;
  for (const float e : {0.0F, 0.5F, 1.0F}) {
    const RateCurve<float> c = curve(kCentre, kMax, e);
    const float slope = rate_setpoint(c, h) / h;
    EXPECT_NEAR(slope, c.centre, (c.max - c.centre) * h + 1.0e-5F) << "e " << e;
  }
  const RateCurve<float> d = defaults();
  EXPECT_NEAR(rate_setpoint(d, h) / h, d.centre, (d.max - d.centre) * h + 1.0e-5F);
}

TEST(StickRates, BeyondRangeIsHeldAndNonFiniteIsZero) {
  const RateCurve<float> c = curve(kCentre, kMax, 0.5F);
  EXPECT_EQ(rate_setpoint(c, 3.0F), kMax);
  EXPECT_EQ(rate_setpoint(c, -3.0F), -kMax);
  EXPECT_EQ(rate_setpoint(c, std::nanf("")), 0.0F);
  EXPECT_EQ(rate_setpoint(c, HUGE_VALF), 0.0F);
}

TEST(StickRates, AxesAreIndependent) {
  RatesConfig<float> cfg;
  cfg.roll = curve(kCentre, kMax, 0.0F);
  cfg.pitch = curve(kCentre, kMax, 1.0F);
  cfg.yaw = curve(2.0F, 4.0F, 0.0F);
  const marv::prim::Vec3<float> out = rate_setpoint(cfg, marv::prim::Vec3<float>(0.5F, 0.5F, -1.0F));
  EXPECT_EQ(out[0], 3.0F);
  EXPECT_EQ(out[1], 0.65625F);
  EXPECT_EQ(out[2], -4.0F);
  EXPECT_EQ(validate(cfg), ConfigError::None);
}

TEST(StickRates, ValidationRefusesEachBadValue) {
  EXPECT_EQ(validate(curve(kCentre, kMax, 0.5F)), ConfigError::None);
  EXPECT_EQ(validate(curve(kMax, kMax, 0.0F)), ConfigError::None);  // c = m: the curve is linear
  EXPECT_EQ(validate(curve(0.0F, kMax, 0.0F)), ConfigError::Centre);
  EXPECT_EQ(validate(curve(-kCentre, kMax, 0.0F)), ConfigError::Centre);
  EXPECT_EQ(validate(curve(kMax, kCentre, 0.0F)), ConfigError::Max);
  EXPECT_EQ(validate(curve(kCentre, kMax, -0.1F)), ConfigError::Expo);
  EXPECT_EQ(validate(curve(kCentre, kMax, 1.1F)), ConfigError::Expo);
  EXPECT_EQ(validate(curve(std::nanf(""), kMax, 0.0F)), ConfigError::NonFinite);
  EXPECT_EQ(validate(curve(kCentre, HUGE_VALF, 0.0F)), ConfigError::NonFinite);
  RatesConfig<float> cfg;
  cfg.roll = curve(kCentre, kMax, 0.0F);
  cfg.pitch = curve(kCentre, kMax, 0.0F);
  cfg.yaw = curve(kCentre, kMax, 2.0F);
  EXPECT_EQ(validate(cfg), ConfigError::Expo);
}

}  // namespace
