// L6 stage (b) T1, line 1: the float coefficients of the low-pass and the notch against a double implementation of
// the design formula (the K form: bilinear transform prewarped at the filter frequency), within the first-order running
// error bound of the float operation sequence. The bound is, per coefficient, half an ulp (u |v|) of every rounded
// operation plus the propagated argument error plus kLibmUlps ulp for each sinf, cosf, tanf (support.hpp, Err).
// Negative control: one coefficient moved to the first float beyond the bound must fail the check.
#include <cmath>
#include <limits>

#include "support.hpp"

namespace {

using namespace marv;
using namespace marv::gyro_chain;
using namespace marv::gyro_chain::test;

// Double-evaluation differences between the K form and the RBJ form (tan, cos, sin each ~1e-16 relative, amplified by
// at most 1 / alpha ~ 1e3 for the sharpest notch): 1e-12 absolute.
constexpr double kFormTol = 1.0e-12;

struct NotchCase {
  const char* name;
  float f0;
  float q;
};

std::vector<NotchCase> notch_cases(const Fixture& fx) {
  std::vector<NotchCase> cs;
  const auto omega = rotor_omegas();
  for (std::size_t m = 0; m < kMotors; ++m) {
    for (std::size_t h = 0; h < kHarmonics; ++h) {
      cs.push_back({"fixture notch", chain_f0(h, omega[m]), fx.config.notch_q[h]});
    }
  }
  // The activation floor, the sharpest and the broadest notch the validator accepts for the fixture, and one just
  // below Nyquist: test values at the ends of the range.
  cs.push_back({"omega_th", static_cast<float>(static_cast<double>(kOmegaThreshold) / (2.0 * kPi)), kScratchQ[0]});
  cs.push_back({"near Nyquist", static_cast<float>(0.9 * fx.fs / 2.0), kScratchQ[2]});
  cs.push_back({"low Q", 400.0F, 0.5F});
  cs.push_back({"high Q", 400.0F, 100.0F});
  return cs;
}

bool within(const BiquadCoeffs<float>& c, const StageErr& r) {
  return std::fabs(static_cast<double>(c.b0) - r.v.b0) <= r.e.b0 && std::fabs(static_cast<double>(c.b1) - r.v.b1) <= r.e.b1 &&
         std::fabs(static_cast<double>(c.b2) - r.v.b2) <= r.e.b2 && std::fabs(static_cast<double>(c.a1) - r.v.a1) <= r.e.a1 &&
         std::fabs(static_cast<double>(c.a2) - r.v.a2) <= r.e.a2;
}

double ulp_of(float x) { return static_cast<double>(std::nextafter(std::fabs(x), std::numeric_limits<float>::infinity()) - std::fabs(x)); }

// The libm assumption: float sin, cos, tan within kLibmUlps ulp of the double function of the same float argument.
void expect_libm(float arg) {
  const double a = static_cast<double>(arg);
  const float s = std::sin(arg);
  const float c = std::cos(arg);
  const float t = std::tan(arg);
  EXPECT_LE(std::fabs(static_cast<double>(s) - std::sin(a)), kLibmUlps * ulp_of(s)) << arg;
  EXPECT_LE(std::fabs(static_cast<double>(c) - std::cos(a)), kLibmUlps * ulp_of(c)) << arg;
  EXPECT_LE(std::fabs(static_cast<double>(t) - std::tan(a)), kLibmUlps * ulp_of(t)) << arg;
}

TEST(L6GyroChainCoefficients, NotchesMatchTheDoubleFormula) {
  const Fixture fx = make_fixture();
  const float two_pi = 2.0F * static_cast<float>(kPi);
  for (const NotchCase& nc : notch_cases(fx)) {
    SCOPED_TRACE(nc.name);
    const BiquadCoeffs<float> c = notch_coeffs<float>(nc.f0, nc.q, fx.period);
    const StageErr r = notch_err(static_cast<double>(nc.f0), nc.q, fx.period, 0.0);
    EXPECT_TRUE(within(c, r));
    // The independent K form agrees with the RBJ form the bound is evaluated on, and the float result is within the
    // bound of it (plus the double difference of the two forms).
    const Stage k = notch_design(static_cast<double>(nc.f0), static_cast<double>(nc.q), static_cast<double>(fx.period));
    EXPECT_NEAR(k.b0, r.v.b0, kFormTol);
    EXPECT_NEAR(k.b1, r.v.b1, kFormTol);
    EXPECT_NEAR(k.b2, r.v.b2, kFormTol);
    EXPECT_NEAR(k.a1, r.v.a1, kFormTol);
    EXPECT_NEAR(k.a2, r.v.a2, kFormTol);
    EXPECT_NEAR(static_cast<double>(c.b0), k.b0, r.e.b0 + kFormTol);
    EXPECT_NEAR(static_cast<double>(c.b1), k.b1, r.e.b1 + kFormTol);
    EXPECT_NEAR(static_cast<double>(c.b2), k.b2, r.e.b2 + kFormTol);
    EXPECT_NEAR(static_cast<double>(c.a1), k.a1, r.e.a1 + kFormTol);
    EXPECT_NEAR(static_cast<double>(c.a2), k.a2, r.e.a2 + kFormTol);
    // Structure of the notch: b0 = b2, a1 = b1 bit for bit.
    EXPECT_EQ(c.b0, c.b2);
    EXPECT_EQ(c.a1, c.b1);
    // The bound is not vacuous: far below 1e-5, which would hide an error of the sharpest notch's order.
    EXPECT_LT(r.e.b1, 1.0e-5);
    EXPECT_LT(r.e.a2, 1.0e-5);
    expect_libm(two_pi * nc.f0 * fx.period);
  }
}

TEST(L6GyroChainCoefficients, LowpassMatchesTheDoubleFormula) {
  const Fixture fx = make_fixture();
  // The rule's cutoff, and two others: test values spanning the range below f_s / 2 = 3200 Hz.
  for (const float fc : {fx.cutoff, 100.0F, 1500.0F}) {
    SCOPED_TRACE(fc);
    const BiquadCoeffs<float> c = lowpass_coeffs<float>(fc, fx.period);
    const StageErr r = lowpass_err(fc, fx.period);
    EXPECT_TRUE(within(c, r));
    const Stage k = lowpass_design(static_cast<double>(fc), static_cast<double>(fx.period));
    EXPECT_NEAR(k.b0, r.v.b0, kFormTol);
    EXPECT_NEAR(k.a1, r.v.a1, kFormTol);
    EXPECT_NEAR(k.a2, r.v.a2, kFormTol);
    EXPECT_NEAR(static_cast<double>(c.b0), k.b0, r.e.b0 + kFormTol);
    EXPECT_NEAR(static_cast<double>(c.b1), k.b1, r.e.b1 + kFormTol);
    EXPECT_NEAR(static_cast<double>(c.b2), k.b2, r.e.b2 + kFormTol);
    EXPECT_NEAR(static_cast<double>(c.a1), k.a1, r.e.a1 + kFormTol);
    EXPECT_NEAR(static_cast<double>(c.a2), k.a2, r.e.a2 + kFormTol);
    EXPECT_EQ(c.b1, 2.0F * c.b0);
    EXPECT_EQ(c.b2, c.b0);
    expect_libm(static_cast<float>(kPi) * fc * fx.period);
  }
}

TEST(L6GyroChainCoefficients, LowpassCutoffFollowsTheRule) {
  // The rule's cutoff gives the digital gain a_min at f_r / 2, from the exact response of the double design.
  const Fixture fx = make_fixture();
  const double fr = fx.fs / static_cast<double>(kDivisor);
  const Stage lp = lowpass_design(static_cast<double>(fx.cutoff), static_cast<double>(fx.period));
  const double gain = std::abs(stage_response(lp, digital_w(fr / 2.0, fx.fs)));
  // 1e-6: the float rounding of f_c and of the period shifts the gain by about 1e-7 (kU relative on K, order 1 sensitivity).
  EXPECT_NEAR(gain, kAMin, 1.0e-6);
}

// Negative control: one coefficient moved to the first float beyond its bound must fail `within`.
TEST(L6GyroChainCoefficients, ControlOneCoefficientOneUlpBeyondTheBoundFails) {
  const Fixture fx = make_fixture();
  const float f0 = chain_f0(0, rotor_omegas()[0]);
  const BiquadCoeffs<float> good = notch_coeffs<float>(f0, fx.config.notch_q[0], fx.period);
  const StageErr r = notch_err(static_cast<double>(f0), fx.config.notch_q[0], fx.period, 0.0);
  ASSERT_TRUE(within(good, r));
  const float* const gp[5] = {&good.b0, &good.b1, &good.b2, &good.a1, &good.a2};
  const double ref[5] = {r.v.b0, r.v.b1, r.v.b2, r.v.a1, r.v.a2};
  const double bound[5] = {r.e.b0, r.e.b1, r.e.b2, r.e.a1, r.e.a2};
  for (int i = 0; i < 5; ++i) {
    BiquadCoeffs<float> bad = good;
    float* const bp[5] = {&bad.b0, &bad.b1, &bad.b2, &bad.a1, &bad.a2};
    const float dir = (static_cast<double>(*gp[i]) >= ref[i]) ? std::numeric_limits<float>::infinity()
                                                                : -std::numeric_limits<float>::infinity();
    while (std::fabs(static_cast<double>(*bp[i]) - ref[i]) <= bound[i]) {
      *bp[i] = std::nextafter(*bp[i], dir);
    }
    // Beyond the bound by at most one ulp, and the check rejects it.
    EXPECT_LE(std::fabs(static_cast<double>(*bp[i]) - ref[i]) - bound[i], ulp_of(*bp[i])) << i;
    EXPECT_FALSE(within(bad, r)) << i;
  }
}

}  // namespace
