// L6 stage (b), the rotor-speed model's rules for inputs the plant itself never produces (its rotor speeds are finite and
// not negative): a non-finite or negative speed is an INVALID motor of value 0, a period below one unit is INVALID,
// each motor is judged on its own. Tested on the model (sim/plant/src/rotor_speed_model.hpp), the unit under the C entry.
#include <cmath>
#include <limits>

#include "rotor_speed_model.hpp"
#include "support.hpp"

namespace {

using namespace marv::plant;
using namespace marv::plant::rotor_test;

RotorSpeedParams grid_params(std::size_t latency = 0) {
  RotorSpeedParams p;
  p.pole_count = kPoleCount;
  p.latency = latency;
  p.exponent_bits = kExponentBits;
  p.mantissa_bits = kMantissaBits;
  p.period_unit_s = kPeriodUnitS;
  return p;
}

RotorSpeedOut one(const std::array<double, kRotorSpeedMotors>& omega) {
  RotorSpeedModel m;
  m.attach(grid_params());
  return m.sample(omega);
}

bool same_zero(float v) {
  const float zero = 0.0F;
  return std::memcmp(&v, &zero, sizeof(float)) == 0;
}

TEST(RotorSpeedModel, NonFiniteAndNegativeSpeedsAreInvalidWithValueZero) {
  constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
  constexpr double kInf = std::numeric_limits<double>::infinity();
  const std::array<double, 5> bad = {kNan, kInf, -kInf, -1.0, -std::numeric_limits<double>::denorm_min()};
  for (const double w : bad) {
    const RotorSpeedOut o = one({w, 1000.0, 1000.0, 1000.0});
    EXPECT_EQ(o.flags, 0b1110U) << "omega " << w;  // motor 1 invalid, motors 2 to 4 valid
    EXPECT_TRUE(same_zero(o.omega[0])) << "omega " << w;
    const Reading want = oracle(1000.0, kPoleCount);
    EXPECT_EQ(std::memcmp(&o.omega[1], &want.omega, sizeof(float)), 0);
  }
}

TEST(RotorSpeedModel, MinusZeroIsAStoppedRotor) {
  const RotorSpeedOut o = one({-0.0, 0.0, 0.0, 0.0});
  EXPECT_EQ(o.flags, all_valid());
  EXPECT_TRUE(same_zero(o.omega[0]));
}

TEST(RotorSpeedModel, EachMotorIsJudgedOnItsOwn) {
  const RotorSpeedOut o = one({std::numeric_limits<double>::quiet_NaN(), 100.0, -1.0, 0.0});
  EXPECT_EQ(o.flags, 0b1010U);  // motors 2 (bit 1) and 4 (bit 3) valid
  EXPECT_TRUE(same_zero(o.omega[0]));
  EXPECT_TRUE(same_zero(o.omega[2]));
  EXPECT_TRUE(same_zero(o.omega[3]));
  const Reading want = oracle(100.0, kPoleCount);
  EXPECT_EQ(std::memcmp(&o.omega[1], &want.omega, sizeof(float)), 0);
  EXPECT_GT(o.omega[1], 0.0F);
}

TEST(RotorSpeedModel, APeriodBelowOneUnitIsInvalid) {
  // 2 pi / (7 * 1 us) = 897.6 krad/s has a period of exactly one unit. 1e6 rad/s is 0.9 of a unit: truncates to 0, the
  // protocol's invalid word. 8e5 rad/s is 1.12 units: m = 1, e = 0, measured.
  const RotorSpeedOut o = one({1.0e6, 8.0e5, 1.0e7, 1.0e300});
  EXPECT_EQ(o.flags, 0b0010U);
  EXPECT_TRUE(same_zero(o.omega[0]));
  EXPECT_GT(o.omega[1], 0.0F);
  EXPECT_TRUE(same_zero(o.omega[2]));
  EXPECT_TRUE(same_zero(o.omega[3]));
  const Reading want = oracle(8.0e5, kPoleCount);
  EXPECT_EQ(std::memcmp(&o.omega[1], &want.omega, sizeof(float)), 0);
}

TEST(RotorSpeedModel, TheDelayLineHoldsTheDecodedSampleAndSample0BeforeTheStart) {
  RotorSpeedModel m;
  m.attach(grid_params(2));
  const std::array<std::array<double, 4>, 4> in = {{{100.0, 200.0, 300.0, 400.0},
                                                    {110.0, 210.0, 310.0, 410.0},
                                                    {120.0, 220.0, 320.0, 420.0},
                                                    {130.0, 230.0, 330.0, 430.0}}};
  std::array<RotorSpeedOut, 4> out{};
  for (std::size_t k = 0; k < in.size(); ++k) {
    out[k] = m.sample(in[k]);
  }
  // Latency 2: samples 0, 1, 2 report sample 0; sample 3 reports sample 1.
  const RotorSpeedOut first = one(in[0]);
  const RotorSpeedOut second = one(in[1]);
  EXPECT_EQ(std::memcmp(&out[0], &first, sizeof(first)), 0);
  EXPECT_EQ(std::memcmp(&out[1], &first, sizeof(first)), 0);
  EXPECT_EQ(std::memcmp(&out[2], &first, sizeof(first)), 0);
  EXPECT_EQ(std::memcmp(&out[3], &second, sizeof(second)), 0);
}

}  // namespace
