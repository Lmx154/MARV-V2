// L6 stage (b), the rotor-speed sensor model of the plant (decision 0013): known answers of the telemetry grid (exact grid
// values decode exactly, a speed between grid points truncates), the stopped rule, the pole count, and the negative
// controls that must fail: a pole count off by 2, rev/s in place of rad/s, round-to-nearest in place of truncation.
// Speeds are set through the plant's initial rotor speeds, so the sample is of exactly that speed (no step has run).
#include <cmath>
#include <limits>

#include "support.hpp"

namespace {

using namespace marv::plant::rotor_test;

using Omegas = std::array<double, MARV_PLANT_N_MOTORS>;

// One sample of a fresh plant whose rotors start at `omega`, on the grid of `cfg`.
marv_plant_rotor_speed_out read(const Omegas& omega, const marv_plant_rotor_speed_config& cfg = grid_config()) {
  PlantHandle plant(omega);
  EXPECT_EQ(marv_plant_rotor_speed_attach(plant.get(), &cfg), MARV_PLANT_OK);
  return sample_ok(plant.get());
}

// The middle of the cell of the period p = m << e: the period p + 2^e / 2 us, so the truncated word is exactly (e, m).
double cell_middle_omega(double m, int e, std::uint32_t poles = kPoleCount) {
  const double step = std::ldexp(1.0, e);
  return omega_of_period_us(m * step + step / 2.0, poles);
}

// 400 speeds, geometric from 14 rad/s (period 64.1 ms, just inside the largest word, 65.4 ms, at 14 poles) to 3500 rad/s
// (period 286 us). 12 and 16 poles are inside the same range. In groups of four, one per motor.
std::vector<Omegas> sweep() {
  constexpr std::size_t kCount = 400;
  constexpr double kLow = 14.0;
  constexpr double kHigh = 3500.0;
  std::vector<Omegas> v;
  const double ratio = std::pow(kHigh / kLow, 1.0 / static_cast<double>(kCount * MARV_PLANT_N_MOTORS - 1));
  double w = kLow;
  for (std::size_t k = 0; k < kCount; ++k) {
    Omegas group{};
    for (double& x : group) {
      x = w;
      w *= ratio;
    }
    v.push_back(group);
  }
  return v;
}

std::array<Reading, MARV_PLANT_N_MOTORS> expected(const Omegas& omega, std::uint32_t poles, Rule rule = Rule::Truncate) {
  std::array<Reading, MARV_PLANT_N_MOTORS> r{};
  for (std::size_t i = 0; i < r.size(); ++i) {
    r[i] = oracle(omega[i], poles, rule);
  }
  return r;
}

TEST(RotorSpeedGrid, ExactGridValuesDecodeExactlyOnEveryExponent) {
  // Mantissas 256 (the smallest normalised one), 300, 400 and 511 (510 at e = 7, where 511 is the stopped word): four
  // motors, one per mantissa, so a motor index shift is seen. Every (e, m) is a different word.
  for (int e = 0; e <= kExponentMax; ++e) {
    const double top = e == kExponentMax ? 510.0 : 511.0;
    const std::array<double, 4> m = {256.0, 300.0, 400.0, top};
    Omegas omega{};
    std::array<float, 4> want{};
    for (std::size_t i = 0; i < 4; ++i) {
      omega[i] = cell_middle_omega(m[i], e);
      want[i] = static_cast<float>(omega_of_period_us(m[i] * std::ldexp(1.0, e), kPoleCount));
    }
    const marv_plant_rotor_speed_out o = read(omega);
    EXPECT_EQ(o.flags, all_valid()) << "e " << e;
    for (std::size_t i = 0; i < 4; ++i) {
      EXPECT_EQ(std::memcmp(&o.omega_rad_s[i], &want[i], sizeof(float)), 0) << "e " << e << " motor " << i;
      if (e > 0) {
        // Non-vacuous: the speed asked for is not the speed on the grid (the cell middle is not its start).
        EXPECT_NE(static_cast<float>(omega[i]), want[i]) << "e " << e << " motor " << i;
      }
    }
  }
}

TEST(RotorSpeedGrid, ASpeedBetweenGridPointsTruncatesTheLowBitsOfThePeriod) {
  // e = 3, m = 300: the cell is the periods [2400, 2408) us. Any period in it decodes to 2400 us; the next cell's start,
  // 2408 us, decodes to 2408 us. The fractions are labelled test values: just inside the start, middle, near the end.
  constexpr int kE = 3;
  constexpr double kM = 300.0;
  const double step = std::ldexp(1.0, kE);
  const double start = kM * step;
  const std::array<double, 4> fractions = {0.01, 0.5, 0.75, 0.99};
  Omegas omega{};
  for (std::size_t i = 0; i < 4; ++i) {
    omega[i] = omega_of_period_us(start + fractions[i] * step, kPoleCount);
  }
  const marv_plant_rotor_speed_out o = read(omega);
  const float low = static_cast<float>(omega_of_period_us(start, kPoleCount));
  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_EQ(std::memcmp(&o.omega_rad_s[i], &low, sizeof(float)), 0) << "fraction " << fractions[i];
  }
  const Omegas next = {omega_of_period_us(start + step + 0.01 * step, kPoleCount), 0.0, 0.0, 0.0};
  const float up = static_cast<float>(omega_of_period_us(start + step, kPoleCount));
  const marv_plant_rotor_speed_out n = read(next);
  EXPECT_EQ(std::memcmp(&n.omega_rad_s[0], &up, sizeof(float)), 0);

  // Control: a round-to-nearest-grid-point rule gives a different reading at fractions 0.75 and 0.99, so this test
  // would fail if the model rounded instead of truncated.
  for (std::size_t i = 2; i < 4; ++i) {
    const Reading nearest = oracle(omega[i], kPoleCount, Rule::Nearest);
    EXPECT_NE(std::memcmp(&o.omega_rad_s[i], &nearest.omega, sizeof(float)), 0) << "fraction " << fractions[i];
  }
}

TEST(RotorSpeedGrid, SweepEqualsTheIndependentOracleAndStaysInsideTheResolutionBound) {
  for (const Omegas& group : sweep()) {
    const marv_plant_rotor_speed_out o = read(group);
    EXPECT_TRUE(matches(o, expected(group, kPoleCount))) << "omega " << group[0];
    for (std::size_t i = 0; i < group.size(); ++i) {
      // Truncation lowers the period, so the speed reads high, by less than one grid step of the period: a relative error
      // below 2^-8 for periods of 256 us and more (the mantissa is then at least 256), plus the float rounding 2^-24.
      const double ratio = static_cast<double>(o.omega_rad_s[i]) / group[i] - 1.0;
      EXPECT_GE(ratio, -0x1p-24) << "omega " << group[i];
      EXPECT_LT(ratio, 0x1p-8 + 0x1p-24) << "omega " << group[i];
    }
  }
}

TEST(RotorSpeedGrid, SweepNegativeControlRoundingInsteadOfTruncationDiffers) {
  std::size_t differing = 0;
  for (const Omegas& group : sweep()) {
    const marv_plant_rotor_speed_out o = read(group);
    differing += matches(o, expected(group, kPoleCount, Rule::Nearest)) ? 0U : 1U;
  }
  EXPECT_GT(differing, 0U);
}

TEST(RotorSpeedPoles, ReadingFollowsTheConfiguredPoleCountAndAWrongOneIsSeen) {
  for (const std::uint32_t poles : {12U, 14U, 16U}) {
    std::size_t wrong_up = 0;
    std::size_t wrong_down = 0;
    for (const Omegas& group : sweep()) {
      const marv_plant_rotor_speed_out o = read(group, grid_config(0, poles));
      EXPECT_TRUE(matches(o, expected(group, poles))) << "poles " << poles << " omega " << group[0];
      // Control: the oracle at a pole count off by 2 (either way) does not match.
      wrong_up += matches(o, expected(group, poles + 2)) ? 0U : 1U;
      wrong_down += matches(o, expected(group, poles - 2)) ? 0U : 1U;
    }
    EXPECT_GT(wrong_up, 0U) << "poles " << poles;
    EXPECT_GT(wrong_down, 0U) << "poles " << poles;
  }
}

TEST(RotorSpeedPoles, TheStoppedThresholdMovesWithThePoleCount) {
  // 15 rad/s: the period is 2 pi / (15 * poles / 2) s = 59.8 ms at 14 poles (inside the 65.4 ms limit), 69.8 ms at 12
  // (stopped), 52.4 ms at 16.
  const Omegas omega = {15.0, 15.0, 15.0, 15.0};
  const std::array<std::uint32_t, 3> poles = {12, 14, 16};
  const std::array<bool, 3> measured = {false, true, true};
  for (std::size_t k = 0; k < poles.size(); ++k) {
    const marv_plant_rotor_speed_out o = read(omega, grid_config(0, poles[k]));
    EXPECT_EQ(o.flags, all_valid());
    EXPECT_EQ(o.omega_rad_s[0] != 0.0F, measured[k]) << "poles " << poles[k];
  }
}

TEST(RotorSpeedUnits, ControlRevPerSecondInPlaceOfRadPerSecondDoesNotMatch) {
  std::size_t equal = 0;
  for (const Omegas& group : sweep()) {
    const marv_plant_rotor_speed_out o = read(group);
    Omegas rev{};
    for (std::size_t i = 0; i < group.size(); ++i) {
      rev[i] = group[i] / kTwoPi;
    }
    equal += matches(o, expected(rev, kPoleCount)) ? 1U : 0U;
  }
  EXPECT_EQ(equal, 0U);
}

TEST(RotorSpeedStopped, ASlowOrStoppedRotorIsAValidReadingOfZero) {
  // The speed whose period is the all-ones word's, 65408 us: omega_th = 2 pi / (65408 us * 7) = 13.7 rad/s at 14 poles.
  const double omega_th = omega_of_period_us(kStoppedPeriod, kPoleCount);
  constexpr double kBelow = 0.999;  // labelled: a 0.1 % margin either side of the threshold
  constexpr double kAbove = 1.001;
  const Omegas omega = {0.0, kBelow * omega_th, kAbove * omega_th, std::numeric_limits<double>::denorm_min()};
  const marv_plant_rotor_speed_out o = read(omega);
  EXPECT_EQ(o.flags, all_valid());
  const float zero = 0.0F;
  EXPECT_EQ(std::memcmp(&o.omega_rad_s[0], &zero, sizeof(float)), 0);  // +0 exactly
  EXPECT_EQ(std::memcmp(&o.omega_rad_s[1], &zero, sizeof(float)), 0);
  EXPECT_GT(o.omega_rad_s[2], 0.0F);
  EXPECT_NEAR(static_cast<double>(o.omega_rad_s[2]) / omega_th, kAbove, 0x1p-7);  // inside the grid step, on the high side
  EXPECT_EQ(std::memcmp(&o.omega_rad_s[3], &zero, sizeof(float)), 0);
  EXPECT_TRUE(matches(o, expected(omega, kPoleCount)));
}

TEST(RotorSpeedStopped, AFastRotorAtTheFixtureMaximumIsMeasured) {
  // The plant's own rotors never exceed omega_max; the fixture's 3800 rad/s has a 236 us period: measured, e = 0.
  const Omegas omega = {kOmegaMax, kOmegaMax, kOmegaMax, kOmegaMax};
  const marv_plant_rotor_speed_out o = read(omega);
  EXPECT_TRUE(matches(o, expected(omega, kPoleCount)));
  EXPECT_GT(o.omega_rad_s[0], 0.0F);
}

}  // namespace
