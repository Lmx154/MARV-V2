// L6 stage (a), T1 (decision 0012, owner decision "Turn-on bias: corners"; Luis, 2026-10-01, stage (a) approval):
// the generated corner builder imu_corner_config(s) gives 2^6 = 64 distinct corners, each axis at +bound or -bound;
// the nominal (all axes 0) is separate from them; every other config field equals the profile config; and an s_i
// outside {-1, 0, +1} is refused.
#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <set>

#include "marv/sim/imu_profile_config.hpp"
#include "marv_plant.h"

namespace {

constexpr std::size_t kAxesPerSensor = 3;                     // x, y, z
constexpr std::size_t kAxes = 2 * kAxesPerSensor;             // gyro x y z, accel x y z (the builder's order)
constexpr std::size_t kCorners = std::size_t{1} << kAxes;     // each axis at +bound or -bound: 2^6

// Corner index c -> signs: bit i set means axis i at +bound, clear means -bound.
std::array<int, kAxes> signs_of(std::size_t c) {
  std::array<int, kAxes> s{};
  for (std::size_t i = 0; i < kAxes; ++i) s[i] = ((c >> i) & 1U) != 0U ? 1 : -1;
  return s;
}

double bound_of(std::size_t axis) {
  return axis < kAxesPerSensor ? marv::sim::kImuGyroTurnOnBiasBound : marv::sim::kImuAccelTurnOnBiasBound;
}

double bias_of(const marv_plant_imu_config& c, std::size_t axis) {
  return axis < kAxesPerSensor ? c.gyro.turn_on_bias[axis] : c.accel.turn_on_bias[axis - kAxesPerSensor];
}

// Every field except the turn-on biases.
void expect_same_except_bias(const marv_plant_imu_config& a, const marv_plant_imu_config& b) {
  EXPECT_EQ(a.struct_size, b.struct_size);
  EXPECT_EQ(a.latency_samples, b.latency_samples);
  EXPECT_EQ(a.gyro.noise_density, b.gyro.noise_density);
  EXPECT_EQ(a.gyro.bias_instability, b.gyro.bias_instability);
  EXPECT_EQ(a.gyro.lsb, b.gyro.lsb);
  EXPECT_EQ(a.gyro.full_scale, b.gyro.full_scale);
  EXPECT_EQ(a.accel.noise_density, b.accel.noise_density);
  EXPECT_EQ(a.accel.bias_instability, b.accel.bias_instability);
  EXPECT_EQ(a.accel.lsb, b.accel.lsb);
  EXPECT_EQ(a.accel.full_scale, b.accel.full_scale);
}

TEST(ImuCorners, SixtyFourDistinctCornersEachAxisAtPlusOrMinusBound) {
  const marv_plant_imu_config nominal = marv::sim::imu_profile_config();
  std::set<std::array<double, kAxes>> seen;
  for (std::size_t c = 0; c < kCorners; ++c) {
    const auto s = signs_of(c);
    const auto cfg = marv::sim::imu_corner_config(s);
    ASSERT_TRUE(cfg.has_value()) << "corner " << c;
    std::array<double, kAxes> biases{};
    for (std::size_t i = 0; i < kAxes; ++i) {
      biases[i] = bias_of(*cfg, i);
      EXPECT_GT(bound_of(i), 0.0) << "axis " << i;
      EXPECT_EQ(biases[i], static_cast<double>(s[i]) * bound_of(i)) << "corner " << c << " axis " << i;
      EXPECT_NE(biases[i], 0.0) << "corner " << c << " axis " << i;  // a corner is never the nominal on any axis
    }
    expect_same_except_bias(*cfg, nominal);
    seen.insert(biases);
  }
  EXPECT_EQ(seen.size(), kCorners);
}

TEST(ImuCorners, TheNominalIsSeparateFromTheCorners) {
  const marv_plant_imu_config nominal = marv::sim::imu_profile_config();
  for (std::size_t i = 0; i < kAxes; ++i) EXPECT_EQ(bias_of(nominal, i), 0.0) << "axis " << i;
  const auto zero = marv::sim::imu_corner_config(std::array<int, kAxes>{});
  ASSERT_TRUE(zero.has_value());
  for (std::size_t i = 0; i < kAxes; ++i) EXPECT_EQ(bias_of(*zero, i), 0.0) << "axis " << i;
  expect_same_except_bias(*zero, nominal);
}

TEST(ImuCorners, ASignOutsideMinusOneZeroOneIsRefused) {
  for (std::size_t i = 0; i < kAxes; ++i) {
    for (const int bad : {2, -2}) {  // the nearest values outside {-1, 0, +1}
      std::array<int, kAxes> s{};
      s[i] = bad;
      EXPECT_FALSE(marv::sim::imu_corner_config(s).has_value()) << "axis " << i << " sign " << bad;
    }
  }
}

}  // namespace
