// T1 known-answer tests of the FC3v2 IMU decode (fw/hal/target/decode, M1 P9): the datasheet sensitivities in SI, the
// chip-to-FRD mounting map, the ImuSample flag contract (imu_sample.hpp: valid bits, saturation per FRD axis, a field
// with its valid bit clear is exactly 0, TempValid implies temp_k > 0) and the stamping rule (strictly increasing t_us).
// Expected values are restated here from ST DS13476 Rev 5 and the SI, not read from the code under test.
#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <limits>

#include <marv/hal_target/lsm6dsv_decode.hpp>
#include <marv/types/imu_sample.hpp>

namespace marv::hal_target {
namespace {

// DS13476 Table 3: 70 mdps/LSB at ±2000 dps, 0.488 mg/LSB at ±16 g; Table 5: 256 LSB/°C, 0 LSB at 25 °C.
constexpr double kGyroRadPerLsb = 0.070 * 3.14159265358979323846 / 180.0;
constexpr double kAccelMs2PerLsb = 0.488e-3 * 9.80665;
constexpr double kTempAtZeroK = 25.0 + 273.15;
// The madflight FC3v2 Rev-C map of the sensor profile: (-x, +y, -z), no permutation.
constexpr ImuMounting kBoard{{0, 1, 2}, {-1, 1, -1}};
constexpr std::uint8_t kGyroTag = 0x01;
constexpr std::uint8_t kAccelTag = 0x02;
constexpr std::uint32_t kOdrHz = 7680;

FifoWord word(std::uint8_t sensor, std::uint8_t slot, std::int16_t x, std::int16_t y, std::int16_t z) {
  FifoWord w{};
  w[0] = static_cast<std::uint8_t>((sensor << 3) | (slot << 1));
  const std::int16_t v[3] = {x, y, z};
  for (std::size_t a = 0; a < 3; ++a) {
    const auto u = static_cast<std::uint16_t>(v[a]);
    w[1 + 2 * a] = static_cast<std::uint8_t>(u & 0xFF);
    w[2 + 2 * a] = static_cast<std::uint8_t>(u >> 8);
  }
  return w;
}

constexpr std::uint32_t kBothValid = (1U << 6) | (1U << 7);  // GyroValid, AccelValid (imu_sample.hpp order)

TEST(Lsm6dsvDecode, ScalesAreTheDatasheetSensitivitiesInSi) {
  const Lsm6dsvScales s = lsm6dsv_scales();
  EXPECT_FLOAT_EQ(s.gyro_rad_s_per_lsb, static_cast<float>(kGyroRadPerLsb));
  EXPECT_FLOAT_EQ(s.accel_m_s2_per_lsb, static_cast<float>(kAccelMs2PerLsb));
  EXPECT_FLOAT_EQ(s.temp_k_per_lsb, 1.0F / 256.0F);
  EXPECT_FLOAT_EQ(s.temp_k_at_zero_lsb, static_cast<float>(kTempAtZeroK));
}

TEST(Lsm6dsvDecode, PairOfOneSlotIsOneFrdSampleOnTheBoardMap) {
  Lsm6dsvAssembler a(kBoard, lsm6dsv_scales());
  ImuSample s{};
  EXPECT_EQ(a.push(word(kGyroTag, 2, 1000, -2000, 3000), s), WordResult::Held);
  ASSERT_EQ(a.push(word(kAccelTag, 2, 0, 0, 2049), s), WordResult::Sample);
  EXPECT_EQ(s.t_us, 0U);
  EXPECT_FLOAT_EQ(s.gyro_rad_s[0], static_cast<float>(-1000 * kGyroRadPerLsb));
  EXPECT_FLOAT_EQ(s.gyro_rad_s[1], static_cast<float>(-2000 * kGyroRadPerLsb));
  EXPECT_FLOAT_EQ(s.gyro_rad_s[2], static_cast<float>(-3000 * kGyroRadPerLsb));
  // Flat, component side up: the chip reads +1 g on its z axis (2049 LSB); FRD z is down, so the specific force is -g.
  EXPECT_FLOAT_EQ(s.accel_m_s2[0], 0.0F);
  EXPECT_FLOAT_EQ(s.accel_m_s2[1], 0.0F);
  EXPECT_FLOAT_EQ(s.accel_m_s2[2], static_cast<float>(-2049 * kAccelMs2PerLsb));
  EXPECT_NEAR(s.accel_m_s2[2], -9.80665F, 0.01F);
  EXPECT_EQ(s.flags, kBothValid);
  EXPECT_EQ(s.temp_k, 0.0F);
}

TEST(Lsm6dsvDecode, AccelerometerFirstPairsTheSame) {
  Lsm6dsvAssembler a(kBoard, lsm6dsv_scales());
  ImuSample s{};
  EXPECT_EQ(a.push(word(kAccelTag, 0, 100, 0, 0), s), WordResult::Held);
  ASSERT_EQ(a.push(word(kGyroTag, 0, 0, 100, 0), s), WordResult::Sample);
  EXPECT_FLOAT_EQ(s.accel_m_s2[0], static_cast<float>(-100 * kAccelMs2PerLsb));
  EXPECT_FLOAT_EQ(s.gyro_rad_s[1], static_cast<float>(100 * kGyroRadPerLsb));
  EXPECT_EQ(s.flags, kBothValid);
}

TEST(Lsm6dsvDecode, PermutedMapMovesValuesAndSaturationBits) {
  const ImuMounting m{{2, 0, 1}, {1, -1, 1}};  // FRD x = +chip z, FRD y = -chip x, FRD z = +chip y
  ASSERT_TRUE(valid(m));
  Lsm6dsvAssembler a(m, lsm6dsv_scales());
  ImuSample s{};
  constexpr std::int16_t kMax = std::numeric_limits<std::int16_t>::max();
  constexpr std::int16_t kMin = std::numeric_limits<std::int16_t>::min();
  EXPECT_EQ(a.push(word(kGyroTag, 1, 10, 20, kMax), s), WordResult::Held);
  ASSERT_EQ(a.push(word(kAccelTag, 1, kMin, 5, 7), s), WordResult::Sample);
  EXPECT_FLOAT_EQ(s.gyro_rad_s[0], static_cast<float>(kMax * kGyroRadPerLsb));
  EXPECT_FLOAT_EQ(s.gyro_rad_s[1], static_cast<float>(-10 * kGyroRadPerLsb));
  EXPECT_FLOAT_EQ(s.gyro_rad_s[2], static_cast<float>(20 * kGyroRadPerLsb));
  EXPECT_FLOAT_EQ(s.accel_m_s2[0], static_cast<float>(7 * kAccelMs2PerLsb));
  EXPECT_FLOAT_EQ(s.accel_m_s2[1], static_cast<float>(-static_cast<double>(kMin) * kAccelMs2PerLsb));
  EXPECT_FLOAT_EQ(s.accel_m_s2[2], static_cast<float>(5 * kAccelMs2PerLsb));
  // Gyro chip z at the limit -> FRD x (GyroSatX, bit 0); accel chip x at the limit -> FRD y (AccelSatY, bit 4).
  EXPECT_EQ(s.flags, kBothValid | (1U << 0) | (1U << 4));
}

TEST(Lsm6dsvDecode, MountingMustBeASignedPermutation) {
  EXPECT_TRUE(valid(kBoard));
  EXPECT_FALSE(valid(ImuMounting{{0, 0, 2}, {1, 1, 1}}));
  EXPECT_FALSE(valid(ImuMounting{{0, 1, 3}, {1, 1, 1}}));
  EXPECT_FALSE(valid(ImuMounting{{0, 1, 2}, {1, 0, 1}}));
  EXPECT_FALSE(valid(ImuMounting{{0, 1, 2}, {1, 2, 1}}));
}

TEST(Lsm6dsvDecode, UnpairedWordBecomesAPartialSampleWithItsMissingFieldZero) {
  Lsm6dsvAssembler a(kBoard, lsm6dsv_scales());
  ImuSample s{};
  constexpr std::int16_t kMax = std::numeric_limits<std::int16_t>::max();
  EXPECT_EQ(a.push(word(kAccelTag, 0, kMax, kMax, kMax), s), WordResult::Held);
  // The next slot's gyroscope word arrives before slot 0's: slot 0 completes with its accelerometer alone.
  ASSERT_EQ(a.push(word(kGyroTag, 1, 50, 0, 0), s), WordResult::Sample);
  EXPECT_EQ(s.flags, (1U << 7) | (1U << 3) | (1U << 4) | (1U << 5));  // AccelValid and AccelSat X, Y, Z only
  EXPECT_EQ(s.gyro_rad_s[0], 0.0F);
  EXPECT_EQ(s.gyro_rad_s[1], 0.0F);
  EXPECT_EQ(s.gyro_rad_s[2], 0.0F);
  // A repeated sensor in the same slot completes the held word too.
  ASSERT_EQ(a.push(word(kGyroTag, 1, 60, 0, 0), s), WordResult::Sample);
  EXPECT_EQ(s.flags, 1U << 6);  // GyroValid only
  EXPECT_FLOAT_EQ(s.gyro_rad_s[0], static_cast<float>(-50 * kGyroRadPerLsb));
  EXPECT_EQ(s.accel_m_s2[0], 0.0F);
  EXPECT_EQ(s.accel_m_s2[1], 0.0F);
  EXPECT_EQ(s.accel_m_s2[2], 0.0F);
  ASSERT_EQ(a.push(word(kAccelTag, 1, 0, 0, 0), s), WordResult::Sample);
  EXPECT_FLOAT_EQ(s.gyro_rad_s[0], static_cast<float>(-60 * kGyroRadPerLsb));
  EXPECT_EQ(s.flags, kBothValid);
}

TEST(Lsm6dsvDecode, AnyOtherTagIsRefusedAndChangesNothing) {
  Lsm6dsvAssembler a(kBoard, lsm6dsv_scales());
  ImuSample s{};
  EXPECT_EQ(a.push(word(kGyroTag, 3, 1, 2, 3), s), WordResult::Held);
  EXPECT_EQ(a.push(word(0x00, 0, 9, 9, 9), s), WordResult::BadTag);  // "FIFO empty"
  EXPECT_EQ(a.push(word(0x03, 0, 9, 9, 9), s), WordResult::BadTag);  // temperature: not batched by the driver
  EXPECT_EQ(a.push(word(0x04, 0, 9, 9, 9), s), WordResult::BadTag);  // timestamp: not batched by the driver
  ASSERT_EQ(a.push(word(kAccelTag, 3, 0, 0, 0), s), WordResult::Sample);
  EXPECT_FLOAT_EQ(s.gyro_rad_s[0], static_cast<float>(-1 * kGyroRadPerLsb));
  EXPECT_EQ(s.flags, kBothValid);
}

TEST(Lsm6dsvDecode, TemperatureIsCarriedWithTempValidAfterTheFirstReading) {
  Lsm6dsvAssembler a(kBoard, lsm6dsv_scales());
  ImuSample s{};
  a.set_temperature(0);
  static_cast<void>(a.push(word(kGyroTag, 0, 0, 0, 0), s));
  ASSERT_EQ(a.push(word(kAccelTag, 0, 0, 0, 0), s), WordResult::Sample);
  EXPECT_EQ(s.flags, kBothValid | (1U << 8));
  EXPECT_FLOAT_EQ(s.temp_k, static_cast<float>(kTempAtZeroK));
  a.set_temperature(256);
  static_cast<void>(a.push(word(kGyroTag, 1, 0, 0, 0), s));
  ASSERT_EQ(a.push(word(kAccelTag, 1, 0, 0, 0), s), WordResult::Sample);
  EXPECT_FLOAT_EQ(s.temp_k, static_cast<float>(kTempAtZeroK + 1.0));
  a.set_temperature(std::numeric_limits<std::int16_t>::min());  // the lowest reading still gives temp_k > 0
  static_cast<void>(a.push(word(kGyroTag, 2, 0, 0, 0), s));
  ASSERT_EQ(a.push(word(kAccelTag, 2, 0, 0, 0), s), WordResult::Sample);
  EXPECT_FLOAT_EQ(s.temp_k, static_cast<float>(kTempAtZeroK - 128.0));
  EXPECT_GT(s.temp_k, 0.0F);
}

TEST(Lsm6dsvDecode, StampsBackStepByTheNominalPeriodAndStrictlyIncrease) {
  ImuStamper st(kOdrHz);
  // A batch of three at t_irq = 1 s: T = 1e6 / 7680 us = 130.208 us, rounded per back-step.
  EXPECT_EQ(st.stamp(1000000, 2), 1000000U - 260U);
  EXPECT_EQ(st.stamp(1000000, 1), 1000000U - 130U);
  EXPECT_EQ(st.stamp(1000000, 0), 1000000U);
  EXPECT_EQ(st.clamps(), 0U);
  // Three back-steps: 390.625 us rounds to 391; a back-step at or below the previous stamp is clamped to it + 1.
  EXPECT_EQ(st.stamp(1000300, 3), 1000001U);  // 1000300 - 391 = 999909 is not above 1000000
  EXPECT_EQ(st.clamps(), 1U);
  EXPECT_EQ(st.stamp(1000300, 2), 1000300U - 260U);
  EXPECT_EQ(st.clamps(), 1U);
  EXPECT_EQ(st.stamp(1000030, 0), 1000041U);  // a late-entered handler's stamp below the previous one
  EXPECT_EQ(st.clamps(), 2U);
}

TEST(Lsm6dsvDecode, StampBeforeOnePeriodHasElapsedIsNotNegative) {
  ImuStamper st(kOdrHz);
  EXPECT_EQ(st.stamp(100, 1), 0U);
  EXPECT_EQ(st.stamp(100, 0), 100U);
  EXPECT_EQ(st.clamps(), 0U);
}

}  // namespace
}  // namespace marv::hal_target
