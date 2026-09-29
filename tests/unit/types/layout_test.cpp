#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "marv/types/actuator.hpp"
#include "marv/types/imu_sample.hpp"
#include "marv/types/time.hpp"

namespace marv {
namespace {

TEST(TimeTypes, AreSixtyFourBitUnsigned) {
  static_assert(std::is_same_v<TimeUs, std::uint64_t>);
  static_assert(std::is_same_v<Tick, std::uint64_t>);
  SUCCEED();
}

TEST(MotorMapping, MatchesCoreSection3ByNameAndNumber) {
  struct Row {
    Motor motor;
    unsigned logical_number;
    std::size_t index;
    const char* position;
  };
  // Core 3: viewed from above, 1 front-right, 2 rear-left, 3 front-left, 4 rear-right; index = number - 1.
  const std::array<Row, 4> table{{{Motor::M1FrontRight, 1, 0, "front-right"},
                                  {Motor::M2RearLeft, 2, 1, "rear-left"},
                                  {Motor::M3FrontLeft, 3, 2, "front-left"},
                                  {Motor::M4RearRight, 4, 3, "rear-right"}}};
  for (const Row& row : table) {
    EXPECT_EQ(static_cast<unsigned>(row.motor), row.logical_number) << row.position;
    EXPECT_EQ(motor_index(row.motor), row.index) << row.position;
    EXPECT_EQ(motor_from_index(row.index), row.motor) << row.position;
  }
  EXPECT_EQ(kQuadXMotors, table.size());
}

TEST(MotorMapping, IsABijectionBetweenMotorsAndIndices) {
  std::array<bool, kQuadXMotors> index_hit{};
  for (std::size_t i = 0; i < kQuadXMotors; ++i) {
    const Motor m = motor_from_index(i);
    EXPECT_EQ(motor_index(m), i);
    ASSERT_LT(motor_index(m), kQuadXMotors);
    EXPECT_FALSE(index_hit[motor_index(m)]) << "index " << i << " maps onto an index already used";
    index_hit[motor_index(m)] = true;
  }
  for (std::size_t i = 0; i < kQuadXMotors; ++i) {
    EXPECT_TRUE(index_hit[i]) << "index " << i << " is not the image of any motor";
  }
  static_assert(motor_index(motor_from_index(0)) == 0);
  static_assert(motor_from_index(motor_index(Motor::M4RearRight)) == Motor::M4RearRight);
}

TEST(ImuFlags, HaveTheDeclaredBitValues) {
  EXPECT_EQ(imu_flag(ImuFlag::GyroSatX), std::uint32_t{1} << 0);
  EXPECT_EQ(imu_flag(ImuFlag::GyroSatY), std::uint32_t{1} << 1);
  EXPECT_EQ(imu_flag(ImuFlag::GyroSatZ), std::uint32_t{1} << 2);
  EXPECT_EQ(imu_flag(ImuFlag::AccelSatX), std::uint32_t{1} << 3);
  EXPECT_EQ(imu_flag(ImuFlag::AccelSatY), std::uint32_t{1} << 4);
  EXPECT_EQ(imu_flag(ImuFlag::AccelSatZ), std::uint32_t{1} << 5);
  EXPECT_EQ(imu_flag(ImuFlag::GyroValid), std::uint32_t{1} << 6);
  EXPECT_EQ(imu_flag(ImuFlag::AccelValid), std::uint32_t{1} << 7);
  EXPECT_EQ(imu_flag(ImuFlag::TempValid), std::uint32_t{1} << 8);
  EXPECT_EQ(static_cast<unsigned>(ImuFlag::Count), 9U);
  EXPECT_EQ(kImuFlagsDefined, 0x1FFU);
}

TEST(ImuFlags, DefinedMaskIsTheUnionOfTheDefinedBitsOnly) {
  std::uint32_t all = 0;
  for (unsigned i = 0; i < static_cast<unsigned>(ImuFlag::Count); ++i) {
    const std::uint32_t bit = imu_flag(static_cast<ImuFlag>(i));
    EXPECT_EQ(all & bit, 0U) << "flag " << i << " overlaps an earlier one";
    all |= bit;
  }
  EXPECT_EQ(all, kImuFlagsDefined);
  EXPECT_EQ(imu_flag(ImuFlag::Count) & kImuFlagsDefined, 0U);
}

TEST(BoundaryStructs, AreTriviallyCopyableAndStandardLayout) {
  static_assert(std::is_trivially_copyable_v<ImuSample>);
  static_assert(std::is_standard_layout_v<ImuSample>);
  static_assert(std::is_trivially_copyable_v<ActuatorOutput<kQuadXMotors, 0>>);
  static_assert(std::is_standard_layout_v<ActuatorOutput<kQuadXMotors, 0>>);
  static_assert(std::is_trivially_copyable_v<ActuatorOutput<kQuadXMotors, 2>>);
  static_assert(std::is_standard_layout_v<ActuatorOutput<kQuadXMotors, 2>>);
  static_assert(std::is_trivially_copyable_v<DshotValue>);
  static_assert(std::is_standard_layout_v<DshotValue>);
  static_assert(std::is_trivially_copyable_v<ServoUs>);
  static_assert(std::is_standard_layout_v<ServoUs>);
  SUCCEED();
}

TEST(ActuatorOutput, DefaultsToStopAndNoPulse) {
  const ActuatorOutput<kQuadXMotors, 2> out{};
  for (const DshotValue& m : out.motor) {
    EXPECT_EQ(m, DshotValue::stop());
  }
  for (const ServoUs& s : out.servo) {
    EXPECT_EQ(s.us, 0);
  }
  EXPECT_EQ(out.motor.size(), kQuadXMotors);
  EXPECT_EQ(out.servo.size(), 2U);
}

TEST(ActuatorOutput, IndexesMotorsByLogicalNumberMinusOne) {
  ActuatorOutput<kQuadXMotors, 0> out{};
  const auto v = DshotValue::from_raw(100);
  ASSERT_TRUE(v.has_value());
  out.motor[motor_index(Motor::M3FrontLeft)] = *v;
  EXPECT_EQ(out.motor[2], *v);
  EXPECT_EQ(out.motor[0], DshotValue::stop());
  EXPECT_EQ(out.motor[1], DshotValue::stop());
  EXPECT_EQ(out.motor[3], DshotValue::stop());
}

}  // namespace
}  // namespace marv
