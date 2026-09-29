#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <span>

#include "marv/hal/hal.hpp"
#include "marv/hal_sim/hal_sim.hpp"
#include "marv/types/actuator.hpp"

namespace marv {
namespace {

constexpr std::size_t kServos = 2;
using Out = ActuatorOutput<kQuadXMotors, kServos>;
constexpr hal_sim::TickPeriod kPeriod{625, 4};

DshotValue dshot(std::uint16_t raw) {
  const auto v = DshotValue::from_raw(raw);
  EXPECT_TRUE(v.has_value()) << raw;
  return v.value_or(DshotValue::stop());
}

class ActuatorLatchTest : public ::testing::Test {
 protected:
  void SetUp() override { hal_sim::setup(kPeriod, motor_latch, servo_latch); }

  static Out sample_output() {
    Out o{};
    o.motor[0] = dshot(100);
    o.motor[1] = dshot(48);
    o.motor[2] = dshot(2047);
    o.motor[3] = dshot(0);
    o.servo[0] = ServoUs{1500};
    o.servo[1] = ServoUs{900};
    return o;
  }

  std::array<DshotValue, kQuadXMotors> motor_latch{};
  std::array<ServoUs, kServos> servo_latch{};
};

TEST_F(ActuatorLatchTest, SetupLeavesTheLatchAtStopAndNoPulse) {
  motor_latch.fill(dshot(500));
  servo_latch.fill(ServoUs{1000});
  hal_sim::setup(kPeriod, motor_latch, servo_latch);
  for (const DshotValue& m : motor_latch) {
    EXPECT_EQ(m, DshotValue::stop());
  }
  for (const ServoUs& s : servo_latch) {
    EXPECT_EQ(s.us, 0);
  }
}

TEST_F(ActuatorLatchTest, AValidWriteInsideATickLandsInTheLatch) {
  const Out out = sample_output();
  hal_sim::begin_tick(0);
  hal_actuators_write(out);
  hal_sim::end_tick();
  EXPECT_EQ(motor_latch, out.motor);
  for (std::size_t i = 0; i < kServos; ++i) {
    EXPECT_EQ(servo_latch[i].us, out.servo[i].us) << i;
  }
}

TEST_F(ActuatorLatchTest, TheSpanOverloadWritesTheSameLatch) {
  const Out out = sample_output();
  hal_sim::begin_tick(0);
  hal_actuators_write(std::span<const DshotValue>{out.motor}, std::span<const ServoUs>{out.servo});
  hal_sim::end_tick();
  EXPECT_EQ(motor_latch, out.motor);
}

TEST_F(ActuatorLatchTest, TheLatchHoldsAcrossTicksThatDoNotWrite) {
  const Out out = sample_output();
  hal_sim::begin_tick(0);
  hal_actuators_write(out);
  hal_sim::end_tick();
  hal_sim::begin_tick(1);
  hal_sim::end_tick();
  EXPECT_EQ(motor_latch, out.motor);
  EXPECT_EQ(servo_latch[0].us, 1500);
}

TEST_F(ActuatorLatchTest, EachTickMayWriteOnce) {
  Out a = sample_output();
  Out b = sample_output();
  b.motor[0] = dshot(200);
  hal_sim::begin_tick(0);
  hal_actuators_write(a);
  hal_sim::end_tick();
  hal_sim::begin_tick(1);
  hal_actuators_write(b);
  hal_sim::end_tick();
  EXPECT_EQ(motor_latch, b.motor);
}

TEST_F(ActuatorLatchTest, ZeroServosIsAValidLatchSize) {
  std::array<DshotValue, kQuadXMotors> m{};
  std::array<ServoUs, 0> s{};
  hal_sim::setup(kPeriod, m, s);
  ActuatorOutput<kQuadXMotors, 0> out{};
  out.motor[1] = dshot(300);
  hal_sim::begin_tick(0);
  hal_actuators_write(out);
  hal_sim::end_tick();
  EXPECT_EQ(m, out.motor);
}

using ActuatorLatchDeathTest = ActuatorLatchTest;

TEST_F(ActuatorLatchDeathTest, WriteBeforeAnyTickPanics) {
  const Out out = sample_output();
  EXPECT_DEATH(hal_actuators_write(out), "hal_panic: .*outside a tick");
}

TEST_F(ActuatorLatchDeathTest, WriteAfterEndTickPanics) {
  const Out out = sample_output();
  EXPECT_DEATH(({
                 hal_sim::begin_tick(0);
                 hal_sim::end_tick();
                 hal_actuators_write(out);
               }),
               "hal_panic: .*outside a tick");
}

TEST_F(ActuatorLatchDeathTest, SecondWriteInOneTickPanics) {
  const Out out = sample_output();
  EXPECT_DEATH(({
                 hal_sim::begin_tick(0);
                 hal_actuators_write(out);
                 hal_actuators_write(out);
               }),
               "hal_panic: .*twice in one tick");
}

TEST_F(ActuatorLatchDeathTest, WrongMotorSpanSizePanics) {
  const Out out = sample_output();
  EXPECT_DEATH(({
                 hal_sim::begin_tick(0);
                 hal_actuators_write(std::span<const DshotValue>{out.motor}.first(kQuadXMotors - 1),
                                     std::span<const ServoUs>{out.servo});
               }),
               "hal_panic: .*span size");
}

TEST_F(ActuatorLatchDeathTest, WrongServoSpanSizePanics) {
  const Out out = sample_output();
  EXPECT_DEATH(({
                 hal_sim::begin_tick(0);
                 hal_actuators_write(std::span<const DshotValue>{out.motor},
                                     std::span<const ServoUs>{out.servo}.first(kServos - 1));
               }),
               "hal_panic: .*span size");
}

TEST_F(ActuatorLatchDeathTest, TypedOutputWithTheWrongSizesPanics) {
  const ActuatorOutput<kQuadXMotors, 0> out{};
  EXPECT_DEATH(({
                 hal_sim::begin_tick(0);
                 hal_actuators_write(out);
               }),
               "hal_panic: .*span size");
}

}  // namespace
}  // namespace marv
