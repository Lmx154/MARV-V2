#include <gtest/gtest.h>

#include <array>

#include "marv/hal/hal.hpp"
#include "marv/hal_sim/hal_sim.hpp"
#include "marv/types/actuator.hpp"
#include "marv/types/manual_control.hpp"

namespace marv {
namespace {

constexpr hal_sim::TickPeriod kPeriod{625, 4};

class ManualControlTest : public ::testing::Test {
 protected:
  void SetUp() override { hal_sim::setup(kPeriod, motor_latch, servo_latch); }

  std::array<DshotValue, kQuadXMotors> motor_latch{};
  std::array<ServoUs, 0> servo_latch{};
};

TEST_F(ManualControlTest, NothingStagedPullsNoFrameAndEveryFieldZero) {
  hal_sim::begin_tick(0);
  const ManualControl m = hal_manual_control();
  EXPECT_EQ(m.frames, 0U);
  EXPECT_EQ(m.t_us, 0U);
  EXPECT_EQ(m.roll, 0.0F);
  EXPECT_EQ(m.pitch, 0.0F);
  EXPECT_EQ(m.yaw, 0.0F);
  EXPECT_EQ(m.throttle, 0.0F);
  EXPECT_EQ(m.link_quality, 0.0F);
  EXPECT_EQ(m.link_t_us, 0U);
  for (const float a : m.channels) {
    EXPECT_EQ(a, 0.0F);
  }
  hal_sim::end_tick();
}

TEST_F(ManualControlTest, AStagedStructIsPulledVerbatimFromTheNextTickOnAndStaysUntilReplaced) {
  ManualControl staged{};
  staged.t_us = 123;
  staged.frames = 7;
  staged.roll = -1.0F;
  staged.pitch = 1.0F;
  staged.yaw = -0.5F;
  staged.throttle = 1.0F;
  staged.channels.front() = 1.0F;
  staged.channels.back() = -1.0F;
  staged.link_quality = 0.5F;
  staged.link_t_us = 99;
  hal_sim::stage_manual_control(staged);
  for (Tick n = 0; n < 3; ++n) {
    hal_sim::begin_tick(n);
    const ManualControl m = hal_manual_control();
    EXPECT_EQ(m.t_us, 123U);
    EXPECT_EQ(m.frames, 7U);
    EXPECT_EQ(m.roll, -1.0F);
    EXPECT_EQ(m.pitch, 1.0F);
    EXPECT_EQ(m.yaw, -0.5F);
    EXPECT_EQ(m.throttle, 1.0F);
    EXPECT_EQ(m.channels.front(), 1.0F);
    EXPECT_EQ(m.channels.back(), -1.0F);
    EXPECT_EQ(m.link_quality, 0.5F);
    EXPECT_EQ(m.link_t_us, 99U);
    hal_sim::end_tick();
  }
  ManualControl later{};
  later.frames = 8;
  hal_sim::stage_manual_control(later);
  hal_sim::begin_tick(3);
  EXPECT_EQ(hal_manual_control().frames, 8U);
  EXPECT_EQ(hal_manual_control().throttle, 0.0F);
  hal_sim::end_tick();
}

TEST_F(ManualControlTest, SetupClearsAStagedStruct) {
  ManualControl staged{};
  staged.frames = 3;
  hal_sim::stage_manual_control(staged);
  hal_sim::setup(kPeriod, motor_latch, servo_latch);
  EXPECT_EQ(hal_manual_control().frames, 0U);
}

using ManualControlDeathTest = ManualControlTest;

TEST_F(ManualControlDeathTest, StagingInsideATickPanics) {
  hal_sim::begin_tick(0);
  EXPECT_DEATH(hal_sim::stage_manual_control({}), "hal_panic: .*stage_manual_control inside a tick");
}

}  // namespace
}  // namespace marv
