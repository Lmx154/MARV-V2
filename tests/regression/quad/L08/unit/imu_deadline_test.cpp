#include <gtest/gtest.h>

#include <array>
#include <cstdint>

#include "marv/hal/hal.hpp"
#include "marv/hal_sim/hal_sim.hpp"
#include "marv/types/actuator.hpp"

namespace marv {
namespace {

constexpr hal_sim::TickPeriod kPeriod{625, 4};

class ImuDeadlineTest : public ::testing::Test {
 protected:
  void SetUp() override { hal_sim::setup(kPeriod, motor_latch, servo_latch); }

  std::array<DshotValue, kQuadXMotors> motor_latch{};
  std::array<ServoUs, 0> servo_latch{};
};

TEST_F(ImuDeadlineTest, NothingStagedPullsZero) {
  hal_sim::begin_tick(0);
  EXPECT_EQ(hal_imu_deadline_misses(), 0U);
  hal_sim::end_tick();
}

TEST_F(ImuDeadlineTest, AStagedCountIsPulledVerbatimFromTheNextTickOnAndStaysUntilReplaced) {
  hal_sim::stage_imu_deadline_misses(3);
  for (Tick n = 0; n < 3; ++n) {
    hal_sim::begin_tick(n);
    EXPECT_EQ(hal_imu_deadline_misses(), 3U);
    hal_sim::end_tick();
  }
  hal_sim::stage_imu_deadline_misses(4);
  hal_sim::begin_tick(3);
  EXPECT_EQ(hal_imu_deadline_misses(), 4U);
  hal_sim::end_tick();
}

TEST_F(ImuDeadlineTest, SetupClearsAStagedCount) {
  hal_sim::stage_imu_deadline_misses(5);
  hal_sim::setup(kPeriod, motor_latch, servo_latch);
  EXPECT_EQ(hal_imu_deadline_misses(), 0U);
}

using ImuDeadlineDeathTest = ImuDeadlineTest;

TEST_F(ImuDeadlineDeathTest, StagingInsideATickPanics) {
  hal_sim::begin_tick(0);
  EXPECT_DEATH(hal_sim::stage_imu_deadline_misses(1), "hal_panic: .*stage_imu_deadline_misses inside a tick");
}

}  // namespace
}  // namespace marv
