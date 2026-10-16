// The quad_fc motor output gate (M1: one state, Disarmed). Through hal_sim: every rate tick writes DshotValue::stop() to
// every motor, a tick that is not a rate tick writes nothing, and the mixer's own output (never stop: its floor is the
// idle command) never reaches the latch, for samples of every kind the rate loop handles (valid, saturated, invalid).
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include <marv/composition.hpp>
#include <marv/hal/hal.hpp>
#include <marv/hal/tick.hpp>
#include <marv/hal_sim/hal_sim.hpp>
#include <marv/mixer/mixer.hpp>
#include <marv/params/param.hpp>
#include <marv/types/actuator.hpp>
#include <marv/types/imu_sample.hpp>

namespace marv {
namespace {

constexpr std::size_t kTicks = 400;
constexpr std::uint16_t kSentinelRaw = 1000;

void init_params_once() {
  static const bool ok = params_init(param_defaults());
  ASSERT_TRUE(ok);
}

DshotValue sentinel() { return DshotValue::from_raw(kSentinelRaw).value_or(DshotValue::stop()); }

enum class Kind : std::uint8_t { Quiet, Moving, Saturated, GyroInvalid, GyroNaN };

ImuSample sample_of(Kind k, std::size_t n) {
  ImuSample s{};
  s.t_us = hal_time_us();
  s.temp_k = 300.0F;
  s.accel_m_s2 = prim::Vec3<float>(0.0F, 0.0F, -9.8F);
  s.flags = imu_flag(ImuFlag::GyroValid) | imu_flag(ImuFlag::AccelValid) | imu_flag(ImuFlag::TempValid);
  const float phase = static_cast<float>(n) * 0.05F;
  switch (k) {
    case Kind::Quiet:
      break;
    case Kind::Moving:
      s.gyro_rad_s = prim::Vec3<float>(2.0F * std::sin(phase), 1.5F * std::cos(phase), 0.5F * std::sin(3.0F * phase));
      break;
    case Kind::Saturated:
      s.gyro_rad_s = prim::Vec3<float>(35.0F, -35.0F, 35.0F);
      s.flags |= imu_flag(ImuFlag::GyroSatX) | imu_flag(ImuFlag::GyroSatY) | imu_flag(ImuFlag::GyroSatZ);
      break;
    case Kind::GyroInvalid:
      s.flags &= ~imu_flag(ImuFlag::GyroValid);
      break;
    case Kind::GyroNaN:
      s.gyro_rad_s = prim::Vec3<float>(std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F);
      break;
  }
  return s;
}

class QuadFcGateTest : public ::testing::TestWithParam<Kind> {
 protected:
  void SetUp() override {
    init_params_once();
    period_ = {static_cast<std::uint32_t>(param_value<ParamId::tick_period_num_us>()),
               static_cast<std::uint32_t>(param_value<ParamId::tick_period_den>())};
    divisor_ = static_cast<std::size_t>(param_value<ParamId::rate_loop_divisor>());
    hal_sim::setup(period_, motor_latch_, servo_latch_);
  }

  hal_sim::TickPeriod period_{};
  std::size_t divisor_ = 1;
  std::array<DshotValue, composition::kMotors> motor_latch_{};
  std::array<ServoUs, composition::kServos> servo_latch_{};
};

TEST_F(QuadFcGateTest, TheMixerOutputOnAZeroRequestIsNeverStop) {
  const mixer::MixerConfig<float> cfg = mixer::load_config();
  const mixer::MixerOutput<float> out = mixer::mix(cfg, mixer::Request<float>{});
  for (const DshotValue& m : out.dshot) {
    EXPECT_NE(m, DshotValue::stop());
  }
}

TEST_P(QuadFcGateTest, EveryRateTickWritesStopToEveryMotorAndNoOtherTickWrites) {
  composition::init();
  std::size_t writes = 0;
  for (std::size_t n = 0; n < kTicks; ++n) {
    motor_latch_.fill(sentinel());
    hal_sim::begin_tick(n);
    composition::tick(sample_of(GetParam(), n));
    hal_sim::end_tick();
    const bool rate_tick = n % divisor_ == 0;
    for (const DshotValue& m : motor_latch_) {
      EXPECT_EQ(m, rate_tick ? DshotValue::stop() : sentinel()) << "tick " << n;
    }
    writes += rate_tick ? 1U : 0U;
  }
  EXPECT_EQ(writes, (kTicks + divisor_ - 1) / divisor_);
}

INSTANTIATE_TEST_SUITE_P(Samples, QuadFcGateTest,
                         ::testing::Values(Kind::Quiet, Kind::Moving, Kind::Saturated, Kind::GyroInvalid,
                                           Kind::GyroNaN));

}  // namespace
}  // namespace marv
