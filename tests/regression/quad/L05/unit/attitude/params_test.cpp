// The L5 attitude configuration read from the product parameter set (attitude::from_params and attitude::load_config):
// every field equals its parameter, the period is T_a = att_loop_ratio * rate_loop_divisor * tick_period_num_us /
// tick_period_den microseconds, the loaded configuration validates, and an invalid parameter value is refused by
// require_valid.
//
// The file asserts no product value, only equality with the parameter reads.
#include <gtest/gtest.h>

#include <cmath>
#include <csignal>
#include <cstdlib>
#include <limits>

#include <sys/prctl.h>

#include <marv/attitude/config.hpp>
#include <marv/params/param.hpp>
#include <marv/prim/constants.hpp>

namespace {

using namespace marv;
using marv::attitude::AttitudeConfig;

bool init_params() {
  static const bool ok = params_init(param_defaults());
  return ok;
}

#define REQUIRE_PARAMS() ASSERT_TRUE(init_params()) << "params_init"

TEST(L5AttitudeParams, EveryFieldEqualsItsParameter) {
  REQUIRE_PARAMS();
  const AttitudeConfig<float> c = attitude::from_params();
  EXPECT_EQ(c.kp, param_get(ParamId::att_kp).value.f32);
  EXPECT_EQ(c.yaw_weight, param_get(ParamId::att_yaw_weight).value.f32);
  EXPECT_EQ(c.tilt_max, param_get(ParamId::angle_tilt_max).value.f32);
  EXPECT_EQ(c.yaw_deadband, param_get(ParamId::yaw_deadband).value.f32);
  EXPECT_EQ(c.yaw_alpha_min, param_get(ParamId::att_yaw_alpha_min).value.f32);
  EXPECT_EQ(c.yaw_t_cross, param_get(ParamId::att_yaw_t_cross).value.f32);
  EXPECT_EQ(c.rate_max[attitude::kRollAxis], param_get(ParamId::rate_max_roll).value.f32);
  EXPECT_EQ(c.rate_max[attitude::kPitchAxis], param_get(ParamId::rate_max_pitch).value.f32);
  EXPECT_EQ(c.rate_max[attitude::kYawAxis], param_get(ParamId::rate_max_yaw).value.f32);
}

TEST(L5AttitudeParams, PeriodIsTheAttitudePeriodExactlyInFloat) {
  REQUIRE_PARAMS();
  const AttitudeConfig<float> c = attitude::from_params();
  // T_a in microseconds = att_loop_ratio * rate_loop_divisor * tick_period_num_us / tick_period_den, from the parameter
  // reads only: integers over an integer are one float quotient, and the conversion to seconds is one division.
  const float ticks = static_cast<float>(param_value<ParamId::att_loop_ratio>()) *
                      static_cast<float>(param_value<ParamId::rate_loop_divisor>());
  const float period_us = ticks * static_cast<float>(param_value<ParamId::tick_period_num_us>()) /
                          static_cast<float>(param_value<ParamId::tick_period_den>());
  EXPECT_EQ(c.period, period_us / static_cast<float>(prim::kMicrosecondsPerSecond));
}

TEST(L5AttitudeParams, PeriodIsAttLoopRatioTimesTheRateLoopPeriodToOneUlp) {
  REQUIRE_PARAMS();
  const AttitudeConfig<float> c = attitude::from_params();
  const float rate_period = static_cast<float>(param_value<ParamId::rate_loop_divisor>()) *
                            static_cast<float>(param_value<ParamId::tick_period_num_us>()) /
                            static_cast<float>(param_value<ParamId::tick_period_den>()) /
                            static_cast<float>(prim::kMicrosecondsPerSecond);
  const float want = static_cast<float>(param_value<ParamId::att_loop_ratio>()) * rate_period;
  // At most four float roundings on each side, relative u = eps/2 each: the two evaluation orders differ by at most
  // 8u = 4 eps.
  EXPECT_NEAR(c.period, want, 4.0F * std::numeric_limits<float>::epsilon() * want);
  // Control: a period one ratio step off is far outside that bound.
  const float off = static_cast<float>(param_value<ParamId::att_loop_ratio>() + 1) * rate_period;
  EXPECT_GT(std::abs(c.period - off), 4.0F * std::numeric_limits<float>::epsilon() * off);
}

TEST(L5AttitudeParams, LoadedConfigurationValidates) {
  REQUIRE_PARAMS();
  const AttitudeConfig<float> c = attitude::load_config();
  EXPECT_EQ(attitude::validate(c), attitude::ConfigError::None);
  const AttitudeConfig<float> p = attitude::from_params();
  EXPECT_EQ(c.period, p.period);
  EXPECT_EQ(c.kp, p.kp);
  EXPECT_EQ(c.yaw_t_cross, p.yaw_t_cross);
}

// The wiring reaches require_valid: a configuration that fails validate is a hal_panic naming the rule (the control of
// the validates test above, on the same from_params read).
TEST(L5AttitudeParams, RequireValidPanicsOnAnInvalidFromParamsConfiguration) {
  REQUIRE_PARAMS();
  AttitudeConfig<float> c = attitude::from_params();
  c.yaw_weight = 0.0F;
  ASSERT_EQ(attitude::validate(c), attitude::ConfigError::YawWeight);
  EXPECT_EXIT(
      {
        (void)prctl(PR_SET_DUMPABLE, 0);
        attitude::require_valid(c);
        std::exit(0);
      },
      ::testing::KilledBySignal(SIGABRT), "attitude: the yaw weight is not in");
}

}  // namespace
