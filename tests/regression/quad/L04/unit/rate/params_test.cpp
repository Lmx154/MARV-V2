// The L4 rate configuration read from the product parameter set (rate::from_params and rate::load_config): every
// field equals its parameter, the period is T = rate_loop_divisor * tick_period_num_us / tick_period_den microseconds,
// and the loaded configuration validates.
//
// The file asserts no product value, only equality with the parameter reads.
#include <gtest/gtest.h>

#include <array>
#include <cstddef>

#include <marv/params/param.hpp>
#include <marv/prim/constants.hpp>
#include <marv/rate/rate_loop.hpp>

namespace {

using namespace marv;
using marv::rate::RateConfig;

constexpr std::size_t kN = rate::kMotors;

bool init_params() {
  static const bool ok = params_init(param_defaults());
  return ok;
}

#define REQUIRE_PARAMS() ASSERT_TRUE(init_params()) << "params_init"

TEST(L4RateParams, EveryFieldEqualsItsParameter) {
  REQUIRE_PARAMS();
  const RateConfig<float> c = rate::from_params();
  EXPECT_EQ(c.kp[rate::kRollAxis], param_get(ParamId::rate_kp_roll).value.f32);
  EXPECT_EQ(c.kp[rate::kPitchAxis], param_get(ParamId::rate_kp_pitch).value.f32);
  EXPECT_EQ(c.kp[rate::kYawAxis], param_get(ParamId::rate_kp_yaw).value.f32);
  EXPECT_EQ(c.ki[rate::kRollAxis], param_get(ParamId::rate_ki_roll).value.f32);
  EXPECT_EQ(c.ki[rate::kPitchAxis], param_get(ParamId::rate_ki_pitch).value.f32);
  EXPECT_EQ(c.ki[rate::kYawAxis], param_get(ParamId::rate_ki_yaw).value.f32);
  EXPECT_EQ(c.kd[rate::kRollAxis], param_get(ParamId::rate_kd_roll).value.f32);
  EXPECT_EQ(c.kd[rate::kPitchAxis], param_get(ParamId::rate_kd_pitch).value.f32);
  EXPECT_EQ(c.kd[rate::kYawAxis], param_get(ParamId::rate_kd_yaw).value.f32);
  EXPECT_EQ(c.tau_ref[rate::kRollAxis], param_get(ParamId::rate_tau_ref_roll).value.f32);
  EXPECT_EQ(c.tau_ref[rate::kPitchAxis], param_get(ParamId::rate_tau_ref_pitch).value.f32);
  EXPECT_EQ(c.tau_ref[rate::kYawAxis], param_get(ParamId::rate_tau_ref_yaw).value.f32);

  const std::array<ParamId, kN> xs{ParamId::rotor_position_m1_x, ParamId::rotor_position_m2_x,
                                   ParamId::rotor_position_m3_x, ParamId::rotor_position_m4_x};
  const std::array<ParamId, kN> ys{ParamId::rotor_position_m1_y, ParamId::rotor_position_m2_y,
                                   ParamId::rotor_position_m3_y, ParamId::rotor_position_m4_y};
  const std::array<ParamId, kN> ss{ParamId::rotor_yaw_sign_m1, ParamId::rotor_yaw_sign_m2,
                                   ParamId::rotor_yaw_sign_m3, ParamId::rotor_yaw_sign_m4};
  for (std::size_t i = 0; i < kN; ++i) {
    EXPECT_EQ(c.rotor_x[i], param_get(xs[i]).value.f32) << "motor " << i + 1;
    EXPECT_EQ(c.rotor_y[i], param_get(ys[i]).value.f32) << "motor " << i + 1;
    EXPECT_EQ(c.yaw_sign[i], static_cast<float>(param_get(ss[i]).value.i32)) << "motor " << i + 1;
  }
  EXPECT_EQ(c.torque_ratio, param_get(ParamId::rotor_torque_ratio).value.f32);
}

TEST(L4RateParams, PeriodIsTheRateLoopPeriodExactlyInFloat) {
  REQUIRE_PARAMS();
  const RateConfig<float> c = rate::from_params();
  // T in microseconds = rate_loop_divisor * tick_period_num_us / tick_period_den, from the parameter reads only: the
  // product of two integers over an integer is one float quotient, and the conversion to seconds is one division.
  const float period_us = static_cast<float>(param_value<ParamId::rate_loop_divisor>()) *
                          static_cast<float>(param_value<ParamId::tick_period_num_us>()) /
                          static_cast<float>(param_value<ParamId::tick_period_den>());
  EXPECT_EQ(c.period, period_us / static_cast<float>(prim::kMicrosecondsPerSecond));
}

TEST(L4RateParams, LoadedConfigurationValidates) {
  REQUIRE_PARAMS();
  const RateConfig<float> c = rate::load_config();
  EXPECT_EQ(rate::validate(c), rate::ConfigError::None);
  const RateConfig<float> p = rate::from_params();
  EXPECT_EQ(c.period, p.period);
  EXPECT_EQ(c.kp[rate::kPitchAxis], p.kp[rate::kPitchAxis]);
}

}  // namespace
