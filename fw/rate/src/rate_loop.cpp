#include "marv/rate/rate_loop.hpp"

#include <marv/hal/hal.hpp>
#include <marv/params/param.hpp>
#include <marv/prim/constants.hpp>

namespace marv::rate {

template class RateLoop<float>;
template ConfigError validate<float>(const RateConfig<float>&) noexcept;

namespace {

template <ParamId Roll, ParamId Pitch, ParamId Yaw>
prim::Vec3<float> load_axes() noexcept {
  return prim::Vec3<float>(param_value<Roll>(), param_value<Pitch>(), param_value<Yaw>());
}

template <ParamId X1, ParamId Y1, ParamId S1, ParamId X2, ParamId Y2, ParamId S2, ParamId X3, ParamId Y3, ParamId S3,
          ParamId X4, ParamId Y4, ParamId S4>
void load_geometry(RateConfig<float>& c) noexcept {
  c.rotor_x = {param_value<X1>(), param_value<X2>(), param_value<X3>(), param_value<X4>()};
  c.rotor_y = {param_value<Y1>(), param_value<Y2>(), param_value<Y3>(), param_value<Y4>()};
  c.yaw_sign = {static_cast<float>(param_value<S1>()), static_cast<float>(param_value<S2>()),
                static_cast<float>(param_value<S3>()), static_cast<float>(param_value<S4>())};
}

const char* message(ConfigError e) noexcept {
  switch (e) {
    case ConfigError::None:
      break;
    case ConfigError::NonFinite:
      return "rate: a gain, tau_ref, the period or an effectiveness entry is not finite";
    case ConfigError::NegativeGain:
      return "rate: a gain kp, ki or kd is negative";
    case ConfigError::TauRef:
      return "rate: tau_ref is not positive";
    case ConfigError::Period:
      return "rate: the design period is not positive";
    case ConfigError::DFilter:
      return "rate: a D filter time constant is negative";
    case ConfigError::Feedforward:
      return "rate: an inertia, the motor time constant or the feed-forward filter time constant is negative";
  }
  return "rate: unknown configuration error";
}

}  // namespace

namespace detail {

void panic_period() noexcept {
  hal_panic("rate: execution spacing differs from the design period by more than 1 us");
}

}  // namespace detail

RateConfig<float> from_params() noexcept {
  RateConfig<float> c;
  c.kp = load_axes<ParamId::rate_kp_roll, ParamId::rate_kp_pitch, ParamId::rate_kp_yaw>();
  c.ki = load_axes<ParamId::rate_ki_roll, ParamId::rate_ki_pitch, ParamId::rate_ki_yaw>();
  c.kd = load_axes<ParamId::rate_kd_roll, ParamId::rate_kd_pitch, ParamId::rate_kd_yaw>();
  c.tau_ref = load_axes<ParamId::rate_tau_ref_roll, ParamId::rate_tau_ref_pitch, ParamId::rate_tau_ref_yaw>();
  // T in microseconds is an exact float quotient (an integer product over an integer), so the one division below is
  // the only rounding of the period.
  const float period_us = static_cast<float>(param_value<ParamId::rate_loop_divisor>()) *
                          static_cast<float>(param_value<ParamId::tick_period_num_us>()) /
                          static_cast<float>(param_value<ParamId::tick_period_den>());
  c.period = period_us / static_cast<float>(prim::kMicrosecondsPerSecond);
  load_geometry<ParamId::rotor_position_m1_x, ParamId::rotor_position_m1_y, ParamId::rotor_yaw_sign_m1,
                ParamId::rotor_position_m2_x, ParamId::rotor_position_m2_y, ParamId::rotor_yaw_sign_m2,
                ParamId::rotor_position_m3_x, ParamId::rotor_position_m3_y, ParamId::rotor_yaw_sign_m3,
                ParamId::rotor_position_m4_x, ParamId::rotor_position_m4_y, ParamId::rotor_yaw_sign_m4>(c);
  c.torque_ratio = param_value<ParamId::rotor_torque_ratio>();
  return c;
}

void require_valid(const RateConfig<float>& c) noexcept {
  const ConfigError e = validate(c);
  if (e != ConfigError::None) {
    hal_panic(message(e));
  }
}

RateConfig<float> load_config() noexcept {
  const RateConfig<float> c = from_params();
  require_valid(c);
  return c;
}

}  // namespace marv::rate
