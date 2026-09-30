#include <marv/hal/hal.hpp>
#include <marv/params/param.hpp>
#include <marv/prim/constants.hpp>

#include "marv/attitude/angle_mode.hpp"
#include "marv/attitude/attitude_law.hpp"
#include "marv/attitude/config.hpp"

namespace marv::attitude {

template class AttitudeLaw<float>;
template class AngleMode<float>;
template ConfigError validate<float>(const AttitudeConfig<float>&) noexcept;

namespace {

const char* message(ConfigError e) noexcept {
  switch (e) {
    case ConfigError::None:
      break;
    case ConfigError::NonFinite:
      return "attitude: a configuration value is not finite";
    case ConfigError::Gain:
      return "attitude: the gain att_kp is negative";
    case ConfigError::YawWeight:
      return "attitude: the yaw weight is not in (0, 1]";
    case ConfigError::Period:
      return "attitude: the design period is not positive";
    case ConfigError::TiltMax:
      return "attitude: the angle-mode tilt limit is not in (0, pi]";
    case ConfigError::Deadband:
      return "attitude: the yaw deadband is not in [0, 1)";
    case ConfigError::AlphaMin:
      return "attitude: att_yaw_alpha_min is not positive";
    case ConfigError::TCross:
      return "attitude: att_yaw_t_cross is not positive";
    case ConfigError::RateMax:
      return "attitude: a rate limit is not positive";
  }
  return "attitude: unknown configuration error";
}

}  // namespace

namespace detail {

void panic_period() noexcept {
  hal_panic("attitude: execution spacing differs from the design period by more than 1 us");
}

}  // namespace detail

AttitudeConfig<float> from_params() noexcept {
  AttitudeConfig<float> c;
  c.kp = param_value<ParamId::att_kp>();
  c.yaw_weight = param_value<ParamId::att_yaw_weight>();
  c.tilt_max = param_value<ParamId::angle_tilt_max>();
  c.yaw_deadband = param_value<ParamId::yaw_deadband>();
  c.yaw_alpha_min = param_value<ParamId::att_yaw_alpha_min>();
  c.yaw_t_cross = param_value<ParamId::att_yaw_t_cross>();
  c.rate_max = prim::Vec3<float>(param_value<ParamId::rate_max_roll>(), param_value<ParamId::rate_max_pitch>(),
                                 param_value<ParamId::rate_max_yaw>());
  // The attitude period in microseconds is an integer product over an integer, so one float quotient (the rate loop's
  // rule), and the conversion to seconds is one division.
  const float ticks = static_cast<float>(param_value<ParamId::att_loop_ratio>()) *
                      static_cast<float>(param_value<ParamId::rate_loop_divisor>());
  const float period_us = ticks * static_cast<float>(param_value<ParamId::tick_period_num_us>()) /
                          static_cast<float>(param_value<ParamId::tick_period_den>());
  c.period = period_us / static_cast<float>(prim::kMicrosecondsPerSecond);
  return c;
}

void require_valid(const AttitudeConfig<float>& c) noexcept {
  const ConfigError e = validate(c);
  if (e != ConfigError::None) {
    hal_panic(message(e));
  }
}

AttitudeConfig<float> load_config() noexcept {
  const AttitudeConfig<float> c = from_params();
  require_valid(c);
  return c;
}

}  // namespace marv::attitude
