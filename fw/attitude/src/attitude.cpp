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
    case ConfigError::Timing:
      return "attitude: the tick period or period ticks is below 1, or rate_period_tolerance_us is not in [0, half a tick)";
  }
  return "attitude: unknown configuration error";
}

}  // namespace

AttitudeConfig<float> from_params(ParamTable t) noexcept {
  AttitudeConfig<float> c;
  c.kp = param_value<ParamId::att_kp>(t);
  c.yaw_weight = param_value<ParamId::att_yaw_weight>(t);
  c.tilt_max = param_value<ParamId::angle_tilt_max>(t);
  c.yaw_deadband = param_value<ParamId::yaw_deadband>(t);
  c.yaw_alpha_min = param_value<ParamId::att_yaw_alpha_min>(t);
  c.yaw_t_cross = param_value<ParamId::att_yaw_t_cross>(t);
  c.rate_max = prim::Vec3<float>(param_value<ParamId::rate_max_roll>(t), param_value<ParamId::rate_max_pitch>(t),
                                 param_value<ParamId::rate_max_yaw>(t));
  // The attitude period in microseconds is an integer product over an integer, so one float quotient (the rate loop's
  // rule), and the conversion to seconds is one division.
  const float ticks = static_cast<float>(param_value<ParamId::att_loop_ratio>(t)) *
                      static_cast<float>(param_value<ParamId::rate_loop_divisor>(t));
  const float period_us = ticks * static_cast<float>(param_value<ParamId::tick_period_num_us>(t)) /
                          static_cast<float>(param_value<ParamId::tick_period_den>(t));
  c.period = period_us / static_cast<float>(prim::kMicrosecondsPerSecond);
  c.tick_period_num_us = param_value<ParamId::tick_period_num_us>(t);
  c.tick_period_den = param_value<ParamId::tick_period_den>(t);
  c.period_ticks = param_value<ParamId::att_loop_ratio>(t) * param_value<ParamId::rate_loop_divisor>(t);
  c.period_tolerance_us = param_value<ParamId::rate_period_tolerance_us>(t);
  return c;
}

AttitudeConfig<float> from_params() noexcept { return from_params(param_active_table()); }

const char* reject_reason(const AttitudeConfig<float>& c) noexcept {
  ConfigError e = validate(c);
  if (e == ConfigError::None) {
    e = validate_timing(c);
  }
  return e == ConfigError::None ? nullptr : message(e);
}

void require_valid(const AttitudeConfig<float>& c) noexcept {
  const char* const why = reject_reason(c);
  if (why != nullptr) {
    hal_panic(why);
  }
}

AttitudeConfig<float> load_config() noexcept {
  const AttitudeConfig<float> c = from_params();
  require_valid(c);
  return c;
}

}  // namespace marv::attitude
