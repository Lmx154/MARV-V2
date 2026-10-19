#include "marv/rate/rate_loop.hpp"

#include <marv/hal/hal.hpp>
#include <marv/params/param.hpp>
#include <marv/prim/constants.hpp>

namespace marv::rate {

template class RateLoop<float>;
template ConfigError validate<float>(const RateConfig<float>&) noexcept;

namespace {

template <ParamId Roll, ParamId Pitch, ParamId Yaw>
prim::Vec3<float> load_axes(const ParamTable& t) noexcept {
  return prim::Vec3<float>(param_value<Roll>(t), param_value<Pitch>(t), param_value<Yaw>(t));
}

template <ParamId X1, ParamId Y1, ParamId S1, ParamId X2, ParamId Y2, ParamId S2, ParamId X3, ParamId Y3, ParamId S3,
          ParamId X4, ParamId Y4, ParamId S4>
void load_geometry(RateConfig<float>& c, const ParamTable& t) noexcept {
  c.rotor_x = {param_value<X1>(t), param_value<X2>(t), param_value<X3>(t), param_value<X4>(t)};
  c.rotor_y = {param_value<Y1>(t), param_value<Y2>(t), param_value<Y3>(t), param_value<Y4>(t)};
  c.yaw_sign = {static_cast<float>(param_value<S1>(t)), static_cast<float>(param_value<S2>(t)),
                static_cast<float>(param_value<S3>(t)), static_cast<float>(param_value<S4>(t))};
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
      return "rate: an inertia, a reference feed-forward inertia, the motor time constant or the feed-forward "
             "filter time constant is negative";
    case ConfigError::Timing:
      return "rate: the tick period or divisor is below 1, or rate_period_tolerance_us is not in [0, half a tick)";
  }
  return "rate: unknown configuration error";
}

}  // namespace

RateConfig<float> from_params(ParamTable t) noexcept {
  RateConfig<float> c;
  c.kp = load_axes<ParamId::rate_kp_roll, ParamId::rate_kp_pitch, ParamId::rate_kp_yaw>(t);
  c.ki = load_axes<ParamId::rate_ki_roll, ParamId::rate_ki_pitch, ParamId::rate_ki_yaw>(t);
  c.kd = load_axes<ParamId::rate_kd_roll, ParamId::rate_kd_pitch, ParamId::rate_kd_yaw>(t);
  c.tau_ref = load_axes<ParamId::rate_tau_ref_roll, ParamId::rate_tau_ref_pitch, ParamId::rate_tau_ref_yaw>(t);
  c.d_filter_tau =
      load_axes<ParamId::rate_d_filter_tau_roll, ParamId::rate_d_filter_tau_pitch, ParamId::rate_d_filter_tau_yaw>(t);
  // T in microseconds is an exact float quotient (an integer product over an integer), so the one division below is
  // the only rounding of the period.
  const float period_us = static_cast<float>(param_value<ParamId::rate_loop_divisor>(t)) *
                          static_cast<float>(param_value<ParamId::tick_period_num_us>(t)) /
                          static_cast<float>(param_value<ParamId::tick_period_den>(t));
  c.period = period_us / static_cast<float>(prim::kMicrosecondsPerSecond);
  c.tick_period_num_us = param_value<ParamId::tick_period_num_us>(t);
  c.tick_period_den = param_value<ParamId::tick_period_den>(t);
  c.rate_loop_divisor = param_value<ParamId::rate_loop_divisor>(t);
  c.period_tolerance_us = param_value<ParamId::rate_period_tolerance_us>(t);
  load_geometry<ParamId::rotor_position_m1_x, ParamId::rotor_position_m1_y, ParamId::rotor_yaw_sign_m1,
                ParamId::rotor_position_m2_x, ParamId::rotor_position_m2_y, ParamId::rotor_yaw_sign_m2,
                ParamId::rotor_position_m3_x, ParamId::rotor_position_m3_y, ParamId::rotor_yaw_sign_m3,
                ParamId::rotor_position_m4_x, ParamId::rotor_position_m4_y, ParamId::rotor_yaw_sign_m4>(c, t);
  c.torque_ratio = param_value<ParamId::rotor_torque_ratio>(t);
  // The feed-forward only while rate_ff_enable is 1; otherwise J, tau_m and T_ff keep RateConfig's 0, the exact PID law
  // (decision 0014, owner decision 2: inert until stage (e)).
  if (param_value<ParamId::rate_ff_enable>(t) == 1) {
    c.inertia = load_axes<ParamId::inertia_xx, ParamId::inertia_yy, ParamId::inertia_zz>(t);
    c.motor_tau = param_value<ParamId::motor_tau>(t);
    c.ff_filter_tau = param_value<ParamId::rate_ff_filter_tau>(t);
  }
  // The reference feed-forward only while rate_ref_ff_enable is 1 (decision 0056); otherwise J_ref keeps
  // RateConfig's 0.
  if (param_value<ParamId::rate_ref_ff_enable>(t) == 1) {
    c.ref_ff_inertia = load_axes<ParamId::inertia_xx, ParamId::inertia_yy, ParamId::inertia_zz>(t);
  }
  return c;
}

RateConfig<float> from_params() noexcept { return from_params(param_active_table()); }

const char* reject_reason(const RateConfig<float>& c) noexcept {
  ConfigError e = validate(c);
  if (e == ConfigError::None) {
    e = validate_timing(c);
  }
  return e == ConfigError::None ? nullptr : message(e);
}

void require_valid(const RateConfig<float>& c) noexcept {
  const char* const why = reject_reason(c);
  if (why != nullptr) {
    hal_panic(why);
  }
}

RateConfig<float> load_config() noexcept {
  const RateConfig<float> c = from_params();
  require_valid(c);
  return c;
}

}  // namespace marv::rate
