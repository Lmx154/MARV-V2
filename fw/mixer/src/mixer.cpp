#include "marv/mixer/mixer.hpp"

#include <marv/hal/hal.hpp>
#include <marv/params/param.hpp>

namespace marv::mixer {

template Allocation<float> allocate<float>(const MixerConfig<float>&, const Request<float>&) noexcept;
template std::array<DshotValue, kMotors> thrust_to_dshot<float>(const MixerConfig<float>&,
                                                                const std::array<float, kMotors>&) noexcept;
template MixerOutput<float> mix<float>(const MixerConfig<float>&, const Request<float>&) noexcept;
template ConfigError validate<float>(const MixerConfig<float>&) noexcept;

namespace {

template <ParamId Thrust, ParamId Roll, ParamId Pitch, ParamId Yaw>
void load_row(MixerConfig<float>& c, std::size_t row) noexcept {
  c.m(row, kThrust) = param_value<Thrust>();
  c.m(row, kRoll) = param_value<Roll>();
  c.m(row, kPitch) = param_value<Pitch>();
  c.m(row, kYaw) = param_value<Yaw>();
}

const char* message(ConfigError e) noexcept {
  switch (e) {
    case ConfigError::None:
      break;
    case ConfigError::NonFinite:
      return "mixer: a mixer entry, rotor_thrust_coeff, idle_speed or a rotor speed limit is not finite";
    case ConfigError::ThrustCoeff:
      return "mixer: rotor_thrust_coeff is not positive";
    case ConfigError::IdleNegative:
      return "mixer: idle_speed is negative";
    case ConfigError::IdleBelowMin:
      return "mixer: idle_speed is below rotor_speed_min";
    case ConfigError::MinNotBelowMax:
      return "mixer: rotor_speed_min is not below rotor_speed_max";
    case ConfigError::IdleNotBelowMax:
      return "mixer: idle_speed is not below rotor_speed_max";
    case ConfigError::ThrustColumn:
      return "mixer: a thrust-column entry mixer_m<i>_thrust is not positive";
    case ConfigError::ZeroTorqueInfeasible:
      return "mixer: zero torque is not achievable between idle and maximum rotor speed";
  }
  return "mixer: unknown configuration error";
}

}  // namespace

MixerConfig<float> from_params() noexcept {
  MixerConfig<float> c;
  load_row<ParamId::mixer_m1_thrust, ParamId::mixer_m1_roll, ParamId::mixer_m1_pitch, ParamId::mixer_m1_yaw>(
      c, motor_index(Motor::M1FrontRight));
  load_row<ParamId::mixer_m2_thrust, ParamId::mixer_m2_roll, ParamId::mixer_m2_pitch, ParamId::mixer_m2_yaw>(
      c, motor_index(Motor::M2RearLeft));
  load_row<ParamId::mixer_m3_thrust, ParamId::mixer_m3_roll, ParamId::mixer_m3_pitch, ParamId::mixer_m3_yaw>(
      c, motor_index(Motor::M3FrontLeft));
  load_row<ParamId::mixer_m4_thrust, ParamId::mixer_m4_roll, ParamId::mixer_m4_pitch, ParamId::mixer_m4_yaw>(
      c, motor_index(Motor::M4RearRight));
  c.thrust_coeff = param_value<ParamId::rotor_thrust_coeff>();
  c.omega_idle = param_value<ParamId::idle_speed>();
  c.omega_min = param_value<ParamId::rotor_speed_min>();
  c.omega_max = param_value<ParamId::rotor_speed_max>();
  return c;
}

void require_valid(const MixerConfig<float>& c) noexcept {
  const ConfigError e = validate(c);
  if (e != ConfigError::None) {
    hal_panic(message(e));
  }
}

MixerConfig<float> load_config() noexcept {
  const MixerConfig<float> c = from_params();
  require_valid(c);
  return c;
}

}  // namespace marv::mixer
