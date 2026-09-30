#include <marv/hal/hal.hpp>

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

void require_valid(const AttitudeConfig<float>& c) noexcept {
  const ConfigError e = validate(c);
  if (e != ConfigError::None) {
    hal_panic(message(e));
  }
}

}  // namespace marv::attitude
