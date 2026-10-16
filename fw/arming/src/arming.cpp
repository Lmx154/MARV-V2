#include "marv/arming/arming.hpp"

#include <marv/hal/hal.hpp>
#include <marv/params/param.hpp>

namespace marv::arming {

template class Arming<float>;

ArmingConfig<float> load_config() noexcept {
  ArmingConfig<float> c;
  const std::int32_t loss = param_value<ParamId::failsafe_loss_us>();
  const std::int32_t hold = param_value<ParamId::failsafe_hold_us>();
  const std::int32_t disarm = param_value<ParamId::failsafe_disarm_us>();
  const std::int32_t recovery = param_value<ParamId::failsafe_recovery_us>();
  if (loss < 1 || hold < 1 || disarm < 1 || recovery < 1) {
    hal_panic("arming: a failsafe time is below 1 us");
  }
  c.loss_us = static_cast<std::uint64_t>(loss);
  c.hold_us = static_cast<std::uint64_t>(hold);
  c.disarm_us = static_cast<std::uint64_t>(disarm);
  c.recovery_us = static_cast<std::uint64_t>(recovery);
  c.arm_threshold = param_value<ParamId::arm_switch_threshold>();
  c.throttle_min = param_value<ParamId::arm_throttle_min>();
  if (!valid(c)) {
    hal_panic("arming: the failsafe times are not ordered loss < hold < disarm, or a threshold is not finite");
  }
  return c;
}

}  // namespace marv::arming
