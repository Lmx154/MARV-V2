#include "marv/arming/arming.hpp"

#include <marv/hal/hal.hpp>
#include <marv/params/param.hpp>

namespace marv::arming {

template class Arming<float>;

namespace {

// Reads the six parameters of `t` into `c`; nullptr if the configuration is valid, else the rule it violates.
const char* read_config(ParamTable t, ArmingConfig<float>& c) noexcept {
  const std::int32_t loss = param_value<ParamId::failsafe_loss_us>(t);
  const std::int32_t hold = param_value<ParamId::failsafe_hold_us>(t);
  const std::int32_t disarm = param_value<ParamId::failsafe_disarm_us>(t);
  const std::int32_t recovery = param_value<ParamId::failsafe_recovery_us>(t);
  if (loss < 1 || hold < 1 || disarm < 1 || recovery < 1) {
    return "arming: a failsafe time is below 1 us";
  }
  c.loss_us = static_cast<std::uint64_t>(loss);
  c.hold_us = static_cast<std::uint64_t>(hold);
  c.disarm_us = static_cast<std::uint64_t>(disarm);
  c.recovery_us = static_cast<std::uint64_t>(recovery);
  c.arm_threshold = param_value<ParamId::arm_switch_threshold>(t);
  c.throttle_min = param_value<ParamId::arm_throttle_min>(t);
  if (!valid(c)) {
    return "arming: the failsafe times are not ordered loss < hold < disarm, or a threshold is not finite";
  }
  return nullptr;
}

}  // namespace

const char* reject_reason(ParamTable table) noexcept {
  ArmingConfig<float> c;
  return read_config(table, c);
}

ArmingConfig<float> load_config(ParamTable table) noexcept {
  ArmingConfig<float> c;
  const char* const why = read_config(table, c);
  if (why != nullptr) {
    hal_panic(why);
  }
  return c;
}

ArmingConfig<float> load_config() noexcept { return load_config(param_active_table()); }

}  // namespace marv::arming
