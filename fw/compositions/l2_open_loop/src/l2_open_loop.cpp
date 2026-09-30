// The L2 open-loop composition. There is no controller: every tick writes one quad-X output whose motor m is the
// DShot value of parameter ol_dshot_m<m>. The IMU sample's content is ignored; only its stamp is checked against
// hal_time_us(). init validates each value as a legal actuator value (0 or the throttle range) and panics naming the
// motor otherwise, so an illegal DShot never reaches the output.
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include <marv/composition.hpp>
#include <marv/hal/hal.hpp>
#include <marv/params/param.hpp>
#include <marv/prim/constants.hpp>
#include <marv/types/actuator.hpp>
#include <marv/types/imu_sample.hpp>

namespace marv::composition {
namespace {

std::array<DshotValue, kMotors> g_command{};

// The parameter value as a DShot value; nullopt for anything outside {0} and the throttle range.
[[nodiscard]] std::optional<DshotValue> command(std::int32_t v) noexcept {
  if (v < 0 || v > static_cast<std::int32_t>(prim::kDshotThrottleMax)) {
    return std::nullopt;
  }
  return DshotValue::from_raw(static_cast<std::uint16_t>(v));
}

void load(Motor m, std::int32_t v, const char* panic_message) noexcept {
  const std::optional<DshotValue> c = command(v);
  if (!c) {
    hal_panic(panic_message);
  }
  g_command[motor_index(m)] = *c;
}

}  // namespace

void init() noexcept {
  g_command = {};
  load(Motor::M1FrontRight, param_value<ParamId::ol_dshot_m1>(),
       "l2_open_loop composition: ol_dshot_m1 (motor 1) is not 0 or a DShot throttle value");
  load(Motor::M2RearLeft, param_value<ParamId::ol_dshot_m2>(),
       "l2_open_loop composition: ol_dshot_m2 (motor 2) is not 0 or a DShot throttle value");
  load(Motor::M3FrontLeft, param_value<ParamId::ol_dshot_m3>(),
       "l2_open_loop composition: ol_dshot_m3 (motor 3) is not 0 or a DShot throttle value");
  load(Motor::M4RearRight, param_value<ParamId::ol_dshot_m4>(),
       "l2_open_loop composition: ol_dshot_m4 (motor 4) is not 0 or a DShot throttle value");
}

void tick(const ImuSample& s) noexcept {
  if (hal_time_us() != s.t_us) {
    hal_panic("l2_open_loop composition: hal_time_us differs from the sample stamp");
  }
  ActuatorOutput<kMotors, kServos> o{};
  o.motor = g_command;
  hal_actuators_write(o);
}

}  // namespace marv::composition
