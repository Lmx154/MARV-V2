// The quad flight composition, M1 subset (quad spec 4.0 M1): the gyro chain, the rate loop and the mixer through the
// shared rate-group step (marv::rate_group) with a zero rate setpoint and zero collective thrust, then the motor output
// gate. There is no script, no chirp and no attitude or vertical loop here.
//
// Every tick: step.filter(sample, due). On the ticks the rate group is due (every rate_loop_divisor ticks, tick 0
// included): step.execute(zero setpoint, zero added torque, zero thrust), so the rate loop and the mixer run on every
// rate tick; the motor output gate then decides the motor values written. On the other ticks nothing is written: the
// HAL latch holds the last command.
//
// The motor output gate is the only place that decides the values passed to hal_actuators_write. In M1 it has one
// state, Disarmed: it writes DshotValue::stop() to every motor and never the mixer's output (the mixer's floor is the
// idle command, never stop). The gate has no input but its state, so the mixer's output cannot reach the write.
// init panics naming the violated rule on an invalid rate, mixer or chain configuration or an invalid rate divisor.
#include <array>
#include <cstddef>
#include <cstdint>

#include <marv/composition.hpp>
#include <marv/hal/hal.hpp>
#include <marv/hal/tick.hpp>
#include <marv/mixer/mixer.hpp>
#include <marv/params/param.hpp>
#include <marv/prim/vec.hpp>
#include <marv/rate/rate_loop.hpp>
#include <marv/rate_group/rate_group.hpp>
#include <marv/sched/rate_groups.hpp>
#include <marv/types/actuator.hpp>
#include <marv/types/imu_sample.hpp>

namespace marv::composition {
namespace {

enum Group : std::size_t { kRate, kGroupCount };

enum class MotorGate : std::uint8_t { Disarmed };

sched::RateGroups<kGroupCount> g_groups;
rate_group::RateGroupStep g_step;
MotorGate g_gate = MotorGate::Disarmed;
Tick g_n = 0;

[[nodiscard]] bool fired(std::uint32_t due, Group g) noexcept { return (due & (std::uint32_t{1} << g)) != 0; }

// The values written to the motors in gate state `gate`.
[[nodiscard]] ActuatorOutput<kMotors, kServos> gate_output(MotorGate gate) noexcept {
  ActuatorOutput<kMotors, kServos> o{};
  switch (gate) {
    case MotorGate::Disarmed:
      break;
  }
  o.motor.fill(DshotValue::stop());
  return o;
}

}  // namespace

void init() noexcept {
  const rate::RateConfig<float> rate_cfg = rate::load_config();
  const mixer::MixerConfig<float> mixer_cfg = mixer::load_config();

  const std::int32_t divisor = param_value<ParamId::rate_loop_divisor>();
  if (divisor < 1 || !g_groups.init({static_cast<std::uint32_t>(divisor)})) {
    hal_panic("quad_fc composition: rate_loop_divisor is below 1");
  }
  g_step.init(rate_cfg, mixer_cfg, rate_group::load_chain_config());
  g_gate = MotorGate::Disarmed;
  g_n = 0;
}

void tick(const ImuSample& s) noexcept {
  const std::uint32_t due = g_groups.due(g_n);
  ++g_n;
  const bool rate_due = fired(due, kRate);
  g_step.filter(s, rate_due);
  if (!rate_due) {
    return;
  }
  static_cast<void>(g_step.execute(prim::Vec3<float>(), prim::Vec3<float>(), 0.0F));
  hal_actuators_write(gate_output(g_gate));
}

}  // namespace marv::composition
