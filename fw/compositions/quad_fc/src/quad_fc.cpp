// The quad flight composition, M1 subset (quad spec 4.0 M1): the gyro chain, the rate loop and the mixer through the
// shared rate-group step (marv::rate_group) with a zero rate setpoint and zero collective thrust, then the motor output
// gate. There is no script, no chirp and no attitude or vertical loop here.
//
// Every tick: step.filter(sample, due). On the ticks the rate group is due (every rate_loop_divisor ticks, tick 0
// included): step.execute(zero setpoint, zero added torque, zero thrust), so the rate loop and the mixer run on every
// rate tick; the motor output gate then decides the motor values written. On the other ticks nothing is written: the
// HAL latch holds the last command.
//
// The motor output gate is the only place that decides the values passed to hal_actuators_write. In M1 it has two
// states. Disarmed: it writes DshotValue::stop() to every motor and never the mixer's output (the mixer's floor is the
// idle command, never stop). MotorTest (the console motor test, decision 0038): exactly one logical motor gets the
// requested value and the other three stop. The gate's inputs are its state, the motor-test request and the stamps, so
// the mixer's output cannot reach the write.
//
// The motor-test request comes from the HAL (hal_motor_test), is read only on a rate tick and is validated here, never
// by the console: motor 1..4, value in [kDshotThrottleMin, motor_test_dshot_max], duration in [1 ms,
// motor_test_duration_max_ms]; otherwise it is refused (the gate stays as it was, a refusal is counted and reported). A
// test ends at its deadline, counted on rate ticks, or on a cancel signal, at the next rate tick; the gate then writes
// stop. The deadline is the first rate tick whose stamp is at least the request's stamp plus its duration, and also the
// first whose count of rate ticks since the request reaches the duration over the nominal rate period (rounded up), so a
// stamp that does not advance cannot extend the test; a stamp before the request's ends it. The gate writes its values
// to the write directly: the mixer's DShot diffuser carry (mixer.hpp) is not reachable from it and is not touched.
// init panics naming the violated rule on an invalid rate, mixer or chain configuration or an invalid rate divisor.
#include <array>
#include <cstddef>
#include <cstdint>

#include <marv/composition.hpp>
#include <marv/hal/hal.hpp>
#include <marv/hal/tick.hpp>
#include <marv/mixer/mixer.hpp>
#include <marv/params/param.hpp>
#include <marv/prim/constants.hpp>
#include <marv/prim/vec.hpp>
#include <marv/rate/rate_loop.hpp>
#include <marv/rate_group/rate_group.hpp>
#include <marv/sched/rate_groups.hpp>
#include <marv/types/actuator.hpp>
#include <marv/types/imu_sample.hpp>
#include <marv/types/time.hpp>

namespace marv::composition {
namespace {

enum Group : std::size_t { kRate, kGroupCount };

enum class MotorGate : std::uint8_t { Disarmed, MotorTest };

// The motor test in progress (MotorGate::MotorTest only).
struct TestRun {
  std::size_t motor = 0;  // index (logical number - 1)
  DshotValue value;
  MotorTestRequest request{};
  TimeUs start_us = 0;
  TimeUs deadline_us = 0;
  std::uint64_t ticks = 0;      // rate ticks since the request, the request's own tick being 0
  std::uint64_t max_ticks = 0;  // the test ends on the rate tick with this count
};

sched::RateGroups<kGroupCount> g_groups;
rate_group::RateGroupStep g_step;
MotorGate g_gate = MotorGate::Disarmed;
TestRun g_run;
MotorTestEvent g_counts;  // the counters of the events reported so far
std::uint32_t g_seen_cancel = 0;
std::uint32_t g_seen_request = 0;
std::int64_t g_cap = 0;
std::int64_t g_max_ms = 0;
std::int64_t g_rate_num = 0;  // the nominal rate period is g_rate_num / g_rate_den microseconds
std::int64_t g_rate_den = 0;
Tick g_n = 0;

[[nodiscard]] bool fired(std::uint32_t due, Group g) noexcept { return (due & (std::uint32_t{1} << g)) != 0; }

// The values written to the motors in gate state `gate`: stop on every motor, except in MotorTest the one motor of
// `run`.
[[nodiscard]] ActuatorOutput<kMotors, kServos> gate_output(MotorGate gate, const TestRun& run) noexcept {
  ActuatorOutput<kMotors, kServos> o{};
  o.motor.fill(DshotValue::stop());
  switch (gate) {
    case MotorGate::Disarmed:
      break;
    case MotorGate::MotorTest:
      o.motor[run.motor] = run.value;
      break;
  }
  return o;
}

void report(MotorTestEventKind kind, MotorTestReason reason, const MotorTestRequest& request) noexcept {
  switch (kind) {
    case MotorTestEventKind::Accepted:
      ++g_counts.accepted;
      break;
    case MotorTestEventKind::Refused:
      ++g_counts.refused;
      break;
    case MotorTestEventKind::Stopped:
      ++g_counts.stopped;
      break;
  }
  MotorTestEvent e = g_counts;
  e.kind = kind;
  e.reason = reason;
  e.request = request;
  hal_motor_test_report(e);
}

void end_test(MotorTestReason reason) noexcept {
  g_gate = MotorGate::Disarmed;
  report(MotorTestEventKind::Stopped, reason, g_run.request);
}

// The first refusing rule of a request, or None.
[[nodiscard]] MotorTestReason check(const MotorTestRequest& r) noexcept {
  if (r.motor < 1 || r.motor > kMotors) {
    return MotorTestReason::Motor;
  }
  if (r.dshot < prim::kDshotThrottleMin || r.dshot > g_cap || !DshotValue::from_raw(static_cast<std::uint16_t>(r.dshot))) {
    return MotorTestReason::Value;
  }
  if (r.duration_ms < 1 || r.duration_ms > g_max_ms) {
    return MotorTestReason::Duration;
  }
  return MotorTestReason::None;
}

void begin_test(const MotorTestRequest& r, TimeUs now) noexcept {
  const MotorTestReason why = check(r);
  if (why != MotorTestReason::None) {
    report(MotorTestEventKind::Refused, why, r);
    return;
  }
  const std::uint64_t duration_us = static_cast<std::uint64_t>(r.duration_ms) * prim::kMicrosecondsPerMillisecond;
  const auto num = static_cast<std::uint64_t>(g_rate_num);
  const auto den = static_cast<std::uint64_t>(g_rate_den);
  g_run.motor = r.motor - 1;
  g_run.value = DshotValue::from_raw(static_cast<std::uint16_t>(r.dshot)).value_or(DshotValue::stop());
  g_run.request = r;
  g_run.start_us = now;
  g_run.deadline_us = now + duration_us;
  g_run.ticks = 0;
  g_run.max_ticks = (duration_us * den + num - 1) / num;
  g_gate = MotorGate::MotorTest;
  report(MotorTestEventKind::Accepted, MotorTestReason::None, r);
}

// On a rate tick, before the write: applies the cancel signal, then the request, then the deadline.
void update_motor_test(TimeUs now) noexcept {
  const MotorTestPoll p = hal_motor_test();
  if (p.cancel_count != g_seen_cancel) {
    g_seen_cancel = p.cancel_count;
    if (g_gate == MotorGate::MotorTest) {
      end_test(p.cancel_reason == MotorTestCancel::Disconnect ? MotorTestReason::Disconnect : MotorTestReason::Cancel);
    }
  }
  if (p.request_count != g_seen_request) {
    g_seen_request = p.request_count;
    if (g_gate == MotorGate::MotorTest) {
      end_test(MotorTestReason::Cancel);
    }
    if (p.request_cancel_count != p.cancel_count) {
      report(MotorTestEventKind::Refused, MotorTestReason::Cancel, p.request);
    } else {
      begin_test(p.request, now);
    }
    return;
  }
  if (g_gate == MotorGate::MotorTest) {
    ++g_run.ticks;
    if (now < g_run.start_us || now >= g_run.deadline_us || g_run.ticks >= g_run.max_ticks) {
      end_test(MotorTestReason::Deadline);
    }
  }
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

  g_cap = param_value<ParamId::motor_test_dshot_max>();
  g_max_ms = param_value<ParamId::motor_test_duration_max_ms>();
  if (g_cap < prim::kDshotThrottleMin || g_cap > prim::kDshotThrottleMax || g_max_ms < 1) {
    hal_panic("quad_fc composition: motor_test_dshot_max is outside the DShot throttle range or the duration is below 1 ms");
  }
  const std::int32_t num = param_value<ParamId::tick_period_num_us>();
  const std::int32_t den = param_value<ParamId::tick_period_den>();
  if (num < 1 || den < 1) {
    hal_panic("quad_fc composition: the tick period is below 1");
  }
  g_rate_num = static_cast<std::int64_t>(divisor) * num;
  g_rate_den = den;
  const MotorTestPoll seen = hal_motor_test();
  g_seen_cancel = seen.cancel_count;
  g_seen_request = seen.request_count;
  g_counts = MotorTestEvent{};
  g_run = TestRun{};
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
  update_motor_test(s.t_us);
  hal_actuators_write(gate_output(g_gate, g_run));
}

}  // namespace marv::composition
