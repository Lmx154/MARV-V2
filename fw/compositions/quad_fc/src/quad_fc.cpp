// The quad flight composition, M2 subset (quad spec 4.0 M2): the gyro chain, the rate loop and the mixer through the
// shared rate-group step (marv::rate_group), the arming and failsafe machine (marv::arming), the stick rates curve
// (marv::stick_rates), then the motor output gate. There is no vertical loop here. The pilot flies the rate loop (acro) or,
// with the angle switch selecting it, the attitude law (angle mode, M4 A2, decision 0043 item 3), below.
//
// Every tick: the boot gyro calibration (marv::estimator::GyroCal, M3 P6) takes the sample's gyro until it completes; once
// complete, its bias is subtracted from the gyro of every sample, before both the estimator and the rate loop's gyro chain,
// and until then nothing is subtracted. The attitude estimator (marv::estimator, the Mahony filter, M3 P2) steps on the sample
// with dt from the stamps (decision 0042), skipping a sample whose GyroValid bit is clear (the next step's dt spans the gap from
// the last stamp it stepped on) and taking no correction from one whose AccelValid bit is clear; step.filter(sample, due); the sample's
// validity and saturation bits are accumulated since the previous rate tick. On the ticks the rate group is due (every rate_loop_divisor ticks, tick 0 included): the pilot's input
// (hal_manual_control), the clock (hal_time_us), the calibration's completion, the estimator's validity and its tilt against arm_small_angle (upright:
// the cosine of the tilt, 1 - 2 (x^2 + y^2) of the estimate's quaternion, above the cosine of the angle, as Betaflight's isUpright)
// go to the arming machine (decisions 0040 and 0042), which decides armed and the
// effective sticks; the rate group executes (armed: the rate setpoint of the sticks through the QF-1 curve and the
// collective thrust of the throttle, below; disarmed: zero setpoint and zero thrust, so the filters keep running); the
// motor test is updated; the gate then decides the motor values written. On the other ticks nothing is written: the HAL
// latch holds the last command.
//
// Angle mode. Selected on a rate tick iff rc_channel_angle is not 0 and ManualControl.channels[rc_channel_angle - 1] >=
// angle_switch_threshold (a value that is not a number is acro). While armed and selected, the sticks the arming machine
// decides (the same limited sticks acro uses, so centred in ArmedFailsafeCentred) go through marv::attitude::AngleMode (roll,
// pitch, yaw sticks to a setpoint quaternion and a yaw-rate command), marv::attitude::AttitudeLaw (to a rate setpoint) and
// RateGroupStep::execute_bypass, with the same collective thrust as acro, from the estimator's quaternion, the calibrated gyro
// and the estimator's validity. On the first rate tick of armed-and-selected both are reset (AngleMode::reset, which drops the
// heading lock; AttitudeLaw::reset, so the first execution carries no spacing check); the law runs only there. The rate loop's
// controller state is not reset on a change of mode. No failsafe phase forces angle mode: the mode channel is the newest valid
// frame's. The law runs on every rate tick, which is its period only when att_loop_ratio is 1 (the product set's value); init panics otherwise. The
// law's faults and timing faults are counted (attitude_counts).
//
// Collective thrust. T = T_min + throttle * (T_max - T_min), linear in collective thrust between the mixer's own limits
// (lead decision, M2 P4), with T_min = sum over the motors of k omega_idle^2 and T_max = sum of k omega_max^2, k the
// mixer's rotor thrust coefficient (mixer::f_min and f_max, per motor): derived from the mixer configuration, no new
// tunable. The throttle is clamped to [0, 1] and the sticks to [-1, 1]; a value that is not finite is 0.
//
// The motor output gate is the only place that decides the values passed to hal_actuators_write. It has three states.
// Disarmed: it writes DshotValue::stop() to every motor and never the mixer's output (the mixer's floor is the idle
// command, never stop). Armed (exactly when the arming machine's output is armed): the mixer's output of this tick.
// MotorTest (the console motor test, decision 0038): exactly one logical motor gets the requested value and the other
// three stop. Armed and MotorTest are never both: a test starts only from Disarmed (a request while armed is refused,
// MotorTestReason::Armed), and the machine's motor_test_active input is true while a test runs, so it refuses to arm.
// Every disarm writes stop on the rate tick it is decided (decision 0040, I1 and I2). On the disarmed-to-armed transition
// the rate loop's controller state restarts (RateGroupStep::reset_controller; the gyro chain keeps running), so the
// integrator wound up on a gyro bias while disarmed is not applied.
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
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include <marv/arming/arming.hpp>
#include <marv/attitude/angle_mode.hpp>
#include <marv/attitude/attitude_law.hpp>
#include <marv/attitude/config.hpp>
#include <marv/composition.hpp>
#include <marv/estimator/gyro_cal.hpp>
#include <marv/estimator/mahony.hpp>
#include <marv/hal/hal.hpp>
#include <marv/hal/tick.hpp>
#include <marv/mixer/mixer.hpp>
#include <marv/params/param.hpp>
#include <marv/prim/constants.hpp>
#include <marv/prim/quat.hpp>
#include <marv/prim/vec.hpp>
#include <marv/rate/rate_loop.hpp>
#include <marv/rate_group/rate_group.hpp>
#include <marv/sched/rate_groups.hpp>
#include <marv/stick_rates/stick_rates.hpp>
#include <marv/types/actuator.hpp>
#include <marv/types/attitude_state.hpp>
#include <marv/types/imu_sample.hpp>
#include <marv/types/time.hpp>

namespace marv::composition {
namespace {

enum Group : std::size_t { kRate, kGroupCount };

enum class MotorGate : std::uint8_t { Disarmed, Armed, MotorTest };

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
arming::Arming<float> g_arming;
stick_rates::RatesConfig<float> g_rates;
float g_thrust_min = 0.0F;  // T_min, N
float g_thrust_max = 0.0F;  // T_max, N
std::size_t g_arm_channel = 0;  // index into ManualControl::channels (rc_channel_arm - 1)
bool g_angle_switch = false;    // rc_channel_angle is not 0
std::size_t g_angle_channel = 0;  // index into ManualControl::channels (rc_channel_angle - 1), when g_angle_switch
float g_angle_threshold = 0.0F;   // angle_switch_threshold
bool g_angle_running = false;     // the latest rate tick was armed and angle
attitude::AngleMode<float> g_angle;
attitude::AttitudeLaw<float> g_attitude;
std::atomic<std::uint32_t> g_attitude_faults{0};         // the law's fault_count, published after each execution
std::atomic<std::uint32_t> g_attitude_timing_faults{0};  // and its timing_fault_count
bool g_imu_invalid = false;     // a sample since the previous rate tick lacked GyroValid or AccelValid
bool g_imu_saturated = false;   // a sample since the previous rate tick had a saturation bit
std::atomic<std::uint8_t> g_arming_state{static_cast<std::uint8_t>(arming::ArmState::Boot)};
estimator::Mahony<float> g_estimator;
estimator::GyroCal<float> g_gyro_cal;
float g_upright_cos = 0.0F;  // cos(arm_small_angle): upright iff the cosine of the estimate's tilt is above it
TimeUs g_estimator_prev_us = 0;  // the stamp of the previous sample the estimator stepped on
bool g_estimator_started = false;
// The accelerometer a sample whose AccelValid bit is clear gives the estimator: no direction, so no correction that step.
const prim::Vec3<float> kNoAccel{std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
                                 std::numeric_limits<float>::quiet_NaN()};
std::atomic<float> g_tilt_xy2{0.0F};  // x^2 + y^2 of the estimate's quaternion, published on rate ticks
std::atomic<float> g_tilt_wz2{1.0F};  // w^2 + z^2
std::atomic<bool> g_estimator_valid{false};

// The saturation bits of ImuSample::flags (gyro and accel, every axis), GyroSatX .. AccelSatZ.
[[nodiscard]] constexpr std::uint32_t saturation_mask() noexcept {
  std::uint32_t m = 0;
  for (auto f = static_cast<unsigned>(ImuFlag::GyroSatX); f <= static_cast<unsigned>(ImuFlag::AccelSatZ); ++f) {
    m |= imu_flag(static_cast<ImuFlag>(f));
  }
  return m;
}

[[nodiscard]] bool fired(std::uint32_t due, Group g) noexcept { return (due & (std::uint32_t{1} << g)) != 0; }

// The values written to the motors in gate state `gate`: stop on every motor, except Armed the mixer's output `mixed`
// and in MotorTest the one motor of `run`.
[[nodiscard]] ActuatorOutput<kMotors, kServos> gate_output(MotorGate gate, const TestRun& run,
                                                           const std::array<DshotValue, kMotors>& mixed) noexcept {
  ActuatorOutput<kMotors, kServos> o{};
  o.motor.fill(DshotValue::stop());
  switch (gate) {
    case MotorGate::Disarmed:
      break;
    case MotorGate::Armed:
      o.motor = mixed;
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
  const MotorTestReason why = g_gate == MotorGate::Armed ? MotorTestReason::Armed : check(r);
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

// A stick or the throttle limited to [lo, 1]; a value that is not finite is 0 (as stick_rates::rate_setpoint reads one).
[[nodiscard]] float limited(float x, float lo) noexcept {
  if (!std::isfinite(x)) {
    return 0.0F;
  }
  return x < lo ? lo : (x > 1.0F ? 1.0F : x);
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

  g_rates.roll = {param_value<ParamId::rates_centre_roll>(), param_value<ParamId::rate_max_roll>(),
                  param_value<ParamId::rates_expo_roll>()};
  g_rates.pitch = {param_value<ParamId::rates_centre_pitch>(), param_value<ParamId::rate_max_pitch>(),
                   param_value<ParamId::rates_expo_pitch>()};
  g_rates.yaw = {param_value<ParamId::rates_centre_yaw>(), param_value<ParamId::rate_max_yaw>(),
                 param_value<ParamId::rates_expo_yaw>()};
  if (stick_rates::validate(g_rates) != stick_rates::ConfigError::None) {
    hal_panic("quad_fc composition: the rates curve is invalid (0 < centre <= max, 0 <= expo <= 1, finite)");
  }
  const std::int32_t channel = param_value<ParamId::rc_channel_arm>();
  if (channel < 1 || static_cast<std::size_t>(channel) > prim::kCrsfRcChannels) {
    hal_panic("quad_fc composition: rc_channel_arm is outside 1..16");
  }
  g_arm_channel = static_cast<std::size_t>(channel) - 1;
  const std::int32_t angle_channel = param_value<ParamId::rc_channel_angle>();
  if (angle_channel < 0 || static_cast<std::size_t>(angle_channel) > prim::kCrsfRcChannels) {
    hal_panic("quad_fc composition: rc_channel_angle is outside 0..16");
  }
  if (angle_channel != 0 &&
      (angle_channel == channel || angle_channel == param_value<ParamId::rc_channel_roll>() ||
       angle_channel == param_value<ParamId::rc_channel_pitch>() ||
       angle_channel == param_value<ParamId::rc_channel_throttle>() ||
       angle_channel == param_value<ParamId::rc_channel_yaw>())) {
    hal_panic("quad_fc composition: rc_channel_angle is the arm channel or a stick channel");
  }
  g_angle_switch = angle_channel != 0;
  g_angle_channel = g_angle_switch ? static_cast<std::size_t>(angle_channel) - 1 : 0;
  g_angle_threshold = param_value<ParamId::angle_switch_threshold>();
  if (param_value<ParamId::att_loop_ratio>() != 1) {
    hal_panic("quad_fc composition: att_loop_ratio is not 1 (the attitude law runs once per rate tick)");
  }
  const attitude::AttitudeConfig<float> attitude_cfg = attitude::load_config();
  g_angle.init(attitude_cfg);
  g_attitude.init(attitude_cfg);
  g_angle_running = false;
  g_attitude_faults.store(0, std::memory_order_relaxed);
  g_attitude_timing_faults.store(0, std::memory_order_relaxed);
  g_arming.init(arming::load_config());
  g_estimator.init(estimator::MahonyConfig<float>{
      param_value<ParamId::estimator_kp>(), param_value<ParamId::estimator_ki>(),
      param_value<ParamId::estimator_accel_healthy_min>(), param_value<ParamId::estimator_accel_healthy_max>()});
  const std::int32_t cal_us = param_value<ParamId::gyro_cal_duration_us>();
  const float cal_movement_max = param_value<ParamId::gyro_cal_movement_max>();
  if (cal_us < 1 || !(cal_movement_max >= 0.0F)) {
    hal_panic("quad_fc composition: gyro_cal_duration_us is below 1 or gyro_cal_movement_max is negative or not a number");
  }
  g_gyro_cal.init(estimator::GyroCalConfig<float>{static_cast<std::uint64_t>(cal_us), cal_movement_max});
  const float small_angle = param_value<ParamId::arm_small_angle>();
  if (!(small_angle > 0.0F && small_angle <= static_cast<float>(prim::kPi))) {
    hal_panic("quad_fc composition: arm_small_angle is outside (0, pi]");
  }
  g_upright_cos = std::cos(small_angle);
  g_estimator_prev_us = 0;
  g_estimator_started = false;
  g_tilt_xy2.store(0.0F, std::memory_order_relaxed);
  g_tilt_wz2.store(1.0F, std::memory_order_relaxed);
  g_estimator_valid.store(false, std::memory_order_relaxed);
  g_thrust_min = static_cast<float>(kMotors) * mixer::f_min(mixer_cfg);
  g_thrust_max = static_cast<float>(kMotors) * mixer::f_max(mixer_cfg);
  g_imu_invalid = false;
  g_imu_saturated = false;
  g_arming_state.store(static_cast<std::uint8_t>(arming::ArmState::Boot), std::memory_order_relaxed);

  const MotorTestPoll seen = hal_motor_test();
  g_seen_cancel = seen.cancel_count;
  g_seen_request = seen.request_count;
  g_counts = MotorTestEvent{};
  g_run = TestRun{};
  g_gate = MotorGate::Disarmed;
  g_n = 0;
}

void tick(const ImuSample& raw) noexcept {
  const std::uint32_t due = g_groups.due(g_n);
  ++g_n;
  const bool rate_due = fired(due, kRate);
  g_gyro_cal.update(raw.gyro_rad_s, (raw.flags & imu_flag(ImuFlag::GyroValid)) != 0, raw.t_us);
  ImuSample s = raw;
  if (g_gyro_cal.complete()) {
    s.gyro_rad_s = raw.gyro_rad_s - g_gyro_cal.bias();
  }
  if ((s.flags & imu_flag(ImuFlag::GyroValid)) != 0) {
    const std::uint64_t since = g_estimator_started && s.t_us > g_estimator_prev_us ? s.t_us - g_estimator_prev_us : 0;
    g_estimator_prev_us = s.t_us;
    g_estimator_started = true;
    const prim::Vec3<float> accel = (s.flags & imu_flag(ImuFlag::AccelValid)) != 0 ? s.accel_m_s2 : kNoAccel;
    g_estimator.step(s.gyro_rad_s, accel,
                     since > std::numeric_limits<std::uint32_t>::max() ? std::numeric_limits<std::uint32_t>::max()
                                                                       : static_cast<std::uint32_t>(since));
  }
  g_step.filter(s, rate_due);
  constexpr std::uint32_t kValid = imu_flag(ImuFlag::GyroValid) | imu_flag(ImuFlag::AccelValid);
  g_imu_invalid = g_imu_invalid || (s.flags & kValid) != kValid;
  g_imu_saturated = g_imu_saturated || (s.flags & saturation_mask()) != 0;
  if (!rate_due) {
    return;
  }
  const ManualControl mc = hal_manual_control();
  const TimeUs now = hal_time_us();
  arming::ArmingInputs<float> in;
  in.now_us = now;
  in.frames = mc.frames;
  in.last_frame_t_us = mc.t_us;
  in.roll = mc.roll;
  in.pitch = mc.pitch;
  in.yaw = mc.yaw;
  in.throttle = mc.throttle;
  in.arm = mc.channels[g_arm_channel];
  in.imu_deadline_missed = hal_imu_deadline_misses() > 0;
  in.imu_samples_valid = !g_imu_invalid;
  in.imu_saturated = g_imu_saturated;
  in.gyro_calibrated = g_gyro_cal.complete();
  in.estimator_valid = g_estimator.valid();
  in.motor_test_active = g_gate == MotorGate::MotorTest;
  const prim::Quat<float>& q = g_estimator.quat();
  in.upright = in.estimator_valid && 1.0F - 2.0F * (q.x * q.x + q.y * q.y) > g_upright_cos;
  g_tilt_xy2.store(q.x * q.x + q.y * q.y, std::memory_order_relaxed);
  g_tilt_wz2.store(q.w * q.w + q.z * q.z, std::memory_order_relaxed);
  g_estimator_valid.store(in.estimator_valid, std::memory_order_relaxed);
  g_imu_invalid = false;
  g_imu_saturated = false;
  const arming::ArmingOutput<float> out = g_arming.step(in);
  g_arming_state.store(static_cast<std::uint8_t>(out.state), std::memory_order_relaxed);
  if (out.armed) {
    if (g_gate != MotorGate::Armed) {
      g_step.reset_controller();  // arming never applies controller state accumulated while disarmed
    }
    g_gate = MotorGate::Armed;
  } else if (g_gate == MotorGate::Armed) {
    g_gate = MotorGate::Disarmed;
  }

  prim::Vec3<float> setpoint;
  float thrust = 0.0F;
  bool angle_run = false;
  if (g_gate == MotorGate::Armed) {
    const prim::Vec3<float> sticks(limited(out.roll, -1.0F), limited(out.pitch, -1.0F), limited(out.yaw, -1.0F));
    thrust = g_thrust_min + limited(out.throttle, 0.0F) * (g_thrust_max - g_thrust_min);
    angle_run = g_angle_switch && mc.channels[g_angle_channel] >= g_angle_threshold;
    if (angle_run) {
      if (!g_angle_running) {
        g_angle.reset();
        g_attitude.reset();
      }
      const AttitudeState<float> state{s.t_us, q, s.gyro_rad_s, in.estimator_valid};
      const attitude::AngleOutput<float> angle =
          g_angle.execute(attitude::AngleSticks<float>{sticks[0], sticks[1], sticks[2]}, state);
      const attitude::AttitudeOutput<float> law = g_attitude.execute(state, angle.q_sp, angle.yaw_rate_cmd);
      g_attitude_faults.store(law.fault_count, std::memory_order_relaxed);
      g_attitude_timing_faults.store(law.timing_fault_count, std::memory_order_relaxed);
      setpoint = law.rate_setpoint;
    } else {
      setpoint = stick_rates::rate_setpoint(g_rates, sticks);
    }
  }
  g_angle_running = angle_run;
  const rate_group::Execution e = angle_run ? g_step.execute_bypass(setpoint, prim::Vec3<float>(), thrust)
                                            : g_step.execute(setpoint, prim::Vec3<float>(), thrust);
  hal_tick_stage(TickStage::Gate);
  update_motor_test(s.t_us);
  hal_actuators_write(gate_output(g_gate, g_run, e.dshot));
}

arming::ArmState arming_status() noexcept {
  return static_cast<arming::ArmState>(g_arming_state.load(std::memory_order_relaxed));
}

AttitudeStatus attitude_status() noexcept {
  const float xy2 = g_tilt_xy2.load(std::memory_order_relaxed);
  const float wz2 = g_tilt_wz2.load(std::memory_order_relaxed);
  return AttitudeStatus{2.0F * std::atan2(std::sqrt(xy2), std::sqrt(wz2)), g_estimator_valid.load(std::memory_order_relaxed)};
}

AttitudeCounts attitude_counts() noexcept {
  return AttitudeCounts{g_attitude_faults.load(std::memory_order_relaxed),
                        g_attitude_timing_faults.load(std::memory_order_relaxed)};
}

RateCounts rate_counts() noexcept { return RateCounts{g_step.execution_count(), g_step.timing_fault_count()}; }

}  // namespace marv::composition
