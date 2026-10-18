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
// Collective thrust. The throttle stick is linear in rotor speed between the mixer's own limits: T = (sqrt(T_min) +
// throttle * (sqrt(T_max) - sqrt(T_min)))^2, which is 4 k (omega_idle + throttle * (omega_max - omega_idle))^2, with
// T_min = sum over the motors of k omega_idle^2 and T_max = sum of k omega_max^2, k the mixer's rotor thrust coefficient
// (mixer::f_min and f_max, per motor): derived from the mixer configuration, no new tunable. This is Betaflight's
// throttle to motor output map (4.5.1 default thr_mid 50, thr_expo 0: the stick reaches the motor output linearly; the
// ESC then maps the output linearly to rotor speed, as the plant and the mixer assume; INFERRED reference, decision 0052,
// Luis 2026-10-17, replacing the thrust-linear map of M2 P4). The two square roots are taken once, at init. The throttle
// is clamped to [0, 1] and the sticks to [-1, 1]; a value that is not finite is 0.
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
// Low throttle (M4 C1, decision 0045; the airmode latch, decision 0050). While armed, the throttle is below arm_throttle_min
// (Betaflight min_check) and the airmode latch is not set, the rate loop's integrator is zeroed after each rate execution
// (RateGroupStep::zero_integrator), in acro and in angle mode: the output of that execution holds at most one increment over a
// zero integrator, and P, D and the filters run as above. The latch g_airmode_active is Betaflight 4.5.1's default airmode
// (core.c:776-783): set on the first armed rate tick whose limited throttle is at or above airmode_activate_throttle, and
// cleared on every tick the gate is not Armed (the gate follows the arming machine's armed output, so a switch, link-loss or
// refusal disarm clears it; a failsafe that keeps the machine armed keeps the latch). Once set, the integrator is not zeroed
// again until a disarm, however low the throttle goes.
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
//
// MARV_QUAD_FC_ESKF (host only; the library marv_composition_quad_fc_eskf, fw/compositions/quad_fc/CMakeLists.txt, decision
// 0055): this source with the attitude and gyro-bias ESKF (marv::estimator::Eskf, parameters eskf_*, validated in validate and
// read in init) stepped beside Mahony on the calibrated gyro from the sample that completes the boot calibration, and its
// quaternion and validity, not Mahony's, going to the arming machine's upright check, the attitude law and the published tilt.
// Mahony is still stepped. Without the definition (the board image, marv_sil_quad_fc) the preprocessed translation unit is
// the one without these lines.
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
#ifdef MARV_QUAD_FC_ESKF
#include <marv/estimator/eskf.hpp>
#endif
#include <marv/hal/hal.hpp>
#include <marv/hal/tick.hpp>
#include <marv/hal_target/dshot.hpp>
#include <marv/hal_target/rc_input.hpp>
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
float g_thrust_root_min = 0.0F;  // sqrt(T_min), sqrt(N)
float g_thrust_root_max = 0.0F;  // sqrt(T_max), sqrt(N)
std::size_t g_arm_channel = 0;  // index into ManualControl::channels (rc_channel_arm - 1)
bool g_angle_switch = false;    // rc_channel_angle is not 0
std::size_t g_angle_channel = 0;  // index into ManualControl::channels (rc_channel_angle - 1), when g_angle_switch
float g_angle_threshold = 0.0F;   // angle_switch_threshold
float g_throttle_min = 0.0F;      // arm_throttle_min, Betaflight min_check: below it, until the airmode latch, the rate integrator is held at zero
float g_airmode_threshold = 0.0F;  // airmode_activate_throttle: armed at or above it sets g_airmode_active
bool g_airmode_active = false;     // the airmode latch (decision 0050): set while armed at the threshold, cleared when not Armed
bool g_angle_running = false;     // the latest rate tick was armed and angle
attitude::AngleMode<float> g_angle;
attitude::AttitudeLaw<float> g_attitude;
std::atomic<std::uint32_t> g_attitude_faults{0};         // the law's fault_count, published after each execution
std::atomic<std::uint32_t> g_attitude_timing_faults{0};  // and its timing_fault_count
bool g_imu_invalid = false;     // a sample since the previous rate tick lacked GyroValid or AccelValid
bool g_imu_saturated = false;   // a sample since the previous rate tick had a saturation bit
std::atomic<std::uint8_t> g_arming_state{static_cast<std::uint8_t>(arming::ArmState::Boot)};
std::atomic<bool> g_maintenance_request{false};  // set by the background loop (request_maintenance), cleared by the rate tick that reads it
std::atomic<bool> g_maintenance_granted{false};  // the arming machine's grant, published after each rate tick
arming::ConfigStatus g_config_status = arming::ConfigStatus::Ok;  // set before init (set_config_status), read by init
estimator::Mahony<float> g_estimator;
estimator::GyroCal<float> g_gyro_cal;
#ifdef MARV_QUAD_FC_ESKF
estimator::Eskf<float> g_eskf;
#endif
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

#ifdef MARV_QUAD_FC_ESKF
[[nodiscard]] estimator::EskfConfig<float> eskf_config(ParamTable t) noexcept {
  estimator::EskfConfig<float> c;
  c.gyro_noise_density = param_value<ParamId::eskf_gyro_noise_density>(t);
  c.gyro_bias_random_walk = param_value<ParamId::eskf_gyro_bias_random_walk>(t);
  c.accel_noise_density = param_value<ParamId::eskf_accel_noise_density>(t);
  c.angle_sigma_init = param_value<ParamId::eskf_angle_sigma_init>(t);
  c.gyro_bias_sigma_init = param_value<ParamId::eskf_gyro_bias_sigma_init>(t);
  c.accel_gate = param_value<ParamId::eskf_accel_gate>(t);
  c.accel_reset_us = static_cast<std::uint64_t>(param_value<ParamId::eskf_accel_reset_us>(t));
  c.angle_var_min = param_value<ParamId::eskf_angle_var_min>(t);
  c.angle_var_max = param_value<ParamId::eskf_angle_var_max>(t);
  c.gyro_bias_var_min = param_value<ParamId::eskf_gyro_bias_var_min>(t);
  c.gyro_bias_var_max = param_value<ParamId::eskf_gyro_bias_var_max>(t);
  c.accel_healthy_min = param_value<ParamId::estimator_accel_healthy_min>(t);
  c.accel_healthy_max = param_value<ParamId::estimator_accel_healthy_max>(t);
  c.accel_fusion_divisor = static_cast<std::uint32_t>(param_value<ParamId::eskf_accel_fusion_divisor>(t));
  c.accel_model_noise = param_value<ParamId::eskf_accel_model_noise>(t);
  return c;
}

// The ESKF configuration's rules: finite, noise densities and the gate positive, the sigmas, the walk and the model error not
// negative, the variance limits ordered, the window and the timeout at least 1, the health window ordered.
[[nodiscard]] const char* eskf_reject_reason(ParamTable t) noexcept {
  const estimator::EskfConfig<float> c = eskf_config(t);
  if (!(c.gyro_noise_density > 0.0F && c.accel_noise_density > 0.0F && c.accel_gate > 0.0F && c.angle_sigma_init > 0.0F)) {
    return "quad_fc composition: an eskf_* noise density, eskf_accel_gate or eskf_angle_sigma_init is not positive";
  }
  if (!(c.gyro_bias_random_walk >= 0.0F && c.gyro_bias_sigma_init >= 0.0F && c.accel_model_noise >= 0.0F)) {
    return "quad_fc composition: eskf_gyro_bias_random_walk, eskf_gyro_bias_sigma_init or eskf_accel_model_noise is negative or not a number";
  }
  if (!(c.angle_var_min >= 0.0F && c.angle_var_min <= c.angle_var_max && c.gyro_bias_var_min >= 0.0F &&
        c.gyro_bias_var_min <= c.gyro_bias_var_max)) {
    return "quad_fc composition: an eskf_* variance limit is negative, not a number or its floor is above its ceiling";
  }
  if (param_value<ParamId::eskf_accel_fusion_divisor>(t) < 1 || param_value<ParamId::eskf_accel_reset_us>(t) < 1) {
    return "quad_fc composition: eskf_accel_fusion_divisor or eskf_accel_reset_us is below 1";
  }
  if (!(c.accel_healthy_min > 0.0F && c.accel_healthy_min < c.accel_healthy_max)) {
    return "quad_fc composition: estimator_accel_healthy_min is not in (0, estimator_accel_healthy_max)";
  }
  return nullptr;
}
#endif

// The checks init and the target's start-ups make, in their order, on `t`; the first failure's text, else null.
[[nodiscard]] const char* reject_reason(ParamTable t) noexcept {
  if (t.size() != kParamCount) {
    return "quad_fc composition: the parameter table does not have one record per parameter";
  }
  if (const char* why = rate::reject_reason(rate::from_params(t)); why != nullptr) {
    return why;
  }
  if (const char* why = mixer::reject_reason(mixer::from_params(t)); why != nullptr) {
    return why;
  }
  const std::int32_t divisor = param_value<ParamId::rate_loop_divisor>(t);
  if (divisor < 1) {
    return "quad_fc composition: rate_loop_divisor is below 1";
  }
  if (const char* why = rate_group::reject_reason(rate_group::chain_from_params(t)); why != nullptr) {
    return why;
  }
  const std::int64_t cap = param_value<ParamId::motor_test_dshot_max>(t);
  const std::int64_t max_ms = param_value<ParamId::motor_test_duration_max_ms>(t);
  if (cap < prim::kDshotThrottleMin || cap > prim::kDshotThrottleMax || max_ms < 1) {
    return "quad_fc composition: motor_test_dshot_max is outside the DShot throttle range or the duration is below 1 ms";
  }
  if (param_value<ParamId::tick_period_num_us>(t) < 1 || param_value<ParamId::tick_period_den>(t) < 1) {
    return "quad_fc composition: the tick period is below 1";
  }
  stick_rates::RatesConfig<float> rates;
  rates.roll = {param_value<ParamId::rates_centre_roll>(t), param_value<ParamId::rate_max_roll>(t),
                param_value<ParamId::rates_expo_roll>(t)};
  rates.pitch = {param_value<ParamId::rates_centre_pitch>(t), param_value<ParamId::rate_max_pitch>(t),
                 param_value<ParamId::rates_expo_pitch>(t)};
  rates.yaw = {param_value<ParamId::rates_centre_yaw>(t), param_value<ParamId::rate_max_yaw>(t),
               param_value<ParamId::rates_expo_yaw>(t)};
  if (stick_rates::validate(rates) != stick_rates::ConfigError::None) {
    return "quad_fc composition: the rates curve is invalid (0 < centre <= max, 0 <= expo <= 1, finite)";
  }
  const std::int32_t channel = param_value<ParamId::rc_channel_arm>(t);
  if (channel < 1 || static_cast<std::size_t>(channel) > prim::kCrsfRcChannels) {
    return "quad_fc composition: rc_channel_arm is outside 1..16";
  }
  const std::int32_t angle_channel = param_value<ParamId::rc_channel_angle>(t);
  if (angle_channel < 0 || static_cast<std::size_t>(angle_channel) > prim::kCrsfRcChannels) {
    return "quad_fc composition: rc_channel_angle is outside 0..16";
  }
  if (angle_channel != 0 &&
      (angle_channel == channel || angle_channel == param_value<ParamId::rc_channel_roll>(t) ||
       angle_channel == param_value<ParamId::rc_channel_pitch>(t) ||
       angle_channel == param_value<ParamId::rc_channel_throttle>(t) ||
       angle_channel == param_value<ParamId::rc_channel_yaw>(t))) {
    return "quad_fc composition: rc_channel_angle is the arm channel or a stick channel";
  }
  if (param_value<ParamId::att_loop_ratio>(t) != 1) {
    return "quad_fc composition: att_loop_ratio is not 1 (the attitude law runs once per rate tick)";
  }
  if (const char* why = attitude::reject_reason(attitude::from_params(t)); why != nullptr) {
    return why;
  }
  if (const char* why = arming::reject_reason(t); why != nullptr) {
    return why;
  }
#ifdef MARV_QUAD_FC_ESKF
  if (const char* why = eskf_reject_reason(t); why != nullptr) {
    return why;
  }
#endif
  const std::int32_t cal_us = param_value<ParamId::gyro_cal_duration_us>(t);
  const float cal_movement_max = param_value<ParamId::gyro_cal_movement_max>(t);
  if (cal_us < 1 || !(cal_movement_max >= 0.0F)) {
    return "quad_fc composition: gyro_cal_duration_us is below 1 or gyro_cal_movement_max is negative or not a number";
  }
  const float small_angle = param_value<ParamId::arm_small_angle>(t);
  if (!(small_angle > 0.0F && small_angle <= static_cast<float>(prim::kPi))) {
    return "quad_fc composition: arm_small_angle is outside (0, pi]";
  }
  // The target's start-ups after init (fw/hal/target/src/main.cpp): motor_output_start, then receiver_start.
  hal_target::PadMap pad{};
  if (!hal_target::make_pad_map({param_value<ParamId::motor_pad_m1>(t), param_value<ParamId::motor_pad_m2>(t),
                                 param_value<ParamId::motor_pad_m3>(t), param_value<ParamId::motor_pad_m4>(t)},
                                pad)) {
    return "motor_pad is not a permutation of the motor outputs";
  }
  if (!hal_target::rc_channel_map_valid({param_value<ParamId::rc_channel_roll>(t), param_value<ParamId::rc_channel_pitch>(t),
                                         param_value<ParamId::rc_channel_throttle>(t),
                                         param_value<ParamId::rc_channel_yaw>(t)})) {
    return "the rc_channel_* parameters are not channel numbers 1 .. 16";
  }
  return nullptr;
}

}  // namespace

Verdict validate(ParamTable table) noexcept { return Verdict{reject_reason(table)}; }

void init() noexcept {
  const Verdict verdict = validate(param_active_table());
  if (!verdict.ok()) {
    hal_panic(verdict.reason);
  }
  // From here the table is accepted: every check the loaders and this function used to make has passed in validate.
  const rate::RateConfig<float> rate_cfg = rate::load_config();
  const mixer::MixerConfig<float> mixer_cfg = mixer::load_config();

  const std::int32_t divisor = param_value<ParamId::rate_loop_divisor>();
  static_cast<void>(g_groups.init({static_cast<std::uint32_t>(divisor)}));
  g_step.init(rate_cfg, mixer_cfg, rate_group::load_chain_config());

  g_cap = param_value<ParamId::motor_test_dshot_max>();
  g_max_ms = param_value<ParamId::motor_test_duration_max_ms>();
  const std::int32_t num = param_value<ParamId::tick_period_num_us>();
  const std::int32_t den = param_value<ParamId::tick_period_den>();
  g_rate_num = static_cast<std::int64_t>(divisor) * num;
  g_rate_den = den;

  g_rates.roll = {param_value<ParamId::rates_centre_roll>(), param_value<ParamId::rate_max_roll>(),
                  param_value<ParamId::rates_expo_roll>()};
  g_rates.pitch = {param_value<ParamId::rates_centre_pitch>(), param_value<ParamId::rate_max_pitch>(),
                   param_value<ParamId::rates_expo_pitch>()};
  g_rates.yaw = {param_value<ParamId::rates_centre_yaw>(), param_value<ParamId::rate_max_yaw>(),
                 param_value<ParamId::rates_expo_yaw>()};
  const std::int32_t channel = param_value<ParamId::rc_channel_arm>();
  g_arm_channel = static_cast<std::size_t>(channel) - 1;
  const std::int32_t angle_channel = param_value<ParamId::rc_channel_angle>();
  g_angle_switch = angle_channel != 0;
  g_angle_channel = g_angle_switch ? static_cast<std::size_t>(angle_channel) - 1 : 0;
  g_angle_threshold = param_value<ParamId::angle_switch_threshold>();
  g_throttle_min = param_value<ParamId::arm_throttle_min>();
  g_airmode_threshold = param_value<ParamId::airmode_activate_throttle>();
  const attitude::AttitudeConfig<float> attitude_cfg = attitude::load_config();
  g_angle.init(attitude_cfg);
  g_attitude.init(attitude_cfg);
  g_angle_running = false;
  g_attitude_faults.store(0, std::memory_order_relaxed);
  g_attitude_timing_faults.store(0, std::memory_order_relaxed);
  g_arming.init(arming::load_config(), g_config_status);
  g_maintenance_request.store(false, std::memory_order_relaxed);
  g_maintenance_granted.store(false, std::memory_order_relaxed);
  g_estimator.init(estimator::MahonyConfig<float>{
      param_value<ParamId::estimator_kp>(), param_value<ParamId::estimator_ki>(),
      param_value<ParamId::estimator_accel_healthy_min>(), param_value<ParamId::estimator_accel_healthy_max>()});
#ifdef MARV_QUAD_FC_ESKF
  g_eskf.init(eskf_config(param_active_table()));
#endif
  const std::int32_t cal_us = param_value<ParamId::gyro_cal_duration_us>();
  const float cal_movement_max = param_value<ParamId::gyro_cal_movement_max>();
  g_gyro_cal.init(estimator::GyroCalConfig<float>{static_cast<std::uint64_t>(cal_us), cal_movement_max});
  const float small_angle = param_value<ParamId::arm_small_angle>();
  g_upright_cos = std::cos(small_angle);
  g_estimator_prev_us = 0;
  g_estimator_started = false;
  g_tilt_xy2.store(0.0F, std::memory_order_relaxed);
  g_tilt_wz2.store(1.0F, std::memory_order_relaxed);
  g_estimator_valid.store(false, std::memory_order_relaxed);
  g_thrust_root_min = std::sqrt(static_cast<float>(kMotors) * mixer::f_min(mixer_cfg));
  g_thrust_root_max = std::sqrt(static_cast<float>(kMotors) * mixer::f_max(mixer_cfg));
  g_imu_invalid = false;
  g_imu_saturated = false;
  g_arming_state.store(static_cast<std::uint8_t>(arming::ArmState::Boot), std::memory_order_relaxed);

  const MotorTestPoll seen = hal_motor_test();
  g_seen_cancel = seen.cancel_count;
  g_seen_request = seen.request_count;
  g_counts = MotorTestEvent{};
  g_run = TestRun{};
  g_gate = MotorGate::Disarmed;
  g_airmode_active = false;
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
#ifdef MARV_QUAD_FC_ESKF
    if (g_gyro_cal.complete()) {
      g_eskf.step(s.gyro_rad_s, accel,
                  since > std::numeric_limits<std::uint32_t>::max() ? std::numeric_limits<std::uint32_t>::max()
                                                                    : static_cast<std::uint32_t>(since));
    }
#endif
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
#ifdef MARV_QUAD_FC_ESKF
  in.estimator_valid = g_eskf.valid();
#else
  in.estimator_valid = g_estimator.valid();
#endif
  in.motor_test_active = g_gate == MotorGate::MotorTest;
  in.maintenance_request = g_maintenance_request.load(std::memory_order_relaxed);
  if (in.maintenance_request) {
    g_maintenance_request.store(false, std::memory_order_relaxed);  // only this tick clears it, so a request is never lost
  }
#ifdef MARV_QUAD_FC_ESKF
  const prim::Quat<float>& q = g_eskf.quat();
#else
  const prim::Quat<float>& q = g_estimator.quat();
#endif
  in.upright = in.estimator_valid && 1.0F - 2.0F * (q.x * q.x + q.y * q.y) > g_upright_cos;
  g_tilt_xy2.store(q.x * q.x + q.y * q.y, std::memory_order_relaxed);
  g_tilt_wz2.store(q.w * q.w + q.z * q.z, std::memory_order_relaxed);
  g_estimator_valid.store(in.estimator_valid, std::memory_order_relaxed);
  g_imu_invalid = false;
  g_imu_saturated = false;
  const arming::ArmingOutput<float> out = g_arming.step(in);
  g_arming_state.store(static_cast<std::uint8_t>(out.state), std::memory_order_relaxed);
  g_maintenance_granted.store(g_arming.maintenance(), std::memory_order_relaxed);
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
  bool throttle_low = false;
  if (g_gate == MotorGate::Armed) {
    const prim::Vec3<float> sticks(limited(out.roll, -1.0F), limited(out.pitch, -1.0F), limited(out.yaw, -1.0F));
    const float throttle = limited(out.throttle, 0.0F);
    if (throttle >= g_airmode_threshold) {
      g_airmode_active = true;
    }
    throttle_low = throttle < g_throttle_min && !g_airmode_active;
    const float thrust_root = g_thrust_root_min + throttle * (g_thrust_root_max - g_thrust_root_min);
    thrust = thrust_root * thrust_root;
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
  if (g_gate != MotorGate::Armed) {
    g_airmode_active = false;
  }
  g_angle_running = angle_run;
  const rate_group::Execution e = angle_run ? g_step.execute_bypass(setpoint, prim::Vec3<float>(), thrust)
                                            : g_step.execute(setpoint, prim::Vec3<float>(), thrust);
  if (throttle_low) {
    g_step.zero_integrator();  // armed below min_check before the airmode latch: zeroed after the output, as Betaflight's pidResetIterm (decisions 0045, 0050)
  }
  hal_tick_stage(TickStage::Gate);
  update_motor_test(s.t_us);
  hal_actuators_write(gate_output(g_gate, g_run, e.dshot));
}

arming::ArmState arming_status() noexcept {
  return static_cast<arming::ArmState>(g_arming_state.load(std::memory_order_relaxed));
}

void set_config_status(arming::ConfigStatus status) noexcept { g_config_status = status; }

arming::ConfigStatus config_status() noexcept { return g_config_status; }

void request_maintenance() noexcept { g_maintenance_request.store(true, std::memory_order_relaxed); }

bool maintenance_granted() noexcept { return g_maintenance_granted.load(std::memory_order_relaxed); }

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
