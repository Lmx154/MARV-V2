// The L7 test composition (quad spec section 4.0 M3, decision 0042 item 4): the l5_attitude_scripted logic (angle mode, the
// attitude law, the rate loop in bypass and the mixer through the shared rate-group step, a piecewise-constant stick script
// and a collective thrust from scenario parameters) with the attitude state from the Mahony estimator (marv::estimator) on
// the IMU sample, the only attitude source: no truth hook, no truth symbol. The chirp and the yaw disturbance of the L5
// composition are not read. Nothing here is a vehicle number; the estimator gains come from the parameters
// (estimator_kp, estimator_ki, estimator_accel_healthy_min and _max, the quad_fc register).
//
// Per tick, in this order:
//   the estimator steps on the sample (gyro and accelerometer, dt from the stamps, as quad_fc: a sample without GyroValid is
//   not stepped, one without AccelValid gives no correction);
//   attitude group, every att_loop_ratio * rate_loop_divisor ticks (tick 0 included):
//     state = {sample stamp, estimate, sample gyro, estimator valid};
//     angle = angle_mode.execute(sticks(t), state); rate_sp = attitude.execute(state, angle.q_sp, angle.yaw_rate_cmd).
//   every tick: step.filter(sample, due).
//   rate group, every rate_loop_divisor ticks (tick 0 included): the rate loop in bypass, allocate, DShot written.
// An invalid estimate (before the first healthy sample) makes the attitude law return a zero rate setpoint and a fault for
// that execution. init panics naming the violated rule on an invalid configuration or scenario parameters.
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include <marv/attitude/angle_mode.hpp>
#include <marv/attitude/attitude_law.hpp>
#include <marv/attitude/config.hpp>
#include <marv/composition.hpp>
#include <marv/estimator/mahony.hpp>
#include <marv/hal/hal.hpp>
#include <marv/hal/tick.hpp>
#include <marv/l5_script.hpp>
#include <marv/mixer/mixer.hpp>
#include <marv/params/param.hpp>
#include <marv/prim/vec.hpp>
#include <marv/rate/rate_loop.hpp>
#include <marv/rate_group/rate_group.hpp>
#include <marv/sched/rate_groups.hpp>
#include <marv/types/actuator.hpp>
#include <marv/types/attitude_state.hpp>
#include <marv/types/imu_sample.hpp>

namespace marv::composition {
namespace {

struct SegmentIds {
  ParamId t_us;
  ParamId roll;
  ParamId pitch;
  ParamId yaw;
};

// The segment capacity K is the number of entries here, which is the number of l5_seg<k>_* entries of the register.
constexpr std::array kSegmentIds{
    SegmentIds{ParamId::l5_seg1_t_us, ParamId::l5_seg1_roll, ParamId::l5_seg1_pitch, ParamId::l5_seg1_yaw},
    SegmentIds{ParamId::l5_seg2_t_us, ParamId::l5_seg2_roll, ParamId::l5_seg2_pitch, ParamId::l5_seg2_yaw},
    SegmentIds{ParamId::l5_seg3_t_us, ParamId::l5_seg3_roll, ParamId::l5_seg3_pitch, ParamId::l5_seg3_yaw},
    SegmentIds{ParamId::l5_seg4_t_us, ParamId::l5_seg4_roll, ParamId::l5_seg4_pitch, ParamId::l5_seg4_yaw},
    SegmentIds{ParamId::l5_seg5_t_us, ParamId::l5_seg5_roll, ParamId::l5_seg5_pitch, ParamId::l5_seg5_yaw},
    SegmentIds{ParamId::l5_seg6_t_us, ParamId::l5_seg6_roll, ParamId::l5_seg6_pitch, ParamId::l5_seg6_yaw},
    SegmentIds{ParamId::l5_seg7_t_us, ParamId::l5_seg7_roll, ParamId::l5_seg7_pitch, ParamId::l5_seg7_yaw},
    SegmentIds{ParamId::l5_seg8_t_us, ParamId::l5_seg8_roll, ParamId::l5_seg8_pitch, ParamId::l5_seg8_yaw}};

constexpr std::size_t kSegments = kSegmentIds.size();

using Script = StickScript<float, kSegments>;
using Vec3f = prim::Vec3<float>;

// Group index order is the dispatch order (sched::RateGroups: lowest index first): the attitude group runs first.
enum Group : std::size_t { kAttitude, kRate, kGroupCount };

sched::RateGroups<kGroupCount> g_groups;
attitude::AngleMode<float> g_angle;
attitude::AttitudeLaw<float> g_attitude;
rate_group::RateGroupStep g_step;
estimator::Mahony<float> g_estimator;
Script g_script;
float g_thrust = 0.0F;
Tick g_n = 0;
TimeUs g_prev_us = 0;  // the stamp of the previous sample the estimator stepped on
bool g_started = false;
Vec3f g_rate_setpoint;

// The accelerometer of a sample whose AccelValid bit is clear: no direction, so no correction that step (as quad_fc).
const Vec3f kNoAccel{std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
                     std::numeric_limits<float>::quiet_NaN()};

[[nodiscard]] const char* message(ScriptError e) noexcept {
  switch (e) {
    case ScriptError::None:
      break;
    case ScriptError::SegmentCount:
      return "l7_attitude_estimated composition: l5_seg_count is not in 0..capacity";
    case ScriptError::SegmentTime:
      return "l7_attitude_estimated composition: segment stamps l5_seg<k>_t_us are not >= 0 and strictly increasing";
    case ScriptError::SegmentStick:
      return "l7_attitude_estimated composition: a segment stick l5_seg<k>_{roll,pitch,yaw} is not finite and in [-1, 1]";
    case ScriptError::ChirpAxis:
    case ScriptError::ChirpAmplitude:
    case ScriptError::ChirpBand:
    case ScriptError::ChirpDuration:
    case ScriptError::ChirpStart:
    case ScriptError::DisturbanceTorque:
    case ScriptError::DisturbanceStart:
      break;
  }
  return "l7_attitude_estimated composition: unknown scenario parameter error";
}

[[nodiscard]] Script load_script() noexcept {
  Script s;
  s.count = param_value<ParamId::l5_seg_count>();
  for (std::size_t k = 0; k < kSegments; ++k) {
    s.segment[k].t_us = param_get(kSegmentIds[k].t_us).value.i32;
    s.segment[k].stick = Vec3f(param_get(kSegmentIds[k].roll).value.f32, param_get(kSegmentIds[k].pitch).value.f32,
                               param_get(kSegmentIds[k].yaw).value.f32);
  }
  return s;
}

[[nodiscard]] bool fired(std::uint32_t due, Group g) noexcept { return (due & (std::uint32_t{1} << g)) != 0; }

}  // namespace

void init() noexcept {
  const rate::RateConfig<float> rate_cfg = rate::load_config();
  const mixer::MixerConfig<float> mixer_cfg = mixer::load_config();
  const attitude::AttitudeConfig<float> attitude_cfg = attitude::load_config();
  g_angle.init(attitude_cfg);
  g_attitude.init(attitude_cfg);

  // The attitude divisor is rate_loop_divisor * att_loop_ratio (decision 0006 E), so the attitude ticks are a subset of
  // the rate ticks. The product is formed in 64 bits and must fit the scheduler's 32.
  const std::int32_t divisor = param_value<ParamId::rate_loop_divisor>();
  const std::int32_t ratio = param_value<ParamId::att_loop_ratio>();
  if (divisor < 1 || ratio < 1) {
    hal_panic("l7_attitude_estimated composition: rate_loop_divisor or att_loop_ratio is below 1");
  }
  const std::uint64_t attitude_divisor = static_cast<std::uint64_t>(divisor) * static_cast<std::uint64_t>(ratio);
  if (attitude_divisor > std::numeric_limits<std::uint32_t>::max() ||
      !g_groups.init({static_cast<std::uint32_t>(attitude_divisor), static_cast<std::uint32_t>(divisor)})) {
    hal_panic("l7_attitude_estimated composition: rate_loop_divisor * att_loop_ratio is out of range");
  }
  g_step.init(rate_cfg, mixer_cfg, rate_group::load_chain_config());
  g_estimator.init(estimator::MahonyConfig<float>{
      param_value<ParamId::estimator_kp>(), param_value<ParamId::estimator_ki>(),
      param_value<ParamId::estimator_accel_healthy_min>(), param_value<ParamId::estimator_accel_healthy_max>()});

  g_thrust = param_value<ParamId::l5_thrust_n>();
  if (!std::isfinite(g_thrust) || !(g_thrust >= 0.0F)) {
    hal_panic("l7_attitude_estimated composition: l5_thrust_n is not finite and >= 0");
  }
  g_script = load_script();
  const ScriptError e = validate(g_script);
  if (e != ScriptError::None) {
    hal_panic(message(e));
  }
  g_rate_setpoint = Vec3f();
  g_prev_us = 0;
  g_started = false;
  g_n = 0;
}

void tick(const ImuSample& s) noexcept {
  if (hal_time_us() != s.t_us) {
    hal_panic("l7_attitude_estimated composition: hal_time_us differs from the sample stamp");
  }
  const std::uint32_t due = g_groups.due(g_n);
  ++g_n;
  if ((s.flags & imu_flag(ImuFlag::GyroValid)) != 0) {
    const std::uint64_t since = g_started && s.t_us > g_prev_us ? s.t_us - g_prev_us : 0;
    g_prev_us = s.t_us;
    g_started = true;
    const Vec3f accel = (s.flags & imu_flag(ImuFlag::AccelValid)) != 0 ? s.accel_m_s2 : kNoAccel;
    g_estimator.step(s.gyro_rad_s, accel,
                     since > std::numeric_limits<std::uint32_t>::max() ? std::numeric_limits<std::uint32_t>::max()
                                                                       : static_cast<std::uint32_t>(since));
  }
  if (fired(due, kAttitude)) {
    const AttitudeState<float> state{s.t_us, g_estimator.quat(), s.gyro_rad_s, g_estimator.valid()};
    const Vec3f stick = sticks_at(g_script, s.t_us);
    const attitude::AngleOutput<float> angle =
        g_angle.execute(attitude::AngleSticks<float>{stick[0], stick[1], stick[2]}, state);
    g_rate_setpoint = g_attitude.execute(state, angle.q_sp, angle.yaw_rate_cmd).rate_setpoint;
  }
  const bool rate_due = fired(due, kRate);
  g_step.filter(s, rate_due);
  if (!rate_due) {
    return;
  }
  const rate_group::Execution e = g_step.execute_bypass(g_rate_setpoint, Vec3f(), g_thrust);
  ActuatorOutput<kMotors, kServos> o{};
  o.motor = e.dshot;
  hal_actuators_write(o);
}

}  // namespace marv::composition
