// The L5 test composition: angle mode, the attitude law (marv::attitude), and the gyro chain, the rate loop in bypass
// and the mixer through the shared rate-group step (marv::rate_group), flown with scripted inputs. Scenario parameters
// (fw/compositions/l5_attitude_scripted/params) give the collective thrust request, a piecewise-constant stick script,
// one rate-setpoint chirp and a constant yaw torque disturbance; nothing here is a vehicle number. The attitude is the
// state latched by attitude_input (truth at L5).
//
// Per tick, in this order (decision 0006 E: the attitude group runs before the rate group in the same tick, and its
// output acts at that tick):
//   attitude group, every att_loop_ratio * rate_loop_divisor ticks (tick 0 included):
//     the latched state must carry the sample stamp (else hal_panic, decision 0006 B);
//     angle = angle_mode.execute(sticks(t), state); rate_sp = attitude.execute(state, angle.q_sp, angle.yaw_rate_cmd).
//   every tick: step.filter(sample, due) (rate_group.hpp: on a rate-group tick the notches are updated first).
//   rate group, every rate_loop_divisor ticks (tick 0 included):
//     step.execute_bypass(rate_sp + chirp(t), disturbance(t), thrust): the rate loop in bypass on the chain output,
//     request = out.torque + disturbance(t), allocate, record_allocation, the rate group's DshotDiffuser (decision 0017);
//     the DShot is written.
// The rate setpoint rate_sp is held between attitude executions. On a tick with no rate group nothing is written: the HAL
// latch holds the last command (hal_sim keeps it between writes). init panics naming the violated rule on an invalid
// attitude, rate, mixer or chain configuration or invalid scenario parameters.
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include <marv/attitude/angle_mode.hpp>
#include <marv/attitude/attitude_law.hpp>
#include <marv/attitude/config.hpp>
#include <marv/composition.hpp>
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
Script g_script;
Chirp<float> g_chirp;
Disturbance<float> g_disturbance;
float g_thrust = 0.0F;
Tick g_n = 0;
AttitudeState<float> g_latch{};
bool g_have_latch = false;
Vec3f g_rate_setpoint;

[[nodiscard]] const char* message(ScriptError e) noexcept {
  switch (e) {
    case ScriptError::None:
      break;
    case ScriptError::SegmentCount:
      return "l5_attitude_scripted composition: l5_seg_count is not in 0..capacity";
    case ScriptError::SegmentTime:
      return "l5_attitude_scripted composition: segment stamps l5_seg<k>_t_us are not >= 0 and strictly increasing";
    case ScriptError::SegmentStick:
      return "l5_attitude_scripted composition: a segment stick l5_seg<k>_{roll,pitch,yaw} is not finite and in [-1, 1]";
    case ScriptError::ChirpAxis:
      return "l5_attitude_scripted composition: l5_chirp_axis is not 0 (none), 1 (roll), 2 (pitch) or 3 (yaw)";
    case ScriptError::ChirpAmplitude:
      return "l5_attitude_scripted composition: l5_chirp_amp_rad_s is not finite and >= 0";
    case ScriptError::ChirpBand:
      return "l5_attitude_scripted composition: l5_chirp_w_lo, l5_chirp_w_hi do not satisfy 0 < w_lo < w_hi";
    case ScriptError::ChirpDuration:
      return "l5_attitude_scripted composition: l5_chirp_dur_us is not > 0";
    case ScriptError::ChirpStart:
      return "l5_attitude_scripted composition: l5_chirp_t0_us is negative";
    case ScriptError::DisturbanceTorque:
      return "l5_attitude_scripted composition: l5_dist_yaw_nm is not finite";
    case ScriptError::DisturbanceStart:
      return "l5_attitude_scripted composition: l5_dist_t0_us is negative";
  }
  return "l5_attitude_scripted composition: unknown scenario parameter error";
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

[[nodiscard]] Chirp<float> load_chirp() noexcept {
  Chirp<float> c;
  c.axis = static_cast<ChirpAxis>(param_value<ParamId::l5_chirp_axis>());
  c.amp = param_value<ParamId::l5_chirp_amp_rad_s>();
  c.w_lo = param_value<ParamId::l5_chirp_w_lo>();
  c.w_hi = param_value<ParamId::l5_chirp_w_hi>();
  c.t0_us = param_value<ParamId::l5_chirp_t0_us>();
  c.dur_us = param_value<ParamId::l5_chirp_dur_us>();
  return c;
}

[[nodiscard]] Disturbance<float> load_disturbance() noexcept {
  Disturbance<float> d;
  d.yaw_nm = param_value<ParamId::l5_dist_yaw_nm>();
  d.t0_us = param_value<ParamId::l5_dist_t0_us>();
  return d;
}

void require_valid_script(ScriptError e) noexcept {
  if (e != ScriptError::None) {
    hal_panic(message(e));
  }
}

[[nodiscard]] bool fired(std::uint32_t due, Group g) noexcept { return (due & (std::uint32_t{1} << g)) != 0; }

}  // namespace

void attitude_input(const AttitudeState<float>& a) noexcept {
  g_latch = a;
  g_have_latch = true;
}

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
    hal_panic("l5_attitude_scripted composition: rate_loop_divisor or att_loop_ratio is below 1");
  }
  const std::uint64_t attitude_divisor = static_cast<std::uint64_t>(divisor) * static_cast<std::uint64_t>(ratio);
  if (attitude_divisor > std::numeric_limits<std::uint32_t>::max() ||
      !g_groups.init({static_cast<std::uint32_t>(attitude_divisor), static_cast<std::uint32_t>(divisor)})) {
    hal_panic("l5_attitude_scripted composition: rate_loop_divisor * att_loop_ratio is out of range");
  }
  g_step.init(rate_cfg, mixer_cfg, rate_group::load_chain_config());

  g_thrust = param_value<ParamId::l5_thrust_n>();
  if (!std::isfinite(g_thrust) || !(g_thrust >= 0.0F)) {
    hal_panic("l5_attitude_scripted composition: l5_thrust_n is not finite and >= 0");
  }
  g_script = load_script();
  require_valid_script(validate(g_script));
  g_chirp = load_chirp();
  require_valid_script(validate(g_chirp));
  g_disturbance = load_disturbance();
  require_valid_script(validate(g_disturbance));
  g_rate_setpoint = Vec3f();
  g_have_latch = false;
  g_n = 0;
}

void tick(const ImuSample& s) noexcept {
  if (hal_time_us() != s.t_us) {
    hal_panic("l5_attitude_scripted composition: hal_time_us differs from the sample stamp");
  }
  const std::uint32_t due = g_groups.due(g_n);
  ++g_n;
  if (fired(due, kAttitude)) {
    if (!g_have_latch || g_latch.t_us != s.t_us) {
      hal_panic("l5_attitude_scripted composition: the latched attitude stamp differs from the sample stamp");
    }
    const Vec3f stick = sticks_at(g_script, s.t_us);
    const attitude::AngleOutput<float> angle = g_angle.execute(attitude::AngleSticks<float>{stick[0], stick[1], stick[2]}, g_latch);
    g_rate_setpoint = g_attitude.execute(g_latch, angle.q_sp, angle.yaw_rate_cmd).rate_setpoint;
  }
  const bool rate_due = fired(due, kRate);
  g_step.filter(s, rate_due);
  if (!rate_due) {
    return;
  }
  const rate_group::Execution e = g_step.execute_bypass(g_rate_setpoint + chirp_rate(g_chirp, s.t_us),
                                                        disturbance_torque(g_disturbance, s.t_us), g_thrust);
  ActuatorOutput<kMotors, kServos> o{};
  o.motor = e.dshot;
  hal_actuators_write(o);
}

}  // namespace marv::composition
