// The L11 test composition: the L5 composition (angle mode, the attitude law, and the gyro chain, the rate loop in bypass
// and the mixer through the shared rate-group step) with the vertical loop (marv::vertical) in place of L5's constant
// collective, flown with scripted inputs. Scenario parameters (fw/compositions/l11_vertical_scripted/params) give the
// hover feed-forward T_h, a piecewise-constant script of sticks and height reference, a constant thrust disturbance and a
// torque disturbance pulse; nothing here is a vehicle number. The attitude and the vertical state are the states latched
// by attitude_input and vertical_input (truth at L11).
//
// Per tick, in this order (decision 0020 section 1, "Rate group": attitude, vertical, rate in the same tick; the vertical
// ticks are a subset of the rate ticks; each group's output acts at that tick):
//   attitude group, every att_loop_ratio * rate_loop_divisor ticks (tick 0 included):
//     the latched attitude must carry the sample stamp (else hal_panic, decision 0006 B);
//     angle = angle_mode.execute(sticks(t), state); rate_sp = attitude.execute(state, angle.q_sp, angle.yaw_rate_cmd).
//   vertical group, every vert_loop_ratio * rate_loop_divisor ticks (tick 0 included):
//     the latched vertical state must carry the sample stamp (else hal_panic, the same rule);
//     thrust = vertical.execute(state, z_ref(t)).thrust_request + thrust_disturbance(t).
//   every tick: step.filter(sample, due) (rate_group.hpp: on a rate-group tick the notches are updated first).
//   rate group, every rate_loop_divisor ticks (tick 0 included):
//     e = step.execute_bypass(rate_sp, torque_pulse(t), thrust): the rate loop in bypass on the chain output, request =
//     out.torque + torque_pulse(t), allocate, the rate loop's record_allocation, the rate group's DshotDiffuser
//     (decision 0017); then vertical.record_allocation(e.alloc, thrust), the collective passed to the mixer and its
//     allocation (decision 0020 section 1, "Anti-windup": recorded after each rate execution); the DShot is written.
// rate_sp and thrust are held between their group's executions. On a tick with no rate group nothing is written: the HAL
// latch holds the last command (hal_sim keeps it between writes). The composition does not act on the vertical loop's
// fault flag, as L5 does not act on the attitude law's: on an invalid state the law's own output is exactly T_h
// (marv/vertical/vertical_law.hpp). init panics naming the violated rule on an invalid attitude, rate, mixer, chain or
// vertical configuration or invalid scenario parameters.
//
// Script: segment k applies for t >= t_k until t_(k+1); before t_1 and with no segment the sticks are zero and z_ref is
// 0 m (L5's stick rule). Thrust disturbance: a constant added to the thrust request for t >= t0, N. Torque pulse: a
// constant torque on one axis (0 roll, 1 pitch, 2 yaw) for t0 <= t < t0 + duration, N*m, added to the rate loop's torque
// before the mixer.
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
#include <marv/mixer/mixer.hpp>
#include <marv/params/param.hpp>
#include <marv/prim/constants.hpp>
#include <marv/prim/vec.hpp>
#include <marv/rate/rate_loop.hpp>
#include <marv/rate_group/rate_group.hpp>
#include <marv/sched/rate_groups.hpp>
#include <marv/types/actuator.hpp>
#include <marv/types/attitude_state.hpp>
#include <marv/types/imu_sample.hpp>
#include <marv/types/time.hpp>
#include <marv/types/vertical_state.hpp>
#include <marv/vertical/config.hpp>
#include <marv/vertical/vertical_law.hpp>

namespace marv::composition {
namespace {

struct SegmentIds {
  ParamId t_us;
  ParamId roll;
  ParamId pitch;
  ParamId yaw;
  ParamId z_ref;
};

// The segment capacity K is the number of entries here, which is the number of l11_seg<k>_* entries of the register.
constexpr std::array kSegmentIds{
    SegmentIds{ParamId::l11_seg1_t_us, ParamId::l11_seg1_roll, ParamId::l11_seg1_pitch, ParamId::l11_seg1_yaw,
               ParamId::l11_seg1_z_ref_m},
    SegmentIds{ParamId::l11_seg2_t_us, ParamId::l11_seg2_roll, ParamId::l11_seg2_pitch, ParamId::l11_seg2_yaw,
               ParamId::l11_seg2_z_ref_m},
    SegmentIds{ParamId::l11_seg3_t_us, ParamId::l11_seg3_roll, ParamId::l11_seg3_pitch, ParamId::l11_seg3_yaw,
               ParamId::l11_seg3_z_ref_m},
    SegmentIds{ParamId::l11_seg4_t_us, ParamId::l11_seg4_roll, ParamId::l11_seg4_pitch, ParamId::l11_seg4_yaw,
               ParamId::l11_seg4_z_ref_m},
    SegmentIds{ParamId::l11_seg5_t_us, ParamId::l11_seg5_roll, ParamId::l11_seg5_pitch, ParamId::l11_seg5_yaw,
               ParamId::l11_seg5_z_ref_m},
    SegmentIds{ParamId::l11_seg6_t_us, ParamId::l11_seg6_roll, ParamId::l11_seg6_pitch, ParamId::l11_seg6_yaw,
               ParamId::l11_seg6_z_ref_m},
    SegmentIds{ParamId::l11_seg7_t_us, ParamId::l11_seg7_roll, ParamId::l11_seg7_pitch, ParamId::l11_seg7_yaw,
               ParamId::l11_seg7_z_ref_m},
    SegmentIds{ParamId::l11_seg8_t_us, ParamId::l11_seg8_roll, ParamId::l11_seg8_pitch, ParamId::l11_seg8_yaw,
               ParamId::l11_seg8_z_ref_m}};

constexpr std::size_t kSegments = kSegmentIds.size();

using Vec3f = prim::Vec3<float>;

struct Segment {
  std::int32_t t_us = 0;  // first stamp the segment applies to, us
  Vec3f stick{};          // shaped sticks [s_r, s_p, s_psi], each in [-1, 1]
  float z_ref_m = 0.0F;   // height reference, NED (z down), m
};

// The used segments are the first `count`.
struct Script {
  std::int32_t count = 0;
  std::array<Segment, kSegments> segment{};
};

struct ThrustDisturbance {
  float thrust_n = 0.0F;   // added to the thrust request, N
  std::int32_t t0_us = 0;  // first stamp it acts at, us
};

struct TorquePulse {
  std::int32_t axis = 0;    // 0 roll, 1 pitch, 2 yaw
  float torque_nm = 0.0F;   // N*m
  std::int32_t t0_us = 0;   // first stamp it acts at, us
  std::int32_t dur_us = 0;  // duration, us
};

enum class ScriptError : std::uint8_t {
  None,
  SegmentCount,
  SegmentTime,
  SegmentStick,
  SegmentHeight,
  ThrustValue,
  ThrustStart,
  TorqueAxis,
  TorqueValue,
  TorqueStart,
  TorqueDuration
};

// Group index order is the dispatch order (sched::RateGroups: lowest index first): attitude, vertical, rate.
enum Group : std::size_t { kAttitude, kVertical, kRate, kGroupCount };

sched::RateGroups<kGroupCount> g_groups;
attitude::AngleMode<float> g_angle;
attitude::AttitudeLaw<float> g_attitude;
vertical::VerticalLaw<float> g_vertical;
rate_group::RateGroupStep g_step;
Script g_script;
ThrustDisturbance g_thrust_disturbance;
TorquePulse g_torque_pulse;
Tick g_n = 0;
AttitudeState<float> g_attitude_latch{};
bool g_have_attitude_latch = false;
VerticalState<float> g_vertical_latch{};
bool g_have_vertical_latch = false;
Vec3f g_rate_setpoint;
float g_thrust = 0.0F;

[[nodiscard]] const char* message(ScriptError e) noexcept {
  switch (e) {
    case ScriptError::None:
      break;
    case ScriptError::SegmentCount:
      return "l11_vertical_scripted composition: l11_seg_count is not in 0..capacity";
    case ScriptError::SegmentTime:
      return "l11_vertical_scripted composition: segment stamps l11_seg<k>_t_us are not >= 0 and strictly increasing";
    case ScriptError::SegmentStick:
      return "l11_vertical_scripted composition: a segment stick l11_seg<k>_{roll,pitch,yaw} is not finite and in [-1, 1]";
    case ScriptError::SegmentHeight:
      return "l11_vertical_scripted composition: a segment height reference l11_seg<k>_z_ref_m is not finite";
    case ScriptError::ThrustValue:
      return "l11_vertical_scripted composition: l11_dist_thrust_n is not finite";
    case ScriptError::ThrustStart:
      return "l11_vertical_scripted composition: l11_dist_thrust_t_us is negative";
    case ScriptError::TorqueAxis:
      return "l11_vertical_scripted composition: l11_dist_torque_axis is not 0 (roll), 1 (pitch) or 2 (yaw)";
    case ScriptError::TorqueValue:
      return "l11_vertical_scripted composition: l11_dist_torque_nm is not finite";
    case ScriptError::TorqueStart:
      return "l11_vertical_scripted composition: l11_dist_torque_t_us is negative";
    case ScriptError::TorqueDuration:
      return "l11_vertical_scripted composition: l11_dist_torque_dur_us is negative";
  }
  return "l11_vertical_scripted composition: unknown scenario parameter error";
}

[[nodiscard]] const char* message(vertical::ConfigError e) noexcept {
  switch (e) {
    case vertical::ConfigError::None:
      break;
    case vertical::ConfigError::NonFinite:
      return "l11_vertical_scripted composition: vert_kp_z, vert_kp_v, vert_ki_v, the vertical period or "
             "vert_hover_thrust_n is not finite";
    case vertical::ConfigError::NegativeGain:
      return "l11_vertical_scripted composition: vert_kp_z, vert_kp_v or vert_ki_v is negative";
    case vertical::ConfigError::Period:
      return "l11_vertical_scripted composition: the vertical period is not positive";
    case vertical::ConfigError::HoverThrust:
      return "l11_vertical_scripted composition: vert_hover_thrust_n is not > 0";
    case vertical::ConfigError::IntegralWithoutProportional:
      return "l11_vertical_scripted composition: vert_ki_v is nonzero while vert_kp_v is 0";
  }
  return "l11_vertical_scripted composition: unknown vertical configuration error";
}

// The first violated rule, else None. Segments: 0 <= count <= K, stamps >= 0 and strictly increasing, sticks finite and
// in [-1, 1], z_ref finite. Disturbances: values finite, stamps >= 0, the pulse axis in 0..2 and its duration >= 0.
// Negated comparisons, so a NaN is rejected.
[[nodiscard]] ScriptError validate(const Script& s) noexcept {
  using std::abs;
  using std::isfinite;
  if (s.count < 0 || static_cast<std::size_t>(s.count) > kSegments) {
    return ScriptError::SegmentCount;
  }
  for (std::size_t k = 0; k < static_cast<std::size_t>(s.count); ++k) {
    if (s.segment[k].t_us < 0 || (k > 0 && !(s.segment[k].t_us > s.segment[k - 1].t_us))) {
      return ScriptError::SegmentTime;
    }
    for (std::size_t a = 0; a < prim::kSpatialDim; ++a) {
      const float v = s.segment[k].stick[a];
      if (!isfinite(v) || !(abs(v) <= 1.0F)) {
        return ScriptError::SegmentStick;
      }
    }
    if (!isfinite(s.segment[k].z_ref_m)) {
      return ScriptError::SegmentHeight;
    }
  }
  return ScriptError::None;
}

[[nodiscard]] ScriptError validate(const ThrustDisturbance& d) noexcept {
  if (!std::isfinite(d.thrust_n)) {
    return ScriptError::ThrustValue;
  }
  if (!(d.t0_us >= 0)) {
    return ScriptError::ThrustStart;
  }
  return ScriptError::None;
}

[[nodiscard]] ScriptError validate(const TorquePulse& p) noexcept {
  if (p.axis < 0 || static_cast<std::size_t>(p.axis) >= prim::kSpatialDim) {
    return ScriptError::TorqueAxis;
  }
  if (!std::isfinite(p.torque_nm)) {
    return ScriptError::TorqueValue;
  }
  if (!(p.t0_us >= 0)) {
    return ScriptError::TorqueStart;
  }
  if (!(p.dur_us >= 0)) {
    return ScriptError::TorqueDuration;
  }
  return ScriptError::None;
}

[[nodiscard]] Script load_script() noexcept {
  Script s;
  s.count = param_value<ParamId::l11_seg_count>();
  for (std::size_t k = 0; k < kSegments; ++k) {
    s.segment[k].t_us = param_get(kSegmentIds[k].t_us).value.i32;
    s.segment[k].stick = Vec3f(param_get(kSegmentIds[k].roll).value.f32, param_get(kSegmentIds[k].pitch).value.f32,
                               param_get(kSegmentIds[k].yaw).value.f32);
    s.segment[k].z_ref_m = param_get(kSegmentIds[k].z_ref).value.f32;
  }
  return s;
}

[[nodiscard]] ThrustDisturbance load_thrust_disturbance() noexcept {
  ThrustDisturbance d;
  d.thrust_n = param_value<ParamId::l11_dist_thrust_n>();
  d.t0_us = param_value<ParamId::l11_dist_thrust_t_us>();
  return d;
}

[[nodiscard]] TorquePulse load_torque_pulse() noexcept {
  TorquePulse p;
  p.axis = param_value<ParamId::l11_dist_torque_axis>();
  p.torque_nm = param_value<ParamId::l11_dist_torque_nm>();
  p.t0_us = param_value<ParamId::l11_dist_torque_t_us>();
  p.dur_us = param_value<ParamId::l11_dist_torque_dur_us>();
  return p;
}

void require_valid_script(ScriptError e) noexcept {
  if (e != ScriptError::None) {
    hal_panic(message(e));
  }
}

// The vertical loop's configuration: the product gains vert_kp_z, vert_kp_v, vert_ki_v, the hover feed-forward
// vert_hover_thrust_n and the period T_v = vert_loop_ratio * rate_loop_divisor * tick_period_num_us / tick_period_den
// microseconds, in seconds (the attitude period's rule, attitude::from_params). hal_panic naming the violated rule
// unless vertical::validate is None.
[[nodiscard]] vertical::VerticalConfig<float> load_vertical_config() noexcept {
  vertical::VerticalConfig<float> c;
  c.kp_z = param_value<ParamId::vert_kp_z>();
  c.kp_v = param_value<ParamId::vert_kp_v>();
  c.ki_v = param_value<ParamId::vert_ki_v>();
  const float ticks = static_cast<float>(param_value<ParamId::vert_loop_ratio>()) *
                      static_cast<float>(param_value<ParamId::rate_loop_divisor>());
  const float period_us = ticks * static_cast<float>(param_value<ParamId::tick_period_num_us>()) /
                          static_cast<float>(param_value<ParamId::tick_period_den>());
  c.period = period_us / static_cast<float>(prim::kMicrosecondsPerSecond);
  c.hover_thrust = param_value<ParamId::vert_hover_thrust_n>();
  const vertical::ConfigError e = vertical::validate(c);
  if (e != vertical::ConfigError::None) {
    hal_panic(message(e));
  }
  return c;
}

// The script's segment at stamp t_us: the last segment with t_k <= t_us, else the zero segment. pre: validate(s) ==
// None.
[[nodiscard]] Segment segment_at(const Script& s, TimeUs t_us) noexcept {
  Segment active;
  for (std::size_t k = 0; k < static_cast<std::size_t>(s.count) && k < kSegments; ++k) {
    if (t_us >= static_cast<TimeUs>(s.segment[k].t_us)) {
      active = s.segment[k];
    }
  }
  return active;
}

// The thrust disturbance at stamp t_us: thrust_n from t0, 0 before.
[[nodiscard]] float thrust_disturbance(const ThrustDisturbance& d, TimeUs t_us) noexcept {
  return t_us >= static_cast<TimeUs>(d.t0_us) ? d.thrust_n : 0.0F;
}

// The torque pulse as a torque request addition: torque_nm on its axis for t0 <= t_us < t0 + dur, 0 otherwise and on
// the other axes. pre: validate(p) == None.
[[nodiscard]] Vec3f torque_pulse(const TorquePulse& p, TimeUs t_us) noexcept {
  Vec3f tau;
  const auto t0_us = static_cast<TimeUs>(p.t0_us);
  if (t_us >= t0_us && t_us - t0_us < static_cast<TimeUs>(p.dur_us)) {
    tau[static_cast<std::size_t>(p.axis)] = p.torque_nm;
  }
  return tau;
}

[[nodiscard]] bool fired(std::uint32_t due, Group g) noexcept { return (due & (std::uint32_t{1} << g)) != 0; }

}  // namespace

void attitude_input(const AttitudeState<float>& a) noexcept {
  g_attitude_latch = a;
  g_have_attitude_latch = true;
}

void vertical_input(const VerticalState<float>& v) noexcept {
  g_vertical_latch = v;
  g_have_vertical_latch = true;
}

void init() noexcept {
  const rate::RateConfig<float> rate_cfg = rate::load_config();
  const mixer::MixerConfig<float> mixer_cfg = mixer::load_config();
  const attitude::AttitudeConfig<float> attitude_cfg = attitude::load_config();
  g_angle.init(attitude_cfg);
  g_attitude.init(attitude_cfg);

  // The attitude divisor is rate_loop_divisor * att_loop_ratio (decision 0006 E) and the vertical divisor
  // rate_loop_divisor * vert_loop_ratio (decision 0020 section 1), so both groups' ticks are subsets of the rate ticks.
  // The products are formed in 64 bits and must fit the scheduler's 32.
  const std::int32_t divisor = param_value<ParamId::rate_loop_divisor>();
  const std::int32_t attitude_ratio = param_value<ParamId::att_loop_ratio>();
  const std::int32_t vertical_ratio = param_value<ParamId::vert_loop_ratio>();
  if (divisor < 1 || attitude_ratio < 1 || vertical_ratio < 1) {
    hal_panic("l11_vertical_scripted composition: rate_loop_divisor, att_loop_ratio or vert_loop_ratio is below 1");
  }
  const std::uint64_t attitude_divisor = static_cast<std::uint64_t>(divisor) * static_cast<std::uint64_t>(attitude_ratio);
  const std::uint64_t vertical_divisor = static_cast<std::uint64_t>(divisor) * static_cast<std::uint64_t>(vertical_ratio);
  if (attitude_divisor > std::numeric_limits<std::uint32_t>::max() ||
      vertical_divisor > std::numeric_limits<std::uint32_t>::max() ||
      !g_groups.init({static_cast<std::uint32_t>(attitude_divisor), static_cast<std::uint32_t>(vertical_divisor),
                      static_cast<std::uint32_t>(divisor)})) {
    hal_panic("l11_vertical_scripted composition: rate_loop_divisor * att_loop_ratio or rate_loop_divisor * "
              "vert_loop_ratio is out of range");
  }
  g_step.init(rate_cfg, mixer_cfg, rate_group::load_chain_config());
  g_vertical.init(load_vertical_config(), mixer_cfg);

  g_script = load_script();
  require_valid_script(validate(g_script));
  g_thrust_disturbance = load_thrust_disturbance();
  require_valid_script(validate(g_thrust_disturbance));
  g_torque_pulse = load_torque_pulse();
  require_valid_script(validate(g_torque_pulse));
  g_rate_setpoint = Vec3f();
  g_thrust = 0.0F;
  g_have_attitude_latch = false;
  g_have_vertical_latch = false;
  g_n = 0;
}

void tick(const ImuSample& s) noexcept {
  if (hal_time_us() != s.t_us) {
    hal_panic("l11_vertical_scripted composition: hal_time_us differs from the sample stamp");
  }
  const std::uint32_t due = g_groups.due(g_n);
  ++g_n;
  if (fired(due, kAttitude)) {
    if (!g_have_attitude_latch || g_attitude_latch.t_us != s.t_us) {
      hal_panic("l11_vertical_scripted composition: the latched attitude stamp differs from the sample stamp");
    }
    const Vec3f stick = segment_at(g_script, s.t_us).stick;
    const attitude::AngleOutput<float> angle =
        g_angle.execute(attitude::AngleSticks<float>{stick[0], stick[1], stick[2]}, g_attitude_latch);
    g_rate_setpoint = g_attitude.execute(g_attitude_latch, angle.q_sp, angle.yaw_rate_cmd).rate_setpoint;
  }
  if (fired(due, kVertical)) {
    if (!g_have_vertical_latch || g_vertical_latch.t_us != s.t_us) {
      hal_panic("l11_vertical_scripted composition: the latched vertical stamp differs from the sample stamp");
    }
    const float z_ref = segment_at(g_script, s.t_us).z_ref_m;
    g_thrust = g_vertical.execute(g_vertical_latch, z_ref).thrust_request +
               thrust_disturbance(g_thrust_disturbance, s.t_us);
  }
  const bool rate_due = fired(due, kRate);
  g_step.filter(s, rate_due);
  if (!rate_due) {
    return;
  }
  const rate_group::Execution e =
      g_step.execute_bypass(g_rate_setpoint, torque_pulse(g_torque_pulse, s.t_us), g_thrust);
  g_vertical.record_allocation(e.alloc, g_thrust);
  ActuatorOutput<kMotors, kServos> o{};
  o.motor = e.dshot;
  hal_actuators_write(o);
}

}  // namespace marv::composition
