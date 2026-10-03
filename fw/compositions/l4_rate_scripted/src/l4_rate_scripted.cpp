// The L4 test composition: the gyro chain, the rate loop and the mixer through the shared rate-group step
// (marv::rate_group), flown with scripted inputs. Scenario parameters (fw/compositions/l4_rate_scripted/params) give
// the collective thrust request, a piecewise-constant body-rate setpoint script and one torque chirp; nothing here is a
// vehicle number. The IMU sample is the chain's input (in the Gazebo runs it is the plant's truth body rate).
//
// Every tick: step.filter(sample, due) (rate_group.hpp: on a due tick the notches are updated first). On the ticks the
// rate group is due (every rate_loop_divisor ticks, tick 0 included): step.execute(script(t), chirp(t) on the chirp
// axis, thrust), the rate loop on the chain output, allocate, record_allocation, thrust_to_dshot; the DShot is written.
// On the other ticks nothing is written: the HAL latch holds the last command (hal_sim keeps it between writes).
// init panics naming the violated rule on an invalid rate, mixer or chain configuration or invalid scenario parameters.
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include <marv/composition.hpp>
#include <marv/hal/hal.hpp>
#include <marv/hal/tick.hpp>
#include <marv/l4_script.hpp>
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

struct SegmentIds {
  ParamId t_us;
  ParamId roll;
  ParamId pitch;
  ParamId yaw;
};

// The segment capacity K is the number of entries here, which is the number of l4_seg<k>_* entries of the register.
constexpr std::array kSegmentIds{
    SegmentIds{ParamId::l4_seg1_t_us, ParamId::l4_seg1_roll, ParamId::l4_seg1_pitch, ParamId::l4_seg1_yaw},
    SegmentIds{ParamId::l4_seg2_t_us, ParamId::l4_seg2_roll, ParamId::l4_seg2_pitch, ParamId::l4_seg2_yaw},
    SegmentIds{ParamId::l4_seg3_t_us, ParamId::l4_seg3_roll, ParamId::l4_seg3_pitch, ParamId::l4_seg3_yaw},
    SegmentIds{ParamId::l4_seg4_t_us, ParamId::l4_seg4_roll, ParamId::l4_seg4_pitch, ParamId::l4_seg4_yaw},
    SegmentIds{ParamId::l4_seg5_t_us, ParamId::l4_seg5_roll, ParamId::l4_seg5_pitch, ParamId::l4_seg5_yaw},
    SegmentIds{ParamId::l4_seg6_t_us, ParamId::l4_seg6_roll, ParamId::l4_seg6_pitch, ParamId::l4_seg6_yaw},
    SegmentIds{ParamId::l4_seg7_t_us, ParamId::l4_seg7_roll, ParamId::l4_seg7_pitch, ParamId::l4_seg7_yaw},
    SegmentIds{ParamId::l4_seg8_t_us, ParamId::l4_seg8_roll, ParamId::l4_seg8_pitch, ParamId::l4_seg8_yaw}};

constexpr std::size_t kSegments = kSegmentIds.size();

using Script = SetpointScript<float, kSegments>;
using Vec3f = prim::Vec3<float>;

enum Group : std::size_t { kRate, kGroupCount };

sched::RateGroups<kGroupCount> g_groups;
rate_group::RateGroupStep g_step;
Script g_script;
Chirp<float> g_chirp;
float g_thrust = 0.0F;
Tick g_n = 0;

[[nodiscard]] const char* message(ScriptError e) noexcept {
  switch (e) {
    case ScriptError::None:
      break;
    case ScriptError::SegmentCount:
      return "l4_rate_scripted composition: l4_seg_count is not in 0..capacity";
    case ScriptError::SegmentTime:
      return "l4_rate_scripted composition: segment stamps l4_seg<k>_t_us are not >= 0 and strictly increasing";
    case ScriptError::SegmentRate:
      return "l4_rate_scripted composition: a segment rate l4_seg<k>_{roll,pitch,yaw} is not finite";
    case ScriptError::ChirpAxis:
      return "l4_rate_scripted composition: l4_chirp_axis is not 0 (none), 1 (roll), 2 (pitch) or 3 (yaw)";
    case ScriptError::ChirpAmplitude:
      return "l4_rate_scripted composition: l4_chirp_amp_nm is not finite and >= 0";
    case ScriptError::ChirpBand:
      return "l4_rate_scripted composition: l4_chirp_w_lo, l4_chirp_w_hi do not satisfy 0 < w_lo < w_hi";
    case ScriptError::ChirpDuration:
      return "l4_rate_scripted composition: l4_chirp_dur_us is not > 0";
    case ScriptError::ChirpStart:
      return "l4_rate_scripted composition: l4_chirp_t0_us is negative";
  }
  return "l4_rate_scripted composition: unknown scenario parameter error";
}

[[nodiscard]] Script load_script() noexcept {
  Script s;
  s.count = param_value<ParamId::l4_seg_count>();
  for (std::size_t k = 0; k < kSegments; ++k) {
    s.segment[k].t_us = param_get(kSegmentIds[k].t_us).value.i32;
    s.segment[k].rate = Vec3f(param_get(kSegmentIds[k].roll).value.f32, param_get(kSegmentIds[k].pitch).value.f32,
                              param_get(kSegmentIds[k].yaw).value.f32);
  }
  return s;
}

[[nodiscard]] Chirp<float> load_chirp() noexcept {
  Chirp<float> c;
  c.axis = static_cast<ChirpAxis>(param_value<ParamId::l4_chirp_axis>());
  c.amp = param_value<ParamId::l4_chirp_amp_nm>();
  c.w_lo = param_value<ParamId::l4_chirp_w_lo>();
  c.w_hi = param_value<ParamId::l4_chirp_w_hi>();
  c.t0_us = param_value<ParamId::l4_chirp_t0_us>();
  c.dur_us = param_value<ParamId::l4_chirp_dur_us>();
  return c;
}

void require_valid_script(ScriptError e) noexcept {
  if (e != ScriptError::None) {
    hal_panic(message(e));
  }
}

[[nodiscard]] bool fired(std::uint32_t due, Group g) noexcept { return (due & (std::uint32_t{1} << g)) != 0; }

}  // namespace

void init() noexcept {
  const rate::RateConfig<float> rate_cfg = rate::load_config();
  const mixer::MixerConfig<float> mixer_cfg = mixer::load_config();

  const std::int32_t divisor = param_value<ParamId::rate_loop_divisor>();
  if (divisor < 1 || !g_groups.init({static_cast<std::uint32_t>(divisor)})) {
    hal_panic("l4_rate_scripted composition: rate_loop_divisor is below 1");
  }
  g_step.init(rate_cfg, mixer_cfg, rate_group::load_chain_config());

  g_thrust = param_value<ParamId::l4_thrust_n>();
  if (!std::isfinite(g_thrust) || !(g_thrust >= 0.0F)) {
    hal_panic("l4_rate_scripted composition: l4_thrust_n is not finite and >= 0");
  }
  g_script = load_script();
  require_valid_script(validate(g_script));
  g_chirp = load_chirp();
  require_valid_script(validate(g_chirp));
  g_n = 0;
}

void tick(const ImuSample& s) noexcept {
  if (hal_time_us() != s.t_us) {
    hal_panic("l4_rate_scripted composition: hal_time_us differs from the sample stamp");
  }
  const std::uint32_t due = g_groups.due(g_n);
  ++g_n;
  const bool rate_due = fired(due, kRate);
  g_step.filter(s, rate_due);
  if (!rate_due) {
    return;
  }
  const rate_group::Execution e =
      g_step.execute(setpoint_at(g_script, s.t_us), chirp_torque(g_chirp, s.t_us), g_thrust);
  ActuatorOutput<kMotors, kServos> o{};
  o.motor = e.dshot;
  hal_actuators_write(o);
}

}  // namespace marv::composition
