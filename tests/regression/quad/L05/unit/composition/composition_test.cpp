// The l5_attitude_scripted composition through the SIL entry points (marv_sil.h, marv_truth.h): per tick j the test calls
// marv_truth_state_set(tick = j) and then marv_sil_tick(j, 1, ...), as the plugin does.
//
// Reference. The test runs an independent tick loop in this executable, built from the same modules (angle mode, the
// attitude law, the rate loop in bypass, the mixer) but scheduled by plain modulo arithmetic (n % (rate_loop_divisor *
// att_loop_ratio), n % rate_loop_divisor) in the order decision 0006 E names: the attitude group, then the rate group,
// in the same tick; the rate setpoint held between attitude executions; the chirp added to the rate setpoint before
// execute_bypass (decision 0006 F); the yaw disturbance added to the torque request before the mixer. The composition's
// DShot output must equal it bit for bit on every tick, for the product divisors and for overridden ones. The reference
// reads its configuration from a copy of the default table with the same overrides applied, so both sides run the same
// parameters. The rows that hold a command (non-rate ticks) are the held command of the last rate tick.
//
// Controls. Every check that compares with the reference has a control reference that differs in exactly the behaviour
// under test (attitude after the rate group, a wrong divisor, the chirp as a torque after the rate loop, no held
// setpoint) and must not reproduce the composition's output.
//
// marv_sil_init succeeds once per process, so every test that initialises runs in a forked child.
#include <gtest/gtest.h>

#include <marv_sil.h>
#include <marv_truth.h>

#include <sys/prctl.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <span>
#include <string>
#include <vector>

#include <marv/attitude/angle_mode.hpp>
#include <marv/attitude/attitude_law.hpp>
#include <marv/attitude/config.hpp>
#include <marv/composition.hpp>
#include <marv/l5_script.hpp>
#include <marv/mixer/mixer.hpp>
#include <marv/params/param.hpp>
#include <marv/params/param_ids.hpp>
#include <marv/prim/constants.hpp>
#include <marv/prim/quat.hpp>
#include <marv/rate/rate_loop.hpp>
#include <marv/rate_group/rate_group.hpp>
#include <marv/types/actuator.hpp>
#include <marv/types/attitude_state.hpp>
#include <marv/types/imu_sample.hpp>

namespace {

using namespace marv;
using composition::Chirp;
using composition::ChirpAxis;
using composition::Disturbance;
using composition::StickScript;

constexpr std::size_t kMotors = kQuadXMotors;
using Dshots = std::array<std::uint16_t, kMotors>;
using Vec3f = prim::Vec3<float>;
constexpr std::uint32_t kGyroValid = 1u << MARV_IMU_GYRO_VALID;
constexpr std::uint32_t kAttitudeValid = 1u << MARV_TRUTH_ATTITUDE_VALID;
constexpr std::size_t kSegments = 8;  // the capacity of the register (l5_seg1..l5_seg8)
using Script = StickScript<float, kSegments>;

bool report_failures_on_stderr() {
  const ::testing::TestResult* r = ::testing::UnitTest::GetInstance()->current_test_info()->result();
  bool any = false;
  for (int i = 0; i < r->total_part_count(); ++i) {
    const ::testing::TestPartResult& part = r->GetTestPartResult(i);
    if (part.failed()) {
      any = true;
      std::fprintf(stderr, "%s:%d: %s\n", part.file_name(), part.line_number(), part.message());
    }
  }
  return any;
}

template <class Body>
void in_child(Body body) {
  EXPECT_EXIT(
      {
        body();
        std::exit(report_failures_on_stderr() ? 1 : 0);
      },
      ::testing::ExitedWithCode(0), "");
}

// ---- overrides: one list for the SIL and for the reference's table ------------------------------------------------

struct Overrides {
  std::vector<marv_sil_param_override> v;
  void f32(ParamId id, float x) {
    marv_sil_param_override o{};
    o.id = static_cast<std::uint32_t>(id);
    o.type = MARV_PARAM_F32;
    o.f32 = x;
    v.push_back(o);
  }
  void i32(ParamId id, std::int32_t x) {
    marv_sil_param_override o{};
    o.id = static_cast<std::uint32_t>(id);
    o.type = MARV_PARAM_I32;
    o.i32 = x;
    v.push_back(o);
  }
};

std::array<ParamRecord, kParamCount> g_table;

// params_init on a copy of the default table with the overrides applied to the values: what the SIL's merged table holds
// as far as the modules read it.
bool init_params_with(const Overrides& ov) {
  const std::span<const ParamRecord, kParamCount> d = param_defaults();
  std::copy(d.begin(), d.end(), g_table.begin());
  for (const marv_sil_param_override& o : ov.v) {
    ParamRecord& rec = g_table[o.id];
    rec.value = o.type == MARV_PARAM_F32 ? ParamValue{ParamType::F32, o.f32, 0} : ParamValue{ParamType::I32, 0.0F, o.i32};
  }
  return params_init(std::span<const ParamRecord, kParamCount>(g_table));
}

struct SegmentIds {
  ParamId t_us;
  ParamId roll;
  ParamId pitch;
  ParamId yaw;
};

constexpr std::array kSegmentIds{
    SegmentIds{ParamId::l5_seg1_t_us, ParamId::l5_seg1_roll, ParamId::l5_seg1_pitch, ParamId::l5_seg1_yaw},
    SegmentIds{ParamId::l5_seg2_t_us, ParamId::l5_seg2_roll, ParamId::l5_seg2_pitch, ParamId::l5_seg2_yaw},
    SegmentIds{ParamId::l5_seg3_t_us, ParamId::l5_seg3_roll, ParamId::l5_seg3_pitch, ParamId::l5_seg3_yaw},
    SegmentIds{ParamId::l5_seg4_t_us, ParamId::l5_seg4_roll, ParamId::l5_seg4_pitch, ParamId::l5_seg4_yaw},
    SegmentIds{ParamId::l5_seg5_t_us, ParamId::l5_seg5_roll, ParamId::l5_seg5_pitch, ParamId::l5_seg5_yaw},
    SegmentIds{ParamId::l5_seg6_t_us, ParamId::l5_seg6_roll, ParamId::l5_seg6_pitch, ParamId::l5_seg6_yaw},
    SegmentIds{ParamId::l5_seg7_t_us, ParamId::l5_seg7_roll, ParamId::l5_seg7_pitch, ParamId::l5_seg7_yaw},
    SegmentIds{ParamId::l5_seg8_t_us, ParamId::l5_seg8_roll, ParamId::l5_seg8_pitch, ParamId::l5_seg8_yaw}};

// A scenario: the composition's scripted inputs as structs, turned into overrides; divisor and ratio are overridden
// when positive.
struct Scenario {
  float thrust = 0.0F;
  Script script;
  Chirp<float> chirp;
  Disturbance<float> dist;
  std::int32_t divisor = 0;
  std::int32_t ratio = 0;

  [[nodiscard]] Overrides overrides() const {
    Overrides ov;
    ov.f32(ParamId::l5_thrust_n, thrust);
    ov.i32(ParamId::l5_seg_count, script.count);
    for (std::size_t k = 0; k < kSegments; ++k) {
      if (static_cast<std::int32_t>(k) < script.count) {
        ov.i32(kSegmentIds[k].t_us, script.segment[k].t_us);
        ov.f32(kSegmentIds[k].roll, script.segment[k].stick[0]);
        ov.f32(kSegmentIds[k].pitch, script.segment[k].stick[1]);
        ov.f32(kSegmentIds[k].yaw, script.segment[k].stick[2]);
      }
    }
    ov.i32(ParamId::l5_chirp_axis, static_cast<std::int32_t>(chirp.axis));
    ov.f32(ParamId::l5_chirp_amp_rad_s, chirp.amp);
    ov.f32(ParamId::l5_chirp_w_lo, chirp.w_lo);
    ov.f32(ParamId::l5_chirp_w_hi, chirp.w_hi);
    ov.i32(ParamId::l5_chirp_t0_us, chirp.t0_us);
    ov.i32(ParamId::l5_chirp_dur_us, chirp.dur_us);
    ov.f32(ParamId::l5_dist_yaw_nm, dist.yaw_nm);
    ov.i32(ParamId::l5_dist_t0_us, dist.t0_us);
    if (divisor > 0) {
      ov.i32(ParamId::rate_loop_divisor, divisor);
    }
    if (ratio > 0) {
      ov.i32(ParamId::att_loop_ratio, ratio);
    }
    return ov;
  }
};

// ---- the configuration of the run ---------------------------------------------------------------------------------

struct Ref {
  mixer::MixerConfig<float> mixer;
  rate::RateConfig<float> rate;
  gyro_chain::GyroChainConfig<float> chain;
  attitude::AttitudeConfig<float> attitude;
  float hover = 0.0F;
  float p_torque_yaw = 0.0F;  // kp_yaw * rate_max_yaw: the P torque of a full-rate yaw error
  std::uint32_t tick_num_us = 0;
  std::uint32_t tick_den = 0;
  std::uint32_t divisor = 0;  // rate_loop_divisor
  std::uint32_t ratio = 0;    // att_loop_ratio
};

float default_f32(ParamId id) { return param_defaults()[static_cast<std::size_t>(id)].value.f32; }
std::int32_t default_i32(ParamId id) { return param_defaults()[static_cast<std::size_t>(id)].value.i32; }

// The scalars of the default table, read without params_init (which may run once per process): enough to build a
// scenario before the overrides are known. The configurations are not filled.
Ref defaults_ref() {
  Ref r;
  r.hover = default_f32(ParamId::mass) * static_cast<float>(prim::kWgs84GammaE);
  r.p_torque_yaw = default_f32(ParamId::rate_kp_yaw) * default_f32(ParamId::rate_max_yaw);
  r.tick_num_us = static_cast<std::uint32_t>(default_i32(ParamId::tick_period_num_us));
  r.tick_den = static_cast<std::uint32_t>(default_i32(ParamId::tick_period_den));
  r.divisor = static_cast<std::uint32_t>(default_i32(ParamId::rate_loop_divisor));
  r.ratio = static_cast<std::uint32_t>(default_i32(ParamId::att_loop_ratio));
  return r;
}

Ref load_ref(const Overrides& ov) {
  if (!init_params_with(ov)) {
    std::fprintf(stderr, "params_init failed\n");
    std::exit(1);
  }
  Ref r;
  r.mixer = mixer::load_config();
  r.rate = rate::load_config();
  r.chain = rate_group::load_chain_config();
  r.attitude = attitude::load_config();
  // The hover thrust is m g with m the card mass and g the WGS 84 equatorial normal gravity of constants.hpp; Gazebo runs
  // use g at the site, the test only needs a valid collective.
  r.hover = param_value<ParamId::mass>() * static_cast<float>(prim::kWgs84GammaE);
  r.p_torque_yaw = param_value<ParamId::rate_kp_yaw>() * param_value<ParamId::rate_max_yaw>();
  r.tick_num_us = static_cast<std::uint32_t>(param_value<ParamId::tick_period_num_us>());
  r.tick_den = static_cast<std::uint32_t>(param_value<ParamId::tick_period_den>());
  r.divisor = static_cast<std::uint32_t>(param_value<ParamId::rate_loop_divisor>());
  r.ratio = static_cast<std::uint32_t>(param_value<ParamId::att_loop_ratio>());
  return r;
}

Dshots dshot_of(const Ref& r, float thrust, const Vec3f& torque) {
  const auto f = mixer::allocate(r.mixer, mixer::Request<float>{thrust, torque}).f;
  const auto d = mixer::thrust_to_dshot(r.mixer, f);
  Dshots out{};
  for (std::size_t i = 0; i < kMotors; ++i) {
    out[i] = d[i].raw();
  }
  return out;
}

TimeUs stamp(const Ref& r, std::uint64_t n) {
  return static_cast<TimeUs>(static_cast<unsigned __int128>(n) * r.tick_num_us / r.tick_den);
}

// The composition's DShot per tick from init for a mixer request {thrust, torque(n)} (decision 0017): mixer::DshotDiffuser
// on its allocation at every rate tick n (n % rate_loop_divisor == 0), the command held in between.
template <class Torque>
std::vector<Dshots> diffused(const Ref& r, float thrust, std::uint32_t k, Torque torque) {
  mixer::DshotDiffuser<float> diffuser;
  diffuser.init(r.mixer);
  std::vector<Dshots> rows(k);
  Dshots held{};
  for (std::uint32_t n = 0; n < k; ++n) {
    if (n % r.divisor == 0) {
      const auto d = diffuser.apply(mixer::allocate(r.mixer, mixer::Request<float>{thrust, torque(n)}).f);
      for (std::size_t i = 0; i < kMotors; ++i) {
        held[i] = d[i].raw();
      }
    }
    rows[n] = held;
  }
  return rows;
}

// The pure-thrust allocation of `thrust` as the composition writes it, per tick from init.
std::vector<Dshots> pure_thrust_rows(const Ref& r, float thrust, std::uint32_t k) {
  return diffused(r, thrust, k, [](std::uint32_t) { return Vec3f(); });
}

// Control: some row differs from the constant stateless command (thrust_to_dshot at every write).
bool differs_from_stateless(const std::vector<Dshots>& rows, const Dshots& stateless) {
  return std::any_of(rows.begin(), rows.end(), [&](const Dshots& d) { return d != stateless; });
}

// ---- the truth trajectory and the gyro ----------------------------------------------------------------------------

// scenario test values: a tilt about the unit axis (0.6, 0, 0.8) of amplitude 0.2 rad at 0.01 rad/tick (a slow sweep over
// the run), with body rates that follow it; amplitude 0 is the level, still vehicle
struct Trajectory {
  float amp = 0.0F;
  static constexpr float kStep = 0.01F;
  static constexpr float kAxisX = 0.6F;
  static constexpr float kAxisZ = 0.8F;

  [[nodiscard]] AttitudeState<float> at(const Ref& r, std::uint64_t n) const {
    if (amp == 0.0F) {
      return AttitudeState<float>{stamp(r, n), prim::Quat<float>::identity(), Vec3f(), true};
    }
    const float angle = amp * std::sin(kStep * static_cast<float>(n));
    const float rate = amp * kStep * std::cos(kStep * static_cast<float>(n)) * static_cast<float>(r.tick_den) /
                       static_cast<float>(r.tick_num_us) * static_cast<float>(prim::kMicrosecondsPerSecond);
    const Vec3f axis(kAxisX, 0.0F, kAxisZ);
    return AttitudeState<float>{stamp(r, n), prim::Quat<float>::from_axis_angle(axis, angle), axis * rate, true};
  }
};

marv_truth_state truth_of(const AttitudeState<float>& a, std::uint64_t tick) {
  marv_truth_state s{};
  s.struct_size = sizeof(marv_truth_state);
  s.flags = a.valid ? kAttitudeValid : 0U;
  s.tick = tick;
  s.q_wxyz[MARV_TRUTH_Q_W] = a.q.w;
  s.q_wxyz[MARV_TRUTH_Q_X] = a.q.x;
  s.q_wxyz[MARV_TRUTH_Q_Y] = a.q.y;
  s.q_wxyz[MARV_TRUTH_Q_Z] = a.q.z;
  s.omega_frd_rad_s = {a.omega_frd[0], a.omega_frd[1], a.omega_frd[2]};
  return s;
}

marv_imu_meas gyro_of(const AttitudeState<float>& a) {
  marv_imu_meas m{};
  m.gyro_rad_s = {a.omega_frd[0], a.omega_frd[1], a.omega_frd[2]};
  m.flags = kGyroValid;
  return m;
}

// ---- the SIL plumbing ---------------------------------------------------------------------------------------------

marv_sil_status init_with(const Ref& r, const Overrides& ov) {
  marv_sil_config c{};
  c.struct_size = sizeof(marv_sil_config);
  c.imu_meas_size = sizeof(marv_imu_meas);
  c.override_size = sizeof(marv_sil_param_override);
  c.tick_period_num_us = r.tick_num_us;
  c.tick_period_den = r.tick_den;
  c.n_overrides = static_cast<std::uint32_t>(ov.v.size());
  c.overrides = ov.v.empty() ? nullptr : ov.v.data();
  c.param_schema_hash = kParamSchemaHash;
  return marv_sil_init(&c);
}

// One tick: marv_sil_tick(tick, 1, ...) with the gyro sample; the DShot row of the tick.
Dshots sil_tick(std::uint64_t tick, const marv_imu_meas& imu) {
  std::uint64_t t = 0;
  Dshots d{};
  marv_sil_out out{};
  out.struct_size = sizeof(marv_sil_out);
  out.capacity_ticks = 1;
  out.t_us = &t;
  out.dshot = d.data();
  out.servo_us = nullptr;
  EXPECT_EQ(marv_sil_tick(tick, 1, &imu, &out), MARV_SIL_OK);
  return d;
}

// Ticks [0, k): the truth state of each tick, then the tick.
std::vector<Dshots> run_sil(const Ref& r, const Trajectory& tr, std::uint32_t k) {
  std::vector<Dshots> rows;
  for (std::uint32_t n = 0; n < k; ++n) {
    const AttitudeState<float> a = tr.at(r, n);
    const marv_truth_state s = truth_of(a, n);
    EXPECT_EQ(marv_truth_state_set(&s), MARV_SIL_OK);
    rows.push_back(sil_tick(n, gyro_of(a)));
  }
  return rows;
}

// ---- the reference loop -------------------------------------------------------------------------------------------

// What a control reference changes: nothing for the reference itself.
struct Variant {
  bool rate_before_attitude = false;    // the rate group runs before the attitude group in a tick
  bool chirp_as_torque = false;         // the chirp is added to the torque request instead of the rate setpoint
  bool setpoint_not_held = false;       // the rate setpoint is zero on ticks without an attitude execution
  std::uint32_t attitude_divisor = 0;   // overrides rate_loop_divisor * att_loop_ratio when nonzero
};

std::vector<Dshots> reference(const Ref& r, const Scenario& sc, const Trajectory& tr, std::uint32_t k, Variant v = {}) {
  attitude::AngleMode<float> angle;
  attitude::AttitudeLaw<float> law;
  rate_group::RateGroupStep step;
  const std::uint32_t attitude_divisor = v.attitude_divisor != 0 ? v.attitude_divisor : r.divisor * r.ratio;
  attitude::AttitudeConfig<float> acfg = r.attitude;
  if (v.attitude_divisor != 0) {
    // a control with another attitude divisor keeps its own period consistent, so the law's spacing check stays quiet
    acfg.period = static_cast<float>(attitude_divisor) * static_cast<float>(r.tick_num_us) /
                  static_cast<float>(r.tick_den) / static_cast<float>(prim::kMicrosecondsPerSecond);
  }
  angle.init(acfg);
  law.init(acfg);
  step.init(r.rate, r.mixer, r.chain);
  Vec3f setpoint;
  Dshots held{};
  std::vector<Dshots> rows;
  for (std::uint32_t n = 0; n < k; ++n) {
    const AttitudeState<float> a = tr.at(r, n);
    ImuSample imu{};
    imu.t_us = a.t_us;
    imu.gyro_rad_s = a.omega_frd;
    imu.flags = kGyroValid;
    const auto attitude_group = [&] {
      if (n % attitude_divisor == 0) {
        const Vec3f stick = composition::sticks_at(sc.script, a.t_us);
        const auto out = angle.execute(attitude::AngleSticks<float>{stick[0], stick[1], stick[2]}, a);
        setpoint = law.execute(a, out.q_sp, out.yaw_rate_cmd).rate_setpoint;
      } else if (v.setpoint_not_held) {
        setpoint = Vec3f();
      }
    };
    const auto rate_group = [&] {
      step.filter(imu, n % r.divisor == 0);
      if (n % r.divisor == 0) {
        const Vec3f chirp = composition::chirp_rate(sc.chirp, a.t_us);
        const auto d = step.execute_bypass(v.chirp_as_torque ? setpoint : setpoint + chirp,
                                           composition::disturbance_torque(sc.dist, a.t_us) +
                                               (v.chirp_as_torque ? chirp : Vec3f()),
                                           sc.thrust)
                           .dshot;
        for (std::size_t i = 0; i < kMotors; ++i) {
          held[i] = d[i].raw();
        }
      }
    };
    if (v.rate_before_attitude) {
      rate_group();
      attitude_group();
    } else {
      attitude_group();
      rate_group();
    }
    rows.push_back(held);
  }
  return rows;
}

// The first tick at which a and b differ, or k when they agree throughout.
std::uint32_t first_difference(const std::vector<Dshots>& a, const std::vector<Dshots>& b) {
  const std::size_t k = std::min(a.size(), b.size());
  for (std::size_t n = 0; n < k; ++n) {
    if (a[n] != b[n]) {
      return static_cast<std::uint32_t>(n);
    }
  }
  return static_cast<std::uint32_t>(k);
}

// ---- scenarios ----------------------------------------------------------------------------------------------------

// scenario test values: sticks [-1, 1] per segment, and the run length in ticks (covers the segments below at every
// divisor used)
constexpr std::uint32_t kRun = 400;

Scenario rich_scenario(const Ref& r) {
  Scenario sc;
  sc.thrust = r.hover;
  sc.script.count = 3;
  sc.script.segment[0] = {0, Vec3f(0.5F, -0.25F, 0.0F)};
  sc.script.segment[1] = {static_cast<std::int32_t>(stamp(r, 90)), Vec3f(0.0F, 0.75F, 0.5F)};
  sc.script.segment[2] = {static_cast<std::int32_t>(stamp(r, 210)), Vec3f(-1.0F, 0.0F, -0.5F)};
  sc.chirp.axis = ChirpAxis::Pitch;
  sc.chirp.amp = 0.5F;  // rad/s
  sc.chirp.w_lo = 20.0F;
  sc.chirp.w_hi = 200.0F;
  sc.chirp.t0_us = static_cast<std::int32_t>(stamp(r, 40));
  sc.chirp.dur_us = 40000;
  sc.dist.yaw_nm = 0.5F * r.p_torque_yaw;
  sc.dist.t0_us = static_cast<std::int32_t>(stamp(r, 150));
  return sc;
}

// A scenario with one thing switched on, level still vehicle: the output moves only through that thing.
Scenario quiet_scenario(const Ref& r) {
  Scenario sc;
  sc.thrust = r.hover;
  return sc;
}

// ---- info and neutral ---------------------------------------------------------------------------------------------

TEST(L5Composition, ReportsTheCompositionFourMotorsNoServosAndItsParameterCount) {
  marv_sil_info info{};
  info.struct_size = sizeof(info);
  ASSERT_EQ(marv_sil_info_get(&info), MARV_SIL_OK);
  EXPECT_EQ(info.n_motors, kMotors);
  EXPECT_EQ(info.n_servos, 0U);
  EXPECT_STREQ(info.composition, "l5_attitude_scripted");
  EXPECT_EQ(info.n_params, kParamCount);
  EXPECT_EQ(info.param_schema_hash, kParamSchemaHash);
}

TEST(L5Composition, HoverThrustLevelStillTruthAndNoScriptGivesThePureThrustAllocation) {
  in_child([] {
    const Scenario sc = quiet_scenario(defaults_ref());
    const Overrides ov = sc.overrides();
    const Ref r = load_ref(ov);
    ASSERT_EQ(init_with(r, ov), MARV_SIL_OK);
    const Dshots want = dshot_of(r, r.hover, Vec3f());
    const Dshots idle = dshot_of(r, 0.0F, Vec3f());
    for (std::size_t m = 0; m < kMotors; ++m) {
      ASSERT_GT(want[m], idle[m]) << "motor " << (m + 1);  // the hover is a real command
    }
    const std::vector<Dshots> rows = run_sil(r, Trajectory{}, 64);
    const std::vector<Dshots> diffused_want = pure_thrust_rows(r, r.hover, 64);
    for (std::size_t n = 0; n < rows.size(); ++n) {
      ASSERT_EQ(rows[n], diffused_want[n]) << "tick " << n;
    }
    EXPECT_TRUE(differs_from_stateless(rows, want)) << "control: stateless rounding reproduces the composition";
  });
}

// ---- the whole tick against the reference -------------------------------------------------------------------------

// The composition equals the reference on every tick, and the control references do not.
void expect_matches_reference(std::int32_t divisor, std::int32_t ratio) {
  in_child([=] {
    Scenario sc;
    sc = rich_scenario(defaults_ref());
    sc.divisor = divisor;
    sc.ratio = ratio;
    const Overrides ov = sc.overrides();
    const Ref r = load_ref(ov);
    Trajectory tr;
    tr.amp = 0.2F;  // scenario test value: rad
    ASSERT_EQ(init_with(r, ov), MARV_SIL_OK);
    const std::vector<Dshots> got = run_sil(r, tr, kRun);
    const std::vector<Dshots> want = reference(r, sc, tr, kRun);
    ASSERT_EQ(got.size(), want.size());
    for (std::uint32_t n = 0; n < kRun; ++n) {
      ASSERT_EQ(got[n], want[n]) << "tick " << n << " divisor " << r.divisor << " ratio " << r.ratio;
    }
    // power: the run is not the pure-thrust allocation, and the attitude group does something on many of its ticks
    const std::vector<Dshots> base = pure_thrust_rows(r, r.hover, kRun);
    std::uint32_t moved = 0;
    for (std::uint32_t n = 0; n < kRun; ++n) {
      moved += got[n] != base[n] ? 1 : 0;
    }
    EXPECT_GT(moved, kRun / 2);  // scenario test value: most ticks carry a non-neutral command
    // controls: each wrong behaviour fails to reproduce the composition's output
    Variant order;
    order.rate_before_attitude = true;
    EXPECT_LT(first_difference(got, reference(r, sc, tr, kRun, order)), kRun) << "attitude after the rate group";
    Variant chirp;
    chirp.chirp_as_torque = true;
    EXPECT_LT(first_difference(got, reference(r, sc, tr, kRun, chirp)), kRun) << "chirp as a torque";
    // These two controls differ from the composition only when the attitude group runs less often than the rate group
    // (att_loop_ratio > 1): with a ratio of 1 a setpoint is never held over a rate tick and the divisor without the ratio
    // is the divisor. They are therefore asserted whenever the ratio exceeds 1, and every overridden-divisor test must
    // have a ratio above 1, so they always run and must differ in this suite.
    if (ratio > 0) {
      ASSERT_GT(r.ratio, 1U) << "an overridden-divisor test must have att_loop_ratio > 1";
    }
    if (r.ratio > 1) {
      Variant hold;
      hold.setpoint_not_held = true;
      EXPECT_LT(first_difference(got, reference(r, sc, tr, kRun, hold)), kRun) << "setpoint not held";
      Variant wrong_divisor;
      wrong_divisor.attitude_divisor = r.divisor;  // the rate divisor alone: the attitude ratio ignored
      EXPECT_LT(first_difference(got, reference(r, sc, tr, kRun, wrong_divisor)), kRun) << "divisor without the ratio";
    }
    Variant other_divisor;
    other_divisor.attitude_divisor = r.divisor * r.ratio + r.divisor;
    EXPECT_LT(first_difference(got, reference(r, sc, tr, kRun, other_divisor)), kRun) << "one rate period too long";
  });
}

TEST(L5CompositionSchedule, MatchesTheReferenceBitForBitAtTheProductDivisors) { expect_matches_reference(0, 0); }

TEST(L5CompositionSchedule, MatchesTheReferenceAtRateDivisorThreeAndRatioTwo) { expect_matches_reference(3, 2); }

TEST(L5CompositionSchedule, MatchesTheReferenceAtRateDivisorOneAndRatioThree) { expect_matches_reference(1, 3); }

// ---- the attitude group: period, and before the rate group in the same tick --------------------------------------

// A roll stick from stamp(s), level still truth and a zero gyro, so the rate loop's torque is nonzero only once the
// attitude output reaches it (the first rate execution is a seed and gives zero torque; the segment starts later).
// The first tick whose DShot differs from the pure-thrust allocation is the first attitude execution at or after s,
// and in that tick: the attitude group ran before the rate group.
// The stick starts at tick m D + offset with m = 2 (past the seed execution of the rate loop); D = rate_loop_divisor *
// att_loop_ratio. The expected first change is the next attitude tick at or after the start: (m + (offset > 0)) D.
// divisor and ratio are overridden when positive, else the product values are used.
void expect_first_change_at_the_attitude_tick(std::int32_t divisor, std::int32_t ratio, std::uint32_t offset) {
  in_child([=] {
    constexpr std::uint32_t kM = 2;  // scenario test value: the second attitude period after the first
    const Ref probe = defaults_ref();
    const std::uint32_t d = divisor > 0 ? static_cast<std::uint32_t>(divisor) : probe.divisor;
    const std::uint32_t n = ratio > 0 ? static_cast<std::uint32_t>(ratio) : probe.ratio;
    const std::uint32_t attitude_divisor = d * n;
    ASSERT_LT(offset, attitude_divisor);
    const std::uint32_t start_tick = (kM * attitude_divisor) + offset;
    const std::uint32_t want_tick = (kM + (offset > 0 ? 1U : 0U)) * attitude_divisor;
    Scenario sc = quiet_scenario(probe);
    sc.script.count = 1;
    sc.script.segment[0] = {static_cast<std::int32_t>(stamp(probe, start_tick)), Vec3f(0.5F, 0.0F, 0.0F)};
    sc.divisor = divisor;
    sc.ratio = ratio;
    const Overrides ov = sc.overrides();
    const Ref r = load_ref(ov);
    ASSERT_EQ(r.divisor * r.ratio, attitude_divisor);
    ASSERT_EQ(init_with(r, ov), MARV_SIL_OK);
    const std::uint32_t k = want_tick + (2 * attitude_divisor);
    const std::vector<Dshots> rows = run_sil(r, Trajectory{}, k);
    const std::vector<Dshots> base = pure_thrust_rows(r, r.hover, k);
    const auto first_change = [&](const std::vector<Dshots>& v) {
      for (std::uint32_t t = 0; t < k; ++t) {
        if (v[t] != base[t]) {
          return t;
        }
      }
      return k;
    };
    EXPECT_EQ(first_change(rows), want_tick) << "divisor " << r.divisor << " ratio " << r.ratio << " start " << start_tick;
    // control: with the rate group first in the tick the same change appears one rate period later, so the test sees the
    // order
    Variant order;
    order.rate_before_attitude = true;
    EXPECT_EQ(first_change(reference(r, sc, Trajectory{}, k, order)), want_tick + r.divisor) << "control: rate group first";
    // control: with another attitude period (a multiple of the rate period whose first tick at or after the start is not
    // the expected one) the change appears at another tick, so the test sees the period
    std::uint32_t other = 0;
    for (const std::uint32_t candidate : {attitude_divisor + r.divisor, 2 * attitude_divisor, 3 * attitude_divisor,
                                          attitude_divisor + (2 * r.divisor)}) {
      if ((((start_tick + candidate - 1) / candidate) * candidate) != want_tick) {
        other = candidate;
        break;
      }
    }
    ASSERT_NE(other, 0U);
    Variant longer;
    longer.attitude_divisor = other;
    EXPECT_NE(first_change(reference(r, sc, Trajectory{}, k, longer)), want_tick) << "control: wrong attitude divisor";
  });
}

TEST(L5CompositionSchedule, ProductDivisorsTheStickActsAtTheAttitudeTickItStartsOn) {
  expect_first_change_at_the_attitude_tick(0, 0, 0);
}

TEST(L5CompositionSchedule, ProductDivisorsTheStickJustAfterAnAttitudeTickWaitsForTheNext) {
  expect_first_change_at_the_attitude_tick(0, 0, 1);
}

TEST(L5CompositionSchedule, DivisorThreeRatioTwoOnTheAttitudeTick) { expect_first_change_at_the_attitude_tick(3, 2, 0); }

TEST(L5CompositionSchedule, DivisorThreeRatioTwoOneRatePeriodAfterTheAttitudeTick) {
  expect_first_change_at_the_attitude_tick(3, 2, 3);
}

TEST(L5CompositionSchedule, DivisorTwoRatioThreeOneTickAfterTheAttitudeTick) { expect_first_change_at_the_attitude_tick(2, 3, 1); }

TEST(L5CompositionSchedule, DivisorOneRatioFourOnTheAttitudeTick) { expect_first_change_at_the_attitude_tick(1, 4, 0); }

TEST(L5CompositionSchedule, DivisorOneRatioFourThreeTicksAfterTheAttitudeTick) {
  expect_first_change_at_the_attitude_tick(1, 4, 3);
}

// ---- freshness ---------------------------------------------------------------------------------------------------

void disable_core_dumps() { (void)prctl(PR_SET_DUMPABLE, 0); }

// Runs ticks [0, stale_tick) with their truth states, then tick stale_tick without one (or with one when `fresh`).
void run_until_stale(bool fresh, bool skip_from_the_start) {
  const Ref probe = defaults_ref();
  Scenario sc = quiet_scenario(probe);
  const Overrides ov = sc.overrides();
  const Ref r = load_ref(ov);
  if (init_with(r, ov) != MARV_SIL_OK) {
    std::exit(1);
  }
  const std::uint32_t attitude_divisor = r.divisor * r.ratio;
  const Trajectory tr;
  for (std::uint32_t n = 0; n <= attitude_divisor; ++n) {
    const AttitudeState<float> a = tr.at(r, n);
    const bool at_stale_tick = n == attitude_divisor;
    const bool skip = skip_from_the_start ? !fresh : (at_stale_tick && !fresh);
    if (!skip) {
      const marv_truth_state s = truth_of(a, n);
      if (marv_truth_state_set(&s) != MARV_SIL_OK) {
        std::exit(1);
      }
    }
    (void)sil_tick(n, gyro_of(a));
  }
  std::exit(0);
}

TEST(L5CompositionFreshness, AnAttitudeTickWithoutAStateForItPanicsNamingTheStamp) {
  EXPECT_EXIT(
      {
        disable_core_dumps();
        run_until_stale(false, false);
      },
      ::testing::KilledBySignal(SIGABRT),
      "hal_panic: l5_attitude_scripted composition: the latched attitude stamp differs from the sample stamp");
}

TEST(L5CompositionFreshness, NoStateEverHandedOverPanicsAtTheFirstAttitudeTick) {
  EXPECT_EXIT(
      {
        disable_core_dumps();
        run_until_stale(false, true);
      },
      ::testing::KilledBySignal(SIGABRT),
      "hal_panic: l5_attitude_scripted composition: the latched attitude stamp differs from the sample stamp");
}

// Control of the two death checks above: the same run with a state for every tick does not panic.
TEST(L5CompositionFreshness, NegativeControlAStateForEveryTickRunsThroughTheAttitudeTick) {
  EXPECT_EXIT(
      {
        disable_core_dumps();
        run_until_stale(true, false);
      },
      ::testing::ExitedWithCode(0), "");
}

// A state is needed only on attitude ticks: between them none is required.
TEST(L5CompositionFreshness, NoStateIsNeededOnTicksWithoutAnAttitudeExecution) {
  in_child([] {
    const Ref probe = defaults_ref();
    const Scenario sc = quiet_scenario(probe);
    const Overrides ov = sc.overrides();
    const Ref r = load_ref(ov);
    ASSERT_EQ(init_with(r, ov), MARV_SIL_OK);
    const std::uint32_t attitude_divisor = r.divisor * r.ratio;
    ASSERT_GE(attitude_divisor, 2U);
    const Trajectory tr;
    for (std::uint32_t n = 0; n < attitude_divisor; ++n) {
      const AttitudeState<float> a = tr.at(r, n);
      if (n == 0) {
        const marv_truth_state s = truth_of(a, n);
        ASSERT_EQ(marv_truth_state_set(&s), MARV_SIL_OK);
      }
      (void)sil_tick(n, gyro_of(a));
    }
  });
}

// ---- the disturbance ----------------------------------------------------------------------------------------------

// A level still vehicle, zero sticks: the rate loop's torque stays zero, so the DShot is the allocation of {thrust,
// (0, 0, d)} from the first rate tick at or after the disturbance stamp and the pure-thrust allocation before it, both
// through the diffuser from init.
void expect_disturbance_from_its_stamp(std::int32_t divisor, std::uint32_t t0_tick) {
  in_child([=] {
    Scenario sc;
    {
      const Ref probe = defaults_ref();
      sc = quiet_scenario(probe);
      sc.dist.yaw_nm = probe.p_torque_yaw;
      sc.dist.t0_us = static_cast<std::int32_t>(stamp(probe, t0_tick));
    }
    sc.divisor = divisor;
    const Overrides ov = sc.overrides();
    const Ref r = load_ref(ov);
    ASSERT_EQ(init_with(r, ov), MARV_SIL_OK);
    const Dshots base = dshot_of(r, r.hover, Vec3f());
    const Dshots pushed = dshot_of(r, r.hover, Vec3f(0.0F, 0.0F, sc.dist.yaw_nm));
    ASSERT_NE(pushed, base);
    const std::uint32_t first_rate_tick = ((t0_tick + r.divisor - 1) / r.divisor) * r.divisor;
    const std::uint32_t k = first_rate_tick + (3 * r.divisor);
    const std::vector<Dshots> rows = run_sil(r, Trajectory{}, k);
    const std::vector<Dshots> want = diffused(r, r.hover, k, [&](std::uint32_t n) {
      return n < first_rate_tick ? Vec3f() : Vec3f(0.0F, 0.0F, sc.dist.yaw_nm);
    });
    for (std::uint32_t n = 0; n < k; ++n) {
      ASSERT_EQ(rows[n], want[n]) << "tick " << n << " disturbance stamp tick " << t0_tick;
    }
    // control: the reference without the disturbance stays at the pure-thrust allocation, so the step above is the disturbance
    Scenario none = sc;
    none.dist.yaw_nm = 0.0F;
    const std::vector<Dshots> quiet = reference(r, none, Trajectory{}, k);
    const std::vector<Dshots> pure = pure_thrust_rows(r, r.hover, k);
    for (std::uint32_t n = 0; n < k; ++n) {
      ASSERT_EQ(quiet[n], pure[n]) << "tick " << n;
    }
  });
}

TEST(L5CompositionDisturbance, SwitchesOnAtItsStampOnARateTick) { expect_disturbance_from_its_stamp(3, 6); }

TEST(L5CompositionDisturbance, SwitchesOnAtTheNextRateTickWhenItsStampFallsBetweenTwo) {
  expect_disturbance_from_its_stamp(3, 7);
}

TEST(L5CompositionDisturbance, AStampOneTickBeforeARateTickActsAtThatRateTick) { expect_disturbance_from_its_stamp(3, 8); }

TEST(L5CompositionDisturbance, AtStampZeroItActsFromTheFirstTick) { expect_disturbance_from_its_stamp(2, 0); }

// ---- the chirp ----------------------------------------------------------------------------------------------------

// The chirp enters the rate setpoint before execute_bypass (decision 0006 F): with a level still vehicle and zero sticks
// the attitude output is zero and the rate loop sees the chirp as a setpoint against a zero gyro, so the torque is
// kp * chirp + the integral. The reference reproduces that bit for bit; the control reference adds the same values to the
// torque request instead, and differs.
void expect_chirp_at_the_rate_setpoint(ChirpAxis axis) {
  in_child([=] {
    Scenario sc;
    {
      const Ref probe = defaults_ref();
      sc = quiet_scenario(probe);
      sc.chirp.axis = axis;
      sc.chirp.amp = 1.0F;  // scenario test values: rad/s, a 20 to 200 rad/s sweep of 0.04 s from tick 16
      sc.chirp.w_lo = 20.0F;
      sc.chirp.w_hi = 200.0F;
      sc.chirp.t0_us = static_cast<std::int32_t>(stamp(probe, 16));
      sc.chirp.dur_us = 40000;
    }
    const Overrides ov = sc.overrides();
    const Ref r = load_ref(ov);
    ASSERT_EQ(init_with(r, ov), MARV_SIL_OK);
    const std::uint32_t k = 320;  // covers 16 + 40000 us / 156.25 us = 272 ticks
    const std::vector<Dshots> got = run_sil(r, Trajectory{}, k);
    EXPECT_EQ(first_difference(got, reference(r, sc, Trajectory{}, k)), k) << "axis " << static_cast<int>(axis);
    const std::vector<Dshots> base = pure_thrust_rows(r, r.hover, k);
    for (std::uint32_t n = 0; n < 16; ++n) {
      ASSERT_EQ(got[n], base[n]) << "tick " << n << " precedes the window";
    }
    std::uint32_t moved = 0;
    for (std::uint32_t n = 16; n < k; ++n) {
      moved += got[n] != base[n] ? 1 : 0;
    }
    EXPECT_GT(moved, 100U);  // scenario test value: the chirp moves the command on many ticks
    Variant torque;
    torque.chirp_as_torque = true;
    EXPECT_LT(first_difference(got, reference(r, sc, Trajectory{}, k, torque)), k) << "control: chirp as a torque";
  });
}

TEST(L5CompositionChirp, RollEntersTheRateSetpoint) { expect_chirp_at_the_rate_setpoint(ChirpAxis::Roll); }

TEST(L5CompositionChirp, PitchEntersTheRateSetpoint) { expect_chirp_at_the_rate_setpoint(ChirpAxis::Pitch); }

TEST(L5CompositionChirp, YawEntersTheRateSetpoint) { expect_chirp_at_the_rate_setpoint(ChirpAxis::Yaw); }

// ---- invalid scenario parameters ---------------------------------------------------------------------------------

void expect_init_panics(const char* what, void (*set)(Overrides&), const char* message) {
  const std::string want = std::string("hal_panic: ") + message;
  EXPECT_EXIT(
      {
        disable_core_dumps();
        const Ref r = load_ref(Overrides{});
        Overrides ov;
        set(ov);
        (void)init_with(r, ov);
        std::exit(0);
      },
      ::testing::KilledBySignal(SIGABRT), want)
      << what;
}

TEST(L5CompositionIllegal, InvalidScenarioParametersPanicAtInitNamingTheRule) {
  expect_init_panics("negative thrust", [](Overrides& o) { o.f32(ParamId::l5_thrust_n, -1.0F); },
                     "l5_attitude_scripted composition: l5_thrust_n is not finite");
  expect_init_panics("negative segment count", [](Overrides& o) { o.i32(ParamId::l5_seg_count, -1); },
                     "l5_attitude_scripted composition: l5_seg_count is not in");
  expect_init_panics("segment count above the capacity", [](Overrides& o) { o.i32(ParamId::l5_seg_count, 9); },
                     "l5_attitude_scripted composition: l5_seg_count is not in");
  expect_init_panics(
      "equal segment stamps",
      [](Overrides& o) {
        o.i32(ParamId::l5_seg_count, 2);
        o.i32(ParamId::l5_seg1_t_us, 100);
        o.i32(ParamId::l5_seg2_t_us, 100);
      },
      "l5_attitude_scripted composition: segment stamps");
  expect_init_panics("decreasing segment stamps",
                     [](Overrides& o) {
                       o.i32(ParamId::l5_seg_count, 2);
                       o.i32(ParamId::l5_seg1_t_us, 200);
                       o.i32(ParamId::l5_seg2_t_us, 100);
                     },
                     "l5_attitude_scripted composition: segment stamps");
  expect_init_panics("negative segment stamp",
                     [](Overrides& o) {
                       o.i32(ParamId::l5_seg_count, 1);
                       o.i32(ParamId::l5_seg1_t_us, -1);
                     },
                     "l5_attitude_scripted composition: segment stamps");
  expect_init_panics("stick above one",
                     [](Overrides& o) {
                       o.i32(ParamId::l5_seg_count, 1);
                       o.f32(ParamId::l5_seg1_roll, std::nextafter(1.0F, 2.0F));
                     },
                     "l5_attitude_scripted composition: a segment stick");
  expect_init_panics("stick below minus one",
                     [](Overrides& o) {
                       o.i32(ParamId::l5_seg_count, 2);
                       o.i32(ParamId::l5_seg1_t_us, 1);
                       o.i32(ParamId::l5_seg2_t_us, 2);
                       o.f32(ParamId::l5_seg2_yaw, -2.0F);
                     },
                     "l5_attitude_scripted composition: a segment stick");
  expect_init_panics("chirp axis out of range", [](Overrides& o) { o.i32(ParamId::l5_chirp_axis, 4); },
                     "l5_attitude_scripted composition: l5_chirp_axis is not");
  expect_init_panics("negative chirp amplitude", [](Overrides& o) { o.f32(ParamId::l5_chirp_amp_rad_s, -0.1F); },
                     "l5_attitude_scripted composition: l5_chirp_amp_rad_s is not");
  expect_init_panics("chirp with the default zero band", [](Overrides& o) { o.i32(ParamId::l5_chirp_axis, 1); },
                     "l5_attitude_scripted composition: l5_chirp_w_lo, l5_chirp_w_hi");
  expect_init_panics(
      "chirp with zero duration",
      [](Overrides& o) {
        o.i32(ParamId::l5_chirp_axis, 3);
        o.f32(ParamId::l5_chirp_w_lo, 1.0F);
        o.f32(ParamId::l5_chirp_w_hi, 2.0F);
      },
      "l5_attitude_scripted composition: l5_chirp_dur_us is not");
  expect_init_panics("negative chirp start",
                     [](Overrides& o) {
                       o.i32(ParamId::l5_chirp_axis, 3);
                       o.f32(ParamId::l5_chirp_w_lo, 1.0F);
                       o.f32(ParamId::l5_chirp_w_hi, 2.0F);
                       o.i32(ParamId::l5_chirp_dur_us, 1000);
                       o.i32(ParamId::l5_chirp_t0_us, -1);
                     },
                     "l5_attitude_scripted composition: l5_chirp_t0_us is negative");
  expect_init_panics("negative disturbance start", [](Overrides& o) { o.i32(ParamId::l5_dist_t0_us, -1); },
                     "l5_attitude_scripted composition: l5_dist_t0_us is negative");
  expect_init_panics("att_loop_ratio zero", [](Overrides& o) { o.i32(ParamId::att_loop_ratio, 0); },
                     "attitude: the design period is not positive");
  expect_init_panics("rate_loop_divisor zero", [](Overrides& o) { o.i32(ParamId::rate_loop_divisor, 0); },
                     "rate: the design period is not positive");
}

// Control of the death checks above: valid values of the same parameters initialise, and the bounds themselves are valid.
TEST(L5CompositionIllegal, NegativeControlValidScenarioValuesInitialise) {
  in_child([] {
    const Ref probe = defaults_ref();
    Overrides ov;
    ov.f32(ParamId::l5_thrust_n, probe.hover);
    ov.i32(ParamId::l5_seg_count, 2);
    ov.i32(ParamId::l5_seg1_t_us, 100);
    ov.f32(ParamId::l5_seg1_roll, 1.0F);
    ov.i32(ParamId::l5_seg2_t_us, 200);
    ov.f32(ParamId::l5_seg2_yaw, -1.0F);
    ov.i32(ParamId::l5_chirp_axis, 3);
    ov.f32(ParamId::l5_chirp_amp_rad_s, 0.1F);
    ov.f32(ParamId::l5_chirp_w_lo, 1.0F);
    ov.f32(ParamId::l5_chirp_w_hi, 2.0F);
    ov.i32(ParamId::l5_chirp_dur_us, 1000);
    ov.f32(ParamId::l5_dist_yaw_nm, -0.25F);
    ov.i32(ParamId::l5_dist_t0_us, 50);
    EXPECT_EQ(init_with(probe, ov), MARV_SIL_OK);
  });
}

}  // namespace
