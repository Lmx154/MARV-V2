// T1 of the l11_vertical_scripted composition through the SIL entry points (marv_sil.h, marv_truth.h,
// marv_truth_vertical.h): per tick j the test calls marv_truth_state_set(tick = j), marv_truth_vertical_set(tick = j) and
// then marv_sil_tick(j, 1, ...), as the plugin does (decision 0020 section 1, the truth feed).
//
// Reference. The test runs an independent tick loop in this executable, built from the same modules (vertical::VerticalLaw,
// mixer::allocate and mixer::DshotDiffuser) but scheduled by plain modulo arithmetic (n % (rate_loop_divisor *
// vert_loop_ratio), n % rate_loop_divisor) in the order decision 0020 section 1 names: the vertical group, then the rate
// group, in the same tick; the thrust request held between vertical executions; the thrust disturbance added to the request
// at the vertical execution; the torque pulse added to the torque request before the mixer at the rate execution. The
// truth attitude is level and still and the sticks are zero in every test, so the attitude and rate loops contribute a zero
// torque (as in the L5 composition test) and the reference's mixer request is {thrust, pulse}. Hover, disturbance, pulse and
// fallback tests also compare with closed-form rows (the allocation of a stated request through the diffuser) that do not
// use the vertical law.
//
// Controls. Every check that compares with the reference has a control reference that differs in exactly the behaviour
// under test (vertical after the rate group, a wrong vertical divisor, a segment stamp a vertical period early or late) and
// must not reproduce the composition's output.
//
// marv_sil_init succeeds once per process, so every test that initialises runs in a forked child.
#include <gtest/gtest.h>

#include <marv_sil.h>
#include <marv_truth.h>
#include <marv_truth_vertical.h>

#include <sys/prctl.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <limits>
#include <span>
#include <string>
#include <vector>

#include <marv/mixer/mixer.hpp>
#include <marv/params/param.hpp>
#include <marv/params/param_ids.hpp>
#include <marv/prim/constants.hpp>
#include <marv/prim/quat.hpp>
#include <marv/types/actuator.hpp>
#include <marv/types/time.hpp>
#include <marv/types/vertical_state.hpp>
#include <marv/vertical/config.hpp>
#include <marv/vertical/vertical_law.hpp>

namespace {

using namespace marv;

constexpr std::size_t kMotors = kQuadXMotors;
using Dshots = std::array<std::uint16_t, kMotors>;
using Vec3f = prim::Vec3<float>;
constexpr std::uint32_t kGyroValid = 1u << MARV_IMU_GYRO_VALID;
constexpr std::uint32_t kAttitudeValid = 1u << MARV_TRUTH_ATTITUDE_VALID;
constexpr std::uint32_t kVerticalValid = 1u << MARV_TRUTH_VERTICAL_VALID;
constexpr std::size_t kSegments = 8;  // the capacity of the register (l11_seg1..l11_seg8)
constexpr std::uint32_t kNever = std::numeric_limits<std::uint32_t>::max();

// The hover collective of the card scenario: l5_thrust_n = f32(m g(phi, h0)) = 7.5705781 N, the hex float of line 9 of
// tests/regression/quad/L05/t3/reference/attitude_t3_q_inputs.txt (0x1.e4845a0000000p+2). vert_hover_thrust_n is a per-run
// scenario value of the same rule (the register's comment); the default 0 is refused at init, so every run sets it.
constexpr float kCardHoverThrustN = 0x1.e4845ap+2F;

// scenario test values, each a small excursion of a truth state or a disturbance, far from the mixer's limits:
constexpr float kHeightOffsetM = 0.1F;      // a height error of 10 cm: the collective moves by about 1 N of the 7.6 N hover
constexpr float kRampM = 0.01F;             // the truth height changes by 1 cm per tick in the group tests
constexpr std::uint32_t kRun = 64;          // ticks per run: covers every stamp used below at the product divisors

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

void disable_core_dumps() { (void)prctl(PR_SET_DUMPABLE, 0); }

// ---- overrides: one list for the SIL and for the reference's table ------------------------------------------------

struct Overrides {
  std::vector<marv_sil_param_override> v;
  // A later override of an id replaces the earlier one (the SIL refuses two overrides of one id).
  void put(const marv_sil_param_override& o) {
    std::erase_if(v, [&](const marv_sil_param_override& e) { return e.id == o.id; });
    v.push_back(o);
  }
  void f32(ParamId id, float x) {
    marv_sil_param_override o{};
    o.id = static_cast<std::uint32_t>(id);
    o.type = MARV_PARAM_F32;
    o.f32 = x;
    put(o);
  }
  void i32(ParamId id, std::int32_t x) {
    marv_sil_param_override o{};
    o.id = static_cast<std::uint32_t>(id);
    o.type = MARV_PARAM_I32;
    o.i32 = x;
    put(o);
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
  ParamId z_ref;
};

constexpr std::array kSegmentIds{
    SegmentIds{ParamId::l11_seg1_t_us, ParamId::l11_seg1_z_ref_m}, SegmentIds{ParamId::l11_seg2_t_us, ParamId::l11_seg2_z_ref_m},
    SegmentIds{ParamId::l11_seg3_t_us, ParamId::l11_seg3_z_ref_m}, SegmentIds{ParamId::l11_seg4_t_us, ParamId::l11_seg4_z_ref_m},
    SegmentIds{ParamId::l11_seg5_t_us, ParamId::l11_seg5_z_ref_m}, SegmentIds{ParamId::l11_seg6_t_us, ParamId::l11_seg6_z_ref_m},
    SegmentIds{ParamId::l11_seg7_t_us, ParamId::l11_seg7_z_ref_m}, SegmentIds{ParamId::l11_seg8_t_us, ParamId::l11_seg8_z_ref_m}};

// The vertical truth state of tick n: z and v_z in NED (z down), or invalid (valid bit clear, fields 0).
struct VertState {
  float z = 0.0F;
  float vz = 0.0F;
  bool valid = true;
};
using Feed = std::function<VertState(std::uint32_t)>;

struct Segment {
  std::int32_t t_us = 0;
  float z_ref_m = 0.0F;
};

// A scenario: the composition's scripted inputs as structs, turned into overrides; divisor and ratio are overridden when
// positive.
struct Scene {
  float hover = kCardHoverThrustN;
  std::vector<Segment> segments;
  float dist_thrust_n = 0.0F;
  std::int32_t dist_thrust_t_us = 0;
  std::int32_t pulse_axis = 0;
  float pulse_nm = 0.0F;
  std::int32_t pulse_t_us = 0;
  std::int32_t pulse_dur_us = 0;
  std::int32_t divisor = 0;
  std::int32_t vratio = 0;
  Feed feed = [](std::uint32_t) { return VertState{}; };

  [[nodiscard]] Overrides overrides() const {
    Overrides ov;
    ov.f32(ParamId::vert_hover_thrust_n, hover);
    ov.i32(ParamId::l11_seg_count, static_cast<std::int32_t>(segments.size()));
    for (std::size_t k = 0; k < segments.size() && k < kSegments; ++k) {
      ov.i32(kSegmentIds[k].t_us, segments[k].t_us);
      ov.f32(kSegmentIds[k].z_ref, segments[k].z_ref_m);
    }
    ov.f32(ParamId::l11_dist_thrust_n, dist_thrust_n);
    ov.i32(ParamId::l11_dist_thrust_t_us, dist_thrust_t_us);
    ov.i32(ParamId::l11_dist_torque_axis, pulse_axis);
    ov.f32(ParamId::l11_dist_torque_nm, pulse_nm);
    ov.i32(ParamId::l11_dist_torque_t_us, pulse_t_us);
    ov.i32(ParamId::l11_dist_torque_dur_us, pulse_dur_us);
    if (divisor > 0) {
      ov.i32(ParamId::rate_loop_divisor, divisor);
    }
    if (vratio > 0) {
      ov.i32(ParamId::vert_loop_ratio, vratio);
    }
    return ov;
  }
};

// ---- the configuration of the run ---------------------------------------------------------------------------------

struct Ref {
  mixer::MixerConfig<float> mixer;
  vertical::VerticalConfig<float> vertical;
  std::uint32_t tick_num_us = 0;
  std::uint32_t tick_den = 0;
  std::uint32_t divisor = 0;  // rate_loop_divisor
  std::uint32_t vratio = 0;   // vert_loop_ratio
  std::array<float, 3> p_torque{};  // kp * rate_max per axis: the P torque of a full-rate error (roll, pitch, yaw)
};

float default_f32(ParamId id) { return param_defaults()[static_cast<std::size_t>(id)].value.f32; }
std::int32_t default_i32(ParamId id) { return param_defaults()[static_cast<std::size_t>(id)].value.i32; }

// The scalars of the default table, read without params_init (which may run once per process): enough to build a scenario
// before the overrides are known. The configurations are not filled.
Ref defaults_ref() {
  Ref r;
  r.tick_num_us = static_cast<std::uint32_t>(default_i32(ParamId::tick_period_num_us));
  r.tick_den = static_cast<std::uint32_t>(default_i32(ParamId::tick_period_den));
  r.divisor = static_cast<std::uint32_t>(default_i32(ParamId::rate_loop_divisor));
  r.vratio = static_cast<std::uint32_t>(default_i32(ParamId::vert_loop_ratio));
  r.p_torque = {default_f32(ParamId::rate_kp_roll) * default_f32(ParamId::rate_max_roll),
                default_f32(ParamId::rate_kp_pitch) * default_f32(ParamId::rate_max_pitch),
                default_f32(ParamId::rate_kp_yaw) * default_f32(ParamId::rate_max_yaw)};
  return r;
}

Ref load_ref(const Overrides& ov) {
  if (!init_params_with(ov)) {
    std::fprintf(stderr, "params_init failed\n");
    std::exit(1);
  }
  Ref r = defaults_ref();
  r.mixer = mixer::load_config();
  r.tick_num_us = static_cast<std::uint32_t>(param_value<ParamId::tick_period_num_us>());
  r.tick_den = static_cast<std::uint32_t>(param_value<ParamId::tick_period_den>());
  r.divisor = static_cast<std::uint32_t>(param_value<ParamId::rate_loop_divisor>());
  r.vratio = static_cast<std::uint32_t>(param_value<ParamId::vert_loop_ratio>());
  r.vertical.kp_z = param_value<ParamId::vert_kp_z>();
  r.vertical.kp_v = param_value<ParamId::vert_kp_v>();
  r.vertical.ki_v = param_value<ParamId::vert_ki_v>();
  r.vertical.hover_thrust = param_value<ParamId::vert_hover_thrust_n>();
  // The vertical period T_v = vert_loop_ratio * rate_loop_divisor ticks of tick_period_num_us / tick_period_den us.
  r.vertical.period = static_cast<float>(r.vratio) * static_cast<float>(r.divisor) * static_cast<float>(r.tick_num_us) /
                      static_cast<float>(r.tick_den) / static_cast<float>(prim::kMicrosecondsPerSecond);
  return r;
}

TimeUs stamp(const Ref& r, std::uint64_t n) {
  return static_cast<TimeUs>(static_cast<unsigned __int128>(n) * r.tick_num_us / r.tick_den);
}

// The stamp of tick n as the int32 a scenario parameter carries.
std::int32_t stamp_i32(const Ref& r, std::uint64_t n) { return static_cast<std::int32_t>(stamp(r, n)); }

Dshots to_dshots(const std::array<DshotValue, kMotors>& d) {
  Dshots out{};
  for (std::size_t i = 0; i < kMotors; ++i) {
    out[i] = d[i].raw();
  }
  return out;
}

Dshots dshot_of(const Ref& r, float thrust, const Vec3f& torque) {
  const auto f = mixer::allocate(r.mixer, mixer::Request<float>{thrust, torque}).f;
  return to_dshots(mixer::thrust_to_dshot(r.mixer, f));
}

// The composition's DShot per tick from init for a mixer request {thrust(n), torque(n)} (decision 0017): the diffuser on
// the allocation at every rate tick n (n % rate_loop_divisor == 0), the command held in between. The request functions
// are closed forms stated by the test; the vertical law is not used.
template <class Thrust, class Torque>
std::vector<Dshots> diffused(const Ref& r, std::uint32_t k, Thrust thrust, Torque torque) {
  mixer::DshotDiffuser<float> diffuser;
  diffuser.init(r.mixer);
  std::vector<Dshots> rows(k);
  Dshots held{};
  for (std::uint32_t n = 0; n < k; ++n) {
    if (n % r.divisor == 0) {
      held = to_dshots(diffuser.apply(mixer::allocate(r.mixer, mixer::Request<float>{thrust(n), torque(n)}).f));
    }
    rows[n] = held;
  }
  return rows;
}

std::vector<Dshots> hover_rows(const Ref& r, float hover, std::uint32_t k) {
  return diffused(r, k, [=](std::uint32_t) { return hover; }, [](std::uint32_t) { return Vec3f(); });
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

// One tick: marv_sil_tick(tick, 1, ...) with a level still gyro sample; the DShot row of the tick.
Dshots sil_tick(std::uint64_t tick) {
  marv_imu_meas imu{};
  imu.flags = kGyroValid;
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

// The level still truth attitude (identity quaternion, zero rates, valid) of tick n.
marv_truth_state level_truth(std::uint64_t tick) {
  marv_truth_state s{};
  s.struct_size = sizeof(marv_truth_state);
  s.flags = kAttitudeValid;
  s.tick = tick;
  s.q_wxyz[MARV_TRUTH_Q_W] = 1.0F;
  return s;
}

marv_truth_vertical vertical_truth(std::uint64_t tick, const VertState& v) {
  marv_truth_vertical s{};
  s.struct_size = sizeof(marv_truth_vertical);
  s.flags = v.valid ? kVerticalValid : 0U;
  s.tick = tick;
  s.z_ned_m = v.valid ? v.z : 0.0F;
  s.vz_ned_m_s = v.valid ? v.vz : 0.0F;
  return s;
}

// Which hand-overs a run withholds: the attitude of tick `attitude_tick`, the vertical state of tick `vertical_tick` or of
// every tick when `vertical_always`.
struct Withhold {
  std::uint32_t attitude_tick = kNever;
  std::uint32_t vertical_tick = kNever;
  bool vertical_always = false;
};

// Ticks [0, k): the truth states of each tick, then the tick.
std::vector<Dshots> run_sil(const Scene& sc, std::uint32_t k, Withhold w = {}) {
  std::vector<Dshots> rows;
  for (std::uint32_t n = 0; n < k; ++n) {
    if (n != w.attitude_tick) {
      const marv_truth_state a = level_truth(n);
      EXPECT_EQ(marv_truth_state_set(&a), MARV_SIL_OK);
    }
    if (!w.vertical_always && n != w.vertical_tick) {
      const marv_truth_vertical v = vertical_truth(n, sc.feed(n));
      EXPECT_EQ(marv_truth_vertical_set(&v), MARV_SIL_OK);
    }
    rows.push_back(sil_tick(n));
  }
  return rows;
}

// Loads the reference configuration, initialises the SIL and runs k ticks; the child's whole body for a plain run.
struct SceneRun {
  Ref r;
  std::vector<Dshots> rows;
};

SceneRun run_scene(const Scene& sc, std::uint32_t k) {
  const Overrides ov = sc.overrides();
  SceneRun run;
  run.r = load_ref(ov);
  EXPECT_EQ(init_with(run.r, ov), MARV_SIL_OK);
  run.rows = run_sil(sc, k);
  return run;
}

// ---- the reference loop -------------------------------------------------------------------------------------------

// What a control reference changes: nothing for the reference itself.
struct Variant {
  bool vertical_after_rate = false;  // the vertical group runs after the rate group in a tick
  std::uint32_t vratio = 0;          // overrides vert_loop_ratio when nonzero (the period is left as configured)
  bool record_allocation = true;     // false: the law is never told what the mixer achieved
};

// The z_ref of the script at stamp t_us: the last segment with t_k <= t_us, else 0 m.
float z_ref_at(const std::vector<Segment>& segments, TimeUs t_us) {
  float z_ref = 0.0F;
  for (const Segment& s : segments) {
    if (t_us >= static_cast<TimeUs>(s.t_us)) {
      z_ref = s.z_ref_m;
    }
  }
  return z_ref;
}

std::vector<Dshots> reference(const Ref& r, const Scene& sc, std::uint32_t k, Variant v = {}) {
  vertical::VerticalLaw<float> law;
  law.init(r.vertical, r.mixer);
  mixer::DshotDiffuser<float> diffuser;
  diffuser.init(r.mixer);
  const std::uint32_t vdivisor = r.divisor * (v.vratio != 0 ? v.vratio : r.vratio);
  float thrust = 0.0F;
  Dshots held{};
  std::vector<Dshots> rows;
  for (std::uint32_t n = 0; n < k; ++n) {
    const TimeUs t_us = stamp(r, n);
    const auto vertical_group = [&] {
      if (n % vdivisor == 0) {
        const VertState f = sc.feed(n);
        const VerticalState<float> s{t_us, f.z, f.vz, f.valid};
        thrust = law.execute(s, z_ref_at(sc.segments, t_us)).thrust_request +
                 (t_us >= static_cast<TimeUs>(sc.dist_thrust_t_us) ? sc.dist_thrust_n : 0.0F);
      }
    };
    const auto rate_group = [&] {
      if (n % r.divisor == 0) {
        Vec3f tau;
        const auto t0 = static_cast<TimeUs>(sc.pulse_t_us);
        if (t_us >= t0 && t_us - t0 < static_cast<TimeUs>(sc.pulse_dur_us)) {
          tau[static_cast<std::size_t>(sc.pulse_axis)] = sc.pulse_nm;
        }
        const mixer::Allocation<float> al = mixer::allocate(r.mixer, mixer::Request<float>{thrust, tau});
        held = to_dshots(diffuser.apply(al.f));
        if (v.record_allocation) {
          law.record_allocation(al, thrust);
        }
      }
    };
    if (v.vertical_after_rate) {
      rate_group();
      vertical_group();
    } else {
      vertical_group();
      rate_group();
    }
    rows.push_back(held);
  }
  return rows;
}

// The first tick at which a and b differ, or the common length when they agree throughout.
std::uint32_t first_difference(const std::vector<Dshots>& a, const std::vector<Dshots>& b) {
  const std::size_t k = std::min(a.size(), b.size());
  for (std::size_t n = 0; n < k; ++n) {
    if (a[n] != b[n]) {
      return static_cast<std::uint32_t>(n);
    }
  }
  return static_cast<std::uint32_t>(k);
}

// The first vertical execution (tick n % (rate_loop_divisor * vert_loop_ratio) == 0) whose stamp is >= t_us.
std::uint32_t first_vertical_tick_from(const Ref& r, std::int32_t t_us) {
  const std::uint32_t vdivisor = r.divisor * r.vratio;
  for (std::uint32_t n = 0;; n += vdivisor) {
    if (stamp(r, n) >= static_cast<TimeUs>(t_us)) {
      return n;
    }
  }
}

// ---- info ---------------------------------------------------------------------------------------------------------

TEST(L11CompositionInfo, ReportsTheCompositionFourMotorsNoServosAndItsParameterCount) {
  marv_sil_info info{};
  info.struct_size = sizeof(info);
  ASSERT_EQ(marv_sil_info_get(&info), MARV_SIL_OK);
  EXPECT_EQ(info.n_motors, kMotors);
  EXPECT_EQ(info.n_servos, 0U);
  EXPECT_STREQ(info.composition, "l11_vertical_scripted");
  EXPECT_EQ(info.n_params, kParamCount);
  EXPECT_EQ(info.param_schema_hash, kParamSchemaHash);
}

// ---- T1 parameter validation --------------------------------------------------------------------------------------

// Rationale: the composition must refuse, at init and naming the rule, a configuration the vertical law cannot run on
// (the default hover thrust 0, a vertical ratio below 1, a non-finite or negative gain), injected only through the
// parameter interface; the control is the same configuration with valid values.
void expect_init_panics(const char* what, void (*set)(Overrides&), const char* message) {
  const std::string want = std::string("hal_panic: l11_vertical_scripted composition: ") + message;
  EXPECT_EXIT(
      {
        disable_core_dumps();
        Scene sc;
        const Overrides valid = sc.overrides();
        const Ref r = load_ref(valid);
        Overrides ov = valid;
        set(ov);
        (void)init_with(r, ov);
        std::exit(0);
      },
      ::testing::KilledBySignal(SIGABRT), want)
      << what;
}

TEST(L11CompositionParams, TheDefaultHoverThrustZeroPanicsAtInitNamingTheRule) {
  EXPECT_EXIT(
      {
        disable_core_dumps();
        const Ref r = load_ref(Overrides{});
        (void)init_with(r, Overrides{});
        std::exit(0);
      },
      ::testing::KilledBySignal(SIGABRT), "hal_panic: l11_vertical_scripted composition: vert_hover_thrust_n is not > 0");
}

TEST(L11CompositionParams, ANonPositiveHoverThrustPanicsAtInit) {
  expect_init_panics("negative hover thrust", [](Overrides& o) { o.f32(ParamId::vert_hover_thrust_n, -1.0F); },
                     "vert_hover_thrust_n is not > 0");
}

TEST(L11CompositionParams, AVerticalRatioBelowOnePanicsAtInit) {
  expect_init_panics("vert_loop_ratio zero", [](Overrides& o) { o.i32(ParamId::vert_loop_ratio, 0); },
                     "rate_loop_divisor, att_loop_ratio or vert_loop_ratio is below 1");
  expect_init_panics("vert_loop_ratio negative", [](Overrides& o) { o.i32(ParamId::vert_loop_ratio, -1); },
                     "rate_loop_divisor, att_loop_ratio or vert_loop_ratio is below 1");
}

TEST(L11CompositionParams, ANegativeGainPanicsAtInit) {
  expect_init_panics("negative vert_kp_z", [](Overrides& o) { o.f32(ParamId::vert_kp_z, -1.0F); },
                     "vert_kp_z, vert_kp_v or vert_ki_v is negative");
  expect_init_panics("negative vert_kp_v", [](Overrides& o) { o.f32(ParamId::vert_kp_v, -1.0F); },
                     "vert_kp_z, vert_kp_v or vert_ki_v is negative");
  expect_init_panics("negative vert_ki_v", [](Overrides& o) { o.f32(ParamId::vert_ki_v, -1.0F); },
                     "vert_kp_z, vert_kp_v or vert_ki_v is negative");
}

// A non-finite value never reaches the composition: the SIL's parameter interface refuses it (MARV_SIL_E_PARAM,
// fw/sil/src/param_override.cpp), so the composition's own non-finite branch is not reachable by injection.
void expect_override_refused(const char* what, ParamId id, float x) {
  in_child([=] {
    Scene sc;
    Overrides ov = sc.overrides();
    const Ref r = load_ref(ov);
    ov.f32(id, x);
    EXPECT_EQ(init_with(r, ov), MARV_SIL_E_PARAM) << what;
  });
}

TEST(L11CompositionParams, ANonFiniteGainIsRefusedByTheParameterInterfaceBeforeInit) {
  expect_override_refused("NaN vert_kp_z", ParamId::vert_kp_z, std::numeric_limits<float>::quiet_NaN());
  expect_override_refused("infinite vert_kp_v", ParamId::vert_kp_v, std::numeric_limits<float>::infinity());
  expect_override_refused("NaN vert_ki_v", ParamId::vert_ki_v, std::numeric_limits<float>::quiet_NaN());
  expect_override_refused("NaN vert_hover_thrust_n", ParamId::vert_hover_thrust_n, std::numeric_limits<float>::quiet_NaN());
}

TEST(L11CompositionParams, AnIntegralGainWithoutAProportionalOnePanicsAtInit) {
  expect_init_panics("vert_kp_v zero with the default vert_ki_v", [](Overrides& o) { o.f32(ParamId::vert_kp_v, 0.0F); },
                     "vert_ki_v is nonzero while vert_kp_v is 0");
}

// Control of the death checks above: the same table with only valid values initialises, and the bounds themselves (a
// zero gain, an integral gain with a proportional one) are valid.
TEST(L11CompositionParams, NegativeControlValidValuesInitialise) {
  in_child([] {
    const Scene sc;
    const Overrides ov = sc.overrides();
    const Ref r = load_ref(ov);
    EXPECT_EQ(init_with(r, ov), MARV_SIL_OK);
  });
}

TEST(L11CompositionParams, NegativeControlZeroGainsAndTheBoundsInitialise) {
  in_child([] {
    const Scene sc;
    Overrides ov = sc.overrides();
    ov.f32(ParamId::vert_kp_z, 0.0F);
    ov.f32(ParamId::vert_kp_v, 0.0F);
    ov.f32(ParamId::vert_ki_v, 0.0F);
    const Ref r = load_ref(ov);
    EXPECT_EQ(init_with(r, ov), MARV_SIL_OK);
  });
}

// ---- T2 hover equality --------------------------------------------------------------------------------------------

// Rationale: at the reference (z = z_ref = 0, v_z = 0) with a level truth attitude the whole composition must reproduce
// the card's hover collective through the mixer and the diffuser, bit for bit, for every tick.
TEST(L11CompositionHover, TheCardHoverThrustAtTheReferenceGivesTheHoverAllocationOnEveryTick) {
  in_child([] {
    const Scene sc;
    const SceneRun run = run_scene(sc, kRun);
    const Dshots idle = dshot_of(run.r, 0.0F, Vec3f());
    const Dshots want = dshot_of(run.r, kCardHoverThrustN, Vec3f());
    for (std::size_t m = 0; m < kMotors; ++m) {
      ASSERT_GT(want[m], idle[m]) << "motor " << (m + 1);  // the hover is a real command
    }
    const std::vector<Dshots> expected = hover_rows(run.r, kCardHoverThrustN, kRun);
    std::uint32_t mismatches = 0;
    for (std::uint32_t n = 0; n < kRun; ++n) {
      mismatches += run.rows[n] != expected[n] ? 1U : 0U;
    }
    EXPECT_EQ(mismatches, 0U);
    EXPECT_EQ(first_difference(run.rows, reference(run.r, sc, kRun)), kRun);
    // control: another hover thrust is another command, so the comparison above is not blind to the collective
    const std::vector<Dshots> other = hover_rows(run.r, 2.0F * kCardHoverThrustN, kRun);
    EXPECT_NE(run.rows[0], other[0]);
  });
}

// ---- T3 sign ------------------------------------------------------------------------------------------------------

// Rationale: z is down-positive, so a vehicle below the reference (z > z_ref) must get more collective than the hover one
// and one above it less (law sign convention, fw/vertical/include/marv/vertical/vertical_law.hpp), and at rest at the
// reference the collective is unchanged; each case also matches the reference loop.
void expect_collective_against_hover(float z, float z_ref, int direction) {
  in_child([=] {
    Scene sc;
    sc.segments = {{0, z_ref}};
    sc.feed = [=](std::uint32_t) { return VertState{z, 0.0F, true}; };
    const SceneRun run = run_scene(sc, kRun);
    const std::vector<Dshots> hover = hover_rows(run.r, kCardHoverThrustN, kRun);
    for (std::uint32_t n = 0; n < kRun; ++n) {
      for (std::size_t m = 0; m < kMotors; ++m) {
        if (direction > 0) {
          ASSERT_GT(run.rows[n][m], hover[n][m]) << "tick " << n << " motor " << (m + 1);
        } else if (direction < 0) {
          ASSERT_LT(run.rows[n][m], hover[n][m]) << "tick " << n << " motor " << (m + 1);
        }
      }
      if (direction == 0) {
        ASSERT_EQ(run.rows[n], hover[n]) << "tick " << n;
      }
    }
    EXPECT_EQ(first_difference(run.rows, reference(run.r, sc, kRun)), kRun);
  });
}

TEST(L11CompositionSign, BelowTheReferenceGivesMoreThanTheHoverCollective) { expect_collective_against_hover(kHeightOffsetM, 0.0F, 1); }

TEST(L11CompositionSign, AboveTheReferenceGivesLessThanTheHoverCollective) { expect_collective_against_hover(-kHeightOffsetM, 0.0F, -1); }

TEST(L11CompositionSign, AtRestAtANonzeroReferenceTheCollectiveIsTheHoverOne) {
  expect_collective_against_hover(kHeightOffsetM, kHeightOffsetM, 0);
}

// ---- T4 script stamps ---------------------------------------------------------------------------------------------

// Rationale: segment k applies from the first vertical execution whose stamp is >= t_k (the L5 segment-stamp rule) and not
// before; before the first segment z_ref is 0 m (the S4a rule). The truth height is constant at kHeightOffsetM, so before
// the first segment the error is -kHeightOffsetM; the two segments set other references. Stamps: the first on a vertical
// tick, the second parameterised by `t2_delta_us` around vertical tick `t2_tick`.
void expect_segment_stamps(std::uint32_t t2_tick, std::int32_t t2_delta_us, std::uint32_t first_vertical_tick_of_second) {
  in_child([=] {
    const Ref probe = defaults_ref();
    Scene sc;
    sc.feed = [](std::uint32_t) { return VertState{kHeightOffsetM, 0.0F, true}; };
    constexpr float kRef1 = -kHeightOffsetM;
    constexpr float kRef2 = 2.0F * kHeightOffsetM;
    const std::uint32_t t1_tick = 2 * probe.divisor * probe.vratio;
    sc.segments = {{stamp_i32(probe, t1_tick), kRef1}, {stamp_i32(probe, t2_tick) + t2_delta_us, kRef2}};
    const SceneRun run = run_scene(sc, kRun);
    const Ref& r = run.r;
    ASSERT_EQ(first_vertical_tick_from(r, sc.segments[0].t_us), t1_tick);
    ASSERT_EQ(first_vertical_tick_from(r, sc.segments[1].t_us), first_vertical_tick_of_second);
    EXPECT_EQ(first_difference(run.rows, reference(r, sc, kRun)), kRun);
    // before the first segment z_ref is 0: the first row is the allocation of T_h - k_v (k_z (0 - z) - v_z), no hover
    const float u0 = r.vertical.kp_v * (r.vertical.kp_z * (0.0F - kHeightOffsetM) - 0.0F);
    EXPECT_EQ(run.rows[0], dshot_of(r, kCardHoverThrustN - u0, Vec3f()));
    EXPECT_NE(run.rows[0], dshot_of(r, kCardHoverThrustN, Vec3f()));
    // controls: a segment applied one vertical period early or late, or the first segment's reference absent, differs
    const std::uint32_t vdivisor = r.divisor * r.vratio;
    for (const std::int32_t shift : {-static_cast<std::int32_t>(vdivisor), static_cast<std::int32_t>(vdivisor)}) {
      Scene moved = sc;
      moved.segments[1].t_us = stamp_i32(r, first_vertical_tick_of_second + shift);
      EXPECT_NE(first_difference(run.rows, reference(r, moved, kRun)), kRun) << "second segment moved by " << shift << " ticks";
    }
    Scene without = sc;
    without.segments.erase(without.segments.begin());
    EXPECT_NE(first_difference(run.rows, reference(r, without, kRun)), kRun);
  });
}

TEST(L11CompositionScript, SegmentsApplyAtTheFirstVerticalExecutionAtOrAfterTheirStampOnATick) {
  expect_segment_stamps(12, 0, 12);
}

TEST(L11CompositionScript, ASecondSegmentStampOneMicrosecondAfterATickWaitsForTheNextVerticalExecution) {
  expect_segment_stamps(12, 1, 14);
}

TEST(L11CompositionScript, ASecondSegmentStampOneMicrosecondBeforeATickActsAtThatTick) {
  expect_segment_stamps(12, -1, 12);
}

TEST(L11CompositionScript, ASecondSegmentStampOnAnOddTickActsAtTheNextVerticalExecution) {
  expect_segment_stamps(13, 0, 14);
}

// ---- T5 thrust disturbance ----------------------------------------------------------------------------------------

// Rationale: at the reference the vertical law's error is 0 on every execution and its integral stays 0, so the request
// is T_h; from the first vertical execution whose stamp is >= the disturbance stamp the collective is T_h + d (the added
// constant, decision 0020 section 4), the allocation of {T_h + d, 0} through the diffuser, and before it T_h.
void expect_thrust_disturbance(std::uint32_t t0_tick, std::int32_t t0_delta_us) {
  in_child([=] {
    const Ref probe = defaults_ref();
    Scene sc;
    sc.dist_thrust_n = 0.5F * kCardHoverThrustN;  // scenario test value: half the hover collective, well inside the mixer
    sc.dist_thrust_t_us = stamp_i32(probe, t0_tick) + t0_delta_us;
    const SceneRun run = run_scene(sc, kRun);
    const Ref& r = run.r;
    const std::uint32_t first = first_vertical_tick_from(r, sc.dist_thrust_t_us);
    ASSERT_GT(first, 0U);
    ASSERT_LT(first, kRun);
    ASSERT_NE(dshot_of(r, kCardHoverThrustN + sc.dist_thrust_n, Vec3f()), dshot_of(r, kCardHoverThrustN, Vec3f()));
    const std::vector<Dshots> want = diffused(
        r, kRun, [&](std::uint32_t n) { return n < first ? kCardHoverThrustN : kCardHoverThrustN + sc.dist_thrust_n; },
        [](std::uint32_t) { return Vec3f(); });
    for (std::uint32_t n = 0; n < kRun; ++n) {
      ASSERT_EQ(run.rows[n], want[n]) << "tick " << n << " first affected tick " << first;
    }
    EXPECT_EQ(first_difference(run.rows, reference(r, sc, kRun)), kRun);
    // control: the stamp ignored (the disturbance from tick 0) is another sequence
    const std::vector<Dshots> from_zero = diffused(
        r, kRun, [&](std::uint32_t) { return kCardHoverThrustN + sc.dist_thrust_n; }, [](std::uint32_t) { return Vec3f(); });
    EXPECT_NE(first_difference(run.rows, from_zero), kRun);
  });
}

TEST(L11CompositionThrustDisturbance, SwitchesOnAtItsStampOnAVerticalTick) { expect_thrust_disturbance(12, 0); }

TEST(L11CompositionThrustDisturbance, AStampOneMicrosecondAfterATickWaitsForTheNextVerticalExecution) {
  expect_thrust_disturbance(12, 1);
}

TEST(L11CompositionThrustDisturbance, AStampBetweenTwoVerticalTicksActsAtTheNextOne) { expect_thrust_disturbance(13, 0); }

// ---- T6 torque pulse ----------------------------------------------------------------------------------------------

// Rationale: on each axis the pulse is a torque added to the rate loop's torque before the mixer for t0 <= t < t0 + dur,
// so the DShot differential of the rate ticks inside the window is the allocation of {T_h, tau on that axis}, outside it
// the hover pattern. tau is half the P torque of a full-rate error on that axis (kp * rate_max of the card, a derived
// scale as in the L5 disturbance test). The window starts and ends on rate ticks, so the closed interval at t0 and the
// open one at t0 + dur are both exercised.
void expect_torque_pulse(std::int32_t axis) {
  in_child([=] {
    const Ref probe = defaults_ref();
    Scene sc;
    sc.pulse_axis = axis;
    sc.pulse_nm = 0.5F * probe.p_torque[static_cast<std::size_t>(axis)];
    sc.pulse_t_us = stamp_i32(probe, 12);
    sc.pulse_dur_us = stamp_i32(probe, 24) - stamp_i32(probe, 12);
    const SceneRun run = run_scene(sc, kRun);
    const Ref& r = run.r;
    Vec3f tau;
    tau[static_cast<std::size_t>(axis)] = sc.pulse_nm;
    ASSERT_NE(dshot_of(r, kCardHoverThrustN, tau), dshot_of(r, kCardHoverThrustN, Vec3f()));
    const auto active = [&](std::uint32_t n) {
      return stamp(r, n) >= static_cast<TimeUs>(sc.pulse_t_us) &&
             stamp(r, n) < static_cast<TimeUs>(sc.pulse_t_us) + static_cast<TimeUs>(sc.pulse_dur_us);
    };
    const std::vector<Dshots> want = diffused(
        r, kRun, [](std::uint32_t) { return kCardHoverThrustN; }, [&](std::uint32_t n) { return active(n) ? tau : Vec3f(); });
    for (std::uint32_t n = 0; n < kRun; ++n) {
      ASSERT_EQ(run.rows[n], want[n]) << "tick " << n << " axis " << axis;
    }
    ASSERT_TRUE(active(12));
    ASSERT_TRUE(active(22));
    ASSERT_FALSE(active(10));
    ASSERT_FALSE(active(24));
    EXPECT_EQ(first_difference(run.rows, reference(r, sc, kRun)), kRun);
    // control: the pulse on another axis, and the pulse over the whole run, are other sequences
    Vec3f other_tau;
    other_tau[static_cast<std::size_t>((axis + 1) % 3)] = sc.pulse_nm;
    const std::vector<Dshots> other_axis = diffused(
        r, kRun, [](std::uint32_t) { return kCardHoverThrustN; }, [&](std::uint32_t n) { return active(n) ? other_tau : Vec3f(); });
    EXPECT_NE(first_difference(run.rows, other_axis), kRun);
    const std::vector<Dshots> always = diffused(
        r, kRun, [](std::uint32_t) { return kCardHoverThrustN; }, [&](std::uint32_t) { return tau; });
    EXPECT_NE(first_difference(run.rows, always), kRun);
  });
}

TEST(L11CompositionTorquePulse, RollAxis) { expect_torque_pulse(0); }

TEST(L11CompositionTorquePulse, PitchAxis) { expect_torque_pulse(1); }

TEST(L11CompositionTorquePulse, YawAxis) { expect_torque_pulse(2); }

// ---- T7 group order and subset ------------------------------------------------------------------------------------

// Rationale: with vert_loop_ratio = 2 the vertical group runs every 2 * rate_loop_divisor ticks, a subset of the rate
// ticks (decision 0020 section 1), so the thrust request changes only on every second rate tick and is held in between;
// the vertical group runs before the rate group, so the rate execution at a vertical tick already uses that tick's new
// request (the control reference with the vertical group after the rate group differs from tick 0 on). The truth height
// ramps on every tick, so any extra vertical execution would show.
TEST(L11CompositionGroups, VerticalRatioTwoChangesTheRequestOnEverySecondRateTickAndHoldsItBetween) {
  in_child([] {
    Scene sc;
    sc.vratio = 2;
    sc.feed = [](std::uint32_t n) { return VertState{kRampM * static_cast<float>(n), 0.0F, true}; };
    const SceneRun run = run_scene(sc, kRun);
    const Ref& r = run.r;
    ASSERT_EQ(r.vratio, 2U);
    ASSERT_GE(r.divisor, 1U);
    const std::uint32_t vdivisor = r.divisor * r.vratio;
    // Rows between rate ticks are the last write. A held request is written again at the next rate tick; the diffuser's
    // carry is at most 1/2 after every write (decision 0017), so the same request differs by at most 1 count per motor,
    // while a new request after one vertical period of the ramp moves the collective by more.
    const auto max_step = [&](std::uint32_t a, std::uint32_t b) {
      int worst = 0;
      for (std::size_t m = 0; m < kMotors; ++m) {
        worst = std::max(worst, std::abs(static_cast<int>(run.rows[a][m]) - static_cast<int>(run.rows[b][m])));
      }
      return worst;
    };
    for (std::uint32_t n = 0; n < kRun; ++n) {
      const std::uint32_t last_rate = n - (n % r.divisor);
      ASSERT_EQ(run.rows[n], run.rows[last_rate]) << "tick " << n << ": no write between rate ticks";
      if (last_rate >= r.divisor) {
        if (last_rate % vdivisor != 0) {
          ASSERT_LE(max_step(last_rate, last_rate - r.divisor), 1) << "tick " << n << ": rate tick with no vertical execution";
        } else {
          ASSERT_GE(max_step(last_rate, last_rate - r.divisor), 2) << "tick " << n << ": vertical tick";
        }
      }
    }
    EXPECT_EQ(first_difference(run.rows, reference(r, sc, kRun)), kRun);
    // controls: vertical after rate, and a vertical execution on every rate tick
    EXPECT_EQ(first_difference(run.rows, reference(r, sc, kRun, Variant{true, 0})), 0U);
    EXPECT_NE(first_difference(run.rows, reference(r, sc, kRun, Variant{false, 1})), kRun);
  });
}

TEST(L11CompositionGroups, VerticalRatioOneRunsTheVerticalGroupOnEveryRateTickBeforeTheRateGroup) {
  in_child([] {
    Scene sc;
    sc.feed = [](std::uint32_t n) { return VertState{kRampM * static_cast<float>(n), 0.0F, true}; };
    const SceneRun run = run_scene(sc, kRun);
    const Ref& r = run.r;
    for (std::uint32_t n = r.divisor; n < kRun; n += r.divisor) {
      ASSERT_NE(run.rows[n], run.rows[n - r.divisor]) << "tick " << n << ": a vertical execution at every rate tick";
    }
    EXPECT_EQ(first_difference(run.rows, reference(r, sc, kRun)), kRun);
    EXPECT_EQ(first_difference(run.rows, reference(r, sc, kRun, Variant{true, 0})), 0U);
  });
}

// ---- T11 anti-windup wiring ---------------------------------------------------------------------------------------

// Rationale (decision 0020 I4): the composition records the mixer's allocation of each rate execution in the law
// (record_allocation), so the integrator freezes while the collective is clamped and e_v drives further into the limit.
// The truth height is kFarBelowM below the reference for kClampTicks ticks, so the request exceeds the mixer's zero-torque
// limit with e_v < 0 throughout, then returns to the reference. Frozen, the integrator stays 0 and the collective after
// the window is the hover one (within the diffuser's 1 count); wound up it is not. The control reference skips
// record_allocation and must differ after the window.
constexpr float kFarBelowM = 20.0F;        // scenario test value: m; the request is about 173 N against a limit near 49 N
constexpr std::uint32_t kClampTicks = 40;  // scenario test value: ticks, 20 vertical executions at the product divisors

TEST(L11CompositionAntiWindup, AClampedCollectiveFreezesTheIntegratorSoTheCollectiveReturnsToHover) {
  in_child([] {
    Scene sc;
    sc.feed = [](std::uint32_t n) { return VertState{n < kClampTicks ? kFarBelowM : 0.0F, 0.0F, true}; };
    const SceneRun run = run_scene(sc, kRun);
    const Ref& r = run.r;
    const float request = kCardHoverThrustN + r.vertical.kp_v * r.vertical.kp_z * kFarBelowM;
    ASSERT_LT(mixer::allocate(r.mixer, mixer::Request<float>{request, Vec3f()}).achieved_thrust, request);  // clamped
    EXPECT_EQ(first_difference(run.rows, reference(r, sc, kRun)), kRun);
    const std::vector<Dshots> hover = hover_rows(r, kCardHoverThrustN, kRun);
    const auto max_step = [](const Dshots& a, const Dshots& b) {
      int worst = 0;
      for (std::size_t m = 0; m < kMotors; ++m) {
        worst = std::max(worst, std::abs(static_cast<int>(a[m]) - static_cast<int>(b[m])));
      }
      return worst;
    };
    for (std::uint32_t n = kClampTicks; n < kRun; ++n) {
      ASSERT_LE(max_step(run.rows[n], hover[n]), 1) << "tick " << n;
    }
    // control: without record_allocation the integrator winds up through the window and the rows differ after it
    Variant wound;
    wound.record_allocation = false;
    const std::vector<Dshots> unfrozen = reference(r, sc, kRun, wound);
    EXPECT_GE(first_difference(run.rows, unfrozen), kClampTicks);
    EXPECT_LT(first_difference(run.rows, unfrozen), kRun);
    for (std::uint32_t n = kClampTicks; n < kRun; ++n) {
      ASSERT_GE(max_step(unfrozen[n], hover[n]), 2) << "tick " << n;
    }
  });
}

// ---- T8 stale latches ---------------------------------------------------------------------------------------------

constexpr const char* kStaleVertical =
    "hal_panic: l11_vertical_scripted composition: the latched vertical stamp differs from the sample stamp";
constexpr const char* kStaleAttitude =
    "hal_panic: l11_vertical_scripted composition: the latched attitude stamp differs from the sample stamp";

// Runs ticks [0, k) with the withheld hand-overs of `w`, with ratio `vratio` when positive.
void run_withholding(Withhold w, std::int32_t vratio, std::uint32_t k) {
  Scene sc;
  sc.vratio = vratio;
  const Overrides ov = sc.overrides();
  const Ref r = load_ref(ov);
  if (init_with(r, ov) != MARV_SIL_OK) {
    std::exit(1);
  }
  (void)run_sil(sc, k, w);
  std::exit(0);
}

// Rationale: a vertical tick without a vertical state of its own tick (the latch is stale) must panic with the
// composition's message rather than fly on the previous state (decision 0006 B freshness, decision 0020 I5).
TEST(L11CompositionFreshness, AVerticalTickWithoutAStateForItPanicsNamingTheStamp) {
  const Ref probe = defaults_ref();
  const std::uint32_t second_vertical_tick = probe.divisor * probe.vratio;
  EXPECT_EXIT(
      {
        disable_core_dumps();
        Withhold w;
        w.vertical_tick = second_vertical_tick;
        run_withholding(w, 0, second_vertical_tick + 1);
      },
      ::testing::KilledBySignal(SIGABRT), kStaleVertical);
}

TEST(L11CompositionFreshness, NoVerticalStateEverHandedOverPanicsAtTheFirstVerticalTick) {
  EXPECT_EXIT(
      {
        disable_core_dumps();
        Withhold w;
        w.vertical_always = true;
        run_withholding(w, 0, 1);
      },
      ::testing::KilledBySignal(SIGABRT), kStaleVertical);
}

TEST(L11CompositionFreshness, AnAttitudeTickWithoutAStateForItStillPanicsNamingTheStamp) {
  const Ref probe = defaults_ref();
  const std::uint32_t second_attitude_tick = probe.divisor * default_i32(ParamId::att_loop_ratio);
  EXPECT_EXIT(
      {
        disable_core_dumps();
        Withhold w;
        w.attitude_tick = second_attitude_tick;
        run_withholding(w, 0, second_attitude_tick + 1);
      },
      ::testing::KilledBySignal(SIGABRT), kStaleAttitude);
}

// Control of the death checks above: the same runs with every hand-over present do not panic.
TEST(L11CompositionFreshness, NegativeControlAStateForEveryTickRunsThroughTheSecondVerticalAndAttitudeTick) {
  EXPECT_EXIT(
      {
        disable_core_dumps();
        run_withholding(Withhold{}, 0, kRun);
      },
      ::testing::ExitedWithCode(0), "");
}

// A vertical state is needed only on vertical ticks: with vert_loop_ratio = 2 the ticks between them need none, and the
// first tick of the next vertical group does.
TEST(L11CompositionFreshness, NoVerticalStateIsNeededOnTicksWithoutAVerticalExecution) {
  const Ref probe = defaults_ref();
  ASSERT_GE(probe.divisor, 1U);
  EXPECT_EXIT(
      {
        disable_core_dumps();
        Withhold w;
        w.vertical_tick = probe.divisor;  // a rate tick with no vertical execution at ratio 2 (vertical ticks every 2 rate ticks)
        run_withholding(w, 2, 3 * probe.divisor);
      },
      ::testing::ExitedWithCode(0), "");
  EXPECT_EXIT(
      {
        disable_core_dumps();
        Withhold w;
        w.vertical_tick = 2 * probe.divisor;  // the vertical tick at ratio 2 (the second vertical group), run one tick into it
        run_withholding(w, 2, 2 * probe.divisor + 1);
      },
      ::testing::KilledBySignal(SIGABRT), kStaleVertical);
}

// ---- T9 fallback --------------------------------------------------------------------------------------------------

// Rationale: an invalid vertical state (valid bit clear) makes the law's output exactly T_h (decision 0020 I5): the
// DShot is the hover allocation through the diffuser, the run continues, and a later valid state resumes the law (from a
// zero integral). A non-finite z with the valid bit set is refused by the truth entry, so it never reaches the
// composition (S1's rule); the status is asserted. The diffuser carries rounding error between writes, so the rows equal
// the closed-form hover rows (a fresh diffuser) only when the window starts at tick 0; a window in mid-run is compared with
// the reference loop, which runs the same diffuser through it.
constexpr std::uint32_t kInvalidTo = 24;  // scenario test value: ticks, a vertical tick at the product divisors

TEST(L11CompositionFallback, AnInvalidVerticalStateFromTheFirstTickGivesExactlyTheHoverCollectiveAndTheRunContinues) {
  in_child([] {
    Scene sc;
    sc.feed = [](std::uint32_t n) { return VertState{kHeightOffsetM, 0.0F, n >= kInvalidTo}; };
    const SceneRun run = run_scene(sc, kRun);
    const Ref& r = run.r;
    ASSERT_EQ(run.rows.size(), kRun);
    const std::vector<Dshots> hover = hover_rows(r, kCardHoverThrustN, kRun);
    for (std::uint32_t n = 0; n < kInvalidTo; ++n) {
      ASSERT_EQ(run.rows[n], hover[n]) << "tick " << n;
    }
    // the law resumes: the offset moves the collective after the window
    EXPECT_NE(run.rows[kRun - 1], hover[kRun - 1]);
    EXPECT_EQ(first_difference(run.rows, reference(r, sc, kRun)), kRun);
    // control: the same truth state valid throughout is not the hover collective inside the window
    Scene valid_scene = sc;
    valid_scene.feed = [](std::uint32_t) { return VertState{kHeightOffsetM, 0.0F, true}; };
    const std::vector<Dshots> flown = reference(r, valid_scene, kRun);
    EXPECT_NE(flown[0], hover[0]);
  });
}

TEST(L11CompositionFallback, AnInvalidWindowInMidRunMatchesTheReferenceAndTheRunContinues) {
  in_child([] {
    const std::uint32_t invalid_from = 16;  // scenario test values: ticks, vertical ticks at the product divisors
    const std::uint32_t invalid_to = 40;
    Scene sc;
    sc.feed = [=](std::uint32_t n) { return VertState{kHeightOffsetM, 0.0F, n < invalid_from || n >= invalid_to}; };
    const SceneRun run = run_scene(sc, kRun);
    const Ref& r = run.r;
    ASSERT_EQ(run.rows.size(), kRun);
    EXPECT_EQ(first_difference(run.rows, reference(r, sc, kRun)), kRun);
    // control: without the window the rows differ inside it
    Scene valid_scene = sc;
    valid_scene.feed = [](std::uint32_t) { return VertState{kHeightOffsetM, 0.0F, true}; };
    EXPECT_LT(first_difference(run.rows, reference(r, valid_scene, kRun)), kRun);
  });
}

TEST(L11CompositionFallback, ANonFiniteHeightIsRefusedByTheTruthEntryAndNeverReachesTheComposition) {
  in_child([] {
    const Scene sc;
    const Overrides ov = sc.overrides();
    const Ref r = load_ref(ov);
    ASSERT_EQ(init_with(r, ov), MARV_SIL_OK);
    const marv_truth_state a = level_truth(0);
    ASSERT_EQ(marv_truth_state_set(&a), MARV_SIL_OK);
    marv_truth_vertical v = vertical_truth(0, VertState{});
    v.z_ned_m = std::numeric_limits<float>::quiet_NaN();
    EXPECT_EQ(marv_truth_vertical_set(&v), MARV_SIL_E_INPUT);
    v.z_ned_m = 0.0F;
    EXPECT_EQ(marv_truth_vertical_set(&v), MARV_SIL_OK);  // control: the same entry with a finite z
  });
}

}  // namespace
