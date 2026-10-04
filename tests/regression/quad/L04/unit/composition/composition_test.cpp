// The l4_rate_scripted composition through the SIL C ABI (marv_sil.h): the hover collective with a zero setpoint and a
// valid zero gyro gives allocate({T_hover, 0}) through the DShot diffuser (decision 0017: mixer::DshotDiffuser from init,
// one write per due tick; thrust_to_dshot's stateless rounding is the control); the rate group writes only on its due
// ticks (the HAL latch holds the command in between); a scripted setpoint step raises the motors the L3 sign contract
// names; a chirp and a segment stamp reach the request exactly; a cleared GyroValid is the rate loop's fault path (zero
// torque, the pure-thrust allocation) while a valid nonzero gyro moves the output (the control of the truth-gyro fault
// check of decision 0005); invalid scenario parameters panic at init.
//
// marv_sil_init succeeds once per process, so every test that initialises runs in a forked child. The child also builds
// the reference: the default parameter table of the composition's set (params_init) feeds mixer::load_config here, in
// this executable, apart from the SIL library's own hidden copy. The composition's parameter runtime is listed before
// marv_mixer in the link, so the one param_get in this executable is that set's (the product ids are a prefix of it).
//
// The hover thrust is m g with m the card mass (parameter `mass`) and g = the WGS 84 equatorial normal gravity of
// constants.hpp (NIMA TR8350.2 Table 3.4); Gazebo runs use g at the site, the test only needs a valid collective.
#include <gtest/gtest.h>

#include <marv_sil.h>

#include <sys/prctl.h>

#include <algorithm>
#include <array>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include <marv/composition.hpp>
#include <marv/l4_script.hpp>
#include <marv/mixer/mixer.hpp>
#include <marv/params/param.hpp>
#include <marv/params/param_ids.hpp>
#include <marv/prim/constants.hpp>
#include <marv/types/actuator.hpp>

namespace {

using namespace marv;

constexpr std::size_t kMotors = kQuadXMotors;
using Dshots = std::array<std::uint16_t, kMotors>;
constexpr std::uint32_t kGyroValid = 1u << MARV_IMU_GYRO_VALID;

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

// ---- reference and SIL plumbing ----------------------------------------------------------------------------------

struct Ref {
  mixer::MixerConfig<float> cfg;
  float mass = 0.0F;
  float hover = 0.0F;
  std::uint32_t tick_num_us = 0;
  std::uint32_t tick_den = 0;
  std::uint32_t divisor = 0;
  float rate_max_roll = 0.0F;
  float rate_max_pitch = 0.0F;
  float rate_max_yaw = 0.0F;
  float kp_roll = 0.0F;
  std::array<std::int32_t, kMotors> yaw_sign{};
};

Ref load_ref() {
  Ref r;
  if (!params_init(param_defaults())) {
    std::fprintf(stderr, "params_init failed\n");
    std::exit(1);
  }
  r.cfg = mixer::load_config();
  r.mass = param_value<ParamId::mass>();
  r.hover = r.mass * static_cast<float>(prim::kWgs84GammaE);
  r.tick_num_us = static_cast<std::uint32_t>(param_value<ParamId::tick_period_num_us>());
  r.tick_den = static_cast<std::uint32_t>(param_value<ParamId::tick_period_den>());
  r.divisor = static_cast<std::uint32_t>(param_value<ParamId::rate_loop_divisor>());
  r.rate_max_roll = param_value<ParamId::rate_max_roll>();
  r.rate_max_pitch = param_value<ParamId::rate_max_pitch>();
  r.rate_max_yaw = param_value<ParamId::rate_max_yaw>();
  r.kp_roll = param_value<ParamId::rate_kp_roll>();
  r.yaw_sign = {param_value<ParamId::rotor_yaw_sign_m1>(), param_value<ParamId::rotor_yaw_sign_m2>(),
                param_value<ParamId::rotor_yaw_sign_m3>(), param_value<ParamId::rotor_yaw_sign_m4>()};
  return r;
}

Dshots dshot_of(const Ref& r, float thrust, const prim::Vec3<float>& torque) {
  const auto f = mixer::allocate(r.cfg, mixer::Request<float>{thrust, torque}).f;
  const auto d = mixer::thrust_to_dshot(r.cfg, f);
  Dshots out{};
  for (std::size_t i = 0; i < kMotors; ++i) {
    out[i] = d[i].raw();
  }
  return out;
}

// The composition's DShot per tick from init (decision 0017): mixer::DshotDiffuser on allocate({thrust, torque(n)}) at
// every due tick n (n % rate_loop_divisor == 0, tick 0 included), the command held in between.
template <class Torque>
std::vector<Dshots> diffused(const Ref& r, float thrust, std::uint32_t k, Torque torque) {
  mixer::DshotDiffuser<float> diffuser;
  diffuser.init(r.cfg);
  std::vector<Dshots> rows(k);
  Dshots held{};
  for (std::uint32_t n = 0; n < k; ++n) {
    if ((n % r.divisor) == 0) {
      const auto d = diffuser.apply(mixer::allocate(r.cfg, mixer::Request<float>{thrust, torque(n)}).f);
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
  return diffused(r, thrust, k, [](std::uint32_t) { return prim::Vec3<float>(); });
}

// Control: some row differs from the constant stateless command (thrust_to_dshot at every write).
bool differs_from_stateless(const std::vector<Dshots>& rows, const Dshots& stateless) {
  return std::any_of(rows.begin(), rows.end(), [&](const Dshots& d) { return d != stateless; });
}

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

TimeUs stamp(const Ref& r, std::uint64_t n) {
  return static_cast<TimeUs>(static_cast<unsigned __int128>(n) * r.tick_num_us / r.tick_den);
}

marv_imu_meas valid_gyro(float x, float y, float z) {
  marv_imu_meas m{};
  m.gyro_rad_s = {x, y, z};
  m.flags = kGyroValid;
  return m;
}

// Ticks [0, k) with the same sample each tick; the rows of DShot per tick.
std::vector<Dshots> run(std::uint32_t k, const marv_imu_meas& sample) {
  std::vector<marv_imu_meas> s(k, sample);
  std::vector<std::uint64_t> t(k, 0);
  std::vector<std::uint16_t> d(static_cast<std::size_t>(k) * kMotors, 0);
  marv_sil_out out{};
  out.struct_size = sizeof(marv_sil_out);
  out.capacity_ticks = k;
  out.t_us = t.data();
  out.dshot = d.data();
  out.servo_us = nullptr;
  EXPECT_EQ(marv_sil_tick(0, k, s.data(), &out), MARV_SIL_OK);
  std::vector<Dshots> rows(k);
  for (std::uint32_t i = 0; i < k; ++i) {
    for (std::size_t m = 0; m < kMotors; ++m) {
      rows[i][m] = d[(static_cast<std::size_t>(i) * kMotors) + m];
    }
  }
  return rows;
}

// A one-segment roll/pitch/yaw step from stamp 0 (scenario test value: the maximum rate of the axis, QF-1).
void add_step(Overrides& ov, std::size_t axis, const Ref& r) {
  ov.i32(ParamId::l4_seg_count, 1);
  ov.i32(ParamId::l4_seg1_t_us, 0);
  const std::array<ParamId, 3> ids{ParamId::l4_seg1_roll, ParamId::l4_seg1_pitch, ParamId::l4_seg1_yaw};
  const std::array<float, 3> rates{r.rate_max_roll, r.rate_max_pitch, r.rate_max_yaw};
  ov.f32(ids[axis], rates[axis]);
}

// scenario test value: run length in ticks; long enough for the prefilter to converge on its step
constexpr std::uint32_t kRun = 4096;

// ---- info ---------------------------------------------------------------------------------------------------------

TEST(L4Composition, ReportsTheCompositionFourMotorsNoServosAndItsParameterCount) {
  marv_sil_info info{};
  info.struct_size = sizeof(info);
  ASSERT_EQ(marv_sil_info_get(&info), MARV_SIL_OK);
  EXPECT_EQ(info.n_motors, kMotors);
  EXPECT_EQ(info.n_servos, 0u);
  EXPECT_STREQ(info.composition, "l4_rate_scripted");
  EXPECT_EQ(info.n_params, kParamCount);
  EXPECT_EQ(info.param_schema_hash, kParamSchemaHash);
}

// ---- hover --------------------------------------------------------------------------------------------------------

TEST(L4Composition, HoverThrustZeroGyroZeroSetpointGivesThePureThrustAllocation) {
  in_child([] {
    const Ref r = load_ref();
    Overrides ov;
    ov.f32(ParamId::l4_thrust_n, r.hover);
    ASSERT_EQ(init_with(r, ov), MARV_SIL_OK);
    const Dshots want = dshot_of(r, r.hover, prim::Vec3<float>());
    // the hover is a real command: above idle on every motor
    const Dshots idle = dshot_of(r, 0.0F, prim::Vec3<float>());
    for (std::size_t m = 0; m < kMotors; ++m) {
      ASSERT_GT(want[m], idle[m]) << "motor " << (m + 1);
    }
    const std::vector<Dshots> rows = run(kRun, valid_gyro(0, 0, 0));
    const std::vector<Dshots> diffused_want = pure_thrust_rows(r, r.hover, kRun);
    for (std::uint32_t i = 0; i < kRun; ++i) {
      ASSERT_EQ(rows[i], diffused_want[i]) << "tick " << i;
    }
    EXPECT_TRUE(differs_from_stateless(rows, want)) << "control: stateless rounding reproduces the composition";
  });
}

TEST(L4Composition, ThePureThrustAllocationDependsOnTheThrustParameter) {
  in_child([] {
    const Ref r = load_ref();
    Overrides ov;
    ov.f32(ParamId::l4_thrust_n, 1.5F * r.hover / 2.0F);  // scenario test value: 3/4 of the hover collective
    ASSERT_EQ(init_with(r, ov), MARV_SIL_OK);
    const Dshots hover = dshot_of(r, r.hover, prim::Vec3<float>());
    const Dshots want = dshot_of(r, 1.5F * r.hover / 2.0F, prim::Vec3<float>());
    ASSERT_NE(want, hover);
    const std::vector<Dshots> rows = run(16, valid_gyro(0, 0, 0));
    const std::vector<Dshots> diffused_want = pure_thrust_rows(r, 1.5F * r.hover / 2.0F, 16);
    for (std::size_t i = 0; i < rows.size(); ++i) {
      ASSERT_EQ(rows[i], diffused_want[i]) << "tick " << i;
    }
  });
}

// ---- writes only on due ticks -------------------------------------------------------------------------------------

// A roll step against a zero gyro moves the command at nearly every execution. With divisor d the command may change
// only at ticks that are multiples of d and must change at some of them.
void expect_writes_on_due_ticks_only(std::int32_t divisor, bool override_divisor) {
  in_child([=] {
    const Ref r = load_ref();
    Overrides ov;
    ov.f32(ParamId::l4_thrust_n, r.hover);
    add_step(ov, 0, r);
    if (override_divisor) {
      ov.i32(ParamId::rate_loop_divisor, divisor);
    }
    ASSERT_EQ(init_with(r, ov), MARV_SIL_OK);
    const auto d = static_cast<std::uint32_t>(override_divisor ? divisor : static_cast<std::int32_t>(r.divisor));
    const std::vector<Dshots> rows = run(kRun, valid_gyro(0, 0, 0));
    std::uint32_t changes_on_due = 0;
    std::uint32_t changes_off_due = 0;
    for (std::uint32_t i = 1; i < kRun; ++i) {
      if (rows[i] != rows[i - 1]) {
        ((i % d) == 0 ? changes_on_due : changes_off_due) += 1;
      }
    }
    EXPECT_EQ(changes_off_due, 0u) << "divisor " << d;
    EXPECT_GT(changes_on_due, 4u) << "divisor " << d;  // scenario test value: the channel shows the writes at all
  });
}

TEST(L4CompositionWrites, OnlyOnDueTicksWithTheProductDivisor) { expect_writes_on_due_ticks_only(0, false); }

TEST(L4CompositionWrites, OnlyOnDueTicksWithDivisorThree) { expect_writes_on_due_ticks_only(3, true); }

// The control: with divisor 1 every tick is due and the command changes on odd ticks too, so the check above would
// have caught a write on a non-due tick.
TEST(L4CompositionWrites, NegativeControlDivisorOneChangesTheCommandOnOddTicks) {
  in_child([] {
    const Ref r = load_ref();
    Overrides ov;
    ov.f32(ParamId::l4_thrust_n, r.hover);
    add_step(ov, 0, r);
    ov.i32(ParamId::rate_loop_divisor, 1);
    ASSERT_EQ(init_with(r, ov), MARV_SIL_OK);
    const std::vector<Dshots> rows = run(kRun, valid_gyro(0, 0, 0));
    std::uint32_t odd_changes = 0;
    for (std::uint32_t i = 1; i < kRun; i += 2) {
      odd_changes += rows[i] != rows[i - 1] ? 1 : 0;
    }
    EXPECT_GT(odd_changes, 4u);
  });
}

// ---- scripted setpoint: sign per axis ----------------------------------------------------------------------------

void expect_step_signs(std::size_t axis, const std::array<bool, kMotors>& raised) {
  in_child([=] {
    const Ref r = load_ref();
    std::array<bool, kMotors> want_raised = raised;
    if (axis == 2) {
      for (std::size_t m = 0; m < kMotors; ++m) {
        want_raised[m] = r.yaw_sign[m] == 1;
      }
    }
    Overrides ov;
    ov.f32(ParamId::l4_thrust_n, r.hover);
    add_step(ov, axis, r);
    ASSERT_EQ(init_with(r, ov), MARV_SIL_OK);
    const Dshots base = dshot_of(r, r.hover, prim::Vec3<float>());
    const std::vector<Dshots> rows = run(kRun, valid_gyro(0, 0, 0));
    const Dshots& last = rows.back();
    for (std::size_t m = 0; m < kMotors; ++m) {
      if (want_raised[m]) {
        EXPECT_GT(last[m], base[m]) << "axis " << axis << " logical motor " << (m + 1) << " should be raised";
      } else {
        EXPECT_LT(last[m], base[m]) << "axis " << axis << " logical motor " << (m + 1) << " should be lowered";
      }
    }
  });
}

TEST(L4CompositionSign, ARollStepRaisesMotorsTwoAndThree) { expect_step_signs(0, {false, true, true, false}); }

TEST(L4CompositionSign, APitchStepRaisesMotorsOneAndThree) { expect_step_signs(1, {true, false, true, false}); }

TEST(L4CompositionSign, AYawStepRaisesTheYawSignPlusOneMotors) { expect_step_signs(2, {}); }

// ---- stamps and counts reach the request -------------------------------------------------------------------------

TEST(L4CompositionScript, ASegmentActsFromItsStampNotBefore) {
  in_child([] {
    const Ref r = load_ref();
    constexpr std::uint64_t kStepTick = 1024;  // scenario test value: a tick well after the first execution
    Overrides ov;
    ov.f32(ParamId::l4_thrust_n, r.hover);
    ov.i32(ParamId::l4_seg_count, 1);
    ov.i32(ParamId::l4_seg1_t_us, static_cast<std::int32_t>(stamp(r, kStepTick)));
    ov.f32(ParamId::l4_seg1_roll, r.rate_max_roll);
    ASSERT_EQ(init_with(r, ov), MARV_SIL_OK);
    const std::vector<Dshots> base = pure_thrust_rows(r, r.hover, kRun);
    const std::vector<Dshots> rows = run(kRun, valid_gyro(0, 0, 0));
    for (std::uint64_t i = 0; i < kStepTick; ++i) {
      ASSERT_EQ(rows[i], base[i]) << "tick " << i << " precedes the segment";
    }
    EXPECT_NE(rows.back(), base.back());
  });
}

TEST(L4CompositionScript, SegmentsAreIgnoredWhileTheCountIsZero) {
  in_child([] {
    const Ref r = load_ref();
    Overrides ov;
    ov.f32(ParamId::l4_thrust_n, r.hover);
    ov.i32(ParamId::l4_seg1_t_us, 0);
    ov.f32(ParamId::l4_seg1_roll, r.rate_max_roll);
    ASSERT_EQ(init_with(r, ov), MARV_SIL_OK);
    const std::vector<Dshots> base = pure_thrust_rows(r, r.hover, 256);
    const std::vector<Dshots> rows = run(256, valid_gyro(0, 0, 0));
    for (std::size_t i = 0; i < rows.size(); ++i) {
      ASSERT_EQ(rows[i], base[i]) << "tick " << i;
    }
  });
}

TEST(L4CompositionScript, TheSecondSegmentReplacesTheFirst) {
  in_child([] {
    const Ref r = load_ref();
    constexpr std::uint64_t kSecondTick = 2048;  // scenario test value
    Overrides ov;
    ov.f32(ParamId::l4_thrust_n, r.hover);
    ov.i32(ParamId::l4_seg_count, 2);
    ov.i32(ParamId::l4_seg1_t_us, 0);
    ov.f32(ParamId::l4_seg1_roll, r.rate_max_roll);
    ov.i32(ParamId::l4_seg2_t_us, static_cast<std::int32_t>(stamp(r, kSecondTick)));
    ov.f32(ParamId::l4_seg2_roll, -r.rate_max_roll);
    ASSERT_EQ(init_with(r, ov), MARV_SIL_OK);
    const Dshots base = dshot_of(r, r.hover, prim::Vec3<float>());
    const std::vector<Dshots> rows = run(kRun, valid_gyro(0, 0, 0));
    // roll up before the second segment (motor 2 raised), roll down after it (motor 2 lowered)
    EXPECT_GT(rows[kSecondTick - 1][1], base[1]);
    EXPECT_LT(rows.back()[1], base[1]);
  });
}

// ---- chirp --------------------------------------------------------------------------------------------------------

// The request is chirp_torque on the chirp axis with a zero rate-loop torque (zero gyro and setpoint), so the DShot at
// every due tick is exactly the allocation of {hover, chirp(t)} through the diffuser, held in between; outside the window
// the request is the pure-thrust one, so before the window the rows are the pure-thrust rows.
TEST(L4CompositionChirp, TheChirpReachesTheRequestOnItsAxisInsideItsWindowOnly) {
  in_child([] {
    const Ref r = load_ref();
    composition::Chirp<float> c;
    c.axis = composition::ChirpAxis::Roll;
    c.amp = r.kp_roll * r.rate_max_roll;  // scenario test value: the P torque of a full-stick roll error
    c.w_lo = 20.0F;                       // scenario test values: 20 to 200 rad/s, 0.2 s, from tick 512
    c.w_hi = 200.0F;
    c.t0_us = static_cast<std::int32_t>(stamp(r, 512));
    c.dur_us = 200000;
    Overrides ov;
    ov.f32(ParamId::l4_thrust_n, r.hover);
    ov.i32(ParamId::l4_chirp_axis, static_cast<std::int32_t>(c.axis));
    ov.f32(ParamId::l4_chirp_amp_nm, c.amp);
    ov.f32(ParamId::l4_chirp_w_lo, c.w_lo);
    ov.f32(ParamId::l4_chirp_w_hi, c.w_hi);
    ov.i32(ParamId::l4_chirp_t0_us, c.t0_us);
    ov.i32(ParamId::l4_chirp_dur_us, c.dur_us);
    ASSERT_EQ(init_with(r, ov), MARV_SIL_OK);
    const std::uint32_t k = 3000;  // covers the window: 512 ticks + 200000 us / 156.25 us = 1792 < 3000
    const std::vector<Dshots> base = pure_thrust_rows(r, r.hover, k);
    const std::vector<Dshots> rows = run(k, valid_gyro(0, 0, 0));
    const auto inside = [&](std::uint32_t i) {
      const TimeUs t = stamp(r, i);
      return t >= static_cast<TimeUs>(c.t0_us) && t < static_cast<TimeUs>(c.t0_us) + static_cast<TimeUs>(c.dur_us);
    };
    const std::vector<Dshots> want = diffused(r, r.hover, k, [&](std::uint32_t i) {
      return inside(i) ? composition::chirp_torque(c, stamp(r, i)) : prim::Vec3<float>();
    });
    std::uint32_t inside_differs = 0;
    Dshots held_stateless{};
    bool stateless_differs = false;
    for (std::uint32_t i = 0; i < k; ++i) {
      if ((i % r.divisor) == 0) {
        held_stateless = dshot_of(r, r.hover, composition::chirp_torque(c, stamp(r, i)));
      }
      stateless_differs = stateless_differs || rows[i] != held_stateless;
      ASSERT_EQ(rows[i], want[i]) << "tick " << i;
      if (!inside(i)) {
        if (stamp(r, i) < static_cast<TimeUs>(c.t0_us)) {
          ASSERT_EQ(rows[i], base[i]) << "tick " << i << " precedes the window";
        }
      } else if (rows[i] != base[i]) {
        ++inside_differs;
      }
    }
    EXPECT_GT(inside_differs, 100u);  // scenario test value: the chirp moves the command on many ticks
    EXPECT_TRUE(stateless_differs) << "control: stateless rounding reproduces the composition";
  });
}

// ---- fault path ---------------------------------------------------------------------------------------------------

TEST(L4CompositionFault, AClearedGyroValidFlagGivesZeroTorqueEvenWithAStepScripted) {
  in_child([] {
    const Ref r = load_ref();
    Overrides ov;
    ov.f32(ParamId::l4_thrust_n, r.hover);
    add_step(ov, 0, r);
    ASSERT_EQ(init_with(r, ov), MARV_SIL_OK);
    const Dshots base = dshot_of(r, r.hover, prim::Vec3<float>());
    const std::vector<Dshots> rows = run(kRun, marv_imu_meas{});  // GyroValid clear, zero fields
    // the fault path writes through the diffuser and does not reset it (decision 0017)
    const std::vector<Dshots> diffused_base = pure_thrust_rows(r, r.hover, kRun);
    for (std::uint32_t i = 0; i < kRun; ++i) {
      ASSERT_EQ(rows[i], diffused_base[i]) << "tick " << i;
    }
    EXPECT_TRUE(differs_from_stateless(rows, base)) << "control: stateless rounding reproduces the composition";
  });
}

// The control: the same scripted step with GyroValid set differs from the pure-thrust allocation, and so does a
// nonzero valid gyro against a zero setpoint.
TEST(L4CompositionFault, NegativeControlAValidGyroMakesTheSameStepDiffer) {
  in_child([] {
    const Ref r = load_ref();
    Overrides ov;
    ov.f32(ParamId::l4_thrust_n, r.hover);
    add_step(ov, 0, r);
    ASSERT_EQ(init_with(r, ov), MARV_SIL_OK);
    const Dshots base = dshot_of(r, r.hover, prim::Vec3<float>());
    EXPECT_NE(run(kRun, valid_gyro(0, 0, 0)).back(), base);
  });
}

TEST(L4CompositionFault, NegativeControlANonzeroValidGyroMovesTheOutputAgainstAZeroSetpoint) {
  in_child([] {
    const Ref r = load_ref();
    Overrides ov;
    ov.f32(ParamId::l4_thrust_n, r.hover);
    ASSERT_EQ(init_with(r, ov), MARV_SIL_OK);
    const Dshots base = dshot_of(r, r.hover, prim::Vec3<float>());
    const std::vector<Dshots> rows = run(kRun, valid_gyro(r.rate_max_roll / 2.0F, 0, 0));  // scenario test value: half a full-stick rate
    EXPECT_NE(rows.back(), base);
  });
}

// ---- invalid scenario parameters ---------------------------------------------------------------------------------

// The expected abort would otherwise wait on the host's core-dump handler.
void disable_core_dumps() {
  (void)prctl(PR_SET_DUMPABLE, 0);
}

void expect_init_panics(const char* what, void (*set)(Overrides&, const Ref&), const char* message) {
  const std::string want = std::string("hal_panic: ") + message;
  EXPECT_EXIT(
      {
        disable_core_dumps();
        const Ref r = load_ref();
        Overrides ov;
        set(ov, r);
        (void)init_with(r, ov);
        std::exit(0);
      },
      ::testing::KilledBySignal(SIGABRT), want)
      << what;
}

TEST(L4CompositionIllegal, InvalidScenarioParametersPanicAtInitNamingTheRule) {
  expect_init_panics("negative thrust", [](Overrides& o, const Ref&) { o.f32(ParamId::l4_thrust_n, -1.0F); },
                     "l4_rate_scripted composition: l4_thrust_n is not finite");
  expect_init_panics("negative segment count", [](Overrides& o, const Ref&) { o.i32(ParamId::l4_seg_count, -1); },
                     "l4_rate_scripted composition: l4_seg_count is not in");
  expect_init_panics("segment count above the capacity", [](Overrides& o, const Ref&) { o.i32(ParamId::l4_seg_count, 9); },
                     "l4_rate_scripted composition: l4_seg_count is not in");
  expect_init_panics(
      "equal segment stamps",
      [](Overrides& o, const Ref&) {
        o.i32(ParamId::l4_seg_count, 2);
        o.i32(ParamId::l4_seg1_t_us, 100);
        o.i32(ParamId::l4_seg2_t_us, 100);
      },
      "l4_rate_scripted composition: segment stamps");
  expect_init_panics("negative segment stamp",
                     [](Overrides& o, const Ref&) {
                       o.i32(ParamId::l4_seg_count, 1);
                       o.i32(ParamId::l4_seg1_t_us, -1);
                     },
                     "l4_rate_scripted composition: segment stamps");
  expect_init_panics("chirp axis out of range", [](Overrides& o, const Ref&) { o.i32(ParamId::l4_chirp_axis, 4); },
                     "l4_rate_scripted composition: l4_chirp_axis is not");
  expect_init_panics("negative chirp amplitude", [](Overrides& o, const Ref&) { o.f32(ParamId::l4_chirp_amp_nm, -0.1F); },
                     "l4_rate_scripted composition: l4_chirp_amp_nm is not");
  expect_init_panics("chirp with the default zero band", [](Overrides& o, const Ref&) { o.i32(ParamId::l4_chirp_axis, 1); },
                     "l4_rate_scripted composition: l4_chirp_w_lo, l4_chirp_w_hi");
  expect_init_panics(
      "chirp with w_hi not above w_lo",
      [](Overrides& o, const Ref&) {
        o.i32(ParamId::l4_chirp_axis, 2);
        o.f32(ParamId::l4_chirp_w_lo, 2.0F);
        o.f32(ParamId::l4_chirp_w_hi, 2.0F);
        o.i32(ParamId::l4_chirp_dur_us, 1000);
      },
      "l4_rate_scripted composition: l4_chirp_w_lo, l4_chirp_w_hi");
  expect_init_panics(
      "chirp with zero duration",
      [](Overrides& o, const Ref&) {
        o.i32(ParamId::l4_chirp_axis, 3);
        o.f32(ParamId::l4_chirp_w_lo, 1.0F);
        o.f32(ParamId::l4_chirp_w_hi, 2.0F);
      },
      "l4_rate_scripted composition: l4_chirp_dur_us is not");
  expect_init_panics("rate_loop_divisor zero", [](Overrides& o, const Ref&) { o.i32(ParamId::rate_loop_divisor, 0); },
                     "rate: the design period is not positive");
}

// Control of the death checks above: valid values of the same parameters initialise.
TEST(L4CompositionIllegal, NegativeControlValidScenarioValuesInitialise) {
  in_child([] {
    const Ref r = load_ref();
    Overrides ov;
    ov.f32(ParamId::l4_thrust_n, r.hover);
    ov.i32(ParamId::l4_seg_count, 2);
    ov.i32(ParamId::l4_seg1_t_us, 100);
    ov.i32(ParamId::l4_seg2_t_us, 200);
    ov.i32(ParamId::l4_chirp_axis, 3);
    ov.f32(ParamId::l4_chirp_amp_nm, 0.1F);
    ov.f32(ParamId::l4_chirp_w_lo, 1.0F);
    ov.f32(ParamId::l4_chirp_w_hi, 2.0F);
    ov.i32(ParamId::l4_chirp_dur_us, 1000);
    EXPECT_EQ(init_with(r, ov), MARV_SIL_OK);
  });
}

}  // namespace
