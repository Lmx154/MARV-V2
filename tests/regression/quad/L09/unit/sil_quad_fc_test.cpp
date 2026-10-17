// The quad_fc composition as a SIL library (marv_sil_quad_fc, HIL S2c, decision 0046 item 4): the staged ManualControl
// reaches the composition through the C ABI (marv_sil_stage_manual_control), and the staged frame stamp, against the tick's
// board clock, is what the arming and failsafe machine ages.
//
// The library is linked, not dlopen-ed, and runs the FC3v2 parameter set at the board's tick rational (read from the set's
// default table, the values marv_sil_init takes). marv_sil_init succeeds once per process, so every test that initialises
// runs in a forked child (the pattern of tests/regression/quad/L00/unit/sil/sil_test_util.hpp). The composition's arming
// state is not exported by the library (G3), so a test reads it from the DShot output: stop (raw 0) while disarmed, a
// throttle command (raw >= 48, the DShot protocol's first throttle value) while armed.
//
//  - StagedSticksArmAndDisarm: still IMU samples, a fresh frame staged on every tick; with the arm switch low through the gyro
//    calibration and the link recovery the output stays stop, ch5 high with throttle low gives a throttle command on every
//    motor, a higher throttle stick gives a higher command on every motor, ch5 low gives stop again.
//  - NoStagingMeansNoLink: nothing staged (the HAL's frames 0): the output stays stop (NoLink); the first test is its control.
//  - FrameAgeBelowLossArms / FrameAgeAtLossNeverArms: each staged frame stamped `age` before its tick (a replay keeps the
//    recorded frame ages): an age one microsecond below failsafe_loss_us arms, an age of failsafe_loss_us never does.
//  - FrameStampDrivesTheFailsafe: staging stops (the last frame stays, its stamp frozen): on every rate tick the output is the
//    live command (within one DShot code) before failsafe_hold_us of frame age, the idle command (sticks centred,
//    throttle 0) from it, and stop from failsafe_disarm_us of age (1.5 s) on, on every rate tick.
//  - StageManualControlValidation: states, null, struct size and the contract of the frame (finite, ranges, frames 0).
#include <gtest/gtest.h>

#include <marv_sil.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>

#include <marv/params/param.hpp>
#include <marv/params/param_ids.hpp>
#include <marv/types/actuator.hpp>

namespace {

using namespace marv;

constexpr std::size_t kMotors = kQuadXMotors;
constexpr std::uint16_t kDshotThrottleMin = 48;  // DShot: 0 is stop, 1..47 are commands, 48..2047 the throttle
constexpr float kQuietAccelZ = -9.8F;            // m/s^2, FRD, at rest and level (the gate test's quiet sample)
constexpr float kTempK = 300.0F;                 // a valid temperature
constexpr float kSwitchHigh = 1.0F;
constexpr float kSwitchLow = -1.0F;
constexpr float kHalfThrottle = 0.5F;
constexpr std::uint64_t kSettleUs = 100000;  // sim time run after a change before the output is read
constexpr std::uint64_t kMarginUs = 1000000;
constexpr std::uint32_t kAllValid = (1U << MARV_IMU_GYRO_VALID) | (1U << MARV_IMU_ACCEL_VALID) | (1U << MARV_IMU_TEMP_VALID);

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

std::int32_t i32_default(ParamId id) { return param_defaults()[static_cast<std::size_t>(id)].value.i32; }

using Dshots = std::array<std::uint16_t, kMotors>;

[[nodiscard]] bool all_stop(const Dshots& d) {
  for (const std::uint16_t v : d) {
    if (v != 0) {
      return false;
    }
  }
  return true;
}

[[nodiscard]] bool all_throttle(const Dshots& d) {
  for (const std::uint16_t v : d) {
    if (v < kDshotThrottleMin) {
      return false;
    }
  }
  return true;
}

[[nodiscard]] bool all_greater(const Dshots& a, const Dshots& b) {
  for (std::size_t m = 0; m < kMotors; ++m) {
    if (a[m] <= b[m]) {
      return false;
    }
  }
  return true;
}

// Within one DShot code on every motor: the held command creeps by a code as the rate loop's filters settle.
[[nodiscard]] bool near(const Dshots& a, const Dshots& b) {
  for (std::size_t m = 0; m < kMotors; ++m) {
    if (a[m] > b[m] + 1 || b[m] > a[m] + 1) {
      return false;
    }
  }
  return true;
}

// The SIL at the FC3v2 tick rational, ticked one sample at a time. stage() is what a replay does before a tick.
class Sil {
 public:
  Sil()
      : num_(static_cast<std::uint32_t>(i32_default(ParamId::tick_period_num_us))),
        den_(static_cast<std::uint32_t>(i32_default(ParamId::tick_period_den))),
        divisor_(static_cast<std::uint64_t>(i32_default(ParamId::rate_loop_divisor))),
        arm_index_(static_cast<std::size_t>(i32_default(ParamId::rc_channel_arm) - 1)) {
    marv_sil_config c{};
    c.struct_size = sizeof(marv_sil_config);
    c.imu_meas_size = sizeof(marv_imu_meas);
    c.override_size = sizeof(marv_sil_param_override);
    c.tick_period_num_us = num_;
    c.tick_period_den = den_;
    c.param_schema_hash = kParamSchemaHash;
    EXPECT_EQ(marv_sil_init(&c), MARV_SIL_OK);
  }

  [[nodiscard]] std::uint64_t stamp(std::uint64_t n) const {
    return (n / den_) * num_ + ((n % den_) * num_) / den_;
  }
  [[nodiscard]] std::uint64_t now_us() const { return stamp(n_); }  // the stamp of the next tick
  [[nodiscard]] std::uint64_t divisor() const { return divisor_; }
  [[nodiscard]] std::uint64_t next_tick() const { return n_; }
  [[nodiscard]] const Dshots& out() const { return out_; }
  [[nodiscard]] std::uint64_t out_t_us() const { return out_t_us_; }

  // A frame as the receiver delivers it: stamped `age_us` before the next tick, sticks as given, the arm switch on its channel.
  marv_manual_control frame(std::uint64_t age_us, float arm, float throttle) {
    marv_manual_control m{};
    m.struct_size = sizeof(marv_manual_control);
    m.frames = ++frames_;
    const std::uint64_t t = now_us();
    m.t_us = t >= age_us ? t - age_us : 0;
    m.throttle = throttle;
    m.channels[arm_index_] = arm;
    return m;
  }

  void stage(const marv_manual_control& m) { ASSERT_EQ(marv_sil_stage_manual_control(&m), MARV_SIL_OK); }

  void tick() {
    marv_imu_meas imu{};
    imu.accel_m_s2 = {0.0F, 0.0F, kQuietAccelZ};
    imu.temp_k = kTempK;
    imu.flags = kAllValid;
    std::uint64_t t = 0;
    std::array<std::uint16_t, kMotors> d{};
    marv_sil_out o{};
    o.struct_size = sizeof(marv_sil_out);
    o.capacity_ticks = 1;
    o.t_us = &t;
    o.dshot = d.data();
    ASSERT_EQ(marv_sil_tick(n_, 1, &imu, &o), MARV_SIL_OK);
    ASSERT_EQ(t, stamp(n_));
    out_ = d;
    out_t_us_ = t;
    ++n_;
  }

  // Runs until the next tick's stamp reaches `until_us`, staging a frame of `age_us` on every tick if `staged`.
  void run_to(std::uint64_t until_us, bool staged, std::uint64_t age_us, float arm, float throttle) {
    while (now_us() < until_us) {
      if (staged) {
        stage(frame(age_us, arm, throttle));
      }
      tick();
    }
  }

  // The time after which a boot gyro calibration is complete and the link has been valid for the recovery time.
  [[nodiscard]] static std::uint64_t ready_us() {
    return static_cast<std::uint64_t>(i32_default(ParamId::gyro_cal_duration_us)) +
           static_cast<std::uint64_t>(i32_default(ParamId::failsafe_recovery_us)) + kMarginUs;
  }

  [[nodiscard]] bool rate_tick(std::uint64_t n) const { return n % divisor_ == 0; }

 private:
  std::uint32_t num_;
  std::uint32_t den_;
  std::uint64_t divisor_;
  std::size_t arm_index_;
  std::uint64_t n_ = 0;
  std::uint32_t frames_ = 0;
  Dshots out_{};
  std::uint64_t out_t_us_ = 0;
};

// Switch low through the calibration and the link recovery, then switch high with the throttle low: returns after the
// settling time. A frame of `age_us` is staged on every tick.
void boot_and_raise_switch(Sil& sil, std::uint64_t age_us) {
  sil.run_to(Sil::ready_us(), true, age_us, kSwitchLow, 0.0F);
  sil.run_to(sil.now_us() + kSettleUs, true, age_us, kSwitchHigh, 0.0F);
}

TEST(SilQuadFc, StagedSticksArmAndDisarm) {
  in_child([] {
    Sil sil;
    sil.run_to(Sil::ready_us(), true, 0, kSwitchLow, 0.0F);
    EXPECT_TRUE(all_stop(sil.out())) << "switch low: disarmed";
    sil.run_to(sil.now_us() + kSettleUs, true, 0, kSwitchHigh, 0.0F);
    EXPECT_TRUE(all_throttle(sil.out())) << "ch5 high, throttle low: armed, a throttle command on every motor";
    const Dshots idle = sil.out();
    sil.run_to(sil.now_us() + kSettleUs, true, 0, kSwitchHigh, kHalfThrottle);
    EXPECT_TRUE(all_greater(sil.out(), idle)) << "a higher throttle stick gives a higher command on every motor";
    sil.run_to(sil.now_us() + kSettleUs, true, 0, kSwitchLow, kHalfThrottle);
    EXPECT_TRUE(all_stop(sil.out())) << "ch5 low: stop";
  });
}

TEST(SilQuadFc, NoStagingMeansNoLink) {
  in_child([] {
    Sil sil;
    sil.run_to(Sil::ready_us() + kSettleUs, false, 0, kSwitchHigh, 0.0F);
    EXPECT_GT(sil.next_tick(), 0U);
    EXPECT_TRUE(all_stop(sil.out()));
  });
}

TEST(SilQuadFc, FrameAgeBelowLossArms) {
  in_child([] {
    Sil sil;
    boot_and_raise_switch(sil, static_cast<std::uint64_t>(i32_default(ParamId::failsafe_loss_us)) - 1);
    EXPECT_TRUE(all_throttle(sil.out()));
  });
}

TEST(SilQuadFc, FrameAgeAtLossNeverArms) {
  in_child([] {
    Sil sil;
    boot_and_raise_switch(sil, static_cast<std::uint64_t>(i32_default(ParamId::failsafe_loss_us)));
    EXPECT_TRUE(all_stop(sil.out()));
  });
}

TEST(SilQuadFc, FrameStampDrivesTheFailsafe) {
  in_child([] {
    Sil sil;
    boot_and_raise_switch(sil, 0);
    const Dshots idle = sil.out();
    ASSERT_TRUE(all_throttle(idle));
    sil.run_to(sil.now_us() + kSettleUs, true, 0, kSwitchHigh, kHalfThrottle);
    const Dshots live = sil.out();
    ASSERT_TRUE(all_greater(live, idle));

    // The last frame staged is the one for the tick before the next; its stamp is that tick's.
    const std::uint64_t last_us = sil.stamp(sil.next_tick() - 1);
    const std::uint64_t hold_us = static_cast<std::uint64_t>(i32_default(ParamId::failsafe_hold_us));
    const std::uint64_t disarm_us = static_cast<std::uint64_t>(i32_default(ParamId::failsafe_disarm_us));
    std::size_t live_ticks = 0;
    std::size_t centred_ticks = 0;
    std::size_t stop_ticks = 0;
    while (sil.now_us() < last_us + disarm_us + kSettleUs) {
      const std::uint64_t n = sil.next_tick();
      sil.tick();
      if (!sil.rate_tick(n)) {
        continue;
      }
      const std::uint64_t age = sil.out_t_us() - last_us;
      if (age >= disarm_us) {
        EXPECT_TRUE(all_stop(sil.out())) << "age " << age << " us: failsafe disarm";
        ++stop_ticks;
      } else if (age >= hold_us) {
        EXPECT_TRUE(near(sil.out(), idle)) << "age " << age << " us: sticks centred, throttle 0";
        ++centred_ticks;
      } else {
        EXPECT_TRUE(near(sil.out(), live)) << "age " << age << " us: the last sticks held";
        ++live_ticks;
      }
    }
    EXPECT_GT(live_ticks, 0U);
    EXPECT_GT(centred_ticks, 0U);
    EXPECT_GT(stop_ticks, 0U);
  });
}

TEST(SilQuadFc, StageManualControlBeforeInitIsRejectedWithoutChangingTheState) {
  marv_manual_control m{};
  m.struct_size = sizeof(marv_manual_control);
  EXPECT_EQ(marv_sil_stage_manual_control(&m), MARV_SIL_E_STATE);
  EXPECT_EQ(marv_sil_stage_manual_control(nullptr), MARV_SIL_E_STATE);  // the state is checked before the arguments
}

TEST(SilQuadFc, StageManualControlValidation) {
  in_child([] {
    Sil sil;
    const marv_manual_control good = sil.frame(0, kSwitchHigh, kHalfThrottle);
    EXPECT_EQ(marv_sil_stage_manual_control(&good), MARV_SIL_OK);
    EXPECT_EQ(marv_sil_stage_manual_control(nullptr), MARV_SIL_E_NULL);

    marv_manual_control m = good;
    m.struct_size = sizeof(marv_manual_control) + 1;
    EXPECT_EQ(marv_sil_stage_manual_control(&m), MARV_SIL_E_ABI);

    constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
    constexpr float kInf = std::numeric_limits<float>::infinity();
    const auto rejected = [&](auto mutate) {
      marv_manual_control bad = good;
      mutate(bad);
      EXPECT_EQ(marv_sil_stage_manual_control(&bad), MARV_SIL_E_INPUT);
    };
    rejected([&](marv_manual_control& b) { b.roll = kNaN; });
    rejected([&](marv_manual_control& b) { b.pitch = kInf; });
    rejected([&](marv_manual_control& b) { b.yaw = -kInf; });
    rejected([&](marv_manual_control& b) { b.throttle = kNaN; });
    rejected([&](marv_manual_control& b) { b.channels[MARV_RC_CHANNELS - 1] = kNaN; });
    rejected([&](marv_manual_control& b) { b.link_quality = kNaN; });
    rejected([&](marv_manual_control& b) { b.roll = std::nextafter(1.0F, 2.0F); });
    rejected([&](marv_manual_control& b) { b.yaw = std::nextafter(-1.0F, -2.0F); });
    rejected([&](marv_manual_control& b) { b.throttle = -std::numeric_limits<float>::min(); });
    rejected([&](marv_manual_control& b) { b.throttle = std::nextafter(1.0F, 2.0F); });
    rejected([&](marv_manual_control& b) { b.channels[0] = std::nextafter(1.0F, 2.0F); });
    rejected([&](marv_manual_control& b) { b.link_quality = std::nextafter(1.0F, 2.0F); });
    rejected([&](marv_manual_control& b) { b.frames = 0; });  // frames 0: every RC field is 0, and good has t_us / sticks set

    marv_manual_control none{};  // no frame yet: frames 0, every RC field 0, a link-statistics frame may have set the quality
    none.struct_size = sizeof(marv_manual_control);
    none.link_quality = 1.0F;
    none.link_t_us = 1;
    EXPECT_EQ(marv_sil_stage_manual_control(&none), MARV_SIL_OK);

    EXPECT_EQ(marv_sil_shutdown(), MARV_SIL_OK);
    EXPECT_EQ(marv_sil_stage_manual_control(&good), MARV_SIL_E_STATE);
  });
}

}  // namespace
