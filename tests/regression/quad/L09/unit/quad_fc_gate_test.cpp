// The quad_fc motor output gate (M1: Disarmed, and MotorTest on the console motor test's request, decision 0038). Through
// hal_sim, with no request staged: every rate tick writes DshotValue::stop() to
// every motor, a tick that is not a rate tick writes nothing, and the mixer's own output (never stop: its floor is the
// idle command) never reaches the latch, for samples of every kind the rate loop handles (valid, saturated, invalid).
// Flight timing never halts it (quad spec 4.0 M1, decision 0034): a late, early, repeated or backwards stamp, or a
// missing sample, is a counted timing fault of the rate loop.
//
// The motor test (decision 0038): a valid request spins exactly the requested logical motor and stops the other three, a
// request outside the limits (motor 1..4, DShot value 48..motor_test_dshot_max, duration 1 ms..motor_test_duration_max_ms)
// is refused and counted with every motor stopped, the test ends at its deadline in rate-tick stamps or at the first rate
// tick after a cancel or a disconnect, and no write ever has two motors nonzero (invariant I1).
//
// The armed gate (M2 P4, decision 0040 Q1): the gate is Armed exactly when the arming machine is; disarmed (and in a
// failsafe disarm, or on an IMU deadline miss) every rate tick writes stop, armed it writes the mixer's output for the
// stick rates and the throttle; a motor test starts only from Disarmed (a request while armed is refused, reason Armed)
// and arming is refused while a test runs.
//
// The boot gyro calibration and the upright check (M3 P6, decision 0042): arming is refused (Calibrating) until a calibration
// duration of still samples has passed, the calibrated bias is then subtracted from the gyro before the estimator and the rate
// loop's chain, and a tilt above the small angle refuses arming (NotUpright). Every armed-gate case starts after the
// calibration (ArmedGateTest::warm_up).
//
// Angle mode (M4 A2, decision 0043 item 3): with rc_channel_angle selecting a channel (the register's default 0 is no switch;
// the tests select channel kAngleChannel, a table override) the sticks go through the attitude law while armed and the switch
// is at or above angle_switch_threshold, and through the rate curve otherwise. The beside reference (ArmedGateTest::
// reference_angle) is an estimator, AngleMode, AttitudeLaw and RateGroupStep::execute_bypass built as quad_fc builds them.
#include <gtest/gtest.h>
#include <sys/prctl.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <csignal>
#include <cstdlib>
#include <limits>
#include <span>
#include <vector>

#include <marv/attitude/angle_mode.hpp>
#include <marv/attitude/attitude_law.hpp>
#include <marv/attitude/config.hpp>
#include <marv/composition.hpp>
#include <marv/estimator/mahony.hpp>
#include <marv/hal/hal.hpp>
#include <marv/hal/tick.hpp>
#include <marv/hal_sim/hal_sim.hpp>
#include <marv/mixer/mixer.hpp>
#include <marv/params/param.hpp>
#include <marv/prim/constants.hpp>
#include <marv/prim/vec.hpp>
#include <marv/rate/rate_loop.hpp>
#include <marv/rate_group/rate_group.hpp>
#include <marv/stick_rates/stick_rates.hpp>
#include <marv/types/actuator.hpp>
#include <marv/types/attitude_state.hpp>
#include <marv/types/imu_sample.hpp>

namespace marv {
namespace {

constexpr std::size_t kTicks = 400;
constexpr std::uint16_t kSentinelRaw = 1000;

// The channel the tests give the angle switch (a scenario test value: any channel but the arm channel 5 and the sticks 1 to 4).
// The register's default is 0, no switch (decision 0043); the test process loads the default table with this one override.
constexpr std::int32_t kAngleChannel = 6;

std::array<ParamRecord, kParamCount> g_table;

// The default table with rc_channel_angle = `angle_channel` (and att_loop_ratio = `att_loop_ratio` if not 0; nothing else changed).
std::span<const ParamRecord, kParamCount> table_with_angle_channel(std::int32_t angle_channel, std::int32_t att_loop_ratio = 0) {
  const std::span<const ParamRecord, kParamCount> d = param_defaults();
  std::copy(d.begin(), d.end(), g_table.begin());
  if (att_loop_ratio != 0) {
    g_table[static_cast<std::size_t>(ParamId::att_loop_ratio)].value = ParamValue{ParamType::I32, 0.0F, att_loop_ratio};
  }
  g_table[static_cast<std::size_t>(ParamId::rc_channel_angle)].value = ParamValue{ParamType::I32, 0.0F, angle_channel};
  return std::span<const ParamRecord, kParamCount>(g_table);
}

void init_params_once() {
  static const bool ok = params_init(table_with_angle_channel(kAngleChannel));
  ASSERT_TRUE(ok);
}

DshotValue sentinel() { return DshotValue::from_raw(kSentinelRaw).value_or(DshotValue::stop()); }

enum class Kind : std::uint8_t { Quiet, Moving, Saturated, GyroInvalid, GyroNaN };

ImuSample sample_of(Kind k, std::size_t n) {
  ImuSample s{};
  s.t_us = hal_time_us();
  s.temp_k = 300.0F;
  s.accel_m_s2 = prim::Vec3<float>(0.0F, 0.0F, -9.8F);
  s.flags = imu_flag(ImuFlag::GyroValid) | imu_flag(ImuFlag::AccelValid) | imu_flag(ImuFlag::TempValid);
  const float phase = static_cast<float>(n) * 0.05F;
  switch (k) {
    case Kind::Quiet:
      break;
    case Kind::Moving:
      s.gyro_rad_s = prim::Vec3<float>(2.0F * std::sin(phase), 1.5F * std::cos(phase), 0.5F * std::sin(3.0F * phase));
      break;
    case Kind::Saturated:
      s.gyro_rad_s = prim::Vec3<float>(35.0F, -35.0F, 35.0F);
      s.flags |= imu_flag(ImuFlag::GyroSatX) | imu_flag(ImuFlag::GyroSatY) | imu_flag(ImuFlag::GyroSatZ);
      break;
    case Kind::GyroInvalid:
      s.flags &= ~imu_flag(ImuFlag::GyroValid);
      break;
    case Kind::GyroNaN:
      s.gyro_rad_s = prim::Vec3<float>(std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F);
      break;
  }
  return s;
}

class QuadFcGateTest : public ::testing::TestWithParam<Kind> {
 protected:
  void SetUp() override {
    init_params_once();
    period_ = {static_cast<std::uint32_t>(param_value<ParamId::tick_period_num_us>()),
               static_cast<std::uint32_t>(param_value<ParamId::tick_period_den>())};
    divisor_ = static_cast<std::size_t>(param_value<ParamId::rate_loop_divisor>());
    hal_sim::setup(period_, motor_latch_, servo_latch_);
  }

  hal_sim::TickPeriod period_{};
  std::size_t divisor_ = 1;
  std::array<DshotValue, composition::kMotors> motor_latch_{};
  std::array<ServoUs, composition::kServos> servo_latch_{};
};

TEST_F(QuadFcGateTest, TheMixerOutputOnAZeroRequestIsNeverStop) {
  const mixer::MixerConfig<float> cfg = mixer::load_config();
  const mixer::MixerOutput<float> out = mixer::mix(cfg, mixer::Request<float>{});
  for (const DshotValue& m : out.dshot) {
    EXPECT_NE(m, DshotValue::stop());
  }
}

TEST_P(QuadFcGateTest, EveryRateTickWritesStopToEveryMotorAndNoOtherTickWrites) {
  composition::init();
  std::size_t writes = 0;
  for (std::size_t n = 0; n < kTicks; ++n) {
    motor_latch_.fill(sentinel());
    hal_sim::begin_tick(n);
    composition::tick(sample_of(GetParam(), n));
    hal_sim::end_tick();
    const bool rate_tick = n % divisor_ == 0;
    for (const DshotValue& m : motor_latch_) {
      EXPECT_EQ(m, rate_tick ? DshotValue::stop() : sentinel()) << "tick " << n;
    }
    writes += rate_tick ? 1U : 0U;
  }
  EXPECT_EQ(writes, (kTicks + divisor_ - 1) / divisor_);
}

INSTANTIATE_TEST_SUITE_P(Samples, QuadFcGateTest,
                         ::testing::Values(Kind::Quiet, Kind::Moving, Kind::Saturated, Kind::GyroInvalid,
                                           Kind::GyroNaN));

// The estimator's input flags (M3 P2, decision 0042): a sample whose GyroValid bit is clear is not stepped on (its NaN gyro does
// not invalidate the estimate, and the next stepped sample's dt spans the gap from the last stamp stepped on), and one whose
// AccelValid bit is clear gives no correction (its accelerometer, here 90 degrees off the estimate, is ignored). Tick 0 is a
// healthy level sample; a roll rate of kRoll rad/s then acts for every tick with no correction, so the tilt at the last rate tick
// is kRoll x (its stamp - tick 0's stamp) to float accumulation, 1e-4 rad (a correction of Kp = 0.25 1/s toward the ignored
// accelerometer would move it by about 1.3e-3 rad, and a dt measured from the last sample seen by the gap, ticks 100..199, of
// 1.3e-2 rad). Control: the same run with the bits set, where the NaN gyro invalidates the estimate and tick 200 initialises it
// again from the accelerometer, so the tilt ends near pi / 2 instead (checked to differ by more than 1 rad).
TEST_F(QuadFcGateTest, SamplesFlaggedInvalidAreNotFedToTheEstimator) {
  constexpr float kRoll = 1.0F;
  constexpr double kTol = 1.0e-4;
  const std::size_t last = divisor_ * (kTicks / divisor_ - 1);  // the last rate tick of the run
  for (const bool flagged : {true, false}) {
    hal_sim::setup(period_, motor_latch_, servo_latch_);
    composition::init();
    TimeUs first_us = 0;
    TimeUs last_us = 0;
    for (std::size_t n = 0; n <= last; ++n) {
      hal_sim::begin_tick(n);
      ImuSample s = sample_of(Kind::Quiet, n);
      if (n > 0) {
        s.gyro_rad_s = prim::Vec3<float>(kRoll, 0.0F, 0.0F);
        s.accel_m_s2 = prim::Vec3<float>(0.0F, -9.8F, 0.0F);
        if (flagged) {
          s.flags &= ~imu_flag(ImuFlag::AccelValid);
        }
        if (n >= 100 && n < 200) {
          s.gyro_rad_s = prim::Vec3<float>(std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F);
          if (flagged) {
            s.flags &= ~imu_flag(ImuFlag::GyroValid);
          }
        }
      }
      if (n == 0) {
        first_us = s.t_us;
      }
      last_us = s.t_us;
      composition::tick(s);
      hal_sim::end_tick();
    }
    const composition::AttitudeStatus a = composition::attitude_status();
    const double expected = static_cast<double>(kRoll) * static_cast<double>(last_us - first_us) / 1.0e6;
    EXPECT_TRUE(a.valid);
    if (flagged) {
      EXPECT_NEAR(static_cast<double>(a.tilt_rad), expected, kTol);
    } else {
      EXPECT_GT(std::fabs(static_cast<double>(a.tilt_rad) - expected), 1.0);
    }
  }
}

// One rate-tick sample off its stamp (Late, Early: displaced by the tolerance plus 1 µs, beyond the tolerance and the
// stamp truncation; Repeated: the previous rate sample's stamp; Backwards: 1 µs before it), or one sample never
// delivered (Missing).
enum class Stamp : std::uint8_t { Late, Early, Repeated, Backwards, Missing };

// Scenario test value: the tick whose sample is off, a rate tick (even) well inside the run.
constexpr std::size_t kOffTick = 200;

struct TimingRun {
  std::uint32_t faults = 0;         // the rate loop's fault count after the run
  std::uint32_t timing_faults = 0;  // of which spacing faults
  std::uint32_t fault_executions = 0;
};

// Runs the composition through hal_sim with the kind's stamp at kOffTick, and beside it a rate-group step built as
// quad_fc's init builds it (rate::load_config, mixer::load_config, rate_group::load_chain_config) fed the same samples,
// whose outputs the composition does not expose. Every rate tick of the composition writes stop to every motor.
TimingRun run_timing(Stamp kind, hal_sim::TickPeriod period, std::size_t divisor,
                     std::array<DshotValue, composition::kMotors>& motor_latch,
                     std::array<ServoUs, composition::kServos>& servo_latch) {
  hal_sim::setup(period, motor_latch, servo_latch);
  composition::init();
  rate_group::RateGroupStep step;
  step.init(rate::load_config(), mixer::load_config(), rate_group::load_chain_config());
  const auto off_us = static_cast<TimeUs>(param_value<ParamId::rate_period_tolerance_us>()) + 1;
  TimingRun r;
  TimeUs last_rate_stamp = 0;
  std::size_t delivered = 0;
  for (std::size_t n = 0; n < kTicks; ++n) {
    hal_sim::begin_tick(n);
    ImuSample s = sample_of(Kind::Quiet, n);
    if (n == kOffTick) {
      switch (kind) {
        case Stamp::Late:
          s.t_us += off_us;
          break;
        case Stamp::Early:
          s.t_us -= off_us;
          break;
        case Stamp::Repeated:
          s.t_us = last_rate_stamp;
          break;
        case Stamp::Backwards:
          s.t_us = last_rate_stamp - 1;
          break;
        case Stamp::Missing:
          hal_sim::end_tick();
          continue;
      }
    }
    motor_latch.fill(sentinel());
    composition::tick(s);
    const bool rate_tick = delivered % divisor == 0;
    ++delivered;
    for (const DshotValue& m : motor_latch) {
      EXPECT_EQ(m, rate_tick ? DshotValue::stop() : sentinel()) << "tick " << n;
    }
    step.filter(s, rate_tick);
    if (rate_tick) {
      const rate_group::Execution e = step.execute(prim::Vec3<float>(), prim::Vec3<float>(), 0.0F);
      r.faults = e.rate.fault_count;
      r.timing_faults = e.rate.timing_fault_count;
      r.fault_executions += e.rate.fault_active ? 1U : 0U;
      last_rate_stamp = s.t_us;
    }
    hal_sim::end_tick();
  }
  return r;
}

TEST_F(QuadFcGateTest, OffPeriodStampsNeverHaltAndAreCountedTimingFaults) {
  // A displaced or repeated stamp puts two spacings off the period (to it, and from it to the next rate sample, the
  // loop re-anchoring on it): two faults. A missing sample puts one spacing off by a whole tick: one fault.
  struct Case {
    Stamp kind;
    const char* name;
    std::uint32_t faults;
  };
  for (const Case& k : {Case{Stamp::Late, "late", 2}, Case{Stamp::Early, "early", 2},
                        Case{Stamp::Repeated, "repeated", 2}, Case{Stamp::Backwards, "backwards", 2},
                        Case{Stamp::Missing, "missing", 1}}) {
    const TimingRun r = run_timing(k.kind, period_, divisor_, motor_latch_, servo_latch_);
    EXPECT_EQ(r.faults, k.faults) << k.name;
    EXPECT_EQ(r.timing_faults, k.faults) << k.name;
    EXPECT_EQ(r.fault_executions, k.faults) << k.name;
  }
}

// ---- The motor test ----

// The limits Luis ruled on 2026-10-16 ("1. 500 2. 3 seconds 3. yes"; the duration raised to 20 s at the M1 bench,
// "20s is good.", decision 0038 amendment), as numbers the test states independently of the parameter table: the
// table must hold exactly these.
constexpr std::uint32_t kLuisDshotCap = 500;
constexpr std::uint32_t kLuisMaxDurationMs = 20000;
constexpr std::uint32_t kMicrosPerMs = 1000;

class MotorTestGateTest : public QuadFcGateTest {
 protected:
  struct Out {
    bool wrote = false;
    std::array<DshotValue, composition::kMotors> motor{};
  };

  void SetUp() override {
    QuadFcGateTest::SetUp();
    composition::init();
  }

  // One tick, number n_, of a sample with the HAL stamp (or `stamp` if given); returns what the gate wrote.
  Out tick_once(const TimeUs* stamp = nullptr) {
    motor_latch_.fill(sentinel());
    hal_sim::begin_tick(n_);
    ImuSample s = sample_of(Kind::Quiet, n_);
    if (stamp != nullptr) {
      s.t_us = *stamp;
    }
    composition::tick(s);
    hal_sim::end_tick();
    ++n_;
    Out o;
    o.motor = motor_latch_;
    o.wrote = motor_latch_[0] != sentinel();
    return o;
  }

  // Ticks until the next rate tick has run (a rate tick is every divisor_ ticks, tick 0 included).
  Out next_rate_tick() {
    Out o = tick_once();
    while ((n_ - 1) % divisor_ != 0) {
      o = tick_once();
    }
    return o;
  }

  // Ticks up to, not including, the next rate tick (the non-rate ticks write nothing).
  void until_rate_tick() {
    while (n_ % divisor_ != 0) {
      const Out o = tick_once();
      EXPECT_FALSE(o.wrote) << "tick " << (n_ - 1);
    }
  }

  static std::size_t nonzero(const Out& o) {
    std::size_t k = 0;
    for (const DshotValue& m : o.motor) {
      k += m != DshotValue::stop() ? 1U : 0U;
    }
    return k;
  }

  void expect_all_stop(const Out& o) {
    for (const DshotValue& m : o.motor) {
      EXPECT_EQ(m, DshotValue::stop());
    }
  }

  void expect_only(const Out& o, std::size_t motor_index, std::uint16_t raw) {
    for (std::size_t i = 0; i < o.motor.size(); ++i) {
      EXPECT_EQ(o.motor[i].raw(), i == motor_index ? raw : 0U) << "motor " << (i + 1);
    }
  }

  // Stages `r` and runs to the next rate tick.
  Out request(const MotorTestRequest& r) {
    until_rate_tick();
    hal_sim::stage_motor_test_request(r);
    return next_rate_tick();
  }

  Tick n_ = 0;
};

TEST_F(MotorTestGateTest, TheLimitsAreLuisRulings) {
  EXPECT_EQ(param_value<ParamId::motor_test_dshot_max>(), static_cast<std::int32_t>(kLuisDshotCap));
  EXPECT_EQ(param_value<ParamId::motor_test_duration_max_ms>(), static_cast<std::int32_t>(kLuisMaxDurationMs));
}

TEST_F(MotorTestGateTest, NoRequestNoMotorTestEvent) {
  for (std::size_t k = 0; k < 4; ++k) {
    expect_all_stop(next_rate_tick());
  }
  hal_sim::stage_motor_test_cancel(MotorTestCancel::Cancel);
  for (std::size_t k = 0; k < 4; ++k) {
    expect_all_stop(next_rate_tick());
  }
  EXPECT_EQ(hal_sim::motor_test_event_count(), 0U);
}

TEST_F(MotorTestGateTest, AValidRequestSpinsExactlyTheRequestedLogicalMotorAndNoTwoAreEverNonzero) {
  for (std::uint32_t motor = 1; motor <= composition::kMotors; ++motor) {
    for (const std::uint32_t value : {static_cast<std::uint32_t>(prim::kDshotThrottleMin), std::uint32_t{100}, kLuisDshotCap}) {
      const std::uint32_t before = hal_sim::motor_test_event_count();
      Out o = request({motor, value, 20});
      expect_only(o, motor - 1, static_cast<std::uint16_t>(value));
      ASSERT_EQ(hal_sim::motor_test_event_count(), before + 1);
      EXPECT_EQ(hal_sim::last_motor_test_event().kind, MotorTestEventKind::Accepted);
      EXPECT_EQ(hal_sim::last_motor_test_event().request.motor, motor);
      EXPECT_EQ(hal_sim::last_motor_test_event().request.dshot, value);
      // To its end: never two nonzero, only that motor, only that value; then stop on every motor.
      for (std::size_t k = 0; k < 400; ++k) {
        o = next_rate_tick();
        EXPECT_LE(nonzero(o), 1U);
        if (nonzero(o) == 1) {
          expect_only(o, motor - 1, static_cast<std::uint16_t>(value));
        }
      }
      expect_all_stop(o);
      EXPECT_EQ(hal_sim::last_motor_test_event().kind, MotorTestEventKind::Stopped);
      EXPECT_EQ(hal_sim::last_motor_test_event().reason, MotorTestReason::Deadline);
    }
  }
}

TEST_F(MotorTestGateTest, ARequestOutsideTheLimitsIsRefusedAndCountedAndEveryMotorStops) {
  struct Case {
    MotorTestRequest r;
    MotorTestReason why;
    const char* name;
  };
  constexpr std::uint32_t kMax32 = std::numeric_limits<std::uint32_t>::max();
  const std::uint32_t below_min = prim::kDshotThrottleMin - 1U;
  const std::vector<Case> cases{
      {{1, below_min, 100}, MotorTestReason::Value, "value 47"},
      {{1, 0, 100}, MotorTestReason::Value, "value 0"},
      {{1, kLuisDshotCap + 1, 100}, MotorTestReason::Value, "value 501"},
      {{1, prim::kDshotThrottleMax, 100}, MotorTestReason::Value, "value 2047"},
      {{1, kMax32, 100}, MotorTestReason::Value, "value 2^32-1"},
      {{0, 100, 100}, MotorTestReason::Motor, "motor 0"},
      {{5, 100, 100}, MotorTestReason::Motor, "motor 5"},
      {{kMax32, 100, 100}, MotorTestReason::Motor, "motor 2^32-1"},
      {{1, 100, 0}, MotorTestReason::Duration, "duration 0"},
      {{1, 100, kLuisMaxDurationMs + 1}, MotorTestReason::Duration, "duration 20001 ms"},
      {{1, 100, kMax32}, MotorTestReason::Duration, "duration 2^32-1"},
  };
  std::uint32_t refused = 0;
  for (const Case& c : cases) {
    const Out o = request(c.r);
    expect_all_stop(o);
    ++refused;
    ASSERT_EQ(hal_sim::motor_test_event_count(), refused) << c.name;
    const MotorTestEvent e = hal_sim::last_motor_test_event();
    EXPECT_EQ(e.kind, MotorTestEventKind::Refused) << c.name;
    EXPECT_EQ(e.reason, c.why) << c.name;
    EXPECT_EQ(e.refused, refused) << c.name;
    EXPECT_EQ(e.accepted, 0U) << c.name;
    for (std::size_t k = 0; k < 4; ++k) {
      expect_all_stop(next_rate_tick());
    }
  }
  // The edges are accepted: value 48, the cap and the maximum duration.
  expect_only(request({4, prim::kDshotThrottleMin, 1}), 3, prim::kDshotThrottleMin);
  expect_only(request({1, kLuisDshotCap, kLuisMaxDurationMs}), 0, static_cast<std::uint16_t>(kLuisDshotCap));
  EXPECT_EQ(hal_sim::last_motor_test_event().kind, MotorTestEventKind::Accepted);
  EXPECT_EQ(hal_sim::last_motor_test_event().accepted, 2U);
}

TEST_F(MotorTestGateTest, TheTestEndsAtTheFirstRateTickWhoseStampIsTheDurationAfterTheRequestsAndNotBefore) {
  for (const std::uint32_t duration_ms : {1U, 7U, 100U, kLuisMaxDurationMs}) {
    until_rate_tick();
    const Tick start = n_;
    hal_sim::stage_motor_test_request({2, 60, duration_ms});
    const TimeUs deadline = hal_sim::stamp_us(period_, start) + duration_ms * kMicrosPerMs;
    Tick end = start;
    while (hal_sim::stamp_us(period_, end) < deadline) {
      end += divisor_;
    }
    std::size_t spun = 0;
    for (Tick k = start; k <= end; k += divisor_) {
      const Out o = next_rate_tick();
      ASSERT_EQ(n_ - 1, k);
      if (k < end) {
        expect_only(o, 1, 60);
        ++spun;
      } else {
        expect_all_stop(o);
      }
    }
    EXPECT_EQ(hal_sim::last_motor_test_event().kind, MotorTestEventKind::Stopped) << duration_ms;
    EXPECT_EQ(hal_sim::last_motor_test_event().reason, MotorTestReason::Deadline) << duration_ms;
    // Spun for no longer than the duration (it ends at the first stamp at or after it, so within one rate period over).
    const TimeUs rate_period_us = (divisor_ * period_.num_us + period_.den - 1) / period_.den;
    EXPECT_LE(static_cast<TimeUs>(spun - 1) * divisor_ * period_.num_us / period_.den, duration_ms * kMicrosPerMs);
    EXPECT_GE(static_cast<TimeUs>(spun) * rate_period_us + rate_period_us, duration_ms * kMicrosPerMs);
    for (std::size_t k = 0; k < 4; ++k) {
      expect_all_stop(next_rate_tick());
    }
  }
}

TEST_F(MotorTestGateTest, AStampThatDoesNotAdvanceCannotExtendTheTestAndAStampBeforeTheRequestEndsIt) {
  const std::uint32_t duration_ms = 100;
  until_rate_tick();
  const Tick start = n_;
  hal_sim::stage_motor_test_request({3, 70, duration_ms});
  const TimeUs frozen = hal_sim::stamp_us(period_, start);
  const std::uint64_t duration_us = static_cast<std::uint64_t>(duration_ms) * kMicrosPerMs;
  const std::uint64_t expect_ticks = (duration_us * period_.den + divisor_ * period_.num_us - 1) / (divisor_ * period_.num_us);
  std::uint64_t spun = 0;
  for (std::size_t k = 0; k < 2 * expect_ticks; ++k) {
    until_rate_tick();
    const Out o = tick_once(&frozen);
    if (nonzero(o) == 0) {
      break;
    }
    ++spun;
  }
  EXPECT_EQ(spun, expect_ticks);
  EXPECT_EQ(hal_sim::last_motor_test_event().reason, MotorTestReason::Deadline);

  until_rate_tick();
  hal_sim::stage_motor_test_request({3, 70, duration_ms});
  const TimeUs now = hal_sim::stamp_us(period_, n_);
  EXPECT_EQ(nonzero(tick_once(&now)), 1U);
  const TimeUs before = now - 1;
  for (std::size_t k = 1; k < divisor_; ++k) {
    EXPECT_FALSE(tick_once().wrote);
  }
  const Out o = tick_once(&before);
  expect_all_stop(o);
  EXPECT_EQ(hal_sim::last_motor_test_event().reason, MotorTestReason::Deadline);
}

TEST_F(MotorTestGateTest, ARateTickStampAtTheDeadlineEndsTheTestAndOneMicrosecondBeforeItDoesNot) {
  const std::uint32_t duration_ms = 100;
  until_rate_tick();
  const TimeUs t0 = hal_sim::stamp_us(period_, n_);
  hal_sim::stage_motor_test_request({1, 70, duration_ms});
  EXPECT_EQ(nonzero(tick_once(&t0)), 1U);
  const TimeUs deadline = t0 + duration_ms * kMicrosPerMs;
  until_rate_tick();
  const TimeUs just_before = deadline - 1;
  EXPECT_EQ(nonzero(tick_once(&just_before)), 1U);
  until_rate_tick();
  expect_all_stop(tick_once(&deadline));
  EXPECT_EQ(hal_sim::last_motor_test_event().reason, MotorTestReason::Deadline);
}

TEST_F(MotorTestGateTest, ACancelOrADisconnectEndsTheTestAtTheNextRateTickAndNeverBefore) {
  for (const MotorTestCancel how : {MotorTestCancel::Cancel, MotorTestCancel::Disconnect}) {
    expect_only(request({1, 100, kLuisMaxDurationMs}), 0, 100);
    // Staged just after a rate tick: the ticks between write nothing, the next rate tick writes stop.
    hal_sim::stage_motor_test_cancel(how);
    const Out o = next_rate_tick();
    expect_all_stop(o);
    const MotorTestEvent e = hal_sim::last_motor_test_event();
    EXPECT_EQ(e.kind, MotorTestEventKind::Stopped);
    EXPECT_EQ(e.reason, how == MotorTestCancel::Cancel ? MotorTestReason::Cancel : MotorTestReason::Disconnect);
    for (std::size_t k = 0; k < 4; ++k) {
      expect_all_stop(next_rate_tick());
    }
  }
  EXPECT_EQ(hal_sim::last_motor_test_event().stopped, 2U);
}

TEST_F(MotorTestGateTest, ARequestAfterACancelStartsAndARequestCancelledAfterItsPublicationIsRefused) {
  hal_sim::stage_motor_test_cancel(MotorTestCancel::Cancel);
  expect_only(request({2, 100, 50}), 1, 100);

  hal_sim::stage_motor_test_request({3, 100, 50});
  hal_sim::stage_motor_test_cancel(MotorTestCancel::Cancel);
  const Out o = next_rate_tick();
  expect_all_stop(o);
  const MotorTestEvent e = hal_sim::last_motor_test_event();
  EXPECT_EQ(e.kind, MotorTestEventKind::Refused);
  EXPECT_EQ(e.reason, MotorTestReason::Cancel);
  for (std::size_t k = 0; k < 4; ++k) {
    expect_all_stop(next_rate_tick());
  }
}

TEST_F(MotorTestGateTest, ANewRequestWhileOneRunsEndsItAndNeverOverlapsIt) {
  expect_only(request({1, 100, kLuisMaxDurationMs}), 0, 100);
  for (std::uint32_t motor = 2; motor <= composition::kMotors; ++motor) {
    const Out o = request({motor, 200, kLuisMaxDurationMs});
    expect_only(o, motor - 1, 200);
  }
  const Out refused = request({1, kLuisDshotCap + 1, 100});
  expect_all_stop(refused);
}

// ---- The armed gate (M2 P4, decisions 0040 and 0041) ----

// Luis's numbers as the test states them, independently of the parameter table (decision 0041 and the arm switch ruling:
// channel 5, +1 armed and -1 disarmed; throttle minimum 1050 us; the failsafe times of Betaflight 4.5.1).
constexpr std::size_t kLuisArmChannel = 5;
constexpr TimeUs kLossUs = 100000;
constexpr TimeUs kHoldUs = 300000;
constexpr TimeUs kDisarmUs = 1500000;
constexpr TimeUs kRecoveryUs = 500000;
constexpr float kSwitchOn = 1.0F;
constexpr float kSwitchOff = -1.0F;
// Betaflight 4.5.1 gyro.c:107, gyroCalibrationDuration 125 x 10 ms: the boot calibration window, as the test states it
// independently of the parameter table.
constexpr TimeUs kCalibrationUs = 1250000;

struct Sticks {
  float roll = 0;
  float pitch = 0;
  float yaw = 0;
  float throttle = 0;
};

class ArmedGateTest : public MotorTestGateTest {
 protected:
  struct RateOut {
    Out out;
    std::array<DshotValue, composition::kMotors> mixed{};  // the expected output if armed, from a beside rate group
    // Angle reference only (use_angle_ref_): what the same tick would give in acro for the same sticks, and with the rate
    // loop's controller restarted before the tick (both from copies of the beside group as it was before the tick).
    std::array<DshotValue, composition::kMotors> acro{};
    std::array<DshotValue, composition::kMotors> reset_alt{};
  };

  void SetUp() override {
    MotorTestGateTest::SetUp();
    mixer_ = mixer::load_config();
    ref_.init(rate::load_config(), mixer_, rate_group::load_chain_config());
    rates_.roll = {param_value<ParamId::rates_centre_roll>(), param_value<ParamId::rate_max_roll>(),
                   param_value<ParamId::rates_expo_roll>()};
    rates_.pitch = {param_value<ParamId::rates_centre_pitch>(), param_value<ParamId::rate_max_pitch>(),
                    param_value<ParamId::rates_expo_pitch>()};
    rates_.yaw = {param_value<ParamId::rates_centre_yaw>(), param_value<ParamId::rate_max_yaw>(),
                  param_value<ParamId::rates_expo_yaw>()};
    frames_ = 0;
    accel_ = sample_of(Kind::Quiet, 0).accel_m_s2;
    const attitude::AttitudeConfig<float> attitude_cfg = attitude::load_config();
    ref_angle_.init(attitude_cfg);
    ref_att_.init(attitude_cfg);
    est_ref_.init(estimator::MahonyConfig<float>{
        param_value<ParamId::estimator_kp>(), param_value<ParamId::estimator_ki>(),
        param_value<ParamId::estimator_accel_healthy_min>(), param_value<ParamId::estimator_accel_healthy_max>()});
    warm_up();
  }

  // M3 P6: the boot gyro calibration refuses arming until a calibration duration of still samples has passed (with the link down
  // the state reads NoLink, which comes first in the refusal order), so the cases that arm start after it: quiet samples, the
  // link down, through the first rate tick whose stamp is past the duration.
  virtual void warm_up() {
    const TimeUs first_us = hal_sim::stamp_us(period_, n_);
    while (hal_sim::stamp_us(period_, n_) - first_us <= kCalibrationUs) {
      (void)rate_tick(nullptr, {});
    }
  }

  // One tick, as tick_once, with the beside rate group fed the same sample (with ref_bias_ subtracted from the tick
  // ref_bias_from_ on, as the composition subtracts its calibrated bias).
  Out step_tick() {
    motor_latch_.fill(sentinel());
    hal_sim::begin_tick(n_);
    ImuSample s = sample_of(kind_, n_);
    s.accel_m_s2 = accel_;
    s.gyro_rad_s = s.gyro_rad_s + bias_;
    if (stamp_fault_ && n_ == stamp_fault_tick_) {  // one rate-tick sample off its stamp, as run_timing does
      const auto off_us = static_cast<TimeUs>(param_value<ParamId::rate_period_tolerance_us>()) + 1;
      switch (stamp_kind_) {
        case Stamp::Late:
          s.t_us += off_us;
          break;
        case Stamp::Early:
          s.t_us -= off_us;
          break;
        case Stamp::Repeated:
          s.t_us = last_rate_stamp_;
          break;
        case Stamp::Backwards:
          s.t_us = last_rate_stamp_ - 1;
          break;
        case Stamp::Missing:
          break;
      }
    }
    composition::tick(s);
    ImuSample beside = s;
    if (n_ >= ref_bias_from_) {
      beside.gyro_rad_s = beside.gyro_rad_s - ref_bias_;
    }
    ref_.filter(beside, n_ % divisor_ == 0);
    fresh_.filter(s, n_ % divisor_ == 0);
    if ((beside.flags & imu_flag(ImuFlag::GyroValid)) != 0) {  // the beside estimator, stepped as the composition steps its own
      const std::uint64_t since = est_ref_started_ && beside.t_us > est_ref_prev_us_ ? beside.t_us - est_ref_prev_us_ : 0;
      est_ref_prev_us_ = beside.t_us;
      est_ref_started_ = true;
      est_ref_.step(beside.gyro_rad_s,
                    (beside.flags & imu_flag(ImuFlag::AccelValid)) != 0
                        ? beside.accel_m_s2
                        : prim::Vec3<float>(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
                                            std::numeric_limits<float>::quiet_NaN()),
                    static_cast<std::uint32_t>(since));
    }
    ref_state_ = AttitudeState<float>{beside.t_us, est_ref_.quat(), beside.gyro_rad_s, est_ref_.valid()};
    if (n_ % divisor_ == 0) {
      last_rate_stamp_ = s.t_us;
    }
    const bool armed = arming::is_armed(composition::arming_status());
    if (armed && !was_armed_ && reset_ref_) {
      ref_.reset_controller();  // as the composition does at the arming edge
    }
    was_armed_ = armed;
    hal_sim::end_tick();
    ++n_;
    Out o;
    o.motor = motor_latch_;
    o.wrote = motor_latch_[0] != sentinel();
    return o;
  }

  // The expected armed output for `want` (sticks as the pilot holds them), from the beside rate group: setpoint through
  // the curve, thrust linear in the throttle between 4 f_min and 4 f_max of the mixer.
  std::array<DshotValue, composition::kMotors> reference(const Sticks& want) {
    const float t_min = static_cast<float>(composition::kMotors) * mixer::f_min(mixer_);
    const float t_max = static_cast<float>(composition::kMotors) * mixer::f_max(mixer_);
    const prim::Vec3<float> sp = stick_rates::rate_setpoint(rates_, prim::Vec3<float>(want.roll, want.pitch, want.yaw));
    return ref_.execute(sp, prim::Vec3<float>(), t_min + want.throttle * (t_max - t_min)).dshot;
  }

  // The rate setpoint the attitude law gives for `want` from the beside estimator's state of the latest tick, through the
  // given AngleMode and AttitudeLaw.
  prim::Vec3<float> angle_setpoint(const Sticks& want, attitude::AngleMode<float>& angle, attitude::AttitudeLaw<float>& law) {
    const attitude::AngleOutput<float> a = angle.execute(attitude::AngleSticks<float>{want.roll, want.pitch, want.yaw}, ref_state_);
    return law.execute(ref_state_, a.q_sp, a.yaw_rate_cmd).rate_setpoint;
  }

  // The expected armed output in angle mode: the setpoint of the beside AngleMode and AttitudeLaw (ref_angle_, ref_att_; the
  // test resets them where the composition does, on entering armed-and-angle) through execute_bypass, with acro's thrust.
  std::array<DshotValue, composition::kMotors> reference_angle(const Sticks& want) {
    angle_sp_ = angle_setpoint(want, ref_angle_, ref_att_);
    return ref_.execute_bypass(angle_sp_, prim::Vec3<float>(), thrust_of(want)).dshot;
  }

  float thrust_of(const Sticks& want) const {
    const float t_min = static_cast<float>(composition::kMotors) * mixer::f_min(mixer_);
    const float t_max = static_cast<float>(composition::kMotors) * mixer::f_max(mixer_);
    return t_min + want.throttle * (t_max - t_min);
  }

  // The beside side of entering armed-and-angle: the composition resets both on that rate tick.
  void reset_angle_reference() {
    ref_angle_.reset();
    ref_att_.reset();
  }

  // Runs to the next rate tick, staging a new frame (stamp = that tick's time) if `pilot` is given, and steps it;
  // `want` are the sticks the beside rate group is executed with (the armed output it would give).
  RateOut rate_tick(const ManualControl* frame, const Sticks& want) {
    while (n_ % divisor_ != 0) {
      const Out o = step_tick();
      EXPECT_FALSE(o.wrote) << "tick " << (n_ - 1);
    }
    if (frame != nullptr) {
      ManualControl m = *frame;
      m.t_us = hal_sim::stamp_us(period_, n_);
      m.frames = ++frames_;
      hal_sim::stage_manual_control(m);
    }
    RateOut r;
    r.out = step_tick();
    if (use_angle_ref_) {
      const float thrust = thrust_of(want);
      rate_group::RateGroupStep acro_alt = ref_;
      r.acro = acro_alt.execute(stick_rates::rate_setpoint(rates_, prim::Vec3<float>(want.roll, want.pitch, want.yaw)),
                                prim::Vec3<float>(), thrust).dshot;
      rate_group::RateGroupStep reset_alt = ref_;
      r.mixed = reference_angle(want);
      reset_alt.reset_controller();
      r.reset_alt = reset_alt.execute_bypass(angle_sp_, prim::Vec3<float>(), thrust).dshot;
    } else {
      r.mixed = reference(want);
    }
    return r;
  }

  ManualControl frame_of(const Sticks& k, float arm) const {
    ManualControl m;
    m.roll = k.roll;
    m.pitch = k.pitch;
    m.yaw = k.yaw;
    m.throttle = k.throttle;
    m.channels[kLuisArmChannel - 1] = arm;
    m.channels[kAngleChannel - 1] = angle_ch_;
    return m;
  }

  RateOut hold(const Sticks& k, float arm, const Sticks& want) {
    const ManualControl m = frame_of(k, arm);
    return rate_tick(&m, want);
  }

  // Valid link with the switch low and the throttle at minimum for longer than the recovery time.
  void link_up() {
    const TimeUs until = hal_sim::stamp_us(period_, n_) + kRecoveryUs + kRecoveryUs / 2;
    while (hal_sim::stamp_us(period_, n_) <= until) {
      expect_all_stop(hold({}, kSwitchOff, {}).out);
    }
    EXPECT_EQ(composition::arming_status(), arming::ArmState::SwitchOff);
  }

  // link_up, then the switch on: armed on that rate tick.
  void arm() {
    link_up();
    const RateOut r = hold({}, kSwitchOn, {});
    ASSERT_EQ(composition::arming_status(), arming::ArmState::Armed);
    EXPECT_EQ(r.out.motor, r.mixed);
  }

  static bool is_armed_output(const Out& o) {
    for (const DshotValue& m : o.motor) {
      if (m == DshotValue::stop()) {
        return false;
      }
    }
    return true;
  }

  mixer::MixerConfig<float> mixer_{};
  rate_group::RateGroupStep ref_;
  stick_rates::RatesConfig<float> rates_{};
  std::uint32_t frames_ = 0;
  Kind kind_ = Kind::Quiet;
  prim::Vec3<float> bias_{};  // added to every sample's gyro
  prim::Vec3<float> accel_{};  // every sample's accelerometer: level, (0, 0, -9.8) m/s^2, unless a case tilts it
  prim::Vec3<float> ref_bias_{};  // what ref_ is fed less, from tick ref_bias_from_ on
  Tick ref_bias_from_ = std::numeric_limits<Tick>::max();
  rate_group::RateGroupStep fresh_;
  attitude::AngleMode<float> ref_angle_;
  attitude::AttitudeLaw<float> ref_att_;
  estimator::Mahony<float> est_ref_;
  TimeUs est_ref_prev_us_ = 0;
  bool est_ref_started_ = false;
  AttitudeState<float> ref_state_{};  // the beside estimator's state after the latest tick's sample
  float angle_ch_ = 0.0F;             // the angle switch channel's value in every staged frame (below the threshold)
  bool use_angle_ref_ = false;        // rate_tick's expected output is reference_angle, not reference
  prim::Vec3<float> angle_sp_{};      // the rate setpoint of the latest reference_angle
  bool stamp_fault_ = false;          // the sample of tick stamp_fault_tick_ is off its stamp by stamp_kind_
  Stamp stamp_kind_ = Stamp::Late;
  Tick stamp_fault_tick_ = 0;
  TimeUs last_rate_stamp_ = 0;        // the stamp of the latest rate-tick sample
  bool was_armed_ = false;
  bool reset_ref_ = true;  // ref_ restarts its controller at the arming edge as the composition does
};

TEST_F(ArmedGateTest, TheArmChannelIsLuisFifth) {
  EXPECT_EQ(param_value<ParamId::rc_channel_arm>(), static_cast<std::int32_t>(kLuisArmChannel));
}

TEST_F(ArmedGateTest, DisarmedWritesStopWithTheSticksAndTheThrottleUpAndTheSwitchNeverCycled) {
  // The switch is on from boot (boot counts as preceded by on): no rising edge, so never armed.
  const Sticks up{1.0F, -1.0F, 0.5F, 1.0F};
  for (std::size_t k = 0; k < 4000; ++k) {
    const RateOut r = hold(up, kSwitchOn, {});
    expect_all_stop(r.out);
  }
  EXPECT_NE(composition::arming_status(), arming::ArmState::Armed);
  // Switched off with the throttle up: still stop.
  for (std::size_t k = 0; k < 50; ++k) {
    expect_all_stop(hold(up, kSwitchOff, {}).out);
  }
  EXPECT_EQ(composition::arming_status(), arming::ArmState::ThrottleUp);
}

TEST_F(ArmedGateTest, ASwitchCycleArmsAndThenTheMixerOutputForTheSticksIsWrittenNotStop) {
  link_up();
  const RateOut idle = hold({}, kSwitchOn, {});
  EXPECT_EQ(composition::arming_status(), arming::ArmState::Armed);
  EXPECT_TRUE(is_armed_output(idle.out));
  EXPECT_EQ(idle.out.motor, idle.mixed);
  for (const DshotValue& m : idle.out.motor) {
    EXPECT_GE(m.raw(), prim::kDshotThrottleMin);
  }
  const Sticks fly{0.5F, -0.25F, 0.75F, 0.5F};
  RateOut r;
  for (std::size_t k = 0; k < 20; ++k) {
    r = hold(fly, kSwitchOn, fly);
    EXPECT_EQ(r.out.motor, r.mixed) << k;
  }
  EXPECT_NE(r.out.motor, idle.out.motor);
  // Full throttle gives more collective thrust than idle; the stick makes the motors differ.
  const Sticks full{0.0F, 0.0F, 0.0F, 1.0F};
  RateOut top;
  for (std::size_t k = 0; k < 20; ++k) {
    top = hold(full, kSwitchOn, full);
    EXPECT_EQ(top.out.motor, top.mixed) << k;
  }
  std::uint32_t sum_idle = 0;
  std::uint32_t sum_top = 0;
  for (std::size_t i = 0; i < composition::kMotors; ++i) {
    sum_idle += idle.out.motor[i].raw();
    sum_top += top.out.motor[i].raw();
  }
  EXPECT_GT(sum_top, sum_idle);
  // Motors 1 and 3 spin opposite ways, so the yaw stick separates them on any geometry (decision 0044).
  EXPECT_NE(r.out.motor[0], r.out.motor[2]);
  // The switch off disarms on that rate tick.
  expect_all_stop(hold(fly, kSwitchOff, fly).out);
  EXPECT_EQ(composition::arming_status(), arming::ArmState::SwitchOff);
}

TEST_F(ArmedGateTest, TheThrottleUpRefusesAndALateCycleArms) {
  link_up();
  const Sticks up{0.0F, 0.0F, 0.0F, 0.5F};
  for (std::size_t k = 0; k < 10; ++k) {
    expect_all_stop(hold(up, kSwitchOn, {}).out);
    EXPECT_EQ(composition::arming_status(), arming::ArmState::ThrottleUp);
  }
  // Throttle down with the switch still on: the edge was spent.
  for (std::size_t k = 0; k < 10; ++k) {
    expect_all_stop(hold({}, kSwitchOn, {}).out);
    EXPECT_EQ(composition::arming_status(), arming::ArmState::SwitchNotCycled);
  }
  expect_all_stop(hold({}, kSwitchOff, {}).out);
  EXPECT_TRUE(is_armed_output(hold({}, kSwitchOn, {}).out));
  EXPECT_EQ(composition::arming_status(), arming::ArmState::Armed);
}

TEST_F(ArmedGateTest, ALostLinkHoldsThenCentresThenWritesStopAtTheDisarmTimeOnThatRateTick) {
  arm();
  const Sticks fly{0.5F, 0.0F, 0.0F, 0.5F};
  const RateOut last = hold(fly, kSwitchOn, fly);
  ASSERT_EQ(last.out.motor, last.mixed);
  const TimeUs t0 = hal_sim::stamp_us(period_, n_ - 1);
  bool stopped = false;
  std::size_t armed_ticks = 0;
  for (std::size_t k = 0; k < 20000 && !stopped; ++k) {
    const TimeUs now = hal_sim::stamp_us(period_, ((n_ + divisor_ - 1) / divisor_) * divisor_);
    const TimeUs age = now - t0;
    const Sticks want = age < kHoldUs ? fly : Sticks{};
    const RateOut r = rate_tick(nullptr, want);
    if (age < kDisarmUs) {
      ASSERT_EQ(r.out.motor, r.mixed) << "age " << age;
      EXPECT_EQ(composition::arming_status(), age < kLossUs ? arming::ArmState::Armed
                                              : age < kHoldUs ? arming::ArmState::ArmedHold
                                                              : arming::ArmState::ArmedFailsafeCentred)
          << "age " << age;
      ++armed_ticks;
    } else {
      expect_all_stop(r.out);
      EXPECT_EQ(composition::arming_status(), arming::ArmState::Failsafe);
      stopped = true;
    }
  }
  EXPECT_TRUE(stopped);
  EXPECT_GT(armed_ticks, 1000U);
  // Still stop, with the switch on and no cycle, until the link is back long enough and the switch cycled.
  for (std::size_t k = 0; k < 20; ++k) {
    expect_all_stop(hold({}, kSwitchOn, {}).out);
  }
}

TEST_F(ArmedGateTest, ADeadLinkFromTheStartNeverArms) {
  for (std::size_t k = 0; k < 4000; ++k) {
    expect_all_stop(rate_tick(nullptr, {}).out);
  }
  EXPECT_EQ(composition::arming_status(), arming::ArmState::NoLink);
}

TEST_F(ArmedGateTest, NonFiniteAndOutOfRangeSticksAndThrottleWhileArmedAreLimitedBeforeTheyReachTheRateLoop) {
  arm();
  constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
  constexpr float kInf = std::numeric_limits<float>::infinity();
  const Sticks bad{kNaN, 5.0F, -kInf, 7.0F};
  const Sticks limited_to{0.0F, 1.0F, 0.0F, 1.0F};
  for (std::size_t k = 0; k < 20; ++k) {
    const RateOut r = hold(bad, kSwitchOn, limited_to);
    ASSERT_EQ(composition::arming_status(), arming::ArmState::Armed);
    EXPECT_EQ(r.out.motor, r.mixed) << k;
  }
  const Sticks neg{-3.0F, 0.0F, 0.0F, -2.0F};
  const Sticks neg_limited{-1.0F, 0.0F, 0.0F, 0.0F};
  for (std::size_t k = 0; k < 20; ++k) {
    const RateOut r = hold(neg, kSwitchOn, neg_limited);
    EXPECT_EQ(r.out.motor, r.mixed) << k;
  }
}

TEST_F(ArmedGateTest, AMotorTestRequestWhileArmedIsRefusedAndCountedAndTheVehicleKeepsFlying) {
  arm();
  const Sticks fly{0.0F, 0.0F, 0.0F, 0.3F};
  hold(fly, kSwitchOn, fly);
  const std::uint32_t events = hal_sim::motor_test_event_count();
  hal_sim::stage_motor_test_request({1, 100, 50});
  const RateOut r = hold(fly, kSwitchOn, fly);
  EXPECT_EQ(r.out.motor, r.mixed);
  ASSERT_EQ(hal_sim::motor_test_event_count(), events + 1);
  const MotorTestEvent e = hal_sim::last_motor_test_event();
  EXPECT_EQ(e.kind, MotorTestEventKind::Refused);
  EXPECT_EQ(e.reason, MotorTestReason::Armed);
  EXPECT_EQ(e.refused, 1U);
  EXPECT_EQ(e.accepted, 0U);
  for (std::size_t k = 0; k < 200; ++k) {
    const RateOut f = hold(fly, kSwitchOn, fly);
    EXPECT_EQ(f.out.motor, f.mixed) << k;
  }
  EXPECT_EQ(hal_sim::motor_test_event_count(), events + 1);
  // Disarmed, the same request is accepted and spins exactly its motor.
  expect_all_stop(hold(fly, kSwitchOff, {}).out);
  hal_sim::stage_motor_test_request({3, 100, 50});
  expect_only(hold({}, kSwitchOff, {}).out, 2, 100);
  EXPECT_EQ(hal_sim::last_motor_test_event().kind, MotorTestEventKind::Accepted);
}

TEST_F(ArmedGateTest, ArmingIsRefusedWhileAMotorTestRunsAndNeverTwoAtOnce) {
  link_up();
  hal_sim::stage_motor_test_request({2, 100, kLuisMaxDurationMs});
  expect_only(hold({}, kSwitchOff, {}).out, 1, 100);
  // The switch on (a rising edge) during the test: refused, the test keeps running, only motor 2 nonzero.
  for (std::size_t k = 0; k < 50; ++k) {
    const Out o = hold({}, kSwitchOn, {}).out;
    expect_only(o, 1, 100);
    EXPECT_EQ(composition::arming_status(), arming::ArmState::MotorTest);
  }
  hal_sim::stage_motor_test_cancel(MotorTestCancel::Cancel);
  expect_all_stop(hold({}, kSwitchOn, {}).out);
  EXPECT_EQ(hal_sim::last_motor_test_event().kind, MotorTestEventKind::Stopped);
  // The test is over but the edge was spent: still stop, then a cycle arms.
  for (std::size_t k = 0; k < 10; ++k) {
    expect_all_stop(hold({}, kSwitchOn, {}).out);
  }
  EXPECT_EQ(composition::arming_status(), arming::ArmState::SwitchNotCycled);
  expect_all_stop(hold({}, kSwitchOff, {}).out);
  const RateOut r = hold({}, kSwitchOn, {});
  EXPECT_EQ(composition::arming_status(), arming::ArmState::Armed);
  EXPECT_EQ(r.out.motor, r.mixed);
}

TEST_F(ArmedGateTest, AnImuDeadlineMissDisarmsOnThatRateTickAndKeepsArmingRefused) {
  arm();
  const Sticks fly{0.0F, 0.0F, 0.0F, 0.3F};
  for (std::size_t k = 0; k < 10; ++k) {
    const RateOut r = hold(fly, kSwitchOn, fly);
    EXPECT_EQ(r.out.motor, r.mixed);
  }
  hal_sim::stage_imu_deadline_misses(1);
  const RateOut r = hold(fly, kSwitchOn, fly);
  expect_all_stop(r.out);
  EXPECT_EQ(composition::arming_status(), arming::ArmState::ImuStale);
  // The ticks after the miss, and a full switch cycle with a good link: stop, still ImuStale.
  for (std::size_t k = 0; k < 20; ++k) {
    expect_all_stop(hold({}, kSwitchOff, {}).out);
  }
  for (std::size_t k = 0; k < 20; ++k) {
    expect_all_stop(hold({}, kSwitchOn, {}).out);
    EXPECT_EQ(composition::arming_status(), arming::ArmState::ImuStale);
  }
}

TEST_F(ArmedGateTest, ASaturatedSampleSinceTheLastRateTickRefusesArmingAndNeverDisarms) {
  link_up();
  // The saturated sample is on the tick before the rate tick: it counts for that rate tick, whose own sample is fine.
  while ((n_ + 1) % divisor_ != 0) {
    step_tick();
  }
  kind_ = Kind::Saturated;
  step_tick();
  kind_ = Kind::Quiet;
  expect_all_stop(hold({}, kSwitchOn, {}).out);
  EXPECT_EQ(composition::arming_status(), arming::ArmState::Saturated);
  expect_all_stop(hold({}, kSwitchOff, {}).out);
  const RateOut r = hold({}, kSwitchOn, {});
  EXPECT_EQ(composition::arming_status(), arming::ArmState::Armed);
  EXPECT_EQ(r.out.motor, r.mixed);
  // Armed, a saturated sample changes nothing.
  while ((n_ + 1) % divisor_ != 0) {
    step_tick();
  }
  kind_ = Kind::Saturated;
  step_tick();
  kind_ = Kind::Quiet;
  const RateOut s2 = hold({}, kSwitchOn, {});
  EXPECT_EQ(composition::arming_status(), arming::ArmState::Armed);
  EXPECT_EQ(s2.out.motor, s2.mixed);
}

TEST_F(ArmedGateTest, ArmingStartsTheRateLoopFromZeroStateNotFromWhatWasAccumulatedWhileDisarmed) {
  // A constant gyro bias while disarmed winds the rate loop's integrator (the beside group `ref_` runs as the composition
  // would without the reset). The first armed output, sticks centred and throttle low, equals that of a freshly
  // initialised rate group fed the same samples.
  bias_ = prim::Vec3<float>(0.2F, -0.3F, 0.15F);
  reset_ref_ = false;
  link_up();
  fresh_.init(rate::load_config(), mixer_, rate_group::load_chain_config());
  const float t_min = static_cast<float>(composition::kMotors) * mixer::f_min(mixer_);
  const ManualControl m = frame_of({}, kSwitchOn);
  while (n_ % divisor_ != 0) {
    step_tick();
  }
  ManualControl staged = m;
  staged.t_us = hal_sim::stamp_us(period_, n_);
  staged.frames = ++frames_;
  hal_sim::stage_manual_control(staged);
  const Out first = step_tick();
  ASSERT_EQ(composition::arming_status(), arming::ArmState::Armed);
  const auto fresh = fresh_.execute(prim::Vec3<float>(), prim::Vec3<float>(), t_min).dshot;
  const auto wound = reference({});
  EXPECT_NE(wound, fresh) << "the bias must have wound the integrator, or the test proves nothing";
  EXPECT_EQ(first.motor, fresh);
}


// ---- Angle mode (M4 A2, decision 0043 item 3) ----

// The switch comparison as the test states it, independently of the parameter table: angle at or above 1700 us, normalized
// (1700 - 1500) / 512 (the arm switch's rule, decision 0043).
constexpr float kAngleThreshold = 0.390625F;
// Smallest difference between two rate setpoints the controls count as different (scenario test value, rad/s).
constexpr float kSetpointApart = 0.01F;

float max_abs_diff(const prim::Vec3<float>& a, const prim::Vec3<float>& b) {
  float m = 0.0F;
  for (std::size_t i = 0; i < 3; ++i) {
    m = std::max(m, std::fabs(a[i] - b[i]));
  }
  return m;
}

class AngleGateTest : public ArmedGateTest {
 protected:
  // The switch up: from the next rate tick the composition resets AngleMode and AttitudeLaw, and so does the reference.
  void enter_angle(float value = kSwitchOn) {
    angle_ch_ = value;
    use_angle_ref_ = true;
    reset_angle_reference();
  }

  void leave_angle() {
    angle_ch_ = kSwitchOff;
    use_angle_ref_ = false;
  }

  // Control: for `want` the angle law (from a fresh copy of the reference's state) and the rate curve give different rate
  // setpoints, so an output equal to the one and not the other is a decision between two different things.
  void expect_laws_differ(const Sticks& want) {
    attitude::AngleMode<float> a = ref_angle_;
    attitude::AttitudeLaw<float> l = ref_att_;
    a.reset();
    l.reset();
    const prim::Vec3<float> angle = angle_setpoint(want, a, l);
    const prim::Vec3<float> acro = stick_rates::rate_setpoint(rates_, prim::Vec3<float>(want.roll, want.pitch, want.yaw));
    EXPECT_GT(max_abs_diff(angle, acro), kSetpointApart);
  }
};

TEST_F(AngleGateTest, TheAngleSwitchDefaultsToNoSwitchAndTheThresholdIsTheArmSwitchsRule) {
  const std::span<const ParamRecord, kParamCount> d = param_defaults();
  EXPECT_EQ(d[static_cast<std::size_t>(ParamId::rc_channel_angle)].value.i32, 0);
  EXPECT_EQ(d[static_cast<std::size_t>(ParamId::angle_switch_threshold)].value.f32, kAngleThreshold);
  EXPECT_EQ(d[static_cast<std::size_t>(ParamId::angle_switch_threshold)].value.f32,
            d[static_cast<std::size_t>(ParamId::arm_switch_threshold)].value.f32);
  EXPECT_EQ(param_value<ParamId::rc_channel_angle>(), kAngleChannel);  // the test process's override
}

// (a) Control: the switch below the threshold is acro, whatever the sticks.
TEST_F(AngleGateTest, TheSwitchBelowTheThresholdFliesAcro) {
  arm();
  const Sticks fly{0.5F, -0.25F, 0.75F, 0.5F};
  expect_laws_differ(fly);
  for (const float v : {kSwitchOff, 0.0F, std::nextafter(kAngleThreshold, 0.0F)}) {
    angle_ch_ = v;
    for (std::size_t k = 0; k < 20; ++k) {
      const RateOut r = hold(fly, kSwitchOn, fly);
      ASSERT_EQ(composition::arming_status(), arming::ArmState::Armed);
      EXPECT_EQ(r.out.motor, r.mixed) << "channel " << v << " tick " << k;
    }
  }
  EXPECT_EQ(composition::attitude_counts().faults, 0U);
}

// (b) The switch at the threshold (and above it), sticks centred, level quiet samples: the attitude law's output.
TEST_F(AngleGateTest, TheSwitchAtOrAboveTheThresholdWithCentredSticksFliesTheAttitudeLaw) {
  arm();
  enter_angle(kAngleThreshold);
  for (const float v : {kAngleThreshold, kSwitchOn}) {
    angle_ch_ = v;  // still in angle: no reset on either side
    for (std::size_t k = 0; k < 30; ++k) {
      const Sticks centred{0.0F, 0.0F, 0.0F, 0.5F};
      const RateOut r = hold(centred, kSwitchOn, centred);
      ASSERT_EQ(composition::arming_status(), arming::ArmState::Armed);
      EXPECT_EQ(r.out.motor, r.mixed) << "channel " << v << " tick " << k;
    }
  }
  EXPECT_EQ(composition::attitude_counts().faults, 0U);
  EXPECT_EQ(composition::attitude_counts().timing_faults, 0U);
}

// (c) A roll stick in angle mode: the attitude law's output, and not acro's.
TEST_F(AngleGateTest, ARollStickInAngleModeGivesTheAttitudeLawsOutputAndNotAcros) {
  arm();
  enter_angle();
  const Sticks roll{0.3F, 0.0F, 0.0F, 0.5F};
  expect_laws_differ(roll);
  for (std::size_t k = 0; k < 30; ++k) {
    const RateOut r = hold(roll, kSwitchOn, roll);
    ASSERT_EQ(composition::arming_status(), arming::ArmState::Armed);
    EXPECT_EQ(r.out.motor, r.mixed) << k;
    if (k == 0) {  // acro's setpoint prefilter starts from rest; the bypassed setpoint is applied at once
      EXPECT_NE(r.out.motor, r.acro);
    }
  }
  EXPECT_EQ(composition::attitude_counts().faults, 0U);
}

// (d) A channel value that is not a number is acro, with the same sticks that give a different output in angle mode.
TEST_F(AngleGateTest, ANotANumberSwitchFliesAcro) {
  arm();
  const Sticks roll{0.3F, 0.0F, 0.0F, 0.5F};
  expect_laws_differ(roll);
  angle_ch_ = std::numeric_limits<float>::quiet_NaN();
  for (std::size_t k = 0; k < 20; ++k) {
    const RateOut r = hold(roll, kSwitchOn, roll);
    ASSERT_EQ(composition::arming_status(), arming::ArmState::Armed);
    EXPECT_EQ(r.out.motor, r.mixed) << k;
  }
}

// (e) AngleMode and AttitudeLaw are reset on the first rate tick of armed-and-angle and on no other. A constant yaw rate on the
// gyro (1 rad/s, a scenario value, from after the calibration) turns the estimate away from the heading the mode locks at
// its first execution, so the yaw setpoint grows with the lock held and is about 0 after a reset. The composition's output
// equals the reference that resets only at entry, for a run staying in angle mode, and again after angle -> acro -> angle,
// where the reference resets again. Controls: the setpoint of a reset-at-every-tick law differs from the persistent one, and
// at the second entry the setpoint of a law whose AngleMode keeps the old lock differs from the reset one.
TEST_F(AngleGateTest, TheResetsHappenOnTheEntryTickOnly) {
  arm();
  bias_ = prim::Vec3<float>(0.0F, 0.0F, 1.0F);
  const Sticks centred{0.0F, 0.0F, 0.0F, 0.5F};
  for (std::size_t k = 0; k < 40; ++k) {
    const RateOut r = hold(centred, kSwitchOn, centred);
    ASSERT_EQ(r.out.motor, r.mixed);
  }
  enter_angle();
  RateOut r;
  for (std::size_t k = 0; k < 300; ++k) {
    r = hold(centred, kSwitchOn, centred);
    ASSERT_EQ(composition::arming_status(), arming::ArmState::Armed);
    EXPECT_EQ(r.out.motor, r.mixed) << "first stay " << k;
  }
  {
    attitude::AngleMode<float> a = ref_angle_;
    attitude::AttitudeLaw<float> l = ref_att_;
    a.reset();
    l.reset();
    EXPECT_GT(max_abs_diff(angle_sp_, angle_setpoint(centred, a, l)), kSetpointApart) << "the lock is not acting";
  }
  leave_angle();
  for (std::size_t k = 0; k < 40; ++k) {
    r = hold(centred, kSwitchOn, centred);
    EXPECT_EQ(r.out.motor, r.mixed) << "acro " << k;
  }
  attitude::AngleMode<float> stale_angle = ref_angle_;
  attitude::AttitudeLaw<float> stale_law = ref_att_;
  stale_law.reset();
  enter_angle();
  for (std::size_t k = 0; k < 300; ++k) {
    r = hold(centred, kSwitchOn, centred);
    EXPECT_EQ(r.out.motor, r.mixed) << "second stay " << k;
    if (k == 0) {
      EXPECT_GT(max_abs_diff(angle_sp_, angle_setpoint(centred, stale_angle, stale_law)), kSetpointApart)
          << "the old lock would not differ";
    }
  }
  EXPECT_EQ(composition::attitude_counts().faults, 0U);
}

// (f) The rate loop's integrator is not reset by a change of mode: a constant gyro bias on roll and pitch (scenario values,
// from after arming) winds it in acro; the first angle tick and the ticks after equal the beside group `ref_`, which is
// restarted at the arming edge only (never at the switch). Control: the same tick from a copy of that group restarted at the
// switch gives a different output, so the integrator was wound.
TEST_F(AngleGateTest, TheRateIntegratorIsNotResetByAModeSwitch) {
  arm();
  bias_ = prim::Vec3<float>(0.05F, -0.05F, 0.0F);
  const Sticks centred{0.0F, 0.0F, 0.0F, 0.5F};
  for (std::size_t k = 0; k < 400; ++k) {
    const RateOut r = hold(centred, kSwitchOn, centred);
    ASSERT_EQ(r.out.motor, r.mixed);
  }
  enter_angle();
  const RateOut first = hold(centred, kSwitchOn, centred);
  ASSERT_EQ(composition::arming_status(), arming::ArmState::Armed);
  EXPECT_EQ(first.out.motor, first.mixed);
  EXPECT_NE(first.out.motor, first.reset_alt) << "the integrator was not wound, or the test is insensitive to it";
  for (std::size_t k = 0; k < 30; ++k) {
    const RateOut r = hold(centred, kSwitchOn, centred);
    EXPECT_EQ(r.out.motor, r.mixed) << k;
  }
  leave_angle();
  for (std::size_t k = 0; k < 30; ++k) {
    const RateOut r = hold(centred, kSwitchOn, centred);
    EXPECT_EQ(r.out.motor, r.mixed) << "back in acro " << k;
  }
}

// (g) With angle selected and armed, one rate-tick sample off its stamp (as OffPeriodStampsNeverHaltAndAreCountedTimingFaults)
// never halts, and the attitude law counts the spacing faults the rate loop counts (two per displaced or repeated stamp), with
// no fault of any other kind. A missing sample is not covered here: the fixture's tick numbering is the composition's.
class AngleTimingTest : public AngleGateTest {
 protected:
  void run(Stamp kind) {
    arm();
    enter_angle();
    const Sticks centred{0.0F, 0.0F, 0.0F, 0.5F};
    for (std::size_t k = 0; k < 40; ++k) {
      const RateOut r = hold(centred, kSwitchOn, centred);
      ASSERT_EQ(r.out.motor, r.mixed);
    }
    EXPECT_EQ(composition::rate_counts().timing_faults, 0U);
    EXPECT_EQ(composition::attitude_counts().timing_faults, 0U);
    stamp_fault_ = true;
    stamp_kind_ = kind;
    stamp_fault_tick_ = ((n_ + divisor_ - 1) / divisor_) * divisor_;  // the next rate tick
    for (std::size_t k = 0; k < 40; ++k) {
      (void)hold(centred, kSwitchOn, centred);
      ASSERT_EQ(composition::arming_status(), arming::ArmState::Armed);
    }
    EXPECT_EQ(composition::rate_counts().timing_faults, 2U);
    EXPECT_EQ(composition::attitude_counts().timing_faults, composition::rate_counts().timing_faults);
    EXPECT_EQ(composition::attitude_counts().faults, composition::attitude_counts().timing_faults);
  }
};

TEST_F(AngleTimingTest, ALateStampIsCountedByBothLoopsAndNeverHalts) { run(Stamp::Late); }
TEST_F(AngleTimingTest, AnEarlyStampIsCountedByBothLoopsAndNeverHalts) { run(Stamp::Early); }
TEST_F(AngleTimingTest, ARepeatedStampIsCountedByBothLoopsAndNeverHalts) { run(Stamp::Repeated); }
TEST_F(AngleTimingTest, ABackwardsStampIsCountedByBothLoopsAndNeverHalts) { run(Stamp::Backwards); }

// (h) init refuses an angle channel outside 0..16 and the arm or a stick channel (the register's AETR map: sticks 1 to 4, arm
// 5), and accepts the others, 0 (no switch) included. Each case is a fresh process (threadsafe death tests), since the
// parameter table loads once per process.
void init_with_angle_channel(std::int32_t angle_channel, std::int32_t att_loop_ratio = 0) {
  (void)prctl(PR_SET_DUMPABLE, 0);
  if (!params_init(table_with_angle_channel(angle_channel, att_loop_ratio))) {
    std::exit(2);
  }
  std::array<DshotValue, composition::kMotors> motor_latch{};
  std::array<ServoUs, composition::kServos> servo_latch{};
  hal_sim::setup({static_cast<std::uint32_t>(param_value<ParamId::tick_period_num_us>()),
                  static_cast<std::uint32_t>(param_value<ParamId::tick_period_den>())},
                 motor_latch, servo_latch);
  composition::init();
  std::exit(0);
}

TEST(QuadFcAngleInitDeathTest, InitRefusesABadAngleChannelAndAcceptsTheOthers) {
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  for (const std::int32_t bad : {-1, 17, 1, 2, 3, 4, 5}) {
    EXPECT_EXIT(init_with_angle_channel(bad), ::testing::KilledBySignal(SIGABRT),
                bad < 0 || bad > 16 ? "hal_panic: quad_fc composition: rc_channel_angle is outside 0..16"
                                    : "hal_panic: quad_fc composition: rc_channel_angle is the arm channel or a stick channel")
        << bad;
  }
  for (const std::int32_t good : {0, 6, 16}) {
    EXPECT_EXIT(init_with_angle_channel(good), ::testing::ExitedWithCode(0), "") << good;
  }
}

// ---- The boot gyro calibration and the upright check (M3 P6, decision 0042) ----

// The fixture without the warm-up: the case itself runs the calibration.
class CalibrationGateTest : public ArmedGateTest {
 protected:
  void warm_up() override {}

  // Ticks (from tick 0, level samples, the link up, the switch low) to the first rate tick at or after `until_us` on the stamps
  // and returns the state there; every rate tick on the way writes stop.
  arming::ArmState run_to(TimeUs until_us) {
    while (hal_sim::stamp_us(period_, n_) < until_us) {
      expect_all_stop(hold({}, kSwitchOff, {}).out);
    }
    return composition::arming_status();
  }
};

// Arming is refused (Calibrating) until the calibration duration of still samples has passed, counted from the first sample's
// stamp: from the recovery time of the link (before it NoLink) up to the last rate tick before the duration the state is
// Calibrating and at the first rate tick at or after it the state is SwitchOff (everything else ready). Betaflight's duration is
// the number the test states (kCalibrationUs); the table must hold it.
TEST_F(CalibrationGateTest, ArmingIsRefusedUntilTheCalibrationDurationOfStillSamplesHasPassed) {
  EXPECT_EQ(param_value<ParamId::gyro_cal_duration_us>(), static_cast<std::int32_t>(kCalibrationUs));
  const TimeUs first_us = hal_sim::stamp_us(period_, 0);
  bool left = false;
  while (!left) {
    const Out o = hold({}, kSwitchOff, {}).out;
    const TimeUs age = hal_sim::stamp_us(period_, n_ - 1) - first_us;
    expect_all_stop(o);
    const arming::ArmState state = composition::arming_status();
    if (age < kRecoveryUs) {
      EXPECT_EQ(state, arming::ArmState::NoLink) << "age " << age;
    } else if (age < kCalibrationUs) {
      EXPECT_EQ(state, arming::ArmState::Calibrating) << "age " << age;
    } else {
      EXPECT_EQ(state, arming::ArmState::SwitchOff) << "age " << age;
      left = true;
    }
  }
  // Calibrated: the switch cycle now arms, and Calibrating does not return.
  EXPECT_EQ(run_to(hal_sim::stamp_us(period_, n_) + kRecoveryUs), arming::ArmState::SwitchOff);
  EXPECT_TRUE(is_armed_output(hold({}, kSwitchOn, {}).out));
  EXPECT_EQ(composition::arming_status(), arming::ArmState::Armed);
}

// A constant gyro bias is subtracted once the calibration is complete, from the rate loop's chain as well as the estimator's
// input. Rate loop: the bias is 0.2, -0.3, 0.1 rad/s on the three axes (scenario values, 1 % to 3 % of the rate loop's
// range; far above what the integrator's reset would hide); the first armed output equals that of a beside rate group
// (ref_) fed the same samples less the bias from the first tick the calibration is complete, with sticks centred. Control:
// the same group fed the samples as they are, from then on, gives a different output after the same ticks, so the equality
// is the subtraction and not an insensitive output.
TEST_F(CalibrationGateTest, TheCalibratedBiasIsSubtractedFromTheRateLoopsGyroChain) {
  bias_ = prim::Vec3<float>(0.2F, -0.3F, 0.1F);
  const TimeUs first_us = hal_sim::stamp_us(period_, 0);
  Tick done = 0;  // the first tick whose stamp is a calibration duration after tick 0's: the first with the bias subtracted
  while (hal_sim::stamp_us(period_, done) - first_us < kCalibrationUs) {
    ++done;
  }
  ref_bias_ = bias_;
  ref_bias_from_ = done;
  EXPECT_EQ(run_to(first_us + kCalibrationUs + kRecoveryUs), arming::ArmState::SwitchOff);
  RateOut r = hold({}, kSwitchOn, {});
  ASSERT_EQ(composition::arming_status(), arming::ArmState::Armed);
  for (std::size_t k = 0; k < 50; ++k) {
    EXPECT_EQ(r.out.motor, r.mixed) << k;
    r = hold({}, kSwitchOn, {});
  }
  ref_bias_from_ = n_;
  ref_bias_ = prim::Vec3<float>();
  for (std::size_t k = 0; k < 50; ++k) {
    r = hold({}, kSwitchOn, {});
  }
  EXPECT_NE(r.out.motor, r.mixed) << "a group fed the bias gives the same output: the test is insensitive to it";
}

// The estimator's input is the corrected gyro: a constant bias (0.02, -0.03, 0.01 rad/s, scenario values) fed from boot tilts
// the estimate while the calibration runs (the correction holds it to about bias / Kp), and from the first corrected sample on
// the estimator sees a zero rate, so the tilt decays at Kp: after T more seconds it is at most tilt_at_calibration
// x exp(-Kp T) x 1.1 (the small-angle decay, 10 % margin) + 1e-4 rad. Fed uncorrected the tilt would settle near |bias_xy| / Kp
// = 0.14 rad instead; here T = 12 s, so the bound is about 2e-3 rad.
TEST_F(CalibrationGateTest, TheCalibratedBiasIsSubtractedFromTheEstimatorsInput) {
  bias_ = prim::Vec3<float>(0.02F, -0.03F, 0.01F);
  const TimeUs first_us = hal_sim::stamp_us(period_, 0);
  TimeUs at_cal_us = 0;
  float tilt_at_cal = 0.0F;
  const TimeUs end_us = first_us + kCalibrationUs + 12000000;
  bool calibrated = false;
  while (hal_sim::stamp_us(period_, n_) < end_us) {
    expect_all_stop(hold({}, kSwitchOff, {}).out);
    if (!calibrated && composition::arming_status() == arming::ArmState::SwitchOff) {
      calibrated = true;
      at_cal_us = hal_sim::stamp_us(period_, n_ - 1);
      tilt_at_cal = composition::attitude_status().tilt_rad;
    }
  }
  ASSERT_TRUE(calibrated);
  const double tilt_end = static_cast<double>(composition::attitude_status().tilt_rad);
  const double seconds = static_cast<double>(hal_sim::stamp_us(period_, n_ - 1) - at_cal_us) / 1.0e6;
  EXPECT_GT(tilt_at_cal, 0.01F) << "the bias did not tilt the estimate while the calibration ran";
  EXPECT_LE(tilt_end, 1.1 * static_cast<double>(tilt_at_cal) * std::exp(-static_cast<double>(param_value<ParamId::estimator_kp>()) * seconds) + 1.0e-4);
}

// The upright check (Betaflight 4.5.1 imu.c:856-859 isUpright, small_angle 25 deg, imu.c:119-121): the estimate's tilt above
// arm_small_angle refuses arming (NotUpright), below it does not. The vehicle sits rolled by 30 deg and by 20 deg (scenario
// values, 5 deg either side of the limit) from boot, the estimate initialised from the first sample.
TEST_F(CalibrationGateTest, ATiltAboveTheSmallAngleRefusesArmingAndBelowItDoesNot) {
  constexpr double kPi = 3.14159265358979323846;
  EXPECT_NEAR(static_cast<double>(param_value<ParamId::arm_small_angle>()), 25.0 * kPi / 180.0, 1.0e-6);
  for (const double roll_deg : {30.0, 20.0}) {
    hal_sim::setup(period_, motor_latch_, servo_latch_);
    composition::init();
    n_ = 0;
    frames_ = 0;
    const double roll = roll_deg * kPi / 180.0;
    const float g = -accel_[2];
    accel_ = prim::Vec3<float>(0.0F, -g * static_cast<float>(std::sin(roll)), -g * static_cast<float>(std::cos(roll)));
    const arming::ArmState state = run_to(kCalibrationUs + kRecoveryUs);
    EXPECT_EQ(state, roll_deg > 25.0 ? arming::ArmState::NotUpright : arming::ArmState::SwitchOff) << roll_deg << " deg";
    EXPECT_NEAR(static_cast<double>(composition::attitude_status().tilt_rad), roll, 1.0e-3) << roll_deg << " deg";
    for (std::size_t k = 0; k < 5; ++k) {
      const Out o = hold({}, kSwitchOn, {}).out;
      EXPECT_EQ(is_armed_output(o), roll_deg < 25.0) << roll_deg << " deg";
    }
    accel_ = sample_of(Kind::Quiet, 0).accel_m_s2;
  }
}

// The law runs once per rate tick, which is its period only for att_loop_ratio 1: init refuses another ratio.
TEST(QuadFcAngleInitDeathTest, InitRefusesAnAttitudeLoopRatioOtherThanOne) {
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  for (const std::int32_t ratio : {2, 3}) {
    EXPECT_EXIT(init_with_angle_channel(kAngleChannel, ratio), ::testing::KilledBySignal(SIGABRT),
                "hal_panic: quad_fc composition: att_loop_ratio is not 1")
        << ratio;
  }
}

}  // namespace
}  // namespace marv
