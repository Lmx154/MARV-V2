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
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include <marv/composition.hpp>
#include <marv/hal/hal.hpp>
#include <marv/hal/tick.hpp>
#include <marv/hal_sim/hal_sim.hpp>
#include <marv/mixer/mixer.hpp>
#include <marv/params/param.hpp>
#include <marv/prim/constants.hpp>
#include <marv/prim/vec.hpp>
#include <marv/rate/rate_loop.hpp>
#include <marv/rate_group/rate_group.hpp>
#include <marv/types/actuator.hpp>
#include <marv/types/imu_sample.hpp>

namespace marv {
namespace {

constexpr std::size_t kTicks = 400;
constexpr std::uint16_t kSentinelRaw = 1000;

void init_params_once() {
  static const bool ok = params_init(param_defaults());
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

}  // namespace
}  // namespace marv
