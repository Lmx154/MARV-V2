// The quad_fc motor output gate (M1: one state, Disarmed). Through hal_sim: every rate tick writes DshotValue::stop() to
// every motor, a tick that is not a rate tick writes nothing, and the mixer's own output (never stop: its floor is the
// idle command) never reaches the latch, for samples of every kind the rate loop handles (valid, saturated, invalid).
// Flight timing never halts it (quad spec 4.0 M1, decision 0034): a late, early, repeated or backwards stamp, or a
// missing sample, is a counted timing fault of the rate loop.
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include <marv/composition.hpp>
#include <marv/hal/hal.hpp>
#include <marv/hal/tick.hpp>
#include <marv/hal_sim/hal_sim.hpp>
#include <marv/mixer/mixer.hpp>
#include <marv/params/param.hpp>
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

}  // namespace
}  // namespace marv
