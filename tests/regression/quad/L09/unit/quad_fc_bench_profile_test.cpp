// The bench motor-step profile (unit U3 BENCH-PROFILE, architect ruling 2026-10-18 item 8; invariant L-7): the composition
// variant marv_composition_quad_fc_bench_profile (quad_fc.cpp with MARV_QUAD_FC_BENCH_PROFILE) through hal_sim on the FC3v2 set
// with the bench-profile register (marv_params_fc3v2_bench_profile).
//
// L-7: the gate writes a profile value only while the arming state is exactly Armed, the bench switch has been in position on
// every rate tick since the arm edge (that tick included) and the profile has not ended; on every other rate tick it writes
// stop to every motor. A kill (the bench switch out, the arm switch off, the link age at failsafe_loss_us, a disarm) stops it on
// the rate tick the kill is visible and it stays stopped until the next arm edge, whatever comes back (the lead 2026-10-18). It
// runs once per arm, never above the bench cap, in the ruling's order and timing to the tick.
//
// Luis's numbers are stated here independently of the parameter table (TheBenchParametersAreLuisRulings checks the table holds
// them). The profile they define is written out as a list of segments (lead-in, then motor by motor, repeat by repeat, level by
// level) and the expected value of a rate tick is the segment its nominal time falls in. The sweeps of the kills over every step
// run a compressed clock (kFast, a scenario test value injected through the table: the ruling's levels, repeats and motors on a
// 2 ms lead-in and 1 ms levels), so every step is a few rate ticks away; the ruling's own clock runs whole in the sequence test
// and at one step per kill in the props-off kill test.
//
// The parameter table loads once per process, so every test that initialises runs its body in a forked child (in_child, the
// pattern of sil_quad_fc_test.cpp).
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <marv/arming/arming.hpp>
#include <marv/bench_profile.hpp>
#include <marv/composition.hpp>
#include <marv/hal/hal.hpp>
#include <marv/hal/tick.hpp>
#include <marv/hal_sim/hal_sim.hpp>
#include <marv/params/param.hpp>
#include <marv/prim/constants.hpp>
#include <marv/prim/vec.hpp>
#include <marv/types/actuator.hpp>
#include <marv/types/imu_sample.hpp>
#include <marv/types/manual_control.hpp>

namespace marv {
namespace {

using arming::ArmState;
using composition::BenchProfilePhase;
using composition::BenchProfileStatus;
using composition::kMotors;

// ---- Luis's rulings of 2026-10-18 (AskUserQuestion), stated independently of the parameter table ----
constexpr std::int32_t kLuisBenchChannel = 6;        // "ch6 fully up (Recommended)"
constexpr float kLuisBenchThreshold = 0.390625F;     // fully up = +100, at or above 1700 us (the arm switch's rule)
constexpr std::uint64_t kLuisLeadInUs = 2000000;     // "2 s lead-in after arming"
constexpr std::uint64_t kLuisLevelUs = 1000000;      // "1 s per level"
constexpr std::uint16_t kLuisBase = 200;             // "DShot 200 -> 450 -> 200"
constexpr std::uint16_t kLuisStep = 450;
constexpr std::uint32_t kLuisRepeats = 3;            // "3 repeats per motor, then the next motor"
constexpr std::uint64_t kLuisTotalMaxUs = 40000000;  // "40 s cap, all 4 motors (Recommended)"
constexpr std::uint16_t kLuisBenchCap = 450;         // "bench cap DShot 450 (console cap 500 unchanged)"
constexpr std::int32_t kLuisConsoleCap = 500;
// The arm switch and the failsafe (decision 0041), as the gate test states them.
constexpr std::size_t kArmChannel = 5;
constexpr TimeUs kLossUs = 100000;
constexpr TimeUs kHoldUs = 300000;
constexpr TimeUs kDisarmUs = 1500000;
constexpr TimeUs kRecoveryUs = 500000;
constexpr TimeUs kCalibrationUs = 1250000;  // Betaflight 4.5.1 gyro.c:107, the boot calibration window
constexpr float kSwitchOn = 1.0F;
constexpr float kSwitchOff = -1.0F;
// The bench switch's three positions on the RadioMaster Pocket (aux x = (us - 1500) / 512: 2012, 1500, 988 us).
constexpr float kBenchUp = 1.0F;
constexpr float kBenchMiddle = 0.0F;
constexpr float kBenchDown = -1.0F;

// A profile: the ruling's structure (lead-in, then for each motor in logical order `repeats` times base, step, base) on a clock.
struct Profile {
  std::uint64_t lead_in_us = 0;
  std::uint64_t level_us = 0;
  std::uint64_t total_max_us = 0;
  std::uint16_t base = 0;
  std::uint16_t step = 0;
  std::uint32_t repeats = 0;
};
constexpr Profile kLuis{kLuisLeadInUs, kLuisLevelUs, kLuisTotalMaxUs, kLuisBase, kLuisStep, kLuisRepeats};
// Scenario test value: the ruling's levels, repeats and motors on a compressed clock (2 ms lead-in, 1 ms levels, 40 ms cap),
// about 7.5 rate ticks of lead-in and 3.75 per level, so a sweep reaches every step in a few rate ticks.
constexpr Profile kFast{2000, 1000, 40000, kLuisBase, kLuisStep, kLuisRepeats};

// One segment of a profile: [start_us, end_us) after the arm edge, `motor` (1-based; 0 none) at `value`.
struct Segment {
  std::uint64_t start_us = 0;
  std::uint64_t end_us = 0;
  std::uint32_t motor = 0;
  std::uint16_t value = 0;
};

// The profile written out as the ruling words it: the lead-in, then motor 1 repeat 1 base, step, base, repeat 2 ..., motor 4.
std::vector<Segment> segments_of(const Profile& p) {
  std::vector<Segment> s;
  std::uint64_t t = 0;
  s.push_back({t, t + p.lead_in_us, 0, 0});
  t += p.lead_in_us;
  for (std::uint32_t motor = 1; motor <= kMotors; ++motor) {
    for (std::uint32_t r = 0; r < p.repeats; ++r) {
      for (const std::uint16_t v : {p.base, p.step, p.base}) {
        s.push_back({t, t + p.level_us, motor, v});
        t += p.level_us;
      }
    }
  }
  return s;
}

// The index of the segment that holds the nominal time num / den us, or the segment count after the last.
std::size_t segment_at(const std::vector<Segment>& s, std::uint64_t num, std::uint64_t den) {
  for (std::size_t i = 0; i < s.size(); ++i) {
    if (num >= s[i].start_us * den && num < s[i].end_us * den) {
      return i;
    }
  }
  return s.size();
}

using Values = std::array<std::uint16_t, kMotors>;

Values values_of(const std::vector<Segment>& s, std::size_t i) {
  Values v{};
  if (i < s.size() && s[i].motor != 0) {
    v[s[i].motor - 1] = s[i].value;
  }
  return v;
}

// ---- The table ----

std::array<ParamRecord, kParamCount> g_table;

using Overrides = std::vector<std::pair<ParamId, double>>;

// The default table with each override (the record keeps its type; a whole number for an i32 parameter).
std::span<const ParamRecord, kParamCount> table_with(const Overrides& o) {
  const std::span<const ParamRecord, kParamCount> d = param_defaults();
  std::copy(d.begin(), d.end(), g_table.begin());
  for (const auto& [id, value] : o) {
    ParamRecord& r = g_table[static_cast<std::size_t>(id)];
    r.value = r.value.type == ParamType::F32 ? ParamValue{ParamType::F32, static_cast<float>(value), 0}
                                             : ParamValue{ParamType::I32, 0.0F, static_cast<std::int32_t>(value)};
  }
  return std::span<const ParamRecord, kParamCount>(g_table);
}

Overrides overrides_of(const Profile& p) {
  return {{ParamId::bench_profile_lead_in_us, static_cast<double>(p.lead_in_us)},
          {ParamId::bench_profile_level_us, static_cast<double>(p.level_us)},
          {ParamId::bench_profile_total_max_us, static_cast<double>(p.total_max_us)}};
}

std::int32_t i32_default(ParamId id) { return param_defaults()[static_cast<std::size_t>(id)].value.i32; }

// ---- The forked child ----

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

// ---- The rig: the composition through hal_sim, one sample per call ----

struct Frame {
  float arm = kSwitchOff;
  float bench = kBenchMiddle;
  float throttle = 0.0F;
  std::optional<TimeUs> t_us = std::nullopt;  // the frame's stamp; this tick's time if absent
};

struct Out {
  Tick n = 0;
  TimeUs stamp = 0;
  std::array<DshotValue, kMotors> motor{};
  ArmState state = ArmState::Boot;
  BenchProfileStatus status{};

  [[nodiscard]] Values values() const {
    Values v{};
    for (std::size_t i = 0; i < kMotors; ++i) {
      v[i] = motor[i].raw();
    }
    return v;
  }
  [[nodiscard]] bool all_stop() const { return values() == Values{}; }
};

class Rig {
 public:
  explicit Rig(std::span<const ParamRecord, kParamCount> table) {
    if (!params_init(table)) {
      ADD_FAILURE() << "params_init rejected the table";
      std::exit(2);
    }
    period_ = {static_cast<std::uint32_t>(param_value<ParamId::tick_period_num_us>()),
               static_cast<std::uint32_t>(param_value<ParamId::tick_period_den>())};
    divisor_ = static_cast<std::size_t>(param_value<ParamId::rate_loop_divisor>());
    hal_sim::setup(period_, motor_latch_, servo_latch_);
    composition::init();
  }

  // The nominal rate period is rate_num() / den() us.
  [[nodiscard]] std::uint64_t rate_num() const { return static_cast<std::uint64_t>(divisor_) * period_.num_us; }
  [[nodiscard]] std::uint64_t den() const { return period_.den; }
  [[nodiscard]] TimeUs now() const { return hal_sim::stamp_us(period_, n_); }

  // One sample tick n_: `deliver` false skips the composition (a lost sample: time advances, no tick). `stamp` replaces the
  // sample's stamp. Returns whether it wrote and what.
  std::pair<bool, Out> sample(bool deliver = true, std::optional<TimeUs> stamp = std::nullopt) {
    for (DshotValue& m : motor_latch_) {
      m = sentinel();
    }
    hal_sim::begin_tick(n_);
    Out o;
    o.n = n_;
    if (deliver) {
      ImuSample s{};
      s.t_us = stamp.value_or(hal_time_us());
      s.temp_k = kTempK;
      s.accel_m_s2 = prim::Vec3<float>(0.0F, 0.0F, kQuietAccelZ);
      s.flags = imu_flag(ImuFlag::GyroValid) | imu_flag(ImuFlag::AccelValid) | imu_flag(ImuFlag::TempValid);
      if (saturate_next_) {
        s.flags |= imu_flag(ImuFlag::GyroSatX);
        saturate_next_ = false;
      }
      o.stamp = s.t_us;
      composition::tick(s);
      ++delivered_;
    }
    hal_sim::end_tick();
    ++n_;
    o.motor = motor_latch_;
    o.state = composition::arming_status();
    o.status = composition::bench_profile_status();
    return {motor_latch_[0] != sentinel(), o};
  }

  // Runs the samples up to the next rate tick (they must write nothing), stages `frame` if given (its stamp: this tick's time
  // unless set), and runs the rate tick (with `stamp` as its sample's stamp if given). Checks the invariants on its write.
  Out rate(const std::optional<Frame>& frame, std::optional<TimeUs> stamp = std::nullopt) {
    while (delivered_ % divisor_ != 0) {
      const auto [wrote, o] = sample();
      EXPECT_FALSE(wrote) << "sample " << o.n << " is not a rate tick";
    }
    if (frame) {
      ManualControl m;
      m.throttle = frame->throttle;
      m.channels[kArmChannel - 1] = frame->arm;
      m.channels[static_cast<std::size_t>(kLuisBenchChannel) - 1] = frame->bench;
      m.t_us = frame->t_us.value_or(now());
      m.frames = ++frames_;
      hal_sim::stage_manual_control(m);
    }
    const auto [wrote, o] = sample(true, stamp);
    EXPECT_TRUE(wrote) << "rate tick " << o.n << " wrote nothing";
    check_invariants(o);
    return o;
  }

  // Rate ticks without a frame (no link) through the boot gyro calibration.
  void warm_up() {
    const TimeUs first = now();
    while (now() - first <= kCalibrationUs) {
      EXPECT_TRUE(rate(std::nullopt).all_stop());
    }
  }

  // A valid link with the arm switch off for longer than the recovery time.
  void link_up(float bench) {
    const TimeUs until = now() + kRecoveryUs + kRecoveryUs / 2;
    while (now() <= until) {
      EXPECT_TRUE(rate(Frame{kSwitchOff, bench}).all_stop());
    }
    EXPECT_EQ(composition::arming_status(), ArmState::SwitchOff);
  }

  // The arm switch on: the arm edge (asserted Armed). `frame_stamp` is the frame's stamp if given.
  Out arm(float bench, std::optional<TimeUs> frame_stamp = std::nullopt) {
    const Out o = rate(Frame{kSwitchOn, bench, 0.0F, frame_stamp});
    EXPECT_EQ(o.state, ArmState::Armed) << "the arm edge did not arm";
    return o;
  }

  void saturate_next() { saturate_next_ = true; }
  [[nodiscard]] std::size_t divisor() const { return divisor_; }
  [[nodiscard]] Tick n() const { return n_; }
  [[nodiscard]] TimeUs stamp_of(Tick n) const { return hal_sim::stamp_us(period_, n); }

  bool console_test = false;  // a console motor test may write (its value, not the profile's)

 private:
  static constexpr std::uint16_t kSentinelRaw = 1000;
  static constexpr float kTempK = 300.0F;
  static constexpr float kQuietAccelZ = -9.8F;

  static DshotValue sentinel() { return DshotValue::from_raw(kSentinelRaw).value_or(DshotValue::stop()); }

  // On every write: never two motors nonzero, never above the bench cap; not Armed (and no console test) is stop everywhere.
  void check_invariants(const Out& o) const {
    std::size_t nonzero = 0;
    for (const DshotValue& m : o.motor) {
      nonzero += m != DshotValue::stop() ? 1U : 0U;
      EXPECT_LE(m.raw(), kLuisBenchCap) << "rate tick " << o.n;
    }
    EXPECT_LE(nonzero, 1U) << "rate tick " << o.n;
    if (o.state != ArmState::Armed && !console_test) {
      EXPECT_TRUE(o.all_stop()) << "rate tick " << o.n << " state " << static_cast<int>(o.state);
    }
  }

  hal_sim::TickPeriod period_{};
  std::size_t divisor_ = 1;
  std::array<DshotValue, kMotors> motor_latch_{};
  std::array<ServoUs, composition::kServos> servo_latch_{};
  Tick n_ = 0;
  std::uint64_t delivered_ = 0;
  std::uint32_t frames_ = 0;
  bool saturate_next_ = false;
};

// The expected values of the k-th rate tick after the arm edge (the edge's own tick k = 0) of a live profile.
Values expected(const Rig& rig, const std::vector<Segment>& s, std::uint64_t k) {
  return values_of(s, segment_at(s, k * rig.rate_num(), rig.den()));
}

// ---- The parameters ----

TEST(BenchProfileTest, TheBenchParametersAreLuisRulingsAndTheConsoleCapIsUnchanged) {
  EXPECT_EQ(i32_default(ParamId::bench_profile_rc_channel), kLuisBenchChannel);
  EXPECT_EQ(param_defaults()[static_cast<std::size_t>(ParamId::bench_profile_switch_threshold)].value.f32, kLuisBenchThreshold);
  EXPECT_EQ(i32_default(ParamId::bench_profile_lead_in_us), static_cast<std::int32_t>(kLuisLeadInUs));
  EXPECT_EQ(i32_default(ParamId::bench_profile_level_us), static_cast<std::int32_t>(kLuisLevelUs));
  EXPECT_EQ(i32_default(ParamId::bench_profile_dshot_base), kLuisBase);
  EXPECT_EQ(i32_default(ParamId::bench_profile_dshot_step), kLuisStep);
  EXPECT_EQ(i32_default(ParamId::bench_profile_repeats), static_cast<std::int32_t>(kLuisRepeats));
  EXPECT_EQ(i32_default(ParamId::bench_profile_total_max_us), static_cast<std::int32_t>(kLuisTotalMaxUs));
  EXPECT_EQ(i32_default(ParamId::bench_profile_dshot_max), kLuisBenchCap);
  EXPECT_EQ(i32_default(ParamId::motor_test_dshot_max), kLuisConsoleCap);
  EXPECT_EQ(i32_default(ParamId::rc_channel_arm), static_cast<std::int32_t>(kArmChannel));
  EXPECT_EQ(i32_default(ParamId::failsafe_loss_us), static_cast<std::int32_t>(kLossUs));
  EXPECT_EQ(i32_default(ParamId::failsafe_disarm_us), static_cast<std::int32_t>(kDisarmUs));
  // The ruled profile is 2 s + 4 motors x 3 repeats x 3 levels x 1 s = 38 s, inside the 40 s cap.
  const std::vector<Segment> s = segments_of(kLuis);
  EXPECT_EQ(s.size(), 1 + kMotors * kLuisRepeats * 3);
  EXPECT_EQ(s.back().end_us, 38000000U);
  EXPECT_LE(s.back().end_us, kLuisTotalMaxUs);
}

// ---- The sequence, to the tick ----

// Armed with the switch up and everything held: every rate tick from the edge to past the cap writes the segment of its nominal
// time (stop in the lead-in, then motor 1 .. 4, each 200, 450, 200 three times, one motor at a time), never above 450, and stop
// from the end on (it never repeats while the arm lasts). Each segment's first rate tick is the first whose stamp is at or after
// the segment's start (the stamp clock agrees with the tick clock), and each level lasts 3750 or 3751 rate ticks (1 s of
// 266.66 us periods). The status follows: LeadIn, Running with the level, motor and value, then Done.
TEST(BenchProfileTest, TheProfileMatchesTheRulingToTheTickAndStopsForGoodAtTheEnd) {
  in_child([] {
    Rig rig(param_defaults());
    rig.warm_up();
    rig.link_up(kBenchUp);
    const std::vector<Segment> s = segments_of(kLuis);
    const Out edge = rig.arm(kBenchUp);
    EXPECT_TRUE(edge.all_stop());
    EXPECT_EQ(edge.status.phase, BenchProfilePhase::LeadIn);
    EXPECT_EQ(edge.status.arms, 1U);
    std::vector<std::uint64_t> ticks_in(s.size() + 1, 0);
    std::size_t prev = 0;
    ticks_in[0] = 1;
    for (std::uint64_t k = 1;; ++k) {
      const Out o = rig.rate(Frame{kSwitchOn, kBenchUp});
      const std::size_t seg = segment_at(s, k * rig.rate_num(), rig.den());
      ASSERT_EQ(o.state, ArmState::Armed) << "k " << k;
      ASSERT_EQ(o.values(), values_of(s, seg)) << "k " << k << " segment " << seg;
      if (seg != prev) {
        ASSERT_EQ(seg, prev + 1) << "k " << k;  // the segments in order, none skipped
        if (seg < s.size()) {
          // The stamp clock: this is the first rate tick whose stamp is at or after the segment's start, the one before is not.
          EXPECT_GE(o.stamp - edge.stamp, s[seg].start_us) << "segment " << seg;
          EXPECT_LT(rig.stamp_of(o.n - rig.divisor()) - edge.stamp, s[seg].start_us) << "segment " << seg;
        }
        prev = seg;
      }
      ++ticks_in[seg];
      if (seg == 0) {
        EXPECT_EQ(o.status.phase, BenchProfilePhase::LeadIn) << "k " << k;
      } else if (seg < s.size()) {
        EXPECT_EQ(o.status.phase, BenchProfilePhase::Running) << "k " << k;
        EXPECT_EQ(o.status.level, seg - 1) << "k " << k;
        EXPECT_EQ(o.status.motor, s[seg].motor) << "k " << k;
        EXPECT_EQ(o.status.dshot, s[seg].value) << "k " << k;
      } else {
        EXPECT_EQ(o.status.phase, BenchProfilePhase::Done) << "k " << k;
        EXPECT_EQ(o.status.motor, 0U);
        EXPECT_EQ(o.status.dshot, 0U);
      }
      if (::testing::Test::HasFailure()) {
        return;
      }
      // Past the 40 s cap by 5 s: still armed, still stop, the profile has not repeated.
      if (o.stamp - edge.stamp > kLuisTotalMaxUs + 5 * kLuisLevelUs) {
        break;
      }
    }
    EXPECT_EQ(prev, s.size());
    for (std::size_t i = 1; i < s.size(); ++i) {
      EXPECT_TRUE(ticks_in[i] == 3750 || ticks_in[i] == 3751) << "segment " << i << ": " << ticks_in[i];
    }
    EXPECT_TRUE(ticks_in[0] == 7500 || ticks_in[0] == 7501) << ticks_in[0];
    EXPECT_EQ(composition::bench_profile_status().arms, 1U);
  });
}

// ---- The arm edge needs the switch in position ----

// Runbook step 1: armed with the bench switch in the middle, down, just below the threshold or not a number, nothing spins on
// any rate tick, and raising the switch fully up after the edge does not start it (SwitchOut until the next arm). A re-arm with
// the switch up runs it (the lead-in, then motor 1 at 200 on the tick its first level starts). Exactly at the threshold is in
// position.
TEST(BenchProfileTest, ArmedWithoutTheSwitchFullyUpNothingSpinsAndRaisingItAfterTheEdgeDoesNotStartIt) {
  in_child([] {
    Rig rig(param_defaults());
    rig.warm_up();
    const std::vector<Segment> s = segments_of(kLuis);
    const float below = std::nextafter(kLuisBenchThreshold, -std::numeric_limits<float>::infinity());
    std::uint32_t arms = 0;
    for (const float bench : {kBenchMiddle, kBenchDown, below, std::numeric_limits<float>::quiet_NaN()}) {
      rig.link_up(bench);
      const Out edge = rig.arm(bench);
      ++arms;
      EXPECT_TRUE(edge.all_stop()) << bench;
      EXPECT_EQ(edge.status.phase, BenchProfilePhase::SwitchOut) << bench;
      EXPECT_EQ(edge.status.arms, arms);
      // Up after the edge, through the lead-in and the first level's start: stop.
      for (std::uint64_t k = 1; k * rig.rate_num() < (kLuisLeadInUs + kLuisLevelUs / 2) * rig.den(); ++k) {
        const Out o = rig.rate(Frame{kSwitchOn, k < 3 ? bench : kBenchUp});
        ASSERT_EQ(o.state, ArmState::Armed);
        ASSERT_TRUE(o.all_stop()) << "bench " << bench << " k " << k;
        ASSERT_EQ(o.status.phase, BenchProfilePhase::SwitchOut);
      }
      EXPECT_TRUE(rig.rate(Frame{kSwitchOff, kBenchUp}).all_stop());
    }
    // Exactly at the threshold, then up: the profile runs.
    for (const float bench : {kLuisBenchThreshold, kBenchUp}) {
      rig.link_up(bench);
      const Out edge = rig.arm(bench);
      EXPECT_EQ(edge.status.phase, BenchProfilePhase::LeadIn) << bench;
      for (std::uint64_t k = 1; k * rig.rate_num() < (kLuisLeadInUs + kLuisLevelUs / 2) * rig.den(); ++k) {
        const Out o = rig.rate(Frame{kSwitchOn, bench});
        ASSERT_EQ(o.values(), expected(rig, s, k)) << "bench " << bench << " k " << k;
      }
      const Out o = rig.rate(Frame{kSwitchOn, bench});
      EXPECT_EQ(o.motor[0].raw(), kLuisBase);
      EXPECT_TRUE(rig.rate(Frame{kSwitchOff, bench}).all_stop());
    }
  });
}

// ---- Every kill at every step ----

enum class Kill : std::uint8_t { SwitchMiddle, SwitchDown, SwitchJustBelow, SwitchNaN, ArmSwitchOff, LinkLoss };

const char* name(Kill k) {
  switch (k) {
    case Kill::SwitchMiddle:
      return "bench switch middle";
    case Kill::SwitchDown:
      return "bench switch down";
    case Kill::SwitchJustBelow:
      return "bench switch just below the threshold";
    case Kill::SwitchNaN:
      return "bench switch not a number";
    case Kill::ArmSwitchOff:
      return "arm switch off";
    case Kill::LinkLoss:
      return "link age at failsafe_loss_us";
  }
  return "";
}

float bench_value(Kill k) {
  switch (k) {
    case Kill::SwitchMiddle:
      return kBenchMiddle;
    case Kill::SwitchDown:
      return kBenchDown;
    case Kill::SwitchJustBelow:
      return std::nextafter(kLuisBenchThreshold, -std::numeric_limits<float>::infinity());
    case Kill::SwitchNaN:
      return std::numeric_limits<float>::quiet_NaN();
    default:
      return kBenchUp;
  }
}

// For every step of the profile (the lead-in and each of the 36 levels) and its first and last rate tick: arms with the switch
// up, checks every rate tick up to the kill tick against the segments, applies `kill` on that tick (the bench switch out of
// position in that tick's frame; the arm switch off in it; or no frame since one whose stamp makes that tick's link age exactly
// failsafe_loss_us), and requires stop on that tick, with the kill's phase. Then everything the kill took comes back (the switch
// up, the frames again, the arm switch on is a new arm so it is not restored) for past the profile's whole length: stop on
// every rate tick (the latch). Then a disarm and a re-arm: the profile runs again from its lead-in. On the kFast clock.
void sweep(Kill kill) {
  Rig rig(table_with(overrides_of(kFast)));
  rig.warm_up();
  rig.link_up(kBenchUp);
  const std::vector<Segment> s = segments_of(kFast);
  const std::uint64_t num = rig.rate_num();
  const std::uint64_t den = rig.den();
  // The k of each step's first and last rate tick.
  std::vector<std::uint64_t> kill_ks;
  {
    std::size_t prev = 0;
    std::uint64_t k = 0;
    for (; segment_at(s, k * num, den) < s.size(); ++k) {
      const std::size_t seg = segment_at(s, k * num, den);
      if (seg != prev) {
        kill_ks.push_back(k - 1);  // the last tick of the step before
        kill_ks.push_back(k);      // the first tick of this one
        prev = seg;
      }
    }
    kill_ks.push_back(k - 1);
    kill_ks.insert(kill_ks.begin(), 0);
  }
  const bool at_edge_possible = kill != Kill::ArmSwitchOff && kill != Kill::LinkLoss;  // those two at the edge are no arm
  std::uint32_t arms = 0;
  for (const std::uint64_t k_kill : kill_ks) {
    if (k_kill == 0 && !at_edge_possible) {
      continue;
    }
    // The edge. For a link loss, its frame (the last one) is stamped so that the kill tick's age is exactly the loss time.
    std::optional<TimeUs> edge_frame_stamp;
    if (kill == Kill::LinkLoss) {
      const Tick n_kill = rig.n() + ((rig.divisor() - rig.n() % rig.divisor()) % rig.divisor()) + k_kill * rig.divisor();
      edge_frame_stamp = rig.stamp_of(n_kill) - kLossUs;
    }
    const float edge_bench = k_kill == 0 ? bench_value(kill) : kBenchUp;
    const Out edge = rig.arm(edge_bench, edge_frame_stamp);
    ++arms;
    Out o = edge;
    for (std::uint64_t k = 1; k <= k_kill; ++k) {
      if (kill == Kill::LinkLoss) {
        o = rig.rate(std::nullopt);
      } else if (k == k_kill) {
        o = rig.rate(Frame{kill == Kill::ArmSwitchOff ? kSwitchOff : kSwitchOn, bench_value(kill)});
      } else {
        o = rig.rate(Frame{kSwitchOn, kBenchUp});
      }
      if (k < k_kill) {
        ASSERT_EQ(o.values(), expected(rig, s, k)) << name(kill) << " kill at k " << k_kill << ", k " << k;
        ASSERT_EQ(o.state, ArmState::Armed);
      }
    }
    // The kill tick.
    ASSERT_TRUE(o.all_stop()) << name(kill) << " kill at k " << k_kill;
    const BenchProfilePhase want = kill == Kill::ArmSwitchOff || kill == Kill::LinkLoss ? BenchProfilePhase::NotArmed
                                                                                        : BenchProfilePhase::SwitchOut;
    ASSERT_EQ(o.status.phase, want) << name(kill) << " kill at k " << k_kill;
    ASSERT_EQ(o.status.arms, arms);
    if (kill == Kill::LinkLoss) {
      ASSERT_EQ(o.state, ArmState::ArmedHold) << "kill at k " << k_kill;
      ASSERT_EQ(o.status.kill_state, ArmState::ArmedHold);
      // One rate tick earlier the age was below the loss time and the profile wrote (unless that was the lead-in).
    } else if (kill == Kill::ArmSwitchOff) {
      ASSERT_EQ(o.state, ArmState::SwitchOff);
      ASSERT_EQ(o.status.kill_state, ArmState::SwitchOff);
    } else {
      ASSERT_EQ(o.state, ArmState::Armed);
    }
    // What the kill took comes back, for past the profile's length: stop on every rate tick.
    const TimeUs back_until = rig.now() + kFast.total_max_us + kFast.total_max_us / 2;
    while (rig.now() < back_until) {
      const Out b = rig.rate(Frame{kill == Kill::ArmSwitchOff ? kSwitchOff : kSwitchOn, kBenchUp});
      ASSERT_TRUE(b.all_stop()) << name(kill) << " kill at k " << k_kill << ": the profile came back at sample " << b.n;
      ASSERT_EQ(b.status.phase, want);
      if (kill != Kill::ArmSwitchOff) {
        ASSERT_EQ(b.state, ArmState::Armed) << "kill at k " << k_kill;  // armed again, the latch holds
      }
    }
    // A disarm, the link valid for the recovery time, and a re-arm: the profile runs again from its lead-in.
    rig.link_up(kBenchUp);
    const Out again = rig.arm(kBenchUp);
    ++arms;
    ASSERT_EQ(again.status.phase, BenchProfilePhase::LeadIn);
    ASSERT_EQ(again.status.arms, arms);
    for (std::uint64_t k = 1; k < 12; ++k) {
      ASSERT_EQ(rig.rate(Frame{kSwitchOn, kBenchUp}).values(), expected(rig, s, k)) << "re-arm after kill at " << k_kill;
    }
    rig.link_up(kBenchUp);
  }
}

TEST(BenchProfileKillTest, TheBenchSwitchInTheMiddleAtEveryStepStopsOnThatTickUntilTheNextArm) {
  in_child([] { sweep(Kill::SwitchMiddle); });
}
TEST(BenchProfileKillTest, TheBenchSwitchDownAtEveryStepStopsOnThatTickUntilTheNextArm) {
  in_child([] { sweep(Kill::SwitchDown); });
}
TEST(BenchProfileKillTest, TheBenchSwitchJustBelowTheThresholdAtEveryStepStopsOnThatTickUntilTheNextArm) {
  in_child([] { sweep(Kill::SwitchJustBelow); });
}
TEST(BenchProfileKillTest, TheBenchSwitchNotANumberAtEveryStepStopsOnThatTickUntilTheNextArm) {
  in_child([] { sweep(Kill::SwitchNaN); });
}
TEST(BenchProfileKillTest, TheArmSwitchOffAtEveryStepStopsOnThatTickUntilTheNextArm) {
  in_child([] { sweep(Kill::ArmSwitchOff); });
}
TEST(BenchProfileKillTest, TheLinkAgeReachingExactlyTheLossTimeAtEveryStepStopsOnThatTickUntilTheNextArm) {
  in_child([] { sweep(Kill::LinkLoss); });
}

// ---- The props-off kills on the ruling's own clock (runbook steps 3 to 5) ----

// Each kill in the middle of motor 2's first 450 level (12.5 s after the edge): the bench switch down, the arm switch off, or
// the transmitter off (no frame after the last one). The rate tick before the kill writes motor 2 at 450, the kill tick writes
// stop. The transmitter off: stop from the first rate tick whose link age is at least 100 ms (ArmedHold), centred from 300 ms,
// disarm from 1.5 s (Failsafe), stop on every tick; with the link back at 1 s the state is Armed again and the profile stays
// stopped (2 s more are checked).
TEST(BenchProfileKillTest, OnTheRulingsClockEachKillStopsOnItsTickAndTheTransmitterOffHoldsThenDisarmsAtOnePointFiveSeconds) {
  in_child([] {
    Rig rig(param_defaults());
    rig.warm_up();
    const std::vector<Segment> s = segments_of(kLuis);
    const std::size_t seg = 1 + kLuisRepeats * 3 + 1;  // motor 2, repeat 1, level 2
    ASSERT_EQ(s[seg].motor, 2U);
    ASSERT_EQ(s[seg].value, kLuisStep);
    const std::uint64_t mid_us = (s[seg].start_us + s[seg].end_us) / 2;
    for (const int kind : {0, 1, 2, 3}) {  // 0 bench down, 1 arm off, 2 transmitter off to the disarm, 3 off and back at 1 s
      rig.link_up(kBenchUp);
      (void)rig.arm(kBenchUp);
      Out o;
      for (std::uint64_t k = 1; k * rig.rate_num() < mid_us * rig.den(); ++k) {
        o = rig.rate(Frame{kSwitchOn, kBenchUp});
        ASSERT_EQ(o.values(), expected(rig, s, k)) << "k " << k;
      }
      ASSERT_EQ(o.values(), (Values{0, kLuisStep, 0, 0}));
      if (kind == 0) {
        o = rig.rate(Frame{kSwitchOn, kBenchDown});
        EXPECT_TRUE(o.all_stop());
        EXPECT_EQ(o.state, ArmState::Armed);
        EXPECT_EQ(o.status.phase, BenchProfilePhase::SwitchOut);
        EXPECT_TRUE(rig.rate(Frame{kSwitchOn, kBenchUp}).all_stop());
        EXPECT_TRUE(rig.rate(Frame{kSwitchOff, kBenchUp}).all_stop());
        continue;
      }
      if (kind == 1) {
        o = rig.rate(Frame{kSwitchOff, kBenchUp});
        EXPECT_TRUE(o.all_stop());
        EXPECT_EQ(o.state, ArmState::SwitchOff);
        EXPECT_EQ(o.status.phase, BenchProfilePhase::NotArmed);
        continue;
      }
      // The transmitter off: the last frame was the previous rate tick's, stamped with its time.
      const TimeUs last_frame = o.stamp;
      bool held = false;
      bool centred = false;
      for (;;) {
        o = rig.rate(std::nullopt);
        const TimeUs age = o.stamp - last_frame;
        if (age < kLossUs) {
          ASSERT_EQ(o.state, ArmState::Armed) << "age " << age;
          ASSERT_EQ(o.values(), (Values{0, kLuisStep, 0, 0})) << "age " << age;
        } else {
          ASSERT_TRUE(o.all_stop()) << "age " << age;
          ASSERT_EQ(o.status.phase, BenchProfilePhase::NotArmed);
          ASSERT_EQ(o.state, age < kHoldUs     ? ArmState::ArmedHold
                             : age < kDisarmUs ? ArmState::ArmedFailsafeCentred
                                               : ArmState::Failsafe)
              << "age " << age;
          held = held || o.state == ArmState::ArmedHold;
          centred = centred || o.state == ArmState::ArmedFailsafeCentred;
          if (o.state == ArmState::Failsafe) {
            break;
          }
        }
        if (kind == 3 && age >= kLuisLevelUs) {
          break;
        }
      }
      EXPECT_TRUE(held);
      EXPECT_TRUE(centred);
      EXPECT_EQ(o.status.kill_state, ArmState::ArmedHold);
      if (kind == 3) {
        // The link back before the disarm: Armed again, and still stop on every tick.
        const TimeUs until = o.stamp + 2 * kLuisLevelUs;
        while (o.stamp < until) {
          o = rig.rate(Frame{kSwitchOn, kBenchUp});
          ASSERT_EQ(o.state, ArmState::Armed);
          ASSERT_TRUE(o.all_stop()) << "the profile came back after the link did";
        }
        EXPECT_TRUE(rig.rate(Frame{kSwitchOff, kBenchUp}).all_stop());
      } else {
        EXPECT_EQ(o.state, ArmState::Failsafe);
        for (int i = 0; i < 4; ++i) {
          EXPECT_TRUE(rig.rate(Frame{kSwitchOn, kBenchUp}).all_stop());  // the frames back, the switch still on: no arm
        }
        EXPECT_TRUE(rig.rate(Frame{kSwitchOff, kBenchUp}).all_stop());
      }
    }
  });
}

// ---- Any arming state but Armed writes stop ----

// With the bench switch up throughout, every rate tick before the arm writes stop through the states the bench passes (Boot
// before the first rate tick, then NoLink, Calibrating, SwitchOff, SwitchNotCycled with the arm switch on from boot, ThrottleUp, Saturated), and so do the disarm
// causes (SwitchOff, Failsafe, ImuStale on a read-deadline miss), each seen. Every state seen that is not Armed wrote stop on
// every rate tick (Rig::check_invariants), and the states Armed, ArmedHold and ArmedFailsafeCentred are covered by the kill tests.
TEST(BenchProfileStateTest, EveryRefusalAndDisarmStateWritesStopWithTheSwitchUp) {
  in_child([] {
    Rig rig(param_defaults());
    EXPECT_EQ(composition::arming_status(), ArmState::Boot);
    std::vector<ArmState> seen;
    const auto note = [&](const Out& o) {
      EXPECT_TRUE(o.all_stop()) << static_cast<int>(o.state);
      // Armed is the one state that may write; the ticks just after the arm edge are in the lead-in (stop), the kill tests' ground.
      if (o.state != ArmState::Armed && std::find(seen.begin(), seen.end(), o.state) == seen.end()) {
        seen.push_back(o.state);
      }
    };
    // The arm switch on from boot, the throttle up: never armed through the calibration and the link's recovery.
    const TimeUs first = rig.now();
    while (rig.now() - first <= kCalibrationUs / 2) {
      note(rig.rate(std::nullopt));
    }
    while (rig.now() - first <= kCalibrationUs + kRecoveryUs + kRecoveryUs / 2) {
      note(rig.rate(Frame{kSwitchOn, kBenchUp, 0.0F}));
    }
    for (int i = 0; i < 8; ++i) {
      note(rig.rate(Frame{kSwitchOff, kBenchUp, 1.0F}));  // ThrottleUp
    }
    rig.saturate_next();
    note(rig.rate(Frame{kSwitchOn, kBenchUp, 0.0F}));  // a saturated sample since the last rate tick refuses this edge
    note(rig.rate(Frame{kSwitchOff, kBenchUp}));
    // Armed, then a failsafe disarm.
    Out o = rig.arm(kBenchUp);
    const TimeUs t0 = rig.stamp_of(o.n);
    while (o.state != ArmState::Failsafe && rig.now() - t0 <= 2 * kDisarmUs) {
      o = rig.rate(std::nullopt);  // the link age counts from the last frame, which came before the arm tick
      note(o);
    }
    EXPECT_EQ(o.state, ArmState::Failsafe);
    rig.link_up(kBenchUp);
    o = rig.arm(kBenchUp);
    // A read-deadline miss: ImuStale on the next rate tick, and arming stays refused.
    hal_sim::stage_imu_deadline_misses(1);
    note(rig.rate(Frame{kSwitchOn, kBenchUp}));
    for (int i = 0; i < 8; ++i) {
      note(rig.rate(Frame{i % 2 == 0 ? kSwitchOff : kSwitchOn, kBenchUp}));
    }
    for (const ArmState want : {ArmState::NoLink, ArmState::Calibrating, ArmState::SwitchNotCycled,
                                ArmState::ThrottleUp, ArmState::Saturated, ArmState::SwitchOff, ArmState::Failsafe,
                                ArmState::ImuStale}) {
      EXPECT_NE(std::find(seen.begin(), seen.end(), want), seen.end()) << "state " << static_cast<int>(want) << " not seen";
    }
  });
}

// Maintenance and a refused config refuse arming with the switch up: stop on every tick.
TEST(BenchProfileStateTest, MaintenanceAndARefusedConfigWriteStopWithTheSwitchUp) {
  in_child([] {
    Rig rig(param_defaults());
    rig.warm_up();
    composition::request_maintenance();
    const TimeUs until = rig.now() + kRecoveryUs + kRecoveryUs / 2;
    while (rig.now() <= until) {
      EXPECT_TRUE(rig.rate(Frame{kSwitchOff, kBenchUp}).all_stop());
    }
    for (int i = 0; i < 8; ++i) {
      const Out o = rig.rate(Frame{i % 2 == 0 ? kSwitchOn : kSwitchOff, kBenchUp});
      EXPECT_EQ(o.state, ArmState::Maintenance);
      EXPECT_TRUE(o.all_stop());
    }
  });
  in_child([] {
    composition::set_config_status(arming::ConfigStatus::Invalid);
    Rig rig(param_defaults());
    rig.warm_up();
    const TimeUs until = rig.now() + kRecoveryUs + kRecoveryUs / 2;
    while (rig.now() <= until) {
      EXPECT_TRUE(rig.rate(Frame{kSwitchOff, kBenchUp}).all_stop());
    }
    for (int i = 0; i < 8; ++i) {
      const Out o = rig.rate(Frame{i % 2 == 0 ? kSwitchOn : kSwitchOff, kBenchUp});
      EXPECT_EQ(o.state, ArmState::ConfigRefused);
      EXPECT_TRUE(o.all_stop());
    }
  });
}

// The console motor test (decision 0038) runs only from Disarmed, as in the flight composition: during it exactly the requested
// motor gets the requested value (the profile adds nothing), arming is refused (MotorTest), and after it every motor stops.
TEST(BenchProfileStateTest, TheConsoleMotorTestIsUnchangedAndAddsNoProfile) {
  in_child([] {
    Rig rig(param_defaults());
    rig.warm_up();
    rig.link_up(kBenchUp);
    rig.console_test = true;
    constexpr std::uint16_t kConsoleDshot = 100;
    constexpr std::uint32_t kConsoleMs = 50;
    hal_sim::stage_motor_test_request({2, kConsoleDshot, kConsoleMs});
    Out o = rig.rate(Frame{kSwitchOff, kBenchUp});
    EXPECT_EQ(o.values(), (Values{0, kConsoleDshot, 0, 0}));
    o = rig.rate(Frame{kSwitchOn, kBenchUp});
    EXPECT_EQ(o.state, ArmState::MotorTest);
    EXPECT_EQ(o.values(), (Values{0, kConsoleDshot, 0, 0}));
    EXPECT_EQ(o.status.phase, BenchProfilePhase::Idle);
    const TimeUs until = rig.now() + kConsoleMs * 1000;
    while (rig.now() <= until) {
      o = rig.rate(Frame{kSwitchOn, kBenchUp});
    }
    EXPECT_TRUE(o.all_stop());
    EXPECT_EQ(o.status.phase, BenchProfilePhase::Idle);
    EXPECT_EQ(o.status.arms, 0U);
  });
}

// ---- Once per arm ----

// Three arms in a row on the kFast clock, each held armed with the switch up for three times the profile's length: each runs
// the whole profile once (the same values on the same ticks) and then stop to the end of the arm; the switch cycled down and
// up while armed after the end starts nothing.
TEST(BenchProfileOnceTest, EachArmRunsTheProfileExactlyOnce) {
  in_child([] {
    Rig rig(table_with(overrides_of(kFast)));
    rig.warm_up();
    const std::vector<Segment> s = segments_of(kFast);
    for (std::uint32_t arm = 1; arm <= 3; ++arm) {
      rig.link_up(kBenchUp);
      const Out edge = rig.arm(kBenchUp);
      EXPECT_EQ(edge.status.arms, arm);
      std::size_t written_levels = 0;
      std::size_t prev = 0;
      for (std::uint64_t k = 1; rig.now() - edge.stamp < 3 * kFast.total_max_us; ++k) {
        const bool cycle = rig.now() - edge.stamp > kFast.total_max_us && k % 4 == 0;
        const Out o = rig.rate(Frame{kSwitchOn, cycle ? kBenchDown : kBenchUp});
        const std::size_t seg = segment_at(s, k * rig.rate_num(), rig.den());
        ASSERT_EQ(o.values(), values_of(s, seg)) << "arm " << arm << " k " << k;
        if (seg != prev && seg < s.size()) {
          ++written_levels;
        }
        prev = seg;
      }
      EXPECT_EQ(written_levels, s.size() - 1) << "arm " << arm;
    }
  });
}

// ---- The stamp end ----

// The tick clock lags the stamps when samples are lost (every other one here, so the rate ticks come at half the rate): the
// profile ends at the first rate tick whose stamp is the cap or more after the edge's (Capped), before its tick clock reaches
// its end. A stamp that stops advancing cannot extend it: the tick clock ends it (Done). A stamp before the edge's ends it
// (StampBack). On the kFast clock.
TEST(BenchProfileStampTest, TheStampCapEndsALaggingProfileAndAStampBeforeTheEdgeEndsIt) {
  in_child([] {
    Rig rig(table_with(overrides_of(kFast)));
    rig.warm_up();
    const std::vector<Segment> s = segments_of(kFast);
    // Lost samples: after the edge, every other sample is not delivered.
    rig.link_up(kBenchUp);
    const Out edge = rig.arm(kBenchUp);
    std::uint64_t k = 0;
    Out o = edge;
    while (true) {
      (void)rig.sample(false);
      ++k;
      o = rig.rate(Frame{kSwitchOn, kBenchUp});
      if (o.stamp - edge.stamp >= kFast.total_max_us) {
        break;
      }
      ASSERT_EQ(o.values(), expected(rig, s, k)) << "k " << k;
      ASSERT_NE(o.status.phase, BenchProfilePhase::Done);
    }
    EXPECT_TRUE(o.all_stop());
    EXPECT_EQ(o.status.phase, BenchProfilePhase::Capped);
    EXPECT_LT(segment_at(s, k * rig.rate_num(), rig.den()), s.size());  // the tick clock had not ended it
    for (int i = 0; i < 8; ++i) {
      EXPECT_TRUE(rig.rate(Frame{kSwitchOn, kBenchUp}).all_stop());
    }
    // A stamp frozen at the edge's: the tick clock ends it at its end.
    rig.link_up(kBenchUp);
    const Out edge2 = rig.arm(kBenchUp);
    for (k = 1;; ++k) {
      o = rig.rate(Frame{kSwitchOn, kBenchUp}, edge2.stamp);
      const std::size_t seg = segment_at(s, k * rig.rate_num(), rig.den());
      ASSERT_EQ(o.values(), values_of(s, seg)) << "k " << k;
      if (seg == s.size()) {
        EXPECT_EQ(o.status.phase, BenchProfilePhase::Done);
        break;
      }
    }
    // A stamp 1 us before the edge's, in a level: stop on that tick.
    rig.link_up(kBenchUp);
    const Out edge3 = rig.arm(kBenchUp);
    for (k = 1; segment_at(s, k * rig.rate_num(), rig.den()) < 2; ++k) {
      o = rig.rate(Frame{kSwitchOn, kBenchUp});
    }
    ASSERT_FALSE(o.all_stop());
    o = rig.rate(Frame{kSwitchOn, kBenchUp}, edge3.stamp - 1);
    EXPECT_TRUE(o.all_stop());
    EXPECT_EQ(o.status.phase, BenchProfilePhase::StampBack);
    EXPECT_TRUE(rig.rate(Frame{kSwitchOn, kBenchUp}).all_stop());
  });
}

// ---- validate and init (V-1) ----

struct RejectCase {
  const char* what;
  Overrides o;
  const char* reason;
  bool init_death = false;  // also run init on it in a forked child (one case per rule: a death test costs seconds)
};

const char* const kChannel = "bench_profile_rc_channel is outside 1..16";
const char* const kCollision = "bench_profile_rc_channel is the arm, the angle or a stick channel";
const char* const kThreshold = "bench_profile_switch_threshold is outside (0, 1]";
const char* const kCap = "bench_profile_dshot_max is below the DShot throttle minimum or above motor_test_dshot_max";
const char* const kLevel = "is below the DShot throttle minimum or above bench_profile_dshot_max";
const char* const kPositive = "bench_profile_level_us, bench_profile_repeats or bench_profile_total_max_us is below 1";
const char* const kTooLong = "is longer than bench_profile_total_max_us";

std::vector<RejectCase> reject_cases() {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double max32 = std::numeric_limits<std::int32_t>::max();
  return {
      {"channel 0", {{ParamId::bench_profile_rc_channel, 0}}, kChannel, true},
      {"channel 17", {{ParamId::bench_profile_rc_channel, 17}}, kChannel},
      {"channel 5 (arm)", {{ParamId::bench_profile_rc_channel, 5}}, kCollision, true},
      {"channel 1 (roll)", {{ParamId::bench_profile_rc_channel, 1}}, kCollision},
      {"channel 2 (pitch)", {{ParamId::bench_profile_rc_channel, 2}}, kCollision},
      {"channel 3 (throttle)", {{ParamId::bench_profile_rc_channel, 3}}, kCollision},
      {"channel 4 (yaw)", {{ParamId::bench_profile_rc_channel, 4}}, kCollision},
      {"channel 8 (angle)", {{ParamId::bench_profile_rc_channel, 8}}, kCollision},
      {"threshold 0 (the middle in position)", {{ParamId::bench_profile_switch_threshold, 0.0}}, kThreshold, true},
      {"threshold -0.5", {{ParamId::bench_profile_switch_threshold, -0.5}}, kThreshold},
      {"threshold above 1 (never in position)",
       {{ParamId::bench_profile_switch_threshold, static_cast<double>(std::nextafter(1.0F, 2.0F))}}, kThreshold},
      {"threshold NaN", {{ParamId::bench_profile_switch_threshold, nan}}, kThreshold},
      {"bench cap 47", {{ParamId::bench_profile_dshot_max, 47}, {ParamId::bench_profile_dshot_base, 47}}, kCap},
      {"bench cap 501 (above the console cap)", {{ParamId::bench_profile_dshot_max, 501}}, kCap, true},
      {"step 451 (above the bench cap)", {{ParamId::bench_profile_dshot_step, 451}}, kLevel, true},
      {"base 451", {{ParamId::bench_profile_dshot_base, 451}}, kLevel},
      {"step 47", {{ParamId::bench_profile_dshot_step, 47}}, kLevel},
      {"base 0 (stop)", {{ParamId::bench_profile_dshot_base, 0}}, kLevel},
      {"step 2047", {{ParamId::bench_profile_dshot_step, 2047}}, kLevel},
      {"lead-in -1", {{ParamId::bench_profile_lead_in_us, -1}}, kPositive, true},
      {"level 0", {{ParamId::bench_profile_level_us, 0}}, kPositive},
      {"repeats 0", {{ParamId::bench_profile_repeats, 0}}, kPositive},
      {"cap 0", {{ParamId::bench_profile_total_max_us, 0}}, kPositive},
      {"cap 38 s - 1 us (the ruled profile is 38 s)", {{ParamId::bench_profile_total_max_us, 37999999}}, kTooLong},
      {"cap 20 s (the first proposal)", {{ParamId::bench_profile_total_max_us, 20000000}}, kTooLong},
      {"repeats 4 (2 + 48 s)", {{ParamId::bench_profile_repeats, 4}}, kTooLong, true},
      {"lead-in above the cap", {{ParamId::bench_profile_lead_in_us, 40000001}}, kTooLong},
      {"repeats and level at the i32 maximum", {{ParamId::bench_profile_repeats, max32}, {ParamId::bench_profile_level_us, max32}},
       kTooLong},
  };
}

TEST(BenchProfileValidateTest, TheDefaultTableAndTheRulesEdgesAreAccepted) {
  EXPECT_TRUE(composition::validate(param_defaults()).ok()) << composition::validate(param_defaults()).reason;
  const std::vector<Overrides> accepted{
      {{ParamId::bench_profile_total_max_us, 38000000}},  // exactly the profile's length
      {{ParamId::bench_profile_dshot_max, 500}, {ParamId::bench_profile_dshot_step, 500}},  // the console cap
      {{ParamId::bench_profile_dshot_base, 48}, {ParamId::bench_profile_dshot_step, 48}},   // the DShot throttle minimum
      {{ParamId::bench_profile_switch_threshold, 1.0}},
      {{ParamId::bench_profile_rc_channel, 16}},
      {{ParamId::bench_profile_rc_channel, 7}},
      {{ParamId::bench_profile_lead_in_us, 0}},
  };
  for (const Overrides& o : accepted) {
    const composition::Verdict v = composition::validate(table_with(o));
    EXPECT_TRUE(v.ok()) << v.reason;
  }
}

TEST(BenchProfileValidateTest, EachRuleRejectsItsTableWithItsReason) {
  for (const RejectCase& c : reject_cases()) {
    const composition::Verdict v = composition::validate(table_with(c.o));
    ASSERT_FALSE(v.ok()) << c.what;
    EXPECT_NE(std::string(v.reason).find(c.reason), std::string::npos) << c.what << ": " << v.reason;
  }
}

std::string regex_escaped(const std::string& text) {
  std::string out;
  for (const char ch : text) {
    if (std::string("\\^$.|?*+()[]{}").find(ch) != std::string::npos) {
      out += '\\';
    }
    out += ch;
  }
  return out;
}

void init_with(const Overrides& o) {
  if (!params_init(table_with(o))) {
    std::exit(2);
  }
  std::array<DshotValue, kMotors> motor_latch{};
  std::array<ServoUs, composition::kServos> servo_latch{};
  hal_sim::setup({static_cast<std::uint32_t>(i32_default(ParamId::tick_period_num_us)),
                  static_cast<std::uint32_t>(i32_default(ParamId::tick_period_den))},
                 motor_latch, servo_latch);
  composition::init();
  std::exit(0);
}

// V-1: init panics iff validate rejects, with validate's reason (one case per rule; a NaN threshold never reaches init, since
// params_init refuses a non-finite F32 record).
TEST(BenchProfileValidateDeathTest, InitPanicsWithTheReasonValidateGives) {
  for (const RejectCase& c : reject_cases()) {
    const composition::Verdict v = composition::validate(table_with(c.o));
    ASSERT_FALSE(v.ok()) << c.what;
    if (!c.init_death) {
      continue;
    }
    EXPECT_EXIT(init_with(c.o), ::testing::KilledBySignal(SIGABRT), std::string("hal_panic: ") + regex_escaped(v.reason))
        << c.what;
  }
  EXPECT_EXIT(init_with({}), ::testing::ExitedWithCode(0), "");
}

}  // namespace
}  // namespace marv
