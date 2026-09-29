#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <optional>

#include "marv/hal/hal.hpp"
#include "marv/hal_sim/hal_sim.hpp"
#include "marv/types/actuator.hpp"

namespace marv::hal_sim {
namespace {

using Wide = unsigned __int128;  // reference arithmetic, test code only

// floor(n * num / den) in 128 bits, reduced modulo 2^64 (the stamp type). The two agree exactly because the
// stamp is (n / den) * num, computed modulo 2^64, plus a term below num.
struct Reference {
  Wide exact;
  TimeUs stamp;
};

Reference reference(TickPeriod p, Tick n) {
  const Wide exact = static_cast<Wide>(n) * p.num_us / p.den;
  return {exact, static_cast<TimeUs>(exact)};
}

constexpr Tick kWindow = 1'000'000;
constexpr Tick kTwo32 = Tick{1} << 32;
constexpr Tick kTwo40 = Tick{1} << 40;

struct Window {
  Tick first;
  const char* name;
};
constexpr std::array<Window, 3> kWindows{{{0, "from zero"},
                                          {kTwo32 - kWindow / 2, "straddling 2^32"},
                                          {kTwo40 - kWindow / 2, "straddling 2^40"}}};

struct CheckResult {
  bool ok = true;
  Tick first_bad = 0;
};

// The one checking function. `stamp` is called once per tick, in order, from `first` for `count` ticks. A clock
// passes if every stamp equals the 128-bit reference and stamps strictly increase. Where the reference itself
// passes 2^64 (only for periods near 2^32 us) the stamp wraps by definition, so increase is not required across it.
template <class StampFn>
CheckResult check_clock(TickPeriod p, Tick first, Tick count, StampFn&& stamp) {
  std::optional<Reference> prev;
  for (Tick n = first; n < first + count; ++n) {
    const TimeUs got = stamp(n);
    const Reference ref = reference(p, n);
    if (got != ref.stamp) {
      return {false, n};
    }
    if (prev && (ref.exact >> 64) == (prev->exact >> 64) && got <= prev->stamp) {
      return {false, n};
    }
    prev = ref;
  }
  return {};
}

const std::array<TickPeriod, 5> kPeriods{{{625, 4}, {12500, 3}, {3125, 24}, {1, 1}, {UINT32_MAX, 1}}};

TEST(StampUs, EqualsTheWideReferenceForEveryPeriodAndWindow) {
  for (const TickPeriod p : kPeriods) {
    ASSERT_TRUE(period_valid(p)) << p.num_us << "/" << p.den;
    for (const Window& w : kWindows) {
      const CheckResult r = check_clock(p, w.first, kWindow, [&](Tick n) { return stamp_us(p, n); });
      EXPECT_TRUE(r.ok) << p.num_us << "/" << p.den << " " << w.name << ": first mismatch at n = " << r.first_bad;
    }
  }
}

TEST(StampUs, IsFloorOfNTimesPeriodAtKnownPoints) {
  constexpr TickPeriod p{625, 4};  // 156.25 us
  static_assert(stamp_us(p, 0) == 0);
  static_assert(stamp_us(p, 1) == 156);
  static_assert(stamp_us(p, 2) == 312);
  static_assert(stamp_us(p, 3) == 468);
  static_assert(stamp_us(p, 4) == 625);
  static_assert(stamp_us(p, 6400) == 1'000'000);
  SUCCEED();
}

TEST(StampUs, StrictlyIncreasesAtEveryPeriodOverTheFirstMillionTicks) {
  for (const TickPeriod p : kPeriods) {
    TimeUs prev = stamp_us(p, 0);
    for (Tick n = 1; n < kWindow; ++n) {
      const TimeUs cur = stamp_us(p, n);
      ASSERT_GT(cur, prev) << p.num_us << "/" << p.den << " at n = " << n;
      prev = cur;
    }
  }
}

TEST(PeriodValid, RefusesZeroDenominatorAndSubMicrosecondPeriods) {
  EXPECT_FALSE(period_valid({1, 0}));
  EXPECT_FALSE(period_valid({0, 0}));
  EXPECT_FALSE(period_valid({0, 1}));
  EXPECT_FALSE(period_valid({3, 4}));
  EXPECT_FALSE(period_valid({624, 625}));
  EXPECT_TRUE(period_valid({4, 4}));
  EXPECT_TRUE(period_valid({1, 1}));
  EXPECT_TRUE(period_valid({625, 4}));
  EXPECT_TRUE(period_valid({UINT32_MAX, UINT32_MAX}));
}

// Negative controls: the checking function must be able to fail. Each fake clock is run through check_clock, the
// same function that passes the real stamp_us above.
TEST(ClockNegativeControl, RoundingClockFailsAtNEquals2) {
  constexpr TickPeriod p{625, 4};
  const CheckResult r = check_clock(p, 0, kWindow, [&](Tick n) -> TimeUs {
    return static_cast<TimeUs>((n * p.num_us + p.den / 2) / p.den);  // 312.5 -> 313
  });
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(r.first_bad, 2U);
}

TEST(ClockNegativeControl, AccumulatingClockFailsAtNEquals4) {
  constexpr TickPeriod p{625, 4};
  TimeUs acc = 0;
  bool first_call = true;
  const CheckResult r = check_clock(p, 0, kWindow, [&](Tick) -> TimeUs {
    if (first_call) {
      first_call = false;
    } else {
      acc += 156;  // truncated period added every tick
    }
    return acc;
  });
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(r.first_bad, 4U);
}

TEST(ClockNegativeControl, TheStrictIncreaseCheckRejectsASubMicrosecondPeriod) {
  constexpr TickPeriod p{1, 2};  // 0.5 us: floor(n / 2) repeats, so even the exact reference does not increase
  ASSERT_FALSE(period_valid(p));
  const CheckResult r = check_clock(p, 0, kWindow, [&](Tick n) { return stamp_us(p, n); });
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(r.first_bad, 1U);
}

TEST(HalTime, IsZeroBeforeTickZeroAndStampThroughoutEachTick) {
  std::array<DshotValue, kQuadXMotors> motor{};
  std::array<ServoUs, 0> servo{};
  constexpr TickPeriod p{625, 4};
  setup(p, motor, servo);
  EXPECT_EQ(hal_time_us(), 0U);
  EXPECT_EQ(hal_time_us(), 0U);
  for (Tick n = 0; n < 64; ++n) {
    begin_tick(n);
    const TimeUs want = reference(p, n).stamp;
    EXPECT_EQ(hal_time_us(), want) << "tick " << n;
    EXPECT_EQ(hal_time_us(), want) << "tick " << n << " second read";
    end_tick();
    EXPECT_EQ(hal_time_us(), want) << "tick " << n << " after end_tick, before the next begin_tick";
  }
}

TEST(HalTime, ReturnsToZeroWhenSetUpAgain) {
  std::array<DshotValue, kQuadXMotors> motor{};
  std::array<ServoUs, 0> servo{};
  setup({625, 4}, motor, servo);
  begin_tick(0);
  begin_tick(1);
  EXPECT_EQ(hal_time_us(), 156U);
  end_tick();
  setup({625, 4}, motor, servo);
  EXPECT_EQ(hal_time_us(), 0U);
  begin_tick(0);
  end_tick();
}

using HalTimeDeathTest = ::testing::Test;

TEST_F(HalTimeDeathTest, BeginTickOutOfOrderPanics) {
  std::array<DshotValue, kQuadXMotors> motor{};
  std::array<ServoUs, 0> servo{};
  constexpr TickPeriod p{625, 4};
  EXPECT_DEATH(({ setup(p, motor, servo); begin_tick(1); }), "hal_panic: .*out of order");
  EXPECT_DEATH(({ setup(p, motor, servo); begin_tick(0); begin_tick(2); }), "hal_panic: .*out of order");
  EXPECT_DEATH(({ setup(p, motor, servo); begin_tick(0); begin_tick(0); }), "hal_panic: .*out of order");
  EXPECT_DEATH(({ setup(p, motor, servo); begin_tick(0); begin_tick(1); begin_tick(0); }), "hal_panic: .*out of order");
}

TEST_F(HalTimeDeathTest, SetupWithAnInvalidPeriodPanics) {
  std::array<DshotValue, kQuadXMotors> motor{};
  std::array<ServoUs, 0> servo{};
  EXPECT_DEATH(setup({1, 0}, motor, servo), "hal_panic: .*invalid tick period");
  EXPECT_DEATH(setup({3, 4}, motor, servo), "hal_panic: .*invalid tick period");
}

}  // namespace
}  // namespace marv::hal_sim
