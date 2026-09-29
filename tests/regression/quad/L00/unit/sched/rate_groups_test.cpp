#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <type_traits>

#include "marv/sched/rate_groups.hpp"
#include "marv/types/time.hpp"

namespace marv::sched {
namespace {

constexpr Tick kWindow = 1'000'000;
constexpr Tick kTwo32 = Tick{1} << 32;
constexpr Tick kTwo40 = Tick{1} << 40;

struct Window {
  Tick begin;
  Tick end;
  const char* name;
};
constexpr std::array<Window, 3> kWindows{{{0, kWindow, "from zero"},
                                          {kTwo32 - kWindow / 2, kTwo32 + kWindow / 2, "straddling 2^32"},
                                          {kTwo40 - kWindow / 2, kTwo40 + kWindow / 2, "straddling 2^40"}}};

enum class BadKind { kFiring, kCount };

struct Bad {
  Tick n;         // first tick whose mask disagrees; for kCount, n_end (the count is only known at the end)
  std::size_t group;
  BadKind kind;
};

// Number of multiples of d in [begin, end): ceil(end / d) - ceil(begin / d). For begin = 0 this is (end - 1) / d + 1.
constexpr Tick expected_count(Tick begin, Tick end, std::uint32_t d) {
  const auto ceil_div = [d](Tick x) { return x / d + (x % d != 0 ? 1 : 0); };
  return ceil_div(end) - ceil_div(begin);
}
static_assert(expected_count(0, 1, 7) == 1 && expected_count(0, 7, 7) == 1 && expected_count(0, 8, 7) == 2);
static_assert(expected_count(1, 8, 7) == 1 && expected_count(7, 8, 7) == 1 && expected_count(8, 14, 7) == 0);

// The one checking function. `due` is called once per tick, in order, for n in [n_begin, n_end). The definition:
// group i fires at tick n iff n is an exact multiple of divisors[i], and no bit outside the N groups is set. After
// the sweep, each group's fire count must equal the closed form. Returns the first disagreement, if any.
template <std::size_t N, class DueFn>
std::optional<Bad> first_bad(DueFn&& due, const std::array<std::uint32_t, N>& divisors, Tick n_begin, Tick n_end) {
  std::array<Tick, N> fired{};
  for (Tick n = n_begin; n < n_end; ++n) {
    const std::uint32_t mask = due(n);
    for (std::size_t i = 0; i < N; ++i) {
      const bool want = (n % divisors[i]) == 0;
      const bool got = ((mask >> i) & 1U) != 0;
      if (want != got) {
        return Bad{n, i, BadKind::kFiring};
      }
      fired[i] += got ? 1 : 0;
    }
    if constexpr (N < std::numeric_limits<std::uint32_t>::digits) {
      if ((mask >> N) != 0) {
        return Bad{n, N, BadKind::kFiring};
      }
    }
  }
  for (std::size_t i = 0; i < N; ++i) {
    if (fired[i] != expected_count(n_begin, n_end, divisors[i])) {
      return Bad{n_end, i, BadKind::kCount};
    }
  }
  return std::nullopt;
}

template <std::size_t N>
RateGroups<N> make(const std::array<std::uint32_t, N>& divisors) {
  RateGroups<N> rg;
  EXPECT_TRUE(rg.init(divisors));
  return rg;
}

template <std::size_t N>
void expect_scheduler_passes(const std::array<std::uint32_t, N>& divisors) {
  const RateGroups<N> rg = make(divisors);
  for (const Window& w : kWindows) {
    const auto bad = first_bad(
        [&](Tick n) { return rg.due(n); }, divisors, w.begin, w.end);
    EXPECT_FALSE(bad.has_value()) << "N=" << N << " " << w.name << ": first bad n=" << (bad ? bad->n : 0)
                                  << " group " << (bad ? bad->group : 0);
  }
}

TEST(RateGroups, HarmonicSetMatchesTheDefinitionInEveryWindow) {
  expect_scheduler_passes<5>({1, 2, 4, 16, 64});
}

TEST(RateGroups, NonHarmonicSetMatchesTheDefinitionInEveryWindow) { expect_scheduler_passes<4>({1, 3, 5, 7}); }

TEST(RateGroups, SingleGroupMatchesTheDefinitionInEveryWindow) { expect_scheduler_passes<1>({1}); }

TEST(RateGroups, MaxWidthMatchesTheDefinitionInEveryWindow) {
  constexpr std::size_t kMax = std::numeric_limits<std::uint32_t>::digits;
  std::array<std::uint32_t, kMax> d{};
  for (std::size_t i = 0; i < kMax; ++i) {
    d[i] = static_cast<std::uint32_t>(i) + 1;
  }
  expect_scheduler_passes<kMax>(d);
  d[kMax - 1] = UINT32_MAX;  // a divisor larger than any window
  expect_scheduler_passes<kMax>(d);
}

TEST(RateGroups, TickZeroFiresEveryGroupAndDivisorIsReadBack) {
  const std::array<std::uint32_t, 4> d{1, 3, 5, 7};
  const RateGroups<4> rg = make(d);
  EXPECT_EQ(rg.due(0), 0xFU);
  for (std::size_t i = 0; i < d.size(); ++i) {
    EXPECT_EQ(rg.divisor(i), d[i]);
  }
}

TEST(RateGroups, DueIsStatelessSoAnyTickCanBeAskedInAnyOrder) {
  const RateGroups<3> rg = make<3>({1, 4, 6});
  // 2^40 mod 4 = 0 and 2^40 mod 6 = 4.
  EXPECT_EQ(rg.due(kTwo40 + 8), 0x7U);
  EXPECT_EQ(rg.due(3), 0x1U);
  EXPECT_EQ(rg.due(kTwo40 + 8), 0x7U);
  EXPECT_EQ(rg.due(kTwo40 + 2), 0x5U);
  EXPECT_EQ(rg.due(kTwo40 + 4), 0x3U);
}

TEST(RateGroups, InitRejectsAZeroDivisorAndStaysUninitialised) {
  RateGroups<3> rg;
  EXPECT_FALSE(rg.init({1, 0, 4}));
  EXPECT_DEATH((void)rg.due(0), "hal_panic: .*before init");
  EXPECT_TRUE(rg.init({1, 2, 4}));
  EXPECT_EQ(rg.due(2), 0x3U);
}

TEST(RateGroups, RejectedReinitKeepsThePreviousConfiguration) {
  RateGroups<2> rg;
  ASSERT_TRUE(rg.init({1, 2}));
  EXPECT_FALSE(rg.init({0, 2}));
  EXPECT_EQ(rg.divisor(1), 2U);
  EXPECT_EQ(rg.due(1), 0x1U);
}

TEST(RateGroups, DueBeforeInitPanics) {
  const RateGroups<2> rg;
  EXPECT_DEATH((void)rg.due(0), "hal_panic: RateGroups::due before init");
}

static_assert(std::is_trivially_copyable_v<RateGroups<1>>);
static_assert(std::is_trivially_copyable_v<RateGroups<5>>);
static_assert(std::is_trivially_copyable_v<RateGroups<32>>);

// Negative controls: broken schedulers through the same checker.

// Fires on (n + 1) % d == 0. Wrong at n = 0 for any d > 1: the definition wants the group to fire, this does not.
struct OffByOne {
  const std::array<std::uint32_t, 4>& d;
  std::uint32_t operator()(Tick n) const {
    std::uint32_t mask = 0;
    for (std::size_t i = 0; i < d.size(); ++i) {
      if ((n + 1) % d[i] == 0) {
        mask |= std::uint32_t{1} << i;
      }
    }
    return mask;
  }
};

// Accumulating counter: a 32-bit tick counter seeded from the first tick asked about, advanced once per call, that
// skips the value 0 when it overflows (the classic "counter == 0 means unset" bug). Indistinguishable from the
// definition until the counter wraps, then it is one tick ahead, so a group with an even divisor misses its tick.
struct SkipsAfterWrap {
  const std::array<std::uint32_t, 5>& d;
  bool seeded = false;
  std::uint32_t counter = 0;
  std::uint32_t operator()(Tick n) {
    if (!seeded) {
      counter = static_cast<std::uint32_t>(n);
      seeded = true;
    }
    std::uint32_t mask = 0;
    for (std::size_t i = 0; i < d.size(); ++i) {
      if (counter % d[i] == 0) {
        mask |= std::uint32_t{1} << i;
      }
    }
    ++counter;
    if (counter == 0) {
      ++counter;
    }
    return mask;
  }
};

TEST(RateGroupsNegativeControl, OffByOneIsReportedAtTickZero) {
  const std::array<std::uint32_t, 4> d{1, 3, 5, 7};
  for (const Window& w : kWindows) {
    const auto bad = first_bad(OffByOne{d}, d, w.begin, w.end);
    ASSERT_TRUE(bad.has_value()) << w.name;
    // Group 0 (d = 1) cannot tell the variants apart; group 1 (d = 3) is the first that can.
    const Tick predicted = w.begin;  // window start: (n + 1) % 3 == 0 differs from n % 3 == 0 for every n
    EXPECT_EQ(bad->n, predicted) << w.name;
    EXPECT_EQ(bad->group, 1U) << w.name;
    EXPECT_EQ(bad->kind, BadKind::kFiring) << w.name;
  }
}

TEST(RateGroupsNegativeControl, CounterThatSkipsAfterOverflowIsReportedAtTheWrap) {
  const std::array<std::uint32_t, 5> d{1, 2, 4, 16, 64};
  {
    const auto bad = first_bad(SkipsAfterWrap{d}, d, kWindows[0].begin, kWindows[0].end);
    EXPECT_FALSE(bad.has_value()) << "the variant is correct until its counter wraps, so it needs a wrap window";
  }
  {
    const auto bad = first_bad(SkipsAfterWrap{d}, d, kWindows[1].begin, kWindows[1].end);
    ASSERT_TRUE(bad.has_value());
    EXPECT_EQ(bad->n, kTwo32);
    EXPECT_EQ(bad->group, 1U);  // d = 2 is the first group that fires at n = 2^32 and is missed
    EXPECT_EQ(bad->kind, BadKind::kFiring);
  }
  {
    const auto bad = first_bad(SkipsAfterWrap{d}, d, kWindows[2].begin, kWindows[2].end);
    ASSERT_TRUE(bad.has_value());
    EXPECT_EQ(bad->n, kTwo40);  // the 32-bit counter wraps at every multiple of 2^32, 2^40 included
    EXPECT_EQ(bad->group, 1U);
    EXPECT_EQ(bad->kind, BadKind::kFiring);
  }
}

}  // namespace
}  // namespace marv::sched
