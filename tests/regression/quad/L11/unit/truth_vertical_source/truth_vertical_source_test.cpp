// The truth-vertical source (quad L11): TruthVertical fills marv_truth_vertical from the plant body as the float casts of
// pos_ned[2] and vel_ned[2] with the valid bit only, or with no bit and zero fields when a cast is not finite; it holds
// them over the host step's ticks while `tick` follows the tick about to run, calls marv_truth_vertical_set before the
// inner source (here the plain SIL source, marv_sil_tick), and returns false without calling it when the set is refused.
//
// The SIL cases use the L11 truth-vertical test library (marv_sil_l11_truth_vertical_test, composition in
// ../truth_vertical): its motor outputs report, every tick, [vertical_input calls since the previous tick, flags, hash of
// the latched VerticalState] (probe_codes.hpp), which the test compares with what the body should have produced.
//
// Negative controls: one float ulp on z or v_z fails the bitwise fill check and changes the reported hash; a state stamped
// one tick late changes the stamp flag and the hash; a set with the wrong tick is refused and the SIL does not run.
#include <gtest/gtest.h>

#include <marv_sil.h>
#include <marv_truth_vertical.h>

#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>

#include <marv/params/param_ids.hpp>
#include <marv/types/vertical_state.hpp>

#include "marv/gz/adapter.hpp"
#include "marv/gz/truth_vertical.hpp"
#include "marv_plant.h"
#include "probe_codes.hpp"

namespace {

// The SIL period: 625 / 4 us (scenario value, as the L2 to L5 adapter tests).
constexpr std::uint32_t kPeriodNumUs = 625;
constexpr std::uint32_t kPeriodDen = 4;
// Scenario values: 3 ticks per host step so that a hold over more than one tick is exercised, over 3 host steps, one
// body each.
constexpr std::uint32_t kTicksPerStep = 3;
constexpr std::uint32_t kStepsHeld = 3;
constexpr std::uint32_t kValidBit = 1u << MARV_TRUTH_VERTICAL_VALID;

struct Body {
  double z;
  double vz;
};

// Scenario values: heights and vertical speeds whose float cast rounds (not exactly representable in float), both signs,
// a signed zero and magnitudes over several decades.
const std::array<Body, 3> kBodies = {{
    {-1.0 / 3.0, 0.1},
    {123.456789012345, -2.718281828459045},
    {-0.0, 1.0e-30},
}};

marv_plant_body plant_body(const Body& b) {
  marv_plant_body p{};
  p.struct_size = sizeof(p);
  p.q_wxyz[0] = 1.0;
  p.pos_ned_m[0] = 7.0;  // the north and east components are not part of the state
  p.pos_ned_m[1] = -7.0;
  p.pos_ned_m[2] = b.z;
  p.vel_ned_m_s[0] = 7.0;
  p.vel_ned_m_s[1] = -7.0;
  p.vel_ned_m_s[2] = b.vz;
  return p;
}

std::uint32_t bits(float f) { return std::bit_cast<std::uint32_t>(f); }

// True iff the state's z and v_z are, bitwise, the float casts of `b`.
bool fill_is_cast_of(const marv_truth_vertical& s, const Body& b) {
  return bits(s.z_ned_m) == bits(static_cast<float>(b.z)) && bits(s.vz_ned_m_s) == bits(static_cast<float>(b.vz));
}

// A command source standing in for marv_sil_tick in the tests that do not run the SIL.
class CountingSource final : public marv::gz::CommandSource {
 public:
  bool dshot(std::uint64_t /*tick*/, marv::gz::Dshot& out) override {
    out = {};
    return true;
  }
};

TEST(L11TruthVerticalSource, ZAndVzAreTheFloatCastsOfTheBodyBitwiseAndTheValidBitIsSet) {
  for (const Body& b : kBodies) {
    CountingSource inner;
    marv::truth::TruthVertical src(inner);
    src.set_body(plant_body(b));
    const marv_truth_vertical& s = src.state();
    EXPECT_TRUE(fill_is_cast_of(s, b));
    EXPECT_EQ(s.struct_size, sizeof(marv_truth_vertical));
    EXPECT_EQ(s.flags, kValidBit);
    // Negative control: one ulp on either field is not the cast.
    marv_truth_vertical ulp = s;
    ulp.z_ned_m = std::nextafter(ulp.z_ned_m, std::numeric_limits<float>::infinity());
    EXPECT_FALSE(fill_is_cast_of(ulp, b));
    ulp = s;
    ulp.vz_ned_m_s = std::nextafter(ulp.vz_ned_m_s, -std::numeric_limits<float>::infinity());
    EXPECT_FALSE(fill_is_cast_of(ulp, b));
  }
}

TEST(L11TruthVerticalSource, ANonFiniteCastClearsTheValidBitAndZeroesTheFields) {
  // A NaN, an infinity, and a finite double beyond the float range (its cast is infinite).
  const std::array<Body, 3> bad = {{
      {std::numeric_limits<double>::quiet_NaN(), 0.5},
      {0.5, -std::numeric_limits<double>::infinity()},
      {static_cast<double>(std::numeric_limits<float>::max()) * 2.0, 0.5},
  }};
  for (const Body& b : bad) {
    CountingSource inner;
    marv::truth::TruthVertical src(inner);
    src.set_body(plant_body(kBodies.front()));
    src.set_body(plant_body(b));
    const marv_truth_vertical& s = src.state();
    EXPECT_EQ(s.struct_size, sizeof(marv_truth_vertical));
    EXPECT_EQ(s.flags, 0u);
    EXPECT_EQ(bits(s.z_ned_m), 0u);
    EXPECT_EQ(bits(s.vz_ned_m_s), 0u);
  }
}

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

// marv_sil_init succeeds once per process: each SIL case runs in a forked child, which reports its failures on stderr and
// exits 1 if there were any.
template <class F>
void in_child(F body) {
  EXPECT_EXIT(
      {
        body();
        std::exit(report_failures_on_stderr() ? 1 : 0);
      },
      ::testing::ExitedWithCode(0), "");
}

void reach_ready() {
  marv_sil_config c{};
  c.struct_size = sizeof(marv_sil_config);
  c.imu_meas_size = sizeof(marv_imu_meas);
  c.override_size = sizeof(marv_sil_param_override);
  c.tick_period_num_us = kPeriodNumUs;
  c.tick_period_den = kPeriodDen;
  c.param_schema_hash = marv::kParamSchemaHash;
  ASSERT_EQ(marv_sil_init(&c), MARV_SIL_OK);
}

std::uint64_t stamp_of(std::uint64_t tick) {
  return static_cast<std::uint64_t>(static_cast<unsigned __int128>(tick) * kPeriodNumUs / kPeriodDen);
}

// What the test composition should have latched for `tick` when the body was `b` at the start of the host step: the
// valid state stamped with the SIL's stamp of the tick.
marv::VerticalState<float> hand_over_of(const Body& b, std::uint64_t tick) {
  return marv::VerticalState<float>{stamp_of(tick), static_cast<float>(b.z), static_cast<float>(b.vz), true};
}

// The row the composition reports for one hand-over since the previous tick of `v`, stamped (or not) as the tick.
verticaltest::Row row_of(const marv::VerticalState<float>& v, bool stamp_matches) {
  const std::uint32_t flags = verticaltest::kFlagHave | (v.valid ? verticaltest::kFlagValid : 0u) |
                              (stamp_matches ? verticaltest::kFlagStampMatches : 0u);
  return verticaltest::row_of(1, flags, verticaltest::hash_of(v));
}

bool same_row(const verticaltest::Row& a, const marv::gz::Dshot& d) {
  return a.calls == d[0] && a.flags == d[1] && a.hash_lo == d[2] && a.hash_hi == d[3];
}

TEST(L11TruthVerticalSource, EveryTickHandsTheHeldStateToTheCompositionBeforeTheSilTick) {
  in_child([] {
    reach_ready();
    marv::gz::SilCommandSource sil;
    marv::truth::TruthVertical src(sil);
    std::uint64_t tick = 0;
    for (std::uint32_t step = 0; step < kStepsHeld; ++step) {
      const Body& b = kBodies[step % kBodies.size()];
      src.set_body(plant_body(b));
      for (std::uint32_t j = 0; j < kTicksPerStep; ++j, ++tick) {
        marv::gz::Dshot d{};
        ASSERT_TRUE(src.dshot(tick, d)) << "tick " << tick;
        EXPECT_EQ(src.truth_status(), MARV_SIL_OK);
        EXPECT_EQ(sil.last_status(), MARV_SIL_OK);
        // The state passed: this tick, the held z and v_z, the valid bit.
        EXPECT_EQ(src.state().tick, tick);
        EXPECT_EQ(src.state().flags, kValidBit);
        EXPECT_TRUE(fill_is_cast_of(src.state(), b));
        // The composition latched exactly this state for this tick, once.
        const marv::VerticalState<float> want = hand_over_of(b, tick);
        EXPECT_TRUE(same_row(row_of(want, true), d)) << "tick " << tick;
        // Negative controls: the row of a state stamped one tick late, or with one ulp off, is not the reported one.
        EXPECT_FALSE(same_row(row_of(hand_over_of(b, tick + 1), false), d)) << "tick " << tick;
        marv::VerticalState<float> ulp = want;
        ulp.z_ned_m = std::nextafter(ulp.z_ned_m, std::numeric_limits<float>::infinity());
        EXPECT_FALSE(same_row(row_of(ulp, true), d)) << "tick " << tick;
        ulp = want;
        ulp.vz_ned_m_s = std::nextafter(ulp.vz_ned_m_s, -std::numeric_limits<float>::infinity());
        EXPECT_FALSE(same_row(row_of(ulp, true), d)) << "tick " << tick;
      }
    }
  });
}

TEST(L11TruthVerticalSource, ANonFiniteBodyIsHandedOverAsAnInvalidState) {
  in_child([] {
    reach_ready();
    marv::gz::SilCommandSource sil;
    marv::truth::TruthVertical src(sil);
    src.set_body(plant_body({std::numeric_limits<double>::quiet_NaN(), 0.5}));
    marv::gz::Dshot d{};
    ASSERT_TRUE(src.dshot(0, d));
    EXPECT_EQ(src.truth_status(), MARV_SIL_OK);
    EXPECT_TRUE(same_row(row_of(marv::VerticalState<float>{stamp_of(0), 0.0F, 0.0F, false}, true), d));
  });
}

TEST(L11TruthVerticalSource, ARefusedSetReturnsFalseWithItsStatusAndTheSilDoesNotRun) {
  in_child([] {
    reach_ready();
    marv::gz::SilCommandSource sil;
    marv::truth::TruthVertical src(sil);
    src.set_body(plant_body(kBodies.front()));
    marv::gz::Dshot d{};
    // A tick that is not the next one: E_TICK, and marv_sil_tick has not run (no stamp yet).
    EXPECT_FALSE(src.dshot(1, d));
    EXPECT_EQ(src.truth_status(), MARV_SIL_E_TICK);
    EXPECT_EQ(sil.last_stamp_us(), 0u);
    // Tick 0 is still the next one: the refused call changed nothing.
    ASSERT_TRUE(src.dshot(0, d));
    EXPECT_EQ(src.truth_status(), MARV_SIL_OK);
    // Tick 2 before tick 1: refused, and tick 1 is not run (the stamp stays that of tick 0).
    EXPECT_FALSE(src.dshot(2, d));
    EXPECT_EQ(src.truth_status(), MARV_SIL_E_TICK);
    EXPECT_EQ(sil.last_stamp_us(), stamp_of(0));
    ASSERT_TRUE(src.dshot(1, d));
    EXPECT_EQ(sil.last_stamp_us(), stamp_of(1));
  });
}

}  // namespace
