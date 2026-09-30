// The truth-attitude source (quad L5): TruthAttitude fills marv_truth_state from the plant body as the float casts of q
// and omega with the valid bit only (padding zero), holds them over the host step's ticks while `tick` follows the tick
// about to run, calls marv_truth_state_set before each marv_sil_tick, and returns false without calling the SIL when the
// set is refused.
//
// The SIL cases use the L5 truth-state test library (marv_sil_l05_truth_test, composition in ../truth_state): its motor
// outputs report, every tick, [attitude_input calls since the previous tick, flags, hash of the latched AttitudeState]
// (probe_codes.hpp), which the test compares with what the body should have produced.
//
// Negative controls: one float ulp on any component of q or omega fails the bitwise fill check and changes the reported
// hash; a state stamped one tick late changes the stamp flag and the hash; a set with the wrong tick is refused and the
// SIL does not run.
#include <gtest/gtest.h>

#include <marv_sil.h>
#include <marv_truth.h>

#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>

#include <marv/params/param_ids.hpp>
#include <marv/types/attitude_state.hpp>

#include "marv/gz/adapter.hpp"
#include "marv/gz/truth_attitude.hpp"
#include "marv/gz/truth_gyro.hpp"
#include "marv_plant.h"
#include "probe_codes.hpp"

namespace {

// The SIL period: 625 / 4 us (scenario value, as the L2 and L4 adapter tests).
constexpr std::uint32_t kPeriodNumUs = 625;
constexpr std::uint32_t kPeriodDen = 4;
// Scenario values: 3 ticks per host step so that a hold over more than one tick is exercised, over 3 host steps, one
// body each.
constexpr std::uint32_t kTicksPerStep = 3;
constexpr std::uint32_t kStepsHeld = 3;
constexpr std::uint32_t kValidBit = 1u << MARV_TRUTH_ATTITUDE_VALID;

struct Body {
  std::array<double, 4> q;
  std::array<double, 3> omega;
};

// Scenario values: attitudes and rates whose float cast rounds (not exactly representable in float), mixed signs, a
// signed zero and magnitudes over several decades. The quaternions are unit to double precision.
const std::array<Body, 3> kBodies = {{
    {{0.9238795325112867, 0.0, 0.3826834323650898, 0.0}, {0.1, -1.0 / 3.0, 2.718281828459045}},
    {{0.5, -0.5, 0.5, -0.5}, {-123.456789012345, 0.7071067811865476, -1.0e-3}},
    {{0.7071067811865476, 0.0, -0.0, 0.7071067811865475}, {0.0, -0.0, 1.0e-30}},
}};

marv_plant_body plant_body(const Body& b) {
  marv_plant_body p{};
  p.struct_size = sizeof(p);
  for (std::size_t i = 0; i < b.q.size(); ++i) {
    p.q_wxyz[i] = b.q[i];
  }
  for (std::size_t i = 0; i < b.omega.size(); ++i) {
    p.omega_frd_rad_s[i] = b.omega[i];
  }
  return p;
}

std::uint32_t bits(float f) { return std::bit_cast<std::uint32_t>(f); }

// True iff the state's q and omega are, bitwise per component, the float casts of `b`.
bool fill_is_cast_of(const marv_truth_state& s, const Body& b) {
  for (std::size_t i = 0; i < b.q.size(); ++i) {
    if (bits(s.q_wxyz[i]) != bits(static_cast<float>(b.q[i]))) {
      return false;
    }
  }
  const std::array<float, 3> w = {s.omega_frd_rad_s.x, s.omega_frd_rad_s.y, s.omega_frd_rad_s.z};
  for (std::size_t i = 0; i < b.omega.size(); ++i) {
    if (bits(w[i]) != bits(static_cast<float>(b.omega[i]))) {
      return false;
    }
  }
  return true;
}

// A command source standing in for marv_sil_tick in the tests that do not run the SIL.
class CountingSource final : public marv::gz::CommandSource {
 public:
  bool dshot(std::uint64_t /*tick*/, marv::gz::Dshot& out) override {
    out = {};
    return true;
  }
};

TEST(L5TruthAttitude, QAndOmegaAreTheFloatCastsOfTheBodyBitwiseAndTheValidBitIsSet) {
  for (const Body& b : kBodies) {
    CountingSource inner;
    marv::truth::TruthAttitude src(inner);
    src.set_body(plant_body(b));
    const marv_truth_state& s = src.state();
    EXPECT_TRUE(fill_is_cast_of(s, b));
    EXPECT_EQ(s.struct_size, sizeof(marv_truth_state));
    EXPECT_EQ(s.flags, kValidBit);
  }
}

TEST(L5TruthAttitude, QIsCanonicalWNotBelowZeroAfterTheFloatCast) {
  // w < 0: all four components negated (signed zeros included); w == 0 exactly (either sign): unchanged.
  const Body negative = {{-0.9238795325112867, 0.0, -0.3826834323650898, -0.0}, {0.1, 0.2, 0.3}};
  const Body zero_w = {{0.0, -0.5, 0.5, -0.7071067811865476}, {0.1, 0.2, 0.3}};
  const Body neg_zero_w = {{-0.0, -0.5, 0.5, -0.7071067811865476}, {0.1, 0.2, 0.3}};
  CountingSource inner;
  marv::truth::TruthAttitude src(inner);
  src.set_body(plant_body(negative));
  for (std::size_t i = 0; i < negative.q.size(); ++i) {
    EXPECT_EQ(bits(src.state().q_wxyz[i]), bits(-static_cast<float>(negative.q[i]))) << "component " << i;
  }
  EXPECT_GE(src.state().q_wxyz[MARV_TRUTH_Q_W], 0.0f);
  for (const Body& b : {zero_w, neg_zero_w}) {
    src.set_body(plant_body(b));
    EXPECT_TRUE(fill_is_cast_of(src.state(), b));
  }
}

TEST(L5TruthAttitude, ThePaddingOfTheStateIsZeroSoTheLoggedBytesAreDeterministic) {
  for (const Body& b : kBodies) {
    CountingSource inner;
    marv::truth::TruthAttitude src(inner);
    src.set_body(plant_body(b));
    const auto* raw = reinterpret_cast<const unsigned char*>(&src.state());
    const std::size_t end_of_fields = offsetof(marv_truth_state, omega_frd_rad_s) + sizeof(marv_vec3f);
    for (std::size_t i = end_of_fields; i < sizeof(marv_truth_state); ++i) {
      EXPECT_EQ(raw[i], 0u) << "byte " << i;
    }
  }
}

TEST(L5TruthAttitude, NegativeControlOneFloatUlpOnAnyComponentFailsTheBitwiseCheck) {
  for (const Body& b : kBodies) {
    CountingSource inner;
    marv::truth::TruthAttitude src(inner);
    src.set_body(plant_body(b));
    ASSERT_TRUE(fill_is_cast_of(src.state(), b));
    for (std::size_t i = 0; i < b.q.size() + b.omega.size(); ++i) {
      for (const float toward : {std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity()}) {
        Body planted = b;
        double& c = i < b.q.size() ? planted.q[i] : planted.omega[i - b.q.size()];
        c = static_cast<double>(std::nextafter(static_cast<float>(c), toward));
        EXPECT_FALSE(fill_is_cast_of(src.state(), planted)) << "component " << i;
      }
    }
  }
}

TEST(L5TruthAttitude, TheStateFollowsEachSetBody) {
  CountingSource inner;
  marv::truth::TruthAttitude src(inner);
  for (const Body& b : kBodies) {
    src.set_body(plant_body(b));
    EXPECT_TRUE(fill_is_cast_of(src.state(), b));
  }
  EXPECT_FALSE(fill_is_cast_of(src.state(), kBodies.front()));
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

// marv_sil_init succeeds once per process, so the case runs in a forked child (as the L4 truth-gyro test).
template <class Fn>
void in_child(Fn fn) {
  EXPECT_EXIT(
      {
        fn();
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
marv::AttitudeState<float> hand_over_of(const Body& b, std::uint64_t tick) {
  return marv::AttitudeState<float>{
      stamp_of(tick),
      marv::prim::Quat<float>{static_cast<float>(b.q[0]), static_cast<float>(b.q[1]), static_cast<float>(b.q[2]),
                              static_cast<float>(b.q[3])},
      marv::prim::Vec3<float>{static_cast<float>(b.omega[0]), static_cast<float>(b.omega[1]),
                              static_cast<float>(b.omega[2])},
      true};
}

// The row the composition reports for one hand-over since the previous tick of `a`, stamped (or not) as the tick.
truthtest::Row row_of(const marv::AttitudeState<float>& a, bool stamp_matches) {
  const std::uint32_t flags = truthtest::kFlagHave | truthtest::kFlagValid |
                              (stamp_matches ? truthtest::kFlagStampMatches : 0u);
  return truthtest::row_of(1, flags, truthtest::hash_of(a));
}

bool same_row(const truthtest::Row& a, const marv::gz::Dshot& d) {
  return a.calls == d[0] && a.flags == d[1] && a.hash_lo == d[2] && a.hash_hi == d[3];
}

TEST(L5TruthAttitude, EveryTickHandsTheHeldStateToTheCompositionBeforeTheSilTick) {
  in_child([] {
    reach_ready();
    marv::truth::TruthGyroSilCommandSource gyro;
    marv::truth::TruthAttitude src(gyro);
    std::uint64_t tick = 0;
    for (std::uint32_t step = 0; step < kStepsHeld; ++step) {
      const Body& b = kBodies[step % kBodies.size()];
      const marv_plant_body body = plant_body(b);
      gyro.set_body(body);
      src.set_body(body);
      for (std::uint32_t j = 0; j < kTicksPerStep; ++j, ++tick) {
        marv::gz::Dshot d{};
        ASSERT_TRUE(src.dshot(tick, d)) << "tick " << tick;
        EXPECT_EQ(src.truth_status(), MARV_SIL_OK);
        EXPECT_EQ(gyro.last_status(), MARV_SIL_OK);
        // The state passed: this tick, the held q and omega, the valid bit.
        EXPECT_EQ(src.state().tick, tick);
        EXPECT_EQ(src.state().flags, kValidBit);
        EXPECT_TRUE(fill_is_cast_of(src.state(), b));
        // The composition latched exactly this state for this tick, once.
        const marv::AttitudeState<float> want = hand_over_of(b, tick);
        EXPECT_TRUE(same_row(row_of(want, true), d)) << "tick " << tick;
        // Negative controls: the row of a state stamped one tick late, or with one ulp off, is not the reported one.
        EXPECT_FALSE(same_row(row_of(hand_over_of(b, tick + 1), false), d)) << "tick " << tick;
        marv::AttitudeState<float> ulp = want;
        ulp.q.w = std::nextafter(ulp.q.w, std::numeric_limits<float>::infinity());
        EXPECT_FALSE(same_row(row_of(ulp, true), d)) << "tick " << tick;
        ulp = want;
        ulp.omega_frd[2] = std::nextafter(ulp.omega_frd[2], -std::numeric_limits<float>::infinity());
        EXPECT_FALSE(same_row(row_of(ulp, true), d)) << "tick " << tick;
      }
    }
  });
}

TEST(L5TruthAttitude, ARefusedSetReturnsFalseWithItsStatusAndTheSilDoesNotRun) {
  in_child([] {
    reach_ready();
    marv::truth::TruthGyroSilCommandSource gyro;
    marv::truth::TruthAttitude src(gyro);
    const marv_plant_body body = plant_body(kBodies.front());
    gyro.set_body(body);
    src.set_body(body);
    marv::gz::Dshot d{};
    // A tick that is not the next one: E_TICK, and marv_sil_tick has not run (no stamp yet).
    EXPECT_FALSE(src.dshot(1, d));
    EXPECT_EQ(src.truth_status(), MARV_SIL_E_TICK);
    EXPECT_EQ(gyro.last_stamp_us(), 0u);
    // Tick 0 is still the next one: the refused call changed nothing.
    ASSERT_TRUE(src.dshot(0, d));
    EXPECT_EQ(src.truth_status(), MARV_SIL_OK);
    // A non-finite truth: E_INPUT, and tick 1 is not run (the stamp stays that of tick 0).
    marv_plant_body nan_body = body;
    nan_body.omega_frd_rad_s[1] = std::numeric_limits<double>::quiet_NaN();
    src.set_body(nan_body);
    EXPECT_FALSE(src.dshot(1, d));
    EXPECT_EQ(src.truth_status(), MARV_SIL_E_INPUT);
    EXPECT_EQ(gyro.last_stamp_us(), stamp_of(0));
    // The same tick with a good state is accepted and run.
    src.set_body(body);
    ASSERT_TRUE(src.dshot(1, d));
    EXPECT_EQ(gyro.last_stamp_us(), stamp_of(1));
  });
}

}  // namespace
