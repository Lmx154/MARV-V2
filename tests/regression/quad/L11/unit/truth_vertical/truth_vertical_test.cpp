// T1 of marv_truth_vertical_set: status codes, the valid-bit rule, replace-on-second-set, hand-over. marv_sil_init
// succeeds once per process, so every test body runs in a forked child (the parent never initialises).
#include <gtest/gtest.h>

#include <marv_sil.h>
#include <marv_truth_vertical.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>

#include <marv/composition.hpp>
#include <marv/params/param_ids.hpp>
#include <marv/types/vertical_state.hpp>

#include "probe_codes.hpp"

namespace {

constexpr std::uint32_t kPeriodNumUs = 625;
constexpr std::uint32_t kPeriodDen = 4;
constexpr std::size_t kMotors = marv::composition::kMotors;
constexpr std::uint32_t kValid = 1u << MARV_TRUTH_VERTICAL_VALID;
constexpr std::size_t kFieldCount = 2;  // z_ned_m, vz_ned_m_s
constexpr float kNan = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();

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

marv_sil_config make_config() {
  marv_sil_config c{};
  c.struct_size = sizeof(marv_sil_config);
  c.imu_meas_size = sizeof(marv_imu_meas);
  c.override_size = sizeof(marv_sil_param_override);
  c.tick_period_num_us = kPeriodNumUs;
  c.tick_period_den = kPeriodDen;
  c.param_schema_hash = marv::kParamSchemaHash;
  return c;
}

void reach_ready() {
  const marv_sil_config cfg = make_config();
  ASSERT_EQ(marv_sil_init(&cfg), MARV_SIL_OK);
}

std::uint64_t stamp_of(std::uint64_t tick) {
  return static_cast<std::uint64_t>(static_cast<unsigned __int128>(tick) * kPeriodNumUs / kPeriodDen);
}

// Field i of the state: 0 z_ned_m, 1 vz_ned_m_s.
float& field(marv_truth_vertical& s, std::size_t i) { return i == 0 ? s.z_ned_m : s.vz_ned_m_s; }

// A state that differs for every i; z is negative (above the NED origin), v_z of both signs.
marv_truth_vertical valid_state(std::uint64_t tick, std::uint32_t i = 0) {
  const auto f = static_cast<float>(i);
  marv_truth_vertical s{};
  s.struct_size = sizeof(marv_truth_vertical);
  s.flags = kValid;
  s.tick = tick;
  s.z_ned_m = -1.5f - f;
  s.vz_ned_m_s = 0.25f - 0.125f * f;
  return s;
}

marv_truth_vertical cleared_state(std::uint64_t tick) {
  marv_truth_vertical s{};
  s.struct_size = sizeof(marv_truth_vertical);
  s.tick = tick;
  return s;
}

marv::VerticalState<float> expected_hand_over(const marv_truth_vertical& s) {
  return marv::VerticalState<float>{stamp_of(s.tick), s.z_ned_m, s.vz_ned_m_s, (s.flags & kValid) != 0};
}

struct TickResult {
  marv_sil_status status;
  std::uint64_t t_us;
  std::array<std::uint16_t, kMotors> dshot;
};

// One marv_sil_tick(tick, 1, ...) with an all-zero IMU sample (valid under the IMU rule).
TickResult run_tick(std::uint64_t tick) {
  const marv_imu_meas imu{};
  TickResult r{};
  marv_sil_out out{};
  out.struct_size = sizeof(marv_sil_out);
  out.capacity_ticks = 1;
  out.t_us = &r.t_us;
  out.dshot = r.dshot.data();
  out.servo_us = nullptr;
  r.status = marv_sil_tick(tick, 1, &imu, &out);
  return r;
}

// What the test composition reports for a tick: `calls` hand-overs since the previous tick, then the latched state.
void expect_reported(const TickResult& r, std::uint64_t tick, std::uint32_t calls,
                     const marv_truth_vertical* latched) {
  ASSERT_EQ(r.status, MARV_SIL_OK);
  EXPECT_EQ(r.t_us, stamp_of(tick));
  std::uint32_t flags = 0;
  std::uint32_t hash = 0;
  if (latched != nullptr) {
    const marv::VerticalState<float> v = expected_hand_over(*latched);
    flags = verticaltest::kFlagHave | (v.valid ? verticaltest::kFlagValid : 0u) |
            (v.t_us == stamp_of(tick) ? verticaltest::kFlagStampMatches : 0u);
    hash = verticaltest::hash_of(v);
  }
  const verticaltest::Row want = verticaltest::row_of(calls, flags, hash);
  EXPECT_EQ(r.dshot[0], want.calls) << "calls of vertical_input since the previous tick";
  EXPECT_EQ(r.dshot[1], want.flags) << "flags of the latched state";
  EXPECT_EQ(r.dshot[2], want.hash_lo) << "latched state";
  EXPECT_EQ(r.dshot[3], want.hash_hi) << "latched state";
}

// A rejected set must not reach the composition: the next tick reports zero hand-overs.
void expect_no_hand_over_at(std::uint64_t tick) { expect_reported(run_tick(tick), tick, 0, nullptr); }

TEST(TruthVertical, EStateBeforeInit) {
  in_child([] {
    const marv_truth_vertical s = valid_state(0);
    EXPECT_EQ(marv_truth_vertical_set(&s), MARV_SIL_E_STATE);
    EXPECT_EQ(marv_truth_vertical_set(nullptr), MARV_SIL_E_STATE);  // the state is checked before the pointer
  });
}

TEST(TruthVertical, EStateAfterShutdown) {
  in_child([] {
    reach_ready();
    ASSERT_EQ(marv_sil_shutdown(), MARV_SIL_OK);
    const marv_truth_vertical s = valid_state(0);
    EXPECT_EQ(marv_truth_vertical_set(&s), MARV_SIL_E_STATE);
  });
}

TEST(TruthVertical, ENullForANullPointer) {
  in_child([] {
    reach_ready();
    EXPECT_EQ(marv_truth_vertical_set(nullptr), MARV_SIL_E_NULL);
    expect_no_hand_over_at(0);
  });
}

TEST(TruthVertical, EAbiForAnyOtherStructSize) {
  in_child([] {
    reach_ready();
    for (const std::uint32_t size : {0u, static_cast<std::uint32_t>(sizeof(marv_truth_vertical)) - 1u,
                                     static_cast<std::uint32_t>(sizeof(marv_truth_vertical)) + 1u, 0xffffffffu}) {
      marv_truth_vertical s = valid_state(0);
      s.struct_size = size;
      EXPECT_EQ(marv_truth_vertical_set(&s), MARV_SIL_E_ABI) << size;
    }
    expect_no_hand_over_at(0);
  });
}

TEST(TruthVertical, EAbiIsCheckedBeforeTheTick) {
  in_child([] {
    reach_ready();
    marv_truth_vertical s = valid_state(7);
    s.struct_size = 1;
    EXPECT_EQ(marv_truth_vertical_set(&s), MARV_SIL_E_ABI);
  });
}

TEST(TruthVertical, ETickUnlessTheTickIsTheNumberOfTicksAlreadyRun) {
  in_child([] {
    reach_ready();
    for (const std::uint64_t tick : {std::uint64_t{1}, std::uint64_t{2}, std::numeric_limits<std::uint64_t>::max()}) {
      const marv_truth_vertical s = valid_state(tick);
      EXPECT_EQ(marv_truth_vertical_set(&s), MARV_SIL_E_TICK) << tick;
    }
    expect_no_hand_over_at(0);

    // One tick has run: the next is tick 1; 0 (already run) and 2 (skipping one) are refused.
    for (const std::uint64_t tick : {std::uint64_t{0}, std::uint64_t{2}}) {
      const marv_truth_vertical s = valid_state(tick);
      EXPECT_EQ(marv_truth_vertical_set(&s), MARV_SIL_E_TICK) << tick;
    }
    expect_no_hand_over_at(1);
    const marv_truth_vertical ok = valid_state(2);
    EXPECT_EQ(marv_truth_vertical_set(&ok), MARV_SIL_OK);
  });
}

TEST(TruthVertical, ETickFollowsTheTicksRunByAMultiTickCall) {
  in_child([] {
    reach_ready();
    constexpr std::uint32_t k = 3;
    const std::array<marv_imu_meas, k> imu{};
    std::array<std::uint64_t, k> t{};
    std::array<std::uint16_t, k * kMotors> dshot{};
    marv_sil_out out{};
    out.struct_size = sizeof(marv_sil_out);
    out.capacity_ticks = k;
    out.t_us = t.data();
    out.dshot = dshot.data();
    ASSERT_EQ(marv_sil_tick(0, k, imu.data(), &out), MARV_SIL_OK);
    const marv_truth_vertical stale = valid_state(k - 1);
    EXPECT_EQ(marv_truth_vertical_set(&stale), MARV_SIL_E_TICK);
    const marv_truth_vertical next = valid_state(k);
    EXPECT_EQ(marv_truth_vertical_set(&next), MARV_SIL_OK);
  });
}

TEST(TruthVertical, EInputForAReservedFlagBit) {
  in_child([] {
    reach_ready();
    for (unsigned bit = MARV_TRUTH_VERTICAL_FLAG_COUNT; bit < 32; ++bit) {
      marv_truth_vertical valid = valid_state(0);
      valid.flags |= 1u << bit;
      EXPECT_EQ(marv_truth_vertical_set(&valid), MARV_SIL_E_INPUT) << "valid set, bit " << bit;
      marv_truth_vertical cleared = cleared_state(0);
      cleared.flags |= 1u << bit;
      EXPECT_EQ(marv_truth_vertical_set(&cleared), MARV_SIL_E_INPUT) << "valid clear, bit " << bit;
    }
    expect_no_hand_over_at(0);
  });
}

TEST(TruthVertical, EInputForAnyNonzeroFieldWithTheValidBitClear) {
  in_child([] {
    reach_ready();
    for (std::size_t i = 0; i < kFieldCount; ++i) {
      for (const float v : {1.0f, -1.0f, std::numeric_limits<float>::denorm_min(), kNan, kInf, -kInf}) {
        marv_truth_vertical s = cleared_state(0);
        field(s, i) = v;
        EXPECT_EQ(marv_truth_vertical_set(&s), MARV_SIL_E_INPUT) << "field " << i << " value " << v;
      }
    }
    expect_no_hand_over_at(0);
  });
}

TEST(TruthVertical, EInputForANonFiniteFieldWithTheValidBitSet) {
  in_child([] {
    reach_ready();
    for (std::size_t i = 0; i < kFieldCount; ++i) {
      for (const float v : {kNan, -kNan, kInf, -kInf}) {
        marv_truth_vertical s = valid_state(0);
        field(s, i) = v;
        EXPECT_EQ(marv_truth_vertical_set(&s), MARV_SIL_E_INPUT) << "field " << i << " value " << v;
      }
    }
    expect_no_hand_over_at(0);
  });
}

TEST(TruthVertical, OkWithTheValidBitSetAndZeroFieldsHandsOverAValidState) {
  in_child([] {
    reach_ready();
    // At the NED origin and at rest is a state like any other: the zero-quaternion rule has no vertical counterpart.
    marv_truth_vertical s = valid_state(0);
    s.z_ned_m = 0.0f;
    s.vz_ned_m_s = -0.0f;
    ASSERT_EQ(marv_truth_vertical_set(&s), MARV_SIL_OK);
    expect_reported(run_tick(0), 0, 1, &s);
  });
}

TEST(TruthVertical, StatusesAreCheckedInTheDocumentedOrder) {
  in_child([] {
    reach_ready();
    marv_truth_vertical s = valid_state(5);  // E_TICK wins over E_INPUT
    s.z_ned_m = kNan;
    EXPECT_EQ(marv_truth_vertical_set(&s), MARV_SIL_E_TICK);
    s.tick = 0;
    EXPECT_EQ(marv_truth_vertical_set(&s), MARV_SIL_E_INPUT);
  });
}

TEST(TruthVertical, OkHandsOverTheStateWithTheSilStampOfThatTick) {
  in_child([] {
    reach_ready();
    const marv_truth_vertical s = valid_state(0, 3);
    ASSERT_EQ(marv_truth_vertical_set(&s), MARV_SIL_OK);
    expect_reported(run_tick(0), 0, 1, &s);
  });
}

TEST(TruthVertical, OkHandsOverEveryTickInTheCallOrderOfTheLoop) {
  in_child([] {
    reach_ready();
    constexpr std::uint32_t ticks = 9;  // the stamp is not a multiple of the period at most ticks (625/4 us)
    for (std::uint32_t j = 0; j < ticks; ++j) {
      const marv_truth_vertical s = valid_state(j, j);
      ASSERT_EQ(marv_truth_vertical_set(&s), MARV_SIL_OK) << "tick " << j;
      expect_reported(run_tick(j), j, 1, &s);
    }
  });
}

TEST(TruthVertical, OkWithTheValidBitClearAndZeroFieldsHandsOverAnInvalidState) {
  in_child([] {
    reach_ready();
    marv_truth_vertical s = cleared_state(0);
    s.z_ned_m = -0.0f;  // negative zero is zero
    ASSERT_EQ(marv_truth_vertical_set(&s), MARV_SIL_OK);
    expect_reported(run_tick(0), 0, 1, &s);
  });
}

TEST(TruthVertical, ASecondSetForTheSameTickReplacesTheFirst) {
  in_child([] {
    reach_ready();
    const marv_truth_vertical first = valid_state(0, 1);
    const marv_truth_vertical second = valid_state(0, 2);
    ASSERT_EQ(marv_truth_vertical_set(&first), MARV_SIL_OK);
    ASSERT_EQ(marv_truth_vertical_set(&second), MARV_SIL_OK);
    expect_reported(run_tick(0), 0, 2, &second);

    // Replacement by an invalid-flag state is a replacement too.
    const marv_truth_vertical third = valid_state(1, 3);
    const marv_truth_vertical fourth = cleared_state(1);
    ASSERT_EQ(marv_truth_vertical_set(&third), MARV_SIL_OK);
    ASSERT_EQ(marv_truth_vertical_set(&fourth), MARV_SIL_OK);
    expect_reported(run_tick(1), 1, 2, &fourth);
  });
}

TEST(TruthVertical, ARejectedSecondSetLeavesTheFirstInPlace) {
  in_child([] {
    reach_ready();
    const marv_truth_vertical first = valid_state(0, 1);
    marv_truth_vertical bad = valid_state(0, 2);
    bad.vz_ned_m_s = kNan;
    ASSERT_EQ(marv_truth_vertical_set(&first), MARV_SIL_OK);
    ASSERT_EQ(marv_truth_vertical_set(&bad), MARV_SIL_E_INPUT);
    expect_reported(run_tick(0), 0, 1, &first);
  });
}

TEST(TruthVertical, SetDoesNotAdvanceTheTickCounter) {
  in_child([] {
    reach_ready();
    const marv_truth_vertical s = valid_state(0);
    ASSERT_EQ(marv_truth_vertical_set(&s), MARV_SIL_OK);
    ASSERT_EQ(marv_truth_vertical_set(&s), MARV_SIL_OK);
    const TickResult wrong = run_tick(1);
    EXPECT_EQ(wrong.status, MARV_SIL_E_TICK);
    expect_reported(run_tick(0), 0, 2, &s);
  });
}

TEST(TruthVertical, AStaleLatchShowsAsAStampMismatchToTheComposition) {
  in_child([] {
    reach_ready();
    const marv_truth_vertical s = valid_state(0);
    ASSERT_EQ(marv_truth_vertical_set(&s), MARV_SIL_OK);
    expect_reported(run_tick(0), 0, 1, &s);
    // The test composition keeps its latch; the freshness rule (stamp equality) is the composition's, and it sees the
    // stale stamp here.
    const TickResult r = run_tick(1);
    ASSERT_EQ(r.status, MARV_SIL_OK);
    EXPECT_EQ(r.dshot[0], verticaltest::code(0));
    EXPECT_EQ(r.dshot[1], verticaltest::code(verticaltest::kFlagHave | verticaltest::kFlagValid));
  });
}

TEST(TruthVertical, StatusStringsOfTheCodesItUses) {
  for (const marv_sil_status s : {MARV_SIL_E_STATE, MARV_SIL_E_NULL, MARV_SIL_E_ABI, MARV_SIL_E_TICK, MARV_SIL_E_INPUT}) {
    const char* text = marv_sil_status_str(s);
    ASSERT_NE(text, nullptr);
    EXPECT_STRNE(text, "unknown status") << s;
  }
}

}  // namespace
