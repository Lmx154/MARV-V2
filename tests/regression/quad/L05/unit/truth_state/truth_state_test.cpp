// T1 of marv_truth_state_set: status codes, replace-on-second-set, hand-over. marv_sil_init succeeds once per process,
// so every test body runs in a forked child (the parent never initialises).
#include <gtest/gtest.h>

#include <marv_sil.h>
#include <marv_truth.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>

#include <marv/composition.hpp>
#include <marv/params/param_ids.hpp>
#include <marv/types/attitude_state.hpp>

#include "probe_codes.hpp"

namespace {

constexpr std::uint32_t kPeriodNumUs = 625;
constexpr std::uint32_t kPeriodDen = 4;
constexpr std::size_t kMotors = marv::composition::kMotors;
constexpr std::uint32_t kValid = 1u << MARV_TRUTH_ATTITUDE_VALID;
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

// A state that differs for every i; the quaternion is deliberately not of unit norm (the entry does not normalise).
marv_truth_state valid_state(std::uint64_t tick, std::uint32_t i = 0) {
  const auto f = static_cast<float>(i);
  marv_truth_state s{};
  s.struct_size = sizeof(marv_truth_state);
  s.flags = kValid;
  s.tick = tick;
  s.q_wxyz[MARV_TRUTH_Q_W] = 0.5f + f;
  s.q_wxyz[MARV_TRUTH_Q_X] = -0.25f;
  s.q_wxyz[MARV_TRUTH_Q_Y] = 0.125f * f;
  s.q_wxyz[MARV_TRUTH_Q_Z] = -2.0f;
  s.omega_frd_rad_s = {0.5f * f, -1.5f, 0.0625f + f};
  return s;
}

marv_truth_state cleared_state(std::uint64_t tick) {
  marv_truth_state s{};
  s.struct_size = sizeof(marv_truth_state);
  s.tick = tick;
  return s;
}

marv::AttitudeState<float> expected_hand_over(const marv_truth_state& s) {
  return marv::AttitudeState<float>{
      stamp_of(s.tick),
      marv::prim::Quat<float>{s.q_wxyz[MARV_TRUTH_Q_W], s.q_wxyz[MARV_TRUTH_Q_X], s.q_wxyz[MARV_TRUTH_Q_Y],
                              s.q_wxyz[MARV_TRUTH_Q_Z]},
      marv::prim::Vec3<float>{s.omega_frd_rad_s.x, s.omega_frd_rad_s.y, s.omega_frd_rad_s.z},
      (s.flags & kValid) != 0};
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
void expect_reported(const TickResult& r, std::uint64_t tick, std::uint32_t calls, const marv_truth_state* latched) {
  ASSERT_EQ(r.status, MARV_SIL_OK);
  EXPECT_EQ(r.t_us, stamp_of(tick));
  std::uint32_t flags = 0;
  std::uint32_t hash = 0;
  if (latched != nullptr) {
    const marv::AttitudeState<float> a = expected_hand_over(*latched);
    flags = truthtest::kFlagHave | (a.valid ? truthtest::kFlagValid : 0u) |
            (a.t_us == stamp_of(tick) ? truthtest::kFlagStampMatches : 0u);
    hash = truthtest::hash_of(a);
  }
  const truthtest::Row want = truthtest::row_of(calls, flags, hash);
  EXPECT_EQ(r.dshot[0], want.calls) << "calls of attitude_input since the previous tick";
  EXPECT_EQ(r.dshot[1], want.flags) << "flags of the latched state";
  EXPECT_EQ(r.dshot[2], want.hash_lo) << "latched state";
  EXPECT_EQ(r.dshot[3], want.hash_hi) << "latched state";
}

// A rejected set must not reach the composition: the next tick reports zero hand-overs.
void expect_no_hand_over_at(std::uint64_t tick) { expect_reported(run_tick(tick), tick, 0, nullptr); }

TEST(TruthState, EStateBeforeInit) {
  in_child([] {
    const marv_truth_state s = valid_state(0);
    EXPECT_EQ(marv_truth_state_set(&s), MARV_SIL_E_STATE);
    EXPECT_EQ(marv_truth_state_set(nullptr), MARV_SIL_E_STATE);  // the state is checked before the pointer
  });
}

TEST(TruthState, EStateAfterShutdown) {
  in_child([] {
    reach_ready();
    ASSERT_EQ(marv_sil_shutdown(), MARV_SIL_OK);
    const marv_truth_state s = valid_state(0);
    EXPECT_EQ(marv_truth_state_set(&s), MARV_SIL_E_STATE);
  });
}

TEST(TruthState, ENullForANullPointer) {
  in_child([] {
    reach_ready();
    EXPECT_EQ(marv_truth_state_set(nullptr), MARV_SIL_E_NULL);
    expect_no_hand_over_at(0);
  });
}

TEST(TruthState, EAbiForAnyOtherStructSize) {
  in_child([] {
    reach_ready();
    for (const std::uint32_t size : {0u, static_cast<std::uint32_t>(sizeof(marv_truth_state)) - 1u,
                                     static_cast<std::uint32_t>(sizeof(marv_truth_state)) + 1u, 0xffffffffu}) {
      marv_truth_state s = valid_state(0);
      s.struct_size = size;
      EXPECT_EQ(marv_truth_state_set(&s), MARV_SIL_E_ABI) << size;
    }
    expect_no_hand_over_at(0);
  });
}

TEST(TruthState, EAbiIsCheckedBeforeTheTick) {
  in_child([] {
    reach_ready();
    marv_truth_state s = valid_state(7);
    s.struct_size = 1;
    EXPECT_EQ(marv_truth_state_set(&s), MARV_SIL_E_ABI);
  });
}

TEST(TruthState, ETickUnlessTheTickIsTheNumberOfTicksAlreadyRun) {
  in_child([] {
    reach_ready();
    for (const std::uint64_t tick : {std::uint64_t{1}, std::uint64_t{2}, std::numeric_limits<std::uint64_t>::max()}) {
      const marv_truth_state s = valid_state(tick);
      EXPECT_EQ(marv_truth_state_set(&s), MARV_SIL_E_TICK) << tick;
    }
    expect_no_hand_over_at(0);

    // One tick has run: the next is tick 1; 0 (already run) and 2 (skipping one) are refused.
    for (const std::uint64_t tick : {std::uint64_t{0}, std::uint64_t{2}}) {
      const marv_truth_state s = valid_state(tick);
      EXPECT_EQ(marv_truth_state_set(&s), MARV_SIL_E_TICK) << tick;
    }
    expect_no_hand_over_at(1);
    const marv_truth_state ok = valid_state(2);
    EXPECT_EQ(marv_truth_state_set(&ok), MARV_SIL_OK);
  });
}

TEST(TruthState, ETickFollowsTheTicksRunByAMultiTickCall) {
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
    const marv_truth_state stale = valid_state(k - 1);
    EXPECT_EQ(marv_truth_state_set(&stale), MARV_SIL_E_TICK);
    const marv_truth_state next = valid_state(k);
    EXPECT_EQ(marv_truth_state_set(&next), MARV_SIL_OK);
  });
}

TEST(TruthState, EInputForAReservedFlagBit) {
  in_child([] {
    reach_ready();
    for (unsigned bit = MARV_TRUTH_FLAG_COUNT; bit < 32; ++bit) {
      marv_truth_state valid = valid_state(0);
      valid.flags |= 1u << bit;
      EXPECT_EQ(marv_truth_state_set(&valid), MARV_SIL_E_INPUT) << "valid set, bit " << bit;
      marv_truth_state cleared = cleared_state(0);
      cleared.flags |= 1u << bit;
      EXPECT_EQ(marv_truth_state_set(&cleared), MARV_SIL_E_INPUT) << "valid clear, bit " << bit;
    }
    expect_no_hand_over_at(0);
  });
}

TEST(TruthState, EInputForAnyNonzeroFieldWithTheValidBitClear) {
  in_child([] {
    reach_ready();
    for (std::size_t i = 0; i < MARV_TRUTH_Q_COUNT + 3; ++i) {
      for (const float v : {1.0f, -1.0f, kNan, kInf}) {
        marv_truth_state s = cleared_state(0);
        if (i < MARV_TRUTH_Q_COUNT) {
          s.q_wxyz[i] = v;
        } else {
          (i == MARV_TRUTH_Q_COUNT ? s.omega_frd_rad_s.x : i == MARV_TRUTH_Q_COUNT + 1 ? s.omega_frd_rad_s.y
                                                                                        : s.omega_frd_rad_s.z) = v;
        }
        EXPECT_EQ(marv_truth_state_set(&s), MARV_SIL_E_INPUT) << "field " << i << " value " << v;
      }
    }
    expect_no_hand_over_at(0);
  });
}

TEST(TruthState, EInputForANonFiniteFieldWithTheValidBitSet) {
  in_child([] {
    reach_ready();
    for (std::size_t i = 0; i < MARV_TRUTH_Q_COUNT + 3; ++i) {
      for (const float v : {kNan, kInf, -kInf}) {
        marv_truth_state s = valid_state(0);
        if (i < MARV_TRUTH_Q_COUNT) {
          s.q_wxyz[i] = v;
        } else {
          (i == MARV_TRUTH_Q_COUNT ? s.omega_frd_rad_s.x : i == MARV_TRUTH_Q_COUNT + 1 ? s.omega_frd_rad_s.y
                                                                                        : s.omega_frd_rad_s.z) = v;
        }
        EXPECT_EQ(marv_truth_state_set(&s), MARV_SIL_E_INPUT) << "field " << i << " value " << v;
      }
    }
    expect_no_hand_over_at(0);
  });
}

TEST(TruthState, EInputForAZeroQuaternionWithTheValidBitSet) {
  in_child([] {
    reach_ready();
    marv_truth_state s = valid_state(0);
    for (std::size_t i = 0; i < MARV_TRUTH_Q_COUNT; ++i) {
      s.q_wxyz[i] = i == 0 ? -0.0f : 0.0f;
    }
    EXPECT_EQ(marv_truth_state_set(&s), MARV_SIL_E_INPUT);
    expect_no_hand_over_at(0);
  });
}

TEST(TruthState, ATinyQuaternionIsNotZero) {
  in_child([] {
    reach_ready();
    marv_truth_state s = valid_state(0);
    for (std::size_t i = 0; i < MARV_TRUTH_Q_COUNT; ++i) {
      s.q_wxyz[i] = 0.0f;
    }
    s.q_wxyz[MARV_TRUTH_Q_Z] = std::numeric_limits<float>::denorm_min();
    EXPECT_EQ(marv_truth_state_set(&s), MARV_SIL_OK);
  });
}

TEST(TruthState, StatusesAreCheckedInTheDocumentedOrder) {
  in_child([] {
    reach_ready();
    marv_truth_state s = valid_state(5);  // E_TICK wins over E_INPUT
    s.q_wxyz[MARV_TRUTH_Q_W] = kNan;
    EXPECT_EQ(marv_truth_state_set(&s), MARV_SIL_E_TICK);
    s.tick = 0;
    EXPECT_EQ(marv_truth_state_set(&s), MARV_SIL_E_INPUT);
  });
}

TEST(TruthState, OkHandsOverTheStateWithTheSilStampOfThatTick) {
  in_child([] {
    reach_ready();
    const marv_truth_state s = valid_state(0, 3);
    ASSERT_EQ(marv_truth_state_set(&s), MARV_SIL_OK);
    expect_reported(run_tick(0), 0, 1, &s);
  });
}

TEST(TruthState, OkHandsOverEveryTickInTheCallOrderOfTheLoop) {
  in_child([] {
    reach_ready();
    constexpr std::uint32_t ticks = 9;  // the stamp is not a multiple of the period at most ticks (625/4 us)
    for (std::uint32_t j = 0; j < ticks; ++j) {
      const marv_truth_state s = valid_state(j, j);
      ASSERT_EQ(marv_truth_state_set(&s), MARV_SIL_OK) << "tick " << j;
      expect_reported(run_tick(j), j, 1, &s);
    }
  });
}

TEST(TruthState, OkWithTheValidBitClearAndZeroFieldsHandsOverAnInvalidState) {
  in_child([] {
    reach_ready();
    marv_truth_state s = cleared_state(0);
    s.q_wxyz[MARV_TRUTH_Q_W] = -0.0f;  // negative zero is zero
    ASSERT_EQ(marv_truth_state_set(&s), MARV_SIL_OK);
    expect_reported(run_tick(0), 0, 1, &s);
  });
}

TEST(TruthState, ASecondSetForTheSameTickReplacesTheFirst) {
  in_child([] {
    reach_ready();
    const marv_truth_state first = valid_state(0, 1);
    const marv_truth_state second = valid_state(0, 2);
    ASSERT_EQ(marv_truth_state_set(&first), MARV_SIL_OK);
    ASSERT_EQ(marv_truth_state_set(&second), MARV_SIL_OK);
    expect_reported(run_tick(0), 0, 2, &second);

    // Replacement by an invalid-flag state is a replacement too.
    const marv_truth_state third = valid_state(1, 3);
    const marv_truth_state fourth = cleared_state(1);
    ASSERT_EQ(marv_truth_state_set(&third), MARV_SIL_OK);
    ASSERT_EQ(marv_truth_state_set(&fourth), MARV_SIL_OK);
    expect_reported(run_tick(1), 1, 2, &fourth);
  });
}

TEST(TruthState, ARejectedSecondSetLeavesTheFirstInPlace) {
  in_child([] {
    reach_ready();
    const marv_truth_state first = valid_state(0, 1);
    marv_truth_state bad = valid_state(0, 2);
    bad.q_wxyz[MARV_TRUTH_Q_X] = kNan;
    ASSERT_EQ(marv_truth_state_set(&first), MARV_SIL_OK);
    ASSERT_EQ(marv_truth_state_set(&bad), MARV_SIL_E_INPUT);
    expect_reported(run_tick(0), 0, 1, &first);
  });
}

TEST(TruthState, SetDoesNotAdvanceTheTickCounter) {
  in_child([] {
    reach_ready();
    const marv_truth_state s = valid_state(0);
    ASSERT_EQ(marv_truth_state_set(&s), MARV_SIL_OK);
    ASSERT_EQ(marv_truth_state_set(&s), MARV_SIL_OK);
    const TickResult wrong = run_tick(1);
    EXPECT_EQ(wrong.status, MARV_SIL_E_TICK);
    expect_reported(run_tick(0), 0, 2, &s);
  });
}

TEST(TruthState, AStaleLatchShowsAsAStampMismatchToTheComposition) {
  in_child([] {
    reach_ready();
    const marv_truth_state s = valid_state(0);
    ASSERT_EQ(marv_truth_state_set(&s), MARV_SIL_OK);
    expect_reported(run_tick(0), 0, 1, &s);
    // The test composition keeps its latch; the freshness rule (stamp equality) is the composition's, and it sees the
    // stale stamp here.
    const TickResult r = run_tick(1);
    ASSERT_EQ(r.status, MARV_SIL_OK);
    EXPECT_EQ(r.dshot[0], truthtest::code(0));
    EXPECT_EQ(r.dshot[1], truthtest::code(truthtest::kFlagHave | truthtest::kFlagValid));
  });
}

TEST(TruthState, StatusStringsOfTheCodesItUses) {
  for (const marv_sil_status s : {MARV_SIL_E_STATE, MARV_SIL_E_NULL, MARV_SIL_E_ABI, MARV_SIL_E_TICK, MARV_SIL_E_INPUT}) {
    const char* text = marv_sil_status_str(s);
    ASSERT_NE(text, nullptr);
    EXPECT_STRNE(text, "unknown status") << s;
  }
}

}  // namespace
