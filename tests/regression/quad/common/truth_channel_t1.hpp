// The T1 helpers and shared test bodies of a truth channel (state, vertical), written once.
// Consolidated from the two suites, decision 0024.
#pragma once

#include <gtest/gtest.h>

#include <marv_sil.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>

#include <marv/composition.hpp>
#include <marv/params/param_ids.hpp>

namespace truth_channel_t1 {

inline constexpr std::uint32_t kPeriodNumUs = 625;
inline constexpr std::uint32_t kPeriodDen = 4;
inline constexpr std::size_t kMotors = marv::composition::kMotors;
inline constexpr float kNan = std::numeric_limits<float>::quiet_NaN();
inline constexpr float kInf = std::numeric_limits<float>::infinity();

inline bool report_failures_on_stderr() {
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

inline marv_sil_config make_config() {
  marv_sil_config c{};
  c.struct_size = sizeof(marv_sil_config);
  c.imu_meas_size = sizeof(marv_imu_meas);
  c.override_size = sizeof(marv_sil_param_override);
  c.tick_period_num_us = kPeriodNumUs;
  c.tick_period_den = kPeriodDen;
  c.param_schema_hash = marv::kParamSchemaHash;
  return c;
}

inline void reach_ready() {
  const marv_sil_config cfg = make_config();
  ASSERT_EQ(marv_sil_init(&cfg), MARV_SIL_OK);
}

inline std::uint64_t stamp_of(std::uint64_t tick) {
  return static_cast<std::uint64_t>(static_cast<unsigned __int128>(tick) * kPeriodNumUs / kPeriodDen);
}

struct TickResult {
  marv_sil_status status;
  std::uint64_t t_us;
  std::array<std::uint16_t, kMotors> dshot;
};

// One marv_sil_tick(tick, 1, ...) with an all-zero IMU sample (valid under the IMU rule).
inline TickResult run_tick(std::uint64_t tick) {
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
template <class C>
void expect_reported(const TickResult& r, std::uint64_t tick, std::uint32_t calls, const typename C::State* latched) {
  ASSERT_EQ(r.status, MARV_SIL_OK);
  EXPECT_EQ(r.t_us, stamp_of(tick));
  std::uint32_t flags = 0;
  std::uint32_t hash = 0;
  if (latched != nullptr) {
    const auto a = C::expected_hand_over(*latched);
    flags = C::kFlagHave | (a.valid ? C::kFlagValid : 0u) | (a.t_us == stamp_of(tick) ? C::kFlagStampMatches : 0u);
    hash = C::hash_of(a);
  }
  const typename C::Row want = C::row_of(calls, flags, hash);
  EXPECT_EQ(r.dshot[0], want.calls) << "calls of " << C::kInputName << " since the previous tick";
  EXPECT_EQ(r.dshot[1], want.flags) << "flags of the latched state";
  EXPECT_EQ(r.dshot[2], want.hash_lo) << "latched state";
  EXPECT_EQ(r.dshot[3], want.hash_hi) << "latched state";
}

// A rejected set must not reach the composition: the next tick reports zero hand-overs.
template <class C>
void expect_no_hand_over_at(std::uint64_t tick) {
  expect_reported<C>(run_tick(tick), tick, 0, nullptr);
}

template <class C>
void EStateBeforeInit() {
  const typename C::State s = C::valid_state(0);
  EXPECT_EQ(C::set(&s), MARV_SIL_E_STATE);
  EXPECT_EQ(C::set(nullptr), MARV_SIL_E_STATE);  // the state is checked before the pointer
}

template <class C>
void EStateAfterShutdown() {
  reach_ready();
  ASSERT_EQ(marv_sil_shutdown(), MARV_SIL_OK);
  const typename C::State s = C::valid_state(0);
  EXPECT_EQ(C::set(&s), MARV_SIL_E_STATE);
}

template <class C>
void ENullForANullPointer() {
  reach_ready();
  EXPECT_EQ(C::set(nullptr), MARV_SIL_E_NULL);
  expect_no_hand_over_at<C>(0);
}

template <class C>
void EAbiForAnyOtherStructSize() {
  using S = typename C::State;
  reach_ready();
  for (const std::uint32_t size : {0u, static_cast<std::uint32_t>(sizeof(S)) - 1u,
                                   static_cast<std::uint32_t>(sizeof(S)) + 1u, 0xffffffffu}) {
    S s = C::valid_state(0);
    s.struct_size = size;
    EXPECT_EQ(C::set(&s), MARV_SIL_E_ABI) << size;
  }
  expect_no_hand_over_at<C>(0);
}

template <class C>
void EAbiIsCheckedBeforeTheTick() {
  reach_ready();
  typename C::State s = C::valid_state(7);
  s.struct_size = 1;
  EXPECT_EQ(C::set(&s), MARV_SIL_E_ABI);
}

template <class C>
void ETickUnlessTheTickIsTheNumberOfTicksAlreadyRun() {
  using S = typename C::State;
  reach_ready();
  for (const std::uint64_t tick : {std::uint64_t{1}, std::uint64_t{2}, std::numeric_limits<std::uint64_t>::max()}) {
    const S s = C::valid_state(tick);
    EXPECT_EQ(C::set(&s), MARV_SIL_E_TICK) << tick;
  }
  expect_no_hand_over_at<C>(0);

  // One tick has run: the next is tick 1; 0 (already run) and 2 (skipping one) are refused.
  for (const std::uint64_t tick : {std::uint64_t{0}, std::uint64_t{2}}) {
    const S s = C::valid_state(tick);
    EXPECT_EQ(C::set(&s), MARV_SIL_E_TICK) << tick;
  }
  expect_no_hand_over_at<C>(1);
  const S ok = C::valid_state(2);
  EXPECT_EQ(C::set(&ok), MARV_SIL_OK);
}

template <class C>
void ETickFollowsTheTicksRunByAMultiTickCall() {
  using S = typename C::State;
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
  const S stale = C::valid_state(k - 1);
  EXPECT_EQ(C::set(&stale), MARV_SIL_E_TICK);
  const S next = C::valid_state(k);
  EXPECT_EQ(C::set(&next), MARV_SIL_OK);
}

template <class C>
void EInputForAReservedFlagBit() {
  using S = typename C::State;
  reach_ready();
  for (unsigned bit = C::kFlagCount; bit < 32; ++bit) {
    S valid = C::valid_state(0);
    valid.flags |= 1u << bit;
    EXPECT_EQ(C::set(&valid), MARV_SIL_E_INPUT) << "valid set, bit " << bit;
    S cleared = C::cleared_state(0);
    cleared.flags |= 1u << bit;
    EXPECT_EQ(C::set(&cleared), MARV_SIL_E_INPUT) << "valid clear, bit " << bit;
  }
  expect_no_hand_over_at<C>(0);
}

template <class C>
void EInputForAnyNonzeroFieldWithTheValidBitClear() {
  using S = typename C::State;
  reach_ready();
  for (std::size_t i = 0; i < C::kFieldCount; ++i) {
    for (const float v : {1.0f, -1.0f, std::numeric_limits<float>::denorm_min(), kNan, kInf, -kInf}) {
      S s = C::cleared_state(0);
      C::field(s, i) = v;
      EXPECT_EQ(C::set(&s), MARV_SIL_E_INPUT) << "field " << i << " value " << v;
    }
  }
  expect_no_hand_over_at<C>(0);
}

template <class C>
void EInputForANonFiniteFieldWithTheValidBitSet() {
  using S = typename C::State;
  reach_ready();
  for (std::size_t i = 0; i < C::kFieldCount; ++i) {
    for (const float v : {kNan, -kNan, kInf, -kInf}) {
      S s = C::valid_state(0);
      C::field(s, i) = v;
      EXPECT_EQ(C::set(&s), MARV_SIL_E_INPUT) << "field " << i << " value " << v;
    }
  }
  expect_no_hand_over_at<C>(0);
}

template <class C>
void StatusesAreCheckedInTheDocumentedOrder() {
  reach_ready();
  typename C::State s = C::valid_state(5);  // E_TICK wins over E_INPUT
  C::field(s, 0) = kNan;
  EXPECT_EQ(C::set(&s), MARV_SIL_E_TICK);
  s.tick = 0;
  EXPECT_EQ(C::set(&s), MARV_SIL_E_INPUT);
}

template <class C>
void OkHandsOverTheStateWithTheSilStampOfThatTick() {
  reach_ready();
  const typename C::State s = C::valid_state(0, 3);
  ASSERT_EQ(C::set(&s), MARV_SIL_OK);
  expect_reported<C>(run_tick(0), 0, 1, &s);
}

template <class C>
void OkHandsOverEveryTickInTheCallOrderOfTheLoop() {
  reach_ready();
  constexpr std::uint32_t ticks = 9;  // the stamp is not a multiple of the period at most ticks (625/4 us)
  for (std::uint32_t j = 0; j < ticks; ++j) {
    const typename C::State s = C::valid_state(j, j);
    ASSERT_EQ(C::set(&s), MARV_SIL_OK) << "tick " << j;
    expect_reported<C>(run_tick(j), j, 1, &s);
  }
}

template <class C>
void OkWithTheValidBitClearAndZeroFieldsHandsOverAnInvalidState() {
  reach_ready();
  typename C::State s = C::cleared_state(0);
  C::field(s, 0) = -0.0f;  // negative zero is zero
  ASSERT_EQ(C::set(&s), MARV_SIL_OK);
  expect_reported<C>(run_tick(0), 0, 1, &s);
}

template <class C>
void ASecondSetForTheSameTickReplacesTheFirst() {
  using S = typename C::State;
  reach_ready();
  const S first = C::valid_state(0, 1);
  const S second = C::valid_state(0, 2);
  ASSERT_EQ(C::set(&first), MARV_SIL_OK);
  ASSERT_EQ(C::set(&second), MARV_SIL_OK);
  expect_reported<C>(run_tick(0), 0, 2, &second);

  // Replacement by an invalid-flag state is a replacement too.
  const S third = C::valid_state(1, 3);
  const S fourth = C::cleared_state(1);
  ASSERT_EQ(C::set(&third), MARV_SIL_OK);
  ASSERT_EQ(C::set(&fourth), MARV_SIL_OK);
  expect_reported<C>(run_tick(1), 1, 2, &fourth);
}

template <class C>
void ARejectedSecondSetLeavesTheFirstInPlace() {
  using S = typename C::State;
  reach_ready();
  const S first = C::valid_state(0, 1);
  S bad = C::valid_state(0, 2);
  C::field(bad, 1) = kNan;
  ASSERT_EQ(C::set(&first), MARV_SIL_OK);
  ASSERT_EQ(C::set(&bad), MARV_SIL_E_INPUT);
  expect_reported<C>(run_tick(0), 0, 1, &first);
}

template <class C>
void SetDoesNotAdvanceTheTickCounter() {
  reach_ready();
  const typename C::State s = C::valid_state(0);
  ASSERT_EQ(C::set(&s), MARV_SIL_OK);
  ASSERT_EQ(C::set(&s), MARV_SIL_OK);
  const TickResult wrong = run_tick(1);
  EXPECT_EQ(wrong.status, MARV_SIL_E_TICK);
  expect_reported<C>(run_tick(0), 0, 2, &s);
}

template <class C>
void AStaleLatchShowsAsAStampMismatchToTheComposition() {
  reach_ready();
  const typename C::State s = C::valid_state(0);
  ASSERT_EQ(C::set(&s), MARV_SIL_OK);
  expect_reported<C>(run_tick(0), 0, 1, &s);
  // The test composition keeps its latch; the freshness rule (stamp equality) is the composition's, and it sees the
  // stale stamp here.
  const TickResult r = run_tick(1);
  ASSERT_EQ(r.status, MARV_SIL_OK);
  EXPECT_EQ(r.dshot[0], C::code(0));
  EXPECT_EQ(r.dshot[1], C::code(C::kFlagHave | C::kFlagValid));
}

inline void StatusStringsOfTheCodesItUses() {
  for (const marv_sil_status s : {MARV_SIL_E_STATE, MARV_SIL_E_NULL, MARV_SIL_E_ABI, MARV_SIL_E_TICK, MARV_SIL_E_INPUT}) {
    const char* text = marv_sil_status_str(s);
    ASSERT_NE(text, nullptr);
    EXPECT_STRNE(text, "unknown status") << s;
  }
}

}  // namespace truth_channel_t1

#define MARV_TRUTH_T1_CASE(Suite, Channel, Name) \
  TEST(Suite, Name) { ::truth_channel_t1::in_child(::truth_channel_t1::Name<Channel>); }

// The tests both channels share, registered as Suite.<Name>; each runs its body in a forked child.
#define MARV_TRUTH_CHANNEL_T1_TESTS(Suite, Channel)                                         \
  MARV_TRUTH_T1_CASE(Suite, Channel, EStateBeforeInit)                                      \
  MARV_TRUTH_T1_CASE(Suite, Channel, EStateAfterShutdown)                                   \
  MARV_TRUTH_T1_CASE(Suite, Channel, ENullForANullPointer)                                  \
  MARV_TRUTH_T1_CASE(Suite, Channel, EAbiForAnyOtherStructSize)                             \
  MARV_TRUTH_T1_CASE(Suite, Channel, EAbiIsCheckedBeforeTheTick)                            \
  MARV_TRUTH_T1_CASE(Suite, Channel, ETickUnlessTheTickIsTheNumberOfTicksAlreadyRun)        \
  MARV_TRUTH_T1_CASE(Suite, Channel, ETickFollowsTheTicksRunByAMultiTickCall)               \
  MARV_TRUTH_T1_CASE(Suite, Channel, EInputForAReservedFlagBit)                             \
  MARV_TRUTH_T1_CASE(Suite, Channel, EInputForAnyNonzeroFieldWithTheValidBitClear)          \
  MARV_TRUTH_T1_CASE(Suite, Channel, EInputForANonFiniteFieldWithTheValidBitSet)            \
  MARV_TRUTH_T1_CASE(Suite, Channel, StatusesAreCheckedInTheDocumentedOrder)                \
  MARV_TRUTH_T1_CASE(Suite, Channel, OkHandsOverTheStateWithTheSilStampOfThatTick)          \
  MARV_TRUTH_T1_CASE(Suite, Channel, OkHandsOverEveryTickInTheCallOrderOfTheLoop)           \
  MARV_TRUTH_T1_CASE(Suite, Channel, OkWithTheValidBitClearAndZeroFieldsHandsOverAnInvalidState) \
  MARV_TRUTH_T1_CASE(Suite, Channel, ASecondSetForTheSameTickReplacesTheFirst)              \
  MARV_TRUTH_T1_CASE(Suite, Channel, ARejectedSecondSetLeavesTheFirstInPlace)               \
  MARV_TRUTH_T1_CASE(Suite, Channel, SetDoesNotAdvanceTheTickCounter)                       \
  MARV_TRUTH_T1_CASE(Suite, Channel, AStaleLatchShowsAsAStampMismatchToTheComposition)      \
  TEST(Suite, StatusStringsOfTheCodesItUses) { ::truth_channel_t1::StatusStringsOfTheCodesItUses(); }
