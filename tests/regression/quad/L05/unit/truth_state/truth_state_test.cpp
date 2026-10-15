// T1 of marv_truth_state_set: status codes, replace-on-second-set, hand-over. The shared cases are in
// truth_channel_t1.hpp; this file holds the attitude channel and the cases only it has.
#include <gtest/gtest.h>

#include <marv_sil.h>
#include <marv_truth.h>

#include <cstddef>
#include <cstdint>
#include <limits>

#include <marv/types/attitude_state.hpp>

#include "probe_codes.hpp"
#include "truth_channel_t1.hpp"

namespace {

using truth_channel_t1::expect_no_hand_over_at;
using truth_channel_t1::reach_ready;

constexpr std::uint32_t kValid = 1u << MARV_TRUTH_ATTITUDE_VALID;
constexpr std::size_t kOmegaCount = 3;  // omega x, y, z follow the quaternion in the field order

struct Attitude {
  using State = marv_truth_state;
  using Row = truthtest::Row;

  static constexpr const char* kInputName = "attitude_input";
  static constexpr unsigned kFlagCount = MARV_TRUTH_FLAG_COUNT;
  static constexpr std::size_t kFieldCount = MARV_TRUTH_Q_COUNT + kOmegaCount;  // q w x y z, omega x y z
  static constexpr std::uint32_t kFlagHave = truthtest::kFlagHave;
  static constexpr std::uint32_t kFlagValid = truthtest::kFlagValid;
  static constexpr std::uint32_t kFlagStampMatches = truthtest::kFlagStampMatches;

  static marv_sil_status set(const marv_truth_state* s) { return marv_truth_state_set(s); }
  static std::uint16_t code(std::uint32_t v) { return truthtest::code(v); }
  static Row row_of(std::uint32_t calls, std::uint32_t flags, std::uint32_t hash) {
    return truthtest::row_of(calls, flags, hash);
  }
  static std::uint32_t hash_of(const marv::AttitudeState<float>& a) { return truthtest::hash_of(a); }

  // Field i of the state: 0..3 the quaternion w x y z, 4..6 omega x y z.
  static float& field(marv_truth_state& s, std::size_t i) {
    if (i < MARV_TRUTH_Q_COUNT) {
      return s.q_wxyz[i];
    }
    return i == MARV_TRUTH_Q_COUNT ? s.omega_frd_rad_s.x : i == MARV_TRUTH_Q_COUNT + 1 ? s.omega_frd_rad_s.y
                                                                                       : s.omega_frd_rad_s.z;
  }

  // A state that differs for every i; the quaternion is deliberately not of unit norm (the entry does not normalise).
  static marv_truth_state valid_state(std::uint64_t tick, std::uint32_t i = 0) {
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

  static marv_truth_state cleared_state(std::uint64_t tick) {
    marv_truth_state s{};
    s.struct_size = sizeof(marv_truth_state);
    s.tick = tick;
    return s;
  }

  static marv::AttitudeState<float> expected_hand_over(const marv_truth_state& s) {
    return marv::AttitudeState<float>{
        truth_channel_t1::stamp_of(s.tick),
        marv::prim::Quat<float>{s.q_wxyz[MARV_TRUTH_Q_W], s.q_wxyz[MARV_TRUTH_Q_X], s.q_wxyz[MARV_TRUTH_Q_Y],
                                s.q_wxyz[MARV_TRUTH_Q_Z]},
        marv::prim::Vec3<float>{s.omega_frd_rad_s.x, s.omega_frd_rad_s.y, s.omega_frd_rad_s.z},
        (s.flags & kValid) != 0};
  }
};

MARV_TRUTH_CHANNEL_T1_TESTS(TruthState, Attitude)

TEST(TruthState, EInputForAZeroQuaternionWithTheValidBitSet) {
  truth_channel_t1::in_child([] {
    reach_ready();
    marv_truth_state s = Attitude::valid_state(0);
    for (std::size_t i = 0; i < MARV_TRUTH_Q_COUNT; ++i) {
      s.q_wxyz[i] = i == 0 ? -0.0f : 0.0f;
    }
    EXPECT_EQ(marv_truth_state_set(&s), MARV_SIL_E_INPUT);
    expect_no_hand_over_at<Attitude>(0);
  });
}

TEST(TruthState, ATinyQuaternionIsNotZero) {
  truth_channel_t1::in_child([] {
    reach_ready();
    marv_truth_state s = Attitude::valid_state(0);
    for (std::size_t i = 0; i < MARV_TRUTH_Q_COUNT; ++i) {
      s.q_wxyz[i] = 0.0f;
    }
    s.q_wxyz[MARV_TRUTH_Q_Z] = std::numeric_limits<float>::denorm_min();
    EXPECT_EQ(marv_truth_state_set(&s), MARV_SIL_OK);
  });
}

}  // namespace
