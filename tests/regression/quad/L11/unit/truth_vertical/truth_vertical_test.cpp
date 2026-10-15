// T1 of marv_truth_vertical_set: status codes, the valid-bit rule, replace-on-second-set, hand-over. The shared cases
// are in truth_channel_t1.hpp; this file holds the vertical channel and the case only it has.
#include <gtest/gtest.h>

#include <marv_sil.h>
#include <marv_truth_vertical.h>

#include <cstddef>
#include <cstdint>

#include <marv/types/vertical_state.hpp>

#include "probe_codes.hpp"
#include "truth_channel_t1.hpp"

namespace {

using truth_channel_t1::expect_reported;
using truth_channel_t1::reach_ready;
using truth_channel_t1::run_tick;

constexpr std::uint32_t kValid = 1u << MARV_TRUTH_VERTICAL_VALID;

struct Vertical {
  using State = marv_truth_vertical;
  using Row = verticaltest::Row;

  static constexpr const char* kInputName = "vertical_input";
  static constexpr unsigned kFlagCount = MARV_TRUTH_VERTICAL_FLAG_COUNT;
  static constexpr std::size_t kFieldCount = 2;  // z_ned_m, vz_ned_m_s
  static constexpr std::uint32_t kFlagHave = verticaltest::kFlagHave;
  static constexpr std::uint32_t kFlagValid = verticaltest::kFlagValid;
  static constexpr std::uint32_t kFlagStampMatches = verticaltest::kFlagStampMatches;

  static marv_sil_status set(const marv_truth_vertical* s) { return marv_truth_vertical_set(s); }
  static std::uint16_t code(std::uint32_t v) { return verticaltest::code(v); }
  static Row row_of(std::uint32_t calls, std::uint32_t flags, std::uint32_t hash) {
    return verticaltest::row_of(calls, flags, hash);
  }
  static std::uint32_t hash_of(const marv::VerticalState<float>& v) { return verticaltest::hash_of(v); }

  // Field i of the state: 0 z_ned_m, 1 vz_ned_m_s.
  static float& field(marv_truth_vertical& s, std::size_t i) { return i == 0 ? s.z_ned_m : s.vz_ned_m_s; }

  // A state that differs for every i; z is negative (above the NED origin), v_z of both signs.
  static marv_truth_vertical valid_state(std::uint64_t tick, std::uint32_t i = 0) {
    const auto f = static_cast<float>(i);
    marv_truth_vertical s{};
    s.struct_size = sizeof(marv_truth_vertical);
    s.flags = kValid;
    s.tick = tick;
    s.z_ned_m = -1.5f - f;
    s.vz_ned_m_s = 0.25f - 0.125f * f;
    return s;
  }

  static marv_truth_vertical cleared_state(std::uint64_t tick) {
    marv_truth_vertical s{};
    s.struct_size = sizeof(marv_truth_vertical);
    s.tick = tick;
    return s;
  }

  static marv::VerticalState<float> expected_hand_over(const marv_truth_vertical& s) {
    return marv::VerticalState<float>{truth_channel_t1::stamp_of(s.tick), s.z_ned_m, s.vz_ned_m_s,
                                      (s.flags & kValid) != 0};
  }
};

MARV_TRUTH_CHANNEL_T1_TESTS(TruthVertical, Vertical)

TEST(TruthVertical, OkWithTheValidBitSetAndZeroFieldsHandsOverAValidState) {
  truth_channel_t1::in_child([] {
    reach_ready();
    // At the NED origin and at rest is a state like any other: the zero-quaternion rule has no vertical counterpart.
    marv_truth_vertical s = Vertical::valid_state(0);
    s.z_ned_m = 0.0f;
    s.vz_ned_m_s = -0.0f;
    ASSERT_EQ(marv_truth_vertical_set(&s), MARV_SIL_OK);
    expect_reported<Vertical>(run_tick(0), 0, 1, &s);
  });
}

}  // namespace
