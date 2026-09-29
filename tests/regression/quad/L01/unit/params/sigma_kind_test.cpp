// Parameter record, quad L1: SigmaKind, the lock flag and vector components as the generator emits them for the L1
// fixture set (tests/regression/quad/L01/fixtures), invariant I1 in params_init, and the SIL override kind rule.
// Process state: no test here makes params_init succeed in the test process (a rejected init consumes nothing);
// every accepting init runs inside an EXPECT_EXIT child.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <span>

#include <marv/params/param.hpp>

#include "marv_sil.h"
#include "param_override.hpp"

namespace marv {
namespace {

using Table = std::array<ParamRecord, kParamCount>;

constexpr std::size_t idx(ParamId id) { return static_cast<std::size_t>(id); }

Table defaults_copy() {
  Table t{};
  std::copy(param_defaults().begin(), param_defaults().end(), t.begin());
  return t;
}

const ParamRecord& def(ParamId id) { return param_defaults()[idx(id)]; }

void expect_rejected(const Table& t) {
  EXPECT_FALSE(params_init(std::span<const ParamRecord, kParamCount>{t}));
  EXPECT_DEATH(static_cast<void>(param_get(static_cast<ParamId>(0))), "param_get before params_init");
}

template <class Mutate>
void reject_with(ParamId id, Mutate mutate) {
  Table t = defaults_copy();
  mutate(t[idx(id)]);
  expect_rejected(t);
}

struct Expect {
  ParamId id;
  SigmaKind kind;
  bool locked;
};

constexpr std::array kExpected{
    Expect{ParamId::l1_known_kg, SigmaKind::Known, false},
    Expect{ParamId::l1_exact_count, SigmaKind::Exact, false},
    Expect{ParamId::l1_unknown_s, SigmaKind::Unknown, false},
    Expect{ParamId::l1_locked_kg, SigmaKind::Known, true},
    Expect{ParamId::l1_arm_m_x, SigmaKind::Known, true},
    Expect{ParamId::l1_arm_m_y, SigmaKind::Unknown, true},
    Expect{ParamId::l1_arm_m_z, SigmaKind::Known, true},
    Expect{ParamId::l1_inertia_kgm2_xx, SigmaKind::Unknown, false},
    Expect{ParamId::l1_inertia_kgm2_yy, SigmaKind::Unknown, false},
    Expect{ParamId::l1_inertia_kgm2_zz, SigmaKind::Unknown, false},
    Expect{ParamId::l1_choice_gain, SigmaKind::Choice, false},
    Expect{ParamId::l1_limit_hz_min, SigmaKind::Choice, false},
    Expect{ParamId::l1_limit_hz_max, SigmaKind::Choice, false},
    Expect{ParamId::l1_motor_gain_m1, SigmaKind::Known, false},
    Expect{ParamId::l1_motor_gain_m2, SigmaKind::Known, false},
    Expect{ParamId::l1_motor_gain_m3, SigmaKind::Known, false},
    Expect{ParamId::l1_motor_gain_m4, SigmaKind::Known, false},
};
static_assert(kExpected.size() == kParamCount);

TEST(ParamRecordL1, KindAndLockFollowTheSource) {
  for (const Expect& e : kExpected) {
    const ParamRecord& r = def(e.id);
    EXPECT_EQ(r.sigma_kind, e.kind) << param_name(e.id);
    EXPECT_EQ(r.locked, e.locked) << param_name(e.id);
    if (e.kind == SigmaKind::Known) {
      EXPECT_GT(r.sigma, 0.0f) << param_name(e.id);
    } else {
      EXPECT_EQ(r.sigma, 0.0f) << param_name(e.id);
      EXPECT_FALSE(std::signbit(r.sigma)) << param_name(e.id);
    }
  }
}

TEST(ParamRecordL1, VectorComponentsKeepTheirOwnValueAndSigma) {
  EXPECT_EQ(def(ParamId::l1_arm_m_x).value.f32, 0.1f);
  EXPECT_EQ(def(ParamId::l1_arm_m_y).value.f32, -0.1f);
  EXPECT_EQ(def(ParamId::l1_arm_m_z).value.f32, 0.0f);
  EXPECT_EQ(def(ParamId::l1_arm_m_x).sigma, 0.001f);
  EXPECT_EQ(def(ParamId::l1_arm_m_z).sigma, 0.002f);
  EXPECT_EQ(def(ParamId::l1_limit_hz_min).value.f32, 10.0f);
  EXPECT_EQ(def(ParamId::l1_limit_hz_max).value.f32, 100.0f);
  EXPECT_EQ(def(ParamId::l1_motor_gain_m4).value.f32, 1.3f);
}

TEST(ParamRecordL1DeathTest, RejectsKnownWithZeroSigma) {
  reject_with(ParamId::l1_known_kg, [](ParamRecord& r) { r.sigma = 0.0f; });
}

TEST(ParamRecordL1DeathTest, RejectsKnownWithNonPositiveOrNonFiniteSigma) {
  reject_with(ParamId::l1_known_kg, [](ParamRecord& r) { r.sigma = -0.0f; });
  reject_with(ParamId::l1_known_kg, [](ParamRecord& r) { r.sigma = -1.0f; });
  reject_with(ParamId::l1_known_kg, [](ParamRecord& r) { r.sigma = std::numeric_limits<float>::infinity(); });
  reject_with(ParamId::l1_known_kg, [](ParamRecord& r) { r.sigma = std::numeric_limits<float>::quiet_NaN(); });
}

TEST(ParamRecordL1DeathTest, RejectsExactUnknownAndChoiceWithNonZeroSigma) {
  reject_with(ParamId::l1_exact_count, [](ParamRecord& r) { r.sigma = 1.0f; });
  reject_with(ParamId::l1_unknown_s, [](ParamRecord& r) { r.sigma = 0.01f; });
  reject_with(ParamId::l1_choice_gain, [](ParamRecord& r) { r.sigma = 0.5f; });
  reject_with(ParamId::l1_unknown_s, [](ParamRecord& r) { r.sigma = std::numeric_limits<float>::min(); });
}

TEST(ParamRecordL1DeathTest, RejectsExactUnknownAndChoiceWithNegativeZeroSigma) {
  reject_with(ParamId::l1_exact_count, [](ParamRecord& r) { r.sigma = -0.0f; });
  reject_with(ParamId::l1_unknown_s, [](ParamRecord& r) { r.sigma = -0.0f; });
  reject_with(ParamId::l1_choice_gain, [](ParamRecord& r) { r.sigma = -0.0f; });
}

TEST(ParamRecordL1DeathTest, RejectsExactUnknownAndChoiceWithNonFiniteSigma) {
  reject_with(ParamId::l1_unknown_s, [](ParamRecord& r) { r.sigma = std::numeric_limits<float>::quiet_NaN(); });
  reject_with(ParamId::l1_choice_gain, [](ParamRecord& r) { r.sigma = std::numeric_limits<float>::infinity(); });
}

TEST(ParamRecordL1DeathTest, RejectsAKindThatDoesNotMatchTheSigma) {
  reject_with(ParamId::l1_known_kg, [](ParamRecord& r) { r.sigma_kind = SigmaKind::Unknown; });
  reject_with(ParamId::l1_known_kg, [](ParamRecord& r) { r.sigma_kind = SigmaKind::Exact; });
  reject_with(ParamId::l1_unknown_s, [](ParamRecord& r) { r.sigma_kind = SigmaKind::Known; });
  reject_with(ParamId::l1_choice_gain, [](ParamRecord& r) { r.sigma_kind = SigmaKind::Known; });
}

TEST(ParamRecordL1DeathTest, RejectsAnOutOfRangeKind) {
  const auto past_last = static_cast<SigmaKind>(static_cast<std::uint8_t>(SigmaKind::Choice) + 1);
  const auto all_bits = static_cast<SigmaKind>(std::numeric_limits<std::uint8_t>::max());
  for (const ParamId id : {ParamId::l1_known_kg, ParamId::l1_unknown_s}) {
    reject_with(id, [&](ParamRecord& r) { r.sigma_kind = past_last; });
    reject_with(id, [&](ParamRecord& r) { r.sigma_kind = all_bits; });
  }
}

bool accepted_after_switching_zero_sigma_kinds() {
  Table t = defaults_copy();
  t[idx(ParamId::l1_unknown_s)].sigma_kind = SigmaKind::Exact;
  t[idx(ParamId::l1_choice_gain)].sigma_kind = SigmaKind::Unknown;
  t[idx(ParamId::l1_exact_count)].sigma_kind = SigmaKind::Choice;
  if (!params_init(std::span<const ParamRecord, kParamCount>{t})) {
    return false;
  }
  return param_get(ParamId::l1_unknown_s).sigma_kind == SigmaKind::Exact &&
         param_get(ParamId::l1_choice_gain).sigma_kind == SigmaKind::Unknown &&
         param_get(ParamId::l1_exact_count).sigma_kind == SigmaKind::Choice;
}

bool defaults_accepted_and_read_back() {
  if (!params_init(param_defaults())) {
    return false;
  }
  for (const Expect& e : kExpected) {
    const ParamRecord got = param_get(e.id);
    if (got.sigma_kind != e.kind || got.locked != e.locked) {
      return false;
    }
  }
  return true;
}

TEST(ParamRecordL1DeathTest, DefaultsPassInitAndKindsAndLocksReadBack) {
  EXPECT_EXIT(std::exit(defaults_accepted_and_read_back() ? EXIT_SUCCESS : EXIT_FAILURE),
              ::testing::ExitedWithCode(0), "");
}

TEST(ParamRecordL1DeathTest, ZeroSigmaKindsAreInterchangeableAtInit) {
  EXPECT_EXIT(std::exit(accepted_after_switching_zero_sigma_kinds() ? EXIT_SUCCESS : EXIT_FAILURE),
              ::testing::ExitedWithCode(0), "");
}

marv_sil_param_override f32_override(ParamId id, float value, float sigma) {
  marv_sil_param_override o{};
  o.id = static_cast<std::uint32_t>(id);
  o.type = MARV_PARAM_F32;
  o.f32 = value;
  o.sigma = sigma;
  return o;
}

TEST(ParamRecordL1SilOverride, KnownWhenSigmaPositiveElseExactWithPositiveZero) {
  const std::array overrides{
      f32_override(ParamId::l1_known_kg, 2.0f, 0.5f),
      f32_override(ParamId::l1_unknown_s, 0.5f, 0.0f),
      f32_override(ParamId::l1_choice_gain, 4.0f, 0.25f),
      f32_override(ParamId::l1_locked_kg, 3.0f, -0.0f),
  };
  Table merged{};
  const sil::OverrideStatus status = sil::apply_overrides(
      param_defaults(), std::span<const marv_sil_param_override>{overrides},
      std::span<ParamRecord, kParamCount>{merged});
  ASSERT_TRUE(status.ok);

  EXPECT_EQ(merged[idx(ParamId::l1_known_kg)].sigma_kind, SigmaKind::Known);
  EXPECT_EQ(merged[idx(ParamId::l1_known_kg)].sigma, 0.5f);
  EXPECT_EQ(merged[idx(ParamId::l1_unknown_s)].sigma_kind, SigmaKind::Exact);
  EXPECT_EQ(merged[idx(ParamId::l1_unknown_s)].sigma, 0.0f);
  EXPECT_EQ(merged[idx(ParamId::l1_choice_gain)].sigma_kind, SigmaKind::Known);
  EXPECT_EQ(merged[idx(ParamId::l1_locked_kg)].sigma_kind, SigmaKind::Exact);
  EXPECT_FALSE(std::signbit(merged[idx(ParamId::l1_locked_kg)].sigma));
  EXPECT_TRUE(merged[idx(ParamId::l1_locked_kg)].locked);

  EXPECT_EQ(merged[idx(ParamId::l1_arm_m_y)].sigma_kind, SigmaKind::Unknown);
  EXPECT_EQ(merged[idx(ParamId::l1_exact_count)].sigma_kind, SigmaKind::Exact);
}

bool merged_overrides_accepted() {
  const std::array overrides{
      f32_override(ParamId::l1_known_kg, 2.0f, 0.0f),
      f32_override(ParamId::l1_unknown_s, 0.5f, 0.3f),
      f32_override(ParamId::l1_choice_gain, 4.0f, -0.0f),
  };
  Table merged{};
  if (!sil::apply_overrides(param_defaults(), std::span<const marv_sil_param_override>{overrides},
                            std::span<ParamRecord, kParamCount>{merged})
           .ok) {
    return false;
  }
  return params_init(std::span<const ParamRecord, kParamCount>{merged}) &&
         param_get(ParamId::l1_known_kg).sigma_kind == SigmaKind::Exact &&
         param_get(ParamId::l1_unknown_s).sigma_kind == SigmaKind::Known &&
         param_get(ParamId::l1_choice_gain).sigma_kind == SigmaKind::Exact;
}

TEST(ParamRecordL1DeathTest, OverriddenTablePassesInit) {
  EXPECT_EXIT(std::exit(merged_overrides_accepted() ? EXIT_SUCCESS : EXIT_FAILURE), ::testing::ExitedWithCode(0), "");
}

}  // namespace
}  // namespace marv
