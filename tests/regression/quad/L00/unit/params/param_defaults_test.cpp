#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <type_traits>
#include <utility>

#include <marv/params/param_ids.hpp>
#include <marv/params/param_types.hpp>

namespace marv {
namespace {

static_assert(std::is_trivially_copyable_v<ParamValue> && std::is_standard_layout_v<ParamValue>);
static_assert(std::is_trivially_copyable_v<ParamRecord> && std::is_standard_layout_v<ParamRecord>);
static_assert(kParamCount > 0);

// The rules params_init applies to every record, except the unit-pointer equality, which needs the runtime.
bool record_valid(const ParamRecord& r) {
  const bool sigma_ok = std::isfinite(r.sigma) && r.sigma >= 0.0f;
  const bool value_ok = r.value.type == ParamType::F32
                            ? std::isfinite(r.value.f32) && r.value.i32 == 0
                            : r.value.type == ParamType::I32 && r.value.f32 == 0.0f;
  const bool strings_ok = r.unit != nullptr && r.source != nullptr && std::strlen(r.unit) > 0 &&
                          std::strlen(r.source) > 0;
  return sigma_ok && value_ok && strings_ok;
}

template <std::size_t I>
void check_id() {
  constexpr auto id = static_cast<ParamId>(I);
  using Traits = ParamTraits<id>;
  static_assert(std::is_same_v<typename Traits::value_type, float> == (Traits::type == ParamType::F32));
  static_assert(std::is_same_v<typename Traits::value_type, std::int32_t> == (Traits::type == ParamType::I32));

  const ParamRecord& r = generated::kParamDefaults[I];
  EXPECT_EQ(r.value.type, Traits::type) << "id " << I;
  EXPECT_TRUE(record_valid(r)) << "id " << I;
  EXPECT_NE(generated::kParamNames[I], nullptr) << "id " << I;
  EXPECT_GT(std::strlen(generated::kParamNames[I]), 0U) << "id " << I;
  EXPECT_FALSE(r.locked) << "id " << I;
  EXPECT_TRUE(r.origin == ParamOrigin::DefaultFromCard || r.origin == ParamOrigin::DefaultFromRegister)
      << "id " << I;
  EXPECT_TRUE(r.method != ParamMethod::Derived || std::strlen(r.source) > 0) << "id " << I;
}

template <std::size_t... I>
void check_all(std::index_sequence<I...>) {
  (check_id<I>(), ...);
}

TEST(ParamDefaults, EveryIdMatchesItsTraitsAndSatisfiesParamsInitRules) {
  check_all(std::make_index_sequence<kParamCount>{});
}

TEST(ParamDefaults, IdsAreDenseAndNamesUnique) {
  EXPECT_EQ(static_cast<std::size_t>(ParamId::fixture_mass_kg), 0U);
  for (std::size_t i = 0; i < kParamCount; ++i) {
    for (std::size_t j = i + 1; j < kParamCount; ++j) {
      EXPECT_STRNE(generated::kParamNames[i], generated::kParamNames[j]);
    }
  }
}

TEST(ParamDefaults, FixtureValuesAreReproducedExactlyAsFloat) {
  const auto& t = generated::kParamDefaults;
  EXPECT_EQ(t[static_cast<std::size_t>(ParamId::fixture_mass_kg)].value.f32, 0.75f);
  EXPECT_EQ(t[static_cast<std::size_t>(ParamId::fixture_time_constant_s)].value.f32, 0.033f);
  EXPECT_EQ(t[static_cast<std::size_t>(ParamId::fixture_drag_ratio)].value.f32, 0.1f);
  EXPECT_EQ(t[static_cast<std::size_t>(ParamId::fixture_time_constant_s)].sigma, 1.0e-3f);
  EXPECT_EQ(t[static_cast<std::size_t>(ParamId::fixture_motor_count)].value.i32, 4);
  EXPECT_EQ(t[static_cast<std::size_t>(ParamId::fixture_gate_count)].value.i32, -3);
  EXPECT_EQ(t[static_cast<std::size_t>(ParamId::fixture_rate_limit_rad_s)].origin, ParamOrigin::DefaultFromRegister);
  EXPECT_EQ(t[static_cast<std::size_t>(ParamId::fixture_inertia_scale_kg)].method, ParamMethod::Derived);
  EXPECT_STREQ(t[static_cast<std::size_t>(ParamId::fixture_drag_ratio)].unit, "1");
}

// Negative controls: each params_init rule must reject a record that breaks it.
TEST(ParamDefaults, ValidatorRejectsBrokenRecords) {
  const ParamRecord good = generated::kParamDefaults[static_cast<std::size_t>(ParamId::fixture_mass_kg)];
  ASSERT_TRUE(record_valid(good));

  ParamRecord r = good;
  r.sigma = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(record_valid(r));
  r = good;
  r.sigma = std::numeric_limits<float>::infinity();
  EXPECT_FALSE(record_valid(r));
  r = good;
  r.sigma = -good.sigma;
  EXPECT_FALSE(record_valid(r));
  r = good;
  r.value.f32 = std::numeric_limits<float>::infinity();
  EXPECT_FALSE(record_valid(r));
  r = good;
  r.value.i32 = 1;
  EXPECT_FALSE(record_valid(r));
  r = good;
  r.unit = nullptr;
  EXPECT_FALSE(record_valid(r));
  r = good;
  r.source = nullptr;
  EXPECT_FALSE(record_valid(r));

  const ParamRecord good_i32 = generated::kParamDefaults[static_cast<std::size_t>(ParamId::fixture_motor_count)];
  ASSERT_TRUE(record_valid(good_i32));
  r = good_i32;
  r.value.f32 = 1.0f;
  EXPECT_FALSE(record_valid(r));
}

}  // namespace
}  // namespace marv
