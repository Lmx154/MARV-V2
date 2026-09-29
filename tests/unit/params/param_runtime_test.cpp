// Parameter runtime after a successful params_init. Process state: this executable initialises the runtime once
// and never sees it uninitialised; the uninitialised-state tests are in param_runtime_uninit_test.cpp (own executable).
#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>
#include <utility>

#include <marv/params/param.hpp>

namespace marv {
namespace {

static_assert(std::is_same_v<decltype(param_value<ParamId::fixture_motor_count>()), std::int32_t>);
static_assert(std::is_same_v<decltype(param_value<ParamId::fixture_mass_kg>()), float>);

template <ParamId Id>
concept ReturnsTraitsType = std::is_same_v<decltype(param_value<Id>()), typename ParamTraits<Id>::value_type> &&
                            (ParamTraits<Id>::type == ParamType::I32
                                 ? std::is_same_v<decltype(param_value<Id>()), std::int32_t>
                                 : std::is_same_v<decltype(param_value<Id>()), float>);

void init_once() {
  static const bool ok = params_init(param_defaults());
  ASSERT_TRUE(ok);
}

template <std::size_t I>
void check_id() {
  constexpr auto id = static_cast<ParamId>(I);
  static_assert(ReturnsTraitsType<id>);

  const ParamRecord got = param_get(id);
  const ParamRecord& want = param_defaults()[I];
  EXPECT_EQ(got.value.type, want.value.type) << "id " << I;
  EXPECT_EQ(got.value.f32, want.value.f32) << "id " << I;
  EXPECT_EQ(got.value.i32, want.value.i32) << "id " << I;
  EXPECT_EQ(got.sigma, want.sigma) << "id " << I;
  EXPECT_EQ(got.origin, want.origin) << "id " << I;
  EXPECT_EQ(got.method, want.method) << "id " << I;
  EXPECT_EQ(got.locked, want.locked) << "id " << I;
  EXPECT_EQ(got.unit, want.unit) << "id " << I;
  EXPECT_EQ(got.source, want.source) << "id " << I;
  EXPECT_STREQ(param_name(id), generated::kParamNames[I]) << "id " << I;

  if constexpr (ParamTraits<id>::type == ParamType::F32) {
    EXPECT_EQ(param_value<id>(), got.value.f32) << "id " << I;
  } else {
    EXPECT_EQ(param_value<id>(), got.value.i32) << "id " << I;
  }
}

template <std::size_t... I>
void check_all(std::index_sequence<I...>) {
  (check_id<I>(), ...);
}

// The L0 contract test: param_get returns value and provenance for every generated id.
TEST(ParamRuntime, ParamGetReturnsTheDefaultsRecordForEveryId) {
  ASSERT_NO_FATAL_FAILURE(init_once());
  check_all(std::make_index_sequence<kParamCount>{});
}

TEST(ParamRuntime, ParamDefaultsIsTheGeneratedTable) {
  EXPECT_EQ(param_defaults().data(), &generated::kParamDefaults[0]);
  EXPECT_EQ(param_defaults().size(), kParamCount);
}

TEST(ParamRuntime, SecondInitIsRefusedAndChangesNothing) {
  ASSERT_NO_FATAL_FAILURE(init_once());
  EXPECT_FALSE(params_init(param_defaults()));

  std::array<ParamRecord, kParamCount> other{};
  std::copy(param_defaults().begin(), param_defaults().end(), other.begin());
  other[0].value.f32 = other[0].value.f32 + 1.0f;
  EXPECT_FALSE(params_init(std::span<const ParamRecord, kParamCount>{other}));

  check_all(std::make_index_sequence<kParamCount>{});
}

TEST(ParamRuntimeDeathTest, InvalidIdPanicsAfterInit) {
  ASSERT_NO_FATAL_FAILURE(init_once());
  EXPECT_DEATH(static_cast<void>(param_get(static_cast<ParamId>(kParamCount))), "invalid parameter id");
}

TEST(ParamRuntimeDeathTest, ParamNameWithInvalidIdPanics) {
  EXPECT_DEATH(static_cast<void>(param_name(static_cast<ParamId>(kParamCount))), "invalid parameter id");
}

}  // namespace
}  // namespace marv
