// Parameter runtime while uninitialised. Process state: no test here ever makes params_init succeed in the test
// process, so the runtime stays uninitialised in this executable (own executable, separate from
// param_runtime_test.cpp). The one accepting init runs inside an EXPECT_EXIT child process.
#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <span>

#include <marv/params/param.hpp>

namespace marv {
namespace {

using Table = std::array<ParamRecord, kParamCount>;

constexpr std::size_t kFirst = 0;
constexpr std::size_t kLast = kParamCount - 1;

Table defaults_copy() {
  Table t{};
  std::copy(param_defaults().begin(), param_defaults().end(), t.begin());
  return t;
}

std::size_t find_index(ParamType type) {
  for (std::size_t i = 0; i < kParamCount; ++i) {
    if (param_defaults()[i].value.type == type) {
      return i;
    }
  }
  ADD_FAILURE() << "no parameter of the requested type in the generated table";
  return 0;
}

// A rejected params_init must leave the runtime uninitialised: param_get still panics for the reason "before init".
void expect_rejected(const Table& t) {
  EXPECT_FALSE(params_init(std::span<const ParamRecord, kParamCount>{t}));
  EXPECT_DEATH(static_cast<void>(param_get(static_cast<ParamId>(0))), "param_get before params_init");
}

template <class Mutate>
void reject_with(std::size_t index, Mutate mutate) {
  Table t = defaults_copy();
  mutate(t[index]);
  expect_rejected(t);
}

TEST(ParamRuntimeUninitDeathTest, ParamGetBeforeInitPanics) {
  EXPECT_DEATH(static_cast<void>(param_get(static_cast<ParamId>(0))), "param_get before params_init");
}

TEST(ParamRuntimeUninitDeathTest, InvalidIdPanics) {
  EXPECT_DEATH(static_cast<void>(param_get(static_cast<ParamId>(kParamCount))), "invalid parameter id");
}

TEST(ParamRuntimeUninit, ParamNameAndDefaultsNeedNoInit) {
  for (std::size_t i = 0; i < kParamCount; ++i) {
    EXPECT_STREQ(param_name(static_cast<ParamId>(i)), generated::kParamNames[i]);
  }
  EXPECT_EQ(param_defaults().data(), &generated::kParamDefaults[0]);
}

TEST(ParamRuntimeUninitDeathTest, RejectsWrongType) {
  for (const std::size_t idx : {kFirst, kLast}) {
    reject_with(idx, [](ParamRecord& r) {
      const bool was_f32 = r.value.type == ParamType::F32;
      r.value.type = was_f32 ? ParamType::I32 : ParamType::F32;
      r.value.f32 = 0.0f;
      r.value.i32 = 0;
    });
  }
}

TEST(ParamRuntimeUninitDeathTest, RejectsForeignUnitPointerWithEqualText) {
  static char foreign[32];
  for (const std::size_t idx : {kFirst, kLast}) {
    const char* generated_unit = param_defaults()[idx].unit;
    ASSERT_LT(std::strlen(generated_unit), sizeof foreign);
    std::strcpy(foreign, generated_unit);
    ASSERT_STREQ(foreign, generated_unit);
    ASSERT_NE(static_cast<const char*>(foreign), generated_unit);
    reject_with(idx, [](ParamRecord& r) { r.unit = foreign; });
  }
}

TEST(ParamRuntimeUninitDeathTest, RejectsNanSigma) {
  for (const std::size_t idx : {kFirst, kLast}) {
    reject_with(idx, [](ParamRecord& r) { r.sigma = std::numeric_limits<float>::quiet_NaN(); });
  }
}

TEST(ParamRuntimeUninitDeathTest, RejectsInfiniteSigma) {
  reject_with(kFirst, [](ParamRecord& r) { r.sigma = std::numeric_limits<float>::infinity(); });
}

TEST(ParamRuntimeUninitDeathTest, RejectsNegativeSigma) {
  for (const std::size_t idx : {kFirst, kLast}) {
    reject_with(idx, [](ParamRecord& r) { r.sigma = -std::numeric_limits<float>::min(); });
  }
}

TEST(ParamRuntimeUninitDeathTest, RejectsInfiniteF32) {
  reject_with(find_index(ParamType::F32), [](ParamRecord& r) { r.value.f32 = std::numeric_limits<float>::infinity(); });
}

TEST(ParamRuntimeUninitDeathTest, RejectsNanF32) {
  reject_with(find_index(ParamType::F32), [](ParamRecord& r) { r.value.f32 = std::numeric_limits<float>::quiet_NaN(); });
}

TEST(ParamRuntimeUninitDeathTest, RejectsNonZeroUnusedMember) {
  reject_with(find_index(ParamType::F32), [](ParamRecord& r) { r.value.i32 = 1; });
  reject_with(find_index(ParamType::I32), [](ParamRecord& r) { r.value.f32 = 1.0f; });
}

TEST(ParamRuntimeUninitDeathTest, RejectsNullSource) {
  for (const std::size_t idx : {kFirst, kLast}) {
    reject_with(idx, [](ParamRecord& r) { r.source = nullptr; });
  }
}

// Runs in a child process: a rejected init consumes nothing, so a following valid init still succeeds.
bool rejected_then_accepted() {
  Table bad = defaults_copy();
  bad[kLast].sigma = std::numeric_limits<float>::quiet_NaN();
  if (params_init(std::span<const ParamRecord, kParamCount>{bad})) {
    return false;
  }
  if (!params_init(param_defaults())) {
    return false;
  }
  for (std::size_t i = 0; i < kParamCount; ++i) {
    const ParamRecord got = param_get(static_cast<ParamId>(i));
    if (got.value.f32 != param_defaults()[i].value.f32 || got.value.i32 != param_defaults()[i].value.i32) {
      return false;
    }
  }
  return !params_init(param_defaults());
}

TEST(ParamRuntimeUninitDeathTest, ValidInitStillSucceedsAfterRejections) {
  EXPECT_EXIT(std::exit(rejected_then_accepted() ? EXIT_SUCCESS : EXIT_FAILURE), ::testing::ExitedWithCode(0), "");
}

}  // namespace
}  // namespace marv
