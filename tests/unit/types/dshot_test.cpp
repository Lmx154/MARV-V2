#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <type_traits>

#include "marv/types/actuator.hpp"

namespace marv {
namespace {

// The oracle is written out here from the cited DShot ranges, independent of the constants under test.
constexpr std::uint32_t kExpectedMin = 48;
constexpr std::uint32_t kExpectedMax = 2047;
constexpr std::uint32_t kRawValues = std::uint32_t{1} << 16;

bool expected_valid(std::uint32_t r) { return r == 0 || (r >= kExpectedMin && r <= kExpectedMax); }

static_assert(prim::kDshotThrottleMin == kExpectedMin);
static_assert(prim::kDshotThrottleMax == kExpectedMax);

TEST(DshotValue, IsNotConstructibleFromARawInteger) {
  static_assert(!std::is_constructible_v<DshotValue, std::uint16_t>);
  static_assert(!std::is_constructible_v<DshotValue, int>);
  static_assert(!std::is_convertible_v<std::uint16_t, DshotValue>);
  static_assert(std::is_default_constructible_v<DshotValue>);
  SUCCEED();
}

TEST(DshotValue, DefaultIsStop) {
  EXPECT_EQ(DshotValue{}.raw(), 0);
  EXPECT_EQ(DshotValue::stop().raw(), 0);
  EXPECT_EQ(DshotValue{}, DshotValue::stop());
  static_assert(DshotValue{}.raw() == 0);
}

TEST(DshotValue, FromRawAcceptsExactlyStopAndThrottleRangeOverAllRawValues) {
  std::uint32_t accepted = 0;
  for (std::uint32_t r = 0; r < kRawValues; ++r) {
    const auto raw = static_cast<std::uint16_t>(r);
    const bool want = expected_valid(r);
    ASSERT_EQ(DshotValue::is_valid_raw(raw), want) << "raw " << r;
    const std::optional<DshotValue> v = DshotValue::from_raw(raw);
    ASSERT_EQ(v.has_value(), want) << "raw " << r;
    if (v.has_value()) {
      ++accepted;
      ASSERT_EQ(v->raw(), raw) << "raw " << r;
    }
  }
  EXPECT_EQ(accepted, 1 + (kExpectedMax - kExpectedMin + 1));
}

TEST(DshotValue, FromRawIsUsableInConstantExpressions) {
  static_assert(DshotValue::from_raw(0).has_value());
  static_assert(!DshotValue::from_raw(1).has_value());
  static_assert(!DshotValue::from_raw(47).has_value());
  static_assert(DshotValue::from_raw(48).has_value());
  static_assert(DshotValue::from_raw(2047).has_value());
  static_assert(!DshotValue::from_raw(2048).has_value());
  SUCCEED();
}

TEST(DshotValue, EqualityComparesRaw) {
  const auto a = DshotValue::from_raw(1000);
  const auto b = DshotValue::from_raw(1000);
  const auto c = DshotValue::from_raw(1001);
  ASSERT_TRUE(a && b && c);
  EXPECT_EQ(*a, *b);
  EXPECT_NE(*a, *c);
  EXPECT_NE(*a, DshotValue::stop());
}

}  // namespace
}  // namespace marv
