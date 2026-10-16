// T1 tests of the CRSF-to-ManualControl conversion (fw/hal/target/decode/rc_input.hpp, M2 P5b). Expected values are
// computed by hand from the TBS CRSF spec (https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md, "0x16 RC Channels
// Packed Payload": us = (ticks - 992) * 5 / 8 + 1500) and the contract of decision 0040 / ManualControl: stick
// x = (us - 1500) / 512 clamped to [-1, 1], throttle = (us - 988) / 1024 clamped to [0, 1], AETR channels 1 to 4,
// pitch = -x(channel 2) (a pushed-forward stick raises the channel and means nose down, core 3 FRD), link quality /100.
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include <marv/hal_target/rc_input.hpp>

namespace marv::hal_target {
namespace {

constexpr RcChannelMap kAetr{1, 2, 3, 4};
constexpr std::uint16_t kMid = 992;
constexpr std::uint16_t kLow = 172;
constexpr std::uint16_t kHigh = 1811;
constexpr float kTol = 1e-6F;

CrsfRcChannels centred() {
  CrsfRcChannels rc;
  rc.raw.fill(kMid);
  return rc;
}

ManualControl convert(const CrsfRcChannels& rc, const RcChannelMap& map = kAetr) {
  ManualControl m;
  EXPECT_TRUE(crsf_to_controls(rc, map, m));
  return m;
}

TEST(RcInput, TicksToMicroseconds) {
  EXPECT_FLOAT_EQ(crsf_ticks_to_us(kMid), 1500.0F);
  EXPECT_FLOAT_EQ(crsf_ticks_to_us(kLow), 987.5F);       // (172 - 992) * 5 / 8 + 1500
  EXPECT_FLOAT_EQ(crsf_ticks_to_us(kHigh), 2011.875F);   // (1811 - 992) * 5 / 8 + 1500
  EXPECT_FLOAT_EQ(crsf_ticks_to_us(0), 880.0F);          // (0 - 992) * 5 / 8 + 1500
  EXPECT_FLOAT_EQ(crsf_ticks_to_us(1984), 2120.0F);      // the extended limit, (1984 - 992) * 5 / 8 + 1500
}

TEST(RcInput, NormalisedSticksAndClamps) {
  EXPECT_EQ(rc_normalised(1500.0F), 0.0F);
  EXPECT_FLOAT_EQ(rc_normalised(1500.0F + 256.0F), 0.5F);
  EXPECT_FLOAT_EQ(rc_normalised(1500.0F - 256.0F), -0.5F);
  EXPECT_EQ(rc_normalised(1500.0F + 512.0F), 1.0F);
  EXPECT_EQ(rc_normalised(1500.0F - 512.0F), -1.0F);
  EXPECT_EQ(rc_normalised(2120.0F), 1.0F);
  EXPECT_EQ(rc_normalised(880.0F), -1.0F);
  // The standard range ends: 172 -> 987.5 us is below -1 and clamps; 1811 -> 2011.875 us is 0.99976.
  EXPECT_EQ(rc_normalised(crsf_ticks_to_us(kLow)), -1.0F);
  EXPECT_NEAR(rc_normalised(crsf_ticks_to_us(kHigh)), 511.875F / 512.0F, kTol);
}

TEST(RcInput, Throttle) {
  EXPECT_EQ(rc_throttle(988.0F), 0.0F);
  EXPECT_FLOAT_EQ(rc_throttle(988.0F + 512.0F), 0.5F);
  EXPECT_EQ(rc_throttle(988.0F + 1024.0F), 1.0F);
  EXPECT_EQ(rc_throttle(987.5F), 0.0F);
  EXPECT_EQ(rc_throttle(880.0F), 0.0F);
  EXPECT_EQ(rc_throttle(2120.0F), 1.0F);
  EXPECT_NEAR(rc_throttle(crsf_ticks_to_us(kHigh)), 1023.875F / 1024.0F, kTol);
  EXPECT_EQ(rc_throttle(crsf_ticks_to_us(kLow)), 0.0F);
}

TEST(RcInput, CentredSticks) {
  const ManualControl m = convert(centred());
  EXPECT_EQ(m.roll, 0.0F);
  EXPECT_EQ(m.pitch, 0.0F);
  EXPECT_EQ(m.yaw, 0.0F);
  EXPECT_FLOAT_EQ(m.throttle, 512.0F / 1024.0F);  // 1500 us is mid-throttle
  for (const float c : m.channels) {
    EXPECT_EQ(c, 0.0F);
  }
  EXPECT_FALSE(std::signbit(m.pitch));  // 0, not -0
}

TEST(RcInput, EveryChannelNormalisedAndAllSixteenCarried) {
  CrsfRcChannels rc = centred();
  for (std::size_t i = 0; i < rc.raw.size(); ++i) {
    rc.raw[i] = static_cast<std::uint16_t>(kLow + i * 100);
  }
  rc.raw[15] = kHigh;
  const ManualControl m = convert(rc);
  for (std::size_t i = 0; i < rc.raw.size(); ++i) {
    const float us = (static_cast<float>(rc.raw[i]) - 992.0F) * 5.0F / 8.0F + 1500.0F;
    const float x = (us - 1500.0F) / 512.0F;
    EXPECT_NEAR(m.channels[i], x < -1.0F ? -1.0F : (x > 1.0F ? 1.0F : x), kTol) << "channel index " << i;
  }
}

TEST(RcInput, SignsAndAetrMap) {
  // Each control is moved alone to full deflection; no other control changes.
  CrsfRcChannels rc = centred();
  rc.raw[0] = kHigh;  // roll stick right
  ManualControl m = convert(rc);
  EXPECT_GT(m.roll, 0.99F);  // right side down is positive roll
  EXPECT_EQ(m.pitch, 0.0F);
  EXPECT_EQ(m.yaw, 0.0F);

  rc = centred();
  rc.raw[1] = kHigh;  // pitch stick forward raises the channel: nose down, negative pitch
  m = convert(rc);
  EXPECT_LT(m.pitch, -0.99F);
  EXPECT_EQ(m.roll, 0.0F);
  rc.raw[1] = kLow;  // pulled back: nose up, positive pitch
  m = convert(rc);
  EXPECT_EQ(m.pitch, 1.0F);

  rc = centred();
  rc.raw[3] = kHigh;  // yaw stick right: nose right, positive yaw
  m = convert(rc);
  EXPECT_GT(m.yaw, 0.99F);
  EXPECT_EQ(m.roll, 0.0F);
  EXPECT_EQ(m.pitch, 0.0F);

  rc = centred();
  rc.raw[2] = kLow;  // throttle is channel 3, minimum at the low end
  m = convert(rc);
  EXPECT_EQ(m.throttle, 0.0F);
  rc.raw[2] = kHigh;
  m = convert(rc);
  EXPECT_NEAR(m.throttle, 1023.875F / 1024.0F, kTol);
}

TEST(RcInput, MapSelectsTheChannel) {
  // Roll on channel 7, pitch on 8, throttle on 9, yaw on 10: only those channels matter.
  const RcChannelMap map{7, 8, 9, 10};
  CrsfRcChannels rc = centred();
  rc.raw[0] = kHigh;  // channel 1 is not a control under this map
  rc.raw[6] = kHigh;
  rc.raw[7] = kLow;
  rc.raw[8] = kLow;
  rc.raw[9] = kLow;
  const ManualControl m = convert(rc, map);
  EXPECT_GT(m.roll, 0.99F);
  EXPECT_EQ(m.pitch, 1.0F);
  EXPECT_EQ(m.throttle, 0.0F);
  EXPECT_EQ(m.yaw, -1.0F);
  EXPECT_GT(m.channels[0], 0.99F);  // channels still carries every channel
}

TEST(RcInput, MapValidation) {
  EXPECT_TRUE(rc_channel_map_valid(kAetr));
  EXPECT_TRUE(rc_channel_map_valid({1, 16, 1, 16}));
  EXPECT_FALSE(rc_channel_map_valid({0, 2, 3, 4}));
  EXPECT_FALSE(rc_channel_map_valid({1, 17, 3, 4}));
  EXPECT_FALSE(rc_channel_map_valid({1, 2, -3, 4}));
  EXPECT_FALSE(rc_channel_map_valid({1, 2, 3, 0}));
  ManualControl m;
  m.roll = 0.25F;
  EXPECT_FALSE(crsf_to_controls(centred(), {1, 2, 3, 17}, m));
  EXPECT_EQ(m.roll, 0.25F);  // unchanged
}

TEST(RcInput, ConversionLeavesTheStampsAlone) {
  ManualControl m;
  m.t_us = 123;
  m.frames = 7;
  m.link_quality = 0.5F;
  m.link_t_us = 99;
  EXPECT_TRUE(crsf_to_controls(centred(), kAetr, m));
  EXPECT_EQ(m.t_us, 123U);
  EXPECT_EQ(m.frames, 7U);
  EXPECT_EQ(m.link_quality, 0.5F);
  EXPECT_EQ(m.link_t_us, 99U);
}

TEST(RcInput, LinkQuality) {
  EXPECT_EQ(rc_link_quality(0), 0.0F);
  EXPECT_FLOAT_EQ(rc_link_quality(50), 0.5F);
  EXPECT_EQ(rc_link_quality(100), 1.0F);
  EXPECT_EQ(rc_link_quality(255), 1.0F);  // above 100 % clamps
}

}  // namespace
}  // namespace marv::hal_target
