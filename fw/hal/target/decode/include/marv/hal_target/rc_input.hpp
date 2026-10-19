#pragma once

// CRSF channel values to the pilot input of the HAL (M2 packet P5b, decision 0040 Q3; ManualControl in
// marv/types/manual_control.hpp). Pure functions, without the Pico SDK, so the L08 T1 tests run them on the host.
//
// Conversion (constants.hpp, TBS CRSF spec "0x16 RC Channels Packed Payload"): ticks -> pulse width
// us = (ticks - 992) * 5 / 8 + 1500; every entry of ManualControl::channels is x = (us - 1500) / 512 clamped to [-1, 1];
// throttle = (us - 988) / 1024 clamped to [0, 1].
//
// Sticks. The map names the 1-based channel number of each control (channel_01 = 1; the radio's own numbering). Signs
// (core 3, FRD: positive roll is right side down, positive pitch nose up, positive yaw nose right):
//   - roll = +x: the roll stick pushed right raises the channel (EdgeTX / Betaflight convention) and means right side down;
//   - yaw = +x: the yaw stick pushed right raises the channel and means nose right;
//   - pitch = -x: the pitch stick pushed forward raises the channel and means nose down, so positive pitch (nose up) is
//     the negated channel.
// The signs are INFERRED from that convention; the bench checks each stick against the console.

#include <cstddef>
#include <cstdint>

#include "marv/hal_target/crsf.hpp"
#include "marv/types/manual_control.hpp"

namespace marv::hal_target {

// 1-based channel numbers of the four controls (the rc_channel_* parameters).
struct RcChannelMap {
  std::int32_t roll = 0;
  std::int32_t pitch = 0;
  std::int32_t throttle = 0;
  std::int32_t yaw = 0;
};

// True if every channel number is in 1 .. kCrsfRcChannels.
[[nodiscard]] bool rc_channel_map_valid(const RcChannelMap& map) noexcept;

// Rule R-1: the four stick channels, the arm channel and the angle channel (0 = no angle switch, skipped) are pairwise
// distinct. Compares only; the ranges are rc_channel_map_valid's and the composition's.
[[nodiscard]] bool rc_channels_distinct(const RcChannelMap& map, std::int32_t arm, std::int32_t angle) noexcept;

// Pulse width in microseconds of an 11-bit channel value (not clamped).
[[nodiscard]] float crsf_ticks_to_us(std::uint16_t ticks) noexcept;

// x = (us - 1500) / 512 clamped to [-1, 1].
[[nodiscard]] float rc_normalised(float us) noexcept;

// throttle = (us - 988) / 1024 clamped to [0, 1].
[[nodiscard]] float rc_throttle(float us) noexcept;

// Sets out.channels, roll, pitch, yaw and throttle from a 0x16 frame. Leaves the other fields of `out` alone. Returns
// false, leaving `out` unchanged, if the map is not valid.
[[nodiscard]] bool crsf_to_controls(const CrsfRcChannels& rc, const RcChannelMap& map, ManualControl& out) noexcept;

// Link quality in [0, 1] from the 0x14 field in percent.
[[nodiscard]] float rc_link_quality(std::uint8_t percent) noexcept;

}  // namespace marv::hal_target
