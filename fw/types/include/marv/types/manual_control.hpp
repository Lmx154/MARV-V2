#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "marv/prim/constants.hpp"
#include "marv/types/time.hpp"

namespace marv {

// The pilot's input as the receiver backend last delivered it (decision 0040 Q3), one pull read (hal_manual_control()).
// The producer converts and never decides validity; the consumer (the arming and failsafe machine) owns staleness.
//
//   - t_us: hal_time_us() when the newest CRC-valid RC frame completed. frames counts the valid RC frames since init
//     (it wraps; a producer skips 0); frames == 0 means none yet, and then every field is 0 (the value-initialised
//     struct). A consumer ages the frame as hal_time_us() - t_us and counts a t_us later than hal_time_us() as no valid
//     frame.
//   - roll, pitch, yaw: decoded from their channels, in [-1, 1], FRD sense (core 3, Signs): positive roll is right
//     side down, positive pitch nose up, positive yaw nose right. throttle in [0, 1] likewise. Centred sticks are 0;
//     minimum throttle is 0.
//   - channels: all prim::kCrsfRcChannels RC channels, channel_01 at index 0, each normalised by the producer to
//     [-1, 1] as x = (us - 1500) / 512, clamped, where us is the channel's pulse width in microseconds. The consumer
//     reads the switches from here by index (the arm switch is channel 5, index 4). Which channel carries which stick,
//     and every channel order and sign, is backend configuration (checked on the bench by stick against console).
//   - link_quality in [0, 1] with its own stamp link_t_us (hal_time_us() when the newest link-statistics frame
//     completed; 0 with link_quality 0 before any). No decision reads it.
struct ManualControl {
  TimeUs t_us = 0;
  std::uint32_t frames = 0;
  float roll = 0;
  float pitch = 0;
  float yaw = 0;
  float throttle = 0;
  std::array<float, prim::kCrsfRcChannels> channels{};
  float link_quality = 0;
  TimeUs link_t_us = 0;
};

static_assert(std::is_trivially_copyable_v<ManualControl> && std::is_standard_layout_v<ManualControl>);

}  // namespace marv
