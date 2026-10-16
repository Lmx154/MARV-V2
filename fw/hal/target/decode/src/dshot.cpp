#include "marv/hal_target/dshot.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "marv/prim/constants.hpp"
#include "marv/types/actuator.hpp"
#include "marv/types/time.hpp"

namespace marv::hal_target {

bool valid(const PadMap& pad) noexcept {
  std::array<bool, kMotorPads> used{};
  for (const std::uint8_t p : pad) {
    if (p >= kMotorPads || used[p]) {
      return false;
    }
    used[p] = true;
  }
  return true;
}

bool make_pad_map(const std::array<std::int32_t, kMotorPads>& values, PadMap& out) noexcept {
  PadMap pad{};
  for (std::size_t i = 0; i < kMotorPads; ++i) {
    if (values[i] < 0 || values[i] >= static_cast<std::int32_t>(kMotorPads)) {
      return false;
    }
    pad[i] = static_cast<std::uint8_t>(values[i]);
  }
  if (!valid(pad)) {
    return false;
  }
  out = pad;
  return true;
}

FrameWords pack_frames(const PadMap& pad, const std::array<DshotValue, kMotorPads>& motor) noexcept {
  std::array<std::uint16_t, kMotorPads> frame_of_pad{};
  for (std::size_t i = 0; i < kMotorPads; ++i) {
    frame_of_pad[pad[i]] = dshot_frame(motor[i].raw());
  }
  FrameWords words{};
  for (unsigned k = 0; k < prim::kDshotFrameBits; ++k) {
    std::uint32_t nibble = 0;
    for (std::size_t p = 0; p < kMotorPads; ++p) {
      const std::uint32_t bit = (frame_of_pad[p] >> (prim::kDshotFrameBits - 1U - k)) & 1U;
      nibble |= bit << p;
    }
    words[k / kNibblesPerWord] |= nibble << ((k % kNibblesPerWord) * kNibbleBits);
  }
  return words;
}

FrameWords stop_words() noexcept {
  PadMap identity{};
  std::array<DshotValue, kMotorPads> stop{};
  for (std::size_t i = 0; i < kMotorPads; ++i) {
    identity[i] = static_cast<std::uint8_t>(i);
    stop[i] = DshotValue::stop();
  }
  return pack_frames(identity, stop);
}

PioClockDivider pio_clock_divider(std::uint32_t clk_hz) noexcept {
  const std::uint64_t steps = std::uint64_t{1} << std::numeric_limits<std::uint8_t>::digits;
  const std::uint64_t pio_hz = std::uint64_t{prim::kDshot600BitRateHz} * pio_timing().cycles_per_bit;
  const std::uint64_t x256 = (std::uint64_t{clk_hz} * steps + pio_hz / 2U) / pio_hz;
  const std::uint64_t integer = x256 / steps;
  if (integer < 1U || integer > std::numeric_limits<std::uint16_t>::max()) {
    return {0, 0};
  }
  return {static_cast<std::uint32_t>(integer), static_cast<std::uint8_t>(x256 % steps)};
}

TimeUs stop_interval_us(std::int32_t rate_loop_divisor, std::int32_t tick_period_num_us,
                        std::int32_t tick_period_den) noexcept {
  const auto num = static_cast<std::uint64_t>(rate_loop_divisor) * static_cast<std::uint64_t>(tick_period_num_us);
  const auto den = static_cast<std::uint64_t>(tick_period_den);
  const TimeUs period = (num + den - 1U) / den;
  return period < frame_time_us() ? frame_time_us() : period;
}

}  // namespace marv::hal_target
