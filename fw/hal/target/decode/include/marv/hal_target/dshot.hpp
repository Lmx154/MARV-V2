#pragma once

// The DShot600 output of the FC3v2 (M1 packet P10; fw/hal/target/src/motor_output.cpp), without the Pico SDK so the L09
// T1 tests build it on the host: the frame encoder, the pad permutation, the packing of four frames into the words the
// PIO state machine shifts out, the PIO bit timing and the stop-frame rule of the panic loop.
//
// Frame (constants.hpp, Betaflight DShot notes): 11 bits of value, 1 bit of telemetry request, 4 bits of CRC =
// (v ^ v>>4 ^ v>>8) & 0xF over the 12 bits above it, sent most significant bit first. The firmware always sends the
// telemetry bit 0 (plain DShot; bidirectional DShot is deferred).
//
// PIO program. One state machine drives the four motor pads together, one bit period per loop of three phases:
//   all pads high for T0H, the pads of the motors whose bit is 1 stay high until T1H, all pads low for the rest.
// Each loop shifts one nibble out of the OSR (bit p of the nibble is the bit of the motor on pad p); a frame is
// kDshotFrameBits nibbles = two 32-bit words, autopulled. The program shifts nothing and never loops on its own: with an
// empty FIFO it waits with every pad low, so a stalled firmware produces no frames (invariant I4).

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "marv/prim/constants.hpp"
#include "marv/types/actuator.hpp"
#include "marv/types/time.hpp"

namespace marv::hal_target {

inline constexpr std::size_t kMotorPads = kQuadXMotors;

// Logical motor index (motor number - 1) to pad (board output OUTn, n = 0..3): pad[i] is the pad of motor i + 1.
using PadMap = std::array<std::uint8_t, kMotorPads>;

// Nibble count of a frame and the words that carry it: a word holds the nibbles of kDshotFrameBits/kNibblesPerWord.
inline constexpr unsigned kNibbleBits = kMotorPads;
inline constexpr unsigned kWordBits = std::numeric_limits<std::uint32_t>::digits;
inline constexpr unsigned kNibblesPerWord = kWordBits / kNibbleBits;
inline constexpr std::size_t kFrameWords = prim::kDshotFrameBits / kNibblesPerWord;
static_assert(kNibbleBits * kNibblesPerWord == kWordBits && kFrameWords * kNibblesPerWord == prim::kDshotFrameBits);
static_assert(prim::kDshotThrottleBits + prim::kDshotTelemetryBits + prim::kDshotCrcBits == prim::kDshotFrameBits);

using FrameWords = std::array<std::uint32_t, kFrameWords>;

// The 16-bit frame of a value (0 or kDshotThrottleMin..kDshotThrottleMax; `value` is 11 bits wide).
[[nodiscard]] constexpr std::uint16_t dshot_frame(std::uint16_t value, bool telemetry = false) noexcept {
  const unsigned payload = (static_cast<unsigned>(value) << prim::kDshotTelemetryBits) | (telemetry ? 1U : 0U);
  const unsigned crc_mask = (1U << prim::kDshotCrcBits) - 1U;
  const unsigned crc = (payload ^ (payload >> prim::kDshotCrcBits) ^ (payload >> (2U * prim::kDshotCrcBits))) & crc_mask;
  return static_cast<std::uint16_t>((payload << prim::kDshotCrcBits) | crc);
}

// True iff `pad` is a permutation of the pads 0 .. kMotorPads - 1 (every pad used once).
[[nodiscard]] bool valid(const PadMap& pad) noexcept;

// The pad map of four parameter values (motor 1 first); false, and `out` unchanged, if a value is outside
// 0 .. kMotorPads - 1 or the four are not a permutation.
[[nodiscard]] bool make_pad_map(const std::array<std::int32_t, kMotorPads>& values, PadMap& out) noexcept;

// The words of one frame per motor, in transmission order (the first nibble first, in the least significant bits of
// word 0): nibble k carries frame bit (kDshotFrameBits - 1 - k) of the motor on pad p in its bit p. pre: valid(pad).
[[nodiscard]] FrameWords pack_frames(const PadMap& pad, const std::array<DshotValue, kMotorPads>& motor) noexcept;

// The panic loop's words: DshotValue::stop() on every motor. Independent of the pad map (a permutation of equal frames).
[[nodiscard]] FrameWords stop_words() noexcept;

// PIO loop timing, in PIO cycles. cycles_per_bit is the smallest count that puts the T0H and T1H edges on whole cycles
// (the denominator of the 3/8 and 6/8 fractions); the delays are the instruction delay fields.
struct PioTiming {
  unsigned cycles_per_bit;
  unsigned all_high;    // cycles with every pad high: T0H
  unsigned data;        // cycles the pad of a 1 stays high alone: T1H - T0H
  unsigned all_low;     // cycles with every pad low: the rest of the bit
  unsigned high_delay;  // delay of the all-high instruction (it takes 1 + delay cycles)
  unsigned data_delay;
  unsigned low_delay;   // delay of the all-low instruction; the shift instruction takes the other low cycle
};

[[nodiscard]] constexpr PioTiming pio_timing() noexcept {
  PioTiming t{};
  t.cycles_per_bit = prim::kDshotBitFractionDen;
  t.all_high = prim::kDshotT0hOfBitNum;
  t.data = prim::kDshotT1hOfBitNum - prim::kDshotT0hOfBitNum;
  t.all_low = prim::kDshotBitFractionDen - prim::kDshotT1hOfBitNum;
  t.high_delay = t.all_high - 1U;
  t.data_delay = t.data - 1U;
  t.low_delay = t.all_low - 2U;
  return t;
}
static_assert(pio_timing().all_high + pio_timing().data + pio_timing().all_low == pio_timing().cycles_per_bit);
static_assert(pio_timing().all_high >= 1U && pio_timing().data >= 1U && pio_timing().all_low >= 2U);

// PIO clock divider for clk_sys = clk_hz: clk_hz / (kDshot600BitRateHz x cycles_per_bit) as the PIO's 16.8 fixed point
// (the fraction is 8 bits, pio_sm_set_clkdiv_int_frac8), rounded to the nearest step. integer == 0 if the divider would
// be below 1 or above the 16-bit integer field.
struct PioClockDivider {
  std::uint32_t integer;
  std::uint8_t frac8;
};
[[nodiscard]] PioClockDivider pio_clock_divider(std::uint32_t clk_hz) noexcept;

// Duration of one frame, rounded up to whole microseconds (16 bit periods).
[[nodiscard]] constexpr TimeUs frame_time_us() noexcept {
  const double us = prim::kDshotFrameBits * prim::kMicrosecondsPerSecond / static_cast<double>(prim::kDshot600BitRateHz);
  const auto whole = static_cast<TimeUs>(us);
  return static_cast<double>(whole) < us ? whole + 1U : whole;
}

// The panic loop's stop-frame interval: the nominal rate-execution period, rate_loop_divisor x tick_period_num_us /
// tick_period_den, rounded up to whole microseconds and never below one frame. pre: divisor >= 1, num >= 1, den >= 1.
[[nodiscard]] TimeUs stop_interval_us(std::int32_t rate_loop_divisor, std::int32_t tick_period_num_us,
                                      std::int32_t tick_period_den) noexcept;

}  // namespace marv::hal_target
