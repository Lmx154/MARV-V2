// T1 tests of the FC3v2 DShot600 output logic (fw/hal/target/decode/dshot.hpp, M1 P10): the frame encoder, the pad
// permutation, the packing of four frames into the PIO words, the PIO bit timing and the panic loop's stop frames.
// Expected values are restated from the Betaflight DShot notes (16-bit frame SSSSSSSSSSSTCCCC, CRC = (v ^ v>>4 ^ v>>8) & 0xF
// over the 12 bits above it; DShot600: 600 kbit/s, bit 1.67 us, T0H 0.625 us, T1H 1.25 us, frame 26.72 us) and computed
// by hand, not read from the code under test.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

#include <marv/hal_target/dshot.hpp>
#include <marv/types/actuator.hpp>

namespace marv::hal_target {
namespace {

// Rebuilds the 16-bit frame of pad `p` from the words, nibble k being the k-th bit period on the wire.
std::uint16_t frame_on_pad(const FrameWords& w, std::size_t p) {
  std::uint16_t f = 0;
  for (unsigned k = 0; k < 16; ++k) {
    const std::uint32_t nibble = (w[k / 8] >> (4 * (k % 8))) & 0xFU;
    f = static_cast<std::uint16_t>((f << 1) | ((nibble >> p) & 1U));
  }
  return f;
}

DshotValue dv(std::uint16_t raw) { return *DshotValue::from_raw(raw); }

TEST(Dshot, FrameKnownAnswers) {
  // 12-bit payload = value << 1 | telemetry; the CRC folds its three nibbles.
  EXPECT_EQ(dshot_frame(0), 0x0000);
  EXPECT_EQ(dshot_frame(48), 0x0606);    // payload 0x060, CRC (0x060 ^ 0x006 ^ 0x000) & 0xF = 6
  EXPECT_EQ(dshot_frame(1047), 0x82E4);  // payload 0x82E, CRC (0x82E ^ 0x082 ^ 0x008) & 0xF = 4
  EXPECT_EQ(dshot_frame(2047), 0xFFEE);  // payload 0xFFE, CRC (0xFFE ^ 0x0FF ^ 0x00F) & 0xF = 0xE
  EXPECT_EQ(dshot_frame(1047, true), 0x82F5);
  EXPECT_EQ(dshot_frame(2047, true), 0xFFFF);
}

TEST(Dshot, CrcFoldsThe12BitPayloadForEveryValue) {
  for (unsigned v = 0; v <= 2047; ++v) {
    for (unsigned t = 0; t <= 1; ++t) {
      const unsigned payload = (v << 1) | t;
      const unsigned crc = (payload ^ (payload >> 4) ^ (payload >> 8)) & 0xFU;
      EXPECT_EQ(dshot_frame(static_cast<std::uint16_t>(v), t == 1), (payload << 4) | crc);
    }
  }
}

TEST(Dshot, PadMapValidation) {
  PadMap out{3, 2, 1, 0};
  EXPECT_TRUE(make_pad_map({0, 1, 2, 3}, out));  // identity
  EXPECT_EQ(out, (PadMap{0, 1, 2, 3}));
  EXPECT_TRUE(make_pad_map({2, 0, 3, 1}, out));
  EXPECT_EQ(out, (PadMap{2, 0, 3, 1}));

  const PadMap kept = out;
  EXPECT_FALSE(make_pad_map({0, 1, 1, 3}, out));   // duplicate
  EXPECT_FALSE(make_pad_map({0, 0, 0, 0}, out));
  EXPECT_FALSE(make_pad_map({0, 1, 2, 4}, out));   // out of range
  EXPECT_FALSE(make_pad_map({0, 1, 2, -1}, out));
  EXPECT_FALSE(make_pad_map({1, 2, 3, 256}, out));  // would wrap to 0 if narrowed before the check
  EXPECT_EQ(out, kept);

  EXPECT_TRUE(valid(PadMap{1, 0, 3, 2}));
  EXPECT_FALSE(valid(PadMap{1, 0, 3, 3}));
  EXPECT_FALSE(valid(PadMap{1, 0, 3, 4}));
}

TEST(Dshot, PackedWordsKnownAnswer) {
  // Only the motor on pad 0 sends 2047 (0xFFEE = 1111 1111 1110 1110, most significant bit first); the others stop.
  const std::array<DshotValue, 4> motor{dv(2047), DshotValue::stop(), DshotValue::stop(), DshotValue::stop()};
  const FrameWords w = pack_frames(PadMap{0, 1, 2, 3}, motor);
  EXPECT_EQ(w[0], 0x11111111U);  // bit periods 0-7: 1 on pad 0
  EXPECT_EQ(w[1], 0x01110111U);  // bit periods 8-15: 1 1 1 0 1 1 1 0, the first in the lowest nibble
}

TEST(Dshot, EveryPermutationPutsEachMotorsFrameOnItsPad) {
  const std::array<DshotValue, 4> motor{dv(48), dv(500), dv(1047), dv(2047)};
  PadMap pad{0, 1, 2, 3};
  int permutations = 0;
  do {
    ASSERT_TRUE(valid(pad));
    const FrameWords w = pack_frames(pad, motor);
    for (std::size_t i = 0; i < 4; ++i) {
      EXPECT_EQ(frame_on_pad(w, pad[i]), dshot_frame(motor[i].raw()));
    }
    ++permutations;
  } while (std::next_permutation(pad.begin(), pad.end()));
  EXPECT_EQ(permutations, 24);
}

TEST(Dshot, PanicWordsAreStopFramesOnEveryPadWhateverThePadMap) {
  const FrameWords stop = stop_words();
  const std::array<DshotValue, 4> all_stop{};
  PadMap pad{0, 1, 2, 3};
  do {
    EXPECT_EQ(stop, pack_frames(pad, all_stop));
  } while (std::next_permutation(pad.begin(), pad.end()));
  for (std::size_t p = 0; p < 4; ++p) {
    EXPECT_EQ(frame_on_pad(stop, p), dshot_frame(DshotValue::stop().raw()));
    EXPECT_EQ(frame_on_pad(stop, p), 0x0000);  // value 0, telemetry 0, CRC 0
  }
  // The control: a throttle on one motor changes the words, so the comparison above can fail.
  const std::array<DshotValue, 4> one{DshotValue::stop(), DshotValue::stop(), dv(48), DshotValue::stop()};
  EXPECT_NE(stop, pack_frames(PadMap{0, 1, 2, 3}, one));
}

TEST(Dshot, PioTimingIsTheDshot600BitOn8Cycles) {
  const PioTiming t = pio_timing();
  EXPECT_EQ(t.cycles_per_bit, 8U);
  EXPECT_EQ(t.all_high, 3U);  // T0H = 3/8 of the bit
  EXPECT_EQ(t.data, 3U);      // T1H = 6/8
  EXPECT_EQ(t.all_low, 2U);
  EXPECT_EQ(t.high_delay, 2U);
  EXPECT_EQ(t.data_delay, 2U);
  EXPECT_EQ(t.low_delay, 0U);  // the shift is the second low cycle

  // clk_sys 150 MHz (RP2354B at its default): 150e6 / (600e3 x 8) = 31.25 = 31 + 64/256, exactly.
  const PioClockDivider d = pio_clock_divider(150000000U);
  EXPECT_EQ(d.integer, 31U);
  EXPECT_EQ(d.frac8, 64U);
  const double pio_hz = 150e6 / (31.0 + 64.0 / 256.0);
  EXPECT_DOUBLE_EQ(pio_hz, 4.8e6);
  const double bit_us = 1e6 * t.cycles_per_bit / pio_hz;
  EXPECT_NEAR(bit_us, 1.67, 0.005);                         // Betaflight table: 1.67 us
  EXPECT_NEAR(1e6 * t.all_high / pio_hz, 0.625, 1e-9);      // T0H
  EXPECT_NEAR(1e6 * (t.all_high + t.data) / pio_hz, 1.25, 1e-9);  // T1H
  EXPECT_NEAR(16 * bit_us, 26.72, 0.1);

  EXPECT_EQ(pio_clock_divider(1000000U).integer, 0U);  // below the bit clock: refused
}

TEST(Dshot, FrameTimeAndStopInterval) {
  EXPECT_EQ(frame_time_us(), 27U);              // 26.67 us rounded up
  EXPECT_EQ(stop_interval_us(2, 3125, 24), 261U);  // the target tick 3125/24 us, divisor 2: 260.4 us rounded up
  EXPECT_EQ(stop_interval_us(2, 625, 4), 313U);    // the PLACEHOLDER tick 625/4 us: 312.5 us
  EXPECT_EQ(stop_interval_us(1, 1, 1), frame_time_us());  // never below one frame
}

}  // namespace
}  // namespace marv::hal_target
