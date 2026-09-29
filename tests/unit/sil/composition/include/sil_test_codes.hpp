// Encodings shared by the test composition (which writes them into DShot values) and the tests (which decode them).
// Every code is a valid raw DShot value: 48 + [0, 1999].
#pragma once

#include <array>
#include <cstdint>

namespace siltest {

inline constexpr std::uint16_t kCodeBase = 48;
inline constexpr std::uint16_t kCodeSpan = 2000;  // codes are kCodeBase + [0, kCodeSpan)

// The eight words of one sample in a fixed order: gyro xyz, accel xyz, temp_k, flags, each as its bit pattern.
using SampleWords = std::array<std::uint32_t, 8>;

// FNV-1a over the words' bytes, folded into a DShot-valid code: distinguishes any two samples that differ in a bit,
// up to a 1 in 2000 collision chance.
inline std::uint16_t sample_code(const SampleWords& w) {
  std::uint32_t h = 2166136261u;
  for (const std::uint32_t word : w) {
    for (unsigned b = 0; b < 4; ++b) {
      h ^= (word >> (8 * b)) & 0xffu;
      h *= 16777619u;
    }
  }
  return static_cast<std::uint16_t>(kCodeBase + h % kCodeSpan);
}

inline std::uint16_t clamp_code(std::uint32_t v) {
  return static_cast<std::uint16_t>(kCodeBase + (v < kCodeSpan ? v : kCodeSpan - 1));
}

// Probe of one parameter record, selected by the value of parameter fixture_gate_count (id 6) when it is a valid id.
// bits 0-1 origin, 2-4 method, 5 lock equals the default's, 6 unit pointer equals the default's,
// 7 source is "sil-override".
inline constexpr std::uint32_t kProbeMethodShift = 2;
inline constexpr std::uint32_t kProbeLockBit = 32;
inline constexpr std::uint32_t kProbeUnitBit = 64;
inline constexpr std::uint32_t kProbeSourceBit = 128;

}  // namespace siltest
