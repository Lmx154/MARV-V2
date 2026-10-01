// The plant's seeded noise source (core 6: the plant owns every random number; quad spec L6 stage (a); decision 0012).
// Everything here is a pure function of its arguments: no state, no allocation, no exceptions, and the same bits on
// every conforming host (integer arithmetic, IEEE double arithmetic without contraction, and the vendored musl log and
// cos; the build forbids -ffast-math and floating-point contraction for this code).
//
// Counter-mode SplitMix64. The mixer is that of the null plant's SplitMix64 (sim/null_plant/include/marv/null_plant/
// prng.hpp:22-26: Steele, Lea, Flood, OOPSLA 2014; constants of Vigna's splitmix64.c, https://prng.di.unimi.it/
// splitmix64.c). The n-th draw of stream `id` under `seed` is
//
//     draw(seed, id, n) = mix64(mix64(seed) + (id * 2^48 + n + 1) * gamma)         (mod 2^64)
//
// with gamma the SplitMix64 golden-gamma increment. The relation to the sequential generator is exact:
// draw(seed, id, n) is the n-th (from 0) output of SplitMix64(mix64(seed) + id * 2^48 * gamma). For id 0 that is
// SplitMix64(mix64(seed)); the seed is mixed first so that seeds that differ by a multiple of gamma do not give shifted
// copies of one stream. Streams of one seed cannot overlap: stream ids are 16 bit and a stream's counter n is below
// 2^48 (the precondition of draw), so distinct (id, n) pairs give distinct counters before the final mix, and a
// counter step of gamma is a bijection mod 2^64 because gamma is odd.
//
// Stream ids are ABI (stream_ids.hpp).
//
// Normals. A normal consumes exactly two draws, fixed forever: the k-th normal of a stream uses draws 2k and 2k + 1,
//     u1 = uniform_open_closed(draw(2k)), u2 = uniform_closed_open(draw(2k + 1)),
//     z  = sqrt(-2 ln u1) * cos(2 pi u2)                          (Box and Muller, Ann. Math. Stat. 29, 1958).
// The sine branch is not used, so a normal never depends on its neighbour and the k-th normal is a pure function of
// (seed, id, k) (k < 2^47). The Box-Muller maps are exact for the uniforms; the uniforms are exact binary fractions.
#pragma once

#include <array>
#include <cstdint>

#include "noise/stream_ids.hpp"

namespace marv::plant::noise {

// The SplitMix64 increment, the golden ratio scaled to 64 bits (null_plant prng.hpp:22; Vigna splitmix64.c).
inline constexpr std::uint64_t kGoldenGamma = 0x9E3779B97F4A7C15ULL;

// Stream ids occupy bits 48..63 of the counter.
inline constexpr unsigned kStreamIdShift = 48U;

// The SplitMix64 output mixer (the finaliser of null_plant prng.hpp:23-26).
[[nodiscard]] constexpr std::uint64_t mix64(std::uint64_t z) noexcept {
  z = (z ^ (z >> 30U)) * 0xBF58476D1CE4E5B9ULL;
  z = (z ^ (z >> 27U)) * 0x94D049BB133111EBULL;
  return z ^ (z >> 31U);
}

// The n-th draw (n < 2^48) of stream `id` under `seed`.
[[nodiscard]] constexpr std::uint64_t draw(std::uint64_t seed, std::uint16_t id, std::uint64_t n) noexcept {
  const std::uint64_t counter = (static_cast<std::uint64_t>(id) << kStreamIdShift) + n + 1U;
  return mix64(mix64(seed) + counter * kGoldenGamma);
}

// Uniform double in (0, 1]: ((bits >> 11) + 1) * 2^-53. bits >> 11 is the top 53 bits as an integer m in [0, 2^53);
// m + 1 is in [1, 2^53], exactly representable in a double (53-bit significand), and the scaling is by a power of two,
// so the result is exact: a multiple of 2^-53 in [2^-53, 1].
[[nodiscard]] constexpr double uniform_open_closed(std::uint64_t bits) noexcept {
  constexpr double kScale = 1.0 / 9007199254740992.0;  // 2^-53, exact
  return static_cast<double>((bits >> 11U) + 1U) * kScale;
}

// Uniform double in [0, 1): (bits >> 11) * 2^-53, the same conversion without the +1: a multiple of 2^-53 in [0, 1).
[[nodiscard]] constexpr double uniform_closed_open(std::uint64_t bits) noexcept {
  constexpr double kScale = 1.0 / 9007199254740992.0;  // 2^-53, exact
  return static_cast<double>(bits >> 11U) * kScale;
}

// The k-th standard normal (k < 2^47) of stream `id` under `seed`: draws 2k and 2k + 1, as in the header comment.
[[nodiscard]] double normal(std::uint64_t seed, std::uint16_t id, std::uint64_t k) noexcept;

// One IMU sample's normals, in this fixed order (P3 of stage (a)): gyro random walk x y z, gyro white x y z, accel
// random walk x y z, accel white x y z.
inline constexpr std::uint64_t kNormalsPerSample = 12;
using SampleNormals = std::array<double, kNormalsPerSample>;

// Sample `sample`'s normals: element j is normal(seed, id, kNormalsPerSample * sample + j).
[[nodiscard]] SampleNormals sample_normals(std::uint64_t seed, std::uint16_t id, std::uint64_t sample) noexcept;

}  // namespace marv::plant::noise
