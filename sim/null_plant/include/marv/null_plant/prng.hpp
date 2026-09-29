// The null plant's one random source (core 6: the plant owns every random number).
//
// Algorithm: SplitMix64 (Steele, Lea, Flood, "Fast splittable pseudorandom number generators", OOPSLA 2014; the
// constants are those of the reference implementation by Sebastiano Vigna, https://prng.di.unimi.it/splitmix64.c).
// It uses only 64-bit unsigned integer arithmetic (modulo 2^64), so the stream is identical on every conforming
// platform and compiler. std::mt19937 distributions are not used: their float outputs are not portable.
//
// Float conversion, exact: take the top 24 bits of the 64-bit output as an integer m in [0, 2^24) and return
// m * 2^-24. Every such m is exactly representable in a float and the scaling is by a power of two, so the result is
// exact: a uniform float in [0, 1) on the grid of multiples of 2^-24.
#pragma once

#include <cstdint>

namespace marv::null_plant {

class SplitMix64 {
 public:
  explicit constexpr SplitMix64(std::uint64_t seed) noexcept : state_{seed} {}

  [[nodiscard]] constexpr std::uint64_t next() noexcept {
    state_ += 0x9E3779B97F4A7C15ULL;
    std::uint64_t z = state_;
    z = (z ^ (z >> 30U)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27U)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31U);
  }

  // Uniform float in [0, 1), multiples of 2^-24.
  [[nodiscard]] constexpr float next_unit() noexcept {
    constexpr float kScale = 1.0F / 16777216.0F;  // 2^-24, exact
    return static_cast<float>(next() >> 40U) * kScale;
  }

 private:
  std::uint64_t state_;
};

}  // namespace marv::null_plant
