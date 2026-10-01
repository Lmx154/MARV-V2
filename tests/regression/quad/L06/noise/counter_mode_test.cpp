// L6 stage (a), T1 (decision 0012): counter mode is the sequential SplitMix64 of the null plant, and streams are keyed
// as documented. The relation, exactly: draw(seed, id, n) is the n-th (from 0) output of
// SplitMix64(mix64(seed) + id * 2^48 * gamma); for id 0, SplitMix64(mix64(seed)). The precedent's keying differs from
// the plain "SplitMix64(seed)" only by the seed pre-mix, so the equality is stated with the mixed seed.
#include <gtest/gtest.h>

#include <cstdint>

#include "marv/null_plant/prng.hpp"
#include "noise/noise.hpp"
#include "noise/stream_ids.hpp"

namespace {

namespace noise = marv::plant::noise;
using marv::null_plant::SplitMix64;

// Labelled test values: a spread of seeds (zero, small, a gamma multiple, all ones, an arbitrary 64-bit pattern) and a
// stream length long enough to cross many counter values; none is a parameter.
constexpr std::uint64_t kSeeds[] = {0U, 1U, 2U, noise::kGoldenGamma, ~std::uint64_t{0}, 0x0123456789ABCDEFULL};
constexpr std::uint64_t kLength = 4096;
constexpr std::uint16_t kIds[] = {0U, 1U, 2U, 255U, 0xFFFFU};

TEST(L06NoiseCounterMode, IdZeroEqualsSequentialSplitMix64OfTheMixedSeed) {
  for (const std::uint64_t seed : kSeeds) {
    SplitMix64 sequential{noise::mix64(seed)};
    for (std::uint64_t n = 0; n < kLength; ++n) {
      ASSERT_EQ(noise::draw(seed, noise::kStreamPrimaryImu, n), sequential.next()) << "seed " << seed << " n " << n;
    }
  }
}

TEST(L06NoiseCounterMode, EveryIdIsTheSequentialGeneratorAtItsOffsetKey) {
  for (const std::uint64_t seed : kSeeds) {
    for (const std::uint16_t id : kIds) {
      const std::uint64_t key =
          noise::mix64(seed) + (static_cast<std::uint64_t>(id) << noise::kStreamIdShift) * noise::kGoldenGamma;
      SplitMix64 sequential{key};
      for (std::uint64_t n = 0; n < kLength; ++n) {
        ASSERT_EQ(noise::draw(seed, id, n), sequential.next()) << "seed " << seed << " id " << id << " n " << n;
      }
    }
  }
}

TEST(L06NoiseCounterMode, Mix64IsTheSequentialOutputFunction) {
  for (const std::uint64_t x : kSeeds) {
    SplitMix64 sequential{x - noise::kGoldenGamma};
    EXPECT_EQ(noise::mix64(x), sequential.next());
  }
}

TEST(L06NoiseCounterMode, DrawIsAPureFunctionOfItsKey) {
  EXPECT_EQ(noise::draw(7, 3, 11), noise::draw(7, 3, 11));
  EXPECT_NE(noise::draw(7, 3, 11), noise::draw(7, 3, 12));
  EXPECT_NE(noise::draw(7, 3, 11), noise::draw(7, 4, 11));
  EXPECT_NE(noise::draw(7, 3, 11), noise::draw(8, 3, 11));
}

TEST(L06NoiseCounterMode, UniformConversionsAreExactAtTheirEnds) {
  constexpr std::uint64_t kAllOnes = ~std::uint64_t{0};
  constexpr double kTwoPow53 = 9007199254740992.0;
  EXPECT_EQ(noise::uniform_closed_open(0), 0.0);
  EXPECT_EQ(noise::uniform_closed_open(kAllOnes), (kTwoPow53 - 1.0) / kTwoPow53);
  EXPECT_LT(noise::uniform_closed_open(kAllOnes), 1.0);
  EXPECT_EQ(noise::uniform_open_closed(0), 1.0 / kTwoPow53);
  EXPECT_GT(noise::uniform_open_closed(0), 0.0);
  EXPECT_EQ(noise::uniform_open_closed(kAllOnes), 1.0);
}

TEST(L06NoiseCounterMode, SampleNormalsAreTheStreamsNormalsAtTheFixedCounters) {
  for (std::uint64_t sample = 0; sample < 64; ++sample) {
    const noise::SampleNormals block = noise::sample_normals(1, noise::kStreamPrimaryImu, sample);
    for (std::uint64_t j = 0; j < noise::kNormalsPerSample; ++j) {
      EXPECT_EQ(block[j], noise::normal(1, noise::kStreamPrimaryImu, noise::kNormalsPerSample * sample + j));
    }
  }
}

}  // namespace
