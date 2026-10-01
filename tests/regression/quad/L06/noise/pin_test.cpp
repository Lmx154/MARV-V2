// L6 stage (a), T1 (decision 0012): the pinned bit pattern of the stream. "The same seed gives a bit-identical run"
// includes every host and both build types: the SHA-256 of the first kPinnedNormals normals of (seed kPinSeed, id 0),
// each as its IEEE 754 binary64 bit pattern in 8 bytes, least significant byte first, must equal kExpectedSha256.
//
// Changing kExpectedSha256, kPinSeed, kPinnedNormals or anything the digest depends on (the generator, the uniform
// conversion, the Box-Muller form, the vendored musl, the build flags) needs a decision record in docs/decisions/ in
// the same change (core 7.3): every recorded noisy run changes with it. host-debug and host-release must both match.
#include <gtest/gtest.h>

#include <array>
#include <bit>
#include <cstdint>
#include <string>

#include "noise/noise.hpp"
#include "marv/sim/sha256.hpp"
#include "support.hpp"

namespace {

namespace noise = marv::plant::noise;

// Labelled test values: the seed (any fixed one) and M = 2^16 normals, enough to run the stream through many binades
// of u1 and many angles, small enough to hash in debug in a blink.
constexpr std::uint64_t kPinSeed = 1;
constexpr std::uint64_t kPinnedNormals = 1U << 16U;
constexpr const char* kExpectedSha256 = "3abc7725a20377a80f385013cab5662c86f5bbcdfe3d96a8cb44b0154ed80852";

std::string digest_of_stream(std::uint64_t seed, std::uint64_t count) {
  marv::sim::Sha256 sha;
  for (std::uint64_t k = 0; k < count; ++k) {
    const auto bits = std::bit_cast<std::uint64_t>(noise::normal(seed, noise::kStreamPrimaryImu, k));
    std::array<std::uint8_t, 8> bytes{};
    for (std::size_t i = 0; i < bytes.size(); ++i) {
      bytes[i] = static_cast<std::uint8_t>(bits >> (8U * i));
    }
    sha.update(bytes.data(), bytes.size());
  }
  return sha.hex();
}

TEST(L06NoisePin, FirstNormalsHaveThePinnedDigest) {
  EXPECT_EQ(digest_of_stream(kPinSeed, kPinnedNormals), std::string{kExpectedSha256});
}

// Control: the digest depends on the stream. A different seed, and one fewer normal, give different digests.
TEST(L06NoisePinControl, DigestChangesWithSeedAndLength) {
  EXPECT_NE(digest_of_stream(kPinSeed + 1U, kPinnedNormals), std::string{kExpectedSha256});
  EXPECT_NE(digest_of_stream(kPinSeed, kPinnedNormals - 1U), std::string{kExpectedSha256});
}

// The SHA-256 itself against the FIPS 180-4 / NIST CSRC example values.
TEST(L06NoisePin, Sha256MatchesTheStandardVectors) {
  marv::sim::Sha256 empty;
  EXPECT_EQ(empty.hex(), "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
  marv::sim::Sha256 abc;
  const std::array<std::uint8_t, 3> text{'a', 'b', 'c'};
  abc.update(text.data(), text.size());
  EXPECT_EQ(abc.hex(), "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  marv::sim::Sha256 two_blocks;
  const std::string long_text = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";
  two_blocks.update(reinterpret_cast<const std::uint8_t*>(long_text.data()), long_text.size());
  EXPECT_EQ(two_blocks.hex(), "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");
}

}  // namespace
