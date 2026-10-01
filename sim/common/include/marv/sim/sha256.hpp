// SHA-256 for the host simulation and its tests: the digest of a run log or of a noise stream, so that "the same seed
// gives a bit-identical run" is a one-line comparison. Not for fw/ (host only; no use of it in the flight image).
//
// Algorithm: NIST FIPS 180-4, "Secure Hash Standard (SHS)", section 6.2 (SHA-256); the constants are those of sections
// 4.2.2 and 5.3.3. The known-answer vectors are checked in the L06 noise pin test
// (L06NoisePin.Sha256MatchesTheStandardVectors).
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace marv::sim {

class Sha256 {
 public:
  void update(const std::uint8_t* data, std::size_t len) {
    for (std::size_t i = 0; i < len; ++i) {
      block_[fill_++] = data[i];
      if (fill_ == kBlockBytes) {
        compress();
        fill_ = 0;
      }
    }
    total_bytes_ += len;
  }

  std::string hex() {
    const std::uint64_t bits = total_bytes_ * 8U;
    const std::uint8_t pad = 0x80;
    update(&pad, 1);
    const std::uint8_t zero = 0;
    while (fill_ != kBlockBytes - 8U) {
      update(&zero, 1);
    }
    std::array<std::uint8_t, 8> length{};
    for (std::size_t i = 0; i < 8; ++i) {
      length[i] = static_cast<std::uint8_t>(bits >> (56U - 8U * i));
    }
    update(length.data(), length.size());
    static constexpr char kDigits[] = "0123456789abcdef";
    std::string out;
    for (const std::uint32_t word : h_) {
      for (unsigned shift = 28; shift <= 28; shift -= 4) {
        out.push_back(kDigits[(word >> shift) & 0xFU]);
        if (shift == 0) {
          break;
        }
      }
    }
    return out;
  }

 private:
  static constexpr std::size_t kBlockBytes = 64;

  static std::uint32_t rotr(std::uint32_t x, unsigned n) { return (x >> n) | (x << (32U - n)); }

  void compress() {
    static constexpr std::uint32_t kK[64] = {
        0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
        0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
        0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
        0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
        0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
        0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
        0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
        0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};
    std::array<std::uint32_t, 64> w{};
    for (std::size_t i = 0; i < 16; ++i) {
      w[i] = (static_cast<std::uint32_t>(block_[4 * i]) << 24U) | (static_cast<std::uint32_t>(block_[4 * i + 1]) << 16U) |
             (static_cast<std::uint32_t>(block_[4 * i + 2]) << 8U) | static_cast<std::uint32_t>(block_[4 * i + 3]);
    }
    for (std::size_t i = 16; i < 64; ++i) {
      const std::uint32_t s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3U);
      const std::uint32_t s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10U);
      w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    std::array<std::uint32_t, 8> v = h_;
    for (std::size_t i = 0; i < 64; ++i) {
      const std::uint32_t s1 = rotr(v[4], 6) ^ rotr(v[4], 11) ^ rotr(v[4], 25);
      const std::uint32_t ch = (v[4] & v[5]) ^ (~v[4] & v[6]);
      const std::uint32_t t1 = v[7] + s1 + ch + kK[i] + w[i];
      const std::uint32_t s0 = rotr(v[0], 2) ^ rotr(v[0], 13) ^ rotr(v[0], 22);
      const std::uint32_t maj = (v[0] & v[1]) ^ (v[0] & v[2]) ^ (v[1] & v[2]);
      const std::uint32_t t2 = s0 + maj;
      v[7] = v[6];
      v[6] = v[5];
      v[5] = v[4];
      v[4] = v[3] + t1;
      v[3] = v[2];
      v[2] = v[1];
      v[1] = v[0];
      v[0] = t1 + t2;
    }
    for (std::size_t i = 0; i < 8; ++i) {
      h_[i] += v[i];
    }
  }

  std::array<std::uint32_t, 8> h_{0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
                                  0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
  std::array<std::uint8_t, kBlockBytes> block_{};
  std::size_t fill_ = 0;
  std::uint64_t total_bytes_ = 0;
};

}  // namespace marv::sim
