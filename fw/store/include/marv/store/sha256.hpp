// SHA-256 for the firmware (NIST FIPS 180-4 section 6.2; the constants are prim::kSha256*, constants.hpp): the config
// file's payload hash, the config hash (core IF-1) and the slot read-back (decision 0053, ground segment GS-11). Static
// state only, no allocation; usable in constant expressions.
#pragma once

#include <array>
#include <climits>
#include <cstddef>
#include <cstdint>
#include <span>

#include "marv/prim/constants.hpp"

namespace marv::store {

inline constexpr std::size_t kDigestBytes = prim::kSha256DigestBytes;
using Digest = std::array<std::uint8_t, kDigestBytes>;

class Sha256 {
 public:
  constexpr void update(std::span<const std::uint8_t> data) noexcept {
    for (const std::uint8_t b : data) {
      block_[fill_] = b;
      ++fill_;
      if (fill_ == block_.size()) {
        compress();
        fill_ = 0;
      }
    }
    bytes_ += data.size();
  }

  // The digest of everything updated so far. The object is spent afterwards.
  [[nodiscard]] constexpr Digest finish() noexcept {
    const std::uint64_t bits = bytes_ * CHAR_BIT;
    const std::array<std::uint8_t, 1> pad{prim::kSha256PadByte};
    update(pad);
    const std::array<std::uint8_t, 1> zero{0};
    while (fill_ != block_.size() - prim::kSha256LengthBytes) {
      update(zero);
    }
    std::array<std::uint8_t, prim::kSha256LengthBytes> length{};
    for (std::size_t i = 0; i < length.size(); ++i) {
      length[length.size() - 1 - i] = static_cast<std::uint8_t>(bits >> (CHAR_BIT * i));
    }
    update(length);
    Digest out{};
    std::size_t k = 0;
    for (const std::uint32_t word : h_) {
      for (std::size_t i = 0; i < sizeof(word); ++i) {
        out[k] = static_cast<std::uint8_t>(word >> (CHAR_BIT * (sizeof(word) - 1 - i)));
        ++k;
      }
    }
    return out;
  }

 private:
  static constexpr unsigned kWordBits = sizeof(std::uint32_t) * CHAR_BIT;
  static constexpr std::size_t kBlockWords = prim::kSha256BlockBytes / sizeof(std::uint32_t);
  // The working variables a..h of FIPS 180-4 section 6.2.2.
  enum Var : std::size_t { kA, kB, kC, kD, kE, kF, kG, kH, kVars };
  static_assert(kVars == prim::kSha256H0.size());
  static_assert(kBlockWords == prim::kSha256LagOldest);

  [[nodiscard]] static constexpr std::uint32_t rotr(std::uint32_t x, unsigned n) noexcept {
    return (x >> n) | (x << (kWordBits - n));
  }

  constexpr void compress() noexcept {
    std::array<std::uint32_t, prim::kSha256K.size()> w{};
    for (std::size_t t = 0; t < kBlockWords; ++t) {
      std::uint32_t word = 0;
      for (std::size_t i = 0; i < sizeof(word); ++i) {
        word = (word << CHAR_BIT) | block_[(t * sizeof(word)) + i];
      }
      w[t] = word;
    }
    for (std::size_t t = kBlockWords; t < w.size(); ++t) {
      const std::uint32_t x0 = w[t - prim::kSha256LagSigma0];
      const std::uint32_t x1 = w[t - prim::kSha256LagSigma1];
      const std::uint32_t s0 = rotr(x0, prim::kSha256SmallSigma0Rotr[0]) ^ rotr(x0, prim::kSha256SmallSigma0Rotr[1]) ^
                               (x0 >> prim::kSha256SmallSigma0Shr);
      const std::uint32_t s1 = rotr(x1, prim::kSha256SmallSigma1Rotr[0]) ^ rotr(x1, prim::kSha256SmallSigma1Rotr[1]) ^
                               (x1 >> prim::kSha256SmallSigma1Shr);
      w[t] = s1 + w[t - prim::kSha256LagPlain] + s0 + w[t - prim::kSha256LagOldest];
    }
    std::array<std::uint32_t, kVars> v = h_;
    for (std::size_t t = 0; t < w.size(); ++t) {
      const std::uint32_t e = v[kE];
      const std::uint32_t a = v[kA];
      const std::uint32_t big1 = rotr(e, prim::kSha256BigSigma1Rotr[0]) ^ rotr(e, prim::kSha256BigSigma1Rotr[1]) ^
                                 rotr(e, prim::kSha256BigSigma1Rotr[2]);
      const std::uint32_t ch = (e & v[kF]) ^ (~e & v[kG]);
      const std::uint32_t t1 = v[kH] + big1 + ch + prim::kSha256K[t] + w[t];
      const std::uint32_t big0 = rotr(a, prim::kSha256BigSigma0Rotr[0]) ^ rotr(a, prim::kSha256BigSigma0Rotr[1]) ^
                                 rotr(a, prim::kSha256BigSigma0Rotr[2]);
      const std::uint32_t maj = (a & v[kB]) ^ (a & v[kC]) ^ (v[kB] & v[kC]);
      v[kH] = v[kG];
      v[kG] = v[kF];
      v[kF] = e;
      v[kE] = v[kD] + t1;
      v[kD] = v[kC];
      v[kC] = v[kB];
      v[kB] = a;
      v[kA] = t1 + big0 + maj;
    }
    for (std::size_t i = 0; i < h_.size(); ++i) {
      h_[i] += v[i];
    }
  }

  std::array<std::uint32_t, kVars> h_ = prim::kSha256H0;
  std::array<std::uint8_t, prim::kSha256BlockBytes> block_{};
  std::size_t fill_ = 0;
  std::uint64_t bytes_ = 0;
};

[[nodiscard]] constexpr Digest sha256(std::span<const std::uint8_t> data) noexcept {
  Sha256 h;
  h.update(data);
  return h.finish();
}

}  // namespace marv::store
