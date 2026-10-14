// Encodings shared by the test composition (which writes what vertical_input handed it into DShot values) and the
// tests (which compute the expected values). Every code is a valid raw DShot value: 48 + [0, 1999].
#pragma once

#include <array>
#include <cstdint>
#include <cstring>

#include <marv/types/vertical_state.hpp>

namespace verticaltest {

inline constexpr std::uint16_t kCodeBase = 48;
inline constexpr std::uint32_t kCodeSpan = 2000;

inline constexpr std::uint32_t kFlagHave = 1u;
inline constexpr std::uint32_t kFlagValid = 2u;
inline constexpr std::uint32_t kFlagStampMatches = 4u;

inline std::uint32_t bits(float f) {
  std::uint32_t u = 0;
  std::memcpy(&u, &f, sizeof u);
  return u;
}

using Words = std::array<std::uint32_t, 5>;

// The fields of one hand-over as words: stamp lo, stamp hi, z, v_z, valid.
inline Words words_of(const marv::VerticalState<float>& v) {
  return {static_cast<std::uint32_t>(v.t_us & 0xffffffffu), static_cast<std::uint32_t>(v.t_us >> 32), bits(v.z_ned_m),
          bits(v.vz_ned_m_s), v.valid ? 1u : 0u};
}

inline std::uint32_t hash_of(const marv::VerticalState<float>& v) {
  std::uint32_t h = 2166136261u;
  for (const std::uint32_t word : words_of(v)) {
    for (unsigned b = 0; b < 4; ++b) {
      h ^= (word >> (8 * b)) & 0xffu;
      h *= 16777619u;
    }
  }
  return h;
}

inline std::uint16_t code(std::uint32_t v) { return static_cast<std::uint16_t>(kCodeBase + v % kCodeSpan); }

// The four DShot codes of a tick: [calls of vertical_input since the previous tick, flags, hash low, hash high].
struct Row {
  std::uint16_t calls;
  std::uint16_t flags;
  std::uint16_t hash_lo;
  std::uint16_t hash_hi;
};

inline Row row_of(std::uint32_t calls, std::uint32_t flags, std::uint32_t hash) {
  return Row{code(calls), code(flags), code(hash), code(hash / kCodeSpan)};
}

}  // namespace verticaltest
