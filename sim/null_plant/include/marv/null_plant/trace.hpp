// Trace hashing and the null plant's own output checks.
//
// Trace hash: FNV-1a, 64 bit (Fowler-Noll-Vo; offset basis 14695981039346656037, prime 1099511628211). Per tick the
// bytes fed are, in this order, all little-endian: n (u64), t_us (u64), each DShot value (u16, motor order 1..N),
// each servo value (u16, servo order). The hash is printed as 16 lower-case hex digits.
#pragma once

#include <cstdint>

namespace marv::null_plant {

class Fnv1a64 {
 public:
  void byte(std::uint8_t b) noexcept {
    h_ ^= b;
    h_ *= 1099511628211ULL;
  }
  void u16(std::uint16_t v) noexcept {
    byte(static_cast<std::uint8_t>(v & 0xFFU));
    byte(static_cast<std::uint8_t>(v >> 8U));
  }
  void u64(std::uint64_t v) noexcept {
    for (unsigned i = 0; i < 8U; ++i) {
      byte(static_cast<std::uint8_t>((v >> (8U * i)) & 0xFFU));
    }
  }
  [[nodiscard]] std::uint64_t value() const noexcept { return h_; }

 private:
  std::uint64_t h_ = 14695981039346656037ULL;
};

// The plant's own computation of floor(n * num / den), in 128-bit arithmetic. Deliberately not hal_sim's function.
[[nodiscard]] std::uint64_t reference_stamp_us(std::uint32_t num_us, std::uint32_t den, std::uint64_t n) noexcept;

// A DShot value the actuator struct may carry: 0 (stop) or 48..2047 (Betaflight DShot notes; core 4). Written from
// the specification, not from the firmware's constants, so the check is independent of the code it checks.
[[nodiscard]] bool dshot_value_ok(std::uint16_t v) noexcept;

}  // namespace marv::null_plant
