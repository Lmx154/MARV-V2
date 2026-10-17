#pragma once

// The wire format of the bench IMU log (M3 packet P3a; fw/hal/target/src/imu_log.cpp, tools/target/imu_log.py decodes it).
// Pure: no Pico SDK, so a host build can encode a frame and check it against the decoder.
//
// A record is: magic (4 bytes), sequence number (u32), item count (u32), the items, CRC-32 (u32) over everything before
// it. Every integer and float is little-endian (a float is its IEEE 754 binary32 bit pattern as a u32). One sequence
// counter runs over data and status records together, from 0, +1 per record sent, so a lost record of either kind is a
// gap.
//   Data record, magic "MILD": `count` samples, each t_us (u64), gyro x y z (f32, rad/s, FRD), accel x y z (f32, m/s^2,
//     FRD), flags (u32, imu_flag bits). The samples are the ones quad_fc's tick receives, in order; temp_k is not logged.
//   Status record, magic "MILS": `count` = kStatusFields counters (u32 each), in StatusField order, cumulative since boot.

#include <array>
#include <bit>
#include <climits>
#include <cstddef>
#include <cstdint>
#include <span>

#include "marv/prim/constants.hpp"
#include "marv/types/imu_sample.hpp"

namespace marv::hal_target::imu_log {

inline constexpr std::array kDataMagic{std::uint8_t{'M'}, std::uint8_t{'I'}, std::uint8_t{'L'}, std::uint8_t{'D'}};
inline constexpr std::array kStatusMagic{std::uint8_t{'M'}, std::uint8_t{'I'}, std::uint8_t{'L'}, std::uint8_t{'S'}};
inline constexpr std::size_t kMagicBytes = kDataMagic.size();
inline constexpr std::size_t kWordBytes = sizeof(std::uint32_t);
inline constexpr std::size_t kHeaderBytes = kMagicBytes + 2 * kWordBytes;
inline constexpr std::size_t kCrcBytes = kWordBytes;
// t_us, gyro and accel components, flags.
inline constexpr std::size_t kSampleBytes =
    sizeof(std::uint64_t) + 2 * prim::kSpatialDim * sizeof(float) + sizeof(std::uint32_t);

// The status record's counters, all cumulative since boot (u32, wrapping).
enum class StatusField : std::uint8_t {
  RingOverflows,   // samples the ring refused because the drain had not kept up (dropped from the log)
  RingHighWater,   // most samples the ring has held at once
  DeadlineMisses,  // hal_imu_deadline_misses()
  FifoOverflows,   // IMU FIFO overruns seen at an interrupt entry
  TimingFaults,    // rate-loop timing faults
  StampClamps,     // stamps raised to keep t_us strictly increasing
  OrderFaults,     // samples whose t_us did not exceed the previous one
  SpiErrors,       // FIFO words with a bad tag
  Delivered,       // samples the driver handed to the composition (low 32 bits)
  Logged,          // samples taken out of the ring: packed into records sent before this status record, or discarded while no host held the port
  Count
};
inline constexpr std::size_t kStatusFields = static_cast<std::size_t>(StatusField::Count);

[[nodiscard]] constexpr std::size_t data_record_bytes(std::size_t samples) noexcept {
  return kHeaderBytes + samples * kSampleBytes + kCrcBytes;
}
inline constexpr std::size_t kStatusRecordBytes = kHeaderBytes + kStatusFields * kWordBytes + kCrcBytes;

namespace detail {
[[nodiscard]] constexpr std::array<std::uint32_t, std::size_t{1} << CHAR_BIT> crc_table() noexcept {
  std::array<std::uint32_t, std::size_t{1} << CHAR_BIT> t{};
  for (std::size_t i = 0; i < t.size(); ++i) {
    auto c = static_cast<std::uint32_t>(i);
    for (int bit = 0; bit < CHAR_BIT; ++bit) {
      c = (c & 1U) != 0 ? (c >> 1) ^ prim::kCrc32PolyReflected : c >> 1;
    }
    t[i] = c;
  }
  return t;
}
inline constexpr std::array<std::uint32_t, std::size_t{1} << CHAR_BIT> kCrcTable = crc_table();
}  // namespace detail

// CRC-32 of `data` (kCrc32PolyReflected: initial all ones, complemented result).
[[nodiscard]] constexpr std::uint32_t crc32(std::span<const std::uint8_t> data) noexcept {
  std::uint32_t c = ~std::uint32_t{0};
  for (const std::uint8_t b : data) {
    c = detail::kCrcTable[(c ^ b) & (detail::kCrcTable.size() - 1)] ^ (c >> CHAR_BIT);
  }
  return ~c;
}

// Writes one record into `out` (pre: large enough for the header, the items and the CRC).
class RecordWriter {
 public:
  explicit RecordWriter(std::span<std::uint8_t> out) noexcept : out_(out) {}

  void begin(const std::array<std::uint8_t, kMagicBytes>& magic, std::uint32_t seq, std::uint32_t count) noexcept {
    n_ = 0;
    for (const std::uint8_t b : magic) {
      out_[n_++] = b;
    }
    put_u32(seq);
    put_u32(count);
  }
  void put_u32(std::uint32_t v) noexcept {
    for (std::size_t i = 0; i < sizeof(v); ++i) {
      out_[n_++] = static_cast<std::uint8_t>(v >> (CHAR_BIT * i));
    }
  }
  void put_u64(std::uint64_t v) noexcept {
    for (std::size_t i = 0; i < sizeof(v); ++i) {
      out_[n_++] = static_cast<std::uint8_t>(v >> (CHAR_BIT * i));
    }
  }
  void put_f32(float v) noexcept { put_u32(std::bit_cast<std::uint32_t>(v)); }
  void put_sample(const ImuSample& s) noexcept {
    put_u64(s.t_us);
    for (std::size_t i = 0; i < prim::kSpatialDim; ++i) {
      put_f32(s.gyro_rad_s[i]);
    }
    for (std::size_t i = 0; i < prim::kSpatialDim; ++i) {
      put_f32(s.accel_m_s2[i]);
    }
    put_u32(s.flags);
  }
  // Appends the CRC; returns the record's length in bytes.
  [[nodiscard]] std::size_t end() noexcept {
    put_u32(crc32(out_.first(n_)));
    return n_;
  }

 private:
  std::span<std::uint8_t> out_;
  std::size_t n_ = 0;
};

}  // namespace marv::hal_target::imu_log
