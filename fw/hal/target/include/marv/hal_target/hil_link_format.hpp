#pragma once

// The wire format of the bench HIL link measurement (HIL stage S1; fw/hal/target/src/hil_link.cpp,
// tools/target/hil_link.py is the host side). Pure: no Pico SDK. Same framing as the bench IMU log (imu_log_format.hpp):
// a record is magic (4 bytes), sequence number (u32), item count (u32), the items, CRC-32 (u32) over everything before it,
// all little-endian; the CRC and the record writer are imu_log's.
//
// Host -> board, one sequence counter over the downlink records, from 0, +1 per record:
//   Downlink record, magic "MHLD": `count` samples, each tick (u64, the host's sample number), gyro x y z (f32), accel x y z
//     (f32), flags (u32): the layout and size of imu_log's sample with a tick number in place of t_us. 1 <= count <= the
//     image's batch limit (kMaxBatch).
// Board -> host, one sequence counter over the uplink records of both kinds, from 0, +1 per record sent:
//   Report record, magic "MHLU", count = 1: tick (u64, echoed from the sample), 4 x u16 placeholder DShot values (motor
//     1..4: the low 16 bits of tick + motor index, so the host can check the content), rx_us (u64, the board clock
//     hal_time_us() when the downlink record holding the sample was read from USB). One report per received sample.
//   Status record, magic "MHLS", once per second: `count` = kStatusFields counters (u32 each), in StatusField order, counted
//     from the moment the host opened the port (DTR), not from boot.

#include <array>
#include <climits>
#include <cstddef>
#include <cstdint>

#include "marv/hal_target/imu_log_format.hpp"
#include "marv/types/actuator.hpp"

namespace marv::hal_target::hil_link {

namespace base = imu_log;

inline constexpr std::array kDownMagic{std::uint8_t{'M'}, std::uint8_t{'H'}, std::uint8_t{'L'}, std::uint8_t{'D'}};
inline constexpr std::array kReportMagic{std::uint8_t{'M'}, std::uint8_t{'H'}, std::uint8_t{'L'}, std::uint8_t{'U'}};
inline constexpr std::array kStatusMagic{std::uint8_t{'M'}, std::uint8_t{'H'}, std::uint8_t{'L'}, std::uint8_t{'S'}};

inline constexpr std::size_t kMotors = kQuadXMotors;
static_assert(kMotors % 2 == 0, "the DShot placeholders are packed two to a word");
inline constexpr std::size_t kSampleBytes = base::kSampleBytes;  // tick u64 in place of t_us
inline constexpr std::size_t kReportItemBytes = sizeof(std::uint64_t) + kMotors * sizeof(std::uint16_t) + sizeof(std::uint64_t);
inline constexpr std::size_t kReportRecordBytes = base::kHeaderBytes + kReportItemBytes + base::kCrcBytes;

enum class StatusField : std::uint8_t {
  DownRecords,      // downlink records that passed the CRC
  DownSamples,      // samples in them
  CrcFailures,      // headers followed by a whole record whose CRC did not match
  BadHeaders,       // magic followed by a count no record has (0 or above kMaxBatch)
  SeqGaps,          // downlink sequence discontinuities
  SeqLost,          // downlink records missing at those discontinuities
  TickGaps,         // sample tick discontinuities (last tick + 1 expected)
  BytesDropped,     // bytes read that belong to no valid record
  RxBytes,          // bytes read from USB
  UpReports,        // report records sent
  MaxArrivalGapUs,  // longest board-clock spacing between two valid downlink records
  MaxSendUs,        // longest single write of the uplink to the USB driver
  BoardTimeUs,      // hal_time_us(), low 32 bits, when this status was built
  MaxBatch,         // the image's batch limit
  Count
};
inline constexpr std::size_t kStatusFields = static_cast<std::size_t>(StatusField::Count);
inline constexpr std::size_t kStatusRecordBytes = base::kHeaderBytes + kStatusFields * base::kWordBytes + base::kCrcBytes;

}  // namespace marv::hal_target::hil_link
