#pragma once

// The wire format of the bench HIL image (HIL stage S3a, decision 0046 items 3-5; fw/hal/target/src/hil.cpp,
// tools/target/hil_fc.py is the host side). Pure: no Pico SDK. The HIL S1 framing (hil_link_format.hpp, imu_log_format.hpp):
// a record is magic (4 bytes), sequence number (u32), item count (u32), the items, CRC-32 (u32) over everything before it,
// all little-endian (a float is its IEEE 754 binary32 bit pattern); the CRC and the record writer are imu_log's.
//
// Host -> board, one sequence counter over the downlink records of both kinds, from 0 at each port open, +1 per record:
//   Sample record, magic "MHSM": `count` samples, 1 <= count <= the image's batch limit (HelloField::MaxBatch); each tick
//     (u64, the host's sample number, +1 per sample), t_us (u64, the sample stamp: the SIL's floor(tick x num / den) for the
//     tick period num / den us), gyro x y z (f32, rad/s, FRD), accel x y z (f32, m/s^2, FRD), flags (u32, imu_flag bits).
//     temp_k is not carried: the board hands the composition 0 (quad_fc reads neither temp_k nor TempValid).
//   Manual-control record, magic "MHMC", count = 1: frames (u32), roll, pitch, yaw, throttle (f32), the kCrsfRcChannels
//     channels (f32, channel_01 first), link_quality (f32): ManualControl's fields without its stamps. The board stamps the
//     record with hal_time_us() when it was read from USB (t_us, and link_t_us; t_us stays 0 when frames is 0), as the
//     receiver stamps a frame, and publishes it through manual_control_publish.
// Board -> host, one sequence counter over the uplink records of all kinds, from 0 at each port open, +1 per record sent:
//   Hello record, magic "MHHS", sent first after the port opens: `count` u32 words, kHelloFields in HelloField order, then
//     the image version string (HelloField::VersionBytes bytes of ASCII, no terminator) packed four bytes to a word,
//     little-endian, the last word padded with zero bytes.
//   DShot record, magic "MHDS": `count` reports, one per hal_actuators_write (one per rate tick), in order: tick (u64, the
//     tick number of the sample whose composition tick wrote), the four DShot values (u16 each, logical motor 1 first),
//     board_us (u64, hal_time_us() when hal_actuators_write ran: the end of that rate tick, after the composition read the
//     clock and hal_manual_control()).
//   ManualControl-arrival record, magic "MHMA", count = 1, one per ManualControl record published: frames (u32, as the host
//     sent it), seq (u32, the downlink sequence number of the record), t_us (u64, the board stamp it was published with:
//     hal_time_us() when the read that completed it returned). With board_us of the DShot reports it gives the frame age
//     each rate tick saw.
//   Status record, magic "MHST", once per second while the port is open: `count` = kStatusFields counters (u32 each), in
//     StatusField order, cumulative since boot.
// Receiver records, only from an image built with MARV_FC3V2_HIL_RECEIVER (HIL stage S4, decision 0046 item 4): there the
// composition's ManualControl comes from the board's own CRSF receiver (receiver.hpp, its one writer) and every
// manual-control record from the host is refused (counted in ManualRefused, never published).
//   Receiver-boot record, magic "MHRB", sent directly after each hello: `count` = kReceiverBootFields u32 words, in
//     ReceiverBootField order (receiver.hpp ReceiverBoot). Its presence is how a host tells this image from the S3 one.
//   Receiver-frame record, magic "MHRF", count = 1, one per change of the receiver's ManualControl the background loop sees
//     (receiver_stats(): a new RC frame or link-statistics frame), while the port is open: frames (u32), t_us (u64), roll,
//     pitch, yaw, throttle (f32), the kCrsfRcChannels channels (f32, channel_01 first), link_quality (f32), link_t_us (u64),
//     seen_us (u64, hal_time_us() when the loop read it), missed (u32: RC frames published since the previous record of this
//     connection that the loop did not see, frames - previous frames - 1 when positive; not corrected for the counter's wrap).
//     With board_us of the DShot reports it gives the sticks and the frame age each rate tick saw (the newest t_us at or
//     before board_us), for the SIL replay of stage S5.
//   Receiver-status record, magic "MHRS", directly after each status record: `count` = kReceiverStatusFields counters (u32
//     each), in ReceiverStatusField order, cumulative since boot.

#include <array>
#include <cstddef>
#include <cstdint>

#include "marv/hal_target/imu_log_format.hpp"
#include "marv/prim/constants.hpp"
#include "marv/types/actuator.hpp"
#include "marv/types/manual_control.hpp"

namespace marv::hal_target::hil {

namespace base = imu_log;

inline constexpr std::array kSampleMagic{std::uint8_t{'M'}, std::uint8_t{'H'}, std::uint8_t{'S'}, std::uint8_t{'M'}};
inline constexpr std::array kManualMagic{std::uint8_t{'M'}, std::uint8_t{'H'}, std::uint8_t{'M'}, std::uint8_t{'C'}};
inline constexpr std::array kHelloMagic{std::uint8_t{'M'}, std::uint8_t{'H'}, std::uint8_t{'H'}, std::uint8_t{'S'}};
inline constexpr std::array kDshotMagic{std::uint8_t{'M'}, std::uint8_t{'H'}, std::uint8_t{'D'}, std::uint8_t{'S'}};
inline constexpr std::array kArrivalMagic{std::uint8_t{'M'}, std::uint8_t{'H'}, std::uint8_t{'M'}, std::uint8_t{'A'}};
inline constexpr std::array kStatusMagic{std::uint8_t{'M'}, std::uint8_t{'H'}, std::uint8_t{'S'}, std::uint8_t{'T'}};
inline constexpr std::array kReceiverBootMagic{std::uint8_t{'M'}, std::uint8_t{'H'}, std::uint8_t{'R'}, std::uint8_t{'B'}};
inline constexpr std::array kReceiverFrameMagic{std::uint8_t{'M'}, std::uint8_t{'H'}, std::uint8_t{'R'}, std::uint8_t{'F'}};
inline constexpr std::array kReceiverStatusMagic{std::uint8_t{'M'}, std::uint8_t{'H'}, std::uint8_t{'R'}, std::uint8_t{'S'}};

inline constexpr std::size_t kMotors = kQuadXMotors;
// tick, t_us, gyro and accel components, flags.
inline constexpr std::size_t kSampleBytes =
    sizeof(std::uint64_t) + sizeof(std::uint64_t) + 2 * prim::kSpatialDim * sizeof(float) + sizeof(std::uint32_t);
// frames, roll, pitch, yaw, throttle, channels, link_quality.
inline constexpr std::size_t kManualBytes = sizeof(ManualControl::frames) + sizeof(ManualControl::roll) +
                                            sizeof(ManualControl::pitch) + sizeof(ManualControl::yaw) +
                                            sizeof(ManualControl::throttle) + sizeof(ManualControl::channels) +
                                            sizeof(ManualControl::link_quality);
// tick, the four DShot values, board_us.
inline constexpr std::size_t kDshotItemBytes = sizeof(std::uint64_t) + kMotors * sizeof(std::uint16_t) + sizeof(std::uint64_t);
// frames, seq, t_us.
inline constexpr std::size_t kArrivalItemBytes = sizeof(std::uint32_t) + sizeof(std::uint32_t) + sizeof(std::uint64_t);
// frames, t_us, roll, pitch, yaw, throttle, channels, link_quality, link_t_us, seen_us, missed.
inline constexpr std::size_t kReceiverFrameBytes = sizeof(ManualControl::frames) + sizeof(ManualControl::t_us) +
                                                   sizeof(ManualControl::roll) + sizeof(ManualControl::pitch) +
                                                   sizeof(ManualControl::yaw) + sizeof(ManualControl::throttle) +
                                                   sizeof(ManualControl::channels) + sizeof(ManualControl::link_quality) +
                                                   sizeof(ManualControl::link_t_us) + sizeof(std::uint64_t) +
                                                   sizeof(std::uint32_t);

[[nodiscard]] constexpr std::size_t record_bytes(std::size_t count, std::size_t item_bytes) noexcept {
  return base::kHeaderBytes + count * item_bytes + base::kCrcBytes;
}

enum class HelloField : std::uint8_t {
  SchemaHashLow,    // kParamSchemaHash of the loaded table, low 32 bits
  SchemaHashHigh,   // and high 32 bits
  ParamCount,       // kParamCount
  ParamValuesCrc,   // CRC-32 (imu_log crc32) over the loaded table's values in ParamId order, each 4 bytes little-endian: the
                    // f32 bit pattern of an F32 record, the i32 of an I32 record
  TickPeriodNumUs,  // the tick period num / den us of the loaded table (tick_period_num_us, tick_period_den)
  TickPeriodDen,
  RateLoopDivisor,  // rate_loop_divisor of the loaded table: a DShot report every this many ticks
  MaxBatch,         // the largest sample record the image accepts, in samples
  DeadlineUs,       // D_HIL: a gap of at least this between sample records is an IMU deadline miss
  ClkSysHz,         // clk_sys, the rate of the cycle counts in the status
  VersionBytes,     // length of the version string that follows
  Count
};
inline constexpr std::size_t kHelloFields = static_cast<std::size_t>(HelloField::Count);

enum class StatusField : std::uint8_t {
  SampleRecords,          // sample records that passed the CRC
  Samples,                // samples in them
  ManualRecords,          // manual-control records published
  ManualRefused,          // manual-control records that passed the CRC but break the ManualControl contract, or reached the
                          // receiver image (not published)
  CrcFailures,            // headers followed by a whole record whose CRC did not match
  BadHeaders,             // magic followed by a count no record of that kind has
  SeqGaps,                // downlink sequence discontinuities
  SeqLost,                // downlink records missing at those discontinuities
  TickGaps,               // sample tick discontinuities (last tick + 1 expected)
  BytesDropped,           // bytes read that belong to no valid record
  RxBytes,                // bytes read from USB
  TicksRun,               // composition ticks run
  QueueOverflows,         // samples refused because the tick queue was full (not ticked)
  Reports,                // DShot reports sent
  ReportOverflows,        // DShot reports lost because the uplink ring was full
  ArmingState,            // composition::arming_status(), the ArmState code (arming.hpp)
  RateExecutions,         // composition::rate_counts().executions, low 32 bits
  RateTimingFaults,       // composition::rate_counts().timing_faults
  AttitudeFaults,         // composition::attitude_counts().faults
  AttitudeTimingFaults,   // composition::attitude_counts().timing_faults
  DeadlineMisses,         // hal_imu_deadline_misses()
  MaxTickCycles,          // longest composition::tick call, clk_sys cycles (DWT CYCCNT)
  MaxFrameCycles,         // longest tick interrupt entry (the ticks of one record), clk_sys cycles
  MaxArrivalGapUs,        // longest board-clock spacing between two sample records
  MaxSendUs,              // longest single write of the uplink to the USB driver
  BoardTimeUs,            // hal_time_us(), low 32 bits, when this status was built
  Count
};
inline constexpr std::size_t kStatusFields = static_cast<std::size_t>(StatusField::Count);
inline constexpr std::size_t kStatusRecordBytes = record_bytes(kStatusFields, base::kWordBytes);

enum class ReceiverBootField : std::uint8_t {
  BaudRequested,  // ReceiverBoot::baud_requested
  BaudActual,     // ReceiverBoot::baud_actual
  IrqPriority,    // ReceiverBoot::irq_priority
  Count
};
inline constexpr std::size_t kReceiverBootFields = static_cast<std::size_t>(ReceiverBootField::Count);

enum class ReceiverStatusField : std::uint8_t {
  RcFrames,         // receiver_stats(): CrsfCounters::rc_frames
  LinkStatsFrames,  // CrsfCounters::link_stats_frames
  SkippedFrames,    // CrsfCounters::skipped_frames
  BadCrc,           // CrsfCounters::bad_crc
  BadLength,        // CrsfCounters::bad_length
  DroppedBytes,     // CrsfCounters::dropped_bytes
  UartErrors,       // ReceiverStats::uart_errors
  FramesSent,       // receiver-frame records sent
  FramesMissed,     // the sum of their `missed`
  Count
};
inline constexpr std::size_t kReceiverStatusFields = static_cast<std::size_t>(ReceiverStatusField::Count);

}  // namespace marv::hal_target::hil
