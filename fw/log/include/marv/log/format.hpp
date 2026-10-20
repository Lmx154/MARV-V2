// The MARV flight-log file format, version 2 (the onboard SD logger, unit U1 LOG-CORE; architect ruling 2026-10-18, memory
// quad-m4/architect-onboard-logger-2026-10-18). The decoder is tools/log/mlog.py; every size and offset below is derived
// field by field (sizeof, the SD block size of constants.hpp), so the layout has one source and no literal.
//
// A log file is a sequence of 512-byte sectors, each self-validating, written in order into a preallocated region:
//   sector  magic "MLOG" (4), session id u64, sector sequence u32, offset of the first record that BEGINS in this sector u16
//           (kNoRecord when none begins here), format version u16, payload (488 bytes), CRC-32 u32 over every byte before it.
//   payload a stream of records back to back; a record may span sectors (its tail leads the next sector's payload). Kind 0 in
//           a record position is padding: the rest of the sector's payload is unused (the last sector of a flush).
// A record is a whole number of 16-byte cells when it travels the ring (Imu, RateTick, ManualControl, Gap); the records the
// logger writes itself (Status, End, Telemetry and the header kinds) have the 4-byte lead kind u8, sub u8, length u16 (bytes
// of the whole record) and any length. Every integer and float is little-endian (a float is its IEEE 754 binary32 bit
// pattern); time fields named *_lo are the low 32 bits of hal_time_us(), the decoder unwraps them.
// Version 2 adds the ImuRaw kind (a 16-byte raw-count IMU record) and the HeaderImuRaw record that carries its scales; every
// version 1 kind is unchanged, so a version 1 file stays decodable and a version 2 file may hold either IMU kind.
#pragma once

#include <array>
#include <bit>
#include <climits>
#include <cstddef>
#include <cstdint>
#include <span>
#include <type_traits>

#include "marv/prim/constants.hpp"
#include "marv/types/actuator.hpp"

namespace marv::log {

// ---- sector framing -------------------------------------------------------------------------------------------------------
inline constexpr std::array<std::uint8_t, sizeof(std::uint32_t)> kSectorMagic{'M', 'L', 'O', 'G'};
inline constexpr std::uint16_t kFormatVersion = 2;
inline constexpr std::size_t kSectorBytes = prim::kSdBlockBytes;

inline constexpr std::size_t kSecOffMagic = 0;
inline constexpr std::size_t kSecOffSession = kSecOffMagic + kSectorMagic.size();
inline constexpr std::size_t kSecOffSeq = kSecOffSession + sizeof(std::uint64_t);
inline constexpr std::size_t kSecOffFirst = kSecOffSeq + sizeof(std::uint32_t);
inline constexpr std::size_t kSecOffVersion = kSecOffFirst + sizeof(std::uint16_t);
inline constexpr std::size_t kSectorHeaderBytes = kSecOffVersion + sizeof(std::uint16_t);
inline constexpr std::size_t kSecCrcBytes = sizeof(std::uint32_t);
inline constexpr std::size_t kSecOffCrc = kSectorBytes - kSecCrcBytes;
inline constexpr std::size_t kSectorPayloadBytes = kSecOffCrc - kSectorHeaderBytes;
// The first-record offset of a sector in which no record begins (a long record's middle).
inline constexpr std::uint16_t kNoRecord = static_cast<std::uint16_t>(kSectorPayloadBytes);

// ---- records --------------------------------------------------------------------------------------------------------------
enum class Kind : std::uint8_t {
  Pad,  // 0: padding, never a record
  Imu,
  RateTick,
  ManualControl,
  Gap,
  Status,
  End,
  Telemetry,
  HeaderInfo,
  HeaderKind,
  HeaderParam,
  HeaderField,
  HeaderTelemetry,
  ImuRaw,
  HeaderImuRaw,
  Count
};
inline constexpr std::size_t kKinds = static_cast<std::size_t>(Kind::Count);

[[nodiscard]] constexpr std::size_t kind_index(Kind k) noexcept { return static_cast<std::size_t>(k); }

// The kinds the tick-context writer pushes into the ring, in the order of their drop counters.
inline constexpr std::size_t kRingKinds = kind_index(Kind::ManualControl) - kind_index(Kind::Imu) + 1;
// ImuRaw is the Imu stream in its compact form: it is counted with the Imu records.
[[nodiscard]] constexpr std::size_t ring_kind_index(Kind k) noexcept {
  return k == Kind::ImuRaw ? 0 : kind_index(k) - kind_index(Kind::Imu);
}

// The ring's unit: 16 bytes. The design's slot is two cells (an IMU record fills one slot); a rate-tick record is one cell,
// which is how the 60 kB/s of the rate-tick stream (not 120 kB/s of half-empty slots) leaves the ring.
inline constexpr std::size_t kCellBytes = 2 * sizeof(std::uint64_t);
inline constexpr std::size_t kSlotBytes = 2 * kCellBytes;

[[nodiscard]] constexpr std::size_t cells_for(std::size_t bytes) noexcept { return (bytes + kCellBytes - 1) / kCellBytes; }

// Lead of the logger-written kinds: kind u8, sub u8, length u16.
inline constexpr std::size_t kLeadOffKind = 0;
inline constexpr std::size_t kLeadOffSub = 1;
inline constexpr std::size_t kLeadOffLength = 2;
inline constexpr std::size_t kLeadBytes = sizeof(std::uint32_t);

// Imu, 32 bytes: one IMU sample (ImuSample without the temperature).
//   kind u8, reserved u8, flags u16 (imu_flag bits), t_lo u32 (us), gyro x y z f32 (rad/s, FRD), accel x y z f32 (m/s2, FRD).
namespace imu {
inline constexpr std::size_t kOffFlags = 2;
inline constexpr std::size_t kOffTime = kOffFlags + sizeof(std::uint16_t);
inline constexpr std::size_t kOffGyro = kOffTime + sizeof(std::uint32_t);
inline constexpr std::size_t kOffAccel = kOffGyro + prim::kSpatialDim * sizeof(float);
inline constexpr std::size_t kBytes = kOffAccel + prim::kSpatialDim * sizeof(float);
}  // namespace imu

// ImuRaw, one cell: the same sample as Imu, as the sensor's int16 counts (the kind is the lossless compact form of Imu: the
// SI values are recomputed from the counts with the scales of the HeaderImuRaw record, as sign x count x scale in binary32).
//   kind u8, flags u8 (imu_raw::kFlag* bits), t_lo16 u16 (the low 16 bits of the time, us), gyro x y z i16, accel x y z i16.
// A count is the chip's value of the sensor axis that feeds FRD axis i (before the mounting sign): that is always an int16, and
// a saturation bit is then exactly "the count is at an int16 limit", so the saturation flags of Imu are not stored. The
// decoder chains the 16-bit times: successive ImuRaw records are less than 2^16 us apart, and a Gap record or a 32-bit stamp
// of a tick-ordered record re-anchors the chain.
namespace imu_raw {
inline constexpr std::size_t kOffFlags = sizeof(std::uint8_t);
inline constexpr std::size_t kOffTime = kOffFlags + sizeof(std::uint8_t);
inline constexpr std::size_t kOffGyro = kOffTime + sizeof(std::uint16_t);
inline constexpr std::size_t kOffAccel = kOffGyro + prim::kSpatialDim * sizeof(std::int16_t);
inline constexpr std::size_t kBytes = kOffAccel + prim::kSpatialDim * sizeof(std::int16_t);
inline constexpr std::uint8_t kFlagGyroValid = 1U << 0;
inline constexpr std::uint8_t kFlagAccelValid = 1U << 1;
inline constexpr std::uint8_t kFlagTempValid = 1U << 2;
}  // namespace imu_raw
static_assert(imu_raw::kBytes == kCellBytes, "the compact IMU record is one cell");

// RateTick, 16 bytes: one rate-loop tick.
//   kind u8, arm_state u8, gate_state u8, reserved u8, now_lo u32 (us), dshot[4] u16 (motor index 0..3, 0 = stop).
namespace rate {
inline constexpr std::size_t kOffArm = 1;
inline constexpr std::size_t kOffGate = 2;
inline constexpr std::size_t kOffTime = sizeof(std::uint32_t);
inline constexpr std::size_t kOffDshot = kOffTime + sizeof(std::uint32_t);
inline constexpr std::size_t kMotors = kQuadXMotors;
}  // namespace rate

// ManualControl, a whole number of cells: one new RC frame.
//   kind u8, reserved (3), frames u32, t_us u64 (the frame's stamp, full), roll pitch yaw throttle f32, channels f32 x
//   prim::kCrsfRcChannels, link_quality f32, link_t_lo u32.
enum class Stick : std::uint8_t { Roll, Pitch, Yaw, Throttle, Count };
namespace manual {
inline constexpr std::size_t kOffFrames = sizeof(std::uint32_t);
inline constexpr std::size_t kOffTime = kOffFrames + sizeof(std::uint32_t);
inline constexpr std::size_t kOffSticks = kOffTime + sizeof(std::uint64_t);
inline constexpr std::size_t kSticks = static_cast<std::size_t>(Stick::Count);
inline constexpr std::size_t kOffChannels = kOffSticks + kSticks * sizeof(float);
inline constexpr std::size_t kOffLinkQuality = kOffChannels + prim::kCrsfRcChannels * sizeof(float);
inline constexpr std::size_t kOffLinkTime = kOffLinkQuality + sizeof(float);
inline constexpr std::size_t kUsedBytes = kOffLinkTime + sizeof(std::uint32_t);
inline constexpr std::size_t kBytes = cells_for(kUsedBytes) * kCellBytes;
}  // namespace manual

// Gap, 32 bytes (one slot): records the tick-context writer dropped because the ring was full, pushed by the writer
// before the first record that fits after the drops (so its place in the stream is the place of the loss).
//   kind u8, reserved (3), t_lo u32 (the following record's time), dropped u32 x kRingKinds (Imu, RateTick, ManualControl).
namespace gap {
inline constexpr std::size_t kOffTime = sizeof(std::uint32_t);
inline constexpr std::size_t kOffDropped = kOffTime + sizeof(std::uint32_t);
inline constexpr std::size_t kBytes = cells_for(kOffDropped + kRingKinds * sizeof(std::uint32_t)) * kCellBytes;
}  // namespace gap

// The size of a ring record by kind; 0 for a kind that does not travel the ring.
[[nodiscard]] constexpr std::size_t ring_record_bytes(Kind k) noexcept {
  switch (k) {
    case Kind::Imu:
      return imu::kBytes;
    case Kind::ImuRaw:
      return imu_raw::kBytes;
    case Kind::RateTick:
      return kCellBytes;
    case Kind::ManualControl:
      return manual::kBytes;
    case Kind::Gap:
      return gap::kBytes;
    default:
      return 0;
  }
}

inline constexpr std::size_t kMaxRingRecordBytes =
    manual::kBytes > gap::kBytes ? (manual::kBytes > imu::kBytes ? manual::kBytes : imu::kBytes) : gap::kBytes;

// Status: the logger's periodic counters. Lead (sub = LoggerState), t_us u64 (full), then u32 counters in StatusField order.
enum class StatusField : std::uint8_t {
  DropsImu,
  DropsRate,
  DropsManual,
  PushedImu,
  PushedRate,
  PushedManual,
  RingHighWaterCells,
  RingBacklogCells,
  ChunksWritten,
  BytesWrittenLo,
  WriteFailures,
  PrepareAbsent,
  PrepareFull,
  PrepareError,
  CloseFailures,
  DiscardedCells,
  TelemetryRecords,
  Sessions,
  Count
};
inline constexpr std::size_t kStatusFields = static_cast<std::size_t>(StatusField::Count);
namespace status {
inline constexpr std::size_t kOffTime = kLeadBytes;
inline constexpr std::size_t kOffCounters = kOffTime + sizeof(std::uint64_t);
inline constexpr std::size_t kBytes = kOffCounters + kStatusFields * sizeof(std::uint32_t);
}  // namespace status

// End: the last record of a session. Lead (sub = EndReason), t_us u64, sectors_written u32 (sectors of this file up to
// and including the one holding this record), reserved u32.
enum class EndReason : std::uint8_t { Disarm, Full, Fault };
namespace end {
inline constexpr std::size_t kOffTime = kLeadBytes;
inline constexpr std::size_t kOffSectors = kOffTime + sizeof(std::uint64_t);
inline constexpr std::size_t kBytes = kOffSectors + 2 * sizeof(std::uint32_t);
}  // namespace end

// Telemetry: the slow stream (the unit-11a snapshot). Lead (sub 0), t_lo u32, then the hook's bytes (version and fields in the
// header's HeaderTelemetry and HeaderField records).
namespace telemetry {
inline constexpr std::size_t kOffTime = kLeadBytes;
inline constexpr std::size_t kOffBody = kOffTime + sizeof(std::uint32_t);
}  // namespace telemetry

// HeaderInfo: lead (sub 0), format_version u16, sector_bytes u16, session_start_us u64, image_hash[32], config_hash[32],
// schema_hash u64, tick_num u32, tick_den u32 (the tick period, us, as num/den), odr_hz f32, boot_sequence u32, boot
// report bytes: config_source, slot, config_status, fault, request, request_fault, then build_flags u8 (kBuildFlag* bits; the
// first of two bytes that were reserved, so a file written before it reads 0) and one reserved byte.
namespace info {
inline constexpr std::size_t kOffVersion = kLeadBytes;
inline constexpr std::size_t kOffSector = kOffVersion + sizeof(std::uint16_t);
inline constexpr std::size_t kOffStart = kOffSector + sizeof(std::uint16_t);
inline constexpr std::size_t kOffImage = kOffStart + sizeof(std::uint64_t);
inline constexpr std::size_t kOffConfig = kOffImage + prim::kSha256DigestBytes;
inline constexpr std::size_t kOffSchema = kOffConfig + prim::kSha256DigestBytes;
inline constexpr std::size_t kOffTickNum = kOffSchema + sizeof(std::uint64_t);
inline constexpr std::size_t kOffTickDen = kOffTickNum + sizeof(std::uint32_t);
inline constexpr std::size_t kOffOdr = kOffTickDen + sizeof(std::uint32_t);
inline constexpr std::size_t kOffBootSequence = kOffOdr + sizeof(float);
inline constexpr std::size_t kOffBoot = kOffBootSequence + sizeof(std::uint32_t);
inline constexpr std::size_t kBootBytes = 2 * sizeof(std::uint32_t);
inline constexpr std::size_t kOffBuildFlags = kOffBoot + kBootBytes - 2 * sizeof(std::uint8_t);
inline constexpr std::size_t kBytes = kOffBoot + kBootBytes;
}  // namespace info
// build_flags bit 0: the file was written by the measurement build MARV_FC3V2_FLIGHT_LOG_EMB6_OPEN (invariant L-5a), whose ring
// need not hold the fast stream for the worst SD stall (core EMB-6 is open there): not a flight image.
inline constexpr std::uint8_t kBuildFlagEmb6Open = 1U << 0;

// HeaderKind: lead (sub = the kind described), record_bytes u16 (0: variable, read the lead's length), then the kind's name
// as a zero-terminated string.
namespace header_kind {
inline constexpr std::size_t kOffRecordBytes = kLeadBytes;
inline constexpr std::size_t kOffName = kOffRecordBytes + sizeof(std::uint16_t);
}  // namespace header_kind

// HeaderParam: lead (sub = ParamType: 0 f32, 1 i32), param id u16, flags u16 (bit 0: locked), value u32 (bits), then the
// name and the unit, each zero-terminated.
namespace header_param {
inline constexpr std::size_t kOffId = kLeadBytes;
inline constexpr std::size_t kOffFlags = kOffId + sizeof(std::uint16_t);
inline constexpr std::size_t kOffValue = kOffFlags + sizeof(std::uint16_t);
inline constexpr std::size_t kOffNames = kOffValue + sizeof(std::uint32_t);
inline constexpr std::uint16_t kFlagLocked = 1;
}  // namespace header_param

// How a field of a record is read. The code is the header's, so the decoder needs no table of its own.
enum class FieldType : std::uint8_t { U8, U16, U32, U64, F32, Bytes, I16 };

// HeaderField: lead (sub = the kind the field belongs to), offset u16, size u8, FieldType u8, then the name and the unit, each
// zero-terminated.
namespace header_field {
inline constexpr std::size_t kOffOffset = kLeadBytes;
inline constexpr std::size_t kOffSize = kOffOffset + sizeof(std::uint16_t);
inline constexpr std::size_t kOffType = kOffSize + sizeof(std::uint8_t);
inline constexpr std::size_t kOffNames = kOffType + sizeof(std::uint8_t);
}  // namespace header_field

// HeaderTelemetry: lead (sub 0), version u32 (0: no telemetry stream yet), field_count u32 (the HeaderField records with sub =
// Kind::Telemetry that follow it).
namespace header_telemetry {
inline constexpr std::size_t kOffVersion = kLeadBytes;
inline constexpr std::size_t kOffFieldCount = kOffVersion + sizeof(std::uint32_t);
inline constexpr std::size_t kBytes = kOffFieldCount + sizeof(std::uint32_t);
}  // namespace header_telemetry

// HeaderImuRaw: lead (sub 0), gyro_rad_s_per_lsb f32, accel_m_s2_per_lsb f32, sign i8 x 3 (the mounting sign of FRD axis x, y,
// z: 1 or -1). The scales of the ImuRaw records of the file (the full scales the sensor runs at, in SI per count).
namespace header_imu_raw {
inline constexpr std::size_t kOffGyroScale = kLeadBytes;
inline constexpr std::size_t kOffAccelScale = kOffGyroScale + sizeof(float);
inline constexpr std::size_t kOffSigns = kOffAccelScale + sizeof(float);
inline constexpr std::size_t kBytes = kOffSigns + prim::kSpatialDim * sizeof(std::int8_t);
}  // namespace header_imu_raw

// ---- byte codec -----------------------------------------------------------------------------------------------------------
template <typename T>
constexpr void put(std::span<std::uint8_t> out, std::size_t off, T v) noexcept {
  static_assert(std::is_unsigned_v<T>);
  for (std::size_t i = 0; i < sizeof(T); ++i) {
    out[off + i] = static_cast<std::uint8_t>(v >> (CHAR_BIT * i));
  }
}
constexpr void put_f32(std::span<std::uint8_t> out, std::size_t off, float v) noexcept {
  put<std::uint32_t>(out, off, std::bit_cast<std::uint32_t>(v));
}
template <typename T>
[[nodiscard]] constexpr T get(std::span<const std::uint8_t> in, std::size_t off) noexcept {
  static_assert(std::is_unsigned_v<T>);
  T v = 0;
  for (std::size_t i = 0; i < sizeof(T); ++i) {
    v = static_cast<T>(v | (static_cast<T>(in[off + i]) << (CHAR_BIT * i)));
  }
  return v;
}
[[nodiscard]] constexpr float get_f32(std::span<const std::uint8_t> in, std::size_t off) noexcept {
  return std::bit_cast<float>(get<std::uint32_t>(in, off));
}

// ---- CRC-32 (constants.hpp kCrc32PolyReflected) -------------------------------------------------------------------------
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

[[nodiscard]] constexpr std::uint32_t crc32(std::span<const std::uint8_t> data) noexcept {
  std::uint32_t c = ~std::uint32_t{0};
  for (const std::uint8_t b : data) {
    c = detail::kCrcTable[(c ^ b) & (detail::kCrcTable.size() - 1)] ^ (c >> CHAR_BIT);
  }
  return ~c;
}

}  // namespace marv::log
