// The per-kind field lists the header carries (header.hpp record_schema), derived from the layout of format.hpp.
#include <array>
#include <cstddef>
#include <cstdint>

#include "marv/log/header.hpp"

namespace marv::log {
namespace {

[[nodiscard]] constexpr std::uint8_t type_bytes(FieldType t) noexcept {
  switch (t) {
    case FieldType::U8:
      return sizeof(std::uint8_t);
    case FieldType::U16:
      return sizeof(std::uint16_t);
    case FieldType::U32:
    case FieldType::F32:
      return sizeof(std::uint32_t);
    case FieldType::U64:
      return sizeof(std::uint64_t);
    case FieldType::Bytes:
      return 0;
  }
  return 0;
}

[[nodiscard]] constexpr FieldDesc fd(const char* name, const char* unit, std::size_t offset, FieldType type) noexcept {
  return FieldDesc{name, unit, static_cast<std::uint16_t>(offset), type_bytes(type), type};
}

constexpr auto kImuFields = std::to_array<FieldDesc>({
    fd("flags", "1", imu::kOffFlags, FieldType::U16),
    fd("t_lo_us", "us", imu::kOffTime, FieldType::U32),
    fd("gyro_x", "rad/s", imu::kOffGyro, FieldType::F32),
    fd("gyro_y", "rad/s", imu::kOffGyro + sizeof(float), FieldType::F32),
    fd("gyro_z", "rad/s", imu::kOffGyro + 2 * sizeof(float), FieldType::F32),
    fd("accel_x", "m/s2", imu::kOffAccel, FieldType::F32),
    fd("accel_y", "m/s2", imu::kOffAccel + sizeof(float), FieldType::F32),
    fd("accel_z", "m/s2", imu::kOffAccel + 2 * sizeof(float), FieldType::F32),
});

constexpr std::array<const char*, rate::kMotors> kDshotNames{"dshot_1", "dshot_2", "dshot_3", "dshot_4"};
constexpr std::size_t kRateLeadFields = 2 + 1;  // arm_state, gate_state, now_lo_us
[[nodiscard]] constexpr std::array<FieldDesc, kRateLeadFields + rate::kMotors> rate_fields() noexcept {
  std::array<FieldDesc, kRateLeadFields + rate::kMotors> f{};
  std::size_t n = 0;
  f[n++] = fd("arm_state", "1", rate::kOffArm, FieldType::U8);
  f[n++] = fd("gate_state", "1", rate::kOffGate, FieldType::U8);
  f[n++] = fd("now_lo_us", "us", rate::kOffTime, FieldType::U32);
  for (std::size_t i = 0; i < rate::kMotors; ++i) {
    f[n++] = fd(kDshotNames[i], "1", rate::kOffDshot + i * sizeof(std::uint16_t), FieldType::U16);
  }
  return f;
}
constexpr auto kRateFields = rate_fields();

constexpr std::array<const char*, prim::kCrsfRcChannels> kChannelNames{
    "ch_01", "ch_02", "ch_03", "ch_04", "ch_05", "ch_06", "ch_07", "ch_08",
    "ch_09", "ch_10", "ch_11", "ch_12", "ch_13", "ch_14", "ch_15", "ch_16"};

constexpr std::size_t kManualLeadFields = 2;
constexpr std::size_t kManualTailFields = 2;
constexpr std::size_t kManualFieldCount = kManualLeadFields + manual::kSticks + prim::kCrsfRcChannels + kManualTailFields;

[[nodiscard]] constexpr std::array<FieldDesc, kManualFieldCount> manual_fields() noexcept {
  std::array<FieldDesc, kManualFieldCount> f{};
  const std::array<const char*, manual::kSticks> sticks{"roll", "pitch", "yaw", "throttle"};
  std::size_t n = 0;
  f[n++] = fd("frames", "1", manual::kOffFrames, FieldType::U32);
  f[n++] = fd("t_us", "us", manual::kOffTime, FieldType::U64);
  for (std::size_t i = 0; i < manual::kSticks; ++i) {
    f[n++] = fd(sticks[i], "1", manual::kOffSticks + i * sizeof(float), FieldType::F32);
  }
  for (std::size_t i = 0; i < prim::kCrsfRcChannels; ++i) {
    f[n++] = fd(kChannelNames[i], "1", manual::kOffChannels + i * sizeof(float), FieldType::F32);
  }
  f[n++] = fd("link_quality", "1", manual::kOffLinkQuality, FieldType::F32);
  f[n++] = fd("link_t_lo_us", "us", manual::kOffLinkTime, FieldType::U32);
  return f;
}
constexpr auto kManualFields = manual_fields();

constexpr auto kGapFields = std::to_array<FieldDesc>({
    fd("t_lo_us", "us", gap::kOffTime, FieldType::U32),
    fd("dropped_imu", "1", gap::kOffDropped, FieldType::U32),
    fd("dropped_rate_tick", "1", gap::kOffDropped + sizeof(std::uint32_t), FieldType::U32),
    fd("dropped_manual_control", "1", gap::kOffDropped + 2 * sizeof(std::uint32_t), FieldType::U32),
});
static_assert(kGapFields.size() == 1 + kRingKinds, "one drop count per ring kind");

constexpr std::array<const char*, kStatusFields> kStatusNames{
    "drops_imu",       "drops_rate_tick",    "drops_manual_control", "pushed_imu",      "pushed_rate_tick",
    "pushed_manual_control", "ring_high_water_cells", "ring_backlog_cells", "chunks_written", "bytes_written_lo",
    "write_failures",  "prepare_absent",     "prepare_full",         "prepare_error",   "close_failures",
    "discarded_cells", "telemetry_records",  "sessions"};

constexpr std::size_t kStatusFieldCount = 2 + kStatusFields;
[[nodiscard]] constexpr std::array<FieldDesc, kStatusFieldCount> status_fields() noexcept {
  std::array<FieldDesc, kStatusFieldCount> f{};
  std::size_t n = 0;
  f[n++] = fd("state", "1", kLeadOffSub, FieldType::U8);
  f[n++] = fd("t_us", "us", status::kOffTime, FieldType::U64);
  for (std::size_t i = 0; i < kStatusFields; ++i) {
    f[n++] = fd(kStatusNames[i], "1", status::kOffCounters + i * sizeof(std::uint32_t), FieldType::U32);
  }
  return f;
}
constexpr auto kStatusFieldList = status_fields();

constexpr auto kEndFields = std::to_array<FieldDesc>({
    fd("reason", "1", kLeadOffSub, FieldType::U8),
    fd("t_us", "us", end::kOffTime, FieldType::U64),
    fd("sectors", "1", end::kOffSectors, FieldType::U32),
});

constexpr auto kTelemetryLead = std::to_array<FieldDesc>({
    fd("t_lo_us", "us", telemetry::kOffTime, FieldType::U32),
});

constexpr auto kSchema = std::to_array<KindSchema>({
    {Kind::Imu, "imu", imu::kBytes, kImuFields},
    {Kind::RateTick, "rate_tick", kCellBytes, kRateFields},
    {Kind::ManualControl, "manual_control", manual::kBytes, kManualFields},
    {Kind::Gap, "gap", gap::kBytes, kGapFields},
    {Kind::Status, "status", 0, kStatusFieldList},
    {Kind::End, "end", end::kBytes, kEndFields},
    {Kind::Telemetry, "telemetry", 0, kTelemetryLead},
});

}  // namespace

std::span<const KindSchema> record_schema() noexcept { return kSchema; }

}  // namespace marv::log
