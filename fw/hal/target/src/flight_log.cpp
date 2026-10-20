// The onboard SD logger in the flight image (marv/hal_target/flight_log.hpp) and its tick hooks (marv/hal_target/log_tick.hpp).
#include "marv/hal_target/flight_log.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <span>

#include "marv/hal/hal.hpp"
#include "marv/hal_target/log_tick.hpp"
#include "marv/log/format.hpp"
#include "marv/prim/constants.hpp"
#include "marv/types/actuator.hpp"
#if defined(MARV_FC3V2_FLIGHT_LOG_EMB6_OPEN)
#include <cstdio>
#include <string_view>

#include "marv/hal_target/console.hpp"
#endif

#ifndef MARV_FC3V2_LOG_STAGING_BYTES
#error "MARV_FC3V2_LOG_STAGING_BYTES is set by fw/hal/target/CMakeLists.txt"
#endif

namespace marv::hal_target {
namespace {

using log::FieldDesc;
using log::FieldType;

static_assert(MARV_FC3V2_LOG_STAGING_BYTES % log::kSectorBytes == 0 && MARV_FC3V2_LOG_STAGING_BYTES / log::kSectorBytes >= 2,
              "the staging chunk is at least two whole sectors (logger.hpp)");
constexpr std::size_t kStagingBytes = MARV_FC3V2_LOG_STAGING_BYTES;

// ---- the slow stream's field list (the Telemetry layout, marv/types/telemetry.hpp) ----------------------------------------
[[nodiscard]] constexpr std::uint8_t bytes_of(FieldType t) noexcept {
  switch (t) {
    case FieldType::U8:
      return sizeof(std::uint8_t);
    case FieldType::U16:
    case FieldType::I16:
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

// At most one field per byte of Telemetry, filled in member order; `count` are used.
struct TelemetryFieldList {
  std::array<FieldDesc, sizeof(Telemetry)> fields{};
  std::size_t count = 0;

  constexpr void add(const char* name, const char* unit, std::size_t offset, FieldType type) noexcept {
    fields[count++] = FieldDesc{name, unit, static_cast<std::uint16_t>(offset), bytes_of(type), type};
  }
  template <std::size_t N>
  constexpr void add(const std::array<const char*, N>& names, const char* unit, std::size_t offset, FieldType type) noexcept {
    for (std::size_t i = 0; i < N; ++i) {
      add(names[i], unit, offset + i * bytes_of(type), type);
    }
  }
};

using AxisNames = std::array<const char*, prim::kSpatialDim>;
using QuatNames = std::array<const char*, kQuatComponents>;
using MotorNames = std::array<const char*, kQuadXMotors>;
constexpr AxisNames kGyro{"gyro_rad_s_x", "gyro_rad_s_y", "gyro_rad_s_z"};
constexpr AxisNames kAccel{"accel_m_s2_x", "accel_m_s2_y", "accel_m_s2_z"};
constexpr AxisNames kRateGyro{"rate_gyro_rad_s_x", "rate_gyro_rad_s_y", "rate_gyro_rad_s_z"};
constexpr QuatNames kQMahony{"q_mahony_w", "q_mahony_x", "q_mahony_y", "q_mahony_z"};
constexpr QuatNames kQEskf{"q_eskf_w", "q_eskf_x", "q_eskf_y", "q_eskf_z"};
constexpr AxisNames kBiasCal{"gyro_bias_cal_x", "gyro_bias_cal_y", "gyro_bias_cal_z"};
constexpr AxisNames kBiasMahony{"gyro_bias_mahony_x", "gyro_bias_mahony_y", "gyro_bias_mahony_z"};
constexpr AxisNames kBiasEskf{"gyro_bias_eskf_x", "gyro_bias_eskf_y", "gyro_bias_eskf_z"};
constexpr AxisNames kRateSetpoint{"rate_setpoint_rad_s_x", "rate_setpoint_rad_s_y", "rate_setpoint_rad_s_z"};
constexpr QuatNames kQSetpoint{"q_setpoint_w", "q_setpoint_x", "q_setpoint_y", "q_setpoint_z"};
constexpr std::array<const char*, prim::kCrsfRcChannels> kChannels{
    "rc_channels_01", "rc_channels_02", "rc_channels_03", "rc_channels_04", "rc_channels_05", "rc_channels_06",
    "rc_channels_07", "rc_channels_08", "rc_channels_09", "rc_channels_10", "rc_channels_11", "rc_channels_12",
    "rc_channels_13", "rc_channels_14", "rc_channels_15", "rc_channels_16"};
constexpr MotorNames kRotorSpeed{"rotor_speed_rad_s_1", "rotor_speed_rad_s_2", "rotor_speed_rad_s_3", "rotor_speed_rad_s_4"};
constexpr MotorNames kMotorDshot{"motor_dshot_1", "motor_dshot_2", "motor_dshot_3", "motor_dshot_4"};

[[nodiscard]] constexpr TelemetryFieldList telemetry_field_list() noexcept {
  TelemetryFieldList f;
  f.add("sequence", "1", offsetof(Telemetry, sequence), FieldType::U64);
  f.add("time_us", "us", offsetof(Telemetry, time_us), FieldType::U64);
  f.add("sample_us", "us", offsetof(Telemetry, sample_us), FieldType::U64);
  f.add("rate_executions", "1", offsetof(Telemetry, rate_executions), FieldType::U64);
  f.add("rc_t_us", "us", offsetof(Telemetry, rc_t_us), FieldType::U64);
  f.add("rc_link_t_us", "us", offsetof(Telemetry, rc_link_t_us), FieldType::U64);
  f.add("rotor_t_us", "us", offsetof(Telemetry, rotor_t_us), FieldType::U64);
  f.add("version", "1", offsetof(Telemetry, version), FieldType::U32);
  f.add("imu_flags", "1", offsetof(Telemetry, imu_flags), FieldType::U32);
  f.add("flags", "1", offsetof(Telemetry, flags), FieldType::U32);
  f.add("rc_frames", "1", offsetof(Telemetry, rc_frames), FieldType::U32);
  f.add("rotor_flags", "1", offsetof(Telemetry, rotor_flags), FieldType::U32);
  f.add("rate_faults", "1", offsetof(Telemetry, rate_faults), FieldType::U32);
  f.add("rate_timing_faults", "1", offsetof(Telemetry, rate_timing_faults), FieldType::U32);
  f.add("attitude_faults", "1", offsetof(Telemetry, attitude_faults), FieldType::U32);
  f.add("attitude_timing_faults", "1", offsetof(Telemetry, attitude_timing_faults), FieldType::U32);
  f.add("imu_deadline_misses", "1", offsetof(Telemetry, imu_deadline_misses), FieldType::U32);
  f.add(kGyro, "rad/s", offsetof(Telemetry, gyro_rad_s), FieldType::F32);
  f.add(kAccel, "m/s2", offsetof(Telemetry, accel_m_s2), FieldType::F32);
  f.add("temp_k", "K", offsetof(Telemetry, temp_k), FieldType::F32);
  f.add(kRateGyro, "rad/s", offsetof(Telemetry, rate_gyro_rad_s), FieldType::F32);
  f.add(kQMahony, "1", offsetof(Telemetry, q_mahony), FieldType::F32);
  f.add(kQEskf, "1", offsetof(Telemetry, q_eskf), FieldType::F32);
  f.add(kBiasCal, "rad/s", offsetof(Telemetry, gyro_bias_cal), FieldType::F32);
  f.add(kBiasMahony, "rad/s", offsetof(Telemetry, gyro_bias_mahony), FieldType::F32);
  f.add(kBiasEskf, "rad/s", offsetof(Telemetry, gyro_bias_eskf), FieldType::F32);
  f.add(kRateSetpoint, "rad/s", offsetof(Telemetry, rate_setpoint_rad_s), FieldType::F32);
  f.add(kQSetpoint, "1", offsetof(Telemetry, q_setpoint), FieldType::F32);
  f.add("thrust_n", "N", offsetof(Telemetry, thrust_n), FieldType::F32);
  f.add("thrust_max_n", "N", offsetof(Telemetry, thrust_max_n), FieldType::F32);
  f.add("rc_roll", "1", offsetof(Telemetry, rc_roll), FieldType::F32);
  f.add("rc_pitch", "1", offsetof(Telemetry, rc_pitch), FieldType::F32);
  f.add("rc_yaw", "1", offsetof(Telemetry, rc_yaw), FieldType::F32);
  f.add("rc_throttle", "1", offsetof(Telemetry, rc_throttle), FieldType::F32);
  f.add(kChannels, "1", offsetof(Telemetry, rc_channels), FieldType::F32);
  f.add("rc_link_quality", "1", offsetof(Telemetry, rc_link_quality), FieldType::F32);
  f.add(kRotorSpeed, "rad/s", offsetof(Telemetry, rotor_speed_rad_s), FieldType::F32);
  f.add(kMotorDshot, "1", offsetof(Telemetry, motor_dshot), FieldType::U16);
  f.add("reserved", "1", offsetof(Telemetry, reserved), FieldType::U16);
  f.add("arm_state", "1", offsetof(Telemetry, arm_state), FieldType::U8);
  f.add("motor_gate", "1", offsetof(Telemetry, motor_gate), FieldType::U8);
  return f;
}

// Every byte of Telemetry is described exactly once, in order: a layout change without a matching list fails to compile.
[[nodiscard]] constexpr bool covers_telemetry(const TelemetryFieldList& f) noexcept {
  std::size_t end = 0;
  for (std::size_t i = 0; i < f.count; ++i) {
    if (f.fields[i].offset != end || f.fields[i].size == 0) {
      return false;
    }
    end += f.fields[i].size;
  }
  return end == sizeof(Telemetry);
}
static_assert(covers_telemetry(telemetry_field_list()), "the slow stream's field list does not match marv::Telemetry");

template <std::size_t N>
[[nodiscard]] constexpr std::array<FieldDesc, N> first_fields(const TelemetryFieldList& f) noexcept {
  std::array<FieldDesc, N> out{};
  for (std::size_t i = 0; i < N; ++i) {
    out[i] = f.fields[i];
  }
  return out;
}
constexpr std::size_t kTelemetryFieldCount = telemetry_field_list().count;
constexpr std::array<FieldDesc, kTelemetryFieldCount> kTelemetryFields =
    first_fields<kTelemetryFieldCount>(telemetry_field_list());

// ---- the logger and what the hooks read --------------------------------------------------------------------------------------
std::optional<log::ArenaLogger> g_logger;
// The arena logger as the hooks reach it: stored (release) once by flight_log_start after the logger and g_scales are set.
std::atomic<log::ArenaLogger*> g_hooks{nullptr};
log::ImuRawScales g_scales{};
log::TelemetryHook g_telemetry{};
bool (*g_telemetry_source)(Telemetry&) noexcept = nullptr;
std::atomic<ArmStateSource> g_arm_source{nullptr};
// Tick context only: the frame count of the last ManualControl pushed.
std::uint32_t g_frames_logged = 0;

// The slow stream's record body: the snapshot's bytes as they are (Telemetry is fixed, padding-free and little-endian).
std::size_t fill_telemetry(void*, std::span<std::uint8_t> out) noexcept {
  Telemetry t{};
  if (g_telemetry_source == nullptr || out.size() < sizeof(Telemetry) || !g_telemetry_source(t)) {
    return 0;
  }
  std::memcpy(out.data(), &t, sizeof(Telemetry));
  return sizeof(Telemetry);
}

#if defined(MARV_FC3V2_FLIGHT_LOG_EMB6_OPEN)
bool g_emb6_open_trigger = false;
std::array<char, kLineCapacity> g_emb6_open_line{};
std::size_t g_emb6_open_line_len = 0;
bool g_emb6_open_line_overflow = false;

void emb6_open_command(std::string_view verb, bool armed) noexcept {
  const log::ArenaLogger* a = flight_log();
  const log::Logger* l = a != nullptr ? a->logger() : nullptr;
  const auto t = static_cast<unsigned long long>(hal_time_us());
  if (verb == "start") {
    if (armed) {
      std::printf("log start: REFUSED: the vehicle is armed (arming starts the logger)\n");
    } else if (l == nullptr || l->state() != log::LoggerState::Ready) {
      std::printf("log start: REFUSED: the logger is not Ready (no lease, no card, or not probed yet)\n");
    } else {
      g_emb6_open_trigger = true;
      std::printf("log start: measurement trigger on (the logger is serviced as if armed; nothing is armed) t=%llu\n", t);
    }
  } else if (verb == "stop") {
    std::printf("log stop: measurement trigger %s t=%llu\n", g_emb6_open_trigger ? "off" : "was already off", t);
    g_emb6_open_trigger = false;
  } else {
    std::printf("log: unknown command (log start | stop)\n");
  }
}
#endif

}  // namespace

void flight_log_start(const FlightLogStart& start) noexcept {
  if (g_logger.has_value() || start.storage == nullptr) {
    return;
  }
  g_scales = start.imu_raw;
  g_telemetry_source = start.telemetry;
  g_telemetry = log::TelemetryHook{kTelemetryVersion, kTelemetryFields, &fill_telemetry, nullptr};
  log::HeaderInfo h = start.header;
  h.telemetry = start.telemetry != nullptr ? &g_telemetry : nullptr;
  h.imu_raw = &g_scales;
#if defined(MARV_FC3V2_FLIGHT_LOG_EMB6_OPEN)
  h.build_flags = static_cast<std::uint8_t>(h.build_flags | log::kBuildFlagEmb6Open);
#endif
  g_logger.emplace(*start.storage, kStagingBytes, start.config, start.config_ctx, h);
  g_hooks.store(&*g_logger, std::memory_order_release);
}

void flight_log_service(TimeUs now, bool armed) noexcept {
  if (!g_logger.has_value()) {
    return;
  }
#if defined(MARV_FC3V2_FLIGHT_LOG_EMB6_OPEN)
  armed = armed || g_emb6_open_trigger;
#endif
  g_logger->service(now, armed);
}

const log::ArenaLogger* flight_log() noexcept { return g_logger.has_value() ? &*g_logger : nullptr; }

std::span<const log::FieldDesc> flight_log_telemetry_fields() noexcept { return kTelemetryFields; }

// ---- the tick hooks (log_tick.hpp) ---------------------------------------------------------------------------------------------
void log_tick_set_arm_source(ArmStateSource source) noexcept { g_arm_source.store(source, std::memory_order_release); }

void log_tick_sample(const ImuSample& s) noexcept {
  log::ArenaLogger* l = g_hooks.load(std::memory_order_acquire);
  if (l != nullptr) {
    static_cast<void>(l->push_imu_raw(s, g_scales));
  }
}

void log_tick_rate(std::span<const DshotValue> motor) noexcept {
  log::ArenaLogger* l = g_hooks.load(std::memory_order_acquire);
  if (l == nullptr) {
    return;
  }
  log::RateTickRecord r;
  r.now_us = hal_time_us();
  const ArmStateSource source = g_arm_source.load(std::memory_order_acquire);
  r.arm_state = source != nullptr ? source() : 0;
  r.gate_state = kGateUnknown;
  for (std::size_t i = 0; i < r.dshot.size() && i < motor.size(); ++i) {
    r.dshot[i] = motor[i];
  }
  static_cast<void>(l->push_rate_tick(r));
}

void log_tick_manual(const ManualControl& m) noexcept {
  if (m.frames == g_frames_logged) {
    return;
  }
  g_frames_logged = m.frames;
  log::ArenaLogger* l = g_hooks.load(std::memory_order_acquire);
  if (l != nullptr) {
    static_cast<void>(l->push_manual(m));
  }
}

#if defined(MARV_FC3V2_FLIGHT_LOG_EMB6_OPEN)
const char flight_log_emb6_open_banner[] =
    "MEASUREMENT BUILD (MARV_FC3V2_FLIGHT_LOG_EMB6_OPEN): not a flight image. Core EMB-6 is open: the log ring may hold less "
    "than the fast stream for the worst SD stall (the post-link check L-5 only warns); the log header carries the flag; the "
    "console trigger `log start` / `log stop` logs without arming.";

bool flight_log_emb6_open_console_byte(char c, bool armed) noexcept {
  if (c != '\n') {
    if (g_emb6_open_line_len < g_emb6_open_line.size()) {
      g_emb6_open_line[g_emb6_open_line_len++] = c;
    } else {
      g_emb6_open_line_overflow = true;
    }
    return false;
  }
  std::string_view line{g_emb6_open_line.data(), g_emb6_open_line_len};
  const bool overflow = g_emb6_open_line_overflow;
  g_emb6_open_line_len = 0;
  g_emb6_open_line_overflow = false;
  if (!line.empty() && line.back() == '\r') {
    line.remove_suffix(1);
  }
  constexpr std::string_view kCmd = "log";
  if (overflow || line.substr(0, kCmd.size()) != kCmd || (line.size() > kCmd.size() && line[kCmd.size()] != ' ')) {
    return false;
  }
  std::string_view verb = line.substr(kCmd.size());
  while (!verb.empty() && verb.front() == ' ') {
    verb.remove_prefix(1);
  }
  while (!verb.empty() && verb.back() == ' ') {
    verb.remove_suffix(1);
  }
  emb6_open_command(verb, armed);
  return true;
}
#endif

}  // namespace marv::hal_target
