// The ground link service (marv/link/service.hpp) on the MAVLink 2 codec (marv/link/codec_mavlink.hpp). It includes no MAVLink
// header: the messages are the plain structs of marv/link/messages.hpp, and the standard enum values are its namespace
// `standard`, generated from the pinned common.xml and minimal.xml.
#include "marv/link/service.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <limits>
#include <span>
#include <tuple>
#include <variant>

#include <marv/arming/arming.hpp>
#include <marv/hal/hal.hpp>
#include <marv/params/param.hpp>
#include <marv/params/param_ids.hpp>
#include <marv/prim/constants.hpp>
#include <marv/store/mcfg.hpp>
#include <marv/store/store.hpp>
#include <marv/types/imu_sample.hpp>
#include <marv/types/telemetry.hpp>
#include <marv/types/time.hpp>

#include "marv/link/codec_mavlink.hpp"

namespace marv::link {
namespace {

// The vehicle's address: the protocol's convention for a vehicle with one autopilot, system 1 and component 1
// (MAV_COMP_ID_AUTOPILOT1 of minimal.xml's MAV_COMPONENT).
constexpr std::uint8_t kSystemId = 1;
constexpr std::uint8_t kComponentId = 1;
// One heartbeat per second, the protocol's convention (https://mavlink.io/en/services/heartbeat.html: "at 1 Hz").
constexpr TimeUs kHeartbeatPeriodUs = static_cast<TimeUs>(prim::kMicrosecondsPerSecond);
// MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN's param1 values the service takes (prim constants): reboot and reboot to the bootloader.
// Our reading of the bootloader one: the vehicle gives the maintenance grant and stays running; the host then resets it into
// the bootloader (picotool, through the USB reset interface the grant opens).
constexpr std::uint32_t kRebootActionReboot = prim::kRebootShutdownActionReboot;
constexpr std::uint32_t kRebootActionBootloader = prim::kRebootShutdownActionBootloader;

using Id = decltype(ParamValue::param_id);
constexpr std::size_t kSourceChunkBytes = std::tuple_size_v<decltype(MarvParamSource::chunk)>;
constexpr std::size_t kRecordChunkBytes = std::tuple_size_v<decltype(MarvParamRecord::source_chunk)>;
constexpr std::size_t kStatusTextBytes = std::tuple_size_v<decltype(Statustext::text)>;
// The longest source text the link can state or report: the largest length its fields carry.
constexpr std::size_t kSourceBytesMax = std::numeric_limits<decltype(MarvParamSource::total_length)>::max();
static_assert(std::numeric_limits<decltype(MarvParamRecord::source_length)>::max() == kSourceBytesMax);
static_assert(kParamCount <= std::numeric_limits<decltype(ParamValue::param_count)>::max());
static_assert(kParamCount <= static_cast<std::size_t>(std::numeric_limits<decltype(ParamRequestRead::param_index)>::max()));

// The record format's enumerations and the dialect's are the same lists (tools/config/mcfg.py, link/marv.xml).
static_assert(static_cast<int>(MarvParamOrigin::Identified) == static_cast<int>(ParamOrigin::Identified));
static_assert(static_cast<int>(MarvParamOrigin::DefaultFromRegister) == static_cast<int>(ParamOrigin::DefaultFromRegister));
static_assert(static_cast<int>(MarvParamMethod::Scenario) == static_cast<int>(ParamMethod::Scenario));
static_assert(static_cast<int>(MarvParamMethod::DesignBudget) == static_cast<int>(ParamMethod::DesignBudget));
static_assert(static_cast<int>(MarvParamMethod::Derived) == static_cast<int>(ParamMethod::Derived));
static_assert(static_cast<int>(MarvParamMethod::Datasheet) == static_cast<int>(ParamMethod::Datasheet));
static_assert(static_cast<int>(MarvParamMethod::Identified) == static_cast<int>(ParamMethod::Identified));
static_assert(static_cast<int>(MarvParamMethod::Measured) == static_cast<int>(ParamMethod::Measured));
static_assert(static_cast<int>(MarvParamSigmaKind::Choice) == static_cast<int>(SigmaKind::Choice));
static_assert(static_cast<int>(MarvParamSigmaKind::Unknown) == static_cast<int>(SigmaKind::Unknown));
static_assert(static_cast<int>(MarvParamSigmaKind::Exact) == static_cast<int>(SigmaKind::Exact));

constexpr const char* kSchemaRefused = "refused: the message's parameter schema hash is not this image's";

// ---- the telemetry streams (service.hpp) ----
enum class Stream : std::uint8_t {
  SysStatus,
  Attitude,
  AttitudeQuaternion,
  RcChannels,
  AttitudeTarget,
  HighresImu,
  ActuatorOutputStatus,
  MarvFcStatus,
  MarvEstimator,
  MarvIdentity,
};
constexpr std::size_t kStreams = static_cast<std::size_t>(Stream::MarvIdentity) + 1;

struct StreamInfo {
  std::uint32_t id;
  std::size_t frame_bytes;  // the longest frame: what the stream is counted at against the budget
  ParamId cap;              // its cap, Hz
  bool snapshot;            // built from the composition's telemetry snapshot
};

constexpr std::array<StreamInfo, kStreams> kStreamInfo{{
    {kSysStatusId, kSysStatusFrameBytes, ParamId::tlm_hz_status, true},
    {kAttitudeId, kAttitudeFrameBytes, ParamId::tlm_hz_attitude, true},
    {kAttitudeQuaternionId, kAttitudeQuaternionFrameBytes, ParamId::tlm_hz_att_quat, true},
    {kRcChannelsId, kRcChannelsFrameBytes, ParamId::tlm_hz_rc, true},
    {kAttitudeTargetId, kAttitudeTargetFrameBytes, ParamId::tlm_hz_att_tgt, true},
    {kHighresImuId, kHighresImuFrameBytes, ParamId::tlm_hz_imu, true},
    {kActuatorOutputStatusId, kActuatorOutputStatusFrameBytes, ParamId::tlm_hz_actuators, true},
    {kMarvFcStatusId, kMarvFcStatusFrameBytes, ParamId::tlm_hz_fc_status, true},
    {kMarvEstimatorId, kMarvEstimatorFrameBytes, ParamId::tlm_hz_estimator, true},
    {kMarvIdentityId, kMarvIdentityFrameBytes, ParamId::tlm_hz_identity, false},
}};

// The arming machine's states and the dialect's are the same list (arming.hpp ArmState, link/marv.xml MARV_ARM_STATE).
static_assert(static_cast<int>(MarvArmState::Boot) == static_cast<int>(arming::ArmState::Boot));
static_assert(static_cast<int>(MarvArmState::NoLink) == static_cast<int>(arming::ArmState::NoLink));
static_assert(static_cast<int>(MarvArmState::Failsafe) == static_cast<int>(arming::ArmState::Failsafe));
static_assert(static_cast<int>(MarvArmState::Armed) == static_cast<int>(arming::ArmState::Armed));
static_assert(static_cast<int>(MarvArmState::ArmedFailsafeCentred) == static_cast<int>(arming::ArmState::ArmedFailsafeCentred));
static_assert(static_cast<int>(MarvArmState::NotUpright) == static_cast<int>(arming::ArmState::NotUpright));
static_assert(static_cast<int>(MarvArmState::ConfigRefused) == static_cast<int>(arming::ArmState::ConfigRefused));
static_assert(static_cast<int>(MarvArmState::Maintenance) == static_cast<int>(arming::ArmState::Maintenance));

struct State {
  bool started = false;
  ServiceConfig config{};
  MavlinkParser parser{};
  std::uint8_t sequence = 0;
  bool heartbeat_sent = false;
  TimeUs last_heartbeat_us = 0;
  std::size_t listing_next = kParamCount;  // the next parameter of a listing; kParamCount: no listing
  store::Digest running_hash{};            // the config hash of the table this boot runs
  // The source text being set: its parameter (kParamCount: none), its length and the bytes received so far.
  std::size_t source_index = kParamCount;
  std::size_t source_total = 0;
  std::size_t source_have = 0;
  std::array<char, kSourceBytesMax> source{};
  bool reboot_pending = false;    // a command was accepted; the reboot waits for the maintenance grant
  // A reboot command waiting for the maintenance grant: whether it reboots once granted, and who asked.
  bool grant_pending = false;
  bool grant_reboots = false;
  Envelope grant_from{};
  bool reboot_requested = false;  // hal_reboot returned: the host stops this run
  std::uint16_t text_id = 0;      // of the latest status text sent in chunks
  // The telemetry streams: each one's cap (Hz) and granted interval (us, 0: stopped) and when it is next due; the budget
  // (bytes per second); the frames the parser accepted and refused (SYS_STATUS); the RC link-loss time (us).
  std::array<float, kStreams> cap_hz{};
  std::array<std::uint32_t, kStreams> interval_us{};
  std::array<TimeUs, kStreams> next_us{};
  float budget_bytes_s = 0.0F;
  std::uint64_t rx_frames = 0;
  std::uint64_t rx_errors = 0;
  TimeUs rc_loss_us = 0;
};

State g;

template <class T>
[[nodiscard]] T narrow(std::size_t v) noexcept {
  return static_cast<T>(v);
}

[[nodiscard]] bool armed() noexcept { return g.config.armed == nullptr || g.config.armed(); }

[[nodiscard]] bool for_us(std::uint8_t system, std::uint8_t component) noexcept {
  return (system == 0 || system == kSystemId) && (component == 0 || component == kComponentId);
}

[[nodiscard]] bool all_zero(const store::Digest& d) noexcept {
  return std::all_of(d.begin(), d.end(), [](std::uint8_t b) { return b == 0; });
}

void send(const Message& message) noexcept {
  std::array<std::uint8_t, kFrameBytesMax> frame{};
  const std::size_t n = encode_mavlink(Envelope{kSystemId, kComponentId, g.sequence}, message, frame);
  if (n == 0) {
    return;
  }
  g.sequence = static_cast<std::uint8_t>(g.sequence + 1);
  g.config.transport.send(std::span<const std::uint8_t>{frame.data(), n});
}

// The staged table (the store's staged copy; the active table if the store has not booted).
[[nodiscard]] ParamTable staged() noexcept {
  const ParamTable t = store::staged_table();
  return t.size() == kParamCount ? t : param_active_table();
}

[[nodiscard]] std::uint32_t bits_of(const marv::ParamValue& v) noexcept {
  return v.type == ParamType::F32 ? std::bit_cast<std::uint32_t>(v.f32) : std::bit_cast<std::uint32_t>(v.i32);
}

[[nodiscard]] std::uint8_t wire_type(ParamType t) noexcept {
  return t == ParamType::F32 ? standard::kMavParamTypeReal32 : standard::kMavParamTypeInt32;
}

[[nodiscard]] bool type_of(std::uint8_t wire, ParamType& out) noexcept {
  if (wire == standard::kMavParamTypeReal32) {
    out = ParamType::F32;
    return true;
  }
  if (wire == standard::kMavParamTypeInt32) {
    out = ParamType::I32;
    return true;
  }
  return false;
}

[[nodiscard]] const char* link_id_text(std::size_t i) noexcept { return marv::generated::kParamLinkIds[i]; }

[[nodiscard]] Id link_id(std::size_t i) noexcept {
  Id id{};
  const char* s = link_id_text(i);
  for (std::size_t k = 0; k < id.size() && s[k] != '\0'; ++k) {
    id[k] = s[k];
  }
  return id;
}

// The parameter whose link id `id` is (zero-padded, or all its bytes used); kParamCount if none.
[[nodiscard]] std::size_t index_of(const Id& id) noexcept {
  for (std::size_t i = 0; i < kParamCount; ++i) {
    if (std::strncmp(id.data(), link_id_text(i), id.size()) == 0) {
      return i;
    }
  }
  return kParamCount;
}

[[nodiscard]] std::size_t source_length(const char* s) noexcept {
  std::size_t n = 0;
  while (s != nullptr && n < kSourceBytesMax && s[n] != '\0') {
    ++n;
  }
  return n;
}

// A status text of the given pieces, in one message if it fits, else in chunks under one nonzero id (the last chunk shorter
// than the field, an empty one after a text that fills its last chunk exactly).
void send_text(std::initializer_list<const char*> pieces) noexcept {
  std::size_t total = 0;
  for (const char* p : pieces) {
    total += std::strlen(p);
  }
  Statustext m{};
  m.severity = standard::kMavSeverityWarning;
  if (total > kStatusTextBytes) {
    g.text_id = static_cast<std::uint16_t>(g.text_id + 1);
    if (g.text_id == 0) {
      g.text_id = 1;
    }
    m.id = g.text_id;
  }
  std::size_t fill = 0;
  for (const char* p : pieces) {
    for (const char* c = p; *c != '\0'; ++c) {
      if (fill == kStatusTextBytes) {
        send(m);
        m.text = {};
        fill = 0;
        m.chunk_seq = static_cast<std::uint8_t>(m.chunk_seq + 1);
      }
      m.text[fill] = *c;
      ++fill;
    }
  }
  send(m);
  if (m.id != 0 && fill == kStatusTextBytes) {
    m.text = {};
    m.chunk_seq = static_cast<std::uint8_t>(m.chunk_seq + 1);
    send(m);
  }
}

void send_value(std::size_t i) noexcept {
  const ParamRecord& r = staged()[i];
  send(ParamValue{link_id(i), bits_of(r.value), wire_type(r.value.type), narrow<std::uint16_t>(kParamCount),
                  narrow<std::uint16_t>(i)});
}

void send_record(std::size_t i) noexcept {
  const ParamRecord& s = staged()[i];
  const ParamRecord& a = param_active_table()[i];
  const std::size_t length = source_length(s.source);
  MarvParamRecord m{};
  m.active_bits = bits_of(a.value);
  m.staged_bits = bits_of(s.value);
  m.sigma = s.sigma;
  m.param_index = narrow<std::uint16_t>(i);
  m.param_count = narrow<std::uint16_t>(kParamCount);
  m.source_length = narrow<std::uint16_t>(length);
  m.origin = static_cast<MarvParamOrigin>(s.origin);
  m.method = static_cast<MarvParamMethod>(s.method);
  m.sigma_kind = static_cast<MarvParamSigmaKind>(s.sigma_kind);
  m.lock = a.locked ? 1 : 0;
  m.source_chunk_length = narrow<std::uint8_t>(std::min(length, kRecordChunkBytes));
  std::copy_n(s.source, m.source_chunk_length, m.source_chunk.begin());
  send(m);
}

void send_parameter(std::size_t i) noexcept {
  send_value(i);
  send_record(i);
}

// A parameter is two frames (send_parameter), each at most kFrameBytesMax.
[[nodiscard]] bool room_for_parameter() noexcept {
  return g.config.transport.writable == nullptr || g.config.transport.writable() >= 2 * kFrameBytesMax;
}

[[nodiscard]] MarvConfigRefusal refusal_of(const store::BootReport& b) noexcept {
  switch (b.status) {
    case arming::ConfigStatus::Ok:
      return MarvConfigRefusal::None;
    case arming::ConfigStatus::Corrupt:
      return MarvConfigRefusal::Format;
    case arming::ConfigStatus::SchemaMismatch:
      return MarvConfigRefusal::Schema;
    case arming::ConfigStatus::Invalid:
      return MarvConfigRefusal::Validation;
    case arming::ConfigStatus::NotValidated:
      return !all_zero(g.config.image_sha256) && b.binding.image_sha256 == g.config.image_sha256
                 ? MarvConfigRefusal::ReportBinding
                 : MarvConfigRefusal::ImageBinding;
  }
  return MarvConfigRefusal::Format;
}

[[nodiscard]] MarvActivationResult activation_of(const store::BootReport& b) noexcept {
  const bool write_failed = b.request_fault == store::Fault::WriteFailed || b.request_fault == store::Fault::ReadBack;
  switch (b.request) {
    case store::BootRequest::None:
      return MarvActivationResult::None;
    case store::BootRequest::Activate:
    case store::BootRequest::Revert:
      if (b.request_fault == store::Fault::None) {
        return MarvActivationResult::Activated;
      }
      return write_failed ? MarvActivationResult::WriteFailed : MarvActivationResult::RefusedAtBoot;
    case store::BootRequest::FactoryReset:
      return b.request_fault == store::Fault::None ? MarvActivationResult::None : MarvActivationResult::WriteFailed;
    case store::BootRequest::Unmatched:
      return MarvActivationResult::RefusedAtBoot;
  }
  return MarvActivationResult::RefusedAtBoot;
}

void send_identity() noexcept {
  const store::BootReport& b = store::boot_report();
  const bool slot = b.source == store::ConfigSource::Slot;
  MarvIdentity m{};
  m.param_schema_hash = kParamSchemaHash;
  m.image_sha256 = g.config.image_sha256;
  m.config_sha256 = g.running_hash;
  if (slot) {
    m.validation_report_id = b.binding.report_id;
  }
  m.config_source = slot ? MarvConfigSource::Active : MarvConfigSource::Factory;
  m.config_state = b.status == arming::ConfigStatus::Ok ? MarvConfigState::Ok : MarvConfigState::Refused;
  m.config_refusal = refusal_of(b);
  m.last_activation_result = activation_of(b);
  m.staged_differs = store::staged_differs() ? 1 : 0;
  send(m);
}

void send_version() noexcept {
  AutopilotVersion m{};
  m.capabilities = standard::kMavProtocolCapabilityParamEncodeBytewise | standard::kMavProtocolCapabilityMavlink2;
  send(m);
}

void send_heartbeat() noexcept {
  const bool is_armed = armed();
  Heartbeat m{};
  m.type = standard::kMavTypeQuadrotor;
  m.autopilot = standard::kMavAutopilotGeneric;
  m.base_mode = is_armed ? standard::kMavModeFlagSafetyArmed : 0;
  m.system_status = is_armed ? standard::kMavStateActive : standard::kMavStateStandby;
  send(m);
}

// ---- telemetry ----------------------------------------------------------------------------------------------------------

[[nodiscard]] std::uint32_t boot_ms(TimeUs t) noexcept {
  return static_cast<std::uint32_t>(t / prim::kMicrosecondsPerMillisecond);
}

template <class Flag>
[[nodiscard]] std::uint8_t bit_if(bool on, Flag flag) noexcept {
  return on ? static_cast<std::uint8_t>(flag) : 0;
}

[[nodiscard]] bool has(std::uint32_t flags, ImuFlag f) noexcept { return (flags & imu_flag(f)) != 0; }

[[nodiscard]] bool has(const Telemetry& t, TelemetryFlag f) noexcept { return (t.flags & telemetry_flag(f)) != 0; }

// The estimate in use (w x y z): the ESKF in the image that flies it, else Mahony.
[[nodiscard]] const std::array<float, kQuatComponents>& attitude_of(const Telemetry& t) noexcept {
  return has(t, TelemetryFlag::EskfInUse) ? t.q_eskf : t.q_mahony;
}

// The RC link is up: a valid frame no older than the link-loss time the arming machine uses (failsafe_loss_us).
[[nodiscard]] bool rc_up(const Telemetry& t) noexcept {
  return t.rc_frames != 0 && t.time_us >= t.rc_t_us && t.time_us - t.rc_t_us <= g.rc_loss_us;
}

[[nodiscard]] std::uint16_t saturated_u16(std::uint64_t v) noexcept {
  return static_cast<std::uint16_t>(std::min<std::uint64_t>(v, std::numeric_limits<std::uint16_t>::max()));
}

// Sensor health (common.xml MAV_SYS_STATUS_SENSOR): the gyro and the accelerometer of the tick's sample valid and not
// saturated, the RC link up (rc_up), the attitude estimate in use valid. load is not measured by these images and is 0; no
// battery is measured. The link's own reception: the frames the parser refused (bad checksum or length) and their share of
// all it decoded.
[[nodiscard]] SysStatus sys_status(const Telemetry& t) noexcept {
  using namespace standard;
  constexpr std::uint32_t kGyroSat = imu_flag(ImuFlag::GyroSatX) | imu_flag(ImuFlag::GyroSatY) | imu_flag(ImuFlag::GyroSatZ);
  constexpr std::uint32_t kAccelSat =
      imu_flag(ImuFlag::AccelSatX) | imu_flag(ImuFlag::AccelSatY) | imu_flag(ImuFlag::AccelSatZ);
  const std::uint32_t present = kMavSysStatusSensor3dGyro | kMavSysStatusSensor3dAccel | kMavSysStatusSensorRcReceiver |
                                kMavSysStatusAhrs;
  std::uint32_t health = 0;
  if (has(t.imu_flags, ImuFlag::GyroValid) && (t.imu_flags & kGyroSat) == 0) {
    health |= kMavSysStatusSensor3dGyro;
  }
  if (has(t.imu_flags, ImuFlag::AccelValid) && (t.imu_flags & kAccelSat) == 0) {
    health |= kMavSysStatusSensor3dAccel;
  }
  if (rc_up(t)) {
    health |= kMavSysStatusSensorRcReceiver;
  }
  if (has(t, TelemetryFlag::EstimatorValid)) {
    health |= kMavSysStatusAhrs;
  }
  SysStatus m{};
  m.onboard_control_sensors_present = present;
  m.onboard_control_sensors_enabled = present;
  m.onboard_control_sensors_health = health;
  m.voltage_battery = std::numeric_limits<std::uint16_t>::max();
  m.current_battery = -1;
  m.battery_remaining = -1;
  const std::uint64_t decoded = g.rx_frames + g.rx_errors;
  m.drop_rate_comm = decoded == 0 ? 0 : saturated_u16(g.rx_errors * prim::kCentiPercentPerUnit / decoded);
  m.errors_comm = saturated_u16(g.rx_errors);
  return m;
}

// The ZYX (yaw, pitch, roll) angles of the body-to-NED attitude in use, and the rate loop's input as the angular rates.
[[nodiscard]] Attitude attitude_message(const Telemetry& t) noexcept {
  const auto& [w, x, y, z] = attitude_of(t);
  Attitude m{};
  m.time_boot_ms = boot_ms(t.time_us);
  m.roll = std::atan2(2.0F * (w * x + y * z), 1.0F - 2.0F * (x * x + y * y));
  m.pitch = std::asin(std::clamp(2.0F * (w * y - z * x), -1.0F, 1.0F));
  m.yaw = std::atan2(2.0F * (w * z + x * y), 1.0F - 2.0F * (y * y + z * z));
  m.rollspeed = t.rate_gyro_rad_s[0];
  m.pitchspeed = t.rate_gyro_rad_s[1];
  m.yawspeed = t.rate_gyro_rad_s[2];
  return m;
}

[[nodiscard]] AttitudeQuaternion attitude_quaternion(const Telemetry& t) noexcept {
  const auto& [w, x, y, z] = attitude_of(t);
  AttitudeQuaternion m{};
  m.time_boot_ms = boot_ms(t.time_us);
  m.q1 = w;
  m.q2 = x;
  m.q3 = y;
  m.q4 = z;
  m.rollspeed = t.rate_gyro_rad_s[0];
  m.pitchspeed = t.rate_gyro_rad_s[1];
  m.yawspeed = t.rate_gyro_rad_s[2];
  return m;  // repr_offset_q all zero: not supported (common.xml)
}

// The channels in microseconds, the inverse of the pilot-input contract's x = (us - 1500) / 512 (manual_control.hpp), and the
// receiver's link quality on rssi's 0 .. 254 scale (UINT8_MAX before any link-statistics frame). Before any RC frame no
// channel is received.
[[nodiscard]] RcChannels rc_channels(const Telemetry& t) noexcept {
  constexpr std::array<std::uint16_t RcChannels::*, prim::kCrsfRcChannels> kChannels{
      &RcChannels::chan1_raw,  &RcChannels::chan2_raw,  &RcChannels::chan3_raw,  &RcChannels::chan4_raw,
      &RcChannels::chan5_raw,  &RcChannels::chan6_raw,  &RcChannels::chan7_raw,  &RcChannels::chan8_raw,
      &RcChannels::chan9_raw,  &RcChannels::chan10_raw, &RcChannels::chan11_raw, &RcChannels::chan12_raw,
      &RcChannels::chan13_raw, &RcChannels::chan14_raw, &RcChannels::chan15_raw, &RcChannels::chan16_raw};
  constexpr std::uint16_t kUnused = std::numeric_limits<std::uint16_t>::max();
  RcChannels m{};
  m.time_boot_ms = boot_ms(t.time_us);
  const bool received = t.rc_frames != 0;
  m.chancount = received ? narrow<std::uint8_t>(prim::kCrsfRcChannels) : std::uint8_t{0};
  for (std::size_t i = 0; i < kChannels.size(); ++i) {
    const float x = std::clamp(t.rc_channels[i], -1.0F, 1.0F);
    m.*kChannels[i] = received ? static_cast<std::uint16_t>(std::lround(prim::kCrsfUsCentre + prim::kRcUsHalfSpan * x)) : kUnused;
  }
  m.chan17_raw = kUnused;
  m.chan18_raw = kUnused;
  m.rssi = t.rc_link_t_us == 0
               ? std::numeric_limits<std::uint8_t>::max()
               : static_cast<std::uint8_t>(std::lround(std::clamp(t.rc_link_quality, 0.0F, 1.0F) * prim::kRcChannelsRssiMax));
  return m;
}

// The rate setpoint, the attitude setpoint in angle mode only (else ATTITUDE_IGNORE) and the collective thrust over the
// largest the throttle commands.
[[nodiscard]] AttitudeTarget attitude_target(const Telemetry& t) noexcept {
  AttitudeTarget m{};
  m.time_boot_ms = boot_ms(t.time_us);
  if (has(t, TelemetryFlag::AngleMode)) {
    m.q = t.q_setpoint;
  } else {
    m.type_mask = standard::kAttitudeTargetTypemaskAttitudeIgnore;
  }
  m.body_roll_rate = t.rate_setpoint_rad_s[0];
  m.body_pitch_rate = t.rate_setpoint_rad_s[1];
  m.body_yaw_rate = t.rate_setpoint_rad_s[2];
  m.thrust = t.thrust_max_n > 0.0F ? t.thrust_n / t.thrust_max_n : 0.0F;
  return m;
}

// The tick's sample as the estimator received it (Telemetry), stamped with the sample's stamp.
[[nodiscard]] HighresImu highres_imu(const Telemetry& t) noexcept {
  using namespace standard;
  HighresImu m{};
  m.time_usec = t.sample_us;
  m.xacc = t.accel_m_s2[0];
  m.yacc = t.accel_m_s2[1];
  m.zacc = t.accel_m_s2[2];
  m.xgyro = t.gyro_rad_s[0];
  m.ygyro = t.gyro_rad_s[1];
  m.zgyro = t.gyro_rad_s[2];
  if (has(t.imu_flags, ImuFlag::AccelValid)) {
    m.fields_updated |= kHighresImuUpdatedXacc | kHighresImuUpdatedYacc | kHighresImuUpdatedZacc;
  }
  if (has(t.imu_flags, ImuFlag::GyroValid)) {
    m.fields_updated |= kHighresImuUpdatedXgyro | kHighresImuUpdatedYgyro | kHighresImuUpdatedZgyro;
  }
  if (has(t.imu_flags, ImuFlag::TempValid)) {
    m.temperature = static_cast<float>(static_cast<double>(t.temp_k) - prim::kKelvinAtZeroCelsius);
    m.fields_updated |= kHighresImuUpdatedTemperature;
  }
  return m;
}

// The DShot values written, motor i at index i - 1 (0 is stop; every motor output is active).
[[nodiscard]] ActuatorOutputStatus actuator_output_status(const Telemetry& t) noexcept {
  ActuatorOutputStatus m{};
  static_assert(std::tuple_size_v<decltype(m.actuator)> >= kQuadXMotors);
  m.time_usec = t.time_us;
  for (std::size_t i = 0; i < kQuadXMotors; ++i) {
    m.active |= std::uint32_t{1} << i;
    m.actuator[i] = static_cast<float>(t.motor_dshot[i]);
  }
  return m;
}

[[nodiscard]] MarvFcStatus fc_status(const Telemetry& t) noexcept {
  MarvFcStatus m{};
  m.time_usec = t.time_us;
  m.rate_executions = t.rate_executions;
  m.rate_faults = t.rate_faults;
  m.rate_timing_faults = t.rate_timing_faults;
  m.attitude_faults = t.attitude_faults;
  m.attitude_timing_faults = t.attitude_timing_faults;
  m.imu_deadline_misses = t.imu_deadline_misses;
  m.rc_frames = t.rc_frames;
  constexpr std::uint32_t kNoFrame = std::numeric_limits<std::uint32_t>::max();
  const TimeUs age = t.time_us > t.rc_t_us ? t.time_us - t.rc_t_us : 0;
  m.rc_age_us = t.rc_frames == 0 ? kNoFrame : static_cast<std::uint32_t>(std::min<TimeUs>(age, kNoFrame - 1));
  m.arm_state = static_cast<MarvArmState>(t.arm_state);
  m.motor_gate = static_cast<MarvMotorGate>(t.motor_gate);
  m.flags = static_cast<std::uint8_t>(
      bit_if(has(t, TelemetryFlag::AngleMode), MarvFcStatusFlag::AngleMode) |
      bit_if(has(t, TelemetryFlag::Airmode), MarvFcStatusFlag::Airmode) |
      bit_if(has(t, TelemetryFlag::ItermHeld), MarvFcStatusFlag::ItermHeld) |
      bit_if(has(t, TelemetryFlag::GyroCalibrated), MarvFcStatusFlag::GyroCalibrated) |
      bit_if(has(t, TelemetryFlag::EstimatorValid), MarvFcStatusFlag::EstimatorValid) |
      bit_if(has(t, TelemetryFlag::Upright), MarvFcStatusFlag::Upright) |
      bit_if(has(t, TelemetryFlag::ImuInvalid), MarvFcStatusFlag::ImuInvalid) |
      bit_if(has(t, TelemetryFlag::ImuSaturated), MarvFcStatusFlag::ImuSaturated));
  m.rate_fault = static_cast<std::uint8_t>(bit_if(has(t, TelemetryFlag::RateFaultActive), MarvRateFaultFlag::Active) |
                                           bit_if(has(t, TelemetryFlag::RateFaultLatched), MarvRateFaultFlag::Latched));
  return m;
}

[[nodiscard]] MarvEstimator estimator_message(const Telemetry& t) noexcept {
  MarvEstimator m{};
  m.time_usec = t.time_us;
  m.q_mahony = t.q_mahony;
  m.q_eskf = t.q_eskf;
  m.gyro_bias_cal = t.gyro_bias_cal;
  m.gyro_bias_mahony = t.gyro_bias_mahony;
  m.gyro_bias_eskf = t.gyro_bias_eskf;
  m.flags = static_cast<std::uint8_t>(bit_if(has(t, TelemetryFlag::MahonyValid), MarvEstimatorFlag::MahonyValid) |
                                      bit_if(has(t, TelemetryFlag::EskfPresent), MarvEstimatorFlag::EskfPresent) |
                                      bit_if(has(t, TelemetryFlag::EskfValid), MarvEstimatorFlag::EskfValid) |
                                      bit_if(has(t, TelemetryFlag::EskfInUse), MarvEstimatorFlag::EskfInUse));
  return m;
}

void send_stream(Stream s, const Telemetry& t) noexcept {
  switch (s) {
    case Stream::SysStatus: return send(sys_status(t));
    case Stream::Attitude: return send(attitude_message(t));
    case Stream::AttitudeQuaternion: return send(attitude_quaternion(t));
    case Stream::RcChannels: return send(rc_channels(t));
    case Stream::AttitudeTarget: return send(attitude_target(t));
    case Stream::HighresImu: return send(highres_imu(t));
    case Stream::ActuatorOutputStatus: return send(actuator_output_status(t));
    case Stream::MarvFcStatus: return send(fc_status(t));
    case Stream::MarvEstimator: return send(estimator_message(t));
    case Stream::MarvIdentity: return send_identity();
  }
}

[[nodiscard]] float rate_or_zero(float hz) noexcept { return std::isfinite(hz) && hz >= 0.0F ? hz : 0.0F; }

// ---- incoming ----------------------------------------------------------------------------------------------------------

[[nodiscard]] bool same_bits(float a, std::uint32_t id) noexcept {
  return std::bit_cast<std::uint32_t>(a) == std::bit_cast<std::uint32_t>(static_cast<float>(id));
}

void send_ack(const Envelope& to, std::uint16_t command, std::uint8_t result) noexcept {
  CommandAck ack{};
  ack.command = command;
  ack.result = result;
  ack.target_system = to.system_id;
  ack.target_component = to.component_id;
  send(ack);
}

// MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN for this autopilot (param2 = 0): reboot (param1 1) or the bootloader (param1 3). Disarmed, it
// asks the tick for the maintenance grant and answers IN_PROGRESS; step_grant answers the rest. Armed: DENIED.
void on_reboot(const Envelope& from, const CommandLong& m) noexcept {
  const bool reboot = same_bits(m.param1, kRebootActionReboot);
  const bool bootloader = same_bits(m.param1, kRebootActionBootloader);
  if ((!reboot && !bootloader) || !same_bits(m.param2, 0) || g.config.request_maintenance == nullptr ||
      g.config.maintenance_granted == nullptr) {
    send_ack(from, m.command, standard::kMavResultUnsupported);
    return;
  }
  if (armed()) {
    send_text({"reboot refused while armed"});
    send_ack(from, m.command, standard::kMavResultDenied);
    return;
  }
  // One grant waits at a time: a newer command replaces a waiting one, which gets no further answer (service.hpp).
  g.grant_pending = true;
  g.grant_reboots = reboot;
  g.grant_from = from;
  g.config.request_maintenance();
  send_ack(from, m.command, standard::kMavResultInProgress);
}

// The stream whose message id param (a COMMAND_LONG float) names; kStreams if none.
[[nodiscard]] std::size_t stream_of(float param) noexcept {
  for (std::size_t k = 0; k < kStreams; ++k) {
    if (same_bits(param, kStreamInfo[k].id)) {
      return k;
    }
  }
  return kStreams;
}

// A stream the vehicle can send: one built from the snapshot needs the composition's telemetry.
[[nodiscard]] bool can_send(std::size_t k) noexcept {
  return k < kStreams && (!kStreamInfo[k].snapshot || g.config.telemetry != nullptr);
}

// The interval (us) granted to stream k asked at `asked_us` (> 0): the longest of the one asked, the one of its cap and the one
// at which it fits the budget beside the other running streams (each at its longest frame), rounded up to whole
// microseconds so the rate never exceeds either; 0 when the cap or the budget leaves it no room.
[[nodiscard]] std::uint32_t grant(std::size_t k, double asked_us) noexcept {
  double used = 0.0;
  for (std::size_t j = 0; j < kStreams; ++j) {
    if (j != k && g.interval_us[j] != 0) {
      used += prim::kMicrosecondsPerSecond / g.interval_us[j] * static_cast<double>(kStreamInfo[j].frame_bytes);
    }
  }
  const double room_hz = (static_cast<double>(g.budget_bytes_s) - used) / static_cast<double>(kStreamInfo[k].frame_bytes);
  const double limit_hz = std::min(static_cast<double>(g.cap_hz[k]), room_hz);
  if (!(limit_hz > 0.0)) {
    return 0;
  }
  const double interval = std::ceil(std::max(asked_us, prim::kMicrosecondsPerSecond / limit_hz));
  constexpr double kLongest = std::numeric_limits<std::int32_t>::max();  // MESSAGE_INTERVAL.interval_us is an int32
  return static_cast<std::uint32_t>(std::min(interval, kLongest));
}

// MAV_CMD_SET_MESSAGE_INTERVAL: param1 the message id, param2 the interval (us; -1 and 0, the default rate, stop the stream).
void on_set_interval(const Envelope& from, const CommandLong& m) noexcept {
  const std::size_t k = stream_of(m.param1);
  if (!can_send(k)) {
    send_ack(from, m.command, standard::kMavResultUnsupported);
    return;
  }
  if (m.param2 == 0.0F || m.param2 == -1.0F) {
    g.interval_us[k] = 0;
    send_ack(from, m.command, standard::kMavResultAccepted);
    return;
  }
  if (!(m.param2 > 0.0F) || !std::isfinite(m.param2)) {
    send_text({"telemetry: the interval is not a number of microseconds above 0, nor -1 or 0"});
    send_ack(from, m.command, standard::kMavResultDenied);
    return;
  }
  const std::uint32_t interval = grant(k, static_cast<double>(m.param2));
  if (interval == 0) {
    g.interval_us[k] = 0;
    send_text({"telemetry: no room for this stream (its tlm_hz_* cap is 0 or the telemetry budget is taken)"});
    send_ack(from, m.command, standard::kMavResultDenied);
    return;
  }
  g.interval_us[k] = interval;
  g.next_us[k] = hal_time_us();
  send_ack(from, m.command, standard::kMavResultAccepted);
}

// MESSAGE_INTERVAL for the message id param names: the interval in force, -1 for a stream that is stopped, 0 for a message the
// vehicle does not stream (common.xml).
void send_interval(float param) noexcept {
  const std::size_t k = stream_of(param);
  MessageInterval m{};
  if (k < kStreams) {
    m.message_id = narrow<std::uint16_t>(kStreamInfo[k].id);
    m.interval_us = !can_send(k) ? 0 : (g.interval_us[k] == 0 ? -1 : static_cast<std::int32_t>(g.interval_us[k]));
  } else if (param >= 0.0F && param <= static_cast<float>(std::numeric_limits<std::uint16_t>::max())) {
    m.message_id = static_cast<std::uint16_t>(param);
  }
  send(m);
}

// MAV_CMD_REQUEST_MESSAGE: AUTOPILOT_VERSION, MESSAGE_INTERVAL (param2 the message id) or one message of a stream, from the
// latest snapshot (TEMPORARILY_REJECTED before the first rate tick).
void on_request(const Envelope& from, const CommandLong& m) noexcept {
  if (same_bits(m.param1, kAutopilotVersionId)) {
    send_ack(from, m.command, standard::kMavResultAccepted);
    send_version();
    return;
  }
  if (same_bits(m.param1, kMessageIntervalId)) {
    send_ack(from, m.command, standard::kMavResultAccepted);
    send_interval(m.param2);
    return;
  }
  const std::size_t k = stream_of(m.param1);
  if (!can_send(k)) {
    send_ack(from, m.command, standard::kMavResultUnsupported);
    return;
  }
  Telemetry t{};
  if (kStreamInfo[k].snapshot && !g.config.telemetry(t)) {
    send_ack(from, m.command, standard::kMavResultTemporarilyRejected);
    return;
  }
  send_ack(from, m.command, standard::kMavResultAccepted);
  send_stream(static_cast<Stream>(k), t);
}

void on_command(const Envelope& from, const CommandLong& m) noexcept {
  if (!for_us(m.target_system, m.target_component)) {
    return;
  }
  if (m.command == standard::kMavCmdPreflightRebootShutdown) {
    on_reboot(from, m);
  } else if (m.command == standard::kMavCmdSetMessageInterval) {
    on_set_interval(from, m);
  } else if (m.command == standard::kMavCmdRequestMessage) {
    on_request(from, m);
  } else {
    send_ack(from, m.command, standard::kMavResultUnsupported);
  }
}

void on_read(const ParamRequestRead& m) noexcept {
  if (!for_us(m.target_system, m.target_component)) {
    return;
  }
  const std::size_t i = m.param_index >= 0 ? static_cast<std::size_t>(m.param_index) : index_of(m.param_id);
  if (i < kParamCount) {
    send_parameter(i);
  }
}

void on_set(const ParamSet& m) noexcept {
  if (!for_us(m.target_system, m.target_component)) {
    return;
  }
  const std::size_t i = index_of(m.param_id);
  if (i >= kParamCount) {
    std::array<char, std::tuple_size_v<Id> + 1> name{};
    std::copy(m.param_id.begin(), m.param_id.end(), name.begin());
    send_text({name.data(), ": no parameter has this id"});
    return;
  }
  ParamType type{};
  const store::StageResult r =
      type_of(m.param_type, type)
          ? store::stage_set(i, type, m.param_value, nullptr)
          : store::StageResult{store::StageRefusal::Type, store::stage_refusal_text(store::StageRefusal::Type)};
  if (!r.ok()) {
    send_text({link_id_text(i), ": ", r.reason});
  }
  send_value(i);
}

void source_read(const Envelope& from, const MarvParamSource& m) noexcept {
  if (m.param_schema_hash != kParamSchemaHash) {
    send_text({"source read ", kSchemaRefused});
    return;
  }
  if (m.param_index >= kParamCount) {
    send_text({"source read refused: no parameter has this index"});
    return;
  }
  const char* text = staged()[m.param_index].source;
  const std::size_t length = source_length(text);
  if (m.offset > length) {
    send_text({link_id_text(m.param_index), ": source read refused: the offset is past the end of the text"});
    return;
  }
  MarvParamSource d{};
  d.param_schema_hash = kParamSchemaHash;
  d.param_index = m.param_index;
  d.offset = m.offset;
  d.total_length = narrow<std::uint16_t>(length);
  d.target_system = from.system_id;
  d.target_component = from.component_id;
  d.operation = MarvParamSourceOp::Data;
  d.chunk_length = narrow<std::uint8_t>(std::min(kSourceChunkBytes, length - m.offset));
  std::copy_n(text + m.offset, d.chunk_length, d.chunk.begin());
  send(d);
}

void source_set(const MarvParamSource& m) noexcept {
  const auto refuse = [](std::initializer_list<const char*> why) {
    g.source_index = kParamCount;
    send_text(why);
  };
  if (m.param_schema_hash != kParamSchemaHash) {
    refuse({"source set ", kSchemaRefused});
    return;
  }
  if (m.param_index >= kParamCount) {
    refuse({"source set refused: no parameter has this index"});
    return;
  }
  const char* name = link_id_text(m.param_index);
  if (m.chunk_length > kSourceChunkBytes) {
    refuse({name, ": source set refused: chunk_length is longer than the chunk"});
    return;
  }
  if (m.offset == 0) {
    if (m.total_length == 0) {
      refuse({name, ": source set refused: the source must be non-empty text"});
      return;
    }
    g.source_index = m.param_index;
    g.source_total = m.total_length;
    g.source_have = 0;
  } else if (g.source_index != m.param_index || m.offset != g.source_have || m.total_length != g.source_total) {
    refuse({name, ": source set refused: a chunk out of order (a text starts again at offset 0)"});
    return;
  }
  if (g.source_have + m.chunk_length > g.source_total) {
    refuse({name, ": source set refused: the chunk runs past total_length"});
    return;
  }
  std::copy_n(m.chunk.begin(), m.chunk_length, g.source.begin() + static_cast<std::ptrdiff_t>(g.source_have));
  g.source_have += m.chunk_length;
  if (g.source_have < g.source_total) {
    return;
  }
  g.source_index = kParamCount;
  const ParamRecord s = staged()[m.param_index];
  const store::StageProvenance p{s.origin, s.method, s.sigma_kind, s.sigma, g.source.data(), g.source_total};
  const store::StageResult r = store::stage_set(m.param_index, s.value.type, bits_of(s.value), &p);
  if (!r.ok()) {
    send_text({name, ": source set refused: ", r.reason});
  }
}

void on_source(const Envelope& from, const MarvParamSource& m) noexcept {
  if (!for_us(m.target_system, m.target_component)) {
    return;
  }
  if (m.operation == MarvParamSourceOp::Read) {
    source_read(from, m);
  } else if (m.operation == MarvParamSourceOp::Set) {
    source_set(m);
  }
}

// The answer to an accepted store command; `reason` is set for a refusal or a failure.
[[nodiscard]] MarvConfigResult started(const store::CommandResult& c, const char*& reason) noexcept {
  switch (c.step) {
    case store::CommandStep::Refused:
      reason = c.reason;
      return c.reason == store::fault_text(store::Fault::NoLastGood) ? MarvConfigResult::DeniedUnavailable
                                                                     : MarvConfigResult::Failed;
    case store::CommandStep::WaitingForGrant:
      g.reboot_pending = true;
      return MarvConfigResult::Accepted;
    case store::CommandStep::RebootRequested:
      g.reboot_requested = true;
      return MarvConfigResult::Accepted;
    case store::CommandStep::RebootFailed:
      reason = c.reason;
      return MarvConfigResult::Failed;
  }
  return MarvConfigResult::Failed;
}

void on_config(const Envelope& from, const MarvConfigCommand& m) noexcept {
  if (!for_us(m.target_system, m.target_component)) {
    return;
  }
  const char* reason = nullptr;
  MarvConfigResult result = MarvConfigResult::Failed;
  if (m.param_schema_hash != kParamSchemaHash) {
    result = MarvConfigResult::DeniedHash;
    reason = kSchemaRefused;
  } else if (armed()) {
    result = MarvConfigResult::DeniedArmed;
    reason = "refused while armed";
  } else if (m.action == MarvConfigAction::Activate) {
    if (all_zero(m.report_id)) {
      result = MarvConfigResult::DeniedReport;
      reason = "refused: no validation report id, so the config cannot be bound to this image and a report (G7)";
    } else if (store::config_hash(staged()) != m.expected_config_sha256) {
      result = MarvConfigResult::DeniedHash;
      reason = "refused: the staged config does not hash to expected_config_sha256";
    } else {
      result = started(store::activate(store::Binding{g.config.image_sha256, m.report_id}), reason);
    }
  } else if (m.action == MarvConfigAction::RevertLastGood) {
    result = started(store::revert_to_last_good(), reason);
  } else if (m.action == MarvConfigAction::ResetFactory) {
    result = started(store::reset_to_factory(), reason);
  } else {
    reason = "refused: not a config action";
  }
  if (reason != nullptr) {
    send_text({"config command ", reason});
  }
  send(MarvConfigAck{from.system_id, from.component_id, m.action, result});
}

void dispatch(const Frame& f) noexcept {
  const Message& m = f.message;
  if (const auto* list = std::get_if<ParamRequestList>(&m)) {
    if (for_us(list->target_system, list->target_component)) {
      g.listing_next = 0;
    }
  } else if (const auto* read = std::get_if<ParamRequestRead>(&m)) {
    on_read(*read);
  } else if (const auto* set = std::get_if<ParamSet>(&m)) {
    on_set(*set);
  } else if (const auto* command = std::get_if<CommandLong>(&m)) {
    on_command(f.envelope, *command);
  } else if (const auto* source = std::get_if<MarvParamSource>(&m)) {
    on_source(f.envelope, *source);
  } else if (const auto* config = std::get_if<MarvConfigCommand>(&m)) {
    on_config(f.envelope, *config);
  }
}

void step_reboot() noexcept {
  const store::CommandResult r = store::reboot_when_granted();
  if (r.step == store::CommandStep::WaitingForGrant) {
    return;
  }
  g.reboot_pending = false;
  if (r.step == store::CommandStep::RebootRequested) {
    g.reboot_requested = true;
  } else {
    send_text({"reboot failed: ", r.reason != nullptr ? r.reason : "no reason given"});
  }
}

// A reboot command waiting for the grant: asked again each step (the tick drops a request it finds armed); DENIED if the vehicle
// is armed before the grant; once granted ACCEPTED, and a reboot goes through the store (hal_reboot).
void step_grant() noexcept {
  const std::uint16_t command = standard::kMavCmdPreflightRebootShutdown;
  if (!g.config.maintenance_granted()) {
    if (armed()) {
      g.grant_pending = false;
      send_text({"reboot refused: the vehicle armed before the maintenance grant"});
      send_ack(g.grant_from, command, standard::kMavResultDenied);
    } else {
      g.config.request_maintenance();
    }
    return;
  }
  g.grant_pending = false;
  if (!g.grant_reboots) {
    send_ack(g.grant_from, command, standard::kMavResultAccepted);
    return;
  }
  const store::CommandResult r = store::reboot_when_granted();
  if (r.step == store::CommandStep::RebootRequested) {
    send_ack(g.grant_from, command, standard::kMavResultAccepted);
    g.reboot_requested = true;
  } else {
    send_text({"reboot failed: ", r.reason != nullptr ? r.reason : "no reason given"});
    send_ack(g.grant_from, command, standard::kMavResultFailed);
  }
}

// Room for a stream's frame beside one parameter's frames (room_for_parameter), so a listing and the answers keep their room.
[[nodiscard]] bool room_for(std::size_t frame_bytes) noexcept {
  return g.config.transport.writable == nullptr || g.config.transport.writable() >= frame_bytes + 2 * kFrameBytesMax;
}

// Every running stream that is due and has room sends once; the next is due one interval later, or one interval from now if
// it fell an interval behind. A stream that waits for room or for the first snapshot stays due.
void step_streams(TimeUs now) noexcept {
  Telemetry t{};
  bool read = false;
  bool have = false;
  for (std::size_t k = 0; k < kStreams; ++k) {
    const std::uint32_t interval = g.interval_us[k];
    if (interval == 0 || now < g.next_us[k]) {
      continue;
    }
    if (kStreamInfo[k].snapshot) {
      if (!read) {
        have = g.config.telemetry(t);
        read = true;
      }
      if (!have) {
        continue;
      }
    }
    if (!room_for(kStreamInfo[k].frame_bytes)) {
      continue;
    }
    send_stream(static_cast<Stream>(k), t);
    g.next_us[k] += interval;
    if (g.next_us[k] <= now) {
      g.next_us[k] = now + interval;
    }
  }
}

}  // namespace

void service_init(const ServiceConfig& config) noexcept {
  if (g.started) {
    hal_panic("link::service_init called twice");
  }
  if (config.transport.receive == nullptr || config.transport.send == nullptr) {
    hal_panic("link::service_init: the transport lacks receive or send");
  }
  g.config = config;
  g.parser.reset();
  g.running_hash = store::config_hash(param_active_table());
  for (std::size_t k = 0; k < kStreams; ++k) {
    const ParamRecord r = param_get(kStreamInfo[k].cap);
    g.cap_hz[k] = r.value.type == ParamType::F32 ? rate_or_zero(r.value.f32) : 0.0F;
  }
  g.budget_bytes_s = rate_or_zero(param_value<ParamId::telemetry_byte_rate_max>());
  g.rc_loss_us = static_cast<TimeUs>(std::max<std::int32_t>(param_value<ParamId::failsafe_loss_us>(), 0));
  g.started = true;
}

bool service_step() noexcept {
  if (!g.started) {
    return false;
  }
  std::array<std::uint8_t, kFrameBytesMax> rx{};
  for (std::size_t n = g.config.transport.receive(rx); n > 0; n = g.config.transport.receive(rx)) {
    for (std::size_t k = 0; k < n; ++k) {
      const Status status = g.parser.push(rx[k]);
      if (status == Status::Ok) {
        ++g.rx_frames;
        dispatch(g.parser.frame());
      } else if (status == Status::Unsupported) {
        ++g.rx_frames;
      } else if (status != Status::Pending) {
        ++g.rx_errors;
      }
    }
  }
  const TimeUs now = hal_time_us();
  if (!g.heartbeat_sent || now - g.last_heartbeat_us >= kHeartbeatPeriodUs) {
    send_heartbeat();
    g.heartbeat_sent = true;
    g.last_heartbeat_us = now;
  }
  if (g.config.transport.connected != nullptr && !g.config.transport.connected()) {
    g.listing_next = kParamCount;
    g.interval_us.fill(0);
  }
  if (g.listing_next < kParamCount && room_for_parameter()) {
    send_parameter(g.listing_next);
    ++g.listing_next;
  }
  step_streams(now);
  if (g.reboot_pending) {
    step_reboot();
  }
  if (g.grant_pending) {
    step_grant();
  }
  return g.reboot_requested;
}

}  // namespace marv::link
