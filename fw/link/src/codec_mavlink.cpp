// The MAVLink v2 codec with the MARV dialect (GS-3: this is the only file that includes MAVLink headers). The generated C
// headers (pymavlink mavgen from link/marv.xml, MIT under the exception in pymavlink's COPYING, THIRD_PARTY_NOTICES.md)
// are SYSTEM includes of this library. Each message of link/scope.txt has a `fill` (plain struct to wire struct) and a
// `take` (wire struct to plain struct) below; the array helpers fail to compile if a plain struct and the dialect disagree.
#include "marv/link/codec_mavlink.hpp"

#include <marv/mavlink.h>

#include <algorithm>
#include <array>
#include <cstring>
#include <type_traits>
#include <variant>

namespace marv::link {
namespace {

template <class T, std::size_t N>
void put(T (&wire)[N], const std::array<T, N>& plain) noexcept {
  std::memcpy(wire, plain.data(), sizeof(wire));
}

template <class T, std::size_t N>
void get(const T (&wire)[N], std::array<T, N>& plain) noexcept {
  std::memcpy(plain.data(), wire, sizeof(wire));
}

template <class E>
std::uint8_t raw(E value) noexcept {
  return static_cast<std::uint8_t>(value);
}

// The raw-bits fields (tools/link/gen_cpp_structs.py RAW_BITS_FIELDS): the bytes are copied, never the value through a float,
// so an INT32 parameter whose bits are a signalling-NaN pattern arrives unchanged.
void put_bits(float& wire, std::uint32_t plain) noexcept {
  static_assert(sizeof(wire) == sizeof(plain));
  std::memcpy(&wire, &plain, sizeof(wire));
}
void get_bits(const float& wire, std::uint32_t& plain) noexcept {
  static_assert(sizeof(wire) == sizeof(plain));
  std::memcpy(&plain, &wire, sizeof(plain));
}

void fill(const Heartbeat& m, mavlink_heartbeat_t& w) noexcept {
  w.type = m.type;
  w.autopilot = m.autopilot;
  w.base_mode = m.base_mode;
  w.custom_mode = m.custom_mode;
  w.system_status = m.system_status;
}
void take(const mavlink_heartbeat_t& w, Heartbeat& m) noexcept {
  m.type = w.type;
  m.autopilot = w.autopilot;
  m.base_mode = w.base_mode;
  m.custom_mode = w.custom_mode;
  m.system_status = w.system_status;
}

void fill(const AutopilotVersion& m, mavlink_autopilot_version_t& w) noexcept {
  w.capabilities = m.capabilities;
  w.flight_sw_version = m.flight_sw_version;
  w.middleware_sw_version = m.middleware_sw_version;
  w.os_sw_version = m.os_sw_version;
  w.board_version = m.board_version;
  put(w.flight_custom_version, m.flight_custom_version);
  put(w.middleware_custom_version, m.middleware_custom_version);
  put(w.os_custom_version, m.os_custom_version);
  w.vendor_id = m.vendor_id;
  w.product_id = m.product_id;
  w.uid = m.uid;
  put(w.uid2, m.uid2);
}
void take(const mavlink_autopilot_version_t& w, AutopilotVersion& m) noexcept {
  m.capabilities = w.capabilities;
  m.flight_sw_version = w.flight_sw_version;
  m.middleware_sw_version = w.middleware_sw_version;
  m.os_sw_version = w.os_sw_version;
  m.board_version = w.board_version;
  get(w.flight_custom_version, m.flight_custom_version);
  get(w.middleware_custom_version, m.middleware_custom_version);
  get(w.os_custom_version, m.os_custom_version);
  m.vendor_id = w.vendor_id;
  m.product_id = w.product_id;
  m.uid = w.uid;
  get(w.uid2, m.uid2);
}

void fill(const ParamRequestList& m, mavlink_param_request_list_t& w) noexcept {
  w.target_system = m.target_system;
  w.target_component = m.target_component;
}
void take(const mavlink_param_request_list_t& w, ParamRequestList& m) noexcept {
  m.target_system = w.target_system;
  m.target_component = w.target_component;
}

void fill(const ParamRequestRead& m, mavlink_param_request_read_t& w) noexcept {
  w.target_system = m.target_system;
  w.target_component = m.target_component;
  put(w.param_id, m.param_id);
  w.param_index = m.param_index;
}
void take(const mavlink_param_request_read_t& w, ParamRequestRead& m) noexcept {
  m.target_system = w.target_system;
  m.target_component = w.target_component;
  get(w.param_id, m.param_id);
  m.param_index = w.param_index;
}

void fill(const ParamSet& m, mavlink_param_set_t& w) noexcept {
  w.target_system = m.target_system;
  w.target_component = m.target_component;
  put(w.param_id, m.param_id);
  put_bits(w.param_value, m.param_value);
  w.param_type = m.param_type;
}
void take(const mavlink_param_set_t& w, ParamSet& m) noexcept {
  m.target_system = w.target_system;
  m.target_component = w.target_component;
  get(w.param_id, m.param_id);
  get_bits(w.param_value, m.param_value);
  m.param_type = w.param_type;
}

void fill(const ParamValue& m, mavlink_param_value_t& w) noexcept {
  put(w.param_id, m.param_id);
  put_bits(w.param_value, m.param_value);
  w.param_type = m.param_type;
  w.param_count = m.param_count;
  w.param_index = m.param_index;
}
void take(const mavlink_param_value_t& w, ParamValue& m) noexcept {
  get(w.param_id, m.param_id);
  get_bits(w.param_value, m.param_value);
  m.param_type = w.param_type;
  m.param_count = w.param_count;
  m.param_index = w.param_index;
}

void fill(const Statustext& m, mavlink_statustext_t& w) noexcept {
  w.severity = m.severity;
  put(w.text, m.text);
  w.id = m.id;
  w.chunk_seq = m.chunk_seq;
}
void take(const mavlink_statustext_t& w, Statustext& m) noexcept {
  m.severity = w.severity;
  get(w.text, m.text);
  m.id = w.id;
  m.chunk_seq = w.chunk_seq;
}

void fill(const CommandLong& m, mavlink_command_long_t& w) noexcept {
  w.target_system = m.target_system;
  w.target_component = m.target_component;
  w.command = m.command;
  w.confirmation = m.confirmation;
  w.param1 = m.param1;
  w.param2 = m.param2;
  w.param3 = m.param3;
  w.param4 = m.param4;
  w.param5 = m.param5;
  w.param6 = m.param6;
  w.param7 = m.param7;
}
void take(const mavlink_command_long_t& w, CommandLong& m) noexcept {
  m.target_system = w.target_system;
  m.target_component = w.target_component;
  m.command = w.command;
  m.confirmation = w.confirmation;
  m.param1 = w.param1;
  m.param2 = w.param2;
  m.param3 = w.param3;
  m.param4 = w.param4;
  m.param5 = w.param5;
  m.param6 = w.param6;
  m.param7 = w.param7;
}

void fill(const CommandAck& m, mavlink_command_ack_t& w) noexcept {
  w.command = m.command;
  w.result = m.result;
  w.progress = m.progress;
  w.result_param2 = m.result_param2;
  w.target_system = m.target_system;
  w.target_component = m.target_component;
}
void take(const mavlink_command_ack_t& w, CommandAck& m) noexcept {
  m.command = w.command;
  m.result = w.result;
  m.progress = w.progress;
  m.result_param2 = w.result_param2;
  m.target_system = w.target_system;
  m.target_component = w.target_component;
}

void fill(const MarvIdentity& m, mavlink_marv_identity_t& w) noexcept {
  w.param_schema_hash = m.param_schema_hash;
  put(w.image_sha256, m.image_sha256);
  put(w.config_sha256, m.config_sha256);
  put(w.validation_report_id, m.validation_report_id);
  w.config_source = raw(m.config_source);
  w.config_state = raw(m.config_state);
  w.config_refusal = raw(m.config_refusal);
  w.last_activation_result = raw(m.last_activation_result);
  w.staged_differs = m.staged_differs;
}
void take(const mavlink_marv_identity_t& w, MarvIdentity& m) noexcept {
  m.param_schema_hash = w.param_schema_hash;
  get(w.image_sha256, m.image_sha256);
  get(w.config_sha256, m.config_sha256);
  get(w.validation_report_id, m.validation_report_id);
  m.config_source = static_cast<MarvConfigSource>(w.config_source);
  m.config_state = static_cast<MarvConfigState>(w.config_state);
  m.config_refusal = static_cast<MarvConfigRefusal>(w.config_refusal);
  m.last_activation_result = static_cast<MarvActivationResult>(w.last_activation_result);
  m.staged_differs = w.staged_differs;
}

void fill(const MarvParamRecord& m, mavlink_marv_param_record_t& w) noexcept {
  w.active_bits = m.active_bits;
  w.staged_bits = m.staged_bits;
  w.sigma = m.sigma;
  w.param_index = m.param_index;
  w.param_count = m.param_count;
  w.source_length = m.source_length;
  w.origin = raw(m.origin);
  w.method = raw(m.method);
  w.sigma_kind = raw(m.sigma_kind);
  w.lock = m.lock;
  w.source_chunk_length = m.source_chunk_length;
  put(w.source_chunk, m.source_chunk);
}
void take(const mavlink_marv_param_record_t& w, MarvParamRecord& m) noexcept {
  m.active_bits = w.active_bits;
  m.staged_bits = w.staged_bits;
  m.sigma = w.sigma;
  m.param_index = w.param_index;
  m.param_count = w.param_count;
  m.source_length = w.source_length;
  m.origin = static_cast<MarvParamOrigin>(w.origin);
  m.method = static_cast<MarvParamMethod>(w.method);
  m.sigma_kind = static_cast<MarvParamSigmaKind>(w.sigma_kind);
  m.lock = w.lock;
  m.source_chunk_length = w.source_chunk_length;
  get(w.source_chunk, m.source_chunk);
}

void fill(const MarvParamSource& m, mavlink_marv_param_source_t& w) noexcept {
  w.param_schema_hash = m.param_schema_hash;
  w.param_index = m.param_index;
  w.offset = m.offset;
  w.total_length = m.total_length;
  w.target_system = m.target_system;
  w.target_component = m.target_component;
  w.operation = raw(m.operation);
  w.chunk_length = m.chunk_length;
  put(w.chunk, m.chunk);
}
void take(const mavlink_marv_param_source_t& w, MarvParamSource& m) noexcept {
  m.param_schema_hash = w.param_schema_hash;
  m.param_index = w.param_index;
  m.offset = w.offset;
  m.total_length = w.total_length;
  m.target_system = w.target_system;
  m.target_component = w.target_component;
  m.operation = static_cast<MarvParamSourceOp>(w.operation);
  m.chunk_length = w.chunk_length;
  get(w.chunk, m.chunk);
}

void fill(const MarvConfigCommand& m, mavlink_marv_config_command_t& w) noexcept {
  w.param_schema_hash = m.param_schema_hash;
  put(w.report_id, m.report_id);
  put(w.expected_config_sha256, m.expected_config_sha256);
  w.target_system = m.target_system;
  w.target_component = m.target_component;
  w.action = raw(m.action);
}
void take(const mavlink_marv_config_command_t& w, MarvConfigCommand& m) noexcept {
  m.param_schema_hash = w.param_schema_hash;
  get(w.report_id, m.report_id);
  get(w.expected_config_sha256, m.expected_config_sha256);
  m.target_system = w.target_system;
  m.target_component = w.target_component;
  m.action = static_cast<MarvConfigAction>(w.action);
}

void fill(const MarvConfigAck& m, mavlink_marv_config_ack_t& w) noexcept {
  w.target_system = m.target_system;
  w.target_component = m.target_component;
  w.action = raw(m.action);
  w.result = raw(m.result);
}
void take(const mavlink_marv_config_ack_t& w, MarvConfigAck& m) noexcept {
  m.target_system = w.target_system;
  m.target_component = w.target_component;
  m.action = static_cast<MarvConfigAction>(w.action);
  m.result = static_cast<MarvConfigResult>(w.result);
}

void fill(const SysStatus& m, mavlink_sys_status_t& w) noexcept {
  w.onboard_control_sensors_present = m.onboard_control_sensors_present;
  w.onboard_control_sensors_enabled = m.onboard_control_sensors_enabled;
  w.onboard_control_sensors_health = m.onboard_control_sensors_health;
  w.load = m.load;
  w.voltage_battery = m.voltage_battery;
  w.current_battery = m.current_battery;
  w.battery_remaining = m.battery_remaining;
  w.drop_rate_comm = m.drop_rate_comm;
  w.errors_comm = m.errors_comm;
  w.errors_count1 = m.errors_count1;
  w.errors_count2 = m.errors_count2;
  w.errors_count3 = m.errors_count3;
  w.errors_count4 = m.errors_count4;
  w.onboard_control_sensors_present_extended = m.onboard_control_sensors_present_extended;
  w.onboard_control_sensors_enabled_extended = m.onboard_control_sensors_enabled_extended;
  w.onboard_control_sensors_health_extended = m.onboard_control_sensors_health_extended;
}
void take(const mavlink_sys_status_t& w, SysStatus& m) noexcept {
  m.onboard_control_sensors_present = w.onboard_control_sensors_present;
  m.onboard_control_sensors_enabled = w.onboard_control_sensors_enabled;
  m.onboard_control_sensors_health = w.onboard_control_sensors_health;
  m.load = w.load;
  m.voltage_battery = w.voltage_battery;
  m.current_battery = w.current_battery;
  m.battery_remaining = w.battery_remaining;
  m.drop_rate_comm = w.drop_rate_comm;
  m.errors_comm = w.errors_comm;
  m.errors_count1 = w.errors_count1;
  m.errors_count2 = w.errors_count2;
  m.errors_count3 = w.errors_count3;
  m.errors_count4 = w.errors_count4;
  m.onboard_control_sensors_present_extended = w.onboard_control_sensors_present_extended;
  m.onboard_control_sensors_enabled_extended = w.onboard_control_sensors_enabled_extended;
  m.onboard_control_sensors_health_extended = w.onboard_control_sensors_health_extended;
}

void fill(const Attitude& m, mavlink_attitude_t& w) noexcept {
  w.time_boot_ms = m.time_boot_ms;
  w.roll = m.roll;
  w.pitch = m.pitch;
  w.yaw = m.yaw;
  w.rollspeed = m.rollspeed;
  w.pitchspeed = m.pitchspeed;
  w.yawspeed = m.yawspeed;
}
void take(const mavlink_attitude_t& w, Attitude& m) noexcept {
  m.time_boot_ms = w.time_boot_ms;
  m.roll = w.roll;
  m.pitch = w.pitch;
  m.yaw = w.yaw;
  m.rollspeed = w.rollspeed;
  m.pitchspeed = w.pitchspeed;
  m.yawspeed = w.yawspeed;
}

void fill(const AttitudeQuaternion& m, mavlink_attitude_quaternion_t& w) noexcept {
  w.time_boot_ms = m.time_boot_ms;
  w.q1 = m.q1;
  w.q2 = m.q2;
  w.q3 = m.q3;
  w.q4 = m.q4;
  w.rollspeed = m.rollspeed;
  w.pitchspeed = m.pitchspeed;
  w.yawspeed = m.yawspeed;
  put(w.repr_offset_q, m.repr_offset_q);
}
void take(const mavlink_attitude_quaternion_t& w, AttitudeQuaternion& m) noexcept {
  m.time_boot_ms = w.time_boot_ms;
  m.q1 = w.q1;
  m.q2 = w.q2;
  m.q3 = w.q3;
  m.q4 = w.q4;
  m.rollspeed = w.rollspeed;
  m.pitchspeed = w.pitchspeed;
  m.yawspeed = w.yawspeed;
  get(w.repr_offset_q, m.repr_offset_q);
}

void fill(const RcChannels& m, mavlink_rc_channels_t& w) noexcept {
  w.time_boot_ms = m.time_boot_ms;
  w.chancount = m.chancount;
  w.chan1_raw = m.chan1_raw;
  w.chan2_raw = m.chan2_raw;
  w.chan3_raw = m.chan3_raw;
  w.chan4_raw = m.chan4_raw;
  w.chan5_raw = m.chan5_raw;
  w.chan6_raw = m.chan6_raw;
  w.chan7_raw = m.chan7_raw;
  w.chan8_raw = m.chan8_raw;
  w.chan9_raw = m.chan9_raw;
  w.chan10_raw = m.chan10_raw;
  w.chan11_raw = m.chan11_raw;
  w.chan12_raw = m.chan12_raw;
  w.chan13_raw = m.chan13_raw;
  w.chan14_raw = m.chan14_raw;
  w.chan15_raw = m.chan15_raw;
  w.chan16_raw = m.chan16_raw;
  w.chan17_raw = m.chan17_raw;
  w.chan18_raw = m.chan18_raw;
  w.rssi = m.rssi;
}
void take(const mavlink_rc_channels_t& w, RcChannels& m) noexcept {
  m.time_boot_ms = w.time_boot_ms;
  m.chancount = w.chancount;
  m.chan1_raw = w.chan1_raw;
  m.chan2_raw = w.chan2_raw;
  m.chan3_raw = w.chan3_raw;
  m.chan4_raw = w.chan4_raw;
  m.chan5_raw = w.chan5_raw;
  m.chan6_raw = w.chan6_raw;
  m.chan7_raw = w.chan7_raw;
  m.chan8_raw = w.chan8_raw;
  m.chan9_raw = w.chan9_raw;
  m.chan10_raw = w.chan10_raw;
  m.chan11_raw = w.chan11_raw;
  m.chan12_raw = w.chan12_raw;
  m.chan13_raw = w.chan13_raw;
  m.chan14_raw = w.chan14_raw;
  m.chan15_raw = w.chan15_raw;
  m.chan16_raw = w.chan16_raw;
  m.chan17_raw = w.chan17_raw;
  m.chan18_raw = w.chan18_raw;
  m.rssi = w.rssi;
}

void fill(const AttitudeTarget& m, mavlink_attitude_target_t& w) noexcept {
  w.time_boot_ms = m.time_boot_ms;
  w.type_mask = m.type_mask;
  put(w.q, m.q);
  w.body_roll_rate = m.body_roll_rate;
  w.body_pitch_rate = m.body_pitch_rate;
  w.body_yaw_rate = m.body_yaw_rate;
  w.thrust = m.thrust;
}
void take(const mavlink_attitude_target_t& w, AttitudeTarget& m) noexcept {
  m.time_boot_ms = w.time_boot_ms;
  m.type_mask = w.type_mask;
  get(w.q, m.q);
  m.body_roll_rate = w.body_roll_rate;
  m.body_pitch_rate = w.body_pitch_rate;
  m.body_yaw_rate = w.body_yaw_rate;
  m.thrust = w.thrust;
}

void fill(const HighresImu& m, mavlink_highres_imu_t& w) noexcept {
  w.time_usec = m.time_usec;
  w.xacc = m.xacc;
  w.yacc = m.yacc;
  w.zacc = m.zacc;
  w.xgyro = m.xgyro;
  w.ygyro = m.ygyro;
  w.zgyro = m.zgyro;
  w.xmag = m.xmag;
  w.ymag = m.ymag;
  w.zmag = m.zmag;
  w.abs_pressure = m.abs_pressure;
  w.diff_pressure = m.diff_pressure;
  w.pressure_alt = m.pressure_alt;
  w.temperature = m.temperature;
  w.fields_updated = m.fields_updated;
  w.id = m.id;
}
void take(const mavlink_highres_imu_t& w, HighresImu& m) noexcept {
  m.time_usec = w.time_usec;
  m.xacc = w.xacc;
  m.yacc = w.yacc;
  m.zacc = w.zacc;
  m.xgyro = w.xgyro;
  m.ygyro = w.ygyro;
  m.zgyro = w.zgyro;
  m.xmag = w.xmag;
  m.ymag = w.ymag;
  m.zmag = w.zmag;
  m.abs_pressure = w.abs_pressure;
  m.diff_pressure = w.diff_pressure;
  m.pressure_alt = w.pressure_alt;
  m.temperature = w.temperature;
  m.fields_updated = w.fields_updated;
  m.id = w.id;
}

void fill(const MessageInterval& m, mavlink_message_interval_t& w) noexcept {
  w.message_id = m.message_id;
  w.interval_us = m.interval_us;
}
void take(const mavlink_message_interval_t& w, MessageInterval& m) noexcept {
  m.message_id = w.message_id;
  m.interval_us = w.interval_us;
}

void fill(const ActuatorOutputStatus& m, mavlink_actuator_output_status_t& w) noexcept {
  w.time_usec = m.time_usec;
  w.active = m.active;
  put(w.actuator, m.actuator);
}
void take(const mavlink_actuator_output_status_t& w, ActuatorOutputStatus& m) noexcept {
  m.time_usec = w.time_usec;
  m.active = w.active;
  get(w.actuator, m.actuator);
}

void fill(const MarvFcStatus& m, mavlink_marv_fc_status_t& w) noexcept {
  w.time_usec = m.time_usec;
  w.rate_executions = m.rate_executions;
  w.rate_faults = m.rate_faults;
  w.rate_timing_faults = m.rate_timing_faults;
  w.attitude_faults = m.attitude_faults;
  w.attitude_timing_faults = m.attitude_timing_faults;
  w.imu_deadline_misses = m.imu_deadline_misses;
  w.rc_frames = m.rc_frames;
  w.rc_age_us = m.rc_age_us;
  w.arm_state = raw(m.arm_state);
  w.motor_gate = raw(m.motor_gate);
  w.flags = m.flags;
  w.rate_fault = m.rate_fault;
}
void take(const mavlink_marv_fc_status_t& w, MarvFcStatus& m) noexcept {
  m.time_usec = w.time_usec;
  m.rate_executions = w.rate_executions;
  m.rate_faults = w.rate_faults;
  m.rate_timing_faults = w.rate_timing_faults;
  m.attitude_faults = w.attitude_faults;
  m.attitude_timing_faults = w.attitude_timing_faults;
  m.imu_deadline_misses = w.imu_deadline_misses;
  m.rc_frames = w.rc_frames;
  m.rc_age_us = w.rc_age_us;
  m.arm_state = static_cast<MarvArmState>(w.arm_state);
  m.motor_gate = static_cast<MarvMotorGate>(w.motor_gate);
  m.flags = w.flags;
  m.rate_fault = w.rate_fault;
}

void fill(const MarvEstimator& m, mavlink_marv_estimator_t& w) noexcept {
  w.time_usec = m.time_usec;
  put(w.q_mahony, m.q_mahony);
  put(w.q_eskf, m.q_eskf);
  put(w.gyro_bias_cal, m.gyro_bias_cal);
  put(w.gyro_bias_mahony, m.gyro_bias_mahony);
  put(w.gyro_bias_eskf, m.gyro_bias_eskf);
  w.flags = m.flags;
}
void take(const mavlink_marv_estimator_t& w, MarvEstimator& m) noexcept {
  m.time_usec = w.time_usec;
  get(w.q_mahony, m.q_mahony);
  get(w.q_eskf, m.q_eskf);
  get(w.gyro_bias_cal, m.gyro_bias_cal);
  get(w.gyro_bias_mahony, m.gyro_bias_mahony);
  get(w.gyro_bias_eskf, m.gyro_bias_eskf);
  m.flags = w.flags;
}

// Wire<T> binds a plain struct to its generated C message: the id, the longest payload, the encoder and the decoder.
template <class T>
struct Wire;

#define MARV_LINK_WIRE(Type, lower, UPPER)                                                                          \
  template <>                                                                                                       \
  struct Wire<Type> {                                                                                               \
    using W = mavlink_##lower##_t;                                                                                  \
    static constexpr std::uint32_t kId = MAVLINK_MSG_ID_##UPPER;                                                    \
    static constexpr std::uint32_t kMaxLength = MAVLINK_MSG_ID_##UPPER##_LEN;                                       \
    static std::uint16_t pack(std::uint8_t system, std::uint8_t component, mavlink_status_t* status,                \
                              mavlink_message_t* out, const W& wire) noexcept {                                     \
      return mavlink_msg_##lower##_encode_status(system, component, status, out, &wire);                            \
    }                                                                                                               \
    static void unpack(const mavlink_message_t& in, W& wire) noexcept { mavlink_msg_##lower##_decode(&in, &wire); } \
  };                                                                                                                \
  static_assert(Wire<Type>::kId == k##Type##Id, "plain struct id and dialect id differ");                           \
  static_assert(k##Type##FrameBytes == MAVLINK_NUM_NON_PAYLOAD_BYTES + MAVLINK_MSG_ID_##UPPER##_LEN,                \
                "plain struct frame length and dialect length differ")

MARV_LINK_WIRE(Heartbeat, heartbeat, HEARTBEAT);
MARV_LINK_WIRE(AutopilotVersion, autopilot_version, AUTOPILOT_VERSION);
MARV_LINK_WIRE(ParamRequestList, param_request_list, PARAM_REQUEST_LIST);
MARV_LINK_WIRE(ParamRequestRead, param_request_read, PARAM_REQUEST_READ);
MARV_LINK_WIRE(ParamSet, param_set, PARAM_SET);
MARV_LINK_WIRE(ParamValue, param_value, PARAM_VALUE);
MARV_LINK_WIRE(Statustext, statustext, STATUSTEXT);
MARV_LINK_WIRE(CommandLong, command_long, COMMAND_LONG);
MARV_LINK_WIRE(CommandAck, command_ack, COMMAND_ACK);
MARV_LINK_WIRE(MarvIdentity, marv_identity, MARV_IDENTITY);
MARV_LINK_WIRE(MarvParamRecord, marv_param_record, MARV_PARAM_RECORD);
MARV_LINK_WIRE(MarvParamSource, marv_param_source, MARV_PARAM_SOURCE);
MARV_LINK_WIRE(MarvConfigCommand, marv_config_command, MARV_CONFIG_COMMAND);
MARV_LINK_WIRE(MarvConfigAck, marv_config_ack, MARV_CONFIG_ACK);
MARV_LINK_WIRE(SysStatus, sys_status, SYS_STATUS);
MARV_LINK_WIRE(Attitude, attitude, ATTITUDE);
MARV_LINK_WIRE(AttitudeQuaternion, attitude_quaternion, ATTITUDE_QUATERNION);
MARV_LINK_WIRE(RcChannels, rc_channels, RC_CHANNELS);
MARV_LINK_WIRE(AttitudeTarget, attitude_target, ATTITUDE_TARGET);
MARV_LINK_WIRE(HighresImu, highres_imu, HIGHRES_IMU);
MARV_LINK_WIRE(MessageInterval, message_interval, MESSAGE_INTERVAL);
MARV_LINK_WIRE(ActuatorOutputStatus, actuator_output_status, ACTUATOR_OUTPUT_STATUS);
MARV_LINK_WIRE(MarvFcStatus, marv_fc_status, MARV_FC_STATUS);
MARV_LINK_WIRE(MarvEstimator, marv_estimator, MARV_ESTIMATOR);

#undef MARV_LINK_WIRE

// kFrameBytesMax (messages.hpp, from the dialect) is the library's unsigned frame of the longest message in scope.
template <class V>
struct LongestPayload;
template <class... T>
struct LongestPayload<std::variant<T...>> {
  static constexpr std::uint32_t kValue = std::max({Wire<T>::kMaxLength...});
};
static_assert(kFrameBytesMax == MAVLINK_NUM_NON_PAYLOAD_BYTES + LongestPayload<Message>::kValue,
              "kFrameBytesMax differs from the generated headers' frame of the longest message");

template <class T>
Status unpack(const mavlink_message_t& in, Message& out) noexcept {
  using Traits = Wire<T>;
  if (in.len > Traits::kMaxLength) {
    return Status::BadLength;
  }
  typename Traits::W wire{};
  Traits::unpack(in, wire);
  T plain{};
  take(wire, plain);
  out = plain;
  return Status::Ok;
}

Status unpack_message(const mavlink_message_t& in, Message& out) noexcept {
  switch (in.msgid) {
    case kHeartbeatId: return unpack<Heartbeat>(in, out);
    case kAutopilotVersionId: return unpack<AutopilotVersion>(in, out);
    case kParamRequestListId: return unpack<ParamRequestList>(in, out);
    case kParamRequestReadId: return unpack<ParamRequestRead>(in, out);
    case kParamSetId: return unpack<ParamSet>(in, out);
    case kParamValueId: return unpack<ParamValue>(in, out);
    case kStatustextId: return unpack<Statustext>(in, out);
    case kCommandLongId: return unpack<CommandLong>(in, out);
    case kCommandAckId: return unpack<CommandAck>(in, out);
    case kMarvIdentityId: return unpack<MarvIdentity>(in, out);
    case kMarvParamRecordId: return unpack<MarvParamRecord>(in, out);
    case kMarvParamSourceId: return unpack<MarvParamSource>(in, out);
    case kMarvConfigCommandId: return unpack<MarvConfigCommand>(in, out);
    case kMarvConfigAckId: return unpack<MarvConfigAck>(in, out);
    case kSysStatusId: return unpack<SysStatus>(in, out);
    case kAttitudeId: return unpack<Attitude>(in, out);
    case kAttitudeQuaternionId: return unpack<AttitudeQuaternion>(in, out);
    case kRcChannelsId: return unpack<RcChannels>(in, out);
    case kAttitudeTargetId: return unpack<AttitudeTarget>(in, out);
    case kHighresImuId: return unpack<HighresImu>(in, out);
    case kMessageIntervalId: return unpack<MessageInterval>(in, out);
    case kActuatorOutputStatusId: return unpack<ActuatorOutputStatus>(in, out);
    case kMarvFcStatusId: return unpack<MarvFcStatus>(in, out);
    case kMarvEstimatorId: return unpack<MarvEstimator>(in, out);
    default: return Status::Unsupported;
  }
}

}  // namespace

std::size_t encode_mavlink(const Envelope& envelope, const Message& message, std::span<std::uint8_t> out) noexcept {
  mavlink_status_t status{};
  status.current_tx_seq = envelope.sequence;
  mavlink_message_t packed{};
  std::visit(
      [&](const auto& plain) {
        using T = std::decay_t<decltype(plain)>;
        typename Wire<T>::W wire{};
        fill(plain, wire);
        Wire<T>::pack(envelope.system_id, envelope.component_id, &status, &packed, wire);
      },
      message);
  std::array<std::uint8_t, MAVLINK_MAX_PACKET_LEN> frame{};
  const std::size_t length = mavlink_msg_to_send_buffer(frame.data(), &packed);
  if (length > out.size()) {
    return 0;
  }
  std::memcpy(out.data(), frame.data(), length);
  return length;
}

MavlinkParser::MavlinkParser(std::uint8_t channel) noexcept : channel_(channel) {
  reset();
}

void MavlinkParser::reset() noexcept {
  if (channel_ < MAVLINK_COMM_NUM_BUFFERS) {
    mavlink_reset_channel_status(channel_);
  }
}

Status MavlinkParser::push(std::uint8_t byte) noexcept {
  if (channel_ >= MAVLINK_COMM_NUM_BUFFERS) {
    return Status::BadCrc;
  }
  mavlink_message_t received;
  mavlink_status_t status;
  // mavlink_frame_char, not mavlink_parse_char: the latter folds a bad checksum into "incomplete".
  switch (mavlink_frame_char(channel_, byte, &received, &status)) {
    case MAVLINK_FRAMING_OK: {
      const Status decoded = unpack_message(received, frame_.message);
      if (decoded == Status::Ok) {
        frame_.envelope = {received.sysid, received.compid, received.seq};
      }
      return decoded;
    }
    case MAVLINK_FRAMING_BAD_CRC:
    case MAVLINK_FRAMING_BAD_SIGNATURE:
      if (byte == MAVLINK_STX) {  // the checksum's last byte may itself start the next frame
        mavlink_frame_char(channel_, byte, &received, &status);
      }
      return Status::BadCrc;
    default: return Status::Pending;
  }
}

}  // namespace marv::link
