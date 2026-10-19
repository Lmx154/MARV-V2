// The sample messages of the link conformance suite: the same values as tests/regression/quad/L00/link/samples.hpp. A change
// here is a change there and in ../vectors/.
import type { Envelope } from '$link/codec';
import {
	MarvActivationResult,
	MarvArmState,
	MarvConfigAction,
	MarvConfigRefusal,
	MarvConfigResult,
	MarvConfigSource,
	MarvConfigState,
	MarvMotorGate,
	MarvParamMethod,
	MarvParamOrigin,
	MarvParamSigmaKind,
	MarvParamSourceOp,
	type Message
} from '$link/generated/messages';

export interface Sample {
	name: string;
	message: Message;
}

export function text(s: string, length: number): Uint8Array {
	const out = new Uint8Array(length);
	out.set(new TextEncoder().encode(s).subarray(0, length));
	return out;
}

export function ramp(first: number, length: number): Uint8Array {
	return Uint8Array.from({ length }, (_, i) => (first + i) & 0xff);
}

export const samples: Sample[] = [
	{ name: 'Heartbeat', message: { kind: 'Heartbeat', type: 2, autopilot: 8, base_mode: 81, custom_mode: 0x01020304, system_status: 4 } },
	{
		name: 'AutopilotVersion',
		message: {
			kind: 'AutopilotVersion',
			capabilities: 0x0123456789abcdefn,
			flight_sw_version: 0x01020300,
			middleware_sw_version: 0x04050600,
			os_sw_version: 0x07080900,
			board_version: 0x0a0b0c0d,
			flight_custom_version: ramp(1, 8),
			middleware_custom_version: ramp(20, 8),
			os_custom_version: ramp(40, 8),
			vendor_id: 0x1234,
			product_id: 0x5678,
			uid: 0xfedcba9876543210n,
			uid2: ramp(100, 18)
		}
	},
	{ name: 'ParamRequestList', message: { kind: 'ParamRequestList', target_system: 1, target_component: 2 } },
	{ name: 'ParamRequestListZero', message: { kind: 'ParamRequestList', target_system: 0, target_component: 0 } },
	{
		name: 'ParamRequestRead',
		message: { kind: 'ParamRequestRead', target_system: 1, target_component: 2, param_id: text('rate_kp_roll', 16), param_index: -1 }
	},
	{
		name: 'ParamSet',
		message: { kind: 'ParamSet', target_system: 1, target_component: 2, param_id: text('rotor_posit_ab12', 16), param_value: 0.25, param_type: 9 }
	},
	{
		name: 'ParamValue',
		message: { kind: 'ParamValue', param_id: text('mass_kg', 16), param_value: -1.5, param_type: 6, param_count: 136, param_index: 17 }
	},
	{ name: 'Statustext', message: { kind: 'Statustext', severity: 4, text: text('config refused: schema', 50), id: 5, chunk_seq: 1 } },
	{
		name: 'CommandLong',
		message: {
			kind: 'CommandLong',
			target_system: 1,
			target_component: 2,
			command: 512,
			confirmation: 1,
			param1: 148,
			param2: 2,
			param3: 3,
			param4: 4,
			param5: 5,
			param6: 6,
			param7: -1.5
		}
	},
	{
		name: 'CommandAck',
		message: { kind: 'CommandAck', command: 512, result: 3, progress: 255, result_param2: -7, target_system: 255, target_component: 190 }
	},
	{
		name: 'MarvIdentity',
		message: {
			kind: 'MarvIdentity',
			param_schema_hash: 0x2ff36d1da5fc3596n,
			image_sha256: ramp(1, 32),
			config_sha256: ramp(50, 32),
			validation_report_id: ramp(100, 32),
			config_source: MarvConfigSource.Active,
			config_state: MarvConfigState.Refused,
			config_refusal: MarvConfigRefusal.ReportBinding,
			last_activation_result: MarvActivationResult.RefusedAtBoot,
			staged_differs: 1
		}
	},
	{
		name: 'MarvParamRecord',
		message: {
			kind: 'MarvParamRecord',
			active_bits: 0x3f800000,
			staged_bits: 0xc0200000,
			sigma: 0.125,
			param_index: 17,
			param_count: 136,
			source_length: 20,
			origin: MarvParamOrigin.Manual,
			method: MarvParamMethod.Datasheet,
			sigma_kind: MarvParamSigmaKind.Known,
			lock: 1,
			source_chunk_length: 20,
			source_chunk: text('UZH NeuroBEM Table 3', 64)
		}
	},
	{
		name: 'MarvParamSource',
		message: {
			kind: 'MarvParamSource',
			param_schema_hash: 0x2ff36d1da5fc3596n,
			param_index: 17,
			offset: 64,
			total_length: 200,
			target_system: 1,
			target_component: 2,
			operation: MarvParamSourceOp.Set,
			chunk_length: 64,
			chunk: text('0123456789012345678901234567890123456789012345678901234567890123', 64)
		}
	},
	{
		name: 'MarvConfigCommand',
		message: {
			kind: 'MarvConfigCommand',
			param_schema_hash: 0x2ff36d1da5fc3596n,
			report_id: ramp(3, 32),
			expected_config_sha256: ramp(70, 32),
			target_system: 1,
			target_component: 2,
			action: MarvConfigAction.Activate
		}
	},
	{
		name: 'MarvConfigAck',
		message: { kind: 'MarvConfigAck', target_system: 255, target_component: 190, action: MarvConfigAction.ResetFactory, result: MarvConfigResult.DeniedArmed }
	},
	// The telemetry streams (ground-segment unit 11).
	{
		name: 'SysStatus',
		message: {
			kind: 'SysStatus',
			onboard_control_sensors_present: 0x00210003,
			onboard_control_sensors_enabled: 0x00210001,
			onboard_control_sensors_health: 0x00010003,
			load: 123,
			voltage_battery: 0xffff,
			current_battery: -1,
			battery_remaining: -1,
			drop_rate_comm: 250,
			errors_comm: 7,
			errors_count1: 1,
			errors_count2: 2,
			errors_count3: 3,
			errors_count4: 4,
			onboard_control_sensors_present_extended: 0x1,
			onboard_control_sensors_enabled_extended: 0x2,
			onboard_control_sensors_health_extended: 0x3
		}
	},
	{
		name: 'Attitude',
		message: { kind: 'Attitude', time_boot_ms: 123456, roll: 0.25, pitch: -0.5, yaw: 1.5, rollspeed: 0.125, pitchspeed: -2, yawspeed: 3 }
	},
	{
		name: 'AttitudeQuaternion',
		message: {
			kind: 'AttitudeQuaternion',
			time_boot_ms: 123456,
			q1: 0.5,
			q2: -0.5,
			q3: 0.5,
			q4: -0.5,
			rollspeed: 0.125,
			pitchspeed: -0.25,
			yawspeed: 4,
			repr_offset_q: [1, 0, -0.75, 0.5]
		}
	},
	{
		name: 'RcChannels',
		message: {
			kind: 'RcChannels',
			time_boot_ms: 123456,
			chancount: 16,
			chan1_raw: 988,
			chan2_raw: 1000,
			chan3_raw: 1100,
			chan4_raw: 1200,
			chan5_raw: 1300,
			chan6_raw: 1400,
			chan7_raw: 1500,
			chan8_raw: 1600,
			chan9_raw: 1700,
			chan10_raw: 1800,
			chan11_raw: 1900,
			chan12_raw: 2000,
			chan13_raw: 2012,
			chan14_raw: 1501,
			chan15_raw: 1499,
			chan16_raw: 1024,
			chan17_raw: 0xffff,
			chan18_raw: 0xffff,
			rssi: 200
		}
	},
	{
		name: 'AttitudeTarget',
		message: {
			kind: 'AttitudeTarget',
			time_boot_ms: 123456,
			type_mask: 128,
			q: [1, -0.25, 0.5, -0.125],
			body_roll_rate: 1.5,
			body_pitch_rate: -2.5,
			body_yaw_rate: 0.75,
			thrust: 0.375
		}
	},
	{
		name: 'HighresImu',
		message: {
			kind: 'HighresImu',
			time_usec: 0x0102030405060708n,
			xacc: 0.5,
			yacc: -1,
			zacc: -9.75,
			xgyro: 0.03125,
			ygyro: -0.0625,
			zgyro: 0.125,
			xmag: 0.25,
			ymag: -0.5,
			zmag: 0.375,
			abs_pressure: 1013.25,
			diff_pressure: 0.5,
			pressure_alt: 120,
			temperature: 31.5,
			fields_updated: 0x103f,
			id: 1
		}
	},
	{ name: 'MessageInterval', message: { kind: 'MessageInterval', message_id: 15206, interval_us: 5051 } },
	{ name: 'MessageIntervalDisabled', message: { kind: 'MessageInterval', message_id: 30, interval_us: -1 } },
	{
		name: 'ActuatorOutputStatus',
		message: {
			kind: 'ActuatorOutputStatus',
			time_usec: 0x1122334455667788n,
			active: 0xf,
			actuator: [48, 2047, 0, 1000, ...Array<number>(28).fill(0)]
		}
	},
	{
		name: 'MarvFcStatus',
		message: {
			kind: 'MarvFcStatus',
			time_usec: 0x0a0b0c0d0e0f1011n,
			rate_executions: 0x123456789n,
			rate_faults: 1,
			rate_timing_faults: 2,
			attitude_faults: 3,
			attitude_timing_faults: 4,
			imu_deadline_misses: 5,
			rc_frames: 6789,
			rc_age_us: 0xffffffff,
			arm_state: MarvArmState.ArmedHold,
			motor_gate: MarvMotorGate.Armed,
			flags: 0xa5,
			rate_fault: 3
		}
	},
	{
		name: 'MarvEstimator',
		message: {
			kind: 'MarvEstimator',
			time_usec: 0x0102030405060708n,
			q_mahony: [1, 0, 0, 0],
			q_eskf: [0.5, 0.5, -0.5, 0.5],
			gyro_bias_cal: [0.001953125, -0.00390625, 0.0078125],
			gyro_bias_mahony: [0, 0, 0],
			gyro_bias_eskf: [-0.25, 0.125, 0.0625],
			flags: 7
		}
	}
];

export function envelopeFor(index: number): Envelope {
	return { systemId: 10 + index, componentId: 190, sequence: index };
}
