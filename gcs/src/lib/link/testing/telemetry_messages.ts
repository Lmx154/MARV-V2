// One message of each telemetry stream, with values that are different in every field (the stream tests of the link and the live view).
import { MarvArmState, MarvMotorGate, type Message } from '../generated/messages';

type Of<K extends Message['kind']> = Extract<Message, { kind: K }>;

export const sysStatus: Of<'SysStatus'> = {
	kind: 'SysStatus',
	onboard_control_sensors_present: 0x00210003,
	onboard_control_sensors_enabled: 0x00210003,
	onboard_control_sensors_health: 0x00200003,
	load: 0,
	voltage_battery: 0xffff,
	current_battery: -1,
	battery_remaining: -1,
	drop_rate_comm: 250,
	errors_comm: 7,
	errors_count1: 0,
	errors_count2: 0,
	errors_count3: 0,
	errors_count4: 0,
	onboard_control_sensors_present_extended: 0,
	onboard_control_sensors_enabled_extended: 0,
	onboard_control_sensors_health_extended: 0
};

export const attitude: Of<'Attitude'> = { kind: 'Attitude', time_boot_ms: 1000, roll: 0.25, pitch: -0.5, yaw: 1.5, rollspeed: 0.125, pitchspeed: -2, yawspeed: 3 };

export const attitudeQuaternion: Of<'AttitudeQuaternion'> = {
	kind: 'AttitudeQuaternion', time_boot_ms: 1000, q1: 0.5, q2: -0.5, q3: 0.5, q4: -0.5, rollspeed: 0, pitchspeed: 0, yawspeed: 0, repr_offset_q: [0, 0, 0, 0]
};

export const rcChannels: Of<'RcChannels'> = {
	kind: 'RcChannels', time_boot_ms: 1000, chancount: 16,
	chan1_raw: 1500, chan2_raw: 1756, chan3_raw: 988, chan4_raw: 1244, chan5_raw: 2012, chan6_raw: 1000, chan7_raw: 1100, chan8_raw: 1200,
	chan9_raw: 1300, chan10_raw: 1400, chan11_raw: 1600, chan12_raw: 1700, chan13_raw: 1800, chan14_raw: 1900, chan15_raw: 2000, chan16_raw: 1024,
	chan17_raw: 0xffff, chan18_raw: 0xffff, rssi: 127
};

export const attitudeTarget: Of<'AttitudeTarget'> = {
	kind: 'AttitudeTarget', time_boot_ms: 1000, type_mask: 128, q: [1, 0, 0, 0], body_roll_rate: 1, body_pitch_rate: -0.5, body_yaw_rate: 0.25, thrust: 0.375
};

export const highresImu: Of<'HighresImu'> = {
	kind: 'HighresImu', time_usec: 5_000_000n, xacc: 0.5, yacc: -1, zacc: -9.75, xgyro: 0.5, ygyro: -0.25, zgyro: 0.125, xmag: 0, ymag: 0, zmag: 0,
	abs_pressure: 0, diff_pressure: 0, pressure_alt: 0, temperature: 31.5, fields_updated: 0x103f, id: 0
};

export const actuatorOutputStatus: Of<'ActuatorOutputStatus'> = {
	kind: 'ActuatorOutputStatus', time_usec: 5_000_000n, active: 0xf, actuator: [48, 1000, 2047, 0, ...Array<number>(28).fill(0)]
};

export const fcStatus: Of<'MarvFcStatus'> = {
	kind: 'MarvFcStatus', time_usec: 5_000_000n, rate_executions: 12345n, rate_faults: 1, rate_timing_faults: 0, attitude_faults: 2, attitude_timing_faults: 0,
	imu_deadline_misses: 3, rc_frames: 6789, rc_age_us: 20000, arm_state: MarvArmState.ArmedHold, motor_gate: MarvMotorGate.Armed,
	flags: 1 | 8 | 16 | 32, rate_fault: 2
};

export const estimator: Of<'MarvEstimator'> = {
	kind: 'MarvEstimator', time_usec: 5_000_000n, q_mahony: [Math.SQRT1_2, Math.SQRT1_2, 0, 0], q_eskf: [Math.SQRT1_2, 0, 0, Math.SQRT1_2],
	gyro_bias_cal: [0.001953125, -0.00390625, 0.0078125], gyro_bias_mahony: [0, 0, 0], gyro_bias_eskf: [-0.25, 0.125, 0.0625], flags: 1 | 2 | 4
};

export const allStreamMessages: readonly Message[] = [
	sysStatus, attitude, attitudeQuaternion, rcChannels, attitudeTarget, highresImu, actuatorOutputStatus, fcStatus, estimator
];
