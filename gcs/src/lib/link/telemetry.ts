// The vehicle's telemetry streams for the live view (GCS-first unit GX-5): asks for the streams with MAV_CMD_SET_MESSAGE_INTERVAL,
// keeps the latest value of each and when it arrived, and stops the streams again. Everything above this file (the live view's
// state and the page) works with the domain values exported here; no message or enum name leaves this directory (GS-3).
//
// The vehicle grants a stream at the longest of the interval asked, the one of the stream's cap (the tlm_hz_* parameters) and the
// one at which it fits the telemetry byte budget (fw/link/src/service.cpp grant()); a request above a cap is therefore safe, and a
// refused one is answered with a COMMAND_ACK that is not "accepted". A vehicle that restarts, or a link that closes, forgets the
// streams (the vehicle zeroes every interval when its transport is not connected), so the client asks again for each stream that
// has not been heard for REQUEST_RETRY_MS while the live view is open (`maintain`).
import { MarvArmState, MarvMotorGate, MESSAGES, type Message } from './generated/messages';
import { type LinkSession, type MessageOf } from './session';

export type StreamId = 'status' | 'attitude' | 'quaternion' | 'rc' | 'target' | 'imu' | 'actuators' | 'fcStatus' | 'estimator';
export const STREAM_IDS: readonly StreamId[] = ['status', 'attitude', 'quaternion', 'rc', 'target', 'imu', 'actuators', 'fcStatus', 'estimator'];

const KIND_OF: Record<StreamId, Message['kind']> = {
	status: 'SysStatus',
	attitude: 'Attitude',
	quaternion: 'AttitudeQuaternion',
	rc: 'RcChannels',
	target: 'AttitudeTarget',
	imu: 'HighresImu',
	actuators: 'ActuatorOutputStatus',
	fcStatus: 'MarvFcStatus',
	estimator: 'MarvEstimator'
};

/**
 * Harness values of the display, not vehicle numbers: the rate at which the live view asks for each stream (Hz). The vehicle lowers
 * a rate above the stream's cap or the byte budget (see above); a panel is stale after SHELL_TIMING.vehicleStaleMs without data,
 * far longer than the slowest period here.
 */
export const DISPLAY_RATES_HZ: Record<StreamId, number> = {
	status: 2,
	attitude: 20,
	quaternion: 10,
	rc: 10,
	target: 20,
	imu: 20,
	actuators: 20,
	fcStatus: 10,
	estimator: 10
};
/** A stream that has not been heard for this long (ms, a labelled harness value) is asked for again. */
export const REQUEST_RETRY_MS = 1000;
/** The window (ms, a labelled harness value) over which a stream's arrival rate is counted. */
export const RATE_WINDOW_MS = 1000;

/** MAV_CMD_SET_MESSAGE_INTERVAL (common.xml); an interval of -1 stops the stream. */
const CMD_SET_MESSAGE_INTERVAL = 511;
const INTERVAL_STOP = -1;
const US_PER_S = 1_000_000;
/** MAV_RESULT_ACCEPTED, and the names of the other results (common.xml). */
const RESULT_ACCEPTED = 0;
const RESULT_NAMES = ['accepted', 'temporarily rejected', 'denied', 'unsupported', 'failed', 'in progress', 'cancelled'] as const;

// ---- domain values ----------------------------------------------------------------------------------------------------------

export type Vec3 = readonly [number, number, number];
export type Quat = readonly [number, number, number, number];

export interface StreamStamp {
	/** When the latest message arrived (the client's clock, ms); null if none has. */
	at: number | null;
	/** Messages received since the last reset. */
	count: number;
	/** The arrival rate over the last full window (Hz); null until one window has passed. */
	hz: number | null;
}

export interface StatusValues {
	gyroHealthy: boolean;
	accelHealthy: boolean;
	rcHealthy: boolean;
	estimatorHealthy: boolean;
	/** The share of the link's received frames that were refused (percent). */
	commDropPct: number;
	commErrors: number;
}

export interface AttitudeValues {
	roll: number;
	pitch: number;
	yaw: number;
	/** The angular rates (rad/s) the rate loop used, body frame. */
	rate: Vec3;
}

export interface QuaternionValues {
	q: Quat;
}

export interface RcValues {
	/** 0 before any RC frame was received. */
	channelCount: number;
	/** Channels 1..16 in microseconds; null for a channel the vehicle does not report. */
	channelsUs: ReadonlyArray<number | null>;
	/** The receiver's link quality, 0..1; null when the vehicle has no link statistics. */
	linkQuality: number | null;
}

export interface TargetValues {
	/** The rate setpoint (rad/s), body frame. */
	rate: Vec3;
	/** The attitude setpoint; null outside angle mode. */
	attitude: Quat | null;
	/** The collective thrust over the largest the throttle commands, 0..1. */
	thrust: number;
}

export interface ImuValues {
	gyro: Vec3;
	accel: Vec3;
	/** Degrees Celsius; null when the sample carries no temperature. */
	temperatureC: number | null;
	gyroValid: boolean;
	accelValid: boolean;
	/** The sample's stamp on the vehicle's clock (us). */
	sampleUs: number;
}

export interface ActuatorValues {
	/** The DShot value written to motor 1..4 (0 is stop). */
	dshot: readonly number[];
}

export interface FcFlags {
	angleMode: boolean;
	airmode: boolean;
	itermHeld: boolean;
	gyroCalibrated: boolean;
	estimatorValid: boolean;
	upright: boolean;
	imuInvalid: boolean;
	imuSaturated: boolean;
}

/** What an arming state means for the pilot: armed (motors may run), a motor test, held off by a fault, or waiting for the pilot or the boot. */
export type ArmClass = 'armed' | 'test' | 'blocked' | 'waiting';

export interface FcStatusValues {
	armState: number;
	armStateName: string;
	armClass: ArmClass;
	motorGate: number;
	motorGateName: string;
	flags: FcFlags;
	rateFaultActive: boolean;
	rateFaultLatched: boolean;
	rateExecutions: number;
	rateFaults: number;
	rateTimingFaults: number;
	attitudeFaults: number;
	attitudeTimingFaults: number;
	imuDeadlineMisses: number;
	rcFrames: number;
	/** The age of the newest RC frame (us); null before any frame. */
	rcAgeUs: number | null;
}

export interface EstimatorValues {
	mahony: Quat;
	eskf: Quat;
	/** The boot calibration's gyro bias, Mahony's and the ESKF's (rad/s). */
	biasCal: Vec3;
	biasMahony: Vec3;
	biasEskf: Vec3;
	mahonyValid: boolean;
	eskfPresent: boolean;
	eskfValid: boolean;
	eskfInUse: boolean;
}

export interface RequestState {
	/** Interval requests sent since the last reset. */
	sent: number;
	/** Interval requests answered with anything but "accepted". */
	refused: number;
	lastRefusal: string | null;
}

export interface TelemetrySnapshot {
	stamps: Record<StreamId, StreamStamp>;
	status: StatusValues | null;
	attitude: AttitudeValues | null;
	quaternion: QuaternionValues | null;
	rc: RcValues | null;
	target: TargetValues | null;
	imu: ImuValues | null;
	actuators: ActuatorValues | null;
	fcStatus: FcStatusValues | null;
	estimator: EstimatorValues | null;
	/** True between `start` and `stop`: the live view wants the streams. */
	wanted: boolean;
	requests: RequestState;
}

// ---- decoding ---------------------------------------------------------------------------------------------------------------

const idOf = (kind: Message['kind']): number => {
	const info = MESSAGES.find((m) => m.kind === kind);
	if (!info) throw new Error(`no ${kind} in the dialect`);
	return info.id;
};

/** MAV_SYS_STATUS_SENSOR_3D_GYRO, _3D_ACCEL, _RC_RECEIVER and MAV_SYS_STATUS_AHRS (common.xml). */
const SENSOR_GYRO = 1;
const SENSOR_ACCEL = 2;
const SENSOR_RC = 65536;
const SENSOR_AHRS = 2097152;
/** The unit of SYS_STATUS drop_rate_comm: centi-percent (common.xml). */
const CENTI_PERCENT_PER_PERCENT = 100;
/** The receiver link quality scale of RC_CHANNELS rssi: 0..254, 255 for "unknown" (common.xml). */
const RSSI_MAX = 254;
const RSSI_UNKNOWN = 255;
/** RC_CHANNELS reports an unused channel as 65535 (UINT16_MAX, common.xml). */
const CHANNEL_UNUSED = 0xffff;
const RC_CHANNELS = 16;
/** ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE (common.xml). */
const TYPEMASK_ATTITUDE_IGNORE = 128;
/** HIGHRES_IMU_UPDATED_XACC | _YACC | _ZACC, _XGYRO | _YGYRO | _ZGYRO and _TEMPERATURE (common.xml). */
const UPDATED_ACCEL = 1 | 2 | 4;
const UPDATED_GYRO = 8 | 16 | 32;
const UPDATED_TEMPERATURE = 4096;
/** The motors of the quad-X airframe (core section 3): the actuator stream carries more slots than the vehicle has motors. */
const DSHOT_MOTORS = 4;
/** The age MARV_FC_STATUS reports for "no RC frame yet" (UINT32_MAX). */
const AGE_NONE = 0xffffffff;

const ARMED_STATES: readonly number[] = [MarvArmState.Armed, MarvArmState.ArmedHold, MarvArmState.ArmedFailsafeCentred];
const BLOCKED_STATES: readonly number[] = [
	MarvArmState.ImuStale, MarvArmState.ImuInvalid, MarvArmState.Saturated, MarvArmState.NoLink, MarvArmState.Failsafe,
	MarvArmState.EstimatorInvalid, MarvArmState.NotUpright, MarvArmState.ConfigRefused
];
const armClassOf = (state: number): ArmClass =>
	ARMED_STATES.includes(state) ? 'armed' : state === MarvArmState.MotorTest ? 'test' : BLOCKED_STATES.includes(state) ? 'blocked' : 'waiting';

const camelWords = (name: string): string => name.replace(/([a-z0-9])([A-Z])/g, '$1 $2').toLowerCase();
const nameOf = (table: Record<string, number>, value: number): string => {
	const entry = Object.entries(table).find(([, v]) => v === value);
	return entry ? camelWords(entry[0]) : `state ${value}`;
};

const vec3 = (x: number, y: number, z: number): Vec3 => [x, y, z];
const quat = (q: ArrayLike<number>): Quat => [q[0] ?? 0, q[1] ?? 0, q[2] ?? 0, q[3] ?? 0];
const has = (bits: number, flag: number): boolean => (bits & flag) === flag;
const any = (bits: number, flag: number): boolean => (bits & flag) !== 0;

export function decodeStatus(m: MessageOf<'SysStatus'>): StatusValues {
	const health = m.onboard_control_sensors_health;
	return {
		gyroHealthy: any(health, SENSOR_GYRO),
		accelHealthy: any(health, SENSOR_ACCEL),
		rcHealthy: any(health, SENSOR_RC),
		estimatorHealthy: any(health, SENSOR_AHRS),
		commDropPct: m.drop_rate_comm / CENTI_PERCENT_PER_PERCENT,
		commErrors: m.errors_comm
	};
}

export function decodeRc(m: MessageOf<'RcChannels'>): RcValues {
	const raw = [
		m.chan1_raw, m.chan2_raw, m.chan3_raw, m.chan4_raw, m.chan5_raw, m.chan6_raw, m.chan7_raw, m.chan8_raw,
		m.chan9_raw, m.chan10_raw, m.chan11_raw, m.chan12_raw, m.chan13_raw, m.chan14_raw, m.chan15_raw, m.chan16_raw
	];
	return {
		channelCount: m.chancount,
		channelsUs: raw.slice(0, RC_CHANNELS).map((v, i) => (i < m.chancount && v !== CHANNEL_UNUSED ? v : null)),
		linkQuality: m.rssi === RSSI_UNKNOWN ? null : Math.min(m.rssi, RSSI_MAX) / RSSI_MAX
	};
}

export function decodeFcStatus(m: MessageOf<'MarvFcStatus'>): FcStatusValues {
	return {
		armState: m.arm_state,
		armStateName: nameOf(MarvArmState, m.arm_state),
		armClass: armClassOf(m.arm_state),
		motorGate: m.motor_gate,
		motorGateName: nameOf(MarvMotorGate, m.motor_gate),
		flags: {
			angleMode: has(m.flags, 1),
			airmode: has(m.flags, 2),
			itermHeld: has(m.flags, 4),
			gyroCalibrated: has(m.flags, 8),
			estimatorValid: has(m.flags, 16),
			upright: has(m.flags, 32),
			imuInvalid: has(m.flags, 64),
			imuSaturated: has(m.flags, 128)
		},
		rateFaultActive: has(m.rate_fault, 1),
		rateFaultLatched: has(m.rate_fault, 2),
		rateExecutions: Number(m.rate_executions),
		rateFaults: m.rate_faults,
		rateTimingFaults: m.rate_timing_faults,
		attitudeFaults: m.attitude_faults,
		attitudeTimingFaults: m.attitude_timing_faults,
		imuDeadlineMisses: m.imu_deadline_misses,
		rcFrames: m.rc_frames,
		rcAgeUs: m.rc_frames === 0 || m.rc_age_us === AGE_NONE ? null : m.rc_age_us
	};
}

export function decodeEstimator(m: MessageOf<'MarvEstimator'>): EstimatorValues {
	return {
		mahony: quat(m.q_mahony),
		eskf: quat(m.q_eskf),
		biasCal: vec3(m.gyro_bias_cal[0] ?? 0, m.gyro_bias_cal[1] ?? 0, m.gyro_bias_cal[2] ?? 0),
		biasMahony: vec3(m.gyro_bias_mahony[0] ?? 0, m.gyro_bias_mahony[1] ?? 0, m.gyro_bias_mahony[2] ?? 0),
		biasEskf: vec3(m.gyro_bias_eskf[0] ?? 0, m.gyro_bias_eskf[1] ?? 0, m.gyro_bias_eskf[2] ?? 0),
		mahonyValid: has(m.flags, 1),
		eskfPresent: has(m.flags, 2),
		eskfValid: has(m.flags, 4),
		eskfInUse: has(m.flags, 8)
	};
}

// ---- the client -------------------------------------------------------------------------------------------------------------

export interface TelemetryOptions {
	now?: () => number;
	/** The rate asked for each stream (Hz); DISPLAY_RATES_HZ by default. */
	ratesHz?: Partial<Record<StreamId, number>>;
}

interface Counting {
	windowStart: number | null;
	windowCount: number;
}

type Values = Omit<TelemetrySnapshot, 'stamps' | 'wanted' | 'requests'>;

function emptyValues(): Values {
	return { status: null, attitude: null, quaternion: null, rc: null, target: null, imu: null, actuators: null, fcStatus: null, estimator: null };
}

function emptyStamps(): Record<StreamId, StreamStamp> {
	const out = {} as Record<StreamId, StreamStamp>;
	for (const id of STREAM_IDS) out[id] = { at: null, count: 0, hz: null };
	return out;
}

export class TelemetryClient {
	private readonly now: () => number;
	private readonly rates: Record<StreamId, number>;
	private stamps = emptyStamps();
	private readonly counting = {} as Record<StreamId, Counting>;
	private readonly lastAsked = {} as Record<StreamId, number | null>;
	private values: Values = emptyValues();
	private wanted = false;
	private requests: RequestState = { sent: 0, refused: 0, lastRefusal: null };
	private awaiting = 0;
	private readonly unsubscribers: Array<() => void> = [];

	constructor(private readonly session: LinkSession, options: TelemetryOptions = {}) {
		this.now = options.now ?? (() => Date.now());
		this.rates = { ...DISPLAY_RATES_HZ, ...options.ratesHz };
		for (const id of STREAM_IDS) {
			this.counting[id] = { windowStart: null, windowCount: 0 };
			this.lastAsked[id] = null;
		}
		const bus = session.bus;
		this.unsubscribers.push(
			bus.on('SysStatus', (m) => this.take('status', { status: decodeStatus(m) })),
			bus.on('Attitude', (m) =>
				this.take('attitude', { attitude: { roll: m.roll, pitch: m.pitch, yaw: m.yaw, rate: vec3(m.rollspeed, m.pitchspeed, m.yawspeed) } })
			),
			bus.on('AttitudeQuaternion', (m) => this.take('quaternion', { quaternion: { q: [m.q1, m.q2, m.q3, m.q4] } })),
			bus.on('RcChannels', (m) => this.take('rc', { rc: decodeRc(m) })),
			bus.on('AttitudeTarget', (m) =>
				this.take('target', {
					target: {
						rate: vec3(m.body_roll_rate, m.body_pitch_rate, m.body_yaw_rate),
						attitude: any(m.type_mask, TYPEMASK_ATTITUDE_IGNORE) ? null : quat(m.q),
						thrust: m.thrust
					}
				})
			),
			bus.on('HighresImu', (m) =>
				this.take('imu', {
					imu: {
						gyro: vec3(m.xgyro, m.ygyro, m.zgyro),
						accel: vec3(m.xacc, m.yacc, m.zacc),
						temperatureC: any(m.fields_updated, UPDATED_TEMPERATURE) ? m.temperature : null,
						gyroValid: has(m.fields_updated, UPDATED_GYRO),
						accelValid: has(m.fields_updated, UPDATED_ACCEL),
						sampleUs: Number(m.time_usec)
					}
				})
			),
			bus.on('ActuatorOutputStatus', (m) => this.take('actuators', { actuators: { dshot: m.actuator.slice(0, DSHOT_MOTORS) } })),
			bus.on('MarvFcStatus', (m) => this.take('fcStatus', { fcStatus: decodeFcStatus(m) })),
			bus.on('MarvEstimator', (m) => this.take('estimator', { estimator: decodeEstimator(m) })),
			bus.on('CommandAck', (m) => {
				if (m.command !== CMD_SET_MESSAGE_INTERVAL || this.awaiting === 0) return;
				this.awaiting--;
				if (m.result === RESULT_ACCEPTED) return;
				this.requests = { ...this.requests, refused: this.requests.refused + 1, lastRefusal: RESULT_NAMES[m.result] ?? `result ${m.result}` };
			})
		);
	}

	dispose(): void {
		for (const u of this.unsubscribers.splice(0)) u();
	}

	/** The latest value of every stream, with when each arrived. */
	snapshot(): TelemetrySnapshot {
		const stamps = {} as Record<StreamId, StreamStamp>;
		for (const id of STREAM_IDS) stamps[id] = { ...this.stamps[id] };
		return { stamps, ...this.values, wanted: this.wanted, requests: { ...this.requests } };
	}

	/** Forget everything learnt from the vehicle (the connection changed); the streams stay wanted and are asked for again. */
	reset(): void {
		this.stamps = emptyStamps();
		for (const id of STREAM_IDS) {
			this.counting[id] = { windowStart: null, windowCount: 0 };
			this.lastAsked[id] = null;
		}
		this.values = emptyValues();
		this.requests = { sent: 0, refused: 0, lastRefusal: null };
		this.awaiting = 0;
	}

	/** The live view opens: asks for every stream (if the vehicle is known; `maintain` asks again once it is). */
	start(): void {
		this.wanted = true;
		this.maintain();
	}

	/** The live view closes: stops every stream the view asked for. */
	stop(): void {
		if (!this.wanted) return;
		this.wanted = false;
		const target = this.session.target();
		if (!target) return;
		for (const id of STREAM_IDS) this.send(id, INTERVAL_STOP, target);
		for (const id of STREAM_IDS) this.lastAsked[id] = null;
	}

	/**
	 * Asks for each wanted stream that was not asked for, or was asked for and not heard, within REQUEST_RETRY_MS. Called by the live
	 * view's tick; sends nothing while the vehicle's address is unknown.
	 */
	maintain(): void {
		if (!this.wanted) return;
		const target = this.session.target();
		if (!target) return;
		const now = this.now();
		for (const id of STREAM_IDS) {
			const asked = this.lastAsked[id];
			const heard = this.stamps[id].at;
			const quiet = heard === null || now - heard > REQUEST_RETRY_MS;
			if (asked !== null && (now - asked <= REQUEST_RETRY_MS || !quiet)) continue;
			this.send(id, Math.round(US_PER_S / this.rates[id]), target);
			this.lastAsked[id] = now;
		}
	}

	private send(id: StreamId, intervalUs: number, target: { systemId: number; componentId: number }): void {
		this.awaiting++;
		this.requests = { ...this.requests, sent: this.requests.sent + 1 };
		this.session.send({
			kind: 'CommandLong', target_system: target.systemId, target_component: target.componentId, command: CMD_SET_MESSAGE_INTERVAL,
			confirmation: 0, param1: idOf(KIND_OF[id]), param2: intervalUs, param3: 0, param4: 0, param5: 0, param6: 0, param7: 0
		});
	}

	private take(id: StreamId, value: Partial<Values>): void {
		const at = this.now();
		this.values = { ...this.values, ...value };
		const stamp = this.stamps[id];
		stamp.at = at;
		stamp.count++;
		const c = this.counting[id];
		if (c.windowStart === null) {
			// The message that opens a window is not counted in it: n messages after it span n periods.
			c.windowStart = at;
			c.windowCount = 0;
			return;
		}
		c.windowCount++;
		const span = at - c.windowStart;
		if (span >= RATE_WINDOW_MS) {
			stamp.hz = (c.windowCount * RATE_WINDOW_MS) / span;
			c.windowStart = at;
			c.windowCount = 0;
		}
	}
}
