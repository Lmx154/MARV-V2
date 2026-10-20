// The decoder of the simulator's sim channel record, version 1 (ground-segment unit GX-8, decision 0056). The one place in the app
// that knows the record's layout; the layout and the argument that publishing it does not perturb the run are in
// sim/gz/plugin/src/sim_telemetry.hpp, and tools/sim/sim_channel.py is the Python reference decoder the two are checked against.
//
// Invariant S-1: this module, with the rest of lib/sim, is the only reader of simulator truth. It imports nothing of the vehicle
// client, and nothing outside lib/sim imports it (tools/ci/check_s1.py).

export type Vec3 = readonly [number, number, number];
export type Vec4 = readonly [number, number, number, number];

/** The record's magic, "MSIM", and the version this decoder reads. */
export const SIM_MAGIC = 'MSIM';
export const SIM_VERSION = 1;
/** The bytes of a version 1 record (sim/gz/plugin/src/sim_telemetry.hpp kSimRecordBytes). */
export const SIM_RECORD_BYTES = 384;

export interface SimFlags {
	/** The injected IMU sample is the plant model's. */
	imuModel: boolean;
	launchPad: boolean;
	ground: boolean;
	hil: boolean;
	/** The pad held the body at the last tick. */
	padHeld: boolean;
	/** The pad has released (the vehicle has lifted off) at least once. */
	padReleased: boolean;
}

/** The board's state as the HIL plugin source reports it; present only in a HIL run. */
export interface SimHil {
	/** The board's arming state code and its name; null before the first status record. */
	armingState: number | null;
	armingName: string | null;
	statusRecords: number;
	receiverFrames: number;
	frameCounter: number;
	mode: 'unknown' | 'acro' | 'angle';
	deadlineMisses: number;
	linkFaults: number;
	reportsUsed: number;
	lagEvents: number;
	/** Roll, pitch, yaw, throttle of the latest receiver frame. */
	sticks: Vec4;
	channels: readonly number[];
	linkQuality: number;
	marginMinUs: number;
	marginMaxUs: number;
	lagUs: number;
}

export interface SimRecord {
	version: number;
	seq: number;
	flags: SimFlags;
	simTimeNs: number;
	tick: number;
	/** Simulated seconds per wall second since the previous record; 0 for the first. */
	rtf: number;
	posNed: Vec3;
	velNed: Vec3;
	/** Hamilton, body FRD to NED, [w, x, y, z]. */
	q: Vec4;
	omegaFrd: Vec3;
	/** Rotor thrust per mass along body -z; drag and the pad's contact force are not in it. */
	specificForceFrd: Vec3;
	rotorSpeed: Vec4;
	imu: { gyro: Vec3; accel: Vec3; flags: number };
	dshotCommanded: Vec4;
	dshotApplied: Vec4;
	touchdowns: number;
	lastTouchdownDownMs: number;
	hil: SimHil | null;
}

export class SimRecordError extends Error {
	constructor(message: string) {
		super(message);
		this.name = 'SimRecordError';
	}
}

/** The board's ArmState names in code order (fw/arming/include/marv/arming/arming.hpp), as the plugin prints them. */
export const ARM_STATE_NAMES: readonly string[] = [
	'boot',
	'switch_off',
	'switch_not_cycled',
	'throttle_up',
	'imu_stale',
	'imu_invalid',
	'saturated',
	'no_link',
	'failsafe',
	'motor_test',
	'armed',
	'armed_hold',
	'armed_failsafe_centred',
	'estimator_invalid',
	'calibrating',
	'not_upright',
	'config_refused',
	'maintenance'
];

/** The armed states (codes 10 to 12 of ARM_STATE_NAMES). */
export function isArmedName(name: string | null): boolean {
	return name !== null && name.startsWith('armed');
}

const MODES = ['unknown', 'acro', 'angle'] as const;
const FLAG_BITS = { imuModel: 0, launchPad: 1, ground: 2, hil: 3, padHeld: 4, padReleased: 5 } as const;
const TWO_POW_32 = 4294967296;

/** Decodes one datagram. Throws SimRecordError for a datagram that is not a record this decoder knows: wrong magic, a version other than 1,
 * or a length field that is not the datagram's or is below version 1's. */
export function decodeSimRecord(data: Uint8Array): SimRecord {
	if (data.length < 8) throw new SimRecordError(`${data.length} bytes: shorter than the record header`);
	const v = new DataView(data.buffer, data.byteOffset, data.byteLength);
	const magic = String.fromCharCode(data[0]!, data[1]!, data[2]!, data[3]!);
	if (magic !== SIM_MAGIC) throw new SimRecordError(`magic ${JSON.stringify(magic)} is not ${SIM_MAGIC}`);
	const version = v.getUint16(4, true);
	if (version !== SIM_VERSION) throw new SimRecordError(`record version ${version} is not ${SIM_VERSION}`);
	const length = v.getUint16(6, true);
	if (length !== data.length || length < SIM_RECORD_BYTES) {
		throw new SimRecordError(`record length field ${length}, datagram ${data.length} bytes, version ${SIM_VERSION} needs ${SIM_RECORD_BYTES}`);
	}
	const u32 = (o: number): number => v.getUint32(o, true);
	const u64 = (o: number): number => u32(o + 4) * TWO_POW_32 + u32(o);
	const f64 = (o: number): number => v.getFloat64(o, true);
	const f32 = (o: number): number => v.getFloat32(o, true);
	const vec3 = (o: number): Vec3 => [f64(o), f64(o + 8), f64(o + 16)];
	const vec4 = (o: number): Vec4 => [f64(o), f64(o + 8), f64(o + 16), f64(o + 24)];
	const flagWord = u32(12);
	const flags = Object.fromEntries(Object.entries(FLAG_BITS).map(([name, bit]) => [name, ((flagWord >>> bit) & 1) === 1])) as unknown as SimFlags;
	const f32x3 = (o: number): Vec3 => [f32(o), f32(o + 4), f32(o + 8)];
	const u16x4 = (o: number): Vec4 => [v.getUint16(o, true), v.getUint16(o + 2, true), v.getUint16(o + 4, true), v.getUint16(o + 6, true)];
	let hil: SimHil | null = null;
	if (flags.hil) {
		const statusRecords = u32(256);
		const state = u32(252);
		const mode = u32(268);
		hil = {
			armingState: statusRecords > 0 ? state : null,
			armingName: statusRecords > 0 ? (ARM_STATE_NAMES[state] ?? 'unknown') : null,
			statusRecords,
			receiverFrames: u32(260),
			frameCounter: u32(264),
			mode: MODES[mode] ?? 'unknown',
			deadlineMisses: u32(272),
			linkFaults: u32(276),
			reportsUsed: u32(280),
			lagEvents: u32(284),
			sticks: [f32(288), f32(292), f32(296), f32(300)],
			channels: Array.from({ length: 16 }, (_, i) => f32(304 + 4 * i)),
			linkQuality: f32(368),
			marginMinUs: f32(372),
			marginMaxUs: f32(376),
			lagUs: f32(380)
		};
	}
	return {
		version,
		seq: u32(8),
		flags,
		simTimeNs: u64(16),
		tick: u64(24),
		rtf: f64(32),
		posNed: vec3(40),
		velNed: vec3(64),
		q: vec4(88),
		omegaFrd: vec3(120),
		specificForceFrd: vec3(144),
		rotorSpeed: vec4(168),
		imu: { gyro: f32x3(200), accel: f32x3(212), flags: u32(224) },
		dshotCommanded: u16x4(228),
		dshotApplied: u16x4(236),
		touchdowns: u32(244),
		lastTouchdownDownMs: f32(248),
		hil
	};
}
