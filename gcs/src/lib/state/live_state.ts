// The live view of the vehicle (GCS-first unit GX-5): the panels of the /vehicle page and the controller that keeps them true.
// computeLive() is pure: the same snapshot and the same `now` give the same panels, so every value and every stale transition is
// tested without a browser. The panels read only the vehicle client's telemetry (S-1: the vehicle panels never read the
// simulator's channel, which has its own, labelled panels and is absent without a simulator).
//
// Staleness is per panel, with the shell's vehicle timeout (SHELL_TIMING.vehicleStaleMs): a panel is stale when any stream it
// shows was not heard within that time, or never. A stale panel keeps its last values on screen, marked stale, as the status bar
// does; it never shows a value as current that is not.
import { readable, writable, type Readable } from 'svelte/store';
import type { VehicleParam } from '$link/vehicle';
import type { Quat, StreamId, TelemetryClient, TelemetrySnapshot, Vec3 } from '$link/telemetry';
import { SHELL_TIMING, type ShellTiming, type Tone } from './shell';

export type PanelId = 'attitude' | 'rates' | 'rc' | 'motors' | 'arming' | 'estimators' | 'faults' | 'imu';

/** How often the panels are recomputed from the telemetry (ms, a labelled harness value: the display's frame period, not a vehicle number). */
export const LIVE_TICK_MS = 100;

/** A degree in radians' terms: 180 / pi (a definition). */
const DEG_PER_RAD = 180 / Math.PI;
const MS_PER_S = 1000;
const US_PER_MS = 1000;
const PERCENT = 100;
/** The receiver's channel contract: a channel in microseconds is x = (us - 1500) / 512 (fw/prim constants kCrsfUsCentre, kRcUsHalfSpan). */
const RC_US_CENTRE = 1500;
const RC_US_HALF_SPAN = 512;
/** The largest DShot throttle value (11 bits; fw/prim constants kDshotThrottleMax, citing the Betaflight DShot notes). */
const DSHOT_THROTTLE_MAX = 2047;
const DASH = '—';

/** The decimals each kind of value is shown with (display precision, not a vehicle number). */
const DECIMALS = { angle: 1, rate: 1, bias: 3, accel: 2, temperature: 1, quaternion: 4, normalised: 2, percent: 0, rate_hz: 1, age: 0 } as const;

export interface Row {
	/** Unique within the panel. */
	key: string;
	/** The heading the row is listed under; null for none. */
	group: string | null;
	name: string;
	value: string;
	unit: string;
	tone: Tone;
	/** A bar's position, 0..1; null for a row without a bar. */
	bar: number | null;
	/** The bar fills from its middle (a signed quantity) rather than from its start. */
	centred: boolean;
}

export interface StreamView {
	id: StreamId;
	/** Never heard, or not heard within the staleness timeout. */
	stale: boolean;
	ageMs: number | null;
	hz: number | null;
}

export interface PanelView {
	id: PanelId;
	title: string;
	streams: StreamView[];
	stale: boolean;
	/** Every stream of the panel has been heard at least once. */
	heard: boolean;
	rows: Row[];
	/** Things the panel cannot show, and why. */
	notes: string[];
}

export interface LiveView {
	panels: PanelView[];
	/** The live view has asked for its streams (it is open). */
	streaming: boolean;
	/** The stick channels are known (the parameters were read), so the sticks are listed by their channels. */
	channelMapKnown: boolean;
	requests: TelemetrySnapshot['requests'];
}

/** The 1-based receiver channel of each stick (the parameters rc_channel_roll, _pitch, _throttle, _yaw of the running config). */
export interface ChannelMap {
	roll: number;
	pitch: number;
	throttle: number;
	yaw: number;
}

export interface LiveContext {
	/** Null until the parameters were read from the vehicle. */
	channelMap: ChannelMap | null;
}

const MAP_PARAMS: Record<keyof ChannelMap, string> = { roll: 'rc_channel_roll', pitch: 'rc_channel_pitch', throttle: 'rc_channel_throttle', yaw: 'rc_channel_yaw' };

/** The stick channels from the parameters read from the vehicle; null if any of the four is missing. */
export function channelMapOf(params: readonly VehicleParam[]): ChannelMap | null {
	const out: Partial<ChannelMap> = {};
	for (const key of Object.keys(MAP_PARAMS) as Array<keyof ChannelMap>) {
		const p = params.find((x) => x.linkId === MAP_PARAMS[key]);
		if (!p || p.activeBits === null || p.type !== 'i32') return null;
		out[key] = p.activeBits | 0;
	}
	return out as ChannelMap;
}

// ---- values -----------------------------------------------------------------------------------------------------------------

function fixed(v: number | null | undefined, decimals: number): string {
	if (v === null || v === undefined || !Number.isFinite(v)) return DASH;
	const text = v.toFixed(decimals);
	// A value that rounds to zero is shown as zero, not as "-0.0".
	return /^-0(\.0*)?$/.test(text) ? text.slice(1) : text;
}

const deg = (rad: number | null | undefined): string => fixed(rad === null || rad === undefined ? null : rad * DEG_PER_RAD, DECIMALS.angle);
const yesNo = (v: boolean | null | undefined): string => (v === null || v === undefined ? DASH : v ? 'yes' : 'no');
const clamp = (v: number, lo: number, hi: number): number => Math.min(hi, Math.max(lo, v));

/** Roll, pitch, yaw (rad) of a body-to-NED quaternion [w, x, y, z]: the Z-Y-X sequence the vehicle's attitude message uses. */
export function eulerOf(q: Quat): Vec3 {
	const [w, x, y, z] = q;
	return [
		Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
		Math.asin(clamp(2 * (w * y - z * x), -1, 1)),
		Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
	];
}

class Rows {
	readonly rows: Row[] = [];
	constructor(private readonly panel: string) {}
	add(group: string | null, key: string, name: string, value: string, unit = '', extra: { tone?: Tone; bar?: number | null; centred?: boolean } = {}): void {
		this.rows.push({ key: `${this.panel}.${key}`, group, name, value, unit, tone: extra.tone ?? 'neutral', bar: extra.bar ?? null, centred: extra.centred ?? false });
	}
	flag(group: string | null, key: string, name: string, on: boolean | null | undefined, badWhen: boolean | null = null): void {
		const tone: Tone = on === null || on === undefined || badWhen === null ? 'neutral' : on === badWhen ? 'bad' : 'ok';
		this.add(group, key, name, yesNo(on), '', { tone });
	}
	angles(group: string, key: string, prefix: string, q: Vec3 | null): void {
		for (const [i, axis] of ['roll', 'pitch', 'yaw'].entries()) this.add(group, `${key}.${axis}`, `${prefix}${axis}`, deg(q?.[i] ?? null), '°');
	}
	vec(group: string, key: string, prefix: string, v: Vec3 | null, scale: number, decimals: number, unit: string): void {
		for (const [i, axis] of ['x', 'y', 'z'].entries()) {
			const c = v?.[i];
			this.add(group, `${key}.${axis}`, `${prefix}${axis}`, fixed(c === undefined ? null : c * scale, decimals), unit);
		}
	}
}

// ---- the panels -------------------------------------------------------------------------------------------------------------

function attitudePanel(s: TelemetrySnapshot, r: Rows, notes: string[]): void {
	const a = s.attitude;
	r.angles('In use', 'use', '', a ? [a.roll, a.pitch, a.yaw] : null);
	const q = s.quaternion?.q;
	r.add('In use', 'use.q', 'quaternion (w x y z)', q ? q.map((c) => fixed(c, DECIMALS.quaternion)).join(' ') : DASH);
	const e = s.estimator;
	r.add('In use', 'use.source', 'estimate in use', e ? (e.eskfInUse ? 'ESKF' : 'Mahony') : DASH);
	r.angles('Mahony', 'mahony', '', e ? eulerOf(e.mahony) : null);
	r.flag('Mahony', 'mahony.valid', 'valid', e?.mahonyValid, false);
	if (e && !e.eskfPresent) {
		r.add('ESKF', 'eskf', 'ESKF', 'not present in this image');
		notes.push('This image does not step the ESKF: only the Mahony estimate exists.');
	} else {
		r.angles('ESKF', 'eskf', '', e ? eulerOf(e.eskf) : null);
		r.flag('ESKF', 'eskf.valid', 'valid', e?.eskfValid, false);
	}
}

function ratesPanel(s: TelemetrySnapshot, r: Rows): void {
	r.vec('Gyro rate', 'gyro', 'rate ', s.attitude?.rate ?? null, DEG_PER_RAD, DECIMALS.rate, '°/s');
	r.vec('Rate setpoint', 'setpoint', 'setpoint ', s.target?.rate ?? null, DEG_PER_RAD, DECIMALS.rate, '°/s');
	const t = s.target;
	if (t?.attitude) r.angles('Attitude setpoint', 'attitude', 'setpoint ', eulerOf(t.attitude));
	else r.add('Attitude setpoint', 'attitude', 'attitude setpoint', t ? 'none (not in angle mode)' : DASH);
	r.add(null, 'thrust', 'thrust', fixed(t ? t.thrust * PERCENT : null, DECIMALS.percent), '% of the largest', { bar: t ? clamp(t.thrust, 0, 1) : null });
}

function rcPanel(s: TelemetrySnapshot, ctx: LiveContext, r: Rows, notes: string[]): void {
	const rc = s.rc;
	const quality = rc?.linkQuality ?? null;
	r.add('Link', 'quality', 'link quality', fixed(quality === null ? null : quality * PERCENT, DECIMALS.percent), '%', { bar: quality, tone: quality === null ? 'neutral' : 'ok' });
	r.add('Link', 'channels', 'channels received', rc ? String(rc.channelCount) : DASH);
	if (rc && rc.channelCount === 0) notes.push('No RC frame has been received: the vehicle reports no channels.');
	if (rc && quality === null) notes.push('The receiver has sent no link statistics: the link quality is unknown.');
	const x = (us: number | null | undefined): number | null => (us === null || us === undefined ? null : clamp((us - RC_US_CENTRE) / RC_US_HALF_SPAN, -1, 1));
	const bar = (v: number | null): number | null => (v === null ? null : (v + 1) / 2);
	if (ctx.channelMap) {
		for (const stick of ['roll', 'pitch', 'throttle', 'yaw'] as const) {
			const channel = ctx.channelMap[stick];
			const v = x(rc?.channelsUs[channel - 1]);
			r.add('Sticks', `stick.${stick}`, `${stick} (channel ${channel})`, fixed(v, DECIMALS.normalised), '', { bar: bar(v), centred: stick !== 'throttle' });
		}
	} else {
		notes.push('The stick channels are not known: read the parameters from the vehicle to show the sticks. The channels below are listed by number.');
	}
	for (let i = 0; i < (rc?.channelsUs.length ?? 16); i++) {
		const us = rc?.channelsUs[i] ?? null;
		r.add('Channels', `ch.${i + 1}`, `channel ${i + 1}`, us === null ? DASH : String(us), 'µs', { bar: bar(x(us)), centred: true });
	}
}

function motorsPanel(s: TelemetrySnapshot, r: Rows, notes: string[]): void {
	const m = s.actuators;
	for (let i = 0; i < (m?.dshot.length ?? 4); i++) {
		const v = m?.dshot[i];
		r.add('DShot', `m.${i + 1}`, `motor ${i + 1}`, v === undefined ? DASH : String(Math.round(v)), 'DShot', { bar: v === undefined ? null : clamp(v / DSHOT_THROTTLE_MAX, 0, 1) });
	}
	r.add('Rotor speed', 'rotor', 'rotor speed', 'not on the link');
	notes.push('Rotor speed: the vehicle measures it (bidirectional DShot) but no stream carries it yet, so this view cannot show it.');
}

function armingPanel(s: TelemetrySnapshot, r: Rows): void {
	const f = s.fcStatus;
	const stateTone: Tone = !f ? 'neutral' : f.armClass === 'armed' ? 'bad' : f.armClass === 'blocked' || f.armClass === 'test' ? 'warn' : 'ok';
	r.add(null, 'state', 'arm state', f ? f.armStateName : DASH, '', { tone: stateTone });
	r.add(null, 'gate', 'motor gate', f ? f.motorGateName : DASH, '', { tone: !f ? 'neutral' : f.motorGate === 0 ? 'ok' : 'bad' });
	r.add(null, 'rcframes', 'RC frames', f ? String(f.rcFrames) : DASH);
	r.add(null, 'rcage', 'RC frame age', f ? (f.rcAgeUs === null ? 'no frame' : fixed(f.rcAgeUs / US_PER_MS, DECIMALS.age)) : DASH, f && f.rcAgeUs !== null ? 'ms' : '');
}

function estimatorsPanel(s: TelemetrySnapshot, r: Rows): void {
	const e = s.estimator;
	r.flag('Status', 'mahony.valid', 'Mahony valid', e?.mahonyValid, false);
	r.flag('Status', 'eskf.present', 'ESKF present', e?.eskfPresent);
	r.flag('Status', 'eskf.valid', 'ESKF valid', e?.eskfPresent ? e.eskfValid : null, false);
	r.add('Status', 'inuse', 'in use', e ? (e.eskfInUse ? 'ESKF' : 'Mahony') : DASH);
	const bias = (key: string, name: string, v: Vec3 | null): void =>
		r.add('Gyro bias', key, name, v ? v.map((c) => fixed(c * DEG_PER_RAD, DECIMALS.bias)).join(' ') : DASH, '°/s (x y z)');
	bias('bias.cal', 'boot calibration', e?.biasCal ?? null);
	bias('bias.mahony', 'Mahony', e?.biasMahony ?? null);
	bias('bias.eskf', 'ESKF', e?.eskfPresent ? e.biasEskf : null);
}

function faultsPanel(s: TelemetrySnapshot, r: Rows): void {
	const f = s.fcStatus;
	const fl = f?.flags;
	r.flag('Flags', 'angle', 'angle mode', fl?.angleMode);
	r.flag('Flags', 'airmode', 'airmode', fl?.airmode);
	r.flag('Flags', 'ihold', 'I-term held', fl?.itermHeld);
	r.flag('Flags', 'gyrocal', 'gyro calibrated', fl?.gyroCalibrated, false);
	r.flag('Flags', 'estvalid', 'estimator valid', fl?.estimatorValid, false);
	r.flag('Flags', 'upright', 'upright', fl?.upright, false);
	r.flag('Flags', 'imuinvalid', 'IMU invalid', fl?.imuInvalid, true);
	r.flag('Flags', 'imusat', 'IMU saturated', fl?.imuSaturated, true);
	r.flag('Rate loop', 'ratefault', 'rate fault active', f?.rateFaultActive, true);
	r.flag('Rate loop', 'ratelatched', 'rate fault latched', f?.rateFaultLatched, true);
	const count = (key: string, name: string, v: number | undefined, bad: boolean): void =>
		r.add('Counters', key, name, v === undefined ? DASH : String(v), '', { tone: v === undefined || !bad ? 'neutral' : v > 0 ? 'bad' : 'ok' });
	count('ratefaults', 'rate faults', f?.rateFaults, true);
	count('ratetiming', 'rate timing faults', f?.rateTimingFaults, true);
	count('attfaults', 'attitude faults', f?.attitudeFaults, true);
	count('atttiming', 'attitude timing faults', f?.attitudeTimingFaults, true);
	count('deadline', 'IMU deadline misses', f?.imuDeadlineMisses, true);
	count('executions', 'rate executions', f?.rateExecutions, false);
	const st = s.status;
	r.flag('Sensors', 'gyro', 'gyro healthy', st?.gyroHealthy, false);
	r.flag('Sensors', 'accel', 'accelerometer healthy', st?.accelHealthy, false);
	r.flag('Sensors', 'rc', 'receiver healthy', st?.rcHealthy, false);
	r.flag('Sensors', 'ahrs', 'attitude healthy', st?.estimatorHealthy, false);
	r.add('Link', 'drop', 'frames refused', fixed(st?.commDropPct, DECIMALS.rate), '%', { tone: st && st.commDropPct > 0 ? 'warn' : 'neutral' });
	r.add('Link', 'errors', 'receive errors', st ? String(st.commErrors) : DASH);
}

function imuPanel(s: TelemetrySnapshot, r: Rows): void {
	const i = s.imu;
	r.vec('Gyro', 'gyro', 'gyro ', i?.gyro ?? null, DEG_PER_RAD, DECIMALS.rate, '°/s');
	r.vec('Accelerometer', 'accel', 'accel ', i?.accel ?? null, 1, DECIMALS.accel, 'm/s²');
	r.add('Temperature', 'temp', 'temperature', i ? fixed(i.temperatureC, DECIMALS.temperature) : DASH, '°C');
	r.flag('Validity', 'gyrovalid', 'gyro valid', i?.gyroValid, false);
	r.flag('Validity', 'accelvalid', 'accelerometer valid', i?.accelValid, false);
}

interface PanelSpec {
	id: PanelId;
	title: string;
	streams: StreamId[];
}

/** The panels and the streams each shows; a panel is stale when any of them is. */
export const PANELS: readonly PanelSpec[] = [
	{ id: 'attitude', title: 'Attitude', streams: ['attitude', 'quaternion', 'estimator'] },
	{ id: 'rates', title: 'Rates and setpoint', streams: ['attitude', 'target'] },
	{ id: 'rc', title: 'Radio', streams: ['rc'] },
	{ id: 'motors', title: 'Motors', streams: ['actuators'] },
	{ id: 'arming', title: 'Arming', streams: ['fcStatus'] },
	{ id: 'estimators', title: 'Estimators', streams: ['estimator'] },
	{ id: 'faults', title: 'Flags and faults', streams: ['fcStatus', 'status'] },
	{ id: 'imu', title: 'IMU', streams: ['imu'] }
];

export function computeLive(snapshot: TelemetrySnapshot, now: number, context: LiveContext = { channelMap: null }, timing: ShellTiming = SHELL_TIMING): LiveView {
	const panels = PANELS.map((spec): PanelView => {
		const r = new Rows(spec.id);
		const notes: string[] = [];
		switch (spec.id) {
			case 'attitude': attitudePanel(snapshot, r, notes); break;
			case 'rates': ratesPanel(snapshot, r); break;
			case 'rc': rcPanel(snapshot, context, r, notes); break;
			case 'motors': motorsPanel(snapshot, r, notes); break;
			case 'arming': armingPanel(snapshot, r); break;
			case 'estimators': estimatorsPanel(snapshot, r); break;
			case 'faults': faultsPanel(snapshot, r); break;
			case 'imu': imuPanel(snapshot, r); break;
		}
		const streams = spec.streams.map((id): StreamView => {
			const stamp = snapshot.stamps[id];
			const ageMs = stamp.at === null ? null : Math.max(0, now - stamp.at);
			const stale = ageMs === null || ageMs > timing.vehicleStaleMs;
			return { id, stale, ageMs, hz: stale ? null : stamp.hz };
		});
		return { id: spec.id, title: spec.title, streams, stale: streams.some((x) => x.stale), heard: streams.every((x) => x.ageMs !== null), rows: r.rows, notes };
	});
	return { panels, streaming: snapshot.wanted, channelMapKnown: context.channelMap !== null, requests: snapshot.requests };
}

/** "20.1 Hz"; "—" for a rate not measured yet. */
export function formatHz(hz: number | null): string {
	return hz === null ? DASH : `${hz.toFixed(DECIMALS.rate_hz)} Hz`;
}

/** "0.4 s"; "never" for a stream not heard. */
export function formatStreamAge(ms: number | null): string {
	return ms === null ? 'never' : `${(ms / MS_PER_S).toFixed(1)} s`;
}

export interface LiveDeps {
	telemetry: Pick<TelemetryClient, 'snapshot' | 'start' | 'stop' | 'maintain'>;
	/** The stick channels of the running config; null (the default) until the parameters were read. */
	channelMap?: Readable<ChannelMap | null>;
	now?: () => number;
	timing?: ShellTiming;
}

/** Keeps the panels true: asks for the streams while the page is open, stops them when it closes, and recomputes the panels against the clock. */
export class LiveController {
	private readonly now: () => number;
	private readonly timing: ShellTiming;
	private readonly store = writable<LiveView | null>(null);
	private timer: ReturnType<typeof setInterval> | null = null;
	private map: ChannelMap | null = null;
	private unsubscribe: (() => void) | null = null;
	readonly view: Readable<LiveView | null> = { subscribe: this.store.subscribe };

	constructor(private readonly deps: LiveDeps) {
		this.now = deps.now ?? (() => Date.now());
		this.timing = deps.timing ?? SHELL_TIMING;
	}

	/** The page opened: asks for the streams and starts the tick. A second call does nothing. */
	start(): void {
		if (this.timer) return;
		this.unsubscribe = (this.deps.channelMap ?? readable<ChannelMap | null>(null)).subscribe((m) => {
			this.map = m;
			this.recompute();
		});
		this.deps.telemetry.start();
		this.timer = setInterval(() => this.tick(), LIVE_TICK_MS);
		this.recompute();
	}

	/** The page closed: stops the tick and the streams. */
	stop(): void {
		if (!this.timer) return;
		clearInterval(this.timer);
		this.timer = null;
		this.unsubscribe?.();
		this.unsubscribe = null;
		this.deps.telemetry.stop();
	}

	/** One step: asks again for a stream that fell silent, and recomputes the panels. */
	tick(): void {
		this.deps.telemetry.maintain();
		this.recompute();
	}

	private recompute(): void {
		this.store.set(computeLive(this.deps.telemetry.snapshot(), this.now(), { channelMap: this.map }, this.timing));
	}
}
