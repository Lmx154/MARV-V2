// The SIM panels (ground-segment unit GX-8): what the page shows of the simulator's sim channel. computeSimView() is pure: the same
// snapshot and the same `now` give the same panels. Everything here is labelled SIM and comes from the simulator, not the vehicle
// (invariant S-1): truth the vehicle cannot know, and what the plugin injects into it.
//
// A simulation is present while records arrive (SIM_TIMING.staleMs); without one there are no panels at all, not stale ones.
import { derived, readable, writable, type Readable } from 'svelte/store';
import type { SimChannelClient, SimSnapshot, SimTiming } from './channel';
import { SIM_TIMING } from './channel';
import { isArmedName, type SimRecord, type Vec3 } from './record';

export type SimTone = 'neutral' | 'ok' | 'warn' | 'bad';
export type SimPanelId = 'run' | 'truth' | 'injected' | 'hil';

export interface SimRow {
	/** Unique within the panel. */
	key: string;
	group: string | null;
	name: string;
	value: string;
	unit: string;
	tone: SimTone;
	/** A bar's position, 0..1; null for a row without a bar. */
	bar: number | null;
}

export interface SimPanelView {
	id: SimPanelId;
	/** Always starts with "SIM". */
	title: string;
	rows: SimRow[];
	notes: string[];
}

export interface SimView {
	/** Records are arriving: the page shows the panels. */
	present: boolean;
	panels: SimPanelView[];
	/** Browser-side counts of the stream. */
	stream: { hz: number | null; count: number; bad: number; lost: number; connected: boolean };
}

/** How often the view is recomputed against the clock (ms, a labelled harness value: the display's frame period). */
export const SIM_TICK_MS = 200;

const DEG_PER_RAD = 180 / Math.PI;
const NS_PER_S = 1e9;
/** The largest DShot throttle value (11 bits; fw/prim constants kDshotThrottleMax). */
const DSHOT_THROTTLE_MAX = 2047;
const DASH = '—';
const DECIMALS = { position: 2, velocity: 2, angle: 1, rate: 1, accel: 2, speed: 1, time: 2, rtf: 2, hz: 1, stick: 2, us: 0 } as const;

function fixed(v: number | null | undefined, decimals: number): string {
	if (v === null || v === undefined || !Number.isFinite(v)) return DASH;
	const text = v.toFixed(decimals);
	return /^-0(\.0*)?$/.test(text) ? text.slice(1) : text;
}

const clamp = (v: number, lo: number, hi: number): number => Math.min(hi, Math.max(lo, v));

/** Roll, pitch, yaw (rad) of a body-to-NED quaternion [w, x, y, z], Z-Y-X. */
export function eulerOf(q: readonly [number, number, number, number]): Vec3 {
	const [w, x, y, z] = q;
	return [Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)), Math.asin(clamp(2 * (w * y - z * x), -1, 1)), Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))];
}

class Rows {
	readonly rows: SimRow[] = [];
	constructor(private readonly panel: string) {}
	add(group: string | null, key: string, name: string, value: string, unit = '', extra: { tone?: SimTone; bar?: number | null } = {}): void {
		this.rows.push({ key: `${this.panel}.${key}`, group, name, value, unit, tone: extra.tone ?? 'neutral', bar: extra.bar ?? null });
	}
	vec(group: string, key: string, prefix: string, v: Vec3, scale: number, decimals: number, unit: string, axes = ['x', 'y', 'z']): void {
		axes.forEach((axis, i) => this.add(group, `${key}.${axis}`, `${prefix}${axis}`, fixed(v[i]! * scale, decimals), unit));
	}
}

function runPanel(r: SimRecord, stream: SimView['stream']): SimPanelView {
	const rows = new Rows('sim-run');
	rows.add('Run', 'kind', 'run', r.flags.hil ? 'HIL (the board flies)' : 'SIL (the firmware in the simulator)');
	rows.add('Run', 'time', 'sim time', fixed(r.simTimeNs / NS_PER_S, DECIMALS.time), 's');
	rows.add('Run', 'rtf', 'real-time factor', fixed(r.rtf, DECIMALS.rtf), '', { tone: r.rtf === 0 ? 'neutral' : 'ok' });
	rows.add('Run', 'tick', 'tick', String(r.tick));
	rows.add('Channel', 'rate', 'records', fixed(stream.hz, DECIMALS.hz), 'Hz');
	rows.add('Channel', 'lost', 'records lost', String(stream.lost), '', { tone: stream.lost > 0 ? 'warn' : 'ok' });
	rows.add('Channel', 'bad', 'not records', String(stream.bad), '', { tone: stream.bad > 0 ? 'warn' : 'ok' });
	rows.add('Pad and ground', 'pad', 'launch pad', !r.flags.launchPad ? 'none' : r.flags.padHeld ? 'holding' : r.flags.padReleased ? 'released' : 'waiting');
	rows.add('Pad and ground', 'ground', 'ground', r.flags.ground ? 'on' : 'off');
	rows.add('Pad and ground', 'touchdowns', 'touchdowns', r.flags.ground ? String(r.touchdowns) : DASH);
	rows.add('Pad and ground', 'impact', 'last impact', r.flags.ground && r.touchdowns > 0 ? fixed(r.lastTouchdownDownMs, DECIMALS.speed) : DASH, 'm/s down');
	return { id: 'run', title: 'SIM · Run', rows: rows.rows, notes: [] };
}

function truthPanel(r: SimRecord): SimPanelView {
	const rows = new Rows('sim-truth');
	rows.vec('Position (NED)', 'pos', '', r.posNed, 1, DECIMALS.position, 'm', ['north', 'east', 'down']);
	rows.add('Position (NED)', 'height', 'height above the start', fixed(-r.posNed[2], DECIMALS.position), 'm');
	rows.vec('Velocity (NED)', 'vel', '', r.velNed, 1, DECIMALS.velocity, 'm/s', ['north', 'east', 'down']);
	const e = eulerOf(r.q);
	rows.add('Attitude', 'roll', 'roll', fixed(e[0] * DEG_PER_RAD, DECIMALS.angle), '°');
	rows.add('Attitude', 'pitch', 'pitch', fixed(e[1] * DEG_PER_RAD, DECIMALS.angle), '°');
	rows.add('Attitude', 'yaw', 'yaw', fixed(e[2] * DEG_PER_RAD, DECIMALS.angle), '°');
	rows.vec('Body rate (FRD)', 'omega', '', r.omegaFrd, DEG_PER_RAD, DECIMALS.rate, '°/s');
	rows.vec('Specific force (FRD)', 'sf', '', r.specificForceFrd, 1, DECIMALS.accel, 'm/s²');
	r.rotorSpeed.forEach((w, i) => rows.add('Rotor speed', `rotor.${i + 1}`, `motor ${i + 1}`, fixed(w, DECIMALS.rate), 'rad/s'));
	return {
		id: 'truth',
		title: 'SIM · Truth',
		rows: rows.rows,
		notes: ['Simulator truth: the vehicle does not know these values. The specific force is the rotor thrust per mass only; the sample injected into the firmware (SIM · Injected) also has drag and the pad.']
	};
}

function injectedPanel(r: SimRecord): SimPanelView {
	const rows = new Rows('sim-injected');
	const hasImu = r.flags.imuModel;
	const imu = hasImu ? r.imu : null;
	rows.vec('Gyro sample', 'gyro', '', imu?.gyro ?? [NaN, NaN, NaN], DEG_PER_RAD, DECIMALS.rate, '°/s');
	rows.vec('Accelerometer sample', 'accel', '', imu?.accel ?? [NaN, NaN, NaN], 1, DECIMALS.accel, 'm/s²');
	r.dshotCommanded.forEach((d, i) =>
		rows.add(r.flags.hil ? 'DShot commanded by the board' : 'DShot commanded by the firmware', `cmd.${i + 1}`, `motor ${i + 1}`, String(d), 'DShot', { bar: clamp(d / DSHOT_THROTTLE_MAX, 0, 1) })
	);
	r.dshotApplied.forEach((d, i) => rows.add('DShot applied to the plant', `applied.${i + 1}`, `motor ${i + 1}`, String(d), 'DShot', { bar: clamp(d / DSHOT_THROTTLE_MAX, 0, 1) }));
	return {
		id: 'injected',
		title: 'SIM · Injected',
		rows: rows.rows,
		notes: [
			'What the simulator feeds the firmware (the IMU sample) and what it takes from it (DShot), at the last tick of the record. The applied command is the commanded one after the simulator\'s command delay.',
			...(hasImu ? [] : ['This world injects no plant-model IMU sample.'])
		]
	};
}

function hilPanel(r: SimRecord): SimPanelView | null {
	const h = r.hil;
	if (!h) return null;
	const rows = new Rows('sim-hil');
	const armed = isArmedName(h.armingName);
	rows.add('Board', 'arming', 'arm state', h.armingName ?? 'no status yet', '', { tone: h.armingName === null ? 'neutral' : armed ? 'bad' : 'ok' });
	rows.add('Board', 'mode', 'flight mode', h.mode === 'unknown' ? DASH : h.mode);
	rows.add('Board', 'deadline', 'IMU deadline misses', String(h.deadlineMisses), '', { tone: h.deadlineMisses > 0 ? 'bad' : 'ok' });
	rows.add('Board', 'faults', 'link faults', String(h.linkFaults), '', { tone: h.linkFaults > 0 ? 'bad' : 'ok' });
	rows.add('Receiver', 'frames', 'frames', String(h.receiverFrames));
	rows.add('Receiver', 'quality', 'link quality', fixed(h.linkQuality * 100, 0), '%');
	['roll', 'pitch', 'yaw', 'throttle'].forEach((n, i) => rows.add('Receiver', `stick.${n}`, n, fixed(h.sticks[i], DECIMALS.stick), '', { bar: clamp(((h.sticks[i] ?? 0) + (n === 'throttle' ? 0 : 1)) / (n === 'throttle' ? 1 : 2), 0, 1) }));
	rows.add('Command margin', 'min', 'smallest', h.reportsUsed > 0 ? fixed(h.marginMinUs, DECIMALS.us) : DASH, 'µs', { tone: h.reportsUsed > 0 && h.marginMinUs < 0 ? 'bad' : 'neutral' });
	rows.add('Command margin', 'max', 'largest', h.reportsUsed > 0 ? fixed(h.marginMaxUs, DECIMALS.us) : DASH, 'µs');
	rows.add('Command margin', 'reports', 'board reports used', String(h.reportsUsed));
	rows.add('Schedule', 'lag', 'lag absorbed', fixed(h.lagUs / 1000, DECIMALS.speed), 'ms');
	rows.add('Schedule', 'lagevents', 'late ticks', String(h.lagEvents), '', { tone: h.lagEvents > 0 ? 'warn' : 'ok' });
	return {
		id: 'hil',
		title: 'SIM · HIL board',
		rows: rows.rows,
		notes: [
			'The HIL image has no link: the board\'s state reaches the GCS here, from the simulator host\'s side of the board\'s port.',
			'Not available: a count of late commands (a late command stops the run, so a running record has none), the board\'s attitude estimate and its other telemetry.'
		]
	};
}

export function computeSimView(s: SimSnapshot, now: number, timing: SimTiming = SIM_TIMING): SimView {
	const stream = { hz: s.hz, count: s.count, bad: s.bad, lost: s.lost, connected: s.connected };
	const present = s.record !== null && s.receivedAt !== null && now - s.receivedAt <= timing.staleMs;
	if (!present || !s.record) return { present: false, panels: [], stream };
	const r = s.record;
	const panels = [runPanel(r, stream), truthPanel(r), injectedPanel(r), hilPanel(r)].filter((p): p is SimPanelView => p !== null);
	return { present: true, panels, stream };
}

export interface SimViewDeps {
	client: Pick<SimChannelClient, 'snapshot' | 'start' | 'stop'>;
	now?: () => number;
	timing?: SimTiming;
}

/** Keeps the SIM view true against the clock; starts the client with start(). */
export class SimViewController {
	private readonly now: () => number;
	private readonly timing: SimTiming;
	private readonly store = writable<SimView>({ present: false, panels: [], stream: { hz: null, count: 0, bad: 0, lost: 0, connected: false } });
	private timer: ReturnType<typeof setInterval> | null = null;
	readonly view: Readable<SimView> = { subscribe: this.store.subscribe };
	/** Whether a simulation is publishing; changes only when that does, so a layout can read it cheaply. */
	readonly present: Readable<boolean>;

	constructor(private readonly deps: SimViewDeps) {
		this.now = deps.now ?? (() => Date.now());
		this.timing = deps.timing ?? SIM_TIMING;
		this.present = derived(this.view, (v, set: (x: boolean) => void) => set(v.present), false);
	}

	start(): void {
		if (this.timer) return;
		this.deps.client.start();
		this.timer = setInterval(() => this.tick(), SIM_TICK_MS);
		this.tick();
	}

	stop(): void {
		if (!this.timer) return;
		clearInterval(this.timer);
		this.timer = null;
		this.deps.client.stop();
	}

	tick(): void {
		this.store.set(computeSimView(this.deps.client.snapshot(), this.now(), this.timing));
	}
}

export const NO_SIM: Readable<boolean> = readable(false);
