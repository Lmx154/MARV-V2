/**
 * The Controller tab's model: the pilot's device (a radio in USB joystick mode, or a gamepad), its stored mapping (GET/PUT/
 * DELETE /api/radio/config) and the live stream (the radio message while subscribed). Raw axes are int16; a switch
 * axis reads raw / 32767 in -1..1, and a profile band is the first whose upper edge is above that (the last, upper 1.0,
 * takes the rest), as marv_ground's ch6_profile does.
 */

export const STICKS = ['roll', 'pitch', 'throttle', 'yaw'] as const;
export type StickName = (typeof STICKS)[number];

export const RAW_MIN = -32768;
export const RAW_MAX = 32767;
export const BANDS_MIN = 2;
export const BANDS_MAX = 6;
export const DEADBAND_MAX = 0.5;
/** The backend's limits on axis and button indices (a plugged-in device's own counts, when fewer, apply instead). */
export const MAX_AXES = 16;
export const MAX_BUTTONS = 32;
/** The least gap between two band edges, and the least travel calibration accepts for an assigned axis (raw). */
export const EDGE_STEP = 0.01;
export const CAL_MIN_TRAVEL = 8000;

/** GET /api/radio/devices, and the radio_devices message. */
export interface RadioDevice {
	path: string;
	name: string;
	axes: number;
	buttons: number;
	has_config: boolean;
}

export interface StickCfg {
	axis: number;
	min: number;
	center: number;
	max: number;
	reverse: boolean;
	/** Fraction of each half of the travel that reads 0. */
	deadband: number;
}

/** The arm input; index/on_above are used when source is axis, button when source is button (which needs disarm_button). */
export interface ArmCfg {
	source: 'axis' | 'button';
	index: number;
	on_above: number;
	button: number;
	require_throttle_low: boolean;
	disarm_button: number | null;
}

export interface Band {
	/** Upper edge in -1..1; the last band's is 1. */
	upper: number;
	profile: string;
}

/** The profile input; index (the axis) and bands are used when source is axis, buttons when source is buttons. */
export interface ProfileCfg {
	source: 'axis' | 'buttons' | 'none';
	index: number;
	bands: Band[];
	/** Button index (as a string) -> profile id. */
	buttons: Record<string, string>;
}

export interface RadioConfig {
	version: number;
	device_name: string;
	sticks: Record<StickName, StickCfg>;
	/** The backend takes only true: the throttle's centre holds height. */
	throttle_centre_hold: boolean;
	arm: ArmCfg;
	profile: ProfileCfg;
}

/** The radio message: what the backend reads and computes from the stored config. */
export interface RadioLive {
	device: string;
	axes: number[];
	buttons: number[];
	normalized: Record<StickName, number>;
	arm: boolean;
	profile: string | null;
}

type Obj = Record<string, unknown>;
const isObj = (v: unknown): v is Obj => typeof v === 'object' && v !== null && !Array.isArray(v);
const num = (v: unknown, d: number): number => (typeof v === 'number' && Number.isFinite(v) ? v : d);
const int = (v: unknown, d: number): number => (typeof v === 'number' && Number.isInteger(v) ? v : d);

export function parseDevices(v: unknown): RadioDevice[] {
	if (!Array.isArray(v)) return [];
	return v.filter(isObj).filter((d) => typeof d.name === 'string' && d.name !== '').map((d) => ({
		path: typeof d.path === 'string' ? d.path : '',
		name: String(d.name),
		axes: int(d.axes, 0),
		buttons: int(d.buttons, 0),
		has_config: Boolean(d.has_config)
	}));
}

export function parseLive(m: Obj): RadioLive {
	const n = isObj(m.normalized) ? m.normalized : {};
	return {
		device: typeof m.device === 'string' ? m.device : '',
		axes: Array.isArray(m.axes) ? m.axes.map((a) => num(a, 0)) : [],
		buttons: Array.isArray(m.buttons) ? m.buttons.map((b) => (b ? 1 : 0)) : [],
		normalized: { roll: num(n.roll, NaN), pitch: num(n.pitch, NaN), throttle: num(n.throttle, NaN), yaw: num(n.yaw, NaN) },
		arm: Boolean(m.arm),
		profile: typeof m.profile === 'string' && m.profile !== '' ? m.profile : null
	};
}

function stick(v: unknown, axis: number): StickCfg {
	const s = isObj(v) ? v : {};
	return {
		axis: int(s.axis, axis),
		min: num(s.min, -32767),
		center: num(s.center, 0),
		max: num(s.max, 32767),
		reverse: Boolean(s.reverse),
		deadband: num(s.deadband, 0.1)
	};
}

/** A config as the backend sends it (the source field and anything else extra are ignored), with defaults for gaps. */
export function parseConfig(v: unknown): RadioConfig {
	const c = isObj(v) ? v : {};
	const s = isObj(c.sticks) ? c.sticks : {};
	const a = isObj(c.arm) ? c.arm : {};
	const p = isObj(c.profile) ? c.profile : {};
	const armSource = a.source === 'button' ? 'button' : 'axis';
	const buttons: Record<string, string> = {};
	if (isObj(p.buttons)) for (const [k, id] of Object.entries(p.buttons)) if (typeof id === 'string') buttons[k] = id;
	return {
		version: int(c.version, 1),
		device_name: typeof c.device_name === 'string' ? c.device_name : '',
		sticks: { roll: stick(s.roll, 0), pitch: stick(s.pitch, 1), throttle: stick(s.throttle, 2), yaw: stick(s.yaw, 3) },
		throttle_centre_hold: c.throttle_centre_hold === undefined ? true : Boolean(c.throttle_centre_hold),
		arm: {
			source: armSource,
			index: int(a.index, 4),
			on_above: num(a.on_above, 0),
			button: int(armSource === 'button' ? (a.button ?? a.index) : a.button, 0),
			require_throttle_low: a.require_throttle_low === undefined ? true : Boolean(a.require_throttle_low),
			disarm_button: typeof a.disarm_button === 'number' && Number.isInteger(a.disarm_button) ? a.disarm_button : null
		},
		profile: {
			source: p.source === 'buttons' ? 'buttons' : p.source === 'none' ? 'none' : 'axis',
			index: int(p.index, 5),
			bands: Array.isArray(p.bands) ? p.bands.filter(isObj).map((b) => ({ upper: num(b.upper, NaN), profile: typeof b.profile === 'string' ? b.profile : '' })) : [],
			buttons
		}
	};
}

/** The config as PUT /api/radio/config takes it and GET returns it (ground/src/radio_json.hpp's to_json): only the active source's fields. */
export function toWire(c: RadioConfig): Obj {
	const arm: Obj =
		c.arm.source === 'axis'
			? { source: 'axis', index: c.arm.index, on_above: c.arm.on_above, require_throttle_low: c.arm.require_throttle_low }
			: { source: 'button', button: c.arm.button, require_throttle_low: c.arm.require_throttle_low };
	arm.disarm_button = c.arm.disarm_button;
	const p = c.profile;
	const profile: Obj =
		p.source === 'axis'
			? { source: 'axis', index: p.index, bands: p.bands.map((b) => ({ upper: b.upper, profile: b.profile })) }
			: p.source === 'buttons'
				? { source: 'buttons', buttons: { ...p.buttons } }
				: { source: 'none' };
	const sticks = Object.fromEntries(STICKS.map((k) => [k, { ...c.sticks[k] }]));
	return { version: c.version, device_name: c.device_name, sticks, throttle_centre_hold: c.throttle_centre_hold, arm, profile };
}

export const sameConfig = (a: RadioConfig | null, b: RadioConfig | null): boolean => JSON.stringify(a && toWire(a)) === JSON.stringify(b && toWire(b));

// ---- switch axes and bands ----

/** A raw axis as a switch reads it, -1..1. */
export const axisValue = (raw: number): number => Math.max(-1, Math.min(1, raw / 32767));

export const bandLower = (bands: Band[], i: number): number => (i > 0 ? bands[i - 1].upper : -1);

/** The band a switch value falls in: the first whose upper edge is above it, else the last; -1 with no bands. */
export function bandIndex(bands: Band[], v: number): number {
	if (!bands.length || !Number.isFinite(v)) return -1;
	const i = bands.findIndex((b) => v < b.upper);
	return i < 0 ? bands.length - 1 : i;
}

const round2 = (v: number): number => Math.round(v * 100) / 100;

/** n equal bands over -1..1, profiles taken in order (repeating the last). */
export function evenBands(n: number, profiles: string[]): Band[] {
	const k = Math.max(BANDS_MIN, Math.min(BANDS_MAX, n));
	return Array.from({ length: k }, (_, i) => ({ upper: i === k - 1 ? 1 : round2(-1 + (2 * (i + 1)) / k), profile: profiles[Math.min(i, profiles.length - 1)] ?? '' }));
}

/** Splits band i in two at its middle (the new lower half takes the same profile); unchanged at BANDS_MAX. */
export function insertBand(bands: Band[], i: number): Band[] {
	if (bands.length >= BANDS_MAX || i < 0 || i >= bands.length) return bands;
	const lo = bandLower(bands, i);
	const mid = round2((lo + bands[i].upper) / 2);
	if (mid - lo < EDGE_STEP || bands[i].upper - mid < EDGE_STEP) return bands;
	return [...bands.slice(0, i), { upper: mid, profile: bands[i].profile }, ...bands.slice(i)];
}

/** Removes band i (its range goes to the band above, or, for the last, the one below extends to 1); unchanged at BANDS_MIN. */
export function removeBand(bands: Band[], i: number): Band[] {
	if (bands.length <= BANDS_MIN || i < 0 || i >= bands.length) return bands;
	const out = bands.filter((_, j) => j !== i).map((b) => ({ ...b }));
	out[out.length - 1].upper = 1;
	return out;
}

/** Swaps band i's profile with its neighbour's (dir -1 below, +1 above); the edges stay where they are. */
export function moveBand(bands: Band[], i: number, dir: -1 | 1): Band[] {
	const j = i + dir;
	if (i < 0 || j < 0 || i >= bands.length || j >= bands.length) return bands;
	const out = bands.map((b) => ({ ...b }));
	[out[i].profile, out[j].profile] = [bands[j].profile, bands[i].profile];
	return out;
}

/** Moves band i's upper edge, kept EDGE_STEP inside its neighbours; the last edge stays 1. */
export function setUpper(bands: Band[], i: number, v: number): Band[] {
	if (i < 0 || i >= bands.length - 1 || !Number.isFinite(v)) return bands;
	const lo = bandLower(bands, i) + EDGE_STEP;
	const hi = bands[i + 1].upper - EDGE_STEP;
	const out = bands.map((b) => ({ ...b }));
	out[i].upper = round2(Math.max(lo, Math.min(hi, v)));
	return out;
}

export const setBandProfile = (bands: Band[], i: number, profile: string): Band[] => bands.map((b, j) => (j === i ? { ...b, profile } : b));

// ---- sticks ----

/** A stick's raw value to -1..1 through its calibration, reverse and deadband (the band rescaled away, as marv_ground's stick()). */
export function normalizeStick(raw: number, s: StickCfg): number {
	const half = raw >= s.center ? s.max - s.center : s.center - s.min;
	let v = half > 0 ? (raw - s.center) / half : 0;
	v = Math.max(-1, Math.min(1, v));
	if (s.reverse) v = -v;
	const a = Math.abs(v);
	if (a <= s.deadband) return 0;
	return Math.sign(v) * Math.min((a - s.deadband) / (1 - s.deadband), 1);
}

// ---- calibration ----

export interface Calibration {
	phase: 'extremes' | 'centre';
	min: number[];
	max: number[];
	/** Sum and count of the centre phase's samples per axis. */
	sum: number[];
	count: number[];
}

export function calStart(axes: number): Calibration {
	return { phase: 'extremes', min: Array(axes).fill(Infinity), max: Array(axes).fill(-Infinity), sum: Array(axes).fill(0), count: Array(axes).fill(0) };
}

/** Folds one raw sample in: the extremes phase widens min/max, the centre phase averages. */
export function calSample(c: Calibration, raw: number[]): Calibration {
	const n = Math.max(c.min.length, raw.length);
	const at = (a: number[], i: number, d: number): number => (i < a.length ? a[i] : d);
	const next: Calibration = { phase: c.phase, min: [], max: [], sum: [], count: [] };
	for (let i = 0; i < n; i++) {
		const r = raw[i];
		const ok = typeof r === 'number' && Number.isFinite(r);
		next.min[i] = c.phase === 'extremes' && ok ? Math.min(at(c.min, i, Infinity), r) : at(c.min, i, Infinity);
		next.max[i] = c.phase === 'extremes' && ok ? Math.max(at(c.max, i, -Infinity), r) : at(c.max, i, -Infinity);
		next.sum[i] = at(c.sum, i, 0) + (c.phase === 'centre' && ok ? r : 0);
		next.count[i] = at(c.count, i, 0) + (c.phase === 'centre' && ok ? 1 : 0);
	}
	return next;
}

export const calCentrePhase = (c: Calibration): Calibration => ({ ...c, phase: 'centre', sum: c.sum.map(() => 0), count: c.count.map(() => 0) });

/** The captured centre of axis i: the centre samples' mean inside the captured range, or null before any. */
export function calCentre(c: Calibration, i: number): number | null {
	if (!c.count[i]) return null;
	const m = Math.round(c.sum[i] / c.count[i]);
	return Number.isFinite(c.min[i]) ? Math.max(c.min[i], Math.min(c.max[i], m)) : m;
}

/** The sticks' min/center/max from the capture; errors name the assigned axes that did not move enough or have no centre. */
export function calFinish(c: Calibration, cfg: RadioConfig): { config: RadioConfig; errors: string[] } {
	const errors: string[] = [];
	const sticks = { ...cfg.sticks };
	for (const k of STICKS) {
		const s = cfg.sticks[k];
		const i = s.axis;
		const lo = c.min[i];
		const hi = c.max[i];
		const mid = calCentre(c, i);
		if (!Number.isFinite(lo) || !Number.isFinite(hi) || hi - lo < CAL_MIN_TRAVEL) {
			errors.push(`${k} (axis ${i}) did not move far enough: move it to both ends`);
			continue;
		}
		if (mid === null) {
			errors.push(`${k} (axis ${i}) has no centre sample`);
			continue;
		}
		sticks[k] = { ...s, min: lo, max: hi, center: mid };
	}
	return { config: { ...cfg, sticks }, errors };
}

// ---- validation ----

/** The field path of a backend-style error, "sticks.yaw.axis: outside 0..7" -> "sticks.yaw.axis"; "" without one. */
export function errorPath(e: string): string {
	const i = e.indexOf(': ');
	return i < 0 ? '' : e.slice(0, i);
}

/** The errors whose path is one of these. */
export const errorsAt = (errors: string[], ...paths: string[]): string[] => errors.filter((e) => paths.includes(errorPath(e)));

/** The Controller page that shows an error next to its field: sticks.* and arm* on Sticks & arming, profile* on Flight modes; null: the top. */
export function errorPage(e: string): 'sticks' | 'modes' | null {
	const p = errorPath(e);
	if (/^sticks\.(roll|pitch|throttle|yaw)(\.(axis|min|center|max|reverse|deadband))?$/.test(p)) return 'sticks';
	if (/^arm(\.(source|index|on_above|button|require_throttle_low|disarm_button))?$/.test(p)) return 'sticks';
	if (/^profile(\.(source|index|bands|bands\[\d+\]\.(upper|profile)|buttons|buttons\.\d+))?$/.test(p)) return 'modes';
	return null;
}

/** A double as Boost.JSON writes it (0.5 -> 5E-1), as the backend's range messages show it. */
const boostDouble = (d: number): string => d.toExponential().replace('e+', 'E').replace('e', 'E');

/** The backend's field reader: each refusal pushed as "path+key: why", the value or null. */
function reader(o: Obj, path: string, e: string[]) {
	const fail = (key: string, why: string): null => {
		e.push(`${path}${key}: ${why}`);
		return null;
	};
	const at = (key: string): unknown => (o[key] === undefined ? fail(key, 'missing') ?? undefined : o[key]);
	const numeric = (key: string): number | null => {
		const v = at(key);
		if (v === undefined) return null;
		return typeof v === 'number' && Number.isFinite(v) ? v : fail(key, 'not a number');
	};
	return {
		fail,
		at,
		integer(key: string, lo: number, hi: number): number | null {
			const d = numeric(key);
			if (d === null) return null;
			if (!Number.isInteger(d)) return fail(key, 'not an integer');
			return d < lo || d > hi ? fail(key, `outside ${lo}..${hi}`) : d;
		},
		number(key: string, lo: number, hi: number): number | null {
			const d = numeric(key);
			if (d === null) return null;
			return d >= lo && d <= hi ? d : fail(key, `outside ${boostDouble(lo)}..${boostDouble(hi)}`);
		},
		boolean(key: string): boolean | null {
			const v = at(key);
			if (v === undefined) return null;
			return typeof v === 'boolean' ? v : fail(key, 'not true or false');
		},
		object(key: string): Obj | null {
			const v = at(key);
			if (v === undefined) return null;
			return isObj(v) ? v : fail(key, 'not an object');
		},
		string(key: string): string | null {
			const v = at(key);
			if (v === undefined) return null;
			return typeof v === 'string' ? v : fail(key, 'not a string');
		}
	};
}

/**
 * ground/src/radio_json.hpp's from_json on a config as sent over the wire: its rules and its messages ("path: why"), the
 * device's axes and buttons capped at MAX_AXES / MAX_BUTTONS (the caps themselves when the device is not plugged in).
 * The backend stops at its first refusal, which is the first entry here; independent parts go on to list theirs.
 */
export function validateWire(v: unknown, profiles: string[], axes = MAX_AXES, buttons = MAX_BUTTONS): string[] {
	if (!isObj(v)) return ['not a JSON object'];
	const e: string[] = [];
	const nAxes = Math.min(axes, MAX_AXES);
	const nButtons = Math.min(buttons, MAX_BUTTONS);
	const f = Math.fround;
	const top = reader(v, '', e);
	top.integer('version', 1, 1);
	if (top.string('device_name') === '') top.fail('device_name', 'empty');

	const JOBS = ['sticks.roll', 'sticks.pitch', 'sticks.throttle', 'sticks.yaw', 'arm', 'profile'];
	const used = new Map<number, number>();
	const claim = (job: number, axis: number): void => {
		const u = used.get(axis);
		if (u !== undefined) e.push(`${JOBS[job]}: axis ${axis} is already ${JOBS[u]}`);
		else used.set(axis, job);
	};
	const sticks = top.object('sticks');
	if (sticks) {
		const sr = reader(sticks, 'sticks.', e);
		STICKS.forEach((k, i) => {
			const o = sr.object(k);
			if (!o) return;
			const r = reader(o, `sticks.${k}.`, e);
			const axis = r.integer('axis', 0, nAxes - 1);
			const min = axis === null ? null : r.integer('min', RAW_MIN, RAW_MAX);
			const center = min === null ? null : r.integer('center', RAW_MIN, RAW_MAX);
			const max = center === null ? null : r.integer('max', RAW_MIN, RAW_MAX);
			const reverse = max === null ? null : r.boolean('reverse');
			const deadband = reverse === null ? null : r.number('deadband', 0, DEADBAND_MAX);
			if (axis === null) return;
			if (min !== null && center !== null && max !== null && deadband !== null) {
				if (!(min < center)) r.fail('min', 'not below center');
				else if (!(center < max)) r.fail('max', 'not above center');
			}
			claim(i, axis);
		});
	}
	if (top.boolean('throttle_centre_hold') === false) top.fail('throttle_centre_hold', "only true (the throttle's centre holds height) is supported");

	let armButton: number | null = null;
	let disarm: number | null = null;
	const arm = top.object('arm');
	if (arm) {
		const r = reader(arm, 'arm.', e);
		const source = r.string('source');
		if (source === 'axis') {
			const i = r.integer('index', 0, nAxes - 1);
			if (i !== null && r.number('on_above', -1, 1) !== null) claim(4, i);
		} else if (source === 'button') {
			armButton = r.integer('button' in arm || !('index' in arm) ? 'button' : 'index', 0, nButtons - 1) ?? -1;
		} else if (source !== null) r.fail('source', 'not "axis" or "button"');
		r.boolean('require_throttle_low');
		const d = arm.disarm_button;
		if (d !== undefined && d !== null) disarm = r.integer('disarm_button', 0, nButtons - 1) ?? -1;
		if (armButton !== null && disarm === null) r.fail('disarm_button', 'required with an arm button');
		else if (armButton !== null && armButton >= 0 && disarm === armButton) r.fail('disarm_button', 'the arm button itself');
	}

	const profile = top.object('profile');
	if (profile) {
		const r = reader(profile, 'profile.', e);
		const source = r.string('source');
		if (source === 'axis') {
			const i = r.integer('index', 0, nAxes - 1);
			const bands = i === null ? undefined : r.at('bands');
			if (i !== null) claim(5, i);
			if (bands !== undefined && !Array.isArray(bands)) r.fail('bands', 'not an array');
			else if (Array.isArray(bands)) {
				if (bands.length < BANDS_MIN || bands.length > BANDS_MAX) r.fail('bands', `not ${BANDS_MIN} to ${BANDS_MAX} bands`);
				else
					for (let b = 0, prev = 0; b < bands.length; ++b) {
						const at = `profile.bands[${b}].`;
						const band = bands[b];
						if (!isObj(band)) {
							e.push(`${at}: not an object`);
							break;
						}
						const br = reader(band, at, e);
						const d = br.number('upper', -1, 1);
						if (d === null) break;
						const id = br.at('profile');
						if (id === undefined) break;
						const upper = f(d);
						if (typeof id !== 'string' || !profiles.includes(id)) br.fail('profile', 'not a profile id');
						else if (b > 0 && !(upper > prev)) br.fail('upper', 'not above the band before');
						else if (upper <= -1) br.fail('upper', 'not above -1');
						else if (b + 1 === bands.length && upper !== 1) br.fail('upper', "the last band's is not 1");
						else {
							prev = upper;
							continue;
						}
						break;
					}
			}
		} else if (source === 'buttons') {
			const map = r.object('buttons');
			if (map) {
				const keys = Object.keys(map);
				if (!keys.length) r.fail('buttons', 'empty');
				for (const key of keys) {
					const b = /^\s*[+-]?\d+$/.test(key) ? Number(key.trim()) : NaN;
					const id = map[key];
					if (!(b >= 0 && b < nButtons)) e.push(`profile.buttons.${key}: not a button 0..${nButtons - 1}`);
					else if (typeof id !== 'string' || !profiles.includes(id)) e.push(`profile.buttons.${key}: not a profile id`);
					else if (b === armButton || b === disarm) e.push(`profile.buttons.${key}: the button already arms or disarms`);
				}
			}
		} else if (source !== null && source !== 'none') r.fail('source', 'not "axis", "buttons" or "none"');
	}
	return e;
}

/** Everything the backend would refuse in this config for a device with these counts (undefined: not plugged in); empty when it can be saved. */
export const validateConfig = (c: RadioConfig, profiles: string[], axes?: number, buttons?: number): string[] => validateWire(toWire(c), profiles, axes, buttons);

// ---- HTTP ----

export type FetchFn = (url: string, init?: { method?: string; body?: string; headers?: Record<string, string> }) => Promise<{ ok: boolean; status: number; json(): Promise<unknown> }>;

/** A refused or failed request: the HTTP status (0: no reply) and the backend's error, else the status or the failure. */
export class ApiError extends Error {
	constructor(
		message: string,
		readonly status: number,
		readonly detail: string
	) {
		super(message);
	}
}

export interface RadioApi {
	devices(): Promise<RadioDevice[]>;
	/** The config, whether it is the stored one or the backend's default, and why a stored one was ignored (else null). */
	config(device: string): Promise<{ config: RadioConfig; source: 'stored' | 'default'; error: string | null }>;
	save(c: RadioConfig): Promise<RadioConfig>;
	reset(device: string): Promise<void>;
}

async function call(f: FetchFn, method: string, url: string, body?: unknown): Promise<unknown> {
	let r: Awaited<ReturnType<FetchFn>>;
	try {
		r = await f(url, body === undefined ? { method } : { method, body: JSON.stringify(body), headers: { 'Content-Type': 'application/json' } });
	} catch (e) {
		const why = e instanceof Error ? e.message : String(e);
		throw new ApiError(`${method} ${url}: ${why}`, 0, why);
	}
	let j: unknown = null;
	try {
		j = await r.json();
	} catch {
		/* no body */
	}
	if (!r.ok) {
		const why = isObj(j) && typeof j.error === 'string' ? j.error : `HTTP ${r.status}`;
		throw new ApiError(`${method} ${url}: ${why}`, r.status, why);
	}
	return j;
}

export function makeRadioApi(f: FetchFn): RadioApi {
	const q = (device: string): string => `/api/radio/config?device=${encodeURIComponent(device)}`;
	return {
		async devices() {
			return parseDevices(await call(f, 'GET', '/api/radio/devices'));
		},
		async config(device) {
			const j = await call(f, 'GET', q(device));
			return {
				config: parseConfig(j),
				source: isObj(j) && j.source === 'stored' ? 'stored' : 'default',
				error: isObj(j) && typeof j.error === 'string' && j.error !== '' ? j.error : null
			};
		},
		async save(c) {
			return parseConfig(await call(f, 'PUT', '/api/radio/config', toWire(c)));
		},
		async reset(device) {
			await call(f, 'DELETE', q(device));
		}
	};
}

/** The backend's radio API, or (?mock, dev only) the mock's in-memory store. */
export async function radioApi(mock: string | null): Promise<RadioApi> {
	if (import.meta.env.DEV && mock !== null) return makeRadioApi((await import('./mock-radio')).mockRadio.fetch);
	return makeRadioApi((url, init) => fetch(url, init));
}
