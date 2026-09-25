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

/** The arm input; index/on_above are used when source is axis, button when source is button. */
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

/** The profile input; axis/bands are used when source is axis, buttons when source is buttons. */
export interface ProfileCfg {
	source: 'axis' | 'buttons' | 'none';
	axis: number;
	bands: Band[];
	/** Button index (as a string) -> profile id. */
	buttons: Record<string, string>;
}

export interface RadioConfig {
	version: number;
	device_name: string;
	sticks: Record<StickName, StickCfg>;
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
			axis: int(p.axis, 5),
			bands: Array.isArray(p.bands) ? p.bands.filter(isObj).map((b) => ({ upper: num(b.upper, NaN), profile: typeof b.profile === 'string' ? b.profile : '' })) : [],
			buttons
		}
	};
}

/** The config as PUT /api/radio/config takes it: only the active source's fields. */
export function toWire(c: RadioConfig): Obj {
	const arm: Obj =
		c.arm.source === 'axis'
			? { source: 'axis', index: c.arm.index, on_above: c.arm.on_above, require_throttle_low: c.arm.require_throttle_low }
			: { source: 'button', button: c.arm.button, require_throttle_low: c.arm.require_throttle_low };
	if (c.arm.disarm_button !== null) arm.disarm_button = c.arm.disarm_button;
	const p = c.profile;
	const profile: Obj =
		p.source === 'axis'
			? { source: 'axis', axis: p.axis, bands: p.bands.map((b) => ({ upper: b.upper, profile: b.profile })) }
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

/** Everything wrong with a config for this device (axes/buttons 0: not checked) and these profile ids; empty when it can be saved. */
export function validateConfig(c: RadioConfig, profiles: string[], axes = 0, buttons = 0): string[] {
	const e: string[] = [];
	const axisOk = (i: number): boolean => Number.isInteger(i) && i >= 0 && (axes <= 0 || i < axes);
	const buttonOk = (i: number): boolean => Number.isInteger(i) && i >= 0 && (buttons <= 0 || i < buttons);
	const raw = (v: number): boolean => Number.isInteger(v) && v >= RAW_MIN && v <= RAW_MAX;
	if (c.version !== 1) e.push(`version ${c.version}: only 1 is known`);
	if (!c.device_name) e.push('no device name');
	const used = new Map<number, string>();
	for (const k of STICKS) {
		const s = c.sticks[k];
		if (!axisOk(s.axis)) e.push(`${k}: axis ${s.axis} is not an axis of this device`);
		else if (used.has(s.axis)) e.push(`${k}: axis ${s.axis} is already ${used.get(s.axis)}`);
		else used.set(s.axis, k);
		if (!raw(s.min) || !raw(s.center) || !raw(s.max)) e.push(`${k}: min, center and max must be whole numbers in ${RAW_MIN}..${RAW_MAX}`);
		else if (!(s.min < s.max)) e.push(`${k}: min must be below max`);
		else if (s.center < s.min || s.center > s.max) e.push(`${k}: center must lie within min..max`);
		if (!(s.deadband >= 0 && s.deadband <= DEADBAND_MAX)) e.push(`${k}: deadband must be 0..${DEADBAND_MAX}`);
	}
	const a = c.arm;
	if (a.source === 'axis') {
		if (!axisOk(a.index)) e.push(`arm: axis ${a.index} is not an axis of this device`);
		else if (used.has(a.index)) e.push(`arm: axis ${a.index} is the ${used.get(a.index)} stick`);
		if (!(a.on_above > -1 && a.on_above < 1)) e.push('arm: the threshold must be strictly inside -1..1');
	} else if (!buttonOk(a.button)) e.push(`arm: button ${a.button} is not a button of this device`);
	if (a.disarm_button !== null) {
		if (!buttonOk(a.disarm_button)) e.push(`arm: disarm button ${a.disarm_button} is not a button of this device`);
		else if (a.source === 'button' && a.disarm_button === a.button) e.push('arm: the disarm button is the arm button');
	}
	const p = c.profile;
	const known = (id: string): boolean => profiles.includes(id);
	if (p.source === 'axis') {
		if (!axisOk(p.axis)) e.push(`flight modes: axis ${p.axis} is not an axis of this device`);
		else if (used.has(p.axis)) e.push(`flight modes: axis ${p.axis} is the ${used.get(p.axis)} stick`);
		else if (a.source === 'axis' && a.index === p.axis) e.push(`flight modes: axis ${p.axis} is the arm switch`);
		if (p.bands.length < BANDS_MIN || p.bands.length > BANDS_MAX) e.push(`flight modes: ${BANDS_MIN} to ${BANDS_MAX} bands, not ${p.bands.length}`);
		p.bands.forEach((b, i) => {
			if (!Number.isFinite(b.upper) || b.upper <= -1 || b.upper > 1) e.push(`flight modes: band ${i + 1}'s upper edge must be in (-1, 1]`);
			else if (i > 0 && !(b.upper > p.bands[i - 1].upper)) e.push(`flight modes: band ${i + 1}'s upper edge must be above band ${i}'s`);
			if (!known(b.profile)) e.push(`flight modes: band ${i + 1} has no known profile`);
		});
		if (p.bands.length && p.bands[p.bands.length - 1].upper !== 1) e.push('flight modes: the last band must end at 1.0');
	} else if (p.source === 'buttons') {
		const entries = Object.entries(p.buttons);
		if (!entries.length) e.push('flight modes: assign at least one button');
		for (const [k, id] of entries) {
			const i = Number(k);
			if (!/^\d+$/.test(k) || !buttonOk(i)) e.push(`flight modes: button ${k} is not a button of this device`);
			else if (a.source === 'button' && i === a.button) e.push(`flight modes: button ${k} is the arm button`);
			else if (a.disarm_button === i) e.push(`flight modes: button ${k} is the disarm button`);
			if (!known(id)) e.push(`flight modes: button ${k} has no known profile`);
		}
	}
	return e;
}

// ---- HTTP ----

export type FetchFn = (url: string, init?: { method?: string; body?: string; headers?: Record<string, string> }) => Promise<{ ok: boolean; status: number; json(): Promise<unknown> }>;

export interface RadioApi {
	devices(): Promise<RadioDevice[]>;
	/** The config and whether it is the stored one or the backend's default. */
	config(device: string): Promise<{ config: RadioConfig; source: 'stored' | 'default' }>;
	save(c: RadioConfig): Promise<RadioConfig>;
	reset(device: string): Promise<void>;
}

async function call(f: FetchFn, method: string, url: string, body?: unknown): Promise<unknown> {
	let r: Awaited<ReturnType<FetchFn>>;
	try {
		r = await f(url, body === undefined ? { method } : { method, body: JSON.stringify(body), headers: { 'Content-Type': 'application/json' } });
	} catch (e) {
		throw new Error(`${method} ${url}: ${e instanceof Error ? e.message : String(e)}`);
	}
	let j: unknown = null;
	try {
		j = await r.json();
	} catch {
		/* no body */
	}
	if (!r.ok) throw new Error(`${method} ${url}: ${isObj(j) && typeof j.error === 'string' ? j.error : `HTTP ${r.status}`}`);
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
			return { config: parseConfig(j), source: isObj(j) && j.source === 'stored' ? 'stored' : 'default' };
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
