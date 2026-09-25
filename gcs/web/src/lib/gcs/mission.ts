/** Mission control: waypoint validation, the controls each mission state allows, and presets kept in the browser. */
import { LocalFrame, toE7 } from './geo';
import type { ClientMsg, LatLonAlt, MissionMode, MissionStatus, Telemetry } from './types';

export type MissionRequest = 'arm' | 'disarm' | 'climb' | 'mission_start' | 'rth' | 'land';
const REQUESTS: readonly MissionRequest[] = ['arm', 'disarm', 'climb', 'mission_start', 'rth', 'land'];
export type MissionMsg = Extract<ClientMsg, { type: MissionRequest }>;
export const isMissionRequest = (r: string): r is MissionRequest => (REQUESTS as readonly string[]).includes(r);

/** Above this (m above home) a disarm asks first. */
export const DISARM_CONFIRM_ALT_M = 0.5;

/** The executor's bounds (ADR-0010 (b)). */
export const MAX_WAYPOINTS = 64;
export const ALT_MIN_M = 0.5;
export const ALT_MAX_M = 200;
export const MAX_RANGE_M = 5000;
/** A mission's speed (ADR-0011 Q3); none flies the setup's guidance cruise speed. */
export const SPEED_MIN_MPS = 0.5;
export const SPEED_MAX_MPS = 20;
/** The executor advances to the next waypoint within this distance of one (ADR-0011 Q3 kArriveWp). */
export const ACCEPT_WP_M = 2;

/** A horizontal point in decimal degrees: the executor's home. */
export type LatLon = { lat: number; lon: number };

const altOk = (alt_m: number): boolean => Number.isFinite(alt_m) && alt_m >= ALT_MIN_M && alt_m <= ALT_MAX_M;

/** Horizontal distance (m) of p from home, in the FC's flat-earth frame about home. */
export function rangeFromHome(p: LatLon, home: LatLon): number {
	const [n, e] = new LocalFrame({ lat_e7: toE7(home.lat), lon_e7: toE7(home.lon), alt_m: 0 }).nedOf({ lat: p.lat, lon: p.lon, alt_m: 0 });
	return Math.hypot(n, e);
}

/** Why a waypoint cannot be flown, or null; the range from home is checked once home is known. */
export function waypointError(w: LatLonAlt, home: LatLon | null = null): string | null {
	if (!Number.isFinite(w.lat) || w.lat < -90 || w.lat > 90) return 'latitude must be within -90..90 deg';
	if (!Number.isFinite(w.lon) || w.lon < -180 || w.lon > 180) return 'longitude must be within -180..180 deg';
	if (!altOk(w.alt_m)) return `altitude must be within ${ALT_MIN_M}..${ALT_MAX_M} m above home`;
	if (home) {
		const d = rangeFromHome(w, home);
		if (!(d <= MAX_RANGE_M)) return `${(d / 1000).toFixed(2)} km from home; at most ${MAX_RANGE_M / 1000} km`;
	}
	return null;
}

const decimal = (s: string): number => (/^\s*[-+]?(\d+\.?\d*|\.\d+)\s*$/.test(s) ? Number(s) : NaN);

/** A waypoint typed as decimal degrees and metres above home, or why it is not one. */
export function parseWaypoint(lat: string, lon: string, alt: string, home: LatLon | null = null): LatLonAlt | string {
	const w = { lat: decimal(lat), lon: decimal(lon), alt_m: decimal(alt) };
	return waypointError(w, home) ?? w;
}

/** Why the list cannot be started as a mission, or null. */
export function missionError(wps: readonly LatLonAlt[], home: LatLon | null = null): string | null {
	if (wps.length === 0) return 'no waypoints';
	if (wps.length > MAX_WAYPOINTS) return `${wps.length} waypoints; at most ${MAX_WAYPOINTS}`;
	for (let i = 0; i < wps.length; i++) {
		const e = waypointError(wps[i], home);
		if (e) return `waypoint ${i + 1}: ${e}`;
	}
	return null;
}

/** Why the mission speed cannot be flown, or null; null means the setup's cruise speed. */
export function speedError(speed_mps: number | null): string | null {
	if (speed_mps === null) return null;
	return Number.isFinite(speed_mps) && speed_mps >= SPEED_MIN_MPS && speed_mps <= SPEED_MAX_MPS ? null : `speed must be within ${SPEED_MIN_MPS}..${SPEED_MAX_MPS} m/s, or empty for cruise`;
}

/** A typed mission speed: null when empty (cruise), the speed, or why it is not one. */
export function parseSpeed(s: string): number | null | string {
	if (s.trim() === '') return null;
	const v = decimal(s);
	return speedError(v) ?? v;
}

/** The mission_start message; speed_mps only when a speed is given, profile only when one is selected. */
export function missionStart(wps: readonly LatLonAlt[], speed_mps: number | null, profile: string | null = null): Extract<ClientMsg, { type: 'mission_start' }> {
	const waypoints = wps.map((w) => ({ ...w }));
	const m: Extract<ClientMsg, { type: 'mission_start' }> = speed_mps === null ? { type: 'mission_start', waypoints } : { type: 'mission_start', waypoints, speed_mps };
	if (profile !== null) m.profile = profile;
	return m;
}

export function climbError(alt_m: number): string | null {
	return altOk(alt_m) ? null : `safe altitude must be within ${ALT_MIN_M}..${ALT_MAX_M} m`;
}

/** The list with row i moved by delta (clamped), as a new array. */
export function moveWaypoint(wps: readonly LatLonAlt[], i: number, delta: number): LatLonAlt[] {
	const out = wps.slice();
	const j = Math.max(0, Math.min(out.length - 1, i + delta));
	const [w] = out.splice(i, 1);
	out.splice(j, 0, w);
	return out;
}

export interface ControlContext {
	/** The last mission_state, or null before the backend reported one. */
	state: MissionMode | null;
	/** Backend open and FC connected. */
	connected: boolean;
	climbAlt: number;
	waypoints: readonly LatLonAlt[];
	/** For the range check; null skips it. */
	home: LatLon | null;
	/** The mission speed; null or absent flies cruise. */
	speed?: number | null;
}

/** Which mission controls may be pressed. */
export function enabledControls(c: ControlContext): Record<MissionRequest, boolean> {
	const s = c.connected ? c.state : null;
	return {
		arm: s === 'disarmed',
		disarm: s !== null && s !== 'disarmed',
		climb: (s === 'armed' || s === 'hold') && climbError(c.climbAlt) === null,
		mission_start: s === 'hold' && missionError(c.waypoints, c.home) === null && speedError(c.speed ?? null) === null,
		rth: s === 'climb' || s === 'hold' || s === 'mission' || s === 'land',
		land: s === 'climb' || s === 'hold' || s === 'mission' || s === 'rth'
	};
}

/** A disarm asks first when the vehicle may be flying. */
export function disarmNeedsConfirm(state: MissionMode | null, alt_m: number): boolean {
	return !(state === 'disarmed' || state === 'armed') || !(alt_m <= DISARM_CONFIRM_ALT_M);
}

/** The executor's reason for display, or null when it gave none. */
export function reasonText(m: MissionStatus | null): string | null {
	return m?.reason.trim() || null;
}

/** Armed and any motor commanded above zero. */
export function propsSpinning(t: Telemetry | null): boolean {
	return t !== null && t.armed && t.motor !== null && t.motor.some((v) => v > 0);
}

export interface MissionPreset {
	name: string;
	waypoints: LatLonAlt[];
	/** Absent: the setup's cruise speed. */
	speed_mps?: number;
}

export const PRESET_KEY = 'marv-gcs.mission-presets';
const FORMAT = 'marv-mission-presets';

interface KeyValue {
	getItem(key: string): string | null;
	setItem(key: string, value: string): void;
}

function parsePreset(v: unknown): MissionPreset | null {
	if (typeof v !== 'object' || v === null) return null;
	const o = v as Record<string, unknown>;
	if (typeof o.name !== 'string' || !o.name.trim() || !Array.isArray(o.waypoints)) return null;
	const waypoints: LatLonAlt[] = [];
	for (const w of o.waypoints) {
		if (typeof w !== 'object' || w === null) return null;
		const r = w as Record<string, unknown>;
		const p = { lat: Number(r.lat), lon: Number(r.lon), alt_m: Number(r.alt_m) };
		if (typeof r.lat !== 'number' || typeof r.lon !== 'number' || typeof r.alt_m !== 'number' || waypointError(p)) return null;
		waypoints.push(p);
	}
	if (o.speed_mps === undefined || o.speed_mps === null) return { name: o.name.trim(), waypoints };
	if (typeof o.speed_mps !== 'number' || speedError(o.speed_mps)) return null;
	return { name: o.name.trim(), waypoints, speed_mps: o.speed_mps };
}

/** Presets from an export file, a bare list, or one preset; invalid entries are named in rejected. */
export function importPresets(parsed: unknown): { presets: MissionPreset[]; rejected: string[] } {
	const o = parsed as Record<string, unknown> | null;
	const list: unknown[] = Array.isArray(parsed) ? parsed : o && Array.isArray(o.presets) ? o.presets : o && typeof o === 'object' ? [o] : [];
	const presets: MissionPreset[] = [];
	const rejected: string[] = [];
	list.forEach((v, i) => {
		const p = parsePreset(v);
		if (p) presets.push(p);
		else rejected.push(typeof (v as { name?: unknown })?.name === 'string' ? String((v as { name: string }).name) : `#${i + 1}`);
	});
	return { presets, rejected };
}

export function exportPresets(presets: readonly MissionPreset[]): { format: string; version: number; presets: MissionPreset[] } {
	return { format: FORMAT, version: 1, presets: presets.map((p) => ({ name: p.name, waypoints: p.waypoints.map((w) => ({ ...w })), ...(p.speed_mps === undefined ? {} : { speed_mps: p.speed_mps }) })) };
}

/** The list with p added, replacing a preset of the same name. */
export function upsertPreset(presets: readonly MissionPreset[], p: MissionPreset): MissionPreset[] {
	const i = presets.findIndex((x) => x.name === p.name);
	return i < 0 ? [...presets, p] : presets.map((x, j) => (j === i ? p : x));
}

/** The stored presets; none when storage is unavailable or holds something else. */
export function readPresets(store: KeyValue | null): MissionPreset[] {
	try {
		const text = store?.getItem(PRESET_KEY);
		return text ? importPresets(JSON.parse(text)).presets : [];
	} catch {
		return [];
	}
}

/** Stores the presets; returns why it could not, or null. */
export function writePresets(store: KeyValue | null, presets: readonly MissionPreset[]): string | null {
	try {
		if (!store) return 'browser storage unavailable';
		store.setItem(PRESET_KEY, JSON.stringify(exportPresets(presets)));
		return null;
	} catch (e) {
		return e instanceof Error ? e.message : String(e);
	}
}
