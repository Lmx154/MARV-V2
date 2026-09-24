import type { GeoPoint, LatLonAlt, LinkInfo, MissionMode, Schema, ServerMsg, SetupHeader, Telemetry } from './types';

type Obj = Record<string, unknown>;

const num = (v: unknown, d = 0): number => (typeof v === 'number' ? v : typeof v === 'boolean' ? Number(v) : d);
const isObj = (v: unknown): v is Obj => typeof v === 'object' && v !== null && !Array.isArray(v);

/** [x,y,z] from an array or an {x,y,z} object. */
function vec3(v: unknown): [number, number, number] {
	if (Array.isArray(v)) return [num(v[0]), num(v[1]), num(v[2])];
	if (isObj(v)) return [num(v.x), num(v.y), num(v.z)];
	return [NaN, NaN, NaN];
}

/** [w,x,y,z] from an array or a {w,x,y,z} object. */
function quat(v: unknown): [number, number, number, number] {
	if (Array.isArray(v)) return [num(v[0]), num(v[1]), num(v[2]), num(v[3])];
	if (isObj(v)) return [num(v.w), num(v.x), num(v.y), num(v.z)];
	return [NaN, NaN, NaN, NaN];
}

const MODES: readonly MissionMode[] = ['disarmed', 'armed', 'climb', 'hold', 'mission', 'rth', 'land'];
const finite = (v: unknown): v is number => typeof v === 'number' && Number.isFinite(v);
const numOrNull = (v: unknown): number | null => (finite(v) ? v : null);

/** {lat, lon, alt_m} in degrees and metres above home, or null without a finite lat and lon. */
function latLonAlt(v: unknown): LatLonAlt | null {
	if (!isObj(v) || !finite(v.lat) || !finite(v.lon)) return null;
	return { lat: v.lat, lon: v.lon, alt_m: num(v.alt_m, NaN) };
}

/** The FC's home, or null while it reports none (home_valid false, or absent with a zero point). */
function home(valid: unknown, h: unknown): GeoPoint | null {
	if (valid === false || !isObj(h) || !finite(h.lat_e7) || !finite(h.lon_e7)) return null;
	if (valid !== true && h.lat_e7 === 0 && h.lon_e7 === 0) return null;
	return { lat_e7: h.lat_e7, lon_e7: h.lon_e7, alt_m: num(h.alt_m, 0) };
}

export function parseHeader(h: unknown): SetupHeader | null {
	if (!isObj(h) || h.schema_hash === undefined) return null;
	const kind = Array.isArray(h.kind) ? h.kind : Array.isArray(h.kinds) ? h.kinds : [];
	return {
		schema_hash: num(h.schema_hash) >>> 0,
		param_count: num(h.param_count),
		kind: kind.map((k) => num(k)),
		running_crc: num(h.running_crc) >>> 0,
		staged_crc: num(h.staged_crc) >>> 0,
		stored_crc: num(h.stored_crc) >>> 0,
		stored_valid: Boolean(h.stored_valid),
		armed: Boolean(h.armed)
	};
}

/** One WS text frame from the backend, or null when it is not one this page acts on. */
export function parseServer(text: string): ServerMsg | null {
	let m: unknown;
	try {
		m = JSON.parse(text);
	} catch {
		return null;
	}
	if (!isObj(m)) return null;
	switch (m.type) {
		case 'link': {
			const link: LinkInfo = { mode: String(m.mode ?? ''), connected: Boolean(m.connected) };
			return { type: 'link', link, header: parseHeader(m.header) };
		}
		case 'setup': {
			const header = parseHeader(m.header ?? m);
			if (!header) return null;
			const values = Array.isArray(m.values) ? m.values.map((v) => num(v, NaN)) : null;
			return { type: 'setup', header, values };
		}
		case 'param':
			return { type: 'param', index: num(m.index, -1), value: num(m.value, NaN) };
		case 'telemetry': {
			const est = isObj(m.est) ? m.est : m;
			const telemetry: Telemetry = {
				p_ned: vec3(est.p_ned),
				q: quat(est.q),
				preset: num(m.preset, 0xff),
				armed: Boolean(m.armed),
				thrust_hover: num(m.thrust_hover, NaN),
				brake: num(m.brake, NaN),
				home: home(m.home_valid, m.home),
				geo: latLonAlt(m.geo ?? est.geo),
				motor: Array.isArray(m.motor) && m.motor.length === 4 ? [num(m.motor[0], NaN), num(m.motor[1], NaN), num(m.motor[2], NaN), num(m.motor[3], NaN)] : null
			};
			return { type: 'telemetry', telemetry };
		}
		case 'mission_state': {
			const state = MODES.find((x) => x === m.state);
			if (!state) return null;
			return {
				type: 'mission_state',
				mission: {
					state,
					wp_index: num(m.wp_index, -1),
					wp_count: num(m.wp_count, 0),
					target: latLonAlt(m.target),
					dist_m: numOrNull(m.dist_m),
					climb_alt_m: numOrNull(m.climb_alt_m),
					home: isObj(m.home) && finite(m.home.lat) && finite(m.home.lon) ? { lat: m.home.lat, lon: m.home.lon } : null,
					reason: typeof m.reason === 'string' ? m.reason : ''
				}
			};
		}
		case 'flash_log':
			return { type: 'flash_log', line: String(m.line ?? '') };
		case 'sim_status':
			return {
				type: 'sim_status',
				sim: {
					running: Boolean(m.running),
					airframe: typeof m.airframe === 'string' && m.airframe ? m.airframe : null,
					env: isObj(m.env) ? m.env : null,
					target: typeof m.target === 'string' && m.target ? m.target : null,
					gui: Boolean(m.gui),
					started_at: finite(m.started_at) ? m.started_at : typeof m.started_at === 'string' && m.started_at ? m.started_at : null,
					pid: numOrNull(m.pid)
				}
			};
		case 'sim_log':
			return { type: 'sim_log', line: String(m.line ?? '') };
		case 'error': {
			const e: Extract<ServerMsg, { type: 'error' }> = { type: 'error', request: String(m.request ?? ''), error: String(m.error ?? '') };
			if (typeof m.family === 'number') e.family = m.family;
			return e;
		}
		default:
			return null;
	}
}

/** Factory kinds may arrive as kind ids; the page works in kind indices (the wire's kind[7]). */
export function normalizeSchema(s: Schema): Schema {
	return {
		...s,
		schema_hash: s.schema_hash >>> 0,
		factory: s.factory.map((f) => ({
			...f,
			kinds: f.kinds.map((k, fi) => (typeof k === 'number' ? k : Math.max(0, s.families[fi]?.kinds.findIndex((d) => d.id === String(k)) ?? 0)))
		}))
	};
}

/** Roll, pitch, yaw (rad, ZYX) of the body-to-NED quaternion [w,x,y,z]. */
export function euler(q: [number, number, number, number]): { roll: number; pitch: number; yaw: number } {
	const [w, x, y, z] = q;
	return {
		roll: Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
		pitch: Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x)))),
		yaw: Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
	};
}
