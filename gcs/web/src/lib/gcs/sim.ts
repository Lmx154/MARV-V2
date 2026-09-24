/** The sim launcher: the airframe catalog, the world's environment, and the controls the launcher allows. */
import type { Airframe, AirframeSpecs, ClientMsg, SimEnv, SimStatus, SimTarget } from './types';

type Obj = Record<string, unknown>;
const isObj = (v: unknown): v is Obj => typeof v === 'object' && v !== null && !Array.isArray(v);
const num = (v: unknown): number => (typeof v === 'number' && Number.isFinite(v) ? v : NaN);

/** The Gazebo worlds' origin (tests/test_geo.cpp). */
export const DEFAULT_ENV: SimEnv = {
	wind_speed_ms: 0,
	wind_dir_deg: 0,
	gust_sigma_ms: 0,
	lat: 47.376388,
	lon: 8.547778,
	elevation_m: 408,
	temperature_c: null,
	pressure_pa: null
};

const SPEC_KEYS: readonly (keyof Omit<AirframeSpecs, 'rotors'>)[] = [
	'mass_kg',
	'ixx',
	'iyy',
	'izz',
	'arm_m',
	'rotor_count',
	'motor_constant',
	'moment_constant',
	'max_rot_velocity',
	'time_constant_up',
	'time_constant_down',
	't_max_n',
	'thrust_to_weight',
	'hover_thrust_frac'
];

function rotors(v: unknown): [number, number][] | undefined {
	if (!Array.isArray(v)) return undefined;
	const out: [number, number][] = [];
	for (const r of v) {
		const p: [number, number] = Array.isArray(r) ? [num(r[0]), num(r[1])] : isObj(r) ? [num(r.x), num(r.y)] : [NaN, NaN];
		if (!p.every(Number.isFinite)) return undefined;
		out.push(p);
	}
	return out.length ? out : undefined;
}

/** GET /api/sim/airframes: a list, or {airframes: [...]}; entries without an id are dropped, missing specs are NaN. */
export function parseAirframes(json: unknown): Airframe[] {
	const list = Array.isArray(json) ? json : isObj(json) && Array.isArray(json.airframes) ? json.airframes : [];
	const out: Airframe[] = [];
	for (const a of list) {
		if (!isObj(a) || typeof a.id !== 'string' || !a.id) continue;
		const s = isObj(a.specs) ? a.specs : {};
		const specs = Object.fromEntries(SPEC_KEYS.map((k) => [k, num(s[k])])) as unknown as AirframeSpecs;
		const r = rotors(s.rotors);
		if (r) specs.rotors = r;
		out.push({ id: a.id, label: typeof a.label === 'string' && a.label ? a.label : a.id, frame: String(a.frame ?? ''), source: String(a.source ?? ''), specs });
	}
	return out;
}

/** Why the environment cannot be launched, or null. Temperature and pressure are optional (null). */
export function envError(e: SimEnv): string | null {
	const bad = (v: unknown): boolean => typeof v !== 'number' || !Number.isFinite(v);
	if (bad(e.wind_speed_ms) || e.wind_speed_ms < 0) return 'wind speed must be 0 or more m/s';
	if (bad(e.wind_dir_deg) || e.wind_dir_deg < 0 || e.wind_dir_deg > 360) return 'wind direction must be within 0..360 deg';
	if (bad(e.gust_sigma_ms) || e.gust_sigma_ms < 0) return 'gust sigma must be 0 or more m/s';
	if (bad(e.lat) || e.lat < -90 || e.lat > 90) return 'latitude must be within -90..90 deg';
	if (bad(e.lon) || e.lon < -180 || e.lon > 180) return 'longitude must be within -180..180 deg';
	if (bad(e.elevation_m)) return 'elevation must be a number of metres';
	if (e.temperature_c !== null && bad(e.temperature_c)) return 'temperature must be a number of deg C, or blank';
	if (e.pressure_pa !== null && (bad(e.pressure_pa) || e.pressure_pa <= 0)) return 'pressure must be a positive number of Pa, or blank';
	return null;
}

/** A blank optional field (temperature, pressure) is null. */
export const optional = (v: unknown): number | null => (typeof v === 'number' && !Number.isNaN(v) ? v : null);

/** The launch request; the direction is sent within [0, 360). */
export function launchMsg(airframe: string, env: SimEnv, gui: boolean, target: SimTarget): Extract<ClientMsg, { type: 'sim_launch' }> {
	return {
		type: 'sim_launch',
		airframe,
		env: { ...env, wind_dir_deg: ((env.wind_dir_deg % 360) + 360) % 360, temperature_c: optional(env.temperature_c), pressure_pa: optional(env.pressure_pa) },
		gui,
		target
	};
}

export interface LauncherContext {
	wsOpen: boolean;
	status: SimStatus | null;
	/** The selected airframe is in the catalog. */
	airframe: boolean;
	envError: string | null;
	/** A launch or stop was sent and neither a sim_status nor an error has answered it. */
	pending: boolean;
}

/** Which launcher buttons may be pressed, and why LAUNCH may not. */
export function launcherControls(c: LauncherContext): { launch: boolean; stop: boolean; why: string | null } {
	const running = c.status?.running ?? false;
	let why: string | null = null;
	if (!c.wsOpen) why = 'backend closed';
	else if (c.pending) why = 'waiting for the backend';
	else if (running) why = 'a sim is running; stop it first';
	else if (!c.airframe) why = 'select an airframe';
	else if (c.envError) why = c.envError;
	return { launch: why === null, stop: c.wsOpen && !c.pending && running, why };
}

/** The air's velocity (m/s, north and east) of a wind of speed s blowing FROM fromDeg (clockwise from north). */
export function windToward(s: number, fromDeg: number): { n: number; e: number } {
	const r = (fromDeg * Math.PI) / 180;
	return { n: -s * Math.cos(r), e: -s * Math.sin(r) };
}

const POINTS = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW'];
export const compass = (deg: number): string => (Number.isFinite(deg) ? POINTS[Math.round((((deg % 360) + 360) % 360) / 45) % 8] : '—');
