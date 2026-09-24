import type { LinkInfo, Schema, ServerMsg, SetupHeader, Telemetry } from './types';

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
			const telemetry: Telemetry = { p_ned: vec3(est.p_ned), q: quat(est.q), preset: num(m.preset, 0xff), armed: Boolean(m.armed) };
			return { type: 'telemetry', telemetry };
		}
		case 'flash_log':
			return { type: 'flash_log', line: String(m.line ?? '') };
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
