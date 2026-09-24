/**
 * Dev-only stand-in for the backend + FC (?mock): answers the client messages with the setup semantics of gcs decision
 * (b), (c). ?mock=armed starts armed (save refused); ?mock=mismatch reports another schema hash. A point-mass vehicle
 * flies the mission API about a fixed home with the executor's rules (ADR-0010 (b), (d)): arm, climb, hold, mission, return
 * to home and hold over it (no automatic landing), land and disarm.
 */
import { LocalFrame, type Ned } from './geo';
import { climbError, missionError, type MissionMsg } from './mission';
import type { ClientMsg, GeoPoint, LatLonAlt, MissionMode, Schema, SetupHeader } from './types';

/** The Gazebo world's origin, as in tests/test_geo.cpp. */
const HOME: GeoPoint = { lat_e7: 473763880, lon_e7: 85477780, alt_m: 408 };
const SPEED_H = 6;
const SPEED_V = 2.5;
const SPEED_LAND = 1;
const ARRIVED_M = 0.3;
/** MOT_SPIN_ARM, and the mock's hover command. */
const SPIN_ARM = 0.1;
const HOVER = 0.68;

interface Setup {
	kind: number[];
	values: number[];
}

const copy = (s: Setup): Setup => ({ kind: s.kind.slice(), values: s.values.slice() });

/** CRC-32 over kinds and f32 values, standing in for the FC's crc of its Setup. */
function crc(s: Setup): number {
	const buf = new DataView(new ArrayBuffer(s.kind.length + 4 * s.values.length));
	s.kind.forEach((k, i) => buf.setUint8(i, k));
	s.values.forEach((v, i) => buf.setFloat32(s.kind.length + 4 * i, v, true));
	let c = 0xffffffff;
	for (let i = 0; i < buf.byteLength; i++) {
		c ^= buf.getUint8(i);
		for (let b = 0; b < 8; b++) c = c & 1 ? (c >>> 1) ^ 0xedb88320 : c >>> 1;
	}
	return (c ^ 0xffffffff) >>> 0;
}

export class MockFc {
	onopen: (() => void) | null = null;
	onmessage: ((ev: { data: string }) => void) | null = null;
	onclose: (() => void) | null = null;

	private readonly specs: { min: number; max: number }[] = [];
	private staged: Setup;
	private running: Setup;
	private stored: Setup;
	private storedValid = true;
	private armed: boolean;
	private readonly hash: number;
	private readonly timer: ReturnType<typeof setInterval>;
	private t = 0;
	private readonly frame = new LocalFrame(HOME);
	private p: Ned = [0, 0, 0];
	private yaw = 0;
	private mode: MissionMode;
	private climbAlt: number | null = null;
	/** Where climb, hold and land hold the horizontal position (and climb/hold the altitude). */
	private hold: Ned = [0, 0, 0];
	private wps: LatLonAlt[] = [];
	private wpIndex = -1;
	/** The executor's home: the horizontal position at arm. */
	private homeNed: [number, number] | null = null;
	/** The return legs still to fly. */
	private legs: Ned[] = [];
	private reason = '';
	private ticks = 0;

	constructor(
		private readonly schema: Schema,
		flag: string
	) {
		for (const f of schema.families) for (const k of f.kinds) for (const p of k.params) this.specs[p.index] = { min: p.min, max: p.max };
		const f0 = schema.factory[0];
		this.staged = { kind: f0.kinds.slice(), values: f0.values.map((v) => Math.fround(v)) };
		this.running = copy(this.staged);
		this.stored = copy(this.staged);
		this.armed = flag === 'armed';
		this.mode = this.armed ? 'armed' : 'disarmed';
		this.hash = flag === 'mismatch' ? (schema.schema_hash ^ 0x5a5a5a5a) >>> 0 : schema.schema_hash;
		setTimeout(() => {
			this.onopen?.();
			this.emit({ type: 'link', mode: 'mock', connected: true, header: this.header() });
			this.missionState();
		}, 50);
		this.timer = setInterval(() => this.telemetry(), 50);
	}

	close(): void {
		clearInterval(this.timer);
		this.onclose?.();
	}

	send(text: string): void {
		const m = JSON.parse(text) as ClientMsg;
		// The link's latency, so pending edits are visible.
		setTimeout(() => this.handle(m), 30);
	}

	private handle(m: ClientMsg): void {
		switch (m.type) {
			case 'request_setup':
				return this.setup(true);
			case 'set_param': {
				const s = this.specs[m.index];
				if (s && Number.isFinite(m.value) && m.value >= s.min && m.value <= s.max) this.staged.values[m.index] = Math.fround(m.value);
				return this.emit({ type: 'param', index: m.index, value: this.staged.values[m.index] ?? 0 });
			}
			case 'set_kind':
				return this.setKind(m.family, m.kind);
			case 'load_factory': {
				const f = this.schema.factory.find((x) => x.id === m.id);
				if (f) this.staged = { kind: f.kinds.slice(), values: f.values.map((v) => Math.fround(v)) };
				return this.setup(true);
			}
			case 'save':
				if (!this.armed) {
					this.stored = copy(this.staged);
					this.storedValid = true;
				}
				return this.setup(false);
			case 'reset':
				this.running = copy(this.staged);
				this.disarm();
				return this.setup(false);
			case 'reboot':
				this.running = copy(this.stored);
				this.staged = copy(this.stored);
				this.disarm();
				return this.setup(true);
			case 'flash':
				return this.flash();
			default:
				return this.mission(m);
		}
	}

	private disarm(reason = ''): void {
		this.armed = false;
		this.p[2] = 0;
		this.enter('disarmed', reason);
	}

	private enter(mode: MissionMode, reason: string, hold: Ned = this.hold): void {
		this.mode = mode;
		this.reason = reason;
		this.hold = hold;
		if (mode !== 'mission') this.wpIndex = -1;
		this.missionState();
	}

	/** Up (never down) to max(altitude now, climb altitude) where it is, then across to home at that altitude. */
	private returnHome(reason: string): void {
		const [n, e, d] = this.p;
		const h = this.homeNed ?? [0, 0];
		const z = -Math.max(-d, this.climbAlt ?? 0);
		this.legs = [
			[n, e, z],
			[h[0], h[1], z]
		];
		this.enter('rth', reason);
	}

	/** The mission requests, refused (as the backend would report it) outside the states that allow them. */
	private mission(m: MissionMsg): void {
		const s = this.mode;
		const refuse = (why: string): void => this.emit({ type: 'error', request: m.type, error: `refused in ${s}: ${why}` });
		const [n, e] = this.p;
		switch (m.type) {
			case 'arm':
				if (s !== 'disarmed') return refuse('already armed');
				this.armed = true;
				this.homeNed = [n, e];
				return this.enter('armed', '');
			case 'disarm':
				if (s === 'disarmed') return refuse('already disarmed');
				return this.disarm();
			case 'climb': {
				if (s !== 'armed' && s !== 'hold') return refuse('climb needs armed or hold');
				const why = climbError(m.alt_m);
				if (why) return refuse(why);
				this.climbAlt = m.alt_m;
				return this.enter('climb', '', [n, e, -m.alt_m]);
			}
			case 'mission_start': {
				if (s !== 'hold') return refuse('climb to the safe altitude first');
				const why = missionError(Array.isArray(m.waypoints) ? m.waypoints : [], this.homeLatLon());
				if (why) return refuse(why);
				this.wps = m.waypoints.map((w) => ({ ...w }));
				this.wpIndex = 0;
				return this.enter('mission', '');
			}
			case 'rth':
				if (s !== 'climb' && s !== 'hold' && s !== 'mission' && s !== 'land') return refuse('not flying');
				return this.returnHome('');
			case 'land':
				if (s !== 'climb' && s !== 'hold' && s !== 'mission' && s !== 'rth') return refuse('not flying');
				return this.enter('land', '', [n, e, 0]);
		}
	}

	private homeLatLon(): { lat: number; lon: number } | null {
		if (!this.homeNed) return null;
		const g = this.frame.latLonOf([this.homeNed[0], this.homeNed[1], 0]);
		return { lat: g.lat, lon: g.lon };
	}

	private target(): Ned | null {
		switch (this.mode) {
			case 'climb':
			case 'hold':
			case 'land':
				return this.hold;
			case 'mission':
				return this.frame.nedOf(this.wps[this.wpIndex]);
			case 'rth':
				return this.legs[0];
			default:
				return null;
		}
	}

	/** One tick of the point mass toward the target; arriving advances the state. */
	private fly(dt: number): void {
		const t = this.target();
		if (!t) return;
		const [dn, de, dd] = [t[0] - this.p[0], t[1] - this.p[1], t[2] - this.p[2]];
		const h = Math.hypot(dn, de);
		const step = Math.min(h, SPEED_H * dt);
		if (h > 1e-6) {
			this.p[0] += (dn / h) * step;
			this.p[1] += (de / h) * step;
		}
		if (h > 1) this.yaw = Math.atan2(de, dn);
		const v = (this.mode === 'land' ? SPEED_LAND : SPEED_V) * dt;
		this.p[2] += Math.max(-v, Math.min(v, dd));
		if (this.mode === 'hold' || Math.hypot(h, dd) > ARRIVED_M) return;
		if (this.mode === 'climb') this.enter('hold', 'altitude reached', t);
		else if (this.mode === 'mission') {
			if (++this.wpIndex < this.wps.length) this.missionState();
			else this.returnHome('mission complete');
		} else if (this.mode === 'rth') {
			this.legs.shift();
			if (this.legs.length) this.missionState();
			else this.enter('hold', 'home reached', t);
		} else if (this.mode === 'land') this.disarm('landed');
	}

	private missionState(): void {
		const t = this.target();
		this.emit({
			type: 'mission_state',
			state: this.mode,
			wp_index: this.wpIndex,
			wp_count: this.wps.length,
			target: t ? this.frame.latLonOf(t) : null,
			dist_m: t ? Math.hypot(t[0] - this.p[0], t[1] - this.p[1], t[2] - this.p[2]) : null,
			climb_alt_m: this.climbAlt,
			home: this.homeLatLon(),
			reason: this.reason
		});
	}

	/**
	 * The FC's class rules: a vehicle change re-stages every family whose kind does not serve it to its first kind that
	 * does; a kind that does not serve the staged vehicle is refused (the header echoes the held kind, the backend adds
	 * the error).
	 */
	private setKind(family: number, kind: number): void {
		const fams = this.schema.families;
		const vf = fams.findIndex((f) => f.id === 'vehicle');
		const def = fams[family]?.kinds[kind];
		const serves = (f: number, k: number, v: string): boolean => fams[f].kinds[k]?.vehicles.includes(v) ?? false;
		const vehicle = fams[vf].kinds[this.staged.kind[vf]].id;
		if (def && family === vf) {
			this.staged.kind[vf] = kind;
			fams.forEach((_, f) => {
				if (!serves(f, this.staged.kind[f], def.id)) this.staged.kind[f] = Math.max(0, fams[f].kinds.findIndex((k) => k.vehicles.includes(def.id)));
			});
		} else if (def && serves(family, kind, vehicle)) {
			this.staged.kind[family] = kind;
		}
		this.setup(false);
		if (this.staged.kind[family] !== kind)
			this.emit({ type: 'error', request: 'set_kind', family, error: `refused: ${def?.id ?? `#${kind}`} does not serve the ${vehicle} vehicle` });
	}

	private flash(): void {
		const lines = ['[mock] closing serial port', '[mock] cmake --build build/fw', '[100%] Built target marv_fw', '[mock] openocd: ** Programming Finished **', '[mock] ** Verified OK **', '[mock] reopening serial port'];
		lines.forEach((line, i) => setTimeout(() => this.emit({ type: 'flash_log', line }), 200 * (i + 1)));
		setTimeout(() => {
			this.running = copy(this.stored);
			this.staged = copy(this.stored);
			this.emit({ type: 'link', mode: 'mock', connected: true, header: this.header() });
			this.setup(true);
		}, 200 * (lines.length + 1));
	}

	private header(): SetupHeader {
		return {
			schema_hash: this.hash,
			param_count: this.staged.values.length,
			kind: this.staged.kind.slice(),
			running_crc: crc(this.running),
			staged_crc: crc(this.staged),
			stored_crc: crc(this.stored),
			stored_valid: this.storedValid,
			armed: this.armed
		};
	}

	private setup(values: boolean): void {
		this.emit(values ? { type: 'setup', header: this.header(), values: this.staged.values } : { type: 'setup', header: this.header() });
	}

	private telemetry(): void {
		this.t += 0.05;
		this.fly(0.05);
		if (++this.ticks % 10 === 0 && this.mode !== 'disarmed') this.missionState();
		const r = crc(this.running);
		const preset = this.schema.factory.find((f) => crc({ kind: f.kinds, values: f.values }) === r)?.id ?? 0xff;
		const yaw = this.yaw;
		const roll = this.armed ? 0.02 * Math.sin(3 * this.t) : 0;
		const q = [Math.cos(yaw / 2) * Math.cos(roll / 2), Math.cos(yaw / 2) * Math.sin(roll / 2), -Math.sin(yaw / 2) * Math.sin(roll / 2), Math.sin(yaw / 2) * Math.cos(roll / 2)];
		this.emit({
			type: 'telemetry',
			est: { p_ned: this.p.slice(), q },
			preset,
			armed: this.armed,
			thrust_hover: 0.6811,
			brake: 0,
			home_valid: true,
			home: HOME,
			geo: this.frame.latLonOf(this.p),
			motor: [0, 1, 2, 3].map((i) => (!this.armed ? 0 : this.mode === 'armed' ? SPIN_ARM : HOVER + 0.01 * Math.sin(3 * this.t + i)))
		});
	}

	private emit(m: unknown): void {
		this.onmessage?.({ data: JSON.stringify(m) });
	}
}
