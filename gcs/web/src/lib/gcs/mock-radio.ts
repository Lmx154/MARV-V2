/**
 * Dev-only stand-in for the backend's radio API (?mock): an in-memory config store behind a fetch-shaped function
 * (GET /api/radio/devices, GET/PUT/DELETE /api/radio/config), and the radio stream's frames. A RadioMaster (8 axes, AETR,
 * CH5 arm, CH6 four-position profile switch) is plugged in and moves its sticks and switches on a 20 s loop; MockFc plugs
 * an Xbox pad in 3 s after it opens. Defaults, validation and replies follow the backend's (ground/src/radio_config.hpp,
 * radio_json.hpp, gcs/src/radio.cpp).
 */
import fixture from '$lib/fixtures/schema.json';
import { axisValue, bandIndex, normalizeStick, parseConfig, STICKS, toWire, validateWire, type FetchFn, type RadioConfig, type RadioDevice } from './radio';

export const MOCK_RADIOMASTER: RadioDevice = { path: '/dev/input/js0', name: 'EdgeTX RadioMaster Pocket Joystick', axes: 8, buttons: 2, has_config: false };
export const MOCK_XBOX: RadioDevice = { path: '/dev/input/js1', name: 'Microsoft X-Box 360 pad', axes: 8, buttons: 11, has_config: false };

const CH6 = [-32767, -10923, 10923, 32767];

/** The backend's default for a device: the RadioMaster's when the name says EdgeTX or RadioMaster, else the Xbox pad's. */
export function mockDefault(name: string): RadioConfig {
	const stick = (axis: number, reverse = false) => ({ axis, min: -32767, center: 0, max: 32767, reverse, deadband: 0.1 });
	if (!/edgetx|radiomaster/i.test(name))
		return parseConfig({
			version: 1,
			device_name: name,
			sticks: { roll: stick(3), pitch: stick(4, true), throttle: stick(1, true), yaw: stick(0) },
			throttle_centre_hold: true,
			arm: { source: 'button', button: 0, require_throttle_low: false, disarm_button: 1 },
			profile: { source: 'buttons', buttons: { '2': 'hold', '3': 'stabilized', '4': 'freestyle', '5': 'agile' } }
		});
	return parseConfig({
		version: 1,
		device_name: name,
		sticks: { roll: stick(0), pitch: stick(1), throttle: stick(2), yaw: stick(3) },
		throttle_centre_hold: true,
		arm: { source: 'axis', index: 4, on_above: 0, require_throttle_low: true, disarm_button: null },
		profile: {
			source: 'axis',
			index: 5,
			bands: [
				{ upper: -0.50002, profile: 'hold' },
				{ upper: 0, profile: 'freestyle' },
				{ upper: 0.5, profile: 'stabilized' },
				{ upper: 1, profile: 'agile' }
			]
		}
	});
}

const clamp16 = (v: number): number => Math.max(-32767, Math.min(32767, Math.round(v)));
const ramp = (u: number, a: number, b: number): number => Math.max(0, Math.min(1, (u - a) / (b - a)));

/** The devices' raw state at time t (s). */
function rawAt(d: RadioDevice, t: number): { axes: number[]; buttons: number[] } {
	const u = t % 20;
	if (!/edgetx|radiomaster/i.test(d.name)) {
		const axes = [9000 * Math.sin(0.5 * t), -12000 * Math.sin(0.3 * t), -32767, 15000 * Math.sin(0.8 * t), -15000 * Math.cos(0.7 * t), -32767, 0, 0].map(clamp16);
		const buttons = Array(d.buttons).fill(0);
		const beat = Math.floor(t / 3) % 4;
		if (t % 3 < 0.5) buttons[2 + beat] = 1;
		if (u > 1 && u < 1.3) buttons[0] = 1;
		if (u > 15 && u < 15.3) buttons[1] = 1;
		return { axes, buttons };
	}
	// Throttle at the bottom while CH5 turns on (u = 1), up to mid-stick, back down before CH5 turns off (u = 13).
	const throttle = -32767 + 32767 * (ramp(u, 2, 4) - ramp(u, 10, 12)) + 3000 * Math.sin(t) * ramp(u, 4, 5) * (1 - ramp(u, 9, 10));
	const axes = [20000 * Math.sin(0.9 * t), 16000 * Math.sin(0.6 * t + 1), throttle, 10000 * Math.sin(0.4 * t), u >= 1 && u < 13 ? 32767 : -32767, CH6[Math.floor(t / 2.5) % 4], 0, -32767].map(clamp16);
	return { axes, buttons: Array(d.buttons).fill(0) };
}

type Res = Awaited<ReturnType<FetchFn>>;
const res = (status: number, body: unknown): Res => ({ ok: status >= 200 && status < 300, status, json: async () => body });

export class MockRadioStore {
	devices: RadioDevice[];
	private readonly stored = new Map<string, RadioConfig>();
	/** Devices whose stored file the backend would ignore, and why (GET's error). */
	private readonly ignored = new Map<string, string>();
	private readonly latch = new Map<string, { armed: boolean; sw: boolean | null; buttons: number[]; profile: string | null }>();

	constructor(
		private readonly profiles: string[],
		devices: RadioDevice[] = [MOCK_RADIOMASTER]
	) {
		this.devices = devices.map((d) => ({ ...d }));
	}

	private device(name: string): RadioDevice | undefined {
		return this.devices.find((d) => d.name === name);
	}

	/** Adds a device; false when one of that name is already there. */
	plug(d: RadioDevice): boolean {
		if (this.device(d.name)) return false;
		this.devices.push({ ...d, has_config: this.hasFile(d.name) });
		return true;
	}

	unplug(name: string): boolean {
		const n = this.devices.length;
		this.devices = this.devices.filter((d) => d.name !== name);
		return this.devices.length !== n;
	}

	configOf(name: string): RadioConfig {
		return this.stored.get(name) ?? mockDefault(name);
	}

	/** Makes the device's stored file one the backend ignores (its default applies, GET says why) until a PUT or DELETE. */
	ignore(name: string, why: string): void {
		this.stored.delete(name);
		this.ignored.set(name, why);
		this.mark(name);
	}

	/** has_config: a file is there, read or ignored. */
	private hasFile(name: string): boolean {
		return this.stored.has(name) || this.ignored.has(name);
	}

	private body(name: string): unknown {
		const why = this.ignored.get(name);
		return { ...toWire(this.configOf(name)), source: this.stored.has(name) ? 'stored' : 'default', ...(why ? { error: why } : {}) };
	}

	private mark(name: string): void {
		const d = this.device(name);
		if (d) d.has_config = this.hasFile(name);
	}

	fetch: FetchFn = async (url, init) => {
		const u = new URL(url, 'http://mock');
		const method = init?.method ?? 'GET';
		if (u.pathname === '/api/radio/devices' && method === 'GET') return res(200, this.devices.map((d) => ({ ...d })));
		if (u.pathname !== '/api/radio/config') return res(404, { error: `no route ${method} ${u.pathname}` });
		const name = u.searchParams.get('device');
		if (method === 'GET' || method === 'DELETE') {
			if (!name) return res(400, { error: 'device: missing' });
			if (method === 'DELETE') {
				this.stored.delete(name);
				this.ignored.delete(name);
				this.mark(name);
			}
			return res(200, this.body(name));
		}
		if (method !== 'PUT') return res(405, { error: 'GET, PUT or DELETE only' });
		let j: unknown;
		try {
			j = JSON.parse(init?.body ?? '');
		} catch (e) {
			return res(400, { error: `not JSON: ${e instanceof Error ? e.message : String(e)}` });
		}
		// Indices are checked against the device's counts when it is plugged in; the first refusal is the reply.
		const dn = typeof j === 'object' && j !== null && !Array.isArray(j) ? (j as Record<string, unknown>).device_name : undefined;
		const d = typeof dn === 'string' ? this.device(dn) : undefined;
		const errors = validateWire(j, this.profiles, d?.axes, d?.buttons);
		if (errors.length) return res(400, { error: errors[0] });
		const c = parseConfig(j);
		this.stored.set(c.device_name, c);
		this.ignored.delete(c.device_name);
		this.mark(c.device_name);
		return res(200, this.body(c.device_name));
	};

	/** The radio message for this device at time t (s), computed from its config; null when it is not plugged in. */
	frame(name: string, t: number): Record<string, unknown> | null {
		const d = this.device(name);
		if (!d) return null;
		const { axes, buttons } = rawAt(d, t);
		const c = this.configOf(name);
		const normalized = Object.fromEntries(STICKS.map((k) => [k, normalizeStick(axes[c.sticks[k].axis] ?? 0, c.sticks[k])]));
		const l = this.latch.get(name) ?? { armed: false, sw: null, buttons: [], profile: null };
		const pressed = (i: number | null): boolean => i !== null && Boolean(buttons[i]) && !l.buttons[i];
		const low = !c.arm.require_throttle_low || normalized.throttle <= -0.95;
		if (c.arm.source === 'axis') {
			const sw = axisValue(axes[c.arm.index] ?? 0) > c.arm.on_above;
			if (!sw) l.armed = false;
			else if (l.sw === false && low) l.armed = true;
			l.sw = sw;
		} else if (pressed(c.arm.button) && low) l.armed = true;
		if (pressed(c.arm.disarm_button)) l.armed = false;
		const p = c.profile;
		if (p.source === 'axis') l.profile = p.bands[bandIndex(p.bands, axisValue(axes[p.index] ?? 0))]?.profile ?? null;
		else if (p.source === 'buttons') {
			for (const [k, id] of Object.entries(p.buttons)) if (pressed(Number(k))) l.profile = id;
		} else l.profile = null;
		l.buttons = buttons;
		this.latch.set(name, l);
		return { type: 'radio', device: name, axes, buttons, normalized, arm: l.armed, profile: l.profile };
	}
}

export const mockRadio = new MockRadioStore(fixture.profiles.map((p) => p.id));
