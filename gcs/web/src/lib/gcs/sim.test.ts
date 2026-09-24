import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import fixture from '$lib/fixtures/schema.json';
import { MockFc } from './mock';
import { normalizeSchema, parseServer } from './protocol';
import { DEFAULT_ENV, envError, launchMsg, launcherControls, parseAirframes, type LauncherContext } from './sim';
import type { Schema, ServerMsg, SimStatus } from './types';

const running: SimStatus = { running: true, airframe: 'x500', env: {}, target: 'sitl', gui: true, started_at: 1, pid: 7 };
const stopped: SimStatus = { ...running, running: false, pid: null };

describe('sim messages', () => {
	it('parses sim_status and sim_log, tolerating extra and missing fields', () => {
		const m = parseServer(
			JSON.stringify({ type: 'sim_status', running: true, airframe: 'x500', env: { wind_speed_ms: 3, temperature_c: null }, target: 'pico', gui: false, started_at: 1790000000.5, pid: 4242, ignored: ['temperature_c'] })
		);
		expect(m).toEqual({
			type: 'sim_status',
			sim: { running: true, airframe: 'x500', env: { wind_speed_ms: 3, temperature_c: null }, target: 'pico', gui: false, started_at: 1790000000.5, pid: 4242 }
		});
		expect(parseServer('{"type":"sim_status"}')).toEqual({
			type: 'sim_status',
			sim: { running: false, airframe: null, env: null, target: null, gui: false, started_at: null, pid: null }
		});
		expect(parseServer('{"type":"sim_status","running":false,"started_at":"2026-09-24T12:00:00Z","pid":"x"}')).toMatchObject({ sim: { started_at: '2026-09-24T12:00:00Z', pid: null } });
		expect(parseServer('{"type":"sim_log","line":"[gz] ready","seq":3}')).toEqual({ type: 'sim_log', line: '[gz] ready' });
	});

	it('parses the airframe catalog loosely', () => {
		const list = parseAirframes({ airframes: [{ id: 'x500', specs: { mass_kg: 2, rotors: [{ x: 1, y: 2 }, [3, 4]], extra: 1 } }, { label: 'no id' }, 'junk'] });
		expect(list).toHaveLength(1);
		expect(list[0]).toMatchObject({ id: 'x500', label: 'x500', frame: '', source: '', specs: { mass_kg: 2, rotors: [[1, 2], [3, 4]] } });
		expect(list[0].specs.ixx).toBeNaN();
		expect(parseAirframes([{ id: 'a', specs: { rotors: [[1, 'x']] } }])[0].specs.rotors).toBeUndefined();
		expect(parseAirframes(null)).toEqual([]);
	});

	it('sends the launch request with the direction in [0, 360) and blank optionals as null', () => {
		const env = { ...DEFAULT_ENV, wind_dir_deg: 360, temperature_c: null as unknown as number, pressure_pa: NaN };
		expect(launchMsg('x3', env, false, 'pico')).toEqual({
			type: 'sim_launch',
			airframe: 'x3',
			env: { ...DEFAULT_ENV, wind_dir_deg: 0, temperature_c: null, pressure_pa: null },
			gui: false,
			target: 'pico'
		});
	});

	it('validates the environment', () => {
		expect(envError(DEFAULT_ENV)).toBeNull();
		expect(envError({ ...DEFAULT_ENV, wind_speed_ms: -1 })).toMatch(/wind speed/);
		expect(envError({ ...DEFAULT_ENV, wind_dir_deg: 361 })).toMatch(/direction/);
		expect(envError({ ...DEFAULT_ENV, gust_sigma_ms: null as unknown as number })).toMatch(/gust/);
		expect(envError({ ...DEFAULT_ENV, lat: 91 })).toMatch(/latitude/);
		expect(envError({ ...DEFAULT_ENV, temperature_c: 20, pressure_pa: 101325 })).toBeNull();
		expect(envError({ ...DEFAULT_ENV, pressure_pa: 0 })).toMatch(/pressure/);
	});

	it('enables LAUNCH only with an open backend, no sim running, nothing pending, an airframe and a valid world', () => {
		const ok: LauncherContext = { wsOpen: true, status: stopped, airframe: true, envError: null, pending: false };
		expect(launcherControls(ok)).toEqual({ launch: true, stop: false, why: null });
		expect(launcherControls({ ...ok, status: null }).launch).toBe(true);
		expect(launcherControls({ ...ok, wsOpen: false })).toEqual({ launch: false, stop: false, why: 'backend closed' });
		expect(launcherControls({ ...ok, pending: true }).launch).toBe(false);
		expect(launcherControls({ ...ok, airframe: false }).why).toBe('select an airframe');
		expect(launcherControls({ ...ok, envError: 'latitude bad' }).why).toBe('latitude bad');
		expect(launcherControls({ ...ok, status: running })).toEqual({ launch: false, stop: true, why: 'a sim is running; stop it first' });
		expect(launcherControls({ ...ok, status: running, pending: true }).stop).toBe(false);
		expect(launcherControls({ ...ok, status: running, wsOpen: false }).stop).toBe(false);
	});
});

describe('mock launcher', () => {
	let got: ServerMsg[];
	let fc: MockFc;
	beforeEach(() => {
		vi.useFakeTimers();
		got = [];
		fc = new MockFc(normalizeSchema(fixture as Schema), '');
		fc.onmessage = (ev) => {
			const m = parseServer(ev.data);
			if (m && m.type !== 'telemetry' && m.type !== 'mission_state') got.push(m);
		};
	});
	afterEach(() => {
		fc.close();
		vi.useRealTimers();
	});
	const status = (): SimStatus | undefined => got.filter((m) => m.type === 'sim_status').map((m) => (m as { sim: SimStatus }).sim).at(-1);

	it('launches, refuses a second launch, and stops', () => {
		vi.advanceTimersByTime(100);
		expect(status()?.running).toBe(false);
		fc.send(JSON.stringify(launchMsg('x500', { ...DEFAULT_ENV, wind_speed_ms: 3, temperature_c: 20 }, true, 'sitl')));
		vi.advanceTimersByTime(2000);
		expect(status()).toMatchObject({ running: true, airframe: 'x500', target: 'sitl', gui: true, pid: 4242 });
		expect(got.filter((m) => m.type === 'sim_log').length).toBeGreaterThan(2);
		fc.send(JSON.stringify(launchMsg('x500', DEFAULT_ENV, true, 'sitl')));
		vi.advanceTimersByTime(100);
		expect(got.at(-1)).toEqual({ type: 'error', request: 'sim_launch', error: 'a sim is already running' });
		fc.send(JSON.stringify({ type: 'sim_stop' }));
		vi.advanceTimersByTime(2000);
		expect(status()?.running).toBe(false);
	});

	it('refuses an unknown airframe and a stop with nothing running', () => {
		fc.send(JSON.stringify(launchMsg('nope', DEFAULT_ENV, false, 'sitl')));
		fc.send(JSON.stringify({ type: 'sim_stop' }));
		vi.advanceTimersByTime(100);
		expect(got.filter((m) => m.type === 'error')).toEqual([
			{ type: 'error', request: 'sim_launch', error: 'unknown airframe nope' },
			{ type: 'error', request: 'sim_stop', error: 'no sim is running' }
		]);
	});
});
