import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import fixture from '$lib/fixtures/schema.json';
import {
	MAX_WAYPOINTS,
	PRESET_KEY,
	climbError,
	disarmNeedsConfirm,
	enabledControls,
	exportPresets,
	importPresets,
	isMissionRequest,
	missionError,
	moveWaypoint,
	parseWaypoint,
	propsSpinning,
	rangeFromHome,
	readPresets,
	reasonText,
	upsertPreset,
	waypointError,
	writePresets,
	type MissionPreset
} from './mission';
import { MockFc } from './mock';
import { normalizeSchema, parseServer } from './protocol';
import type { LatLonAlt, MissionMode, MissionStatus, Schema, ServerMsg, Telemetry } from './types';

const wp = (lat: number, lon: number, alt_m: number): LatLonAlt => ({ lat, lon, alt_m });
const route = [wp(47.3765, 8.5478, 10), wp(47.3766, 8.5479, 15)];

describe('waypoint validation', () => {
	it('accepts in-range coordinates with an altitude in 0.5..200 m', () => {
		expect(waypointError(wp(47.37, 8.54, 10))).toBeNull();
		expect(waypointError(wp(-90, 180, 0.5))).toBeNull();
		expect(waypointError(wp(-90, 180, 200))).toBeNull();
	});
	it('bounds: altitude 0.5..200 m, within 5 km of home, 1..64 waypoints, climb 0.5..200 m', () => {
		expect(waypointError(wp(47.37, 8.54, 0.4))).toBe('altitude must be within 0.5..200 m above home');
		expect(waypointError(wp(47.37, 8.54, 200.1))).toBe('altitude must be within 0.5..200 m above home');
		const home = { lat: 47.376388, lon: 8.547778 };
		// 1 deg of latitude is ~111.2 km here: 0.044 deg is ~4.89 km, 0.046 deg ~5.11 km.
		expect(rangeFromHome({ lat: home.lat + 0.044, lon: home.lon }, home)).toBeCloseTo(4893, -1);
		expect(waypointError(wp(home.lat + 0.044, home.lon, 10), home)).toBeNull();
		expect(waypointError(wp(home.lat + 0.046, home.lon, 10), home)).toBe('5.11 km from home; at most 5 km');
		expect(waypointError(wp(home.lat + 0.046, home.lon, 10))).toBeNull(); // no home, no range check
		const many = Array.from({ length: MAX_WAYPOINTS }, () => wp(47.37, 8.54, 10));
		expect(missionError(many)).toBeNull();
		expect(missionError([...many, wp(47.37, 8.54, 10)])).toBe('65 waypoints; at most 64');
		expect(missionError([route[0], wp(home.lat, home.lon + 0.1, 10)], home)).toMatch(/^waypoint 2: 7\.\d\d km from home; at most 5 km$/);
		expect(climbError(0.5)).toBeNull();
		expect(climbError(200)).toBeNull();
		expect(climbError(0.4)).toBe('safe altitude must be within 0.5..200 m');
		expect(climbError(201)).toBe('safe altitude must be within 0.5..200 m');
	});
	it('rejects out-of-range, non-finite and non-positive values', () => {
		expect(waypointError(wp(90.1, 0, 10))).toMatch(/latitude/);
		expect(waypointError(wp(0, -180.5, 10))).toMatch(/longitude/);
		expect(waypointError(wp(0, 0, 0))).toMatch(/altitude/);
		expect(waypointError(wp(0, 0, -3))).toMatch(/altitude/);
		expect(waypointError(wp(NaN, 0, 10))).toMatch(/latitude/);
	});
	it('parses typed decimal degrees, and says why not', () => {
		expect(parseWaypoint(' 47.3765', '-8.5', '12.5')).toEqual(wp(47.3765, -8.5, 12.5));
		expect(parseWaypoint('47,3', '8', '10')).toMatch(/latitude/);
		expect(parseWaypoint('47', '', '10')).toMatch(/longitude/);
		expect(parseWaypoint('47', '8', '0')).toMatch(/altitude/);
	});
	it('a mission needs at least one waypoint, each valid', () => {
		expect(missionError([])).toBe('no waypoints');
		expect(missionError(route)).toBeNull();
		expect(missionError([route[0], wp(0, 0, 0)])).toMatch(/^waypoint 2: altitude/);
	});
	it('moves rows within the list', () => {
		expect(moveWaypoint(route, 1, -1)).toEqual([route[1], route[0]]);
		expect(moveWaypoint(route, 1, 1)).toEqual(route);
	});
});

describe('presets', () => {
	const presets: MissionPreset[] = [
		{ name: 'square', waypoints: route },
		{ name: 'one', waypoints: [route[0]] }
	];
	const memory = () => {
		const m = new Map<string, string>();
		return { getItem: (k: string) => m.get(k) ?? null, setItem: (k: string, v: string) => void m.set(k, v), m };
	};

	it('save and load round trip through storage', () => {
		const s = memory();
		expect(writePresets(s, presets)).toBeNull();
		expect(readPresets(s)).toEqual(presets);
	});
	it('storage failures do not throw', () => {
		const broken = {
			getItem: () => {
				throw new Error('denied');
			},
			setItem: () => {
				throw new Error('quota');
			}
		};
		expect(readPresets(broken)).toEqual([]);
		expect(writePresets(broken, presets)).toBe('quota');
		expect(readPresets(null)).toEqual([]);
		const s = memory();
		s.m.set(PRESET_KEY, '{not json');
		expect(readPresets(s)).toEqual([]);
	});
	it('export then import round trips, through JSON text', () => {
		const back = importPresets(JSON.parse(JSON.stringify(exportPresets(presets))));
		expect(back).toEqual({ presets, rejected: [] });
	});
	it('import takes a bare list or one preset and names what it rejects', () => {
		expect(importPresets(presets).presets).toEqual(presets);
		expect(importPresets(presets[0]).presets).toEqual([presets[0]]);
		const r = importPresets([presets[0], { name: 'bad', waypoints: [wp(0, 0, -1)] }, { waypoints: [] }, 7]);
		expect(r.presets).toEqual([presets[0]]);
		expect(r.rejected).toEqual(['bad', '#3', '#4']);
	});
	it('saving under an existing name replaces it', () => {
		const next = upsertPreset(presets, { name: 'one', waypoints: route });
		expect(next.map((p) => p.name)).toEqual(['square', 'one']);
		expect(next[1].waypoints).toEqual(route);
		expect(upsertPreset(presets, { name: 'new', waypoints: route })).toHaveLength(3);
	});
});

describe('control enable rules', () => {
	const on = (state: MissionMode | null, over: Partial<Parameters<typeof enabledControls>[0]> = {}) =>
		Object.entries(enabledControls({ state, connected: true, climbAlt: 10, waypoints: route, home: null, ...over }))
			.filter(([, v]) => v)
			.map(([k]) => k)
			.sort();

	it('per mission state', () => {
		expect(on('disarmed')).toEqual(['arm']);
		expect(on('armed')).toEqual(['climb', 'disarm']);
		expect(on('climb')).toEqual(['disarm', 'land', 'rth']);
		expect(on('hold')).toEqual(['climb', 'disarm', 'land', 'mission_start', 'rth']);
		expect(on('mission')).toEqual(['disarm', 'land', 'rth']);
		expect(on('rth')).toEqual(['disarm', 'land']);
		expect(on('land')).toEqual(['disarm', 'rth']);
	});
	it('nothing without a state or a connection; climb and start need valid inputs', () => {
		expect(on(null)).toEqual([]);
		expect(on('hold', { connected: false })).toEqual([]);
		expect(on('armed', { climbAlt: 0 })).toEqual(['disarm']);
		expect(on('armed', { climbAlt: NaN })).toEqual(['disarm']);
		expect(on('hold', { waypoints: [] })).not.toContain('mission_start');
		expect(on('hold', { waypoints: [wp(0, 0, 0)] })).not.toContain('mission_start');
		expect(on('armed', { climbAlt: 250 })).toEqual(['disarm']);
		expect(on('hold', { home: { lat: 47.3765, lon: 8.65 } })).not.toContain('mission_start');
		expect(on('hold', { home: { lat: 47.3765, lon: 8.5478 } })).toContain('mission_start');
	});
	it('disarm asks first when above 0.5 m or flying', () => {
		expect(disarmNeedsConfirm('armed', 0.2)).toBe(false);
		expect(disarmNeedsConfirm('disarmed', 0)).toBe(false);
		expect(disarmNeedsConfirm('armed', 0.6)).toBe(true);
		expect(disarmNeedsConfirm('armed', NaN)).toBe(true);
		expect(disarmNeedsConfirm('hold', 0)).toBe(true);
		expect(disarmNeedsConfirm('land', 0.1)).toBe(true);
	});
	it('names the mission requests', () => {
		expect(['arm', 'disarm', 'climb', 'mission_start', 'rth', 'land'].every(isMissionRequest)).toBe(true);
		expect(isMissionRequest('save')).toBe(false);
	});
});

describe('message parsing', () => {
	it('mission_state, tolerating extra fields', () => {
		const m = parseServer(
			JSON.stringify({ type: 'mission_state', state: 'mission', wp_index: 1, wp_count: 3, target: { lat: 47.1, lon: 8.2, alt_m: 20, x: 1 }, dist_m: 12.5, climb_alt_m: 20, extra: true })
		);
		expect(m).toEqual({
			type: 'mission_state',
			mission: { state: 'mission', wp_index: 1, wp_count: 3, target: { lat: 47.1, lon: 8.2, alt_m: 20 }, dist_m: 12.5, climb_alt_m: 20, home: null, reason: '' }
		});
	});
	it('mission_state with null target and distances; an unknown state is ignored', () => {
		const m = parseServer(JSON.stringify({ type: 'mission_state', state: 'armed', wp_index: -1, wp_count: 0, target: null, dist_m: null, climb_alt_m: null, home: null, reason: '' }));
		expect(m).toEqual({ type: 'mission_state', mission: { state: 'armed', wp_index: -1, wp_count: 0, target: null, dist_m: null, climb_alt_m: null, home: null, reason: '' } });
		expect(parseServer(JSON.stringify({ type: 'mission_state', state: 'orbit' }))).toBeNull();
	});
	it('mission_state home and reason; wp_index defaults to -1 (none)', () => {
		const m = parseServer(JSON.stringify({ type: 'mission_state', state: 'disarmed', wp_count: 0, home: { lat: 47.3, lon: 8.5 }, reason: 'landed' }));
		expect(m?.type === 'mission_state' && m.mission).toMatchObject({ wp_index: -1, home: { lat: 47.3, lon: 8.5 }, reason: 'landed' });
		expect(reasonText(m?.type === 'mission_state' ? m.mission : null)).toBe('landed');
		const s = parseServer(JSON.stringify({ type: 'mission_state', state: 'hold', reason: 'telemetry stale', home: { lat: 'x' } }));
		expect(s?.type === 'mission_state' && s.mission.home).toBeNull();
		expect(reasonText(s?.type === 'mission_state' ? s.mission : null)).toBe('telemetry stale');
		const n = parseServer(JSON.stringify({ type: 'mission_state', state: 'hold', reason: 7 }));
		expect(n?.type === 'mission_state' && n.mission.reason).toBe('');
		expect(reasonText(n?.type === 'mission_state' ? n.mission : null)).toBeNull();
		expect(reasonText(null)).toBeNull();
	});
	it('telemetry motor[4] and the props-spinning indicator', () => {
		const tel = (o: object): Telemetry | null => {
			const t = parseServer(JSON.stringify({ type: 'telemetry', est: { p_ned: [0, 0, 0], q: [1, 0, 0, 0] }, ...o }));
			return t?.type === 'telemetry' ? t.telemetry : null;
		};
		expect(tel({ armed: true, motor: [0.1, 0.1, 0.1, 0.1] })?.motor).toEqual([0.1, 0.1, 0.1, 0.1]);
		expect(tel({ armed: true })?.motor).toBeNull();
		expect(tel({ armed: true, motor: [0.1, 0.1] })?.motor).toBeNull();
		expect(propsSpinning(tel({ armed: true, motor: [0, 0, 0.1, 0] }))).toBe(true);
		expect(propsSpinning(tel({ armed: true, motor: [0, 0, 0, 0] }))).toBe(false);
		expect(propsSpinning(tel({ armed: false, motor: [0.5, 0.5, 0.5, 0.5] }))).toBe(false);
		expect(propsSpinning(tel({ armed: true }))).toBe(false);
		expect(propsSpinning(tel({ armed: true, motor: [null, 'x', 0, 0] }))).toBe(false);
		expect(propsSpinning(null)).toBe(false);
	});
	it('telemetry carries geo and a valid home', () => {
		const base = { type: 'telemetry', armed: true, est: { p_ned: [1, 2, -3], q: [1, 0, 0, 0] } };
		const t = parseServer(JSON.stringify({ ...base, geo: { lat: 47.2, lon: 8.3, alt_m: 3 }, home_valid: true, home: { lat_e7: 472000000, lon_e7: 83000000, alt_m: 408 } }));
		expect(t?.type === 'telemetry' && t.telemetry.geo).toEqual({ lat: 47.2, lon: 8.3, alt_m: 3 });
		expect(t?.type === 'telemetry' && t.telemetry.home).toEqual({ lat_e7: 472000000, lon_e7: 83000000, alt_m: 408 });
		const n = parseServer(JSON.stringify({ ...base, home_valid: false, home: { lat_e7: 1, lon_e7: 2, alt_m: 0 } }));
		expect(n?.type === 'telemetry' && n.telemetry.home).toBeNull();
		expect(n?.type === 'telemetry' && n.telemetry.geo).toBeNull();
	});
	it('mission errors keep their request', () => {
		expect(parseServer(JSON.stringify({ type: 'error', request: 'climb', error: 'refused' }))).toEqual({ type: 'error', request: 'climb', error: 'refused' });
	});
});

describe('mock mission', () => {
	beforeEach(() => vi.useFakeTimers());
	afterEach(() => vi.useRealTimers());

	const start = () => {
		const fc = new MockFc(normalizeSchema(fixture as Schema), '');
		const states: MissionStatus[] = [];
		const all: MissionStatus[] = [];
		const errors: Extract<ServerMsg, { type: 'error' }>[] = [];
		const motors: number[][] = [];
		fc.onmessage = (ev) => {
			const m = parseServer(ev.data);
			if (m?.type === 'mission_state') {
				all.push(m.mission);
				if (states.at(-1)?.state !== m.mission.state) states.push(m.mission);
			}
			if (m?.type === 'error') errors.push(m);
			if (m?.type === 'telemetry' && m.telemetry.motor) motors.push(m.telemetry.motor);
		};
		const send = (m: object, ms = 100): void => {
			fc.send(JSON.stringify(m));
			vi.advanceTimersByTime(ms);
		};
		vi.advanceTimersByTime(100);
		return { fc, states, all, errors, motors, send };
	};

	it('flies the waypoints, returns home and holds there (no auto-land); a manual land lands and disarms', () => {
		const { fc, states, all, errors, motors, send } = start();
		send({ type: 'mission_start', waypoints: route });
		expect(errors.map((e) => e.request)).toEqual(['mission_start']); // refused while disarmed
		expect(motors.at(-1)).toEqual([0, 0, 0, 0]);
		send({ type: 'arm' }, 1000);
		expect(motors.at(-1)).toEqual([0.1, 0.1, 0.1, 0.1]);
		expect(states.at(-1)?.home).not.toBeNull();
		send({ type: 'climb', alt_m: 5 }, 5000);
		expect(states.at(-1)).toMatchObject({ state: 'hold', reason: 'altitude reached', wp_index: -1 });
		send({ type: 'mission_start', waypoints: route }, 500);
		expect(all.at(-1)).toMatchObject({ state: 'mission', wp_index: 0, wp_count: 2 });
		vi.advanceTimersByTime(120_000);
		expect(states.map((s) => s.state)).toEqual(['disarmed', 'armed', 'climb', 'hold', 'mission', 'rth', 'hold']);
		expect(states[5]).toMatchObject({ reason: 'mission complete', wp_index: -1 });
		expect(states[6].reason).toBe('home reached');
		expect(all.some((s) => s.state === 'mission' && s.wp_index === 1)).toBe(true);
		const n = all.length;
		vi.advanceTimersByTime(2000);
		expect(all.length - n).toBeGreaterThanOrEqual(3); // every 500 ms while engaged
		expect(all.at(-1)?.state).toBe('hold');
		expect(all.at(-1)?.dist_m).toBeLessThan(0.3);
		send({ type: 'land' }, 60_000);
		expect(states.map((s) => s.state).slice(-2)).toEqual(['land', 'disarmed']);
		expect(states.at(-1)?.reason).toBe('landed');
		expect(motors.at(-1)).toEqual([0, 0, 0, 0]);
		const m = all.length;
		vi.advanceTimersByTime(2000);
		expect(all.length).toBe(m); // disarmed: only on change
		fc.close();
		expect(errors).toHaveLength(1);
	});

	it('refuses what the state does not allow, as errors', () => {
		const { fc, states, errors, send } = start();
		send({ type: 'disarm' });
		send({ type: 'rth' });
		send({ type: 'land' });
		send({ type: 'climb', alt_m: 5 });
		send({ type: 'arm' });
		send({ type: 'arm' });
		send({ type: 'mission_start', waypoints: route });
		send({ type: 'rth' });
		send({ type: 'climb', alt_m: 250 });
		send({ type: 'climb', alt_m: 5 }, 5000);
		send({ type: 'mission_start', waypoints: [wp(47.5, 8.5478, 10)] });
		send({ type: 'mission_start', waypoints: [] });
		expect(errors.map((e) => e.request)).toEqual(['disarm', 'rth', 'land', 'climb', 'arm', 'mission_start', 'rth', 'climb', 'mission_start', 'mission_start']);
		expect(errors.every((e) => e.error.startsWith('refused in '))).toBe(true);
		expect(states.at(-1)?.state).toBe('hold');
		send({ type: 'land' }, 500);
		send({ type: 'rth' }, 40); // rth is allowed from land
		expect(states.at(-1)?.state).toBe('rth');
		send({ type: 'land' }, 500); // and land from rth
		expect(states.at(-1)?.state).toBe('land');
		send({ type: 'disarm' });
		expect(states.at(-1)?.state).toBe('disarmed');
		fc.close();
	});
});
