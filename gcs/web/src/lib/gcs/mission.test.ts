import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import fixture from '$lib/fixtures/schema.json';
import {
	PRESET_KEY,
	disarmNeedsConfirm,
	enabledControls,
	exportPresets,
	importPresets,
	isMissionRequest,
	missionError,
	moveWaypoint,
	parseWaypoint,
	readPresets,
	upsertPreset,
	waypointError,
	writePresets,
	type MissionPreset
} from './mission';
import { MockFc } from './mock';
import { normalizeSchema, parseServer } from './protocol';
import type { LatLonAlt, MissionMode, MissionStatus, Schema, ServerMsg } from './types';

const wp = (lat: number, lon: number, alt_m: number): LatLonAlt => ({ lat, lon, alt_m });
const route = [wp(47.3765, 8.5478, 10), wp(47.3766, 8.5479, 15)];

describe('waypoint validation', () => {
	it('accepts in-range coordinates with a positive altitude', () => {
		expect(waypointError(wp(47.37, 8.54, 10))).toBeNull();
		expect(waypointError(wp(-90, 180, 0.1))).toBeNull();
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
		Object.entries(enabledControls({ state, connected: true, climbAlt: 10, waypoints: route, ...over }))
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
		expect(on('land')).toEqual(['disarm']);
	});
	it('nothing without a state or a connection; climb and start need valid inputs', () => {
		expect(on(null)).toEqual([]);
		expect(on('hold', { connected: false })).toEqual([]);
		expect(on('armed', { climbAlt: 0 })).toEqual(['disarm']);
		expect(on('armed', { climbAlt: NaN })).toEqual(['disarm']);
		expect(on('hold', { waypoints: [] })).not.toContain('mission_start');
		expect(on('hold', { waypoints: [wp(0, 0, 0)] })).not.toContain('mission_start');
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
		expect(m).toEqual({ type: 'mission_state', mission: { state: 'mission', wp_index: 1, wp_count: 3, target: { lat: 47.1, lon: 8.2, alt_m: 20 }, dist_m: 12.5, climb_alt_m: 20 } });
	});
	it('mission_state with null target and distances; an unknown state is ignored', () => {
		const m = parseServer(JSON.stringify({ type: 'mission_state', state: 'armed', wp_index: 0, wp_count: 0, target: null, dist_m: null, climb_alt_m: null }));
		expect(m).toEqual({ type: 'mission_state', mission: { state: 'armed', wp_index: 0, wp_count: 0, target: null, dist_m: null, climb_alt_m: null } });
		expect(parseServer(JSON.stringify({ type: 'mission_state', state: 'orbit' }))).toBeNull();
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

	it('arms, climbs, holds, flies the waypoints, returns home, lands and disarms', () => {
		const fc = new MockFc(normalizeSchema(fixture as Schema), '');
		const states: MissionStatus[] = [];
		const errors: ServerMsg[] = [];
		fc.onmessage = (ev) => {
			const m = parseServer(ev.data);
			if (m?.type === 'mission_state' && states.at(-1)?.state !== m.mission.state) states.push(m.mission);
			if (m?.type === 'error') errors.push(m);
		};
		vi.advanceTimersByTime(100);
		fc.send(JSON.stringify({ type: 'mission_start', waypoints: route }));
		vi.advanceTimersByTime(100);
		expect(errors).toHaveLength(1); // refused while disarmed
		fc.send(JSON.stringify({ type: 'arm' }));
		vi.advanceTimersByTime(1000);
		fc.send(JSON.stringify({ type: 'climb', alt_m: 5 }));
		vi.advanceTimersByTime(5000);
		expect(states.at(-1)?.state).toBe('hold');
		fc.send(JSON.stringify({ type: 'mission_start', waypoints: route }));
		vi.advanceTimersByTime(120_000);
		fc.close();
		expect(states.map((s) => s.state)).toEqual(['disarmed', 'armed', 'climb', 'hold', 'mission', 'rth', 'land', 'disarmed']);
		expect(errors).toHaveLength(1);
	});
});
