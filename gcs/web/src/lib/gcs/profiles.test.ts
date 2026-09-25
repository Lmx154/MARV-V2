import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import fixture from '$lib/fixtures/schema.json';
import { missionStart } from './mission';
import { MockFc } from './mock';
import { profileMsg, profileStatus, profiledRows, reportedProfile } from './profiles';
import { normalizeSchema, parseServer } from './protocol';
import type { MissionStatus, Schema, Telemetry } from './types';

const schema = normalizeSchema(fixture as Schema);
const wps = [{ lat: 47.38, lon: 8.547778, alt_m: 10 }];

describe('profiled params from the schema', () => {
	const rows = profiledRows(schema);

	it('the four profiles in order, and eight profiled params as one row each', () => {
		expect(schema.profiles.map((p) => p.id)).toEqual(['hold', 'freestyle', 'stabilized', 'agile']);
		expect(rows.map((r) => r.id)).toEqual(['cruise_speed', 'acc_xy', 'acc_up', 'acc_dn', 'jerk', 'yaw_rate_auto', 'tilt_max_deg', 'input_tc']);
		for (const r of rows) {
			expect(r.cells.map((c) => c.id)).toEqual(schema.profiles.map((p) => `${r.id}.${p.id}`));
			expect(r.cells.map((c) => c.profile)).toEqual(schema.profiles.map((p) => p.id));
			expect(r.cells.map((c) => c.index)).toEqual([0, 1, 2, 3].map((i) => r.cells[0].index + i));
			const def = schema.families[r.family].kinds[r.kind];
			for (const c of r.cells) expect(def.params).toContain(c);
		}
	});
	it('rows carry the family and kind, the label and unit, and the per-profile defaults', () => {
		const at = (id: string) => rows.find((r) => r.id === id)!;
		const cruise = at('cruise_speed');
		expect(schema.families[cruise.family].id).toBe('guidance');
		expect(schema.families[cruise.family].kinds[cruise.kind].id).toBe('trajectory');
		expect(cruise).toMatchObject({ label: 'Cruise speed', unit: 'm/s' });
		expect(cruise.cells.map((c) => c.default)).toEqual([5, 10, 5, 5]);
		expect(at('input_tc').cells.map((c) => c.default)).toEqual([0.1, 0.05, 0.2, 0.1]);
		expect(schema.families[at('tilt_max_deg').family].id).toBe('controller');
	});
	it('shared params are not rows; a schema without profiles has none', () => {
		expect(rows.some((r) => r.id === 'xy_vel_max')).toBe(false);
		expect(profiledRows({ ...schema, profiles: [] })).toEqual([]);
		expect(normalizeSchema({ ...schema, profiles: undefined } as unknown as Schema).profiles).toEqual([]);
	});
});

describe('profile selector', () => {
	it('sends the profile id', () => {
		expect(JSON.parse(JSON.stringify(profileMsg('stabilized')))).toEqual({ type: 'profile', profile: 'stabilized' });
		expect(schema.profiles.map((p) => profileMsg(p.id).profile)).toEqual(['hold', 'freestyle', 'stabilized', 'agile']);
	});
	it('mission_start carries the selected profile only when one is selected', () => {
		expect(JSON.parse(JSON.stringify(missionStart(wps, null, 'agile')))).toEqual({ type: 'mission_start', waypoints: wps, profile: 'agile' });
		expect(JSON.parse(JSON.stringify(missionStart(wps, 7.5, 'hold')))).toEqual({ type: 'mission_start', waypoints: wps, speed_mps: 7.5, profile: 'hold' });
		expect(JSON.parse(JSON.stringify(missionStart(wps, null)))).toEqual({ type: 'mission_start', waypoints: wps });
		expect(JSON.parse(JSON.stringify(missionStart(wps, null, null)))).toEqual({ type: 'mission_start', waypoints: wps });
	});
});

describe('requested vs reported profile', () => {
	const telem = (profile?: string): Telemetry => ({ p_ned: [0, 0, 0], q: [1, 0, 0, 0], preset: 0, armed: false, thrust_hover: 0, brake: 0, home: null, geo: null, motor: null, ...(profile === undefined ? {} : { profile }) });
	const mission = (profile?: string): MissionStatus => ({ state: 'hold', wp_index: -1, wp_count: 0, target: null, dist_m: null, climb_alt_m: null, home: null, reason: '', ...(profile === undefined ? {} : { profile }) });

	it('the FC-reported profile: telemetry, else mission_state, else none', () => {
		expect(reportedProfile(telem('freestyle'), mission('stabilized'))).toBe('freestyle');
		expect(reportedProfile(telem('hold'), mission('stabilized'))).toBe('hold');
		expect(reportedProfile(telem(), mission('stabilized'))).toBe('stabilized');
		expect(reportedProfile(null, mission())).toBeNull();
		expect(reportedProfile(null, null)).toBeNull();
	});
	it('labels both and flags a mismatch only when both are known and differ', () => {
		const p = schema.profiles;
		expect(profileStatus(p, 'freestyle', 'freestyle')).toEqual({ requested: 'Freestyle', reported: 'Freestyle', mismatch: false });
		expect(profileStatus(p, 'agile', 'hold')).toEqual({ requested: 'Agile', reported: 'Hold', mismatch: true });
		expect(profileStatus(p, null, 'stabilized')).toEqual({ requested: '—', reported: 'Stabilized', mismatch: false });
		expect(profileStatus(p, 'stabilized', null)).toEqual({ requested: 'Stabilized', reported: '—', mismatch: false });
		expect(profileStatus(p, 'freestyle', '#7')).toEqual({ requested: 'Freestyle', reported: '#7', mismatch: true });
	});
	it('parses profile from telemetry and mission_state; absent or invalid leaves it unset', () => {
		const t = (profile: unknown) => {
			const m = parseServer(JSON.stringify({ type: 'telemetry', est: { p_ned: [0, 0, 0], q: [1, 0, 0, 0] }, profile }));
			return m?.type === 'telemetry' ? m.telemetry.profile : 'no telemetry';
		};
		const s = (profile: unknown) => {
			const m = parseServer(JSON.stringify({ type: 'mission_state', state: 'hold', profile }));
			return m?.type === 'mission_state' ? m.mission.profile : 'no mission_state';
		};
		expect([t('stabilized'), t('hold'), t('#7'), t(undefined), t(2), t(''), t(null)]).toEqual(['stabilized', 'hold', '#7', undefined, undefined, undefined, undefined]);
		expect([s('agile'), s('#9'), s(undefined), s(null), s(3)]).toEqual(['agile', '#9', undefined, undefined, undefined]);
	});
});

describe('mock profile', () => {
	beforeEach(() => vi.useFakeTimers());
	afterEach(() => vi.useRealTimers());

	const run = () => {
		const fc = new MockFc(schema, '');
		const telem: Telemetry[] = [];
		const states: MissionStatus[] = [];
		const errors: { request: string; error: string }[] = [];
		fc.onmessage = (ev) => {
			const m = parseServer(ev.data);
			if (m?.type === 'telemetry') telem.push(m.telemetry);
			if (m?.type === 'mission_state') states.push(m.mission);
			if (m?.type === 'error') errors.push({ request: m.request, error: m.error });
		};
		const send = (m: object, ms = 100): void => {
			fc.send(JSON.stringify(m));
			vi.advanceTimersByTime(ms);
		};
		vi.advanceTimersByTime(100);
		return { fc, telem, states, errors, send };
	};

	it('echoes the selected profile id in telemetry and mission_state in any state; arm selects hold', () => {
		const { fc, telem, states, errors, send } = run();
		expect(telem.at(-1)?.profile).toBe('hold');
		send({ type: 'profile', profile: 'stabilized' });
		expect(states.at(-1)).toMatchObject({ state: 'disarmed', profile: 'stabilized' });
		expect(telem.at(-1)?.profile).toBe('stabilized');
		expect(errors).toEqual([]);
		send({ type: 'profile', profile: 'agile' });
		send({ type: 'arm' });
		expect(states.at(-1)).toMatchObject({ state: 'armed', profile: 'hold' });
		send({ type: 'climb', alt_m: 5 }, 5000);
		send({ type: 'profile', profile: 'freestyle' });
		expect(states.at(-1)).toMatchObject({ state: 'hold', profile: 'freestyle' });
		expect(telem.at(-1)?.profile).toBe('freestyle');
		send({ type: 'mission_start', waypoints: wps, profile: 'agile' });
		expect(states.at(-1)).toMatchObject({ state: 'mission', profile: 'agile' });
		expect(errors).toEqual([]);
		fc.close();
	});

	it('refuses a numeric or unknown profile as the backend does, keeping the active one', () => {
		const { fc, telem, states, errors, send } = run();
		send({ type: 'profile', profile: 'freestyle' });
		const want = 'want {profile: hold|freestyle|stabilized|agile}';
		send({ type: 'profile', profile: 2 });
		send({ type: 'profile', profile: 'sport' });
		send({ type: 'profile' });
		expect(errors).toEqual([
			{ request: 'profile', error: want },
			{ request: 'profile', error: want },
			{ request: 'profile', error: want }
		]);
		expect(telem.at(-1)?.profile).toBe('freestyle');
		send({ type: 'arm' });
		send({ type: 'climb', alt_m: 5 }, 5000);
		send({ type: 'mission_start', waypoints: wps, profile: 3 });
		expect(errors.at(-1)).toEqual({ request: 'mission_start', error: 'want {waypoints: [{lat, lon, alt_m}], speed_mps?: number, profile?: hold|freestyle|stabilized|agile}' });
		expect(states.at(-1)).toMatchObject({ state: 'hold', profile: 'hold' });
		fc.close();
	});

	it("flies a long leg at the profile's cruise speed and horizontal acceleration", () => {
		const leg = (profile: string) => {
			const { fc, telem, send } = run();
			send({ type: 'arm' });
			send({ type: 'climb', alt_m: 10 }, 8000);
			const n = telem.length;
			send({ type: 'mission_start', waypoints: wps, profile }, 30_000);
			fc.close();
			const v = telem.slice(n + 1).map((t, i) => Math.hypot(t.p_ned[0] - telem[n + i].p_ned[0], t.p_ned[1] - telem[n + i].p_ned[1]) / 0.05);
			const a = v.slice(1).map((x, i) => Math.abs(x - v[i]) / 0.05);
			return { v: Math.max(...v), a: Math.max(...a) };
		};
		const hold = leg('hold');
		const freestyle = leg('freestyle');
		const stabilized = leg('stabilized');
		expect(hold.v).toBeCloseTo(5, 1);
		expect(freestyle.v).toBeCloseTo(10, 1);
		expect(hold.a).toBeLessThanOrEqual(3 + 1e-3);
		expect(freestyle.a).toBeGreaterThan(3 + 1e-3);
		expect(freestyle.a).toBeLessThanOrEqual(5 + 1e-3);
		expect(stabilized.a).toBeLessThanOrEqual(2.5 + 1e-3);
	});
});
