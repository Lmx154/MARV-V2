import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import fixture from '$lib/fixtures/schema.json';
import { MockFc } from './mock';
import { MOCK_RADIOMASTER, MOCK_XBOX, MockRadioStore, mockDefault } from './mock-radio';
import { normalizeSchema, parseServer } from './protocol';
import {
	axisValue,
	bandIndex,
	calCentre,
	calCentrePhase,
	calFinish,
	calSample,
	calStart,
	evenBands,
	insertBand,
	makeRadioApi,
	moveBand,
	normalizeStick,
	parseConfig,
	removeBand,
	sameConfig,
	setBandProfile,
	setUpper,
	toWire,
	validateConfig,
	type Band,
	type FetchFn,
	type RadioConfig
} from './radio';
import type { Schema, ServerMsg } from './types';

const IDS = ['hold', 'freestyle', 'stabilized', 'agile'];
const four: Band[] = [
	{ upper: -0.5, profile: 'hold' },
	{ upper: 0, profile: 'freestyle' },
	{ upper: 0.5, profile: 'stabilized' },
	{ upper: 1, profile: 'agile' }
];
const ascending = (b: Band[]): boolean => b.every((x, i) => i === 0 || x.upper > b[i - 1].upper) && b[b.length - 1].upper === 1;
const rm = (): RadioConfig => mockDefault(MOCK_RADIOMASTER.name);

describe('bands', () => {
	it('splits, removes, reorders and moves edges keeping them ascending with the last at 1', () => {
		const split = insertBand(four, 1);
		expect(split.map((b) => b.upper)).toEqual([-0.5, -0.25, 0, 0.5, 1]);
		expect(split[1].profile).toBe('freestyle');
		const six = insertBand(split, 4);
		expect(six).toHaveLength(6);
		expect(insertBand(six, 0)).toBe(six);
		expect(ascending(six)).toBe(true);

		const dropLast = removeBand(four, 3);
		expect(dropLast.map((b) => b.upper)).toEqual([-0.5, 0, 1]);
		expect(dropLast.map((b) => b.profile)).toEqual(['hold', 'freestyle', 'stabilized']);
		const dropFirst = removeBand(four, 0);
		expect(dropFirst.map((b) => b.upper)).toEqual([0, 0.5, 1]);
		const two = removeBand(dropLast, 1);
		expect(removeBand(two, 0)).toBe(two);

		const up = moveBand(four, 1, -1);
		expect(up.map((b) => b.profile)).toEqual(['freestyle', 'hold', 'stabilized', 'agile']);
		expect(up.map((b) => b.upper)).toEqual(four.map((b) => b.upper));
		expect(moveBand(four, 3, 1)).toBe(four);
		expect(four[0].profile).toBe('hold');

		expect(setUpper(four, 1, 0.8).map((b) => b.upper)).toEqual([-0.5, 0.49, 0.5, 1]);
		expect(setUpper(four, 1, -0.9)[1].upper).toBe(-0.49);
		expect(setUpper(four, 3, 0.2)).toBe(four);
		expect(setBandProfile(four, 2, 'agile')[2].profile).toBe('agile');

		const even = evenBands(3, IDS);
		expect(even.map((b) => b.upper)).toEqual([-0.33, 0.33, 1]);
		expect(evenBands(9, ['hold'])).toHaveLength(6);
		expect(ascending(evenBands(5, IDS))).toBe(true);
	});

	it('highlights the band the switch is in', () => {
		expect(bandIndex(four, axisValue(-32767))).toBe(0);
		expect(bandIndex(four, axisValue(-16500))).toBe(0);
		expect(bandIndex(four, axisValue(-16300))).toBe(1);
		expect(bandIndex(four, axisValue(-10923))).toBe(1);
		expect(bandIndex(four, axisValue(0))).toBe(2);
		expect(bandIndex(four, axisValue(10923))).toBe(2);
		expect(bandIndex(four, axisValue(16384))).toBe(3);
		expect(bandIndex(four, axisValue(32767))).toBe(3);
		expect(bandIndex(four, axisValue(-32768))).toBe(0);
		expect(bandIndex(four, NaN)).toBe(-1);
		expect(bandIndex([], 0)).toBe(-1);
	});
});

describe('calibration', () => {
	it('captures min and max from the extremes and the mean centre', () => {
		let c = calStart(4);
		for (const s of [
			[0, 0, -30000, 0],
			[-31000, 29000, -32000, 100],
			[30500, -28000, 32100, -100],
			[100, 0, 0]
		])
			c = calSample(c, s);
		expect(c.min).toEqual([-31000, -28000, -32000, -100]);
		expect(c.max).toEqual([30500, 29000, 32100, 100]);
		c = calCentrePhase(c);
		c = calSample(c, [10, -20, 1, 5]);
		c = calSample(c, [30, -40, 3, 99999]);
		expect(c.min[0]).toBe(-31000);
		expect(calCentre(c, 0)).toBe(20);
		expect(calCentre(c, 1)).toBe(-30);
		expect(calCentre(c, 3)).toBe(100);

		const base = rm();
		const { config, errors } = calFinish(c, base);
		expect(config.sticks.roll).toMatchObject({ axis: 0, min: -31000, center: 20, max: 30500 });
		expect(config.sticks.throttle).toMatchObject({ min: -32000, center: 2, max: 32100 });
		expect(config.sticks.yaw).toEqual(base.sticks.yaw);
		expect(errors).toEqual(['yaw (axis 3) did not move far enough: move it to both ends']);
		expect(base.sticks.roll.min).toBe(-32767);
	});

	it('names a stick with no centre sample', () => {
		let c = calSample(calStart(4), [-30000, -30000, -30000, -30000]);
		c = calCentrePhase(calSample(c, [30000, 30000, 30000, 30000]));
		expect(calFinish(c, rm()).errors).toHaveLength(4);
		expect(calFinish(c, rm()).errors[0]).toContain('no centre');
	});

	it('normalizes a stick through its calibration, reverse and deadband', () => {
		const s = { axis: 0, min: -30000, center: 1000, max: 32000, reverse: false, deadband: 0.1 };
		expect(normalizeStick(1000, s)).toBe(0);
		expect(normalizeStick(32000, s)).toBe(1);
		expect(normalizeStick(-30000, s)).toBe(-1);
		expect(normalizeStick(1000 + 0.55 * 31000, s)).toBeCloseTo(0.5, 6);
		expect(normalizeStick(32000, { ...s, reverse: true })).toBe(-1);
	});
});

describe('config validation', () => {
	it('accepts the two defaults', () => {
		expect(validateConfig(rm(), IDS, 8, 2)).toEqual([]);
		expect(validateConfig(mockDefault(MOCK_XBOX.name), IDS, 8, 11)).toEqual([]);
	});

	it('refuses band counts, order, the last edge and unknown profiles', () => {
		const c = rm();
		const at = (bands: Band[]): string[] => validateConfig({ ...c, profile: { ...c.profile, bands } }, IDS, 8, 2);
		expect(at(four.slice(3))).toContain('flight modes: 2 to 6 bands, not 1');
		expect(at([...evenBands(6, IDS), { upper: 1, profile: 'hold' }]).some((e) => e.includes('not 7'))).toBe(true);
		expect(at([four[1], four[0], four[2], four[3]])).toContain("flight modes: band 2's upper edge must be above band 1's");
		expect(at([four[0], four[1], four[2], { upper: 0.9, profile: 'agile' }])).toContain('flight modes: the last band must end at 1.0');
		expect(at([four[0], { upper: 1, profile: 'sport' }])).toContain('flight modes: band 2 has no known profile');
		expect(at([{ upper: -1, profile: 'hold' }, four[3]])).toContain("flight modes: band 1's upper edge must be in (-1, 1]");
	});

	it('refuses axes, buttons and stick ranges the device does not have', () => {
		const c = rm();
		c.sticks.pitch.axis = 0;
		c.sticks.yaw.axis = 9;
		c.sticks.throttle.min = 5;
		c.sticks.throttle.max = 5;
		c.sticks.roll.deadband = 0.7;
		c.profile.axis = 4;
		const e = validateConfig(c, IDS, 8, 2);
		expect(e).toContain('pitch: axis 0 is already roll');
		expect(e).toContain('yaw: axis 9 is not an axis of this device');
		expect(e).toContain('throttle: min must be below max');
		expect(e).toContain('roll: deadband must be 0..0.5');
		expect(validateConfig({ ...rm(), arm: { ...rm().arm, index: 2 } }, IDS, 8, 2)).toContain('arm: axis 2 is the throttle stick');
		expect(e).toContain('flight modes: axis 4 is the arm switch');

		const x = mockDefault(MOCK_XBOX.name);
		x.profile.buttons['0'] = 'hold';
		x.profile.buttons['1'] = 'agile';
		x.profile.buttons['12'] = 'nope';
		const ex = validateConfig(x, IDS, 8, 11);
		expect(ex).toContain('flight modes: button 0 is the arm button');
		expect(ex).toContain('flight modes: button 1 is the disarm button');
		expect(ex).toContain('flight modes: button 12 is not a button of this device');
		expect(ex).toContain('flight modes: button 12 has no known profile');
		x.arm.disarm_button = 0;
		expect(validateConfig(x, IDS, 8, 11)).toContain('arm: the disarm button is the arm button');
		expect(validateConfig({ ...x, profile: { ...x.profile, buttons: {} } }, IDS)).toContain('flight modes: assign at least one button');
		expect(validateConfig({ ...rm(), version: 2, device_name: '' }, IDS)).toEqual(['version 2: only 1 is known', 'no device name']);
	});
});

describe('wire shape', () => {
	it('parses loosely and writes only the active source', () => {
		const c = parseConfig({ ...toWire(mockDefault(MOCK_XBOX.name)), source: 'stored', extra: 1 });
		expect(c.arm).toEqual({ source: 'button', index: 4, on_above: 0, button: 0, require_throttle_low: false, disarm_button: 1 });
		const w = toWire(c);
		expect(w.arm).toEqual({ source: 'button', button: 0, require_throttle_low: false, disarm_button: 1 });
		expect(w.profile).toEqual({ source: 'buttons', buttons: { '2': 'hold', '3': 'stabilized', '4': 'freestyle', '5': 'agile' } });
		expect(toWire(rm()).arm).toEqual({ source: 'axis', index: 4, on_above: 0, require_throttle_low: true });
		expect(toWire({ ...rm(), profile: { ...rm().profile, source: 'none' } }).profile).toEqual({ source: 'none' });
		expect(sameConfig(rm(), parseConfig(toWire(rm())))).toBe(true);
		expect(sameConfig(rm(), { ...rm(), throttle_centre_hold: false })).toBe(false);
	});

	it('parses the radio and radio_devices messages', () => {
		const m = parseServer(JSON.stringify({ type: 'radio', device: 'X', axes: [1, 'x'], buttons: [0, 1, true], normalized: { roll: 0.5 }, arm: 1, profile: 'agile' }));
		expect(m).toEqual({
			type: 'radio',
			radio: { device: 'X', axes: [1, 0], buttons: [0, 1, 1], normalized: { roll: 0.5, pitch: NaN, throttle: NaN, yaw: NaN }, arm: true, profile: 'agile' }
		});
		expect(parseServer('{"type":"radio","profile":null}')).toMatchObject({ radio: { profile: null, device: '' } });
		expect(parseServer(JSON.stringify({ type: 'radio_devices', devices: [MOCK_XBOX, { name: '' }, 'junk'] }))).toEqual({ type: 'radio_devices', devices: [MOCK_XBOX] });
	});
});

describe('API client', () => {
	const reply =
		(status: number, body: unknown, seen?: { url: string; init?: unknown }[]): FetchFn =>
		async (url, init) => {
			seen?.push({ url, init });
			return { ok: status < 300, status, json: async () => (body === undefined ? Promise.reject(new Error('no body')) : body) };
		};

	it('reports the backend error, the status without one, and a network failure', async () => {
		await expect(makeRadioApi(reply(400, { error: 'arm: axis 9 out of range' })).save(rm())).rejects.toThrow('PUT /api/radio/config: arm: axis 9 out of range');
		await expect(makeRadioApi(reply(404, undefined)).devices()).rejects.toThrow('GET /api/radio/devices: HTTP 404');
		const down: FetchFn = async () => {
			throw new TypeError('Failed to fetch');
		};
		await expect(makeRadioApi(down).config('pad')).rejects.toThrow('GET /api/radio/config?device=pad: Failed to fetch');
	});

	it('encodes the device name and sends the config as JSON', async () => {
		const seen: { url: string; init?: unknown }[] = [];
		const r = await makeRadioApi(reply(200, { ...toWire(rm()), source: 'stored' }, seen)).config('RM TX16S & co');
		expect(seen[0].url).toBe('/api/radio/config?device=RM%20TX16S%20%26%20co');
		expect(r.source).toBe('stored');
		await makeRadioApi(reply(200, toWire(rm()), seen)).save(rm());
		expect(seen[1].init).toEqual({ method: 'PUT', body: JSON.stringify(toWire(rm())), headers: { 'Content-Type': 'application/json' } });
		await makeRadioApi(reply(200, {}, seen)).reset('pad');
		expect(seen[2]).toEqual({ url: '/api/radio/config?device=pad', init: { method: 'DELETE' } });
	});
});

describe('mock round trip', () => {
	it('stores, validates and resets a config through the client', async () => {
		const store = new MockRadioStore(IDS);
		const api = makeRadioApi(store.fetch);
		expect(await api.devices()).toEqual([MOCK_RADIOMASTER]);
		const first = await api.config(MOCK_RADIOMASTER.name);
		expect(first.source).toBe('default');
		const c = first.config;
		c.profile.bands = moveBand(c.profile.bands, 0, 1);
		const saved = await api.save(c);
		expect(sameConfig(saved, c)).toBe(true);
		expect((await api.config(MOCK_RADIOMASTER.name)).source).toBe('stored');
		expect((await api.devices())[0].has_config).toBe(true);
		c.profile.bands = [{ upper: 0.5, profile: 'hold' }];
		await expect(api.save(c)).rejects.toThrow('flight modes: 2 to 6 bands, not 1; flight modes: the last band must end at 1.0');
		await api.reset(MOCK_RADIOMASTER.name);
		const back = await api.config(MOCK_RADIOMASTER.name);
		expect(back.source).toBe('default');
		expect(sameConfig(back.config, mockDefault(MOCK_RADIOMASTER.name))).toBe(true);
		expect(store.plug(MOCK_XBOX)).toBe(true);
		expect(store.plug(MOCK_XBOX)).toBe(false);
		expect((await api.devices()).map((d) => d.name)).toEqual([MOCK_RADIOMASTER.name, MOCK_XBOX.name]);
	});

	it("streams frames whose profile follows CH6 through the stored bands and whose arm needs the throttle low", () => {
		const store = new MockRadioStore(IDS);
		const at = (t: number) => store.frame(MOCK_RADIOMASTER.name, t) as { axes: number[]; arm: boolean; profile: string };
		expect(at(0.5)).toMatchObject({ arm: false, profile: 'hold' });
		expect(at(1.2).arm).toBe(true);
		expect(at(3).profile).toBe('freestyle');
		expect(at(5.5).profile).toBe('stabilized');
		expect(at(8).profile).toBe('agile');
		expect(at(13.5).arm).toBe(false);
		expect(store.frame('nobody', 0)).toBeNull();
	});
});

describe('mock radio_subscribe', () => {
	let got: ServerMsg[];
	let fc: MockFc;
	beforeEach(() => {
		vi.useFakeTimers();
		got = [];
		fc = new MockFc(normalizeSchema(fixture as unknown as Schema), '');
		fc.onmessage = (ev) => {
			const m = parseServer(ev.data);
			if (m) got.push(m);
		};
	});
	afterEach(() => {
		fc.close();
		vi.useRealTimers();
	});

	it('streams at the telemetry rate, plugs the pad in, refuses an unknown device and stops on null', () => {
		fc.send(JSON.stringify({ type: 'radio_subscribe', device: MOCK_RADIOMASTER.name }));
		vi.advanceTimersByTime(1030);
		const radio = got.filter((m) => m.type === 'radio');
		expect(radio.length).toBeGreaterThanOrEqual(19);
		expect(radio[0]).toMatchObject({ radio: { device: MOCK_RADIOMASTER.name } });
		vi.advanceTimersByTime(2500);
		expect(got.some((m) => m.type === 'radio_devices' && m.devices.some((d) => d.name === MOCK_XBOX.name))).toBe(true);
		fc.send(JSON.stringify({ type: 'radio_subscribe', device: 'nobody' }));
		vi.advanceTimersByTime(100);
		expect(got.filter((m) => m.type === 'error').at(-1)).toEqual({ type: 'error', request: 'radio_subscribe', error: 'no device named nobody' });
		fc.send(JSON.stringify({ type: 'radio_subscribe', device: null }));
		got = [];
		vi.advanceTimersByTime(500);
		expect(got.some((m) => m.type === 'radio')).toBe(false);
	});
});
