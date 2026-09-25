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
	errorPage,
	errorPath,
	errorsAt,
	evenBands,
	insertBand,
	ApiError,
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
	validateWire,
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
	it('accepts the two defaults, on the device and unplugged', () => {
		expect(validateConfig(rm(), IDS, 8, 2)).toEqual([]);
		expect(validateConfig(rm(), IDS)).toEqual([]);
		expect(validateConfig(mockDefault(MOCK_XBOX.name), IDS, 8, 11)).toEqual([]);
		expect(rm().profile.bands.map((b) => b.upper)).toEqual([-0.50002, 0, 0.5, 1]);
	});

	it("refuses band counts, order, the last edge and unknown profiles with the backend's messages", () => {
		const c = rm();
		const at = (bands: Band[]): string[] => validateConfig({ ...c, profile: { ...c.profile, bands } }, IDS, 8, 2);
		expect(at(four.slice(3))).toEqual(['profile.bands: not 2 to 6 bands']);
		expect(at([...evenBands(6, IDS), { upper: 1, profile: 'hold' }])).toEqual(['profile.bands: not 2 to 6 bands']);
		expect(at([four[1], four[0], four[2], four[3]])).toEqual(['profile.bands[1].upper: not above the band before']);
		expect(at([four[0], four[1], four[2], { upper: 0.9, profile: 'agile' }])).toEqual(["profile.bands[3].upper: the last band's is not 1"]);
		expect(at([four[0], { upper: 1, profile: 'sport' }])).toEqual(['profile.bands[1].profile: not a profile id']);
		expect(at([{ upper: -1, profile: 'hold' }, four[3]])).toEqual(['profile.bands[0].upper: not above -1']);
		expect(at([{ upper: -1.5, profile: 'hold' }, four[3]])).toEqual(['profile.bands[0].upper: outside -1E0..1E0']);
		expect(at([four[0], { upper: 0.999999999, profile: 'agile' }])).toEqual([]);
	});

	it('refuses stick axes twice, arm or profile on a stick axis, and indices past the device', () => {
		const c = rm();
		c.sticks.pitch.axis = 0;
		c.sticks.yaw.axis = 9;
		c.sticks.throttle.center = -32767;
		c.sticks.roll.deadband = 0.7;
		c.profile.index = 4;
		expect(validateConfig(c, IDS, 8, 2)).toEqual([
			'sticks.roll.deadband: outside 0E0..5E-1',
			'sticks.pitch: axis 0 is already sticks.roll',
			'sticks.throttle.min: not below center',
			'sticks.yaw.axis: outside 0..7',
			'profile: axis 4 is already arm'
		]);
		expect(validateConfig(c, IDS)).toEqual([
			'sticks.roll.deadband: outside 0E0..5E-1',
			'sticks.pitch: axis 0 is already sticks.roll',
			'sticks.throttle.min: not below center',
			'profile: axis 4 is already arm'
		]);
		expect(validateConfig({ ...rm(), arm: { ...rm().arm, index: 2 } }, IDS, 8, 2)).toEqual(['arm: axis 2 is already sticks.throttle']);
		expect(validateConfig({ ...rm(), profile: { ...rm().profile, index: 3 } }, IDS, 8, 2)).toEqual(['profile: axis 3 is already sticks.yaw']);
		expect(validateConfig({ ...rm(), arm: { ...rm().arm, on_above: 1.5 } }, IDS)).toEqual(['arm.on_above: outside -1E0..1E0']);
		expect(validateConfig({ ...rm(), throttle_centre_hold: false }, IDS)).toEqual(["throttle_centre_hold: only true (the throttle's centre holds height) is supported"]);
		expect(validateConfig({ ...rm(), version: 2, device_name: '' }, IDS)).toEqual(['version: outside 1..1', 'device_name: empty']);
		const s = rm();
		s.sticks.roll.min = 1.5;
		s.sticks.pitch.max = NaN;
		expect(validateConfig(s, IDS)).toEqual(['sticks.roll.min: not an integer', 'sticks.pitch.max: not a number']);
		expect(validateConfig(rm(), IDS, 20, 40)).toEqual([]);
		const wide = rm();
		wide.sticks.yaw.axis = 15;
		expect(validateConfig(wide, IDS, 20)).toEqual([]);
		wide.sticks.yaw.axis = 16;
		expect(validateConfig(wide, IDS, 20)).toEqual(['sticks.yaw.axis: outside 0..15']);
	});

	it('needs a disarm button with an arm button, and keeps profile buttons off both', () => {
		const x = mockDefault(MOCK_XBOX.name);
		x.profile.buttons['0'] = 'hold';
		x.profile.buttons['1'] = 'agile';
		x.profile.buttons['12'] = 'nope';
		expect(validateConfig(x, IDS, 8, 11)).toEqual([
			'profile.buttons.0: the button already arms or disarms',
			'profile.buttons.1: the button already arms or disarms',
			'profile.buttons.12: not a button 0..10'
		]);
		expect(validateConfig(x, IDS)).toContain('profile.buttons.12: not a profile id');
		const y = mockDefault(MOCK_XBOX.name);
		expect(validateConfig({ ...y, arm: { ...y.arm, disarm_button: null } }, IDS, 8, 11)).toEqual(['arm.disarm_button: required with an arm button']);
		expect(validateConfig({ ...y, arm: { ...y.arm, disarm_button: 0 } }, IDS, 8, 11)).toEqual(['arm.disarm_button: the arm button itself']);
		expect(validateConfig({ ...y, arm: { ...y.arm, button: 11 } }, IDS, 8, 11)).toEqual(['arm.button: outside 0..10']);
		expect(validateConfig({ ...y, profile: { ...y.profile, buttons: {} } }, IDS)).toEqual(['profile.buttons: empty']);
		const r = rm();
		expect(validateConfig({ ...r, arm: { ...r.arm, disarm_button: 1 }, profile: { ...r.profile, source: 'buttons', buttons: { '1': 'hold' } } }, IDS)).toEqual([
			'profile.buttons.1: the button already arms or disarms'
		]);
		expect(validateConfig({ ...r, arm: { ...r.arm, disarm_button: 1 }, profile: { ...r.profile, source: 'buttons', buttons: { '0': 'hold' } } }, IDS)).toEqual([]);
	});

	it('reads the wire as the backend does: missing fields, types, the arm button by index', () => {
		expect(validateWire([], IDS)).toEqual(['not a JSON object']);
		expect(validateWire({}, IDS)[0]).toBe('version: missing');
		const w = toWire(mockDefault(MOCK_XBOX.name)) as Record<string, Record<string, unknown>>;
		expect(validateWire({ ...w, arm: { source: 'button', index: 7, require_throttle_low: false, disarm_button: 1 } }, IDS)).toEqual([]);
		expect(validateWire({ ...w, arm: { source: 'button', index: 3, require_throttle_low: false, disarm_button: 1 } }, IDS)).toEqual(['profile.buttons.3: the button already arms or disarms']);
		expect(validateWire({ ...w, arm: { ...w.arm, source: 'switch' } }, IDS)).toEqual(['arm.source: not "axis" or "button"']);
		expect(validateWire({ ...w, profile: { source: 'knob' } }, IDS)).toEqual(['profile.source: not "axis", "buttons" or "none"']);
		expect(validateWire({ ...w, sticks: { ...w.sticks, yaw: { ...(w.sticks.yaw as object), reverse: 1 } } }, IDS)).toEqual(['sticks.yaw.reverse: not true or false']);
		expect(errorPath('sticks.yaw.axis: outside 0..7')).toBe('sticks.yaw.axis');
		expect(errorPath('no path here')).toBe('');
		expect(errorPage('sticks.yaw.axis: outside 0..7')).toBe('sticks');
		expect(errorPage('arm: axis 2 is already sticks.throttle')).toBe('sticks');
		expect(errorPage('profile.bands[2].upper: not above -1')).toBe('modes');
		expect(errorPage('profile.buttons.12: not a button 0..10')).toBe('modes');
		expect(errorPage('device_name: empty')).toBeNull();
		expect(errorPage("throttle_centre_hold: only true (the throttle's centre holds height) is supported")).toBeNull();
		expect(errorsAt(['arm: x', 'arm.index: y', 'sticks.roll: z'], 'arm', 'arm.index')).toEqual(['arm: x', 'arm.index: y']);
	});
});

describe('wire shape', () => {
	it('parses loosely and writes only the active source', () => {
		const c = parseConfig({ ...toWire(mockDefault(MOCK_XBOX.name)), source: 'stored', extra: 1 });
		expect(c.arm).toEqual({ source: 'button', index: 4, on_above: 0, button: 0, require_throttle_low: false, disarm_button: 1 });
		const w = toWire(c);
		expect(w.arm).toEqual({ source: 'button', button: 0, require_throttle_low: false, disarm_button: 1 });
		expect(w.profile).toEqual({ source: 'buttons', buttons: { '2': 'hold', '3': 'stabilized', '4': 'freestyle', '5': 'agile' } });
		expect(toWire(rm()).arm).toEqual({ source: 'axis', index: 4, on_above: 0, require_throttle_low: true, disarm_button: null });
		expect(toWire(rm()).profile).toMatchObject({ source: 'axis', index: 5 });
		expect(Object.keys(toWire(rm()).profile as object)).toEqual(['source', 'index', 'bands']);
		expect(parseConfig({ profile: { source: 'axis', index: 6, bands: [] } }).profile.index).toBe(6);
		expect(toWire({ ...rm(), profile: { ...rm().profile, source: 'none' } }).profile).toEqual({ source: 'none' });
		expect(sameConfig(rm(), parseConfig(toWire(rm())))).toBe(true);
		expect(sameConfig(rm(), { ...rm(), throttle_centre_hold: false })).toBe(false);
		// GET's body for the Xbox pad, as gcs/src/radio.cpp writes it (to_json + source).
		const got = parseConfig(JSON.parse(
			'{"version":1,"device_name":"Microsoft X-Box 360 pad","sticks":{"roll":{"axis":3,"min":-32767,"center":0,"max":32767,"reverse":false,"deadband":0.1},"pitch":{"axis":4,"min":-32767,"center":0,"max":32767,"reverse":true,"deadband":0.1},"throttle":{"axis":1,"min":-32767,"center":0,"max":32767,"reverse":true,"deadband":0.1},"yaw":{"axis":0,"min":-32767,"center":0,"max":32767,"reverse":false,"deadband":0.1}},"throttle_centre_hold":true,"arm":{"source":"button","button":0,"require_throttle_low":false,"disarm_button":1},"profile":{"source":"buttons","buttons":{"2":"hold","3":"stabilized","4":"freestyle","5":"agile"}},"source":"default"}'
		));
		expect(sameConfig(got, mockDefault(MOCK_XBOX.name))).toBe(true);
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
		await expect(makeRadioApi(reply(400, { error: 'sticks.yaw.axis: outside 0..7' })).save(rm())).rejects.toThrow('PUT /api/radio/config: sticks.yaw.axis: outside 0..7');
		await expect(makeRadioApi(reply(400, { error: 'sticks.yaw.axis: outside 0..7' })).save(rm())).rejects.toMatchObject({ status: 400, detail: 'sticks.yaw.axis: outside 0..7' });
		await expect(makeRadioApi(reply(500, { error: 'rename to /x: EACCES' })).save(rm())).rejects.toBeInstanceOf(ApiError);
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
		expect(r.error).toBeNull();
		const ignored = await makeRadioApi(reply(200, { ...toWire(rm()), source: 'default', error: '/c/x.json: sticks.yaw.axis: outside 0..15' })).config('x');
		expect(ignored).toMatchObject({ source: 'default', error: '/c/x.json: sticks.yaw.axis: outside 0..15' });
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
		await expect(api.save(c)).rejects.toMatchObject({ status: 400, detail: 'profile.bands: not 2 to 6 bands' });
		c.profile.bands = moveBand(c.profile.bands, 0, 1);
		const bad = await store.fetch('/api/radio/config', { method: 'PUT', body: JSON.stringify({ ...toWire(rm()), sticks: { ...(toWire(rm()).sticks as object), yaw: { ...rm().sticks.yaw, axis: 8 } } }) });
		expect(await bad.json()).toEqual({ error: 'sticks.yaw.axis: outside 0..7' });
		expect(await (await store.fetch('/api/radio/config', { method: 'PUT', body: '{' })).json()).toMatchObject({ error: expect.stringMatching(/^not JSON: /) });
		expect(await (await store.fetch('/api/radio/config?device=', { method: 'GET' })).json()).toEqual({ error: 'device: missing' });
		store.ignore(MOCK_RADIOMASTER.name, 'x.json: version: outside 1..1');
		expect(await api.config(MOCK_RADIOMASTER.name)).toMatchObject({ source: 'default', error: 'x.json: version: outside 1..1' });
		expect((await api.devices())[0].has_config).toBe(true);
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
