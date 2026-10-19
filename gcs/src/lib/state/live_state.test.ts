// The live view's panels: each stream becomes the values of its panel, staleness is per panel against the shell's vehicle timeout, and
// the controller asks for the streams while the page is open and stops them when it closes.
import { afterEach, describe, expect, it } from 'vitest';
import { get, writable } from 'svelte/store';
import { LinkSession } from '$link/session';
import { TelemetryClient, STREAM_IDS } from '$link/telemetry';
import { FakeVehicle } from '$link/testing/fake_vehicle';
import * as m from '$link/testing/telemetry_messages';
import type { VehicleParam } from '$link/vehicle';
import { channelMapOf, computeLive, eulerOf, LiveController, LIVE_TICK_MS, PANELS, type ChannelMap, type PanelId, type PanelView } from './live_state';
import { SHELL_TIMING } from './shell';

const flush = () => new Promise((r) => setTimeout(r, 0));
const STALE = SHELL_TIMING.vehicleStaleMs;
const T0 = 50_000;

const open: Array<() => void> = [];
afterEach(() => {
	while (open.length) open.pop()?.();
});

async function setup() {
	const clock = { t: T0 };
	const vehicle = new FakeVehicle({ params: [] });
	const session = new LinkSession(vehicle.link, { heartbeatTimeoutMs: 600_000, now: () => clock.t });
	const telemetry = new TelemetryClient(session, { now: () => clock.t });
	open.push(() => telemetry.dispose(), () => session.dispose());
	vehicle.heartbeat();
	await flush();
	const view = (map: ChannelMap | null = null) => computeLive(telemetry.snapshot(), clock.t, { channelMap: map });
	return { clock, vehicle, session, telemetry, view };
}

const panel = (views: PanelView[], id: PanelId): PanelView => views.find((p) => p.id === id)!;
/** The value (with its unit) of a row of a panel, as the page shows it. */
const shown = (p: PanelView, key: string): string => {
	const r = p.rows.find((x) => x.key === `${p.id}.${key}`);
	if (!r) throw new Error(`no row ${p.id}.${key} in ${p.rows.map((x) => x.key).join(', ')}`);
	return r.unit ? `${r.value} ${r.unit}` : r.value;
};

describe('eulerOf', () => {
	it('is zero for the identity and a quarter turn about each axis for the matching quaternion', () => {
		expect(eulerOf([1, 0, 0, 0])).toEqual([0, 0, 0]);
		const [roll] = eulerOf([Math.SQRT1_2, Math.SQRT1_2, 0, 0]);
		expect(roll).toBeCloseTo(Math.PI / 2, 12);
		const [, pitch] = eulerOf([Math.SQRT1_2, 0, Math.SQRT1_2, 0]);
		expect(pitch).toBeCloseTo(Math.PI / 2, 12);
		const [, , yaw] = eulerOf([Math.SQRT1_2, 0, 0, Math.SQRT1_2]);
		expect(yaw).toBeCloseTo(Math.PI / 2, 12);
	});
});

describe('each stream becomes the values of its panel', () => {
	it('before any data every panel is stale, unheard and shows no value', async () => {
		const { view } = await setup();
		const v = view();
		expect(v.panels.map((p) => p.id)).toEqual(PANELS.map((p) => p.id));
		for (const p of v.panels) {
			expect(p.stale).toBe(true);
			expect(p.heard).toBe(false);
			expect(p.streams.every((s) => s.stale && s.ageMs === null)).toBe(true);
		}
		expect(shown(panel(v.panels, 'attitude'), 'use.roll')).toBe('— °');
	});

	it('attitude: the angles in use, the in-use quaternion, Mahony and the ESKF as roll, pitch, yaw', async () => {
		const { vehicle, view } = await setup();
		vehicle.emit(m.attitude);
		vehicle.emit(m.attitudeQuaternion);
		vehicle.emit(m.estimator);
		await flush();
		const p = panel(view().panels, 'attitude');
		expect(shown(p, 'use.roll')).toBe('14.3 °');
		expect(shown(p, 'use.pitch')).toBe('-28.6 °');
		expect(shown(p, 'use.yaw')).toBe('85.9 °');
		expect(shown(p, 'use.q')).toBe('0.5000 -0.5000 0.5000 -0.5000');
		expect(shown(p, 'use.source')).toBe('Mahony');
		expect(shown(p, 'mahony.roll')).toBe('90.0 °');
		expect(shown(p, 'mahony.pitch')).toBe('0.0 °');
		expect(shown(p, 'eskf.yaw')).toBe('90.0 °');
		expect(shown(p, 'eskf.valid')).toBe('yes');
	});

	it('attitude: an image without the ESKF says so and shows no ESKF angles', async () => {
		const { vehicle, view } = await setup();
		vehicle.emit({ ...m.estimator, flags: 1, q_eskf: [0, 0, 0, 0] });
		await flush();
		const p = panel(view().panels, 'attitude');
		expect(shown(p, 'eskf')).toBe('not present in this image');
		expect(p.rows.some((r) => r.key === 'attitude.eskf.roll')).toBe(false);
		expect(p.notes.join(' ')).toContain('does not step the ESKF');
	});

	it('rates: the gyro rate, the rate setpoint and the thrust, in degrees per second and percent', async () => {
		const { vehicle, view } = await setup();
		vehicle.emit(m.attitude);
		vehicle.emit(m.attitudeTarget);
		await flush();
		const p = panel(view().panels, 'rates');
		expect(shown(p, 'gyro.x')).toBe('7.2 °/s');
		expect(shown(p, 'gyro.y')).toBe('-114.6 °/s');
		expect(shown(p, 'gyro.z')).toBe('171.9 °/s');
		expect(shown(p, 'setpoint.x')).toBe('57.3 °/s');
		expect(shown(p, 'setpoint.y')).toBe('-28.6 °/s');
		expect(shown(p, 'thrust')).toBe('38 % of the largest');
		expect(p.rows.find((r) => r.key === 'rates.thrust')?.bar).toBeCloseTo(0.375, 6);
		expect(shown(p, 'attitude')).toBe('none (not in angle mode)');
	});

	it('rates: in angle mode the attitude setpoint is shown as angles', async () => {
		const { vehicle, view } = await setup();
		vehicle.emit({ ...m.attitudeTarget, type_mask: 0, q: [Math.SQRT1_2, Math.SQRT1_2, 0, 0] });
		await flush();
		expect(shown(panel(view().panels, 'rates'), 'attitude.roll')).toBe('90.0 °');
	});

	it('radio: the channels in microseconds, the link quality and, with the channel map, the sticks', async () => {
		const { vehicle, view } = await setup();
		vehicle.emit(m.rcChannels);
		await flush();
		const without = panel(view().panels, 'rc');
		expect(shown(without, 'ch.1')).toBe('1500 µs');
		expect(shown(without, 'ch.5')).toBe('2012 µs');
		expect(shown(without, 'quality')).toBe('50 %');
		expect(shown(without, 'channels')).toBe('16');
		expect(without.rows.some((r) => r.key.startsWith('rc.stick.'))).toBe(false);
		expect(without.notes.join(' ')).toContain('stick channels are not known');

		const withMap = panel(view({ roll: 1, pitch: 2, throttle: 3, yaw: 4 }).panels, 'rc');
		expect(shown(withMap, 'stick.roll')).toBe('0.00');
		expect(shown(withMap, 'stick.pitch')).toBe('0.50');
		expect(shown(withMap, 'stick.throttle')).toBe('-1.00');
		expect(shown(withMap, 'stick.yaw')).toBe('-0.50');
		expect(withMap.rows.find((r) => r.key === 'rc.stick.throttle')).toMatchObject({ bar: 0, centred: false });
		expect(withMap.rows.find((r) => r.key === 'rc.stick.pitch')).toMatchObject({ bar: 0.75, centred: true });
	});

	it('radio: no RC frame and no link statistics are said so, not shown as values', async () => {
		const { vehicle, view } = await setup();
		vehicle.emit({ ...m.rcChannels, chancount: 0, rssi: 255, chan1_raw: 0xffff });
		await flush();
		const p = panel(view().panels, 'rc');
		expect(shown(p, 'ch.1')).toBe('— µs');
		expect(shown(p, 'quality')).toBe('— %');
		expect(p.notes.join(' ')).toContain('No RC frame');
		expect(p.notes.join(' ')).toContain('link quality is unknown');
	});

	it('motors: the DShot value of each motor, with the rotor speed said not to be on the link', async () => {
		const { vehicle, view } = await setup();
		vehicle.emit(m.actuatorOutputStatus);
		await flush();
		const p = panel(view().panels, 'motors');
		expect([1, 2, 3, 4].map((i) => shown(p, `m.${i}`))).toEqual(['48 DShot', '1000 DShot', '2047 DShot', '0 DShot']);
		expect(p.rows.find((r) => r.key === 'motors.m.3')?.bar).toBe(1);
		expect(shown(p, 'rotor')).toBe('not on the link');
	});

	it('arming: the arm state and the motor gate, with their tone', async () => {
		const { vehicle, view } = await setup();
		vehicle.emit(m.fcStatus);
		await flush();
		const p = panel(view().panels, 'arming');
		expect(shown(p, 'state')).toBe('armed hold');
		expect(shown(p, 'gate')).toBe('armed');
		expect(shown(p, 'rcframes')).toBe('6789');
		expect(shown(p, 'rcage')).toBe('20 ms');
		expect(p.rows.find((r) => r.key === 'arming.state')?.tone).toBe('bad');
	});

	it('estimators: the validity flags and the three gyro biases in degrees per second', async () => {
		const { vehicle, view } = await setup();
		vehicle.emit(m.estimator);
		await flush();
		const p = panel(view().panels, 'estimators');
		expect(shown(p, 'mahony.valid')).toBe('yes');
		expect(shown(p, 'eskf.present')).toBe('yes');
		expect(shown(p, 'inuse')).toBe('Mahony');
		expect(shown(p, 'bias.cal')).toBe('0.112 -0.224 0.448 °/s (x y z)');
		expect(shown(p, 'bias.eskf')).toBe('-14.324 7.162 3.581 °/s (x y z)');
	});

	it('flags and faults: the composition flags, the counters (bad when above zero) and the sensor health', async () => {
		const { vehicle, view } = await setup();
		vehicle.emit(m.fcStatus);
		vehicle.emit(m.sysStatus);
		await flush();
		const p = panel(view().panels, 'faults');
		expect(shown(p, 'angle')).toBe('yes');
		expect(shown(p, 'airmode')).toBe('no');
		expect(shown(p, 'imuinvalid')).toBe('no');
		expect(shown(p, 'ratefault')).toBe('no');
		expect(shown(p, 'ratelatched')).toBe('yes');
		expect(p.rows.find((r) => r.key === 'faults.ratelatched')?.tone).toBe('bad');
		expect(shown(p, 'ratefaults')).toBe('1');
		expect(p.rows.find((r) => r.key === 'faults.ratetiming')).toMatchObject({ value: '0', tone: 'ok' });
		expect(shown(p, 'deadline')).toBe('3');
		expect(shown(p, 'executions')).toBe('12345');
		expect(shown(p, 'gyro')).toBe('yes');
		expect(shown(p, 'rc')).toBe('no');
		expect(shown(p, 'drop')).toBe('2.5 %');
		expect(shown(p, 'errors')).toBe('7');
	});

	it('IMU: gyro, accelerometer, temperature and the validity of the sample', async () => {
		const { vehicle, view } = await setup();
		vehicle.emit(m.highresImu);
		await flush();
		const p = panel(view().panels, 'imu');
		expect(shown(p, 'gyro.x')).toBe('28.6 °/s');
		expect(shown(p, 'gyro.y')).toBe('-14.3 °/s');
		expect(shown(p, 'accel.z')).toBe('-9.75 m/s²');
		expect(shown(p, 'temp')).toBe('31.5 °C');
		expect(shown(p, 'gyrovalid')).toBe('yes');
		vehicle.emit({ ...m.highresImu, fields_updated: 0x0007 });
		await flush();
		const q = panel(view().panels, 'imu');
		expect(shown(q, 'gyrovalid')).toBe('no');
		expect(shown(q, 'temp')).toBe('— °C');
	});
});

describe('staleness is per panel', () => {
	it('a panel is stale when any stream it shows is, and only the panels with that stream', async () => {
		const { clock, vehicle, telemetry } = await setup();
		for (const msg of m.allStreamMessages) vehicle.emit(msg);
		await flush();
		const at = (dt: number) => {
			clock.t = T0 + dt;
			return computeLive(telemetry.snapshot(), clock.t).panels;
		};
		expect(at(0).every((p) => !p.stale && p.heard)).toBe(true);
		expect(at(STALE).every((p) => !p.stale)).toBe(true);
		expect(at(STALE + 1).every((p) => p.stale && p.heard)).toBe(true);

		// Only the attitude stream keeps arriving: the panels that show it and nothing else stale stay live.
		clock.t = T0 + STALE;
		vehicle.emit(m.attitude);
		await flush();
		const later = at(STALE + 1);
		const staleIds = later.filter((p) => p.stale).map((p) => p.id);
		expect(staleIds.sort()).toEqual(['arming', 'attitude', 'estimators', 'faults', 'imu', 'motors', 'rates', 'rc']);
		// attitude and rates still carry a stale second stream (estimator / target); the stream flags say which one.
		expect(panel(later, 'rates').streams.map((s) => [s.id, s.stale])).toEqual([['attitude', false], ['target', true]]);
		expect(panel(later, 'attitude').streams.map((s) => [s.id, s.stale])).toEqual([['attitude', false], ['quaternion', true], ['estimator', true]]);
	});

	it('every panel has its own stale flag and a fresh stream clears only its own panel', async () => {
		const { clock, vehicle, telemetry } = await setup();
		for (const msg of m.allStreamMessages) vehicle.emit(msg);
		await flush();
		clock.t = T0 + STALE + 100;
		vehicle.emit(m.highresImu);
		await flush();
		const panels = computeLive(telemetry.snapshot(), clock.t).panels;
		expect(panel(panels, 'imu').stale).toBe(false);
		expect(panels.filter((p) => !p.stale).map((p) => p.id)).toEqual(['imu']);
	});

	it('a stale panel keeps its last values', async () => {
		const { clock, vehicle, telemetry } = await setup();
		vehicle.emit(m.highresImu);
		await flush();
		clock.t = T0 + STALE * 5;
		const p = panel(computeLive(telemetry.snapshot(), clock.t).panels, 'imu');
		expect(p.stale).toBe(true);
		expect(shown(p, 'temp')).toBe('31.5 °C');
	});
});

describe('channelMapOf', () => {
	const param = (linkId: string, value: number, type: 'i32' | 'f32' = 'i32'): VehicleParam => ({
		index: 0, linkId, type, activeBits: value >>> 0, stagedBits: value >>> 0, sigma: null, sigmaBits: null, origin: null, method: null, sigmaKind: null,
		locked: false, hasRecord: true, sourceLength: null, source: null, sourcePartial: ''
	});
	const four = [param('rc_channel_roll', 1), param('rc_channel_pitch', 2), param('rc_channel_throttle', 3), param('rc_channel_yaw', 4)];

	it('reads the four stick channels from the running config, and is null if any is missing or not an integer', () => {
		expect(channelMapOf(four)).toEqual({ roll: 1, pitch: 2, throttle: 3, yaw: 4 });
		expect(channelMapOf(four.slice(1))).toBeNull();
		expect(channelMapOf([...four.slice(1), param('rc_channel_roll', 1, 'f32')])).toBeNull();
		expect(channelMapOf([])).toBeNull();
	});
});

describe('the controller', () => {
	it('asks for the streams on start, recomputes the panels against the clock, and stops the streams on stop', async () => {
		const { clock, vehicle, telemetry } = await setup();
		const map = writable<ChannelMap | null>(null);
		const live = new LiveController({ telemetry, channelMap: map, now: () => clock.t });
		const sent = () => vehicle.received.filter((x) => x.kind === 'CommandLong' && x.command === 511);
		expect(sent()).toHaveLength(0);
		live.start();
		expect(sent()).toHaveLength(STREAM_IDS.length);
		expect(get(live.view)?.streaming).toBe(true);
		expect(get(live.view)?.panels.every((p) => p.stale)).toBe(true);

		for (const msg of m.allStreamMessages) vehicle.emit(msg);
		await flush();
		live.tick();
		expect(get(live.view)?.panels.every((p) => !p.stale)).toBe(true);

		// The control: with the vehicle silent the same panels turn stale as the clock passes the timeout, with no new data at all.
		clock.t += STALE + 1;
		live.tick();
		expect(get(live.view)?.panels.every((p) => p.stale)).toBe(true);

		map.set({ roll: 1, pitch: 2, throttle: 3, yaw: 4 });
		expect(get(live.view)?.channelMapKnown).toBe(true);

		live.stop();
		expect(sent().slice(-STREAM_IDS.length).every((x) => x.kind === 'CommandLong' && x.param2 === -1)).toBe(true);
		const count = sent().length;
		live.tick();
		expect(sent()).toHaveLength(count);
	});

	it('ticks on its own timer while started and not after stop', async () => {
		const { clock, telemetry } = await setup();
		let computed = 0;
		const counting = { ...telemetry, snapshot: () => (computed++, telemetry.snapshot()), start: () => telemetry.start(), stop: () => telemetry.stop(), maintain: () => telemetry.maintain() };
		const live = new LiveController({ telemetry: counting, now: () => clock.t });
		live.start();
		const afterStart = computed;
		await new Promise((r) => setTimeout(r, LIVE_TICK_MS * 3.5));
		expect(computed).toBeGreaterThan(afterStart + 1);
		live.stop();
		const stopped = computed;
		await new Promise((r) => setTimeout(r, LIVE_TICK_MS * 2.5));
		expect(computed).toBe(stopped);
	});
});
