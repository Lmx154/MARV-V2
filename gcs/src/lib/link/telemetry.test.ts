// The telemetry client: the requests it sends (and stops), the decode of each stream into the domain values, the per-stream arrival
// stamps, the retry of a stream that fell silent and the reset.
import { afterEach, describe, expect, it } from 'vitest';
import { DISPLAY_RATES_HZ, REQUEST_RETRY_MS, STREAM_IDS, TelemetryClient, type StreamId } from './telemetry';
import { MESSAGES, type Message } from './generated/messages';
import { LinkSession } from './session';
import { FakeVehicle } from './testing/fake_vehicle';
import * as m from './testing/telemetry_messages';

const flush = () => new Promise((r) => setTimeout(r, 0));
const SET_MESSAGE_INTERVAL = 511;
const idOf = (kind: Message['kind']): number => MESSAGES.find((x) => x.kind === kind)!.id;
const KIND: Record<StreamId, Message['kind']> = {
	status: 'SysStatus', attitude: 'Attitude', quaternion: 'AttitudeQuaternion', rc: 'RcChannels', target: 'AttitudeTarget', imu: 'HighresImu',
	actuators: 'ActuatorOutputStatus', fcStatus: 'MarvFcStatus', estimator: 'MarvEstimator'
};

const open: Array<() => void> = [];
afterEach(() => {
	while (open.length) open.pop()?.();
});

async function setup(options: { beat?: boolean } = {}) {
	const clock = { t: 10_000 };
	const vehicle = new FakeVehicle({ params: [] });
	const session = new LinkSession(vehicle.link, { heartbeatTimeoutMs: 600_000, now: () => clock.t });
	const telemetry = new TelemetryClient(session, { now: () => clock.t });
	open.push(() => telemetry.dispose(), () => session.dispose());
	if (options.beat !== false) {
		vehicle.heartbeat();
		await flush();
	}
	const requests = () =>
		vehicle.received.flatMap((x) => (x.kind === 'CommandLong' && x.command === SET_MESSAGE_INTERVAL ? [{ id: x.param1, intervalUs: x.param2, to: x.target_system }] : []));
	return { clock, vehicle, session, telemetry, requests };
}

describe('the requests', () => {
	it('asks for each stream at its display rate once the vehicle is known, and for nothing before', async () => {
		const { vehicle, telemetry, requests } = await setup({ beat: false });
		telemetry.start();
		expect(requests()).toEqual([]);
		vehicle.heartbeat();
		await flush();
		telemetry.maintain();
		expect(requests()).toHaveLength(STREAM_IDS.length);
		for (const id of STREAM_IDS) {
			expect(requests()).toContainEqual({ id: idOf(KIND[id]), intervalUs: Math.round(1_000_000 / DISPLAY_RATES_HZ[id]), to: 1 });
		}
	});

	it('does not ask again for a stream that is being heard, and stops every stream on stop()', async () => {
		const { clock, vehicle, telemetry, requests } = await setup();
		telemetry.start();
		const first = requests().length;
		for (const msg of m.allStreamMessages) vehicle.emit(msg);
		await flush();
		clock.t += REQUEST_RETRY_MS * 2;
		for (const msg of m.allStreamMessages) vehicle.emit(msg);
		await flush();
		telemetry.maintain();
		expect(requests()).toHaveLength(first);

		telemetry.stop();
		const stops = requests().slice(first);
		expect(stops).toHaveLength(STREAM_IDS.length);
		expect(stops.every((x) => x.intervalUs === -1)).toBe(true);
		expect(new Set(stops.map((x) => x.id)).size).toBe(STREAM_IDS.length);
		// Control: once stopped it asks for nothing more, however silent the vehicle is.
		clock.t += REQUEST_RETRY_MS * 10;
		telemetry.maintain();
		expect(requests()).toHaveLength(first + STREAM_IDS.length);
		expect(telemetry.snapshot().wanted).toBe(false);
	});

	it('asks again, and only for the streams that fell silent', async () => {
		const { clock, vehicle, telemetry, requests } = await setup();
		telemetry.start();
		const first = requests().length;
		vehicle.emit(m.attitude);
		await flush();
		clock.t += REQUEST_RETRY_MS + 1;
		vehicle.emit(m.attitude);
		await flush();
		telemetry.maintain();
		const again = requests().slice(first).map((x) => x.id);
		expect(again).toHaveLength(STREAM_IDS.length - 1);
		expect(again).not.toContain(idOf('Attitude'));
		// A request just sent is not repeated at once.
		telemetry.maintain();
		expect(requests()).toHaveLength(first + STREAM_IDS.length - 1);
	});

	it('counts the requests the vehicle answers with anything but accepted', async () => {
		const { vehicle, telemetry } = await setup();
		telemetry.start();
		vehicle.emit({ kind: 'CommandAck', command: SET_MESSAGE_INTERVAL, result: 0, progress: 0, result_param2: 0, target_system: 255, target_component: 190 });
		vehicle.emit({ kind: 'CommandAck', command: SET_MESSAGE_INTERVAL, result: 2, progress: 0, result_param2: 0, target_system: 255, target_component: 190 });
		vehicle.emit({ kind: 'CommandAck', command: 512, result: 3, progress: 0, result_param2: 0, target_system: 255, target_component: 190 });
		await flush();
		expect(telemetry.snapshot().requests).toEqual({ sent: STREAM_IDS.length, refused: 1, lastRefusal: 'denied' });
	});
});

describe('each stream becomes a value', () => {
	it('decodes all nine streams and stamps their arrival', async () => {
		const { clock, vehicle, telemetry } = await setup();
		const before = telemetry.snapshot();
		for (const id of STREAM_IDS) expect(before.stamps[id]).toEqual({ at: null, count: 0, hz: null });
		clock.t = 20_000;
		for (const msg of m.allStreamMessages) vehicle.emit(msg);
		await flush();
		const s = telemetry.snapshot();
		for (const id of STREAM_IDS) expect(s.stamps[id]).toMatchObject({ at: 20_000, count: 1 });

		expect(s.status).toEqual({ gyroHealthy: true, accelHealthy: true, rcHealthy: false, estimatorHealthy: true, commDropPct: 2.5, commErrors: 7 });
		expect(s.attitude).toEqual({ roll: 0.25, pitch: -0.5, yaw: 1.5, rate: [0.125, -2, 3] });
		expect(s.quaternion).toEqual({ q: [0.5, -0.5, 0.5, -0.5] });
		expect(s.rc?.channelCount).toBe(16);
		expect(s.rc?.channelsUs.slice(0, 5)).toEqual([1500, 1756, 988, 1244, 2012]);
		expect(s.rc?.linkQuality).toBeCloseTo(127 / 254, 6);
		expect(s.target).toEqual({ rate: [1, -0.5, 0.25], attitude: null, thrust: 0.375 });
		expect(s.imu).toEqual({ gyro: [0.5, -0.25, 0.125], accel: [0.5, -1, -9.75], temperatureC: 31.5, gyroValid: true, accelValid: true, sampleUs: 5_000_000 });
		expect(s.actuators?.dshot).toEqual([48, 1000, 2047, 0]);
		expect(s.fcStatus).toMatchObject({
			armStateName: 'armed hold', armClass: 'armed', motorGateName: 'armed', rateFaultActive: false, rateFaultLatched: true, rateExecutions: 12345,
			rateFaults: 1, attitudeFaults: 2, imuDeadlineMisses: 3, rcFrames: 6789, rcAgeUs: 20000,
			flags: { angleMode: true, airmode: false, itermHeld: false, gyroCalibrated: true, estimatorValid: true, upright: true, imuInvalid: false, imuSaturated: false }
		});
		expect(s.estimator).toMatchObject({ mahonyValid: true, eskfPresent: true, eskfValid: true, eskfInUse: false });
		expect(s.estimator?.eskf[3]).toBeCloseTo(Math.SQRT1_2, 6);
		expect(s.estimator?.biasEskf).toEqual([-0.25, 0.125, 0.0625]);
	});

	it('reads the "none" encodings of the vehicle: no RC frame, no link statistics, no temperature, attitude ignored, no angle setpoint', async () => {
		const { vehicle, telemetry } = await setup();
		vehicle.emit({ ...m.rcChannels, chancount: 0, rssi: 255, chan1_raw: 0xffff, chan2_raw: 0xffff });
		vehicle.emit({ ...m.highresImu, fields_updated: 0x003f, temperature: 0 });
		vehicle.emit({ ...m.attitudeTarget, type_mask: 0, q: [1, 0, 0, 0] });
		vehicle.emit({ ...m.fcStatus, rc_frames: 0, rc_age_us: 0xffffffff, arm_state: 0 });
		await flush();
		const s = telemetry.snapshot();
		expect(s.rc).toMatchObject({ channelCount: 0, linkQuality: null });
		expect(s.rc?.channelsUs.every((c) => c === null)).toBe(true);
		expect(s.imu?.temperatureC).toBeNull();
		expect(s.target?.attitude).toEqual([1, 0, 0, 0]);
		expect(s.fcStatus).toMatchObject({ rcAgeUs: null, armStateName: 'boot', armClass: 'waiting' });
	});

	it('forgets everything on reset, and keeps the streams wanted', async () => {
		const { vehicle, telemetry } = await setup();
		telemetry.start();
		for (const msg of m.allStreamMessages) vehicle.emit(msg);
		await flush();
		telemetry.reset();
		const s = telemetry.snapshot();
		expect(s.attitude).toBeNull();
		expect(s.fcStatus).toBeNull();
		expect(s.stamps.attitude).toEqual({ at: null, count: 0, hz: null });
		expect(s.wanted).toBe(true);
		expect(s.requests.sent).toBe(0);
	});

	it('measures the arrival rate over a window', async () => {
		const { clock, vehicle, telemetry } = await setup();
		clock.t = 20_000;
		for (let i = 0; i <= 20; i++) {
			vehicle.emit(m.attitude);
			await flush();
			clock.t += 50;
		}
		expect(telemetry.snapshot().stamps.attitude.hz).toBeCloseTo(20, 0);
		expect(telemetry.snapshot().stamps.imu.hz).toBeNull();
	});
});

