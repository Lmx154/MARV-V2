// The Radio page's state: the channel bars, the mapping derived from the parameters, rule R-1 as a warning, the detection of a moved
// switch, the thresholds against the live channel, the display checks and the lock display.
import { afterEach, describe, expect, it } from 'vitest';
import { get, writable } from 'svelte/store';
import { f32ToBits, i32ToBits } from '$link/bits';
import { LinkSession } from '$link/session';
import { TelemetryClient } from '$link/telemetry';
import { FakeVehicle } from '$link/testing/fake_vehicle';
import * as m from '$link/testing/telemetry_messages';
import type { ParamRow } from '$lib/params/merge';
import {
	beginDetection, candidateFindings, CENTRE_TOLERANCE, computeRadio, emptyExcursions, findingsOf, IDLE, MOVED_US, normalised, observe, RadioController,
	rangeUs, stepDetection, throttleOf, type Detection, type Excursions, type RadioView
} from './radio_state';
import { SHELL_TIMING } from './shell';

/** A bar by its index in the view (0-based). */
const barOf = (v: RadioView, i: number) => v.bars[i]!;
const flush = () => new Promise((r) => setTimeout(r, 0));
const STALE = SHELL_TIMING.vehicleStaleMs;
const T0 = 50_000;

const open: Array<() => void> = [];
afterEach(() => {
	while (open.length) open.pop()?.();
});

/** A parameter row as the merged table has it (only what the radio state reads matters). */
function row(name: string, type: 'i32' | 'f32', value: number, extra: Partial<ParamRow> & { staged?: number } = {}): ParamRow {
	const bits = type === 'i32' ? i32ToBits(value) : f32ToBits(value);
	const staged = extra.staged === undefined ? bits : type === 'i32' ? i32ToBits(extra.staged) : f32ToBits(extra.staged);
	const { staged: _ignored, ...rest } = extra;
	void _ignored;
	return {
		key: name, name, linkId: name.slice(0, 16), index: 0, type, unit: '1', group: 'register', method: 'scenario', origin: 'DefaultFromRegister', sigmaKind: 'Choice',
		sigma: null, source: 'test', sourceFromVehicle: false, sourceLength: null, locked: false, lockInfo: null, factoryBits: bits, activeBits: bits, stagedBits: staged,
		differs: staged !== bits, changed: false, status: 'matched', notes: [], ...rest
	};
}

const LOCK = { by: 'Luis', on: '2026-10-17', via: 'manual' };
const ARM_THRESHOLD = 0.390625;
const THROTTLE_MIN = 0.060546875;

function rows(over: Record<string, Partial<ParamRow> & { staged?: number; value?: number }> = {}, bench = true): ParamRow[] {
	const def = (name: string, type: 'i32' | 'f32', value: number, extra: Partial<ParamRow> = {}): ParamRow => {
		const o = over[name] ?? {};
		return row(name, type, o.value ?? value, { ...extra, ...o });
	};
	return [
		def('rc_channel_roll', 'i32', 1),
		def('rc_channel_pitch', 'i32', 2),
		def('rc_channel_throttle', 'i32', 3),
		def('rc_channel_yaw', 'i32', 4),
		def('rc_channel_arm', 'i32', 5, { locked: true, lockInfo: LOCK }),
		def('rc_channel_angle', 'i32', 8),
		def('arm_switch_threshold', 'f32', ARM_THRESHOLD, { locked: true, lockInfo: LOCK }),
		def('angle_switch_threshold', 'f32', ARM_THRESHOLD),
		def('arm_throttle_min', 'f32', THROTTLE_MIN, { locked: true, lockInfo: LOCK }),
		def('airmode_activate_throttle', 'f32', 0.29296875, { locked: true, lockInfo: LOCK }),
		def('failsafe_recovery_us', 'i32', 500000, { locked: true, lockInfo: LOCK }),
		def('estimator_kp', 'f32', 0.25),
		...(bench ? [def('bench_profile_rc_channel', 'i32', 6), def('bench_profile_switch_threshold', 'f32', ARM_THRESHOLD)] : [])
	];
}

async function setup() {
	const clock = { t: T0 };
	const vehicle = new FakeVehicle({ params: [] });
	const session = new LinkSession(vehicle.link, { heartbeatTimeoutMs: 600_000, now: () => clock.t });
	const telemetry = new TelemetryClient(session, { now: () => clock.t });
	open.push(() => telemetry.dispose(), () => session.dispose());
	vehicle.heartbeat();
	await flush();
	const rc = async (over: Partial<typeof m.rcChannels> = {}) => {
		vehicle.emit({ ...m.rcChannels, ...over });
		await flush();
	};
	const view = (r: readonly ParamRow[] = rows(), extra: { activity?: Excursions; detection?: Detection } = {}): RadioView =>
		computeRadio({ snapshot: telemetry.snapshot(), rows: r, now: clock.t, activity: extra.activity ?? emptyExcursions(), detection: extra.detection ?? IDLE });
	return { clock, vehicle, session, telemetry, rc, view };
}

/** All 16 channels at the centre except the given ones (1-based channel -> us). */
const channels = (set: Record<number, number> = {}): Array<number | null> => Array.from({ length: 16 }, (_, i) => set[i + 1] ?? 1500);

describe('the contract', () => {
	it('x = (us - 1500) / 512 clamped, throttle = (us - 988) / 1024 clamped', () => {
		expect(normalised(1500)).toBe(0);
		expect(normalised(2012)).toBe(1);
		expect(normalised(988)).toBe(-1);
		expect(normalised(3000)).toBe(1);
		expect(normalised(1700)).toBe(ARM_THRESHOLD);
		expect(throttleOf(988)).toBe(0);
		expect(throttleOf(2012)).toBe(1);
		expect(throttleOf(1050)).toBe(THROTTLE_MIN);
	});
});

describe('the channel bars', () => {
	it('lists 16 channels in microseconds and normalised, with the controls that read each', async () => {
		const { rc, view } = await setup();
		await rc();
		const v = view();
		expect(v.bars).toHaveLength(16);
		const b = (c: number) => v.bars[c - 1]!;
		expect(b(5)).toMatchObject({ channel: 5, us: 2012, x: 1, bar: 1, roles: ['arm'] });
		expect(b(3)).toMatchObject({ us: 988, x: -1, bar: 0, roles: ['throttle'] });
		expect(b(1)).toMatchObject({ us: 1500, x: 0, bar: 0.5, roles: ['roll'] });
		expect(b(2).x).toBeCloseTo(0.5, 12);
		expect(b(8).roles).toEqual(['angle']);
		expect(b(6).roles).toEqual(['bench']);
		expect(b(9).roles).toEqual([]);
	});

	it('shows a channel the vehicle does not report as unknown, not as zero', async () => {
		const { rc, view } = await setup();
		await rc({ chancount: 4 });
		const v = view();
		expect(barOf(v, 3).us).toBe(1244);
		expect(barOf(v, 4)).toMatchObject({ us: null, x: null, bar: null });
		expect(barOf(v, 15).us).toBeNull();
	});

	it('shows the link quality, and nothing for a receiver that sends none', async () => {
		const { rc, view } = await setup();
		await rc({ rssi: 127 });
		expect(view().rc.linkQuality).toBeCloseTo(0.5, 12);
		await rc({ rssi: 255 });
		expect(view().rc.linkQuality).toBeNull();
	});

	it('is unheard before any frame, live after one, and stale once the vehicle has been silent for the timeout, keeping its values', async () => {
		const { clock, rc, view } = await setup();
		const before = view();
		expect(before.rc).toMatchObject({ heard: false, stale: true, ageMs: null });
		expect(before.bars.every((b) => b.us === null)).toBe(true);
		await rc();
		expect(view().rc).toMatchObject({ heard: true, stale: false });
		clock.t += STALE + 1;
		const later = view();
		expect(later.rc).toMatchObject({ heard: true, stale: true });
		expect(barOf(later, 4).us).toBe(2012);
	});

	it('marks a channel that has moved since the page was opened', () => {
		let activity = observe(emptyExcursions(), channels());
		activity = observe(activity, channels({ 14: 2012 }));
		activity = observe(activity, channels({ 14: 1000 }));
		expect(rangeUs(activity, 14)).toBe(1012);
		const v = computeRadio({ snapshot: emptySnapshot(), rows: rows(), now: T0, activity, detection: IDLE });
		expect(barOf(v, 13)).toMatchObject({ movedUs: 1012, active: true, detected: false });
		expect(barOf(v, 0)).toMatchObject({ movedUs: 0, active: false });
	});
});

function emptySnapshot() {
	const t = new TelemetryClient(new LinkSession(new FakeVehicle({ params: [] }).link));
	open.push(() => t.dispose());
	return t.snapshot();
}

describe('the mapping, derived from the vehicle parameters', () => {
	it('names the channel of each control, the roll to yaw sticks, the arm, angle and bench switches', async () => {
		const { rc, view } = await setup();
		await rc();
		const v = view();
		expect(v.mapKnown).toBe(true);
		expect(Object.fromEntries(v.map.map((r) => [r.id, r.active]))).toEqual({ roll: 1, pitch: 2, throttle: 3, yaw: 4, arm: 5, angle: 8, bench: 6 });
		expect(v.map.find((r) => r.id === 'arm')?.locked).toBe(true);
		expect(v.map.find((r) => r.id === 'angle')?.locked).toBe(false);
		expect(v.map.find((r) => r.id === 'angle')?.liveUs).toBe(1200);
	});

	it('is unknown until the parameters are read', async () => {
		const { rc, view } = await setup();
		await rc();
		const v = view([]);
		expect(v.mapKnown).toBe(false);
		expect(v.map.every((r) => r.active === null && !r.present)).toBe(true);
		expect(v.bars.every((b) => b.roles.length === 0)).toBe(true);
	});

	it('has no bench-profile switch in an image that does not carry the bench profile', async () => {
		const { rc, view } = await setup();
		await rc();
		const v = view(rows({}, false));
		expect(v.mapKnown).toBe(true);
		expect(v.map.find((r) => r.id === 'bench')).toMatchObject({ present: false, active: null });
		expect(v.switches.map((s) => s.id)).toEqual(['arm', 'angle']);
		expect(barOf(v, 5).roles).toEqual([]);
	});

	it('shows a staged remap beside the running one, the controls moving to the staged channel', async () => {
		const { rc, view } = await setup();
		await rc();
		const v = view(rows({ rc_channel_angle: { staged: 14 } }));
		const angle = v.map.find((r) => r.id === 'angle');
		expect(angle).toMatchObject({ active: 8, staged: 14, differs: true });
		expect(barOf(v, 7)).toMatchObject({ roles: ['angle'], stagedRoles: [] });
		expect(barOf(v, 13)).toMatchObject({ roles: [], stagedRoles: ['angle'] });
		// The thresholds still read the running config's channel: that is what the vehicle uses now.
		expect(v.switches.find((s) => s.id === 'angle')?.channel).toBe(8);
	});
});

describe('rule R-1: a collision is a warning before staging', () => {
	it('the default map has no finding, and angle 0 (no switch) is not compared', () => {
		expect(findingsOf({ roll: 1, pitch: 2, throttle: 3, yaw: 4, arm: 5, angle: 8, bench: 6 })).toEqual([]);
		expect(findingsOf({ roll: 1, pitch: 2, throttle: 3, yaw: 4, arm: 5, angle: 0 })).toEqual([]);
	});

	it('names the controls that share a channel: sticks, the arm switch and the angle switch', () => {
		const f = findingsOf({ roll: 5, pitch: 2, throttle: 3, yaw: 4, arm: 5, angle: 8 });
		expect(f).toHaveLength(1);
		expect(f[0]).toMatchObject({ kind: 'collision', functions: ['roll', 'arm'] });
		expect(f[0]!.text).toContain('Channel 5 is used by Roll and Arm switch');
		expect(findingsOf({ roll: 1, pitch: 2, throttle: 3, yaw: 4, arm: 5, angle: 3 })[0]!.functions).toEqual(['throttle', 'angle']);
		expect(findingsOf({ roll: 1, pitch: 2, throttle: 3, yaw: 4, arm: 5, angle: 8, bench: 8 })[0]!.functions).toEqual(['angle', 'bench']);
	});

	it('flags a channel outside 1 .. 16 (the angle channel outside 0 .. 16)', () => {
		expect(findingsOf({ roll: 17 }).map((f) => f.kind)).toEqual(['range']);
		expect(findingsOf({ roll: 0 }).map((f) => f.kind)).toEqual(['range']);
		expect(findingsOf({ angle: 0 })).toEqual([]);
		expect(findingsOf({ angle: 17 }).map((f) => f.kind)).toEqual(['range']);
		expect(findingsOf({ angle: 2.5 }).map((f) => f.kind)).toEqual(['range']);
	});

	it('warns about a candidate value before it is staged, and only about the control being set', () => {
		const map = { roll: 1, pitch: 2, throttle: 3, yaw: 4, arm: 5, angle: 8 };
		expect(candidateFindings(map, 'angle', 14)).toEqual([]);
		const f = candidateFindings(map, 'angle', 5);
		expect(f).toHaveLength(1);
		expect(f[0]!.functions).toEqual(['arm', 'angle']);
		// A collision that exists elsewhere is not blamed on this control.
		expect(candidateFindings({ ...map, pitch: 1 }, 'angle', 14)).toEqual([]);
		// A swap in one step collides (the vehicle refuses it): the warning says which.
		expect(candidateFindings(map, 'roll', 2)[0]!.functions).toEqual(['roll', 'pitch']);
	});

	it('reports the findings of the running map and of the staged map separately', async () => {
		const { rc, view } = await setup();
		await rc();
		const v = view(rows({ rc_channel_angle: { staged: 5 } }));
		expect(v.findingsActive).toEqual([]);
		expect(v.findingsStaged).toHaveLength(1);
		expect(v.findingsStaged[0]!.functions).toEqual(['arm', 'angle']);
	});
});

describe('detection: move the switch you want', () => {
	it('stays listening while nothing moves, and ignores the scatter of a resting stick', () => {
		let d: Detection = beginDetection('angle');
		d = stepDetection(d, channels());
		d = stepDetection(d, channels({ 1: 1520, 2: 1480 }));
		expect(d.phase).toBe('listening');
	});

	it('finds the one channel that moved by a switch travel, wherever the switch started', () => {
		let d: Detection = beginDetection('angle');
		d = stepDetection(d, channels({ 14: 2012 }));
		expect(d.phase).toBe('listening');
		d = stepDetection(d, channels({ 14: 988 }));
		expect(d).toMatchObject({ phase: 'found', fn: 'angle', channel: 14, moved: 1024 });
	});

	it('finds a channel at the threshold of movement and not below it', () => {
		const d = stepDetection(beginDetection('arm'), channels({ 9: 1500 }));
		expect(stepDetection(d, channels({ 9: 1500 + MOVED_US - 1 })).phase).toBe('listening');
		expect(stepDetection(d, channels({ 9: 1500 + MOVED_US })).phase).toBe('found');
	});

	it('keeps its answer once found: a later movement of another channel does not change it', () => {
		let d = stepDetection(beginDetection('angle'), channels());
		d = stepDetection(d, channels({ 14: 2012 }));
		d = stepDetection(d, channels({ 14: 988 }));
		const found = d;
		d = stepDetection(d, channels({ 7: 988 }));
		d = stepDetection(d, channels({ 7: 2012 }));
		expect(d).toBe(found);
	});

	it('says so when two channels moved in the same frame, and names them', () => {
		let d = stepDetection(beginDetection('angle'), channels());
		d = stepDetection(d, channels({ 3: 988, 12: 2012 }));
		expect(d).toMatchObject({ phase: 'ambiguous', channels: [3, 12] });
	});

	it('highlights the channel that moved in the bars, and an idle detection highlights none', () => {
		let d = stepDetection(beginDetection('angle'), channels());
		d = stepDetection(d, channels({ 14: 2012 }));
		d = stepDetection(d, channels({ 14: 1000 }));
		const v = computeRadio({ snapshot: emptySnapshot(), rows: rows(), now: T0, activity: emptyExcursions(), detection: d });
		expect(v.bars.filter((b) => b.detected).map((b) => b.channel)).toEqual([14]);
		const idle = computeRadio({ snapshot: emptySnapshot(), rows: rows(), now: T0, activity: emptyExcursions(), detection: IDLE });
		expect(idle.bars.some((b) => b.detected)).toBe(false);
	});

	it('does nothing for a channel the vehicle does not report', () => {
		let d = stepDetection(beginDetection('angle'), channels());
		d = stepDetection(d, channels().map((v, i) => (i === 13 ? null : v)));
		expect(d.phase).toBe('listening');
	});
});

describe('hints: a switch that moved on a channel nothing uses', () => {
	it('names the unmapped channel that moved while the angle switch has not (the ch14 / ch8 case)', () => {
		let activity = observe(emptyExcursions(), channels());
		activity = observe(activity, channels({ 14: 2012 }));
		activity = observe(activity, channels({ 14: 988 }));
		const v = computeRadio({ snapshot: emptySnapshot(), rows: rows(), now: T0, activity, detection: IDLE });
		expect(v.hints).toHaveLength(1);
		expect(v.hints[0]!).toContain('Channel 14 moved by 1024 µs and no control uses it');
		expect(v.hints[0]!).toContain('the arm switch (channel 5) and the angle switch (channel 8) has not moved');
	});

	it('says nothing once the switch itself has moved, or when the moved channel is used', () => {
		let activity = observe(emptyExcursions(), channels());
		activity = observe(activity, channels({ 14: 2012, 8: 2012, 5: 2012 }));
		activity = observe(activity, channels({ 14: 988, 8: 988, 5: 988 }));
		expect(computeRadio({ snapshot: emptySnapshot(), rows: rows(), now: T0, activity, detection: IDLE }).hints).toEqual([]);
		let onlyUsed = observe(emptyExcursions(), channels());
		onlyUsed = observe(onlyUsed, channels({ 5: 2012 }));
		onlyUsed = observe(onlyUsed, channels({ 5: 988 }));
		expect(computeRadio({ snapshot: emptySnapshot(), rows: rows(), now: T0, activity: onlyUsed, detection: IDLE }).hints).toEqual([]);
	});

	it('does not count a channel a staged remap would use as unmapped', () => {
		let activity = observe(emptyExcursions(), channels());
		activity = observe(activity, channels({ 14: 2012 }));
		activity = observe(activity, channels({ 14: 988 }));
		const v = computeRadio({ snapshot: emptySnapshot(), rows: rows({ rc_channel_angle: { staged: 14 } }), now: T0, activity, detection: IDLE });
		expect(v.hints).toEqual([]);
	});
});

describe('the switch thresholds against the live channel', () => {
	const at = async (us5: number, us8: number, r: readonly ParamRow[] = rows()) => {
		const { rc, view } = await setup();
		await rc({ chan5_raw: us5, chan8_raw: us8 });
		const v = view(r);
		return { arm: v.switches.find((s) => s.id === 'arm')!, angle: v.switches.find((s) => s.id === 'angle')!, v };
	};

	it('is above the threshold at +100 and below at -100 (ch5 and ch8 as the register has them)', async () => {
		const high = await at(2012, 988);
		expect(high.arm).toMatchObject({ channel: 5, position: 'at or above', threshold: ARM_THRESHOLD, thresholdUs: 1700, us: 2012, x: 1 });
		expect(high.angle).toMatchObject({ channel: 8, position: 'below', us: 988 });
		const low = await at(988, 2012);
		expect(low.arm.position).toBe('below');
		expect(low.angle.position).toBe('at or above');
		expect(low.arm.text).toContain('OFF now');
		expect(high.arm.text).toContain('ON now');
	});

	it('uses the arming comparison: at the threshold counts as on, one microsecond under does not', async () => {
		expect((await at(1700, 988)).arm.position).toBe('at or above');
		expect((await at(1699, 988)).arm.position).toBe('below');
	});

	it('says there is no angle channel when it is 0', async () => {
		const { angle } = await at(2012, 2012, rows({ rc_channel_angle: { value: 0 } }));
		expect(angle).toMatchObject({ channel: 0, position: 'none' });
	});

	it('is unknown without the parameters, and with a channel the vehicle does not report', async () => {
		expect((await at(2012, 2012, [])).arm.position).toBe('unknown');
		const { rc, view } = await setup();
		await rc({ chancount: 4 });
		expect(view().switches.find((s) => s.id === 'arm')).toMatchObject({ position: 'unknown', us: null });
	});

	it('shows the bench-profile switch against its own threshold when the image has it', async () => {
		const { rc, view } = await setup();
		await rc({ chan6_raw: 2012 });
		expect(view().switches.find((s) => s.id === 'bench')).toMatchObject({ channel: 6, position: 'at or above' });
		await rc({ chan6_raw: 1500 });
		expect(view().switches.find((s) => s.id === 'bench')?.position).toBe('below');
	});
});

describe('the display checks', () => {
	const check = (v: RadioView, id: string) => v.checks.find((c) => c.id === id)!;

	it('sticks are centred within the tolerance, and not outside it', async () => {
		const { rc, view } = await setup();
		const edge = 1500 + Math.floor(CENTRE_TOLERANCE * 512);
		await rc({ chan1_raw: 1500, chan2_raw: edge, chan4_raw: edge + 2 });
		const v = view();
		expect(check(v, 'centre.roll').status).toBe('ok');
		expect(check(v, 'centre.pitch').status).toBe('ok');
		expect(check(v, 'centre.yaw').status).toBe('fail');
		expect(check(v, 'centre.yaw').detail).toContain('PLACEHOLDER');
	});

	it('is a negative control: a stick held off centre fails and the default frame of the test messages (pitch 1756) fails', async () => {
		const { rc, view } = await setup();
		await rc();
		expect(check(view(), 'centre.pitch').status).toBe('fail');
		expect(check(view(), 'centre.roll').status).toBe('ok');
	});

	it('the throttle is low when below the arming minimum, from the vehicle\'s own parameter', async () => {
		const { rc, view } = await setup();
		await rc({ chan3_raw: 988 });
		expect(check(view(), 'throttle.low').status).toBe('ok');
		await rc({ chan3_raw: 1049 });
		expect(check(view(), 'throttle.low').status).toBe('ok');
		// The arming rule is "below": exactly arm_throttle_min is not low.
		await rc({ chan3_raw: 1050 });
		expect(check(view(), 'throttle.low').status).toBe('fail');
		await rc({ chan3_raw: 1500 });
		expect(check(view(), 'throttle.low').status).toBe('fail');
	});

	it('follows a remapped stick channel, and is unknown without the parameters or the channel', async () => {
		const { rc, view } = await setup();
		await rc({ chan3_raw: 2000, chan9_raw: 988 });
		expect(check(view(rows({ rc_channel_throttle: { value: 9 } })), 'throttle.low').status).toBe('ok');
		expect(check(view([]), 'throttle.low').status).toBe('unknown');
		expect(check(view([]), 'centre.roll').status).toBe('unknown');
		await rc({ chancount: 2 });
		expect(check(view(), 'centre.yaw').status).toBe('unknown');
		expect(check(view(rows().filter((r) => r.name !== 'arm_throttle_min')), 'throttle.low').status).toBe('unknown');
	});
});

describe('the lock display', () => {
	it('lists the locked radio parameters with their ruling, and the unlocked ones as not locked', async () => {
		const { rc, view } = await setup();
		await rc();
		const locks = view().locks;
		const by = Object.fromEntries(locks.map((l) => [l.name, l]));
		expect(by.rc_channel_arm!.locked).toBe(true);
		expect(by.rc_channel_arm!.text).toBe('locked by Luis 2026-10-17 (manual); change only by a firmware build');
		expect(by.arm_switch_threshold!.locked).toBe(true);
		expect(by.arm_throttle_min!.locked).toBe(true);
		expect(by.airmode_activate_throttle!.locked).toBe(true);
		expect(by.failsafe_recovery_us!.locked).toBe(true);
		expect(by.rc_channel_angle).toMatchObject({ locked: false, text: 'not locked' });
		expect(by.rc_channel_roll!.locked).toBe(false);
		expect(by.estimator_kp).toBeUndefined();
	});
});

describe('the controller', () => {
	it('asks for the streams on start, follows the frames into the detection and the activity, and stops the streams on stop', async () => {
		const { clock, vehicle, telemetry } = await setup();
		const r = writable<readonly ParamRow[]>(rows());
		const radio = new RadioController({ telemetry, rows: r, now: () => clock.t });
		const sent = () => vehicle.received.filter((x) => x.kind === 'CommandLong' && x.command === 511);
		expect(sent()).toHaveLength(0);
		radio.start();
		expect(sent().length).toBeGreaterThan(0);
		expect(get(radio.view)?.rc.heard).toBe(false);

		const frame = async (set: Record<number, number>) => {
			const raw = Object.fromEntries(Object.entries(set).map(([c, us]) => [`chan${c}_raw`, us]));
			vehicle.emit({ ...m.rcChannels, ...Object.fromEntries(Array.from({ length: 16 }, (_, i) => [`chan${i + 1}_raw`, 1500])), ...raw });
			await flush();
			radio.tick();
		};
		await frame({});
		radio.detect('angle');
		expect(get(radio.view)?.detection.phase).toBe('listening');
		await frame({ 14: 2012 });
		await frame({ 14: 988 });
		expect(get(radio.view)?.detection).toMatchObject({ phase: 'found', channel: 14 });
		expect(get(radio.view)?.bars[13]?.active).toBe(true);
		expect(get(radio.view)?.hints[0]!).toContain('Channel 14');

		radio.cancelDetection();
		expect(get(radio.view)?.detection.phase).toBe('idle');
		radio.resetActivity();
		expect(get(radio.view)?.bars[13]?.active).toBe(false);

		// The control: with the vehicle silent the view turns stale as the clock passes the timeout.
		expect(get(radio.view)?.rc.stale).toBe(false);
		clock.t += STALE + 1;
		radio.tick();
		expect(get(radio.view)?.rc.stale).toBe(true);

		radio.stop();
		expect(sent().slice(-9).every((x) => x.kind === 'CommandLong' && x.param2 === -1)).toBe(true);
		const count = sent().length;
		radio.tick();
		expect(sent()).toHaveLength(count);
	});

	it('forgets what moved when the connection changes (the telemetry counters restart)', async () => {
		const { clock, vehicle, telemetry } = await setup();
		const radio = new RadioController({ telemetry, rows: writable<readonly ParamRow[]>(rows()), now: () => clock.t });
		radio.start();
		vehicle.emit({ ...m.rcChannels, chan14_raw: 988 });
		await flush();
		radio.tick();
		vehicle.emit({ ...m.rcChannels, chan14_raw: 2012 });
		await flush();
		radio.tick();
		expect(get(radio.view)?.bars[13]?.active).toBe(true);
		telemetry.reset();
		radio.tick();
		expect(get(radio.view)?.bars[13]?.active).toBe(false);
		radio.stop();
	});
});
