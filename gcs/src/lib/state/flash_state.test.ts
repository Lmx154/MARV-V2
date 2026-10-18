// The Flash flow against the fake vehicle and a fake helper (ground-segment unit 15): nothing reaches the vehicle or the helper's
// flash without a passing gate and a disarmed vehicle; the flash asks for the maintenance grant first; the identity read back after
// the flash is compared with the plan's expectation, and a mismatch is reported.
import { afterEach, describe, expect, it } from 'vitest';
import { get, readable } from 'svelte/store';
import { LinkSession } from '$link/session';
import { FakeVehicle, type FakeVehicleOptions } from '$link/testing/fake_vehicle';
import type { HelperEvent, HelperReply } from '$link/transport';
import { VehicleClient } from '$link/vehicle';
import { fakeParams } from '$lib/params/testing/fixture';
import type { FlashPlan } from '$lib/flash/flash';
import { FlashController } from './flash_state';

const flush = () => new Promise((r) => setTimeout(r, 0));
const wait = (ms: number) => new Promise((r) => setTimeout(r, ms));
const TIMING = { replyMs: 150, grantMs: 150, listingIdleMs: 60, listingRounds: 2, refusalGraceMs: 20, sourceAttempts: 2 };
const OLD_IMAGE = 'ab'.repeat(32);
const NEW_IMAGE = '12'.repeat(32);
const CONFIG = 'cd'.repeat(32);

const open: Array<() => void> = [];
afterEach(() => {
	while (open.length) open.pop()?.();
});

interface Setup {
	armed?: boolean;
	rebootAnswer?: FakeVehicleOptions['rebootAnswer'];
	/** The gate of the plan: passes, or the helper's reason. */
	gate?: { passes: boolean; reason: string | null };
	/** What the vehicle reports after the flash. */
	flashed?: { image: Uint8Array; config?: Uint8Array };
	flashOk?: boolean;
	flashRefuses?: boolean;
	/** The plan the helper gives the second time (the report store changed under the page). */
	secondGate?: { passes: boolean; reason: string | null };
}

async function setup(o: Setup = {}) {
	const options: FakeVehicleOptions = { params: fakeParams(), armed: o.armed ?? false, rebootAnswer: o.rebootAnswer, imageSha256: Uint8Array.from({ length: 32 }, () => 0xab), configSha256: Uint8Array.from({ length: 32 }, () => 0xcd) };
	const vehicle = new FakeVehicle(options);
	const session = new LinkSession(vehicle.link, { heartbeatTimeoutMs: 60_000 });
	const client = new VehicleClient(session, { timing: TIMING });
	open.push(() => session.dispose(), () => client.dispose());
	vehicle.heartbeat();
	await flush();
	await client.requestAll();
	await flush();

	const handlers = new Set<(e: HelperEvent) => void>();
	const requests: Array<{ cmd: string; args: Record<string, unknown> }> = [];
	const order: string[] = [];
	let plans = 0;
	const emit = (e: HelperEvent) => {
		for (const h of handlers) h(e);
	};
	const planOf = (gate: { passes: boolean; reason: string | null }): FlashPlan => ({
		image: { hash: NEW_IMAGE, tree: 'm33', name: 'marv_fc3v2', schema_hash: '0x3ae00eb50938acba' },
		config: { which: 'factory', hash: CONFIG, expected_source: 'factory', why: 'the board runs its factory table' },
		gate: { ...gate, report_id: gate.passes ? 'ee'.repeat(32) : null, verdict: gate.passes ? 'PASS' : null },
		can_validate: true,
		allowed: gate.passes,
		refusal: gate.passes ? null : gate.reason
	});
	const request = async (cmd: string, args: Record<string, unknown> = {}): Promise<HelperReply> => {
		requests.push({ cmd, args });
		order.push(cmd);
		if (cmd === 'flash_plan') {
			plans++;
			const gate = plans > 1 && o.secondGate ? o.secondGate : (o.gate ?? { passes: true, reason: null });
			return { ok: true, plan: planOf(gate) };
		}
		if (cmd === 'flash') {
			if (o.flashRefuses) throw new Error('not flashed, picotool was not started: no validation report');
			setTimeout(async () => {
				// the helper gives the port up, picotool runs, the port is opened again: the GCS resets the vehicle at both events (app.ts)
				emit({ event: 'device_closed', reason: 'flashing' });
				client.reset();
				emit({ event: 'flash_progress', image_hash: NEW_IMAGE, stage: 'picotool', line: 'Verifying Flash: OK' });
				if (o.flashOk === false) {
					emit({ event: 'flash_done', image_hash: NEW_IMAGE, ok: false, stage: 'picotool', error: 'picotool load exited with status 1' });
					return;
				}
				emit({ event: 'device_opened', device: 'serial /dev/ttyACM1 115200' });
				client.reset();
				if (o.flashed) {
					options.imageSha256 = o.flashed.image;
					if (o.flashed.config) options.configSha256 = o.flashed.config;
				}
				emit({ event: 'flash_done', image_hash: NEW_IMAGE, ok: true, stage: 'done', error: null });
				vehicle.heartbeat();
			}, 5);
			return { ok: true, started: true, expected: { image_hash: NEW_IMAGE, config_hash: CONFIG, config_source: 'factory' } };
		}
		return { ok: true };
	};
	const controller = new FlashController({
		request,
		onEvent: (h) => {
			handlers.add(h);
			return () => handlers.delete(h);
		},
		vehicle: client,
		session,
		helperState: readable('open'),
		device: readable('serial /dev/ttyACM1 115200'),
		timing: { flashDoneMs: 2000, identityMs: 1500, identityRetryMs: 100 }
	});
	// the vehicle reports its identity at the first heartbeat (VehicleClient); the fake answers
	await wait(20);
	return { vehicle, client, session, controller, requests, order, options };
}

const grants = (v: FakeVehicle) => v.received.filter((m) => m.kind === 'CommandLong' && m.command === 246);
const hex = (b: Uint8Array) => Array.from(b, (x) => x.toString(16).padStart(2, '0')).join('');

describe('the Flash button', () => {
	it('is off until the gate has been checked and passes for this vehicle, and flash() sends nothing while it is off', async () => {
		const { controller, requests, vehicle } = await setup();
		expect(get(controller.whyCannot)).toBe('pick an image');
		controller.setImage(NEW_IMAGE);
		expect(get(controller.whyCannot)).toContain('not been checked');
		await controller.flash();
		expect(get(controller.state).error).toContain('not sent');
		expect(requests.map((r) => r.cmd)).toEqual([]);
		expect(grants(vehicle)).toHaveLength(0);
		await controller.check();
		expect(get(controller.whyCannot)).toBeNull();
	});

	it('stays off without a passing report and names the reason', async () => {
		const { controller, requests, vehicle } = await setup({ gate: { passes: false, reason: 'no validation report for image 121212121212 with config cdcdcdcdcdcd' } });
		controller.setImage(NEW_IMAGE);
		await controller.check();
		expect(get(controller.whyCannot)).toContain('no validation report');
		await controller.flash();
		expect(requests.map((r) => r.cmd)).toEqual(['flash_plan']);
		expect(grants(vehicle)).toHaveLength(0);
	});

	it('is off while the vehicle is armed and flash() sends neither the grant request nor the helper flash', async () => {
		const { controller, requests, vehicle, session } = await setup();
		controller.setImage(NEW_IMAGE);
		await controller.check();
		vehicle.armed = true;
		vehicle.heartbeat();
		await flush();
		expect(get(session.state).armed).toBe(true);
		expect(get(controller.whyCannot)).toContain('armed');
		await controller.flash();
		expect(requests.map((r) => r.cmd)).toEqual(['flash_plan']);
		expect(grants(vehicle)).toHaveLength(0);
	});
});

describe('flash()', () => {
	it('re-checks the gate, asks for the maintenance grant, flashes, waits for the vehicle and reads the image hash back', async () => {
		const { controller, requests, order, vehicle } = await setup({ flashed: { image: Uint8Array.from({ length: 32 }, () => 0x12) } });
		controller.setImage(NEW_IMAGE);
		await controller.check();
		await controller.flash();
		const s = get(controller.state);
		expect(s.error).toBeNull();
		expect(s.phase).toBe('done');
		expect(s.result?.verified).toBe(true);
		expect(s.result?.checks.map((c) => c.ok)).toEqual([true, true, true]);
		expect(s.progress).toContain('picotool: Verifying Flash: OK');
		expect(order).toEqual(['flash_plan', 'flash_plan', 'flash']);
		const g = grants(vehicle);
		expect(g).toHaveLength(1);
		expect(g[0]).toMatchObject({ command: 246, param1: 3 });
		expect(requests[2]?.args).toMatchObject({ image_hash: NEW_IMAGE, board: { armed: false, config_source: 'factory' } });
		expect(s.inMaintenance).toBe(false);
	});

	it('reports a mismatch when the vehicle reports another image hash after the flash', async () => {
		const { controller } = await setup({ flashed: { image: Uint8Array.from({ length: 32 }, () => 0x99) } });
		controller.setImage(NEW_IMAGE);
		await controller.check();
		await controller.flash();
		const s = get(controller.state);
		expect(s.phase).toBe('failed');
		expect(s.result?.verified).toBe(false);
		expect(s.result?.checks.filter((c) => !c.ok).map((c) => c.what)).toEqual(['image hash']);
		expect(s.result?.checks[0]?.got).toBe(hex(Uint8Array.from({ length: 32 }, () => 0x99)));
		expect(s.error).toContain('image hash');
	});

	it('reports a mismatch of the config hash as well', async () => {
		const { controller } = await setup({ flashed: { image: Uint8Array.from({ length: 32 }, () => 0x12), config: Uint8Array.from({ length: 32 }, () => 0x77) } });
		controller.setImage(NEW_IMAGE);
		await controller.check();
		await controller.flash();
		const s = get(controller.state);
		expect(s.result?.verified).toBe(false);
		expect(s.result?.checks.filter((c) => !c.ok).map((c) => c.what)).toEqual(['config hash']);
	});

	it('does not ask for the grant when the second gate check fails (the report store changed), and never flashes', async () => {
		const { controller, order, vehicle } = await setup({ secondGate: { passes: false, reason: 'the stored report says FAIL' } });
		controller.setImage(NEW_IMAGE);
		await controller.check();
		await controller.flash();
		expect(get(controller.state).error).toContain('the stored report says FAIL');
		expect(order).toEqual(['flash_plan', 'flash_plan']);
		expect(grants(vehicle)).toHaveLength(0);
	});

	it('does not flash when the vehicle denies the grant or does not answer, and says nothing was flashed', async () => {
		for (const [rebootAnswer, word] of [[2, 'refused'], ['silent', 'no answer'], [3, 'unsupported']] as const) {
			const { controller, order, vehicle } = await setup({ rebootAnswer });
			controller.setImage(NEW_IMAGE);
			await controller.check();
			await controller.flash();
			const s = get(controller.state);
			expect(s.phase).toBe('failed');
			expect(s.error).toContain(word);
			expect(s.error).toContain('Nothing was flashed');
			expect(order).not.toContain('flash');
			expect(grants(vehicle)).toHaveLength(1);
			expect(s.inMaintenance).toBe(false);
			while (open.length) open.pop()?.();
		}
	});

	it('tells that the vehicle stays in maintenance when the helper refuses after the grant, and when the flash fails', async () => {
		const refused = await setup({ flashRefuses: true });
		refused.controller.setImage(NEW_IMAGE);
		await refused.controller.check();
		await refused.controller.flash();
		expect(get(refused.controller.state).error).toContain('picotool was not started');
		expect(get(refused.controller.state).error).toContain('maintenance grant');
		expect(get(refused.controller.state).inMaintenance).toBe(true);
		const failed = await setup({ flashOk: false });
		failed.controller.setImage(NEW_IMAGE);
		await failed.controller.check();
		await failed.controller.flash();
		expect(get(failed.controller.state).error).toContain('picotool load exited with status 1');
		expect(get(failed.controller.state).result).toBeNull();
	});
});
