// The validate and activate flow of the Parameters page against the fake vehicle and a fake helper: sources are read in full, the view
// goes to the helper, the report comes back and is verified, and the activation command goes out only through the gate.
import { afterEach, describe, expect, it } from 'vitest';
import { get } from 'svelte/store';
import { f32ToBits } from '$link/bits';
import { LinkSession } from '$link/session';
import { FakeVehicle } from '$link/testing/fake_vehicle';
import type { HelperEvent, HelperReply } from '$link/transport';
import { VehicleClient } from '$link/vehicle';
import { parseManifest } from '$lib/params/manifest';
import { fakeParams, LONG_SOURCE, manifestJson } from '$lib/params/testing/fixture';
import { sha256Hex } from '$lib/params/validation';
import { ValidationController } from './validation_state';

const flush = () => new Promise((r) => setTimeout(r, 0));
const TIMING = { replyMs: 150, listingIdleMs: 60, listingRounds: 2, refusalGraceMs: 20, sourceAttempts: 2 };
const manifest = parseManifest(manifestJson());
const IMAGE = 'ab'.repeat(32);

const open: Array<() => void> = [];
afterEach(() => {
	while (open.length) open.pop()?.();
});

async function setup(verdict: 'PASS' | 'FAIL' = 'PASS') {
	const vehicle = new FakeVehicle({ params: fakeParams() });
	const session = new LinkSession(vehicle.link, { heartbeatTimeoutMs: 60_000 });
	const client = new VehicleClient(session, { timing: TIMING });
	open.push(() => session.dispose(), () => client.dispose());
	vehicle.heartbeat();
	await flush();
	await client.requestAll();

	const handlers = new Set<(e: HelperEvent) => void>();
	const reports = new Map<string, Uint8Array>();
	const requests: Array<{ cmd: string; args: Record<string, unknown> }> = [];
	let reportOverride: ((bytes: Uint8Array) => Uint8Array) | null = null;
	const hashOf = (view: unknown) => sha256Hex(new TextEncoder().encode(JSON.stringify(view)));
	const deps = {
		request: async (cmd: string, args: Record<string, unknown> = {}): Promise<HelperReply> => {
			requests.push({ cmd, args });
			const configHash = await hashOf(args.view);
			if (cmd === 'config_report') return { ok: true, config_hash: configHash, stored: false, verdict: null };
			const bytes = new TextEncoder().encode(
				JSON.stringify({
					format: 'marv-validation-report/1', verdict,
					image: { sha256: args.image_hash }, config: { config_hash: configHash },
					checks: [{ id: 'records', title: 'record checks', result: verdict, values: ['137 records'], failures: verdict === 'PASS' ? [] : ['rate roll margin -5.7 deg'] }]
				}) + '\n'
			);
			const id = await sha256Hex(bytes);
			reports.set(`${args.image_hash}/${configHash}`, bytes);
			setTimeout(() => {
				for (const h of handlers) h({ event: 'validate_progress', image_hash: String(args.image_hash), config_hash: configHash, line: 'validate_config: records: PASS' });
				for (const h of handlers) h({ event: 'validate_done', image_hash: String(args.image_hash), config_hash: configHash, verdict, report_id: id, exit_status: 0 });
			}, 5);
			return { ok: true, config_hash: configHash, started: true };
		},
		onEvent: (h: (e: HelperEvent) => void) => {
			handlers.add(h);
			return () => handlers.delete(h);
		},
		report: async (image: string, config: string) => {
			const bytes = reports.get(`${image}/${config}`);
			if (!bytes) throw new Error('no report');
			return reportOverride ? reportOverride(bytes) : bytes;
		},
		vehicle: client,
		manifest: () => manifest
	};
	const controller = new ValidationController(deps);
	const why = () => controller.whyCannotActivate(get(client.state), get(controller.state), get(controller.fingerprint));
	return { vehicle, client, controller, requests, why, tamper: (f: (b: Uint8Array) => Uint8Array) => (reportOverride = f) };
}

const configCommands = (v: FakeVehicle) => v.received.filter((m) => m.kind === 'MarvConfigCommand');

describe('validate and activate', () => {
	it('reads every long source, validates the view, verifies the report and activates with its id and the config hash', async () => {
		const { vehicle, controller, requests, why } = await setup();
		expect(why()).toContain('no validation report');
		await controller.validate();
		const s = get(controller.state);
		expect(s.error).toBeNull();
		expect(s.phase).toBe('done');
		expect(s.progress).toEqual(['validate_config: records: PASS']);
		expect(s.record?.report?.verdict).toBe('PASS');
		expect(s.record?.reportId).toBe(s.record?.computedId);
		const view = requests[0]!.args.view as { sections: Array<{ records: Array<{ source: string }> }> };
		expect(view.sections[0]!.records[1]!.source).toBe(LONG_SOURCE);
		expect(requests[0]!.args.image_hash).toBe(IMAGE);
		expect(why()).toBeNull();

		await controller.activate();
		const sent = configCommands(vehicle);
		expect(sent).toHaveLength(1);
		const command = sent[0]! as unknown as { report_id: Uint8Array; expected_config_sha256: Uint8Array };
		expect(Buffer.from(command.report_id).toString('hex')).toBe(s.record?.reportId);
		expect(Buffer.from(command.expected_config_sha256).toString('hex')).toBe(s.record?.configHash);
		expect(get(controller.state).command).toMatchObject({ action: 'activate' });
	});

	it('refuses the activation of a FAIL report and sends nothing', async () => {
		const { vehicle, controller, why } = await setup('FAIL');
		await controller.validate();
		expect(get(controller.state).record?.report?.verdict).toBe('FAIL');
		expect(why()).toContain('says FAIL');
		await controller.activate();
		expect(configCommands(vehicle)).toHaveLength(0);
		expect(get(controller.state).command).toMatchObject({ result: 'error', text: expect.stringContaining('says FAIL') });
	});

	it('refuses to trust a report whose bytes are not what the helper announced', async () => {
		const { vehicle, controller, why, tamper } = await setup();
		tamper((b) => b.slice(0, b.length - 1));
		await controller.validate();
		expect(get(controller.state).error).toContain('SHA-256');
		expect(why()).toContain('SHA-256');
		await controller.activate();
		expect(configCommands(vehicle)).toHaveLength(0);
	});

	it('refuses once the staged config changed after the validation', async () => {
		const { vehicle, client, controller, why } = await setup();
		await controller.validate();
		expect(why()).toBeNull();
		const staged = await client.stage('rate_kp', 'f32', f32ToBits(13), 'a test change');
		expect(staged.accepted).toBe(true);
		expect(why()).toContain('changed since it was validated');
		await controller.activate();
		expect(configCommands(vehicle)).toHaveLength(0);
	});

	it('revert and reset send the vehicle command and drop the report', async () => {
		const { vehicle, controller } = await setup();
		await controller.validate();
		expect(get(controller.state).record).not.toBeNull();
		await controller.command('revert');
		expect(configCommands(vehicle)).toHaveLength(1);
		expect(get(controller.state).record).toBeNull();
		await controller.command('reset-factory');
		expect(configCommands(vehicle)).toHaveLength(2);
	});

	it('says why a validation could not start', async () => {
		const { client } = await setup();
		const c = new ValidationController({
			request: async () => ({ ok: true }),
			onEvent: () => () => undefined,
			report: async () => new Uint8Array(),
			vehicle: client,
			manifest: () => null
		});
		await c.validate();
		expect(get(c.state)).toMatchObject({ phase: 'failed', error: expect.stringContaining('no manifest') });
	});
});
