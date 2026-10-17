// The vehicle client against the fake vehicle (src/lib/link/testing/fake_vehicle.ts): the identity, the parameter listing with
// provenance records, source reads in chunks, staging with the source text and the refusals of decision 0053 Q1 and Q2.
import { afterEach, describe, expect, it } from 'vitest';
import { get } from 'svelte/store';
import { bitsToWireNumber, f32ToBits, i32ToBits } from './bits';
import { LinkSession } from './session';
import { DEFAULT_SOURCE, FakeVehicle } from './testing/fake_vehicle';
import { NO_SOURCE_ERROR, VehicleClient } from './vehicle';
import { fakeParams, LONG_SOURCE, SPECS } from '../params/testing/fixture';

const flush = () => new Promise((r) => setTimeout(r, 0));
const TIMING = { replyMs: 150, listingIdleMs: 60, listingRounds: 2, refusalGraceMs: 20 };

const open: Array<() => void> = [];
afterEach(() => {
	while (open.length) open.pop()?.();
});

async function setup(options: { armed?: boolean; answersIdentity?: boolean; capabilities?: bigint } = {}) {
	const vehicle = new FakeVehicle({ params: fakeParams(), ...options });
	const session = new LinkSession(vehicle.link, { heartbeatTimeoutMs: 60_000 });
	const client = new VehicleClient(session, { timing: TIMING });
	open.push(() => session.dispose(), () => client.dispose());
	vehicle.heartbeat();
	await flush();
	return { vehicle, session, client };
}

const index = (linkId: string): number => SPECS.findIndex((s) => s.linkId === linkId);
const row = (client: VehicleClient, linkId: string) => get(client.state).params.find((p) => p.linkId === linkId)!;
const setsSent = (v: FakeVehicle) => v.received.filter((m) => m.kind === 'ParamSet');

describe('identity and capabilities', () => {
	it('asks for the identity and the version when the first heartbeat arrives', async () => {
		const { client } = await setup();
		const s = get(client.state);
		expect(s.identity).toMatchObject({
			schemaHash: '0x3ae00eb50938acba',
			imageSha256: 'ab'.repeat(32),
			configSha256: 'cd'.repeat(32),
			validationReportId: null,
			configSource: 'factory',
			configState: 'ok',
			configRefusal: 'none',
			stagedDiffers: false
		});
		expect(s.bytewise).toBe(true);
	});

	it('reads a missing byte-wise capability as false', async () => {
		const { client } = await setup({ capabilities: 2n });
		expect(get(client.state).bytewise).toBe(false);
	});
});

describe('the parameter listing', () => {
	it('collects every value and record, bit-exact for REAL32 and INT32', async () => {
		const { client } = await setup();
		expect(await client.requestAll()).toBe('complete');
		const s = get(client.state);
		expect(s.count).toBe(SPECS.length);
		expect(s.params).toHaveLength(SPECS.length);
		expect(row(client, 'mass')).toMatchObject({ type: 'f32', activeBits: f32ToBits(0.732), stagedBits: f32ToBits(0.732), method: 'measured', sigmaKind: 'Unknown', locked: false, origin: 'DefaultFromCard' });
		// int -1 is the bit pattern of a NaN as a float; it must come back as exactly those bits
		expect(row(client, 'rotor_yaw_s_2713')).toMatchObject({ type: 'i32', activeBits: 0xffffffff, stagedBits: 0xffffffff, index: 2 });
		expect(row(client, 'arm_throttle_min').locked).toBe(true);
		expect(row(client, 'inertia_xx').sigma).toBeCloseTo(0.00107, 6);
		expect(row(client, 'estimator_gyro_b').linkId).toBe('estimator_gyro_b'); // 16 bytes, no terminator
	});

	it('asks again for a value or record that did not arrive', async () => {
		const { vehicle, client } = await setup();
		vehicle.dropFirstValueFor.add(1);
		vehicle.dropFirstRecordFor.add(3);
		expect(await client.requestAll()).toBe('complete');
		expect(get(client.state).params.every((p) => p.linkId !== null && p.hasRecord)).toBe(true);
		expect(vehicle.received.filter((m) => m.kind === 'ParamRequestRead').length).toBeGreaterThan(0);
	});

	it('reports an incomplete listing with the missing indices', async () => {
		const { vehicle, client } = await setup();
		vehicle.neverSendFor.add(4);
		expect(await client.requestAll()).toBe('incomplete');
		expect(get(client.state).missing).toEqual([4]);
	});

	it('refuses to list without a heartbeat', async () => {
		const vehicle = new FakeVehicle({ params: fakeParams() });
		const session = new LinkSession(vehicle.link);
		const client = new VehicleClient(session, { timing: TIMING });
		await expect(client.requestAll()).rejects.toThrow('no heartbeat');
		session.dispose();
	});
});

describe('source texts', () => {
	it('carries the first chunk in the record and reads the rest in chunks', async () => {
		const { vehicle, client } = await setup();
		await client.requestAll();
		const i = index('inertia_xx');
		expect(row(client, 'inertia_xx')).toMatchObject({ sourceLength: LONG_SOURCE.length, source: null });
		expect(row(client, 'inertia_xx').sourcePartial).toBe(LONG_SOURCE.slice(0, 64));
		expect(await client.readSource(i)).toBe(LONG_SOURCE);
		expect(row(client, 'inertia_xx').source).toBe(LONG_SOURCE);
		const reads = vehicle.received.filter((m) => m.kind === 'MarvParamSource' && m.operation === 0);
		expect(reads.map((m) => (m as { offset: number }).offset)).toEqual([64, 128, 192, 256, 320, 384]);
		// a short source is complete in the record
		expect(row(client, 'mass').source).toBe('Luis weighed it');
	});
});

describe('staging', () => {
	it('stages a REAL32 value with its source and reads the source back', async () => {
		const { vehicle, client } = await setup();
		await client.requestAll();
		const result = await client.stage('rate_kp', 'f32', f32ToBits(15), 'tuned on the bench, see the log of 2026-10-17 '.repeat(3));
		expect(result).toMatchObject({ accepted: true, echoBits: f32ToBits(15), reasons: [], sourceStored: true });
		const slot = vehicle.slots[index('rate_kp')]!;
		expect(slot.staged).toBe(f32ToBits(15));
		expect(slot.active).toBe(f32ToBits(12.5));
		expect(slot.text).toContain('tuned on the bench');
		const r = row(client, 'rate_kp');
		expect(r).toMatchObject({ stagedBits: f32ToBits(15), activeBits: f32ToBits(12.5), origin: 'Manual', method: 'scenario', sigmaKind: 'Choice' });
		expect(r.source).toBe(slot.text);
		expect(get(client.state).results.rate_kp?.accepted).toBe(true);
		const wire = setsSent(vehicle)[0] as { param_type: number; param_value: number };
		expect(wire.param_type).toBe(9);
		expect(wire.param_value).toBe(Math.fround(15));
	});

	it('stages INT32 values as their bits, also those that are NaN patterns as floats', async () => {
		const { vehicle, client } = await setup();
		await client.requestAll();
		const slot = vehicle.slots[index('rotor_yaw_s_2713')]!;
		expect(await client.stage('rotor_yaw_s_2713', 'i32', i32ToBits(1), 'motor 1 is clockwise')).toMatchObject({ accepted: true, echoBits: 1 });
		expect(slot.staged).toBe(1);
		expect(await client.stage('rotor_yaw_s_2713', 'i32', i32ToBits(-1), 'motor 1 is counter-clockwise')).toMatchObject({ accepted: true, echoBits: 0xffffffff });
		expect(slot.staged).toBe(0xffffffff);
		expect(row(client, 'rotor_yaw_s_2713').stagedBits).toBe(0xffffffff);
		const sets = setsSent(vehicle) as Array<{ param_type: number }>;
		expect(sets.map((m) => m.param_type)).toEqual([6, 6]);
	});

	it('does not send a write without a source text', async () => {
		const { vehicle, session, client } = await setup();
		await client.requestAll();
		const before = get(session.state).txBytes;
		for (const empty of ['', '   ', '\n\t']) {
			await expect(client.stage('rate_kp', 'f32', f32ToBits(15), empty)).rejects.toThrow(NO_SOURCE_ERROR);
		}
		expect(get(session.state).txBytes).toBe(before);
		expect(setsSent(vehicle)).toHaveLength(0);
		expect(vehicle.slots[index('rate_kp')]!.staged).toBe(f32ToBits(12.5));
	});

	it('does not send a write for a locked parameter, a wrong type or an unknown name', async () => {
		const { vehicle, client } = await setup();
		await client.requestAll();
		await expect(client.stage('arm_throttle_min', 'f32', f32ToBits(0.5), 'x')).rejects.toThrow('locked');
		await expect(client.stage('rate_kp', 'i32', i32ToBits(3), 'x')).rejects.toThrow('not i32');
		await expect(client.stage('nope', 'f32', 0, 'x')).rejects.toThrow('no parameter');
		expect(setsSent(vehicle)).toHaveLength(0);
	});

	it('does not send a bit pattern the float field would change', async () => {
		const { vehicle, client } = await setup();
		await client.requestAll();
		await expect(client.stage('rotor_yaw_s_2713', 'i32', 0x7f800001, 'x')).rejects.toThrow('cannot be carried exactly');
		expect(setsSent(vehicle)).toHaveLength(0);
	});

	it('does not send while the heartbeat says armed', async () => {
		const { vehicle, client } = await setup();
		await client.requestAll();
		vehicle.armed = true;
		vehicle.heartbeat();
		await flush();
		await expect(client.stage('rate_kp', 'f32', f32ToBits(15), 'x')).rejects.toThrow('armed');
		expect(setsSent(vehicle)).toHaveLength(0);
	});

	it('shows the unchanged echo and the refusal text when the vehicle refuses (armed, the GCS had not seen the change yet)', async () => {
		const { vehicle, client } = await setup();
		await client.requestAll();
		vehicle.armed = true; // no heartbeat since: the GCS still believes disarmed, the vehicle decides
		const result = await client.stage('rate_kp', 'f32', f32ToBits(15), 'a source');
		expect(result.accepted).toBe(false);
		expect(result.echoBits).toBe(f32ToBits(12.5));
		expect(result.reasons.join(' ')).toContain('armed');
		expect(result.sourceStored).toBeNull();
		expect(row(client, 'rate_kp').stagedBits).toBe(f32ToBits(12.5));
		expect(vehicle.received.filter((m) => m.kind === 'MarvParamSource')).toHaveLength(0);
		expect(get(client.state).notices.at(-1)?.text).toContain('armed');
	});

	it('shows the unchanged echo and the reason for a locked parameter written behind the client\'s back', async () => {
		const { vehicle, session, client } = await setup();
		await client.requestAll();
		const slot = vehicle.slots[index('arm_throttle_min')]!;
		session.send({
			kind: 'ParamSet', target_system: 1, target_component: 1, param_id: new TextEncoder().encode('arm_throttle_min').slice(0, 16),
			param_value: bitsToWireNumber(f32ToBits(0.9)), param_type: 9
		});
		await flush();
		expect(slot.staged).toBe(f32ToBits(0.05));
		expect(row(client, 'arm_throttle_min').stagedBits).toBe(f32ToBits(0.05));
		expect(get(client.state).notices.at(-1)?.text).toContain('locked');
	});

	it('reports a source the vehicle did not store', async () => {
		const { vehicle, client } = await setup();
		await client.requestAll();
		vehicle.ignoreSourceSet = true;
		const result = await client.stage('rate_kp', 'f32', f32ToBits(15), 'a source');
		expect(result).toMatchObject({ accepted: true, sourceStored: false });
		expect(result.note).toContain('differs');
		expect(vehicle.slots[index('rate_kp')]!.text).toBe(DEFAULT_SOURCE);
	});

	it('serialises writes', async () => {
		const { vehicle, client } = await setup();
		await client.requestAll();
		const [a, b] = await Promise.all([
			client.stage('rate_kp', 'f32', f32ToBits(15), 'first'),
			client.stage('mass', 'f32', f32ToBits(0.8), 'second')
		]);
		expect(a.accepted && b.accepted).toBe(true);
		expect(vehicle.slots[index('rate_kp')]!.text).toBe('first');
		expect(vehicle.slots[index('mass')]!.text).toBe('second');
	});
});

describe('notices and config commands', () => {
	it('reassembles a chunked status text', async () => {
		const { vehicle, client } = await setup();
		const chunk = (text: string, seq: number, last: boolean) => {
			const bytes = new Uint8Array(50);
			bytes.set(new TextEncoder().encode(text));
			if (!last) bytes.fill(0x78, text.length);
			return { kind: 'Statustext', severity: 4, text: bytes, id: 7, chunk_seq: seq } as const;
		};
		vehicle.emit(chunk('', 0, false));
		vehicle.emit(chunk('end', 1, true));
		await flush();
		expect(get(client.state).notices.at(-1)?.text).toBe('x'.repeat(50) + 'end');
	});

	it('sends a config command with the schema hash and returns the vehicle\'s answer', async () => {
		const { vehicle, client } = await setup();
		await expect(client.configCommand('reset-factory')).resolves.toBe('denied: unavailable');
		const sent = vehicle.received.find((m) => m.kind === 'MarvConfigCommand') as { param_schema_hash: bigint; action: number };
		expect(sent.param_schema_hash).toBe(0x3ae00eb50938acban);
		expect(sent.action).toBe(3);
		expect(get(client.state).configAck).toMatchObject({ action: 'reset-factory', result: 'denied: unavailable' });
	});

	it('forgets everything on reset', async () => {
		const { client } = await setup();
		await client.requestAll();
		client.reset();
		expect(get(client.state)).toMatchObject({ identity: null, bytewise: null, params: [], count: null, listing: 'idle' });
	});
});
