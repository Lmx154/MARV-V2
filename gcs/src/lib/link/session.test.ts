import { afterEach, describe, expect, it, vi } from 'vitest';
import { get } from 'svelte/store';
import { MavlinkParser } from './mavlink';
import { LinkSession } from './session';
import { FakeVehicle, FAKE_COMPONENT_ID, FAKE_SYSTEM_ID } from './testing/fake_vehicle';

const flush = () => new Promise((r) => setTimeout(r, 0));

describe('the link session', () => {
	afterEach(() => vi.useRealTimers());

	it('turns bytes into typed messages on the bus and learns the vehicle address and arming state', async () => {
		const vehicle = new FakeVehicle({ params: [] });
		const session = new LinkSession(vehicle.link);
		const kinds: string[] = [];
		session.bus.on('Heartbeat', (m) => kinds.push(`heartbeat:${m.base_mode}`));
		expect(get(session.state).heartbeat).toBe('no heartbeat');
		expect(session.target()).toBeNull();
		vehicle.armed = true;
		vehicle.heartbeat();
		await flush();
		expect(kinds).toEqual(['heartbeat:128']);
		expect(session.target()).toEqual({ systemId: FAKE_SYSTEM_ID, componentId: FAKE_COMPONENT_ID });
		expect(get(session.state)).toMatchObject({ heartbeat: 'alive', armed: true, frames: 1, badFrames: 0 });
		session.dispose();
	});

	it('assembles a frame delivered in two pieces and survives garbage between frames', async () => {
		const vehicle = new FakeVehicle({ params: [] });
		const session = new LinkSession(vehicle.link);
		const seen: string[] = [];
		session.bus.on('Statustext', () => seen.push('text'));
		vehicle.emitSplit({ kind: 'Statustext', severity: 6, text: new Uint8Array(50), id: 0, chunk_seq: 0 }, 5);
		await flush();
		expect(seen).toEqual(['text']);
		session.dispose();
	});

	it('counts a corrupt frame and goes on', async () => {
		let handler: ((b: Uint8Array) => void) | null = null;
		const session = new LinkSession({ send: () => undefined, subscribe: (h) => ((handler = h), () => undefined) });
		const frames: Uint8Array[] = [];
		const sink = { send: (b: Uint8Array) => frames.push(b), subscribe: () => () => undefined };
		const out = new LinkSession(sink);
		out.send({ kind: 'Heartbeat', type: 6, autopilot: 8, base_mode: 0, custom_mode: 0, system_status: 4 });
		const good = frames[0] as Uint8Array;
		const bad = good.slice();
		bad[bad.length - 1] = (bad[bad.length - 1] ?? 0) ^ 0xff;
		(handler as unknown as (b: Uint8Array) => void)(bad);
		(handler as unknown as (b: Uint8Array) => void)(good);
		expect(get(session.state)).toMatchObject({ frames: 1, badFrames: 1, heartbeat: 'alive' });
	});

	it('says "no heartbeat" when the heartbeat stops, and alive again when it returns', async () => {
		vi.useFakeTimers();
		const vehicle = new FakeVehicle({ params: [] });
		const session = new LinkSession(vehicle.link, { heartbeatTimeoutMs: 3000 });
		vehicle.heartbeat();
		await vi.advanceTimersByTimeAsync(10);
		expect(get(session.heartbeat)).toBe('alive');
		await vi.advanceTimersByTimeAsync(2900);
		expect(get(session.heartbeat)).toBe('alive');
		await vi.advanceTimersByTimeAsync(200);
		expect(get(session.heartbeat)).toBe('no heartbeat');
		vehicle.heartbeat();
		await vi.advanceTimersByTimeAsync(10);
		expect(get(session.heartbeat)).toBe('alive');
		session.reset();
		expect(get(session.state)).toMatchObject({ heartbeat: 'no heartbeat', vehicle: null, armed: null });
		session.dispose();
	});

	it('sends frames the codec reads back with the GCS address and a running sequence', () => {
		const frames: Uint8Array[] = [];
		const session = new LinkSession({ send: (b) => frames.push(b), subscribe: () => () => undefined });
		const msg = { kind: 'ParamRequestList', target_system: 1, target_component: 1 } as const;
		session.send(msg);
		session.send(msg);
		const parser = new MavlinkParser();
		const envelopes = frames.map((f) => {
			for (const b of f) parser.push(b);
			return parser.frame.envelope;
		});
		expect(envelopes.map((e) => e.sequence)).toEqual([0, 1]);
		expect(envelopes[0]).toMatchObject({ systemId: 255, componentId: 190 });
		expect(get(session.state).txBytes).toBe(frames[0]!.length * 2);
	});
});
