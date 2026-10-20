// The replay of a recorded ground log (GCS-first unit GX-11, GS-8): the controller's requests and events, and the claim that matters, that the
// bytes a replay delivers decode to the values the live session showed. Both sides here go through the page's own path: the helper
// connection (a WebSocket of binary frames) -> LinkSession -> TelemetryClient. The live side delivers the recorded stream cut one way, the
// replay delivers it cut another, between the helper's replay_state events; the decoded snapshots must be equal, and one corrupted byte of the
// replay must make them differ (the control).
import { afterEach, describe, expect, it } from 'vitest';
import { get } from 'svelte/store';
import { LinkSession } from '$link/session';
import { TelemetryClient, type TelemetrySnapshot } from '$link/telemetry';
import { buildGroundLog, framesOf, readGroundLog } from '$link/testing/ground_log';
import * as m from '$link/testing/telemetry_messages';
import type { Message } from '$link/generated/messages';
import { HelperConnection, HelperError, type HelperEvent, type HelperReply, type WebSocketLike } from '$link/transport';
import { IDLE_STATUS, progressText, ReplayController, statusOf, type ReplayStatus } from './replay_state';
import { computeBar } from './shell';
import type { HelperStatus } from './helper_api';

const LOG = '/runs/20261020T100000Z/ground.log';
const status = (over: Partial<ReplayStatus> = {}): ReplayStatus => ({ ...IDLE_STATUS, ...over });

class Fake {
	calls: Array<{ cmd: string; args: Record<string, unknown> | undefined }> = [];
	answers = new Map<string, HelperReply | Error>();
	handlers = new Set<(e: HelperEvent) => void>();
	started = 0;
	controller = new ReplayController({
		request: async (cmd, args) => {
			this.calls.push({ cmd, args });
			const a = this.answers.get(cmd) ?? { ok: true };
			if (a instanceof Error) throw a;
			return a;
		},
		onEvent: (h) => {
			this.handlers.add(h);
			return () => this.handlers.delete(h);
		},
		started: () => void this.started++
	});
	push(e: HelperEvent) {
		this.handlers.forEach((h) => h(e));
	}
	get state() {
		return get(this.controller.state);
	}
}

describe('the replay controller', () => {
	it('lists the ground logs and the status the helper reports, and keeps only what is a log', async () => {
		const f = new Fake();
		f.answers.set('replay_list', {
			ok: true,
			logs: [{ path: LOG, name: '20261020T100000Z', size: 100, current: false, rx_records: 3, rx_bytes: 90, duration_s: 1.5 }, { nonsense: true }],
			replay: status({ state: 'finished', log: LOG, records_total: 3, records_sent: 3 })
		});
		expect(f.state.logs).toBeNull();
		await f.controller.load();
		expect(f.state.logs?.map((l) => l.path)).toEqual([LOG]);
		expect(f.state.status.state).toBe('finished');
		expect(f.state.error).toBeNull();
	});

	it('asks for a replay with the log and the mode, and shows the helper\'s refusal verbatim', async () => {
		const f = new Fake();
		await f.controller.play(LOG, 'fast');
		expect(f.calls).toEqual([{ cmd: 'replay_start', args: { log: LOG, mode: 'fast' } }]);
		f.answers.set('replay_start', new HelperError({ ok: false, error: 'the helper has a device open (udp 127.0.0.1:1): close it first' }, 'replay_start'));
		await f.controller.play(LOG, 'paced');
		expect(f.state.error).toBe('the helper has a device open (udp 127.0.0.1:1): close it first');
		await f.controller.play(LOG, 'paced').catch(() => undefined);
		f.answers.set('replay_start', { ok: true, replay: status({ state: 'playing', log: LOG, mode: 'paced' }) });
		await f.controller.play(LOG, 'paced');
		expect(f.state.error).toBeNull();
	});

	it('forgets the vehicle once when a replay begins, follows the events, and ignores events that are not its own', () => {
		const f = new Fake();
		f.controller.start();
		f.push({ event: 'device_opened', device: 'x' });
		f.push({ event: 'replay_state', state: 'bogus' });
		expect(f.started).toBe(0);
		f.push({ event: 'replay_state', ...status({ state: 'playing', log: LOG, mode: 'fast', records_total: 10 }) });
		f.push({ event: 'replay_state', ...status({ state: 'playing', log: LOG, mode: 'fast', records_total: 10, records_sent: 4, position_s: 0.4, duration_s: 1 }) });
		expect(f.started).toBe(1);
		expect(f.state.status.records_sent).toBe(4);
		expect(progressText(f.state.status)).toBe('0.4 s of 1.0 s · 4 of 10 chunks');
		f.push({ event: 'replay_state', ...status({ state: 'finished', log: LOG, records_total: 10, records_sent: 10 }) });
		expect(f.state.status.state).toBe('finished');
		f.push({ event: 'replay_state', ...status({ state: 'playing', log: LOG }) });
		expect(f.started).toBe(2);
	});

	it('does not move the status back to idle with a late answer to replay_start', async () => {
		const f = new Fake();
		f.controller.start();
		f.answers.set('replay_start', { ok: true, replay: status({ state: 'playing', log: LOG }) });
		f.push({ event: 'replay_state', ...status({ state: 'finished', log: LOG, records_sent: 3, records_total: 3 }) });
		await f.controller.play(LOG, 'fast');
		expect(f.state.status.state).toBe('finished');
	});

	it('stops with replay_stop and shows a refusal', async () => {
		const f = new Fake();
		await f.controller.stop();
		expect(f.calls.map((c) => c.cmd)).toEqual(['replay_stop']);
		f.answers.set('replay_stop', new HelperError({ ok: false, error: 'no' }, 'replay_stop'));
		await f.controller.stop();
		expect(f.state.error).toBe('no');
	});

	it('reads a status object only if its state is one of the five', () => {
		expect(statusOf({ state: 'idle' })).toEqual(IDLE_STATUS);
		expect(statusOf({ state: 'running' })).toBeNull();
		expect(statusOf(null)).toBeNull();
		expect(statusOf({ state: 'failed', reason: 'x' })?.reason).toBe('x');
	});
});

describe('the status bar during a replay', () => {
	const helperStatus = (replay: ReplayStatus): HelperStatus =>
		({ device: null, replay, autoconnect: { enabled: true, suspended: null, owner: null, prefer: null, reason: null }, runs: { active: null, last: null }, host: null }) as unknown as HelperStatus;
	const linkFields = (replay: ReplayStatus | undefined) =>
		computeBar({ helper: 'open', status: replay ? helperStatus(replay) : null, statusAt: 0, session: null, identity: null }, 0).find((i) => i.id === 'link')!.fields;

	it('names the recorded session in the link indicator, so a recording is not taken for a vehicle', () => {
		const field = linkFields(status({ state: 'playing', log: LOG }))?.find((x) => x.name === 'replay');
		expect(field?.value).toBe('playing: 20261020T100000Z');
		expect(field?.title).toContain(LOG);
	});

	it('has no replay field with no replay (control)', () => {
		expect(linkFields(status())?.some((x) => x.name === 'replay')).toBe(false);
		expect(linkFields(undefined)?.some((x) => x.name === 'replay')).toBe(false);
	});
});

// ---- the same decode path ---------------------------------------------------------------------------------------------------

class FakeSocket implements WebSocketLike {
	binaryType = 'blob';
	readyState = 1;
	sent: Array<ArrayBufferLike | ArrayBufferView | string> = [];
	onopen: ((ev: unknown) => void) | null = null;
	onclose: ((ev: unknown) => void) | null = null;
	onerror: ((ev: unknown) => void) | null = null;
	onmessage: ((ev: { data: unknown }) => void) | null = null;
	send(data: ArrayBufferLike | ArrayBufferView | string): void {
		this.sent.push(data);
	}
	close(): void {}
	deliver(data: unknown): void {
		this.onmessage?.({ data });
	}
}

const open: Array<() => void> = [];
afterEach(() => {
	while (open.length) open.pop()?.();
});

/** The decoded values the page holds after `deliver` fed the helper connection its frames (the arrival times are the page's own clock, not decoded values). */
async function decoded(deliver: (socket: FakeSocket) => void): Promise<Omit<TelemetrySnapshot, 'stamps'>> {
	const socket = new FakeSocket();
	const conn = new HelperConnection('ws://127.0.0.1:1/ws', () => socket);
	const p = conn.connect();
	socket.onopen?.({});
	await p;
	const session = new LinkSession(conn, { heartbeatTimeoutMs: 600_000 });
	const telemetry = new TelemetryClient(session);
	open.push(() => telemetry.dispose(), () => session.dispose());
	deliver(socket);
	const { stamps: _stamps, ...values } = telemetry.snapshot();
	void _stamps;
	return values;
}

const chunked = (stream: Uint8Array, size: number): Uint8Array[] => {
	const out: Uint8Array[] = [];
	for (let at = 0; at < stream.length; at += size) out.push(stream.slice(at, at + size));
	return out;
};

describe('replayed bytes decode to what the live session showed', () => {
	// A recorded session: two rounds of every stream (the second with other values, so that a lost frame of it shows), written into a ground log
	// with the chunks as the operating system handed them over.
	const second: Message[] = m.allStreamMessages.map((x) => (x.kind === 'Attitude' ? { ...x, roll: 0.75 } : x.kind === 'MarvEstimator' ? { ...x, gyro_bias_eskf: [0.5, 0.25, 0.125] } : x));
	const frames = framesOf([...m.allStreamMessages, ...second]);
	const FRAME_HEADER_BYTES = 10;
	const SECOND_ATTITUDE_PAYLOAD = frames.slice(0, m.allStreamMessages.length + 1).reduce((n, f) => n + f.length, 0) + FRAME_HEADER_BYTES; // the first payload byte of the second round's attitude frame
	const stream = Uint8Array.from(frames.flatMap((f) => [...f]));
	const LIVE_CHUNK = 37; // labelled harness values: read sizes that cut frames in the middle, differently for the live read and the replay's
	const REPLAY_CHUNK = 101;
	const log = buildGroundLog(chunked(stream, LIVE_CHUNK).map((bytes, i) => ({ atMs: 1000 + i * 10, bytes })));
	const recorded = readGroundLog(log).map((c) => c.bytes);

	const live = () => decoded((socket) => recorded.forEach((b) => socket.deliver(b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength))));
	const replayed = (corrupt = false) =>
		decoded((socket) => {
			// What the helper sends: the replay_state event first, then the same bytes in its own chunks, then the end event.
			socket.deliver(JSON.stringify({ event: 'replay_state', ...status({ state: 'playing', log: LOG }) }));
			const bytes = Uint8Array.from(recorded.flatMap((b) => [...b]));
			if (corrupt) bytes[SECOND_ATTITUDE_PAYLOAD] = (bytes[SECOND_ATTITUDE_PAYLOAD] ?? 0) ^ 0xff;
			for (const c of chunked(bytes, REPLAY_CHUNK)) socket.deliver(c.buffer.slice(c.byteOffset, c.byteOffset + c.byteLength));
			socket.deliver(JSON.stringify({ event: 'replay_state', ...status({ state: 'finished', log: LOG }) }));
		});

	it('are equal for every stream, with the replay cut differently from the live read', async () => {
		const a = await live();
		const b = await replayed();
		expect(a.status).not.toBeNull();
		expect(Object.entries(a).every(([k, v]) => k === 'wanted' || k === 'requests' || v !== null)).toBe(true);
		expect(b).toEqual(a);
	});

	it('differ when one byte of the replay is corrupted (the control)', async () => {
		const a = await live();
		const b = await replayed(true);
		expect(b).not.toEqual(a);
	});
});
