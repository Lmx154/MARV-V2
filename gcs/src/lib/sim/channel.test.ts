import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import { SIM_TIMING, SimChannelClient, type SimSocket } from './channel';
import { simUrlOf } from './instance';
import { buildRecord } from './testing/build';

class FakeSocket implements SimSocket {
	binaryType = '';
	readyState = 0;
	onopen: ((ev: unknown) => void) | null = null;
	onclose: ((ev: unknown) => void) | null = null;
	onerror: ((ev: unknown) => void) | null = null;
	onmessage: ((ev: { data: unknown }) => void) | null = null;
	closed = false;
	constructor(readonly url: string) {}
	close(): void {
		this.closed = true;
	}
	open(): void {
		this.readyState = 1;
		this.onopen?.({});
	}
	deliver(bytes: Uint8Array): void {
		this.onmessage?.({ data: bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength) });
	}
	drop(): void {
		this.onclose?.({});
	}
}

function client() {
	const sockets: FakeSocket[] = [];
	let clock = 10_000;
	const c = new SimChannelClient({ url: 'ws://h/ws/sim?token=t', factory: (u) => (sockets.push(new FakeSocket(u)), sockets[sockets.length - 1]!), now: () => clock });
	return { c, sockets, advance: (ms: number) => (clock += ms), last: () => sockets[sockets.length - 1]! };
}

describe('the sim channel client', () => {
	beforeEach(() => vi.useFakeTimers());
	afterEach(() => vi.useRealTimers());

	it('is derived from the helper\'s /ws URL with the same host and token', () => {
		expect(simUrlOf('ws://127.0.0.1:8765/ws?token=abc')).toBe('ws://127.0.0.1:8765/ws/sim?token=abc');
	});

	it('decodes each message, counts records, bad datagrams and gaps, and measures the arrival rate', () => {
		const { c, sockets, advance } = client();
		c.start();
		expect(sockets).toHaveLength(1);
		sockets[0]!.open();
		expect(c.snapshot().connected).toBe(true);
		for (const seq of [0, 1, 2, 5]) {
			sockets[0]!.deliver(buildRecord({ seq }));
			advance(50);
		}
		sockets[0]!.deliver(new Uint8Array([1, 2, 3])); // not a record
		const s = c.snapshot();
		expect(s.count).toBe(4);
		expect(s.bad).toBe(1);
		expect(s.lost).toBe(2); // 3 and 4
		expect(s.record?.seq).toBe(5);
		expect(s.hz).toBeCloseTo(20, 5);
	});

	it('does not count a restarted run (the sequence goes back) as a loss', () => {
		const { c, sockets } = client();
		c.start();
		sockets[0]!.deliver(buildRecord({ seq: 90 }));
		sockets[0]!.deliver(buildRecord({ seq: 0 }));
		expect(c.snapshot().lost).toBe(0);
		expect(c.snapshot().record?.seq).toBe(0);
	});

	it('connects again after the socket closes, and not after stop()', () => {
		const { c, sockets } = client();
		c.start();
		sockets[0]!.open();
		sockets[0]!.drop();
		expect(c.snapshot().connected).toBe(false);
		expect(sockets).toHaveLength(1);
		vi.advanceTimersByTime(SIM_TIMING.retryMs);
		expect(sockets).toHaveLength(2);
		c.stop();
		expect(sockets[1]!.closed).toBe(true);
		sockets[1]!.drop(); // a close event after stop() reconnects nothing
		vi.advanceTimersByTime(SIM_TIMING.retryMs * 3);
		expect(sockets).toHaveLength(2);
	});

	it('never sends: the channel is one way', () => {
		const { c, sockets } = client();
		c.start();
		expect('send' in sockets[0]!).toBe(false);
	});
});
