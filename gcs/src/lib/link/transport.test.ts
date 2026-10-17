import { describe, expect, it } from 'vitest';
import { get } from 'svelte/store';
import { HelperConnection, type WebSocketLike } from './transport';

class FakeSocket implements WebSocketLike {
	binaryType = 'blob';
	readyState = 0;
	sent: Array<ArrayBufferLike | ArrayBufferView | string> = [];
	onopen: ((ev: unknown) => void) | null = null;
	onclose: ((ev: unknown) => void) | null = null;
	onerror: ((ev: unknown) => void) | null = null;
	onmessage: ((ev: { data: unknown }) => void) | null = null;
	send(data: ArrayBufferLike | ArrayBufferView | string): void {
		this.sent.push(data);
	}
	close(): void {
		this.readyState = 3;
		this.onclose?.({});
	}
	open(): void {
		this.readyState = 1;
		this.onopen?.({});
	}
	deliver(data: unknown): void {
		this.onmessage?.({ data });
	}
}

function make(): { conn: HelperConnection; socket: FakeSocket; urls: string[] } {
	const urls: string[] = [];
	let socket!: FakeSocket;
	const conn = new HelperConnection('ws://127.0.0.1:1/ws?token=t', (url) => {
		urls.push(url);
		return (socket = new FakeSocket());
	});
	return { conn, get socket() { return socket; }, urls } as unknown as { conn: HelperConnection; socket: FakeSocket; urls: string[] };
}

describe('the helper connection', () => {
	it('opens with binary frames as ArrayBuffers and reports its state', async () => {
		const t = make();
		expect(get(t.conn.state)).toBe('closed');
		const p = t.conn.connect();
		expect(get(t.conn.state)).toBe('connecting');
		t.socket.open();
		await p;
		expect(get(t.conn.state)).toBe('open');
		expect(t.socket.binaryType).toBe('arraybuffer');
		expect(t.urls).toEqual(['ws://127.0.0.1:1/ws?token=t']);
		t.conn.close();
		expect(get(t.conn.state)).toBe('closed');
	});

	it('relays device bytes both ways and counts them', async () => {
		const t = make();
		const p = t.conn.connect();
		t.socket.open();
		await p;
		const got: number[][] = [];
		const off = t.conn.subscribe((b) => got.push([...b]));
		t.socket.deliver(new Uint8Array([1, 2, 3]).buffer);
		t.conn.send(new Uint8Array([9, 8]));
		expect(got).toEqual([[1, 2, 3]]);
		expect(t.socket.sent).toHaveLength(1);
		expect(get(t.conn.bytes)).toEqual({ rxBytes: 3, txBytes: 2 });
		off();
		t.socket.deliver(new Uint8Array([4]).buffer);
		expect(got).toHaveLength(1);
	});

	it('correlates control replies by id, rejects a failed command with the helper\'s text and delivers events', async () => {
		const t = make();
		const p = t.conn.connect();
		t.socket.open();
		await p;
		const events: string[] = [];
		t.conn.onEvent((e) => events.push(e.event));
		const ok = t.conn.request('open', { udp_port: 14550 });
		const sent = JSON.parse(t.socket.sent[0] as string) as { cmd: string; id: number; udp_port: number };
		expect(sent).toMatchObject({ cmd: 'open', udp_port: 14550 });
		const bad = t.conn.request('close');
		const sentBad = JSON.parse(t.socket.sent[1] as string) as { id: number };
		t.socket.deliver(JSON.stringify({ ok: false, id: sentBad.id, error: 'nothing is open' }));
		t.socket.deliver(JSON.stringify({ event: 'device_opened', device: 'udp 127.0.0.1:14550' }));
		t.socket.deliver(JSON.stringify({ ok: true, id: sent.id }));
		await expect(ok).resolves.toMatchObject({ ok: true });
		await expect(bad).rejects.toThrow('nothing is open');
		expect(events).toEqual(['device_opened']);
	});

	it('fails the pending requests when the connection closes and refuses to send when it is not open', async () => {
		const t = make();
		const p = t.conn.connect();
		t.socket.open();
		await p;
		const pending = t.conn.request('status');
		t.socket.close();
		await expect(pending).rejects.toThrow('closed');
		expect(() => t.conn.send(new Uint8Array(1))).toThrow('not open');
		await expect(t.conn.request('status')).rejects.toThrow('not open');
	});
});
