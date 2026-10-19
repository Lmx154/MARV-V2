// The browser's side of the helper (tools/gs/helper.py): one WebSocket at /ws. Binary frames are raw device bytes in both
// directions; text frames are JSON control messages {cmd, id?} answered {ok, id?, ...}, plus pushed events {event, ...}.
// This file does not parse device bytes; the session above it does.
import { writable, type Readable } from 'svelte/store';

/** What the session needs of a transport: bytes out, bytes in. */
export interface ByteLink {
	send(bytes: Uint8Array): void;
	subscribe(handler: (bytes: Uint8Array) => void): () => void;
}

export type HelperConnectionState = 'closed' | 'connecting' | 'open';

export interface HelperReply {
	ok: boolean;
	id?: number;
	error?: string;
	[key: string]: unknown;
}

/** The error of a command the helper answered {ok: false}: its text, and the whole reply (a refusal may list several reasons). */
export class HelperError extends Error {
	constructor(readonly reply: HelperReply, cmd: string) {
		super(reply.error ?? `${cmd} failed`);
		this.name = 'HelperError';
	}
}

export interface HelperEvent {
	event: string;
	[key: string]: unknown;
}

export interface WebSocketLike {
	binaryType: string;
	readyState: number;
	send(data: ArrayBufferLike | ArrayBufferView | string): void;
	close(): void;
	onopen: ((ev: unknown) => void) | null;
	onclose: ((ev: unknown) => void) | null;
	onerror: ((ev: unknown) => void) | null;
	onmessage: ((ev: { data: unknown }) => void) | null;
}

export type WebSocketFactory = (url: string) => WebSocketLike;

/** WebSocket.OPEN (RFC 6455 / WHATWG). */
const WS_OPEN = 1;
/** PLACEHOLDER: how long a control request waits for the helper's answer before it fails (a harness value, not a design number). */
export const CONTROL_TIMEOUT_MS = 10_000;

export class HelperConnection implements ByteLink {
	private socket: WebSocketLike | null = null;
	private readonly handlers = new Set<(bytes: Uint8Array) => void>();
	private readonly eventHandlers = new Set<(event: HelperEvent) => void>();
	private readonly pending = new Map<number, { resolve: (r: HelperReply) => void; reject: (e: Error) => void; timer: ReturnType<typeof setTimeout> }>();
	private nextId = 1;
	private readonly stateStore = writable<HelperConnectionState>('closed');
	private readonly counters = writable({ rxBytes: 0, txBytes: 0 });
	readonly state: Readable<HelperConnectionState> = { subscribe: this.stateStore.subscribe };
	/** Bytes carried to and from the device through this connection (the GCS's own count). */
	readonly bytes: Readable<{ rxBytes: number; txBytes: number }> = { subscribe: this.counters.subscribe };
	private rx = 0;
	private tx = 0;

	constructor(
		private readonly url: string,
		private readonly factory: WebSocketFactory = (u) => new WebSocket(u) as unknown as WebSocketLike
	) {}

	connect(): Promise<void> {
		if (this.socket) return Promise.reject(new Error('already connected'));
		this.stateStore.set('connecting');
		return new Promise((resolve, reject) => {
			const ws = this.factory(this.url);
			ws.binaryType = 'arraybuffer';
			this.socket = ws;
			ws.onopen = () => {
				this.stateStore.set('open');
				resolve();
			};
			ws.onerror = () => {
				if (this.socket === ws && ws.readyState !== WS_OPEN) reject(new Error('could not connect to the helper'));
			};
			ws.onclose = () => {
				if (this.socket !== ws) return;
				this.socket = null;
				this.stateStore.set('closed');
				for (const p of this.pending.values()) {
					clearTimeout(p.timer);
					p.reject(new Error('the helper connection closed'));
				}
				this.pending.clear();
				reject(new Error('the helper connection closed'));
			};
			ws.onmessage = (ev) => this.onMessage(ev.data);
		});
	}

	close(): void {
		this.socket?.close();
	}

	send(bytes: Uint8Array): void {
		if (!this.socket || this.socket.readyState !== WS_OPEN) throw new Error('the helper connection is not open');
		this.socket.send(bytes);
		this.tx += bytes.length;
		this.counters.set({ rxBytes: this.rx, txBytes: this.tx });
	}

	subscribe(handler: (bytes: Uint8Array) => void): () => void {
		this.handlers.add(handler);
		return () => this.handlers.delete(handler);
	}

	onEvent(handler: (event: HelperEvent) => void): () => void {
		this.eventHandlers.add(handler);
		return () => this.eventHandlers.delete(handler);
	}

	/** Sends a control command and resolves with the helper's answer; an {ok: false} answer rejects with its error text. */
	request(cmd: string, args: Record<string, unknown> = {}): Promise<HelperReply> {
		if (!this.socket || this.socket.readyState !== WS_OPEN) return Promise.reject(new Error('the helper connection is not open'));
		const id = this.nextId++;
		const socket = this.socket;
		return new Promise((resolve, reject) => {
			const timer = setTimeout(() => {
				this.pending.delete(id);
				reject(new Error(`the helper did not answer ${cmd}`));
			}, CONTROL_TIMEOUT_MS);
			this.pending.set(id, {
				resolve: (r) => (r.ok ? resolve(r) : reject(new HelperError(r, cmd))),
				reject,
				timer
			});
			socket.send(JSON.stringify({ cmd, id, ...args }));
		});
	}

	private onMessage(data: unknown): void {
		if (typeof data === 'string') {
			let msg: Record<string, unknown>;
			try {
				msg = JSON.parse(data) as Record<string, unknown>;
			} catch {
				return;
			}
			if (typeof msg.event === 'string') {
				for (const h of this.eventHandlers) h(msg as HelperEvent);
			} else if (typeof msg.id === 'number') {
				const p = this.pending.get(msg.id);
				if (p) {
					this.pending.delete(msg.id);
					clearTimeout(p.timer);
					p.resolve(msg as HelperReply);
				}
			}
			return;
		}
		const bytes = data instanceof ArrayBuffer ? new Uint8Array(data) : ArrayBuffer.isView(data) ? new Uint8Array(data.buffer, data.byteOffset, data.byteLength) : null;
		if (!bytes) return;
		this.rx += bytes.length;
		this.counters.set({ rxBytes: this.rx, txBytes: this.tx });
		for (const h of this.handlers) h(bytes);
	}
}
