// The browser's end of the board's console (ground-segment unit GX-16, tools/gs/console.py): one WebSocket to the helper's /ws/console, text
// frames both ways. Down: the console's output as the helper decoded it. Up: a text frame is sent to the board exactly as it is. This client
// parses nothing and adds nothing; it is separate from the helper connection that carries the link (HelperConnection, /ws).
import { writable, type Readable } from 'svelte/store';

/** The helper's path for the console (tools/gs/console.py WS_PATH). */
export const CONSOLE_WS_PATH = '/ws/console';
/** Wait before connecting again after the socket closed or failed (ms). PLACEHOLDER (harness): the helper takes one console client, and a reloaded page meets the old one's close. */
export const CONSOLE_RETRY_MS = 1000;
/** WebSocket.OPEN (RFC 6455 / WHATWG). */
const WS_OPEN = 1;

/** The URL of the console channel for a helper's /ws URL (same host, same token). */
export function consoleUrlOf(wsUrl: string): string {
	const url = new URL(wsUrl);
	url.pathname = CONSOLE_WS_PATH;
	return url.toString();
}

export interface ConsoleSocket {
	readyState: number;
	send(data: string): void;
	close(): void;
	onopen: ((ev: unknown) => void) | null;
	onclose: ((ev: unknown) => void) | null;
	onerror: ((ev: unknown) => void) | null;
	onmessage: ((ev: { data: unknown }) => void) | null;
}

/** What the controller needs of the channel. */
export interface TextChannel {
	/** Whether the socket is open. */
	readonly connected: Readable<boolean>;
	start(): void;
	stop(): void;
	/** Sends the text as one frame; false (nothing sent) when the socket is not open. */
	send(text: string): boolean;
	onText(handler: (text: string) => void): () => void;
}

export interface ConsoleChannelDeps {
	url: string;
	factory?: (url: string) => ConsoleSocket;
	retryMs?: number;
}

export class ConsoleChannel implements TextChannel {
	private readonly factory: (url: string) => ConsoleSocket;
	private readonly retryMs: number;
	private socket: ConsoleSocket | null = null;
	private wanted = false;
	private retry: ReturnType<typeof setTimeout> | null = null;
	private readonly handlers = new Set<(text: string) => void>();
	private readonly store = writable(false);
	readonly connected: Readable<boolean> = { subscribe: this.store.subscribe };

	constructor(private readonly deps: ConsoleChannelDeps) {
		this.factory = deps.factory ?? ((u) => new WebSocket(u) as unknown as ConsoleSocket);
		this.retryMs = deps.retryMs ?? CONSOLE_RETRY_MS;
	}

	/** Connects, and connects again after a close, until stop(). A second call does nothing. */
	start(): void {
		if (this.wanted) return;
		this.wanted = true;
		this.open();
	}

	stop(): void {
		this.wanted = false;
		if (this.retry) clearTimeout(this.retry);
		this.retry = null;
		const s = this.socket;
		this.socket = null;
		s?.close();
		this.store.set(false);
	}

	send(text: string): boolean {
		if (!this.socket || this.socket.readyState !== WS_OPEN) return false;
		this.socket.send(text);
		return true;
	}

	onText(handler: (text: string) => void): () => void {
		this.handlers.add(handler);
		return () => this.handlers.delete(handler);
	}

	private open(): void {
		let ws: ConsoleSocket;
		try {
			ws = this.factory(this.deps.url);
		} catch {
			this.scheduleRetry();
			return;
		}
		this.socket = ws;
		ws.onopen = () => this.store.set(true);
		ws.onerror = () => undefined;
		ws.onclose = () => {
			if (this.socket !== ws) return;
			this.socket = null;
			this.store.set(false);
			this.scheduleRetry();
		};
		ws.onmessage = (ev) => {
			if (typeof ev.data !== 'string') return;
			for (const h of this.handlers) h(ev.data);
		};
	}

	private scheduleRetry(): void {
		if (!this.wanted || this.retry) return;
		this.retry = setTimeout(() => {
			this.retry = null;
			if (this.wanted && !this.socket) this.open();
		}, this.retryMs);
	}
}
