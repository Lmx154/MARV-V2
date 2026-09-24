/** The page's connection to the GCS backend: /api/schema, /api/link and /ws, or the dev-only mock (?mock). */
import { normalizeSchema, parseServer } from './protocol';
import { linkHolder } from './resources';
import { parseAirframes } from './sim';
import type { Airframe, ClientMsg, LinkInfo, Schema, ServerMsg } from './types';

interface Socket {
	onopen: (() => void) | null;
	onmessage: ((ev: { data: string }) => void) | null;
	onclose: (() => void) | null;
	send(text: string): void;
	close(): void;
}

/** The ?mock flag ('' when present without a value), or null. Honoured only in dev builds. */
export function mockFlag(url: URL): string | null {
	return import.meta.env.DEV ? url.searchParams.get('mock') : null;
}

export async function loadSchema(mock: string | null): Promise<Schema> {
	if (import.meta.env.DEV && mock !== null) return normalizeSchema((await import('$lib/fixtures/schema.json')).default as Schema);
	const r = await fetch('/api/schema');
	if (!r.ok) throw new Error(`GET /api/schema: ${r.status}`);
	return normalizeSchema((await r.json()) as Schema);
}

export async function loadLink(mock: string | null): Promise<LinkInfo | null> {
	if (mock !== null) return { mode: 'mock', connected: false, via: null, target: null };
	try {
		const r = await fetch('/api/link');
		if (!r.ok) return null;
		const j = (await r.json()) as Partial<LinkInfo>;
		return {
			mode: String(j.mode ?? ''),
			connected: Boolean(j.connected),
			via: typeof j.via === 'string' ? j.via : null,
			target: typeof j.target === 'string' ? j.target : null,
			holder: linkHolder(j.holder)
		};
	} catch {
		return null;
	}
}

/** The sim's airframe catalog (GET /api/sim/airframes). */
export async function loadAirframes(mock: string | null): Promise<Airframe[]> {
	if (import.meta.env.DEV && mock !== null) return parseAirframes((await import('./mock')).MOCK_AIRFRAMES);
	const r = await fetch('/api/sim/airframes');
	if (!r.ok) throw new Error(`GET /api/sim/airframes: ${r.status}`);
	return parseAirframes(await r.json());
}

export interface Connection {
	send(m: ClientMsg): void;
	close(): void;
}

/** Opens /ws (reopening 1 s after it drops) and asks for the setup on every open. */
export function connect(
	mock: string | null,
	schema: Schema,
	on: { message: (m: ServerMsg) => void; open: (open: boolean) => void }
): Connection {
	let sock: Socket | null = null;
	let closed = false;
	let retry: ReturnType<typeof setTimeout> | undefined;

	const open = async (): Promise<void> => {
		if (import.meta.env.DEV && mock !== null) {
			const { MockFc } = await import('./mock');
			sock = new MockFc(schema, mock);
		} else {
			const proto = location.protocol === 'https:' ? 'wss:' : 'ws:';
			sock = new WebSocket(`${proto}//${location.host}/ws`) as unknown as Socket;
		}
		sock.onopen = () => {
			on.open(true);
			sock?.send(JSON.stringify({ type: 'request_setup' } satisfies ClientMsg));
		};
		sock.onmessage = (ev) => {
			const m = parseServer(ev.data);
			if (m) on.message(m);
		};
		sock.onclose = () => {
			on.open(false);
			sock = null;
			if (!closed) retry = setTimeout(() => void open(), 1000);
		};
	};
	void open();

	return {
		send(m) {
			try {
				sock?.send(JSON.stringify(m));
			} catch {
				/* not open yet: the setup is re-requested on open */
			}
		},
		close() {
			closed = true;
			clearTimeout(retry);
			sock?.close();
		}
	};
}
