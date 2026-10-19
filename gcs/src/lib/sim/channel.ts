// The browser's end of the sim channel (ground-segment unit GX-8): one WebSocket to the helper's /ws/sim, one datagram per message, each
// decoded by record.ts. It reads nothing from the vehicle link and the vehicle client reads nothing from here (invariant S-1,
// tools/ci/check_s1.py). The channel is one way: this client never sends.
//
// "Present" is a property of the stream, not of the connection: a simulation is present while records arrive within SIM_TIMING.staleMs.
// When they stop, the SIM panels are not marked stale, they are removed (the simulator is gone); the page says so.
import { writable, type Readable } from 'svelte/store';
import { decodeSimRecord, type SimRecord } from './record';

export interface SimTiming {
	/** A simulation has stopped publishing when no record arrived for this long (ms). */
	staleMs: number;
	/** Wait before connecting again after the socket closed or failed (ms). */
	retryMs: number;
}

/** Harness values of the display, not simulator numbers. staleMs: the record period is 1/20 s of SIM time and a run at a real-time factor of
 * 0.1 sends one every 0.5 s of wall time; 2 s is four of those (PLACEHOLDER: replaced if the channel rate changes). */
export const SIM_TIMING: SimTiming = { staleMs: 2000, retryMs: 2000 };
/** The records the arrival rate is measured over (a labelled harness value). */
const RATE_WINDOW = 20;
const MS_PER_S = 1000;

export interface SimSnapshot {
	/** The latest record, and the browser clock (ms) when it arrived; null before the first. */
	record: SimRecord | null;
	receivedAt: number | null;
	/** Records decoded, datagrams that were not records of version 1, and records missed (gaps in the sequence number). */
	count: number;
	bad: number;
	lost: number;
	/** Wall rate of the arriving records, Hz; null until measured. */
	hz: number | null;
	connected: boolean;
}

export interface SimSocket {
	binaryType: string;
	readyState: number;
	close(): void;
	onopen: ((ev: unknown) => void) | null;
	onclose: ((ev: unknown) => void) | null;
	onerror: ((ev: unknown) => void) | null;
	onmessage: ((ev: { data: unknown }) => void) | null;
}

export interface SimChannelDeps {
	url: string;
	factory?: (url: string) => SimSocket;
	now?: () => number;
	timing?: SimTiming;
}

export class SimChannelClient {
	private readonly factory: (url: string) => SimSocket;
	private readonly now: () => number;
	private readonly timing: SimTiming;
	private socket: SimSocket | null = null;
	private wanted = false;
	private retry: ReturnType<typeof setTimeout> | null = null;
	private arrivals: number[] = [];
	private snap: SimSnapshot = { record: null, receivedAt: null, count: 0, bad: 0, lost: 0, hz: null, connected: false };
	private readonly store = writable<SimSnapshot>(this.snap);
	/** The snapshot, pushed on every record and on every connection change. */
	readonly state: Readable<SimSnapshot> = { subscribe: this.store.subscribe };

	constructor(private readonly deps: SimChannelDeps) {
		this.factory = deps.factory ?? ((u) => new WebSocket(u) as unknown as SimSocket);
		this.now = deps.now ?? (() => Date.now());
		this.timing = deps.timing ?? SIM_TIMING;
	}

	snapshot(): SimSnapshot {
		return this.snap;
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
		this.update({ connected: false });
	}

	private open(): void {
		let ws: SimSocket;
		try {
			ws = this.factory(this.deps.url);
		} catch {
			this.scheduleRetry();
			return;
		}
		ws.binaryType = 'arraybuffer';
		this.socket = ws;
		ws.onopen = () => this.update({ connected: true });
		ws.onerror = () => undefined;
		ws.onclose = () => {
			if (this.socket !== ws) return;
			this.socket = null;
			this.update({ connected: false });
			this.scheduleRetry();
		};
		ws.onmessage = (ev) => this.onMessage(ev.data);
	}

	private scheduleRetry(): void {
		if (!this.wanted || this.retry) return;
		this.retry = setTimeout(() => {
			this.retry = null;
			if (this.wanted && !this.socket) this.open();
		}, this.timing.retryMs);
	}

	private onMessage(data: unknown): void {
		const bytes = data instanceof ArrayBuffer ? new Uint8Array(data) : ArrayBuffer.isView(data) ? new Uint8Array(data.buffer, data.byteOffset, data.byteLength) : null;
		if (!bytes) return;
		this.ingest(bytes);
	}

	/** One datagram (also the entry of the tests). */
	ingest(bytes: Uint8Array): void {
		const t = this.now();
		let record: SimRecord;
		try {
			record = decodeSimRecord(bytes);
		} catch {
			this.update({ bad: this.snap.bad + 1 });
			return;
		}
		const previous = this.snap.record;
		const gap = previous && record.seq > previous.seq ? record.seq - previous.seq - 1 : 0;
		// A restarted run counts from 0 again: a sequence that went back is a new run, not a loss.
		this.arrivals = previous && record.seq <= previous.seq ? [t] : [...this.arrivals, t].slice(-RATE_WINDOW);
		const first = this.arrivals[0]!;
		const hz = this.arrivals.length > 1 && t > first ? ((this.arrivals.length - 1) * MS_PER_S) / (t - first) : null;
		this.update({ record, receivedAt: t, count: this.snap.count + 1, lost: this.snap.lost + gap, hz });
	}

	private update(patch: Partial<SimSnapshot>): void {
		this.snap = { ...this.snap, ...patch };
		this.store.set(this.snap);
	}
}
