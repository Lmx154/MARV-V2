// The replay of a recorded ground log (GCS-first unit GX-11, ground spec GS-8). The helper (tools/gs/replay.py) lists the ground logs of its
// runs, and on request feeds one log's received bytes to this page as the binary frames of /ws, the frames a live device's bytes arrive in:
// the codec, the session and the vehicle client decode them as they decode a live vehicle, and the Vehicle page shows the recorded session.
// This controller sends the helper's commands (replay_list, replay_start, replay_stop) and keeps what the Logs page shows; every refusal is
// shown as the helper words it. A replay sends nothing to a vehicle: the helper has no device then and drops what this page sends meanwhile.
import { get, writable, type Readable } from 'svelte/store';
import type { HelperEvent, HelperReply } from '$link/transport';

export type ReplayMode = 'paced' | 'fast';
export type ReplayPhase = 'idle' | 'playing' | 'finished' | 'stopped' | 'failed';

/** One ground log the helper may replay (tools/gs/replay.py summarise). */
export interface ReplayLog {
	path: string;
	/** The run directory's name. */
	name: string;
	size: number;
	/** The running session's own log. */
	current: boolean;
	wall_utc?: string;
	rx_records?: number;
	rx_bytes?: number;
	tx_records?: number;
	events?: number;
	/** The span of the received chunks (s). */
	duration_s?: number;
	/** Set instead of the rest when the file is not a ground log. */
	error?: string;
}

export interface ReplayStatus {
	state: ReplayPhase;
	reason: string | null;
	log: string | null;
	mode: ReplayMode | null;
	records_total: number;
	records_sent: number;
	bytes_total: number;
	bytes_sent: number;
	position_s: number;
	duration_s: number;
	/** Bytes this page sent during the replay: counted and dropped by the helper. */
	tx_dropped_bytes: number;
}

export interface ReplayState {
	/** The ground logs; null until read. */
	logs: ReplayLog[] | null;
	status: ReplayStatus;
	/** The helper's refusal of the last command, verbatim. */
	error: string | null;
	busy: boolean;
}

export interface ReplayDeps {
	request(cmd: string, args?: Record<string, unknown>): Promise<HelperReply>;
	onEvent(handler: (event: HelperEvent) => void): () => void;
	/** A replay begins: forget the vehicle (and the connection's manifest), so that the recorded session's identity is learnt from its bytes. */
	started(): void;
}

const isObject = (v: unknown): v is Record<string, unknown> => typeof v === 'object' && v !== null && !Array.isArray(v);
const messageOf = (e: unknown): string => (e instanceof Error ? e.message : String(e));
const PHASES: readonly string[] = ['idle', 'playing', 'finished', 'stopped', 'failed'];

export const IDLE_STATUS: ReplayStatus = {
	state: 'idle', reason: null, log: null, mode: null, records_total: 0, records_sent: 0, bytes_total: 0, bytes_sent: 0, position_s: 0, duration_s: 0, tx_dropped_bytes: 0
};

/** The helper's replay status, or null for something that is not one. */
export function statusOf(v: unknown): ReplayStatus | null {
	if (!isObject(v) || typeof v.state !== 'string' || !PHASES.includes(v.state)) return null;
	return { ...IDLE_STATUS, ...(v as Partial<ReplayStatus>), state: v.state as ReplayPhase };
}

function logOf(v: unknown): ReplayLog | null {
	if (!isObject(v) || typeof v.path !== 'string' || typeof v.name !== 'string' || typeof v.size !== 'number') return null;
	return v as unknown as ReplayLog;
}

/** "12.3 s of 60.0 s" and the share of the chunks sent. */
export function progressText(s: ReplayStatus): string {
	return `${s.position_s.toFixed(1)} s of ${s.duration_s.toFixed(1)} s · ${s.records_sent} of ${s.records_total} chunks`;
}

export class ReplayController {
	private readonly store = writable<ReplayState>({ logs: null, status: IDLE_STATUS, error: null, busy: false });
	readonly state: Readable<ReplayState> = { subscribe: this.store.subscribe };
	private off: (() => void) | null = null;

	constructor(private readonly deps: ReplayDeps) {}

	/** Starts following the helper's replay events; returns what stops it. A second call does nothing. */
	start(): () => void {
		if (!this.off) this.off = this.deps.onEvent((e) => this.onEvent(e));
		return () => {
			this.off?.();
			this.off = null;
		};
	}

	private onEvent(e: HelperEvent): void {
		if (e.event !== 'replay_state') return;
		const status = statusOf(e);
		if (!status) return;
		const before = get(this.store).status.state;
		if (status.state === 'playing' && before !== 'playing') this.deps.started();
		this.store.update((s) => ({ ...s, status }));
	}

	/** The ground logs the helper lists, and the replay's status. */
	async load(): Promise<void> {
		if (get(this.store).busy) return;
		this.store.update((s) => ({ ...s, busy: true }));
		try {
			const reply = await this.deps.request('replay_list');
			const logs = (Array.isArray(reply.logs) ? reply.logs : []).map(logOf).filter((l): l is ReplayLog => l !== null);
			const status = statusOf(reply.replay);
			this.store.update((s) => ({ ...s, logs, status: status ?? s.status, error: null, busy: false }));
		} catch (e) {
			this.store.update((s) => ({ ...s, error: messageOf(e), busy: false }));
		}
	}

	/** Asks the helper to replay a log; the helper answers at once and the progress and the end come as events. */
	async play(path: string, mode: ReplayMode): Promise<void> {
		this.store.update((s) => ({ ...s, error: null }));
		try {
			const reply = await this.deps.request('replay_start', { log: path, mode });
			const status = statusOf(reply.replay);
			// The playing event comes first on the wire; the answer may be later than the end of a short replay: never go back from it.
			if (status && get(this.store).status.state === 'idle') this.store.update((s) => ({ ...s, status }));
		} catch (e) {
			this.store.update((s) => ({ ...s, error: messageOf(e) }));
		}
	}

	async stop(): Promise<void> {
		try {
			await this.deps.request('replay_stop');
		} catch (e) {
			this.store.update((s) => ({ ...s, error: messageOf(e) }));
		}
	}
}
