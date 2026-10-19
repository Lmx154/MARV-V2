// The shell of the GCS (GCS-first unit GX-2): the status bar that is on screen on every page, and the controller that keeps it
// true. computeBar() is pure: the same inputs and the same `now` give the same indicators, so every stale transition is tested
// without a browser. The ShellController owns what feeds it: the helper's status (read every STATUS_POLL_MS), the clock tick
// (every TICK_MS, so that an age and a stale flag move without any new data) and the helper's events (device_opened,
// device_closed, sil_relaunched, ...), which used to be handled in app.ts.
//
// Staleness is per source. A vehicle-sourced indicator is stale when nothing was heard from the vehicle for
// SHELL_TIMING.vehicleStaleMs; a helper-sourced indicator is stale when the helper's status is older than
// SHELL_TIMING.helperStaleMs. A stale indicator keeps its last value on screen, marked stale: it never shows a value as current
// that is not.
import { get, writable, type Readable } from 'svelte/store';
import type { HelperConnectionState, HelperEvent } from '$link/transport';
import type { SessionState } from '$link/session';
import type { VehicleIdentity } from '$link/vehicle';
import type { HelperStatus } from './helper_api';

export type IndicatorId = 'helper' | 'link' | 'identity' | 'arming' | 'run' | 'host';
export type Tone = 'neutral' | 'ok' | 'warn' | 'bad';

export interface ShellTiming {
	/** A vehicle-sourced indicator is stale when nothing was heard from the vehicle for this long (ms). */
	vehicleStaleMs: number;
	/** A helper-sourced indicator is stale when the helper's status is older than this (ms). */
	helperStaleMs: number;
}

/** Harness values of the display, not vehicle numbers: they decide only what the operator is told is current. */
export const SHELL_TIMING: ShellTiming = {
	// The same 3 missed beats of the conventional 1 Hz heartbeat as HEARTBEAT_TIMEOUT_MS in session.ts (PLACEHOLDER there).
	vehicleStaleMs: 3000,
	// Five status polls: a helper that answers none of them for this long is not being heard.
	helperStaleMs: 10_000
};
/** How often the helper's status is read (ms, a labelled harness value). */
export const STATUS_POLL_MS = 2000;
/** How often the bar is recomputed against the clock (ms, a labelled harness value). */
export const TICK_MS = 500;

export interface IndicatorField {
	name: string;
	value: string;
}

export interface Indicator {
	id: IndicatorId;
	/** The accessible name of the group. */
	label: string;
	summary: string;
	tone: Tone;
	stale: boolean;
	fields: IndicatorField[];
}

export interface BarInputs {
	helper: HelperConnectionState;
	status: HelperStatus | null;
	/** When the status was read (the shell's clock, ms); null if never. */
	statusAt: number | null;
	session: SessionState | null;
	identity: VehicleIdentity | null;
}

const SHORT_HEX = 12;
const MS_PER_S = 1000;
const BYTES_PER_GIB = 1024 ** 3;

/** "0.4 s"; "never" for no time. */
export function formatAge(ms: number | null): string {
	return ms === null ? 'never' : `${(Math.max(0, ms) / MS_PER_S).toFixed(1)} s`;
}

function age(at: number | null, now: number): number | null {
	return at === null ? null : now - at;
}

function staleAfter(at: number | null, now: number, limitMs: number): boolean {
	return at === null || now - at > limitMs;
}

/** Why the vehicle is (not) arming, from what the link carries: the heartbeat's armed bit and system status, and the config state of the identity. */
export function armingReason(session: SessionState | null, identity: VehicleIdentity | null): string {
	if (!session || session.armed === null) return 'no heartbeat received';
	if (session.armed) return `vehicle ${session.systemStatus ?? 'status unknown'}`;
	if (identity?.configState === 'refused') return `config refused (${identity.configRefusal}): the vehicle does not arm`;
	return `vehicle ${session.systemStatus ?? 'status unknown'}`;
}

export function computeBar(inputs: BarInputs, now: number, timing: ShellTiming = SHELL_TIMING): Indicator[] {
	const { helper, status, statusAt, session, identity } = inputs;
	const helperStale = staleAfter(statusAt, now, timing.helperStaleMs);
	const frameStale = staleAfter(session?.lastFrameAt ?? null, now, timing.vehicleStaleMs);
	const heartbeatStale = staleAfter(session?.lastHeartbeatAt ?? null, now, timing.vehicleStaleMs);

	const helperIndicator: Indicator = {
		id: 'helper',
		label: 'Helper',
		summary: helper,
		tone: helper === 'open' ? 'ok' : 'bad',
		stale: helperStale,
		fields: [
			{ name: 'status age', value: formatAge(age(statusAt, now)) },
			{
				name: 'autoconnect',
				value: !status ? 'unknown' : !status.autoconnect.enabled ? 'off' : status.autoconnect.suspended ? `suspended: ${status.autoconnect.suspended}` : 'on'
			}
		]
	};

	const heartbeat = session?.heartbeat ?? 'no heartbeat';
	const linkIndicator: Indicator = {
		id: 'link',
		label: 'Link',
		summary: heartbeat,
		tone: heartbeat === 'alive' && !frameStale ? 'ok' : 'warn',
		stale: frameStale,
		fields: [
			{ name: 'device', value: status ? (status.device ?? 'none') : 'unknown' },
			{ name: 'rx age', value: formatAge(age(session?.lastFrameAt ?? null, now)) },
			{ name: 'heartbeat age', value: formatAge(age(session?.lastHeartbeatAt ?? null, now)) }
		]
	};

	const refused = identity?.configState === 'refused';
	const identityIndicator: Indicator = identity
		? {
				id: 'identity',
				label: 'Identity',
				summary: `image ${identity.imageSha256.slice(0, SHORT_HEX)}`,
				tone: refused ? 'bad' : 'ok',
				stale: frameStale,
				fields: [
					{ name: 'config', value: identity.configSha256.slice(0, SHORT_HEX) },
					{ name: 'source', value: identity.configSource },
					{ name: 'state', value: refused ? `refused: ${identity.configRefusal}` : 'ok' },
					{ name: 'report', value: identity.validationReportId ? identity.validationReportId.slice(0, SHORT_HEX) : 'none' },
					{ name: 'staged', value: identity.stagedDiffers ? 'differs' : 'same' }
				]
			}
		: { id: 'identity', label: 'Identity', summary: 'not reported', tone: 'neutral', stale: true, fields: [] };

	const armed = session?.armed ?? null;
	const armingIndicator: Indicator = {
		id: 'arming',
		label: 'Arming',
		summary: armed === null ? 'unknown' : armed ? 'armed' : 'disarmed',
		tone: armed === null ? 'neutral' : armed ? 'bad' : 'ok',
		stale: heartbeatStale,
		fields: [{ name: 'reason', value: armingReason(session, identity) }]
	};

	const active = status?.runs.active ?? null;
	const last = status?.runs.last ?? null;
	const runIndicator: Indicator = {
		id: 'run',
		label: 'Run',
		summary: !status ? 'unknown' : active ? `${active.kind} running` : last ? `${last.kind} ${last.state}` : 'none',
		tone: !status ? 'neutral' : active ? 'ok' : 'neutral',
		stale: helperStale,
		fields: !status || !last
			? []
			: [
					{ name: 'run', value: last.run_id },
					{ name: 'relaunches', value: String(last.relaunches) },
					{ name: 'exit', value: last.exit_status === null ? '—' : String(last.exit_status) }
				]
	};

	const host = status?.host ?? null;
	const lock = host?.heavy_lock ?? null;
	const gib = (bytes: number | null): string => (bytes === null ? '—' : (bytes / BYTES_PER_GIB).toFixed(1));
	const hostIndicator: Indicator = {
		id: 'host',
		label: 'Host',
		summary: !host ? 'unknown' : lock?.held ? 'heavy lock held' : 'heavy lock free',
		tone: !host ? 'neutral' : lock?.held ? 'warn' : 'ok',
		stale: helperStale,
		fields: !host
			? []
			: [
					{ name: 'load', value: host.loadavg ? `${host.loadavg[0]?.toFixed(2) ?? '—'} / ${host.cpus ?? '—'} cpus` : '—' },
					{ name: 'mem free', value: `${gib(host.mem_available_bytes)} / ${gib(host.mem_total_bytes)} GiB` },
					{ name: 'lock holder', value: lock?.held ? (lock.holder ?? 'unknown') : 'none' }
				]
	};

	return [helperIndicator, linkIndicator, identityIndicator, armingIndicator, runIndicator, hostIndicator];
}

export interface AutoconnectEvent {
	state: string;
	source: string | null;
	device: string | null;
	reason: string | null;
}

export interface ShellDeps {
	readStatus(): Promise<HelperStatus>;
	onEvent(handler: (event: HelperEvent) => void): () => void;
	helperState: Readable<HelperConnectionState>;
	session: Readable<SessionState> | null;
	identity: Readable<VehicleIdentity | null> | null;
	/** The clock (ms); Date.now by default. */
	now?: () => number;
	/** Runs a read and records its failure under `source` (Problems.run). */
	guard(source: string, what: () => Promise<void>): Promise<unknown>;
	/** A device was opened: forget the old vehicle. */
	deviceOpened(): void;
	/** The device closed: forget the vehicle. */
	deviceClosed(): void;
	/** The SIL was started again on the same link: forget the vehicle, so that its identity is read again from the next beat. */
	relaunched(): void;
	timing?: ShellTiming;
}

/** Keeps the status bar true: polls the helper's status, ticks the clock and reacts to the helper's events. */
export class ShellController {
	private readonly now: () => number;
	private readonly timing: ShellTiming;
	private readonly statusStore = writable<HelperStatus | null>(null);
	private readonly statusAt = writable<number | null>(null);
	private readonly deviceStore = writable<string | null>(null);
	private readonly autoconnectStore = writable<AutoconnectEvent | null>(null);
	private readonly barStore = writable<Indicator[]>([]);
	private pollTimer: ReturnType<typeof setInterval> | null = null;
	private tickTimer: ReturnType<typeof setInterval> | null = null;
	private offEvent: (() => void) | null = null;
	private readonly unsubscribes: Array<() => void> = [];

	readonly status: Readable<HelperStatus | null> = { subscribe: this.statusStore.subscribe };
	/** The device the helper has open, from its events and status. */
	readonly device: Readable<string | null> = { subscribe: this.deviceStore.subscribe };
	/** The latest autoconnect event the helper pushed. */
	readonly autoconnect: Readable<AutoconnectEvent | null> = { subscribe: this.autoconnectStore.subscribe };
	readonly bar: Readable<Indicator[]> = { subscribe: this.barStore.subscribe };

	private helperValue: HelperConnectionState = 'closed';
	private sessionValue: SessionState | null = null;
	private identityValue: VehicleIdentity | null = null;
	private statusAtValue: number | null = null;
	private statusValue: HelperStatus | null = null;

	constructor(private readonly deps: ShellDeps) {
		this.now = deps.now ?? (() => Date.now());
		this.timing = deps.timing ?? SHELL_TIMING;
		this.unsubscribes.push(
			deps.helperState.subscribe((v) => {
				this.helperValue = v;
				this.recompute();
			}),
			this.statusStore.subscribe((v) => (this.statusValue = v)),
			this.statusAt.subscribe((v) => (this.statusAtValue = v))
		);
		if (deps.session) {
			this.unsubscribes.push(
				deps.session.subscribe((v) => {
					this.sessionValue = v;
					this.recompute();
				})
			);
		}
		if (deps.identity) {
			this.unsubscribes.push(
				deps.identity.subscribe((v) => {
					this.identityValue = v;
					this.recompute();
				})
			);
		}
		this.recompute();
	}

	/** One read of the helper's status; a failure goes to the problems under 'status' and leaves the old status (which then goes stale). */
	async refresh(): Promise<void> {
		await this.deps.guard('status', async () => {
			const s = await this.deps.readStatus();
			this.statusStore.set(s);
			this.statusAt.set(this.now());
			this.deviceStore.set(s.device);
			this.recompute();
		});
	}

	/** Starts the polling, the tick and the event handling; a second call does nothing. */
	start(): void {
		if (this.pollTimer) return;
		this.offEvent = this.deps.onEvent((event) => this.onEvent(event));
		this.pollTimer = setInterval(() => void this.refresh(), STATUS_POLL_MS);
		this.tickTimer = setInterval(() => this.recompute(), TICK_MS);
		void this.refresh();
	}

	stop(): void {
		if (this.pollTimer) clearInterval(this.pollTimer);
		if (this.tickTimer) clearInterval(this.tickTimer);
		this.pollTimer = null;
		this.tickTimer = null;
		this.offEvent?.();
		this.offEvent = null;
	}

	dispose(): void {
		this.stop();
		for (const u of this.unsubscribes.splice(0)) u();
	}

	/** What the helper tells of itself, by event. Every event is followed by a re-read of the status. */
	onEvent(event: HelperEvent): void {
		if (event.event === 'device_opened') {
			this.deviceStore.set(typeof event.device === 'string' ? event.device : null);
			this.deps.deviceOpened();
		} else if (event.event === 'device_closed') {
			this.deviceStore.set(null);
			this.deps.deviceClosed();
		} else if (event.event === 'sil_relaunched') {
			this.deps.relaunched();
		} else if (event.event === 'autoconnect') {
			const text = (v: unknown): string | null => (typeof v === 'string' ? v : null);
			this.autoconnectStore.set({ state: text(event.state) ?? 'unknown', source: text(event.source), device: text(event.device), reason: text(event.reason) });
		}
		void this.refresh();
	}

	private recompute(): void {
		const now = this.now();
		this.barStore.set(
			computeBar(
				{ helper: this.helperValue, status: this.statusValue, statusAt: this.statusAtValue, session: this.sessionValue, identity: this.identityValue },
				now,
				this.timing
			)
		);
	}

	/** The bar as of now (for a test). */
	current(): Indicator[] {
		return get(this.barStore);
	}
}
