// The Build panel's logic (GCS-first unit GX-14): the helper (tools/gs/build.py) builds the presets on its allow-list, on an explicit click
// only, under the host-wide heavy lock. The browser sends a preset name and nothing else, holds no copy of the allow-list (the helper
// lists it), and shows every refusal as the helper words it. A build runs in the helper (it answers at once and pushes build_started,
// build_progress and build_done), so a cold build is a progress tail, not a request that times out. The panel never builds by itself:
// opening the page asks for the status (a dry run) and nothing more.
import { get, writable, type Readable } from 'svelte/store';
import { HelperError, type HelperEvent, type HelperReply } from '$link/transport';

/** One entry of the helper's allow-list. */
export interface BuildPresetInfo {
	preset: string;
	tree: string;
	configure: string;
	cache: Record<string, string>;
	targets: string[];
}

/** The `ninja -n` status of a preset: "up to date", "N steps", "not configured" or an error text. */
export interface DryRun {
	kind: 'up_to_date' | 'steps' | 'not_configured' | 'error';
	steps: number | null;
	text: string;
}

/** A build as the helper keeps it: the running one or the latest. */
export interface BuildRecord {
	running: boolean;
	preset: string;
	tree: string;
	describe: string;
	started_utc: string;
	finished_utc: string | null;
	ok: boolean | null;
	cancelled: boolean;
	exit_status: number | null;
	error: string | null;
	step: number | null;
	steps: number | null;
	tail: string[];
	log: string;
	images: Array<{ hash: string; name: string; kind: string }>;
}

export interface BuildState {
	/** The helper's allow-list; null until the helper answered. */
	allowed: BuildPresetInfo[] | null;
	preset: string;
	status: DryRun | null;
	/** The reasons the helper would refuse a build now (verbatim). */
	refusals: string[];
	current: BuildRecord | null;
	/** The helper's refusals of the last command, and errors. */
	notices: string[];
	busy: 'status' | 'build' | 'cancel' | null;
}

export interface BuildDeps {
	request(cmd: string, args?: Record<string, unknown>): Promise<HelperReply>;
	onEvent(handler: (event: HelperEvent) => void): () => void;
	/** The image library is read again when a build ends (the new image and its describe string). */
	refreshImages(): Promise<void>;
}

/** PLACEHOLDER (harness value): the progress lines the panel keeps; the helper keeps its own tail (tools/gs/build.py BUILD_TAIL_LINES). */
export const TAIL_LINES = 40;

const isObject = (v: unknown): v is Record<string, unknown> => typeof v === 'object' && v !== null && !Array.isArray(v);

/** The reasons of a failed command: the reasons the helper lists (all of them), else its one error text. */
export function reasonsOf(e: unknown): string[] {
	if (e instanceof HelperError) {
		const list = e.reply.refusals;
		if (Array.isArray(list) && list.length > 0 && list.every((x) => typeof x === 'string')) return list as string[];
	}
	return [e instanceof Error ? e.message : String(e)];
}

/** The helper's build record, or null for something that is not one. */
export function recordOf(v: unknown): BuildRecord | null {
	if (!isObject(v) || typeof v.preset !== 'string' || typeof v.describe !== 'string' || typeof v.running !== 'boolean') return null;
	return { ...(v as unknown as BuildRecord), tail: Array.isArray(v.tail) ? (v.tail as string[]) : [], images: Array.isArray(v.images) ? (v.images as BuildRecord['images']) : [] };
}

function allowedOf(v: unknown): BuildPresetInfo[] | null {
	if (!Array.isArray(v)) return null;
	return v.filter((p): p is BuildPresetInfo => isObject(p) && typeof p.preset === 'string' && typeof p.tree === 'string') as BuildPresetInfo[];
}

function dryRunOf(v: unknown): DryRun | null {
	return isObject(v) && typeof v.text === 'string' && typeof v.kind === 'string' ? (v as unknown as DryRun) : null;
}

/** What the page shows of a record: running, ok, failed or cancelled. */
export function buildOutcome(r: BuildRecord | null): 'none' | 'running' | 'ok' | 'failed' | 'cancelled' {
	if (!r) return 'none';
	if (r.running) return 'running';
	if (r.cancelled) return 'cancelled';
	return r.ok ? 'ok' : 'failed';
}

/** Why the Build button is off, or null when it is on. The helper's refusals are shown beside it, not decided here. */
export function whyNoBuild(s: BuildState, helperOpen: boolean): string | null {
	if (!helperOpen) return 'the helper is not connected';
	if (s.busy) return 'a command is in progress';
	if (s.current?.running) return `a build is running (${s.current.preset}): one build at a time`;
	if (s.preset === '') return 'pick a preset';
	if (s.refusals.length > 0) return s.refusals[0] ?? null;
	return null;
}

export class BuildController {
	private readonly store = writable<BuildState>({ allowed: null, preset: '', status: null, refusals: [], current: null, notices: [], busy: null });
	readonly state: Readable<BuildState> = { subscribe: this.store.subscribe };
	private off: (() => void) | null = null;

	constructor(private readonly deps: BuildDeps) {}

	/** Starts following the helper's build events; returns what stops it. A second call does nothing. */
	start(): () => void {
		if (!this.off) this.off = this.deps.onEvent((e) => this.onEvent(e));
		return () => {
			this.off?.();
			this.off = null;
		};
	}

	private onEvent(e: HelperEvent): void {
		if (typeof e.preset !== 'string') return;
		if (e.event === 'build_started') {
			this.store.update((s) => ({
				...s,
				current: {
					running: true, preset: e.preset as string, tree: String(e.tree ?? ''), describe: String(e.describe ?? ''), started_utc: String(e.started_utc ?? ''),
					finished_utc: null, ok: null, cancelled: false, exit_status: null, error: null, step: null, steps: null, tail: [], log: '', images: []
				}
			}));
		} else if (e.event === 'build_progress') {
			const line = typeof e.line === 'string' ? e.line : '';
			this.store.update((s) =>
				s.current && s.current.running && s.current.preset === e.preset
					? {
							...s,
							current: {
								...s.current,
								step: typeof e.step === 'number' ? e.step : s.current.step,
								steps: typeof e.steps === 'number' ? e.steps : s.current.steps,
								tail: [...s.current.tail.slice(-(TAIL_LINES - 1)), line]
							}
						}
					: s
			);
		} else if (e.event === 'build_done') {
			this.store.update((s) => ({
				...s,
				current: s.current
					? {
							...s.current,
							running: false,
							ok: e.ok === true,
							cancelled: e.cancelled === true,
							exit_status: typeof e.exit_status === 'number' ? e.exit_status : null,
							error: typeof e.error === 'string' ? e.error : null,
							finished_utc: typeof e.finished_utc === 'string' ? e.finished_utc : null,
							images: Array.isArray(e.images) ? (e.images as BuildRecord['images']) : []
						}
					: s.current
			}));
			void this.afterDone();
		}
	}

	private async afterDone(): Promise<void> {
		try {
			await this.deps.refreshImages();
		} catch {
			// the library is shown as it was; the page's own problem list reports the failed read
		}
		await this.refresh();
	}

	/** Asks the helper for the allow-list, the latest build and the status (a dry run) of the chosen preset. Starts nothing. */
	async refresh(): Promise<void> {
		if (get(this.store).busy === 'status') return;
		this.store.update((s) => ({ ...s, busy: s.busy ?? 'status' }));
		try {
			const first = get(this.store);
			const args: Record<string, unknown> = first.preset !== '' ? { preset: first.preset } : {};
			let reply = await this.deps.request('build_status', args);
			let block = isObject(reply.build) ? reply.build : {};
			const allowed = allowedOf(block.allowed);
			const chosen = first.preset !== '' ? first.preset : (allowed?.[0]?.preset ?? '');
			if (first.preset === '' && chosen !== '') {
				reply = await this.deps.request('build_status', { preset: chosen });
				block = isObject(reply.build) ? reply.build : {};
			}
			this.store.update((s) => ({
				...s,
				allowed: allowedOf(block.allowed) ?? s.allowed,
				preset: chosen,
				status: dryRunOf(block.status),
				refusals: Array.isArray(block.refusals) ? (block.refusals as string[]) : [],
				current: recordOf(block.current) ?? s.current,
				notices: [],
				busy: s.busy === 'status' ? null : s.busy
			}));
		} catch (e) {
			this.store.update((s) => ({ ...s, notices: reasonsOf(e), busy: s.busy === 'status' ? null : s.busy }));
		}
	}

	async setPreset(preset: string): Promise<void> {
		this.store.update((s) => ({ ...s, preset, status: null, refusals: [], notices: [] }));
		await this.refresh();
	}

	/** The explicit click: asks the helper to build the chosen preset. The helper answers at once; progress and the end are events. */
	async build(): Promise<void> {
		const s = get(this.store);
		if (s.busy || s.preset === '') return;
		this.store.update((st) => ({ ...st, busy: 'build', notices: [] }));
		try {
			const reply = await this.deps.request('build', { preset: s.preset });
			const record = recordOf(reply.build);
			this.store.update((st) => ({ ...st, busy: null, current: st.current?.running && st.current.preset === s.preset ? st.current : (record ?? st.current) }));
		} catch (e) {
			this.store.update((st) => ({ ...st, busy: null }));
			await this.refresh(); // the refusals shown beside the button are read again, then the refusal of this click is shown
			this.store.update((st) => ({ ...st, notices: reasonsOf(e) }));
		}
	}

	async cancel(): Promise<void> {
		if (get(this.store).busy) return;
		this.store.update((st) => ({ ...st, busy: 'cancel', notices: [] }));
		try {
			await this.deps.request('build_cancel');
			this.store.update((st) => ({ ...st, busy: null }));
		} catch (e) {
			this.store.update((st) => ({ ...st, busy: null, notices: reasonsOf(e) }));
		}
	}
}
