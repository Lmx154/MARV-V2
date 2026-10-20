// The Bench page's logic (GCS-first unit GX-10): the preflight of an S4b hardware-in-the-loop launch with every refusal shown as the
// helper words it, the hold that keeps autoconnect off the board, the launch and stop, the verdict form, and the SWD flash of the bench
// images. The page talks to the helper only (tools/gs/hil.py): it opens no device and checks nothing itself, so a refusal is always the
// helper's own text. The motor-capable bench images are listed but the helper refuses them in this unit.
import { get, writable, type Readable } from 'svelte/store';
import { HelperError, type HelperReply } from '$link/transport';
import type { HelperStatus, RunManifest } from './helper_api';

export type CheckState = 'ok' | 'refused' | 'skipped';

/** One check of the helper's preflight (tools/gs/hil.py): probe, board, hil_image, esc_battery, host_idle, ports. */
export interface PreflightCheck {
	id: string;
	state: CheckState;
	detail: string;
}

export interface Preflight {
	allowed: boolean;
	refusals: string[];
	checks: PreflightCheck[];
	port: string | null;
	seconds: number;
	argv: string[] | null;
}

/** The checks in the order the helper lists them, with the words the page shows for each. */
export const CHECK_TITLES: Record<string, string> = {
	probe: 'Debug probe',
	board: 'Board',
	hil_image: 'HIL image',
	esc_battery: 'ESC battery unplugged',
	host_idle: 'Host idle',
	ports: 'Board ports closed, autoconnect suspended'
};

export const VERDICT_MODES = ['ANGLE', 'ACRO'] as const;
export type VerdictMode = (typeof VERDICT_MODES)[number];

/** The verdict as the helper stores it (<run dir>/runs/<run id>/out/verdict.json). */
export interface VerdictRecord {
	schema: number;
	run_id: string;
	mode: VerdictMode;
	text: string;
	saved_utc: string;
	report: string | null;
}

/** The image kinds of the SWD flash (tools/gs/hil.py BENCH_KINDS); a motor-capable kind is refused by the helper in this unit. */
export const BENCH_KINDS: ReadonlyArray<{ kind: string; label: string; motorCapable: boolean }> = [
	{ kind: 'hil', label: 'hil: the HIL receiver image (drives no motor pad)', motorCapable: false },
	{ kind: 'bench-log', label: 'bench-log: the SD logger bench image (motor-capable)', motorCapable: true },
	{ kind: 'bench-profile', label: 'bench-profile: the motor-step profile image (motor-capable)', motorCapable: true }
];

/** The flash.sh run the helper reports in its status. */
export interface SwdFlash {
	running: boolean;
	kind: string;
	elf: string;
	elf_sha256: string;
	ok: boolean | null;
	exit_status: number | null;
	error?: string | null;
	tail: string[];
}

/** The status the helper adds for the Bench page (tools/gs/hil.py HilGcs.status). */
export interface HilStatus {
	hold: boolean;
	hold_run: string | null;
	verdict: VerdictRecord | null;
	swd_flash: SwdFlash | null;
}

/** The seconds an S4b run is asked for when the field is left alone (tools/gs/hil.py S4B_SECONDS, the S4b runbook's --seconds 300). */
export const DEFAULT_SECONDS = '300';

// A decimal number as JSON writes it (the same rule as the Simulation page's values).
const DECIMAL = /^[+]?(?:[0-9]+\.?[0-9]*|\.[0-9]+)(?:[eE][+-]?[0-9]+)?$/;

/** The seconds text as a positive finite number; null when it is not one. */
export function parseSeconds(text: string): number | null {
	const t = text.trim();
	if (!DECIMAL.test(t)) return null;
	const n = Number(t);
	return Number.isFinite(n) && n > 0 ? n : null;
}

/** The reasons of a failed command: the reasons the helper lists (all of them), else its one error text. */
export function reasonsOf(e: unknown): string[] {
	if (e instanceof HelperError) {
		const list = e.reply.refusals;
		if (Array.isArray(list) && list.length > 0 && list.every((x) => typeof x === 'string')) return list as string[];
	}
	return [e instanceof Error ? e.message : String(e)];
}

/** What the page shows of the helper's status: the latest HIL run, the hold, the saved verdict and the flash. */
export interface BenchView {
	/** The latest run of this helper session when it is a HIL run, else null. */
	run: RunManifest | null;
	running: boolean;
	hold: boolean;
	verdict: VerdictRecord | null;
	swd: SwdFlash | null;
	/** Where the run's files, the verdict among them, are (the run's out directory). */
	outDir: string | null;
}

export function benchView(status: HelperStatus | null): BenchView {
	const last = status?.runs.last ?? null;
	const run = last && last.kind === 'hil' ? last : null;
	const hil = status?.hil ?? null;
	return {
		run,
		running: run !== null && status?.runs.active?.id === run.run_id,
		hold: hil?.hold ?? false,
		verdict: hil?.verdict ?? null,
		swd: hil?.swd_flash ?? null,
		outDir: run?.out_dir ?? null
	};
}

export interface VerdictForm {
	mode: VerdictMode | '';
	text: string;
}

export interface BenchState {
	/** The acknowledgement the operator ticks; the helper asks for it again on every preflight and launch. */
	ack: boolean;
	seconds: string;
	preflight: Preflight | null;
	/** The helper's refusals of the last command (verbatim), shown next to the buttons. */
	notices: string[];
	verdictForm: VerdictForm;
	verdictNotices: string[];
	kind: string;
	flashNotices: string[];
	busy: 'hold' | 'preflight' | 'launch' | 'stop' | 'verdict' | 'flash' | null;
}

export interface BenchDeps {
	request(cmd: string, args?: Record<string, unknown>): Promise<HelperReply>;
	/** Reads the helper's status again (the page shows the run, the hold and the flash from it). */
	refresh(): Promise<void>;
}

export class BenchController {
	private readonly store = writable<BenchState>({
		ack: false,
		seconds: DEFAULT_SECONDS,
		preflight: null,
		notices: [],
		verdictForm: { mode: '', text: '' },
		verdictNotices: [],
		kind: BENCH_KINDS[0]!.kind,
		flashNotices: [],
		busy: null
	});
	readonly state: Readable<BenchState> = { subscribe: this.store.subscribe };

	constructor(private readonly deps: BenchDeps) {}

	/** A change of the launch form: the checklist and the messages were about the form as it was. */
	private edit(change: (s: BenchState) => Partial<BenchState>): void {
		this.store.update((s) => ({ ...s, ...change(s), preflight: null, notices: [] }));
	}

	setAck(ack: boolean): void {
		this.edit(() => ({ ack }));
	}

	setSeconds(seconds: string): void {
		this.edit(() => ({ seconds }));
	}

	setMode(mode: VerdictMode | ''): void {
		this.store.update((s) => ({ ...s, verdictForm: { ...s.verdictForm, mode }, verdictNotices: [] }));
	}

	setVerdictText(text: string): void {
		this.store.update((s) => ({ ...s, verdictForm: { ...s.verdictForm, text }, verdictNotices: [] }));
	}

	setKind(kind: string): void {
		this.store.update((s) => ({ ...s, kind, flashNotices: [] }));
	}

	private begin(busy: NonNullable<BenchState['busy']>): boolean {
		if (get(this.store).busy) return false;
		this.store.update((s) => ({ ...s, busy }));
		return true;
	}

	private end(change: Partial<BenchState>): void {
		this.store.update((s) => ({ ...s, ...change, busy: null }));
	}

	/** The options of a preflight or a launch, or null with the notice that keeps the form from being sent. */
	private options(): Record<string, unknown> | null {
		const s = get(this.store);
		const seconds = parseSeconds(s.seconds);
		if (seconds === null) {
			this.store.update((st) => ({ ...st, notices: [`"${s.seconds}" is not a positive number of seconds`] }));
			return null;
		}
		return { esc_battery_unplugged: s.ack, seconds };
	}

	/** Takes the hold (closes the helper's serial port, suspends autoconnect) or releases it. */
	async setHold(on: boolean): Promise<void> {
		if (!this.begin('hold')) return;
		try {
			await this.deps.request('hil_hold', { on });
			this.end({ notices: [], preflight: null });
		} catch (e) {
			this.end({ notices: reasonsOf(e) });
		}
		await this.deps.refresh();
	}

	/** Asks the helper for the six checks; starts nothing. */
	async check(): Promise<void> {
		const options = this.options();
		if (!options || !this.begin('preflight')) return;
		try {
			const preflight = (await this.deps.request('hil_preflight', options)).preflight as Preflight;
			this.end({ preflight, notices: [] });
		} catch (e) {
			this.end({ preflight: null, notices: reasonsOf(e) });
		}
	}

	async launch(): Promise<void> {
		const options = this.options();
		if (!options || !this.begin('launch')) return;
		try {
			await this.deps.request('hil_start', options);
			this.end({ preflight: null, notices: [] });
		} catch (e) {
			const preflight = e instanceof HelperError && e.reply.preflight ? (e.reply.preflight as Preflight) : null;
			this.end({ preflight, notices: reasonsOf(e) });
		}
		await this.deps.refresh();
	}

	async stop(): Promise<void> {
		if (!this.begin('stop')) return;
		try {
			await this.deps.request('run_stop');
			this.end({ notices: [] });
		} catch (e) {
			this.end({ notices: reasonsOf(e) });
		}
		await this.deps.refresh();
	}

	/** Saves the verdict of the run `runId` (the helper writes it once, next to the run's report). */
	async saveVerdict(runId: string): Promise<void> {
		const form = get(this.store).verdictForm;
		if (form.mode === '') {
			this.store.update((s) => ({ ...s, verdictNotices: ['choose the flight mode flown: ANGLE or ACRO'] }));
			return;
		}
		if (!this.begin('verdict')) return;
		try {
			await this.deps.request('hil_verdict', { run_id: runId, mode: form.mode, text: form.text });
			this.end({ verdictNotices: [] });
		} catch (e) {
			this.end({ verdictNotices: reasonsOf(e) });
		}
		await this.deps.refresh();
	}

	/** Flashes the chosen bench image kind over SWD; the helper refuses a motor-capable kind. */
	async flash(): Promise<void> {
		if (!this.begin('flash')) return;
		try {
			await this.deps.request('flash_swd', { kind: get(this.store).kind });
			this.end({ flashNotices: [] });
		} catch (e) {
			this.end({ flashNotices: reasonsOf(e) });
		}
		await this.deps.refresh();
	}
}
