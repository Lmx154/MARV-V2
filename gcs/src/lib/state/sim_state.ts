// The Simulation page's logic (GCS-first unit GX-3): the run definition form, its mapping to the helper's run_plan / run_start
// request, where each of the helper's refusals is shown, and the controller that plans, launches and stops a SIL run.
//
// The page speaks to the helper only (S-1): the run state comes from the helper's status, which the shell reads; this controller
// never opens a device. On a launch the helper's autoconnect opens the vehicle's link, and the status bar shows it.
// Every override is a SIL override: a firmware parameter value forced in the simulation only, never in a config that goes to a
// board (invariant O-1). The page marks every override row and the run with SIL_LABEL.
import { get, writable, type Readable } from 'svelte/store';
import { HelperError, type HelperReply } from '$link/transport';
import { parseManifest } from '$lib/params/manifest';
import type { ImageList } from '$lib/params/library';
import type { HelperStatus, RunManifest } from './helper_api';

/** The label of a run with an override and of its overrides (tools/sim/sil_run.py LABEL; the helper's plan and manifest carry it). */
export const SIL_LABEL = 'sil-override';

export interface OverrideRow {
	/** A key for the row in the page; not sent. */
	id: number;
	name: string;
	/** The text of the value as typed. */
	value: string;
}

export interface SimForm {
	/** A repository-relative `vehicles/<name>.yaml`; '' leaves the choice to the run script (no `card` in the request). */
	card: string;
	overrides: OverrideRow[];
	gui: boolean;
	/** Fly the run from a USB joystick (GX-9): the helper gives the run its pilot port and its world no stick script. */
	pilot?: boolean;
}

/** The options of a sil run (tools/gs/runs.py OPTIONS). */
export interface RunRequest {
	kind: 'sil';
	card?: string;
	overrides?: Array<{ name: string; value: number }>;
	gui: boolean;
	pilot?: boolean;
}

/** The build tree whose parameter manifest the helper checks overrides against (tools/sim/sil_run.py GZ_BUILD). */
export const SIL_BUILD_TREE = 'host-gz-quadfc';

/** A firmware parameter that the helper accepts as an override. */
export interface SimParameter {
	name: string;
	type: string;
	unit: string;
	locked: boolean;
}

/** What the helper's run_plan answers for a sil run (tools/gs/runs.py RunSupervisor.plan, and its refusals). */
export interface RunPlan {
	kind: string;
	run_id: string;
	argv: string[];
	options: Record<string, unknown>;
	label: string | null;
	overrides: Array<{ name: string; type: string; value: number }>;
	out_dir: string;
	udp_port: number | null;
	allowed: boolean;
	refusals: string[];
}

/** Where a message is shown: next to the card field, next to an override row, or with the run's buttons. */
export type Target = { kind: 'card' } | { kind: 'row'; id: number } | { kind: 'run' };

export interface Notice {
	/** The helper's text, verbatim; or the form's own, for something it will not send. */
	text: string;
	source: 'helper' | 'form';
}

export interface Placed {
	card: Notice[];
	rows: Record<number, Notice[]>;
	run: Notice[];
}

export const emptyPlaced = (): Placed => ({ card: [], rows: {}, run: [] });

// A decimal number as JSON writes it: Number() alone would also read '', '0x10' and 'Infinity'.
const DECIMAL = /^[+-]?(?:[0-9]+\.?[0-9]*|\.[0-9]+)(?:[eE][+-]?[0-9]+)?$/;

/** The value text as a number; null when it is not a finite decimal number. */
export function parseValue(text: string): number | null {
	const t = text.trim();
	if (!DECIMAL.test(t)) return null;
	const n = Number(t);
	return Number.isFinite(n) ? n : null;
}

const isBlank = (r: OverrideRow): boolean => r.name.trim() === '' && r.value.trim() === '';

/** The request a form stands for, or the problems that keep it from being sent (a blank row is no override). */
export function buildRequest(form: SimForm): { request: RunRequest; problems: [] } | { request: null; problems: Array<{ target: Target; notice: Notice }> } {
	const problems: Array<{ target: Target; notice: Notice }> = [];
	const overrides: Array<{ name: string; value: number }> = [];
	for (const row of form.overrides) {
		if (isBlank(row)) continue;
		const target: Target = { kind: 'row', id: row.id };
		const value = parseValue(row.value);
		if (row.name.trim() === '') problems.push({ target, notice: { text: 'name the parameter', source: 'form' } });
		else if (value === null) problems.push({ target, notice: { text: `"${row.value}" is not a number`, source: 'form' } });
		else overrides.push({ name: row.name.trim(), value });
	}
	if (problems.length > 0) return { request: null, problems };
	const request: RunRequest = { kind: 'sil', gui: form.gui };
	if (form.card !== '') request.card = form.card;
	if (overrides.length > 0) request.overrides = overrides;
	if (form.pilot) request.pilot = true;
	return { request, problems: [] };
}

/** The override row that a helper message about an override names (tools/sim/sil_run.py: `override <name>: ...`, `override '<name>': ...`). */
function rowOf(message: string, form: SimForm): OverrideRow | null {
	const m = /^override (?:'([^']*)'|"([^"]*)"|([^:]*)):/.exec(message);
	if (!m) return null;
	const name = m[1] ?? m[2] ?? m[3] ?? '';
	const rows = form.overrides.filter((r) => r.name.trim() === name);
	if (rows.length === 0) return null;
	return message.endsWith('given twice') ? rows[rows.length - 1]! : rows[0]!;
}

/** Puts each message where it concerns: a card message by the card, an override message by its row, any other (a run already going,
 * the heavy lock held, no build) with the run's buttons. The text is never changed. */
export function placeMessages(messages: string[], form: SimForm, source: Notice['source'] = 'helper'): Placed {
	const placed = emptyPlaced();
	for (const text of [...new Set(messages)]) {
		const notice: Notice = { text, source };
		const row = rowOf(text, form);
		if (row) (placed.rows[row.id] ??= []).push(notice);
		else if (/^(unknown card|card must be)/.test(text)) placed.card.push(notice);
		else placed.run.push(notice);
	}
	return placed;
}

/** The reasons of a failed command: the reasons the helper lists (all of them), else its one error text. */
export function reasonsOf(e: unknown): string[] {
	if (e instanceof HelperError) {
		const list = e.reply.refusals;
		if (Array.isArray(list) && list.length > 0 && list.every((x) => typeof x === 'string')) return list as string[];
	}
	return [e instanceof Error ? e.message : String(e)];
}

/** The run the page shows: the run that is going, else the latest of this helper session; null before any. */
export interface RunView {
	running: boolean;
	run: RunManifest | null;
	/** SIL_LABEL when the run carries an override. */
	label: string | null;
}

export function runView(status: HelperStatus | null): RunView {
	const run = status?.runs.last ?? null;
	return { running: status?.runs.active != null, run, label: run?.label ?? null };
}

export interface SimState {
	/** The cards the helper accepts; null until read. */
	cards: string[] | null;
	cardsError: string | null;
	/** The parameters of the SIL image's manifest, offered as names; null until read. */
	parameters: SimParameter[] | null;
	/** Why there is no list of parameters (the helper still checks every name). */
	parametersNote: string | null;
	form: SimForm;
	/** The plan last asked for, until the form changes. */
	plan: RunPlan | null;
	placed: Placed;
	busy: 'plan' | 'launch' | 'stop' | null;
}

export interface SimDeps {
	request(cmd: string, args?: Record<string, unknown>): Promise<HelperReply>;
	listImages(): Promise<ImageList>;
	artifact(hash: string, name: string): Promise<unknown>;
	/** Reads the helper's status again (the page shows the run from it). */
	refresh(): Promise<void>;
}

export class SimController {
	private nextRow = 1;
	private readonly store = writable<SimState>({
		cards: null,
		cardsError: null,
		parameters: null,
		parametersNote: null,
		form: { card: '', overrides: [], gui: false, pilot: false },
		plan: null,
		placed: emptyPlaced(),
		busy: null
	});
	readonly state: Readable<SimState> = { subscribe: this.store.subscribe };

	constructor(private readonly deps: SimDeps) {}

	async loadCards(): Promise<void> {
		try {
			const reply = await this.deps.request('cards');
			const cards = Array.isArray(reply.cards) ? reply.cards.filter((c): c is string => typeof c === 'string') : [];
			this.store.update((s) => ({ ...s, cards, cardsError: null }));
		} catch (e) {
			this.store.update((s) => ({ ...s, cardsError: e instanceof Error ? e.message : String(e) }));
		}
	}

	/** The names an override can have: the parameters of the manifest of the SIL image in the library (the file the helper checks against). */
	async loadParameters(): Promise<void> {
		try {
			const entry = (await this.deps.listImages()).images.find((e) => e.kind === 'sil' && e.tree === SIL_BUILD_TREE && e.artifacts.params_manifest);
			if (!entry) {
				this.store.update((s) => ({ ...s, parameters: null, parametersNote: `The image library holds no SIL image with a parameter manifest in ${SIL_BUILD_TREE}: build it; the helper refuses any name it cannot check.` }));
				return;
			}
			const manifest = parseManifest(await this.deps.artifact(entry.hash, 'params_manifest'));
			const parameters = manifest.params.map((p) => ({ name: p.name, type: p.type, unit: p.unit, locked: p.locked }));
			this.store.update((s) => ({ ...s, parameters, parametersNote: null }));
		} catch (e) {
			this.store.update((s) => ({ ...s, parameters: null, parametersNote: e instanceof Error ? e.message : String(e) }));
		}
	}

	/** A change of the form: the plan and the messages were about the form as it was. */
	private edit(change: (form: SimForm) => SimForm): void {
		this.store.update((s) => ({ ...s, form: change(s.form), plan: null, placed: emptyPlaced() }));
	}

	setCard(card: string): void {
		this.edit((f) => ({ ...f, card }));
	}

	setGui(gui: boolean): void {
		this.edit((f) => ({ ...f, gui }));
	}

	setPilot(pilot: boolean): void {
		this.edit((f) => ({ ...f, pilot }));
	}

	addOverride(): void {
		const id = this.nextRow++;
		this.edit((f) => ({ ...f, overrides: [...f.overrides, { id, name: '', value: '' }] }));
	}

	setOverride(id: number, patch: Partial<Pick<OverrideRow, 'name' | 'value'>>): void {
		this.edit((f) => ({ ...f, overrides: f.overrides.map((r) => (r.id === id ? { ...r, ...patch } : r)) }));
	}

	removeOverride(id: number): void {
		this.edit((f) => ({ ...f, overrides: f.overrides.filter((r) => r.id !== id) }));
	}

	/** The form as the helper's request, or the problems it was refused for here (shown by their rows). */
	private prepare(): RunRequest | null {
		const form = get(this.store).form;
		const built = buildRequest(form);
		if (built.request) return built.request;
		const placed = emptyPlaced();
		for (const { target, notice } of built.problems) {
			if (target.kind === 'row') (placed.rows[target.id] ??= []).push(notice);
			else if (target.kind === 'card') placed.card.push(notice);
			else placed.run.push(notice);
		}
		this.store.update((s) => ({ ...s, plan: null, placed }));
		return null;
	}

	private begin(busy: 'plan' | 'launch' | 'stop'): boolean {
		if (get(this.store).busy) return false;
		this.store.update((s) => ({ ...s, busy }));
		return true;
	}

	private end(change: Partial<SimState>): void {
		this.store.update((s) => ({ ...s, ...change, busy: null }));
	}

	/** What a launch would run, from the helper (it starts nothing). */
	async plan(): Promise<void> {
		const request = this.prepare();
		if (!request || !this.begin('plan')) return;
		const form = get(this.store).form;
		try {
			const plan = (await this.deps.request('run_plan', { ...request })).plan as RunPlan;
			this.end({ plan, placed: placeMessages(plan.refusals, form) });
		} catch (e) {
			this.end({ plan: null, placed: placeMessages(reasonsOf(e), form) });
		}
	}

	async launch(): Promise<void> {
		const request = this.prepare();
		if (!request || !this.begin('launch')) return;
		const form = get(this.store).form;
		try {
			await this.deps.request('run_start', { ...request });
			this.end({ plan: null, placed: emptyPlaced() });
		} catch (e) {
			this.end({ plan: null, placed: placeMessages(reasonsOf(e), form) });
		}
		await this.deps.refresh();
	}

	async stop(): Promise<void> {
		if (!this.begin('stop')) return;
		const form = get(this.store).form;
		try {
			await this.deps.request('run_stop');
			this.end({ plan: null, placed: emptyPlaced() });
		} catch (e) {
			this.end({ placed: placeMessages(reasonsOf(e), form) });
		}
		await this.deps.refresh();
	}
}
