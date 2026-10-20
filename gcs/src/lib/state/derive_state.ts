// The Derive panel's logic (GCS-first unit GX-13, unit 10 of the params-first ruling): a vehicle card, edits with the source text of each,
// the helper's derivation (tools/gs/derive.py: the repository's flatten, params_gen and .mcfg steps on a copy in the helper's run
// directory; the repository card is never written), its reports and lint findings, and the hand-over to the flows that exist: the helper's
// validation of the derived config for an image of the library (G7: the report is fetched and its id checked here, as the Parameters page
// does) and staging of its values on the vehicle, after which the Parameters page validates and activates them.
// The browser holds no copy of the rules. The helper lists the cards and their editable entries, judges every edit (a lock, a key, a lint
// finding, a missing source) and says why it refuses; this file only keeps the form, sends it and shows the answer. A derivation runs in
// the helper (it answers at once, then pushes derive_started, derive_step and derive_done), so a minute of flattening is a progress line
// and not a request that times out.
import { get, writable, type Readable } from 'svelte/store';
import { HelperError, type HelperEvent, type HelperReply } from '$link/transport';
import type { ParamType } from '$link/bits';
import { makeRecord, type ValidationRecord } from '$lib/params/validation';

export interface DeriveEntry {
	key: string;
	value: unknown;
	unit: string | null;
	shape: string | null;
	method: string | null;
	source: string | null;
	sigma: unknown;
	status: string[] | null;
	locked: boolean;
	lock: { by?: string; on?: string; via?: string } | null;
}

export interface DeriveFinding {
	file: string;
	entry: string;
	reason: string;
}

export interface DeriveChange {
	name: string;
	unit: string;
	type: string;
	from_bits: string;
	to_bits: string;
	method: string;
	locked: boolean;
}

export interface DeriveImage {
	hash: string;
	name: string;
	kind: string;
	tree: string;
}

/** A derivation as the helper keeps it: the running one or the latest. */
export interface DeriveRecord {
	id: string;
	running: boolean;
	state: string;
	ok: boolean | null;
	card: string;
	edits: Array<{ key: string; value: unknown; source: string; old_value: unknown; old_source: string | null }>;
	started_utc: string;
	finished_utc: string | null;
	error: string | null;
	findings: DeriveFinding[];
	step: string | null;
	tail: string[];
	config_hash: string | null;
	schema_hash: string | null;
	reports: string[];
	images: DeriveImage[];
	changes: DeriveChange[] | null;
	baseline: { hash: string; name: string; tree: string } | null;
}

export interface DeriveInput {
	value: string;
	source: string;
}

export interface DeriveValidation {
	phase: 'idle' | 'validating' | 'done' | 'failed';
	progress: string[];
	record: ValidationRecord | null;
	error: string | null;
}

export interface DeriveStaging {
	phase: 'idle' | 'staging' | 'done' | 'failed';
	staged: string[];
	same: number;
	notes: string[];
}

export interface DeriveState {
	cards: string[] | null;
	card: string;
	vehicle: string | null;
	entries: DeriveEntry[] | null;
	inputs: Record<string, DeriveInput>;
	current: DeriveRecord | null;
	/** The lint findings of the last refused derivation (verbatim) and the helper's other refusals and errors. */
	findings: DeriveFinding[];
	notices: string[];
	busy: 'cards' | 'card' | 'derive' | 'cancel' | 'report' | 'validate' | 'stage' | null;
	reportName: string | null;
	reportText: string | null;
	image: string;
	validation: DeriveValidation;
	staging: DeriveStaging;
}

/** The vehicle as staging needs it: the parameters it lists and the write of one with its source text. */
export interface DeriveVehicle {
	/** null when parameters can be written now, else why not. */
	whyCannotStage(): string | null;
	/** The parameter schema hash the connected vehicle's identity reports ("0x" and 16 hex digits), null until it has. */
	schemaHash(): string | null;
	params(): Array<{ linkId: string | null; stagedBits: number | null; locked: boolean | null }>;
	stage(linkId: string, type: ParamType, bits: number, source: string): Promise<{ accepted: boolean; reasons: string[] }>;
}

export interface DeriveDeps {
	request(cmd: string, args?: Record<string, unknown>): Promise<HelperReply>;
	onEvent(handler: (event: HelperEvent) => void): () => void;
	report(imageHash: string, configHash: string): Promise<Uint8Array>;
	vehicle: DeriveVehicle | null;
}

/** PLACEHOLDER (harness value): the progress lines the panel keeps; the helper keeps its own tail (tools/gs/derive.py DERIVE_TAIL_LINES). */
export const TAIL_LINES = 40;

const isObject = (v: unknown): v is Record<string, unknown> => typeof v === 'object' && v !== null && !Array.isArray(v);
const messageOf = (e: unknown): string => (e instanceof Error ? e.message : String(e));
const bytesToHex = (v: number): string => (v >>> 0).toString(16).padStart(8, '0');

/** The findings of a refused command: the lint's own lines when the helper lists them. */
export function findingsOf(v: unknown): DeriveFinding[] {
	if (!Array.isArray(v)) return [];
	return v.filter(isObject).map((f) => ({ file: String(f.file ?? ''), entry: String(f.entry ?? ''), reason: String(f.reason ?? '') }));
}

/** The reasons of a failed command: the refusals the helper lists (all of them), else its one error text. */
export function reasonsOf(e: unknown): string[] {
	if (e instanceof HelperError) {
		const list = e.reply.refusals;
		if (Array.isArray(list) && list.length > 0 && list.every((x) => typeof x === 'string')) return list as string[];
	}
	return [messageOf(e)];
}

/** The helper's derivation record, or null for something that is not one. */
export function recordOf(v: unknown): DeriveRecord | null {
	if (!isObject(v) || typeof v.id !== 'string' || typeof v.state !== 'string' || typeof v.card !== 'string') return null;
	return {
		...(v as unknown as DeriveRecord),
		running: v.running === true,
		edits: Array.isArray(v.edits) ? (v.edits as DeriveRecord['edits']) : [],
		findings: findingsOf(v.findings),
		tail: Array.isArray(v.tail) ? (v.tail as string[]) : [],
		reports: Array.isArray(v.reports) ? (v.reports as string[]) : [],
		images: Array.isArray(v.images) ? (v.images as DeriveImage[]) : [],
		changes: Array.isArray(v.changes) ? (v.changes as DeriveChange[]) : null,
		baseline: isObject(v.baseline) ? (v.baseline as DeriveRecord['baseline']) : null
	};
}

/** What the page shows of a record. */
export function deriveOutcome(r: DeriveRecord | null): 'none' | 'running' | 'ok' | 'failed' | 'cancelled' | 'refused' {
	if (!r) return 'none';
	if (r.running) return 'running';
	if (r.state === 'cancelled') return 'cancelled';
	if (r.state === 'refused') return 'refused';
	return r.ok ? 'ok' : 'failed';
}

export type ParsedValue = { ok: true; value: number | number[] | string } | { ok: false; error: string };

/**
 * The text typed for an entry as the value the helper takes: a number, a comma-separated list of numbers for an entry whose value is a list,
 * or the text itself for a categorical entry (a spin direction). Whether it is a legal value is the helper's lint's to say.
 */
export function parseValueInput(text: string, entry: DeriveEntry): ParsedValue {
	const t = text.trim();
	if (t === '') return { ok: false, error: 'no value' };
	if (typeof entry.value === 'string' && entry.value !== 'UNKNOWN') return { ok: true, value: t };
	const parts = t.split(/[\s,]+/).filter((p) => p !== '');
	const nums = parts.map(Number);
	if (nums.some((n) => !Number.isFinite(n)) || parts.some((p) => !/^[-+]?(\d+\.?\d*|\.\d+)([eE][-+]?\d+)?$/.test(p))) return { ok: false, error: `"${t}" is not a number or a list of numbers` };
	if (Array.isArray(entry.value)) {
		if (nums.length !== entry.value.length) return { ok: false, error: `${entry.key} takes ${entry.value.length} numbers, not ${nums.length}` };
		return { ok: true, value: nums };
	}
	if (nums.length !== 1) return { ok: false, error: `${entry.key} takes one number` };
	return { ok: true, value: nums[0] as number };
}

export function showValue(v: unknown): string {
	return Array.isArray(v) ? v.join(', ') : String(v);
}

const EMPTY_VALIDATION: DeriveValidation = { phase: 'idle', progress: [], record: null, error: null };
const EMPTY_STAGING: DeriveStaging = { phase: 'idle', staged: [], same: 0, notes: [] };

export class DeriveController {
	private readonly store = writable<DeriveState>({
		cards: null, card: '', vehicle: null, entries: null, inputs: {}, current: null, findings: [], notices: [], busy: null, reportName: null,
		reportText: null, image: '', validation: EMPTY_VALIDATION, staging: EMPTY_STAGING
	});
	readonly state: Readable<DeriveState> = { subscribe: this.store.subscribe };
	private off: (() => void) | null = null;

	constructor(private readonly deps: DeriveDeps) {}

	/** Starts following the helper's derive events; returns what stops it. A second call does nothing. */
	start(): () => void {
		if (!this.off) this.off = this.deps.onEvent((e) => this.onEvent(e));
		return () => {
			this.off?.();
			this.off = null;
		};
	}

	private patch(p: Partial<DeriveState>): void {
		this.store.update((s) => ({ ...s, ...p }));
	}

	private onEvent(e: HelperEvent): void {
		if (e.event === 'derive_started' && typeof e.id === 'string') {
			this.patch({
				current: {
					id: e.id, running: true, state: 'running', ok: null, card: String(e.card ?? ''), edits: [], started_utc: '', finished_utc: null, error: null, findings: [],
					step: null, tail: [], config_hash: null, schema_hash: null, reports: [], images: [], changes: null, baseline: null
				}
			});
		} else if (e.event === 'derive_step' && typeof e.step === 'string') {
			this.store.update((s) =>
				s.current && s.current.id === e.id
					? { ...s, current: { ...s.current, step: e.step as string, tail: [...s.current.tail.slice(-(TAIL_LINES - 1)), `step ${String(e.index)}/${String(e.steps)}: ${String(e.step)}`] } }
					: s
			);
		} else if (e.event === 'derive_done' && typeof e.id === 'string') {
			void this.afterDone(String(e.id));
		}
	}

	private async afterDone(id: string): Promise<void> {
		try {
			const reply = await this.deps.request('derive_status');
			const block = isObject(reply.derive) ? reply.derive : {};
			const rec = recordOf(block.current);
			if (rec && rec.id === id) {
				const images = rec.images;
				this.store.update((s) => ({
					...s, current: rec, findings: rec.findings, image: images.find((i) => i.hash === s.image)?.hash ?? images.find((i) => i.kind === 'board')?.hash ?? images[0]?.hash ?? '',
					validation: EMPTY_VALIDATION, staging: EMPTY_STAGING, reportName: null, reportText: null
				}));
			}
		} catch (e) {
			this.patch({ notices: reasonsOf(e) });
		}
	}

	/** Asks the helper for the cards and the editable entries of the first (or the chosen) one. Starts nothing. */
	async load(): Promise<void> {
		if (get(this.store).busy) return;
		this.patch({ busy: 'cards', notices: [] });
		try {
			const reply = await this.deps.request('cards');
			const cards = Array.isArray(reply.cards) ? (reply.cards as string[]) : [];
			const card = get(this.store).card !== '' ? get(this.store).card : (cards[0] ?? '');
			const status = await this.deps.request('derive_status');
			const block = isObject(status.derive) ? status.derive : {};
			this.patch({ cards, current: recordOf(block.current) ?? get(this.store).current });
			if (card !== '') await this.setCard(card);
		} catch (e) {
			this.patch({ notices: reasonsOf(e) });
		} finally {
			this.patch({ busy: null });
		}
	}

	async setCard(card: string): Promise<void> {
		this.patch({ card, entries: null, inputs: {}, findings: [], notices: [] });
		try {
			const reply = await this.deps.request('derive_card', { card });
			const entries = Array.isArray(reply.entries) ? (reply.entries as DeriveEntry[]) : [];
			this.patch({ entries, vehicle: typeof reply.vehicle === 'string' ? reply.vehicle : null });
		} catch (e) {
			this.patch({ notices: reasonsOf(e) });
		}
	}

	setInput(key: string, patch: Partial<DeriveInput>): void {
		this.store.update((s) => {
			const old = s.inputs[key] ?? { value: '', source: '' };
			const next = { ...old, ...patch };
			const inputs = { ...s.inputs };
			if (next.value === '' && next.source === '') delete inputs[key];
			else inputs[key] = next;
			return { ...s, inputs };
		});
	}

	/** The edits the form holds, as the helper takes them, or why it cannot be sent yet. An entry is edited when its value is filled in. */
	editsOf(s: DeriveState): { edits: Array<{ key: string; value: unknown; source: string }> } | { error: string } {
		const edits = [];
		for (const entry of s.entries ?? []) {
			const input = s.inputs[entry.key];
			if (!input || input.value.trim() === '') {
				if (input && input.source.trim() !== '') return { error: `${entry.key}: a source text without a new value` };
				continue;
			}
			const parsed = parseValueInput(input.value, entry);
			if (!parsed.ok) return { error: `${entry.key}: ${parsed.error}` };
			if (input.source.trim() === '') return { error: `${entry.key}: the new value needs its source or rationale text` };
			edits.push({ key: entry.key, value: parsed.value, source: input.source });
		}
		return { edits };
	}

	/** Why the Derive button is off, or null. A derivation with no edit is allowed: it reproduces the card as it is. */
	whyNoDerive(s: DeriveState, helperOpen: boolean): string | null {
		if (!helperOpen) return 'the helper is not connected';
		if (s.busy) return 'a command is in progress';
		if (s.current?.running) return 'a derivation is running: one at a time';
		if (s.card === '' || s.entries === null) return 'pick a card';
		const edits = this.editsOf(s);
		return 'error' in edits ? edits.error : null;
	}

	/** The explicit click. The helper lints the copy first and answers with its findings, or answers at once and runs the derivation. */
	async derive(): Promise<void> {
		const s = get(this.store);
		const edits = this.editsOf(s);
		if (s.busy || 'error' in edits) return;
		this.patch({ busy: 'derive', findings: [], notices: [], reportName: null, reportText: null, validation: EMPTY_VALIDATION, staging: EMPTY_STAGING });
		try {
			const reply = await this.deps.request('derive', { card: s.card, edits: edits.edits });
			const rec = recordOf(reply.derive);
			this.store.update((st) => ({ ...st, current: st.current?.running ? st.current : (rec ?? st.current) }));
		} catch (e) {
			const findings = e instanceof HelperError ? findingsOf(e.reply.findings) : [];
			const rec = e instanceof HelperError ? recordOf(e.reply.derive) : null;
			this.patch({ findings, notices: findings.length > 0 ? [messageOf(e)] : reasonsOf(e), current: rec ?? get(this.store).current });
		} finally {
			this.patch({ busy: null });
		}
	}

	async cancel(): Promise<void> {
		if (get(this.store).busy) return;
		this.patch({ busy: 'cancel', notices: [] });
		try {
			await this.deps.request('derive_cancel');
		} catch (e) {
			this.patch({ notices: reasonsOf(e) });
		} finally {
			this.patch({ busy: null });
		}
	}

	/** Reads one derivation report (a text file of the derivation's work directory) from the helper. */
	async openReport(name: string): Promise<void> {
		const s = get(this.store);
		if (s.busy || !s.current) return;
		this.patch({ busy: 'report', reportName: name, reportText: null, notices: [] });
		try {
			const reply = await this.deps.request('derive_report', { derive_id: s.current.id, name });
			this.patch({ reportText: typeof reply.text === 'string' ? reply.text : '' });
		} catch (e) {
			this.patch({ notices: reasonsOf(e) });
		} finally {
			this.patch({ busy: null });
		}
	}

	setImage(hash: string): void {
		this.patch({ image: hash, validation: EMPTY_VALIDATION });
	}

	whyNoValidate(s: DeriveState, helperOpen: boolean): string | null {
		if (!helperOpen) return 'the helper is not connected';
		if (s.busy) return 'a command is in progress';
		if (!s.current?.ok || !s.current.config_hash) return 'no derived config';
		if (s.current.images.length === 0) return 'no image of the library has this config\'s parameter schema: build one (Build panel) and derive again';
		if (s.image === '') return 'pick an image';
		return null;
	}

	/** G7 for the derived config: the helper's validator for (image, config); the report is fetched and its id checked here. */
	async validate(): Promise<void> {
		const s = get(this.store);
		const cur = s.current;
		if (s.busy || !cur?.ok || !cur.config_hash || s.image === '') return;
		const imageHash = s.image;
		const configHash = cur.config_hash;
		this.patch({ busy: 'validate', validation: { phase: 'validating', progress: [], record: null, error: null } });
		const setValidation = (p: Partial<DeriveValidation>) => this.store.update((st) => ({ ...st, validation: { ...st.validation, ...p } }));
		let finished: (e: HelperEvent) => void = () => undefined;
		const done = new Promise<HelperEvent>((resolve) => (finished = resolve));
		const off = this.deps.onEvent((e) => {
			if (e.image_hash !== imageHash || e.config_hash !== configHash) return;
			if (e.event === 'validate_progress') this.store.update((st) => ({ ...st, validation: { ...st.validation, progress: [...st.validation.progress, String(e.line)] } }));
			else if (e.event === 'validate_done') finished(e);
		});
		try {
			await this.deps.request('validate', { image_hash: imageHash, config: configHash });
			const end = await done;
			if (typeof end.report_id !== 'string' || typeof end.verdict !== 'string') throw new Error('the validator produced no report (see the progress lines)');
			const bytes = await this.deps.report(imageHash, configHash);
			const record = await makeRecord(imageHash, configHash, end.report_id, bytes, '');
			if (record.report === null) throw new Error(`the report cannot be read: ${record.parseError}`);
			if (record.reportId !== record.computedId) throw new Error('the report id announced by the helper is not the SHA-256 of the report');
			setValidation({ phase: 'done', record });
		} catch (e) {
			setValidation({ phase: 'failed', error: messageOf(e) });
		} finally {
			off();
			this.patch({ busy: null });
		}
	}

	whyNoStage(s: DeriveState): string | null {
		if (s.busy) return 'a command is in progress';
		if (!s.current?.ok || !s.current.config_hash) return 'no derived config';
		const vehicle = this.deps.vehicle;
		if (!vehicle) return 'no vehicle link';
		const why = vehicle.whyCannotStage();
		if (why !== null) return why;
		const theirs = vehicle.schemaHash();
		if (theirs === null) return 'the vehicle has not reported its identity';
		const mine = s.current.schema_hash ?? '';
		if (mine.toLowerCase() !== theirs.toLowerCase()) {
			return `the derived config's parameter schema ${mine || 'unknown'} is not the connected vehicle's ${theirs}: its values would not match the vehicle's parameters`;
		}
		return null;
	}

	/**
	 * Hands the values to the flow the Parameters page owns: every unlocked parameter of the derived config whose value differs from the one staged on
	 * the vehicle is staged with its own source text (the vehicle refuses what it must). Nothing is validated or activated here: the Parameters page
	 * does that on the staged config, with the validator and the report gate it always uses.
	 */
	async stage(): Promise<void> {
		const s = get(this.store);
		const cur = s.current;
		const vehicle = this.deps.vehicle;
		if (s.busy || !cur?.ok || !vehicle) return;
		const refused = this.whyNoStage(s);
		if (refused !== null) {
			this.patch({ staging: { phase: 'failed', staged: [], same: 0, notes: [refused] } });
			return;
		}
		this.patch({ busy: 'stage', staging: { phase: 'staging', staged: [], same: 0, notes: [] } });
		const set = (p: Partial<DeriveStaging>) => this.store.update((st) => ({ ...st, staging: { ...st.staging, ...p } }));
		try {
			const reply = await this.deps.request('derive_values', { derive_id: cur.id });
			const params = Array.isArray(reply.params) ? (reply.params as Array<{ name: string; link_id: string; type: ParamType; bits: number; locked: boolean; source: string }>) : [];
			const byLink = new Map(vehicle.params().filter((p) => p.linkId !== null).map((p) => [p.linkId as string, p]));
			const staged: string[] = [];
			const notes: string[] = [];
			let same = 0;
			for (const p of params) {
				const v = byLink.get(p.link_id);
				if (!v) {
					notes.push(`${p.name}: the vehicle does not list it`);
					continue;
				}
				if (v.stagedBits !== null && bytesToHex(v.stagedBits) === bytesToHex(p.bits)) {
					same += 1;
					continue;
				}
				if (p.locked || v.locked) {
					notes.push(`${p.name}: locked, not staged`);
					continue;
				}
				const result = await vehicle.stage(p.link_id, p.type, p.bits, p.source);
				if (result.accepted) staged.push(p.name);
				else notes.push(`${p.name}: refused by the vehicle${result.reasons.length > 0 ? ` (${result.reasons.join('; ')})` : ''}`);
				set({ staged: [...staged], notes: [...notes] });
			}
			set({ phase: notes.length > 0 && staged.length === 0 && same === 0 ? 'failed' : 'done', staged, same, notes });
		} catch (e) {
			set({ phase: 'failed', notes: [messageOf(e)] });
		} finally {
			this.patch({ busy: null });
		}
	}
}
