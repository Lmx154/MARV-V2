/**
 * The staged setup as the page knows it, and the rules of the setup protocol (gcs decision (b), (c)):
 * set_param is answered by the value the FC HOLDS; a save landed iff stored_crc == staged_crc afterwards.
 */
import type { FactorySetup, KindDef, ParamSpec, Schema, SetupHeader } from './types';

export interface Rejected {
	requested: number;
	held: number;
	/** No echo arrived in time. */
	timeout?: boolean;
}

export interface SetupState {
	header: SetupHeader | null;
	/** Staged values as the FC last reported them, by absolute index. */
	values: number[];
	/** Index -> value sent, awaiting its echo (one outstanding request per index). */
	pending: Record<number, number>;
	/** Index -> latest edit made while a request was outstanding; sent when the echo arrives. */
	queued: Record<number, number>;
	/** Index -> the last edit the FC did not take. */
	rejected: Record<number, Rejected>;
	save: 'idle' | 'pending' | 'landed' | 'refused';
}

export function emptySetup(): SetupState {
	return { header: null, values: [], pending: {}, queued: {}, rejected: {}, save: 'idle' };
}

export interface ParamRef {
	family: number;
	kind: number;
	key: string;
	spec: ParamSpec;
}

export function paramCount(schema: Schema): number {
	let n = 0;
	for (const f of schema.families) for (const k of f.kinds) n += k.params.length;
	return n;
}

/** "family.kind.id" -> param, the export/import key. */
export function paramKeys(schema: Schema): Map<string, ParamRef> {
	const out = new Map<string, ParamRef>();
	schema.families.forEach((f, fi) =>
		f.kinds.forEach((k, ki) => k.params.forEach((spec) => out.set(`${f.id}.${k.id}.${spec.id}`, { family: fi, kind: ki, key: `${f.id}.${k.id}.${spec.id}`, spec })))
	);
	return out;
}

/** The FC's schema is this page's: same hash and parameter count. */
export function schemaMatches(schema: Schema, header: SetupHeader): boolean {
	return header.schema_hash >>> 0 === schema.schema_hash >>> 0 && header.param_count === paramCount(schema);
}

/** f32 equality: the FC holds float, the page edits in double. */
export const sameF32 = (a: number, b: number): boolean => Math.fround(a) === Math.fround(b);

/** The value to show for a param: the latest edit in flight, else what the FC holds. */
export function shown(st: SetupState, index: number): number {
	return st.queued[index] ?? st.pending[index] ?? st.values[index];
}

/** Record an edit. Returns the value to send now, or null when it waits for the outstanding echo. */
export function editParam(st: SetupState, index: number, value: number): number | null {
	delete st.rejected[index];
	if (index in st.pending) {
		st.queued[index] = value;
		return null;
	}
	st.pending[index] = value;
	return value;
}

/** The FC's kParamValue: the value it holds. Returns a queued edit to send next, or null. */
export function paramEcho(st: SetupState, index: number, held: number): number | null {
	if (index >= 0 && index < st.values.length) st.values[index] = held;
	const requested = st.pending[index];
	delete st.pending[index];
	const next = st.queued[index];
	delete st.queued[index];
	if (next !== undefined && !sameF32(next, held)) {
		st.pending[index] = next;
		return next;
	}
	if (requested !== undefined && next === undefined && !sameF32(requested, held)) st.rejected[index] = { requested, held };
	return null;
}

/** No echo in time: drop the request and mark it unconfirmed. */
export function expireParam(st: SetupState, index: number): void {
	const requested = st.queued[index] ?? st.pending[index];
	delete st.pending[index];
	delete st.queued[index];
	if (requested !== undefined) st.rejected[index] = { requested, held: st.values[index], timeout: true };
}

/** A save landed iff the stored setup is now the staged one. */
export const saveLanded = (h: SetupHeader): boolean => h.stored_crc >>> 0 === h.staged_crc >>> 0;

/** A kSetupHeader (with the staged values when the reply carries them). */
export function applyHeader(st: SetupState, header: SetupHeader, values: number[] | null): void {
	st.header = header;
	if (values) {
		st.values = values.slice();
		st.pending = {};
		st.queued = {};
		st.rejected = {};
	}
	if (st.save === 'pending') st.save = saveLanded(header) ? 'landed' : 'refused';
}

/** Where the page shows a backend error: next to the control whose request it answers. */
export type ErrorSlot = 'factory' | 'actions' | 'params' | 'link';

export function errorSlot(request: string): ErrorSlot {
	if (request === 'load_factory') return 'factory';
	if (request === 'save' || request === 'reset' || request === 'reboot' || request === 'flash') return 'actions';
	if (request === 'set_param' || request === 'set_kind') return 'params';
	return 'link';
}

/** A backend error for a request: the save it answers is no longer pending. Returns the slot to show it in. */
export function requestError(st: SetupState, request: string): ErrorSlot {
	if (request === 'save' && st.save === 'pending') st.save = 'idle';
	return errorSlot(request);
}

/** Plain-word state of running / staged / stored. */
export function statusWords(h: SetupHeader): { text: string; tone: 'ok' | 'warn' | 'bad' }[] {
	const out: { text: string; tone: 'ok' | 'warn' | 'bad' }[] = [];
	out.push(h.running_crc === h.staged_crc ? { text: 'running = staged', tone: 'ok' } : { text: 'staged differs from running (Apply runs it)', tone: 'warn' });
	if (!h.stored_valid) out.push({ text: 'no valid setup in flash (boots factory 0)', tone: 'bad' });
	else out.push(saveLanded(h) ? { text: 'saved to flash', tone: 'ok' } : { text: 'unsaved changes', tone: 'warn' });
	return out;
}

/** Index of the vehicle family, whose staged kind is the vehicle class every other family must serve. */
export function vehicleFamily(schema: Schema): number {
	return schema.families.findIndex((f) => f.id === 'vehicle');
}

/** The staged vehicle kind's id ('uav', 'rocket'), from the FC's kinds; undefined before a header. */
export function vehicleId(schema: Schema, kinds: number[]): string | undefined {
	const vf = vehicleFamily(schema);
	return vf < 0 ? undefined : schema.families[vf].kinds[kinds[vf]]?.id;
}

/**
 * The kinds a family's card offers: every vehicle class on the vehicle card; elsewhere only the kinds that serve the
 * staged vehicle, plus the kind the FC holds.
 */
export function kindOptions(schema: Schema, family: number, kinds: number[]): { kind: number; def: KindDef }[] {
	const all = schema.families[family]?.kinds.map((def, kind) => ({ kind, def })) ?? [];
	const vehicle = vehicleId(schema, kinds);
	if (family === vehicleFamily(schema) || vehicle === undefined) return all;
	return all.filter((o) => o.kind === kinds[family] || o.def.vehicles.includes(vehicle));
}

/** The card a backend error belongs beside: the family of a refused set_kind, else null. */
export function refusedFamily(m: { request: string; family?: number }): number | null {
	return m.request === 'set_kind' && m.family !== undefined ? m.family : null;
}

export function kindOf(schema: Schema, family: number, kind: number): KindDef | undefined {
	return schema.families[family]?.kinds[kind];
}

/** The values "modified" is measured against: the factory setup running this kind, else the kind's defaults. */
export function referenceValues(schema: Schema, family: number, kind: number): Record<string, number> {
	const def = kindOf(schema, family, kind);
	if (!def) return {};
	const f: FactorySetup | undefined = schema.factory.find((s) => s.kinds[family] === kind);
	return Object.fromEntries(def.params.map((p) => [p.id, f?.values[p.index] ?? p.default]));
}

/** The factory setup the staged kinds and values equal, else null. */
export function matchingFactory(schema: Schema, kinds: number[], values: number[]): FactorySetup | null {
	return (
		schema.factory.find((f) => f.kinds.every((k, i) => k === kinds[i]) && f.values.length === values.length && f.values.every((v, i) => sameF32(v, values[i]))) ??
		null
	);
}

export interface SetupFile {
	format: 'marv-setup';
	schema_hash: number;
	kinds: Record<string, string>;
	values: Record<string, number>;
}

/** The staged setup as JSON keyed family.kind.id (every kind's values: the Setup holds them all). */
export function exportSetup(schema: Schema, kinds: number[], values: number[]): SetupFile {
	const out: SetupFile = { format: 'marv-setup', schema_hash: schema.schema_hash, kinds: {}, values: {} };
	schema.families.forEach((f, fi) => {
		const k = f.kinds[kinds[fi]];
		if (k) out.kinds[f.id] = k.id;
	});
	for (const [key, ref] of paramKeys(schema)) out.values[key] = values[ref.spec.index];
	return out;
}

export interface ImportPlan {
	kinds: { family: number; kind: number }[];
	params: { index: number; value: number }[];
	/** Keys this schema does not have: reported, not applied. */
	unknown: string[];
	/** Known keys with a value the FC would refuse (not finite, out of range): reported, not applied. */
	invalid: string[];
}

/** Map an exported file onto this schema by id; indices are never trusted across schemas. */
export function planImport(schema: Schema, file: unknown): ImportPlan {
	const plan: ImportPlan = { kinds: [], params: [], unknown: [], invalid: [] };
	const f = (typeof file === 'object' && file !== null ? file : {}) as Partial<SetupFile>;
	const kinds = typeof f.kinds === 'object' && f.kinds !== null ? f.kinds : {};
	for (const [fam, kid] of Object.entries(kinds)) {
		const fi = schema.families.findIndex((x) => x.id === fam);
		const ki = fi < 0 ? -1 : schema.families[fi].kinds.findIndex((k) => k.id === kid);
		if (ki < 0) plan.unknown.push(`${fam}=${String(kid)}`);
		else plan.kinds.push({ family: fi, kind: ki });
	}
	const keys = paramKeys(schema);
	const values = typeof f.values === 'object' && f.values !== null ? f.values : {};
	for (const [key, v] of Object.entries(values)) {
		const ref = keys.get(key);
		if (!ref) plan.unknown.push(key);
		else if (typeof v !== 'number' || !Number.isFinite(v) || v < ref.spec.min || v > ref.spec.max) plan.invalid.push(key);
		else plan.params.push({ index: ref.spec.index, value: v });
	}
	return plan;
}
