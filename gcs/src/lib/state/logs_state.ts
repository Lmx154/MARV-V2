// The Logs page's logic (GCS-first unit GX-15): the onboard logger's .mlog files pulled from the SD card. The helper (tools/gs/logs.py) copies a
// file into its library with its SHA-256, reads it with logger U1's decoder (tools/log/mlog.py) and answers a decimated min/max series of
// chosen fields; the browser decodes nothing and never receives the records. This controller sends the helper's commands (log_list, log_import,
// log_series) and keeps what the page shows; every refusal is shown as the helper words it. An import runs in the helper (it answers at once and
// pushes log_import_progress and log_import_done), so a file of a GiB is a progress bar, not a request that times out. IF-7 (log retrieval over
// the link) is not here.
import { get, writable, type Readable } from 'svelte/store';
import type { HelperEvent, HelperReply } from '$link/transport';

/** One problem the decoder reports (a sector that failed its check, a hole, a record cut by one). */
export interface LogProblem {
	sector: number;
	reason: string;
	detail: string;
}

export interface KindInfo {
	count: number;
	t_min_us: number | null;
	t_max_us: number | null;
	fields: string[];
}

/** The decoder's report of a file as the helper structures it (logs.py summarise). */
export interface LogSummary {
	sectors_ok: number;
	sectors_total: number;
	session: string | null;
	ended: boolean;
	ends_inside_record: boolean;
	stopped_at: number | null;
	clean: boolean;
	problems: LogProblem[];
	records: Record<string, number>;
	kinds: Record<string, KindInfo>;
	gaps: { count: number; dropped: Record<string, number>; shown: Array<Record<string, number>> };
	end: Record<string, unknown> | null;
	header: { image_hash: string | null; config_hash: string | null; schema_hash: string | null; odr_hz: number | null; params: number };
	/** Set instead of the rest when the decoder raised on the file. */
	decoder_error?: string;
}

export interface LogEntry {
	sha256: string;
	size: number;
	name: string;
	source: string;
	imported_utc: string;
	summary: LogSummary;
	/** Only on the answer of an import: the file was in the library already. */
	already_imported?: boolean;
}

/** A .mlog on a mounted card (<mount>/marv/log). */
export interface CardFile {
	path: string;
	name: string;
	mount: string;
	size: number;
}

export interface SeriesField {
	min: Array<number | null>;
	max: Array<number | null>;
	skipped: number;
}

export interface SeriesView {
	sha256: string;
	kind: string;
	t0_us: number;
	t1_us: number;
	bucket_us: number;
	buckets: number;
	records: number;
	n: number[];
	fields: Record<string, SeriesField>;
}

/** PLACEHOLDER (harness value): the buckets a plot asks for; about the pixel width of a plot. The helper's own limits are in tools/gs/logs.py. */
export const PLOT_BUCKETS = 600;

const US_PER_S = 1_000_000;
const KIB = 1024;
const SHORT_SHA = 12;
const SIGNIFICANT = 7;
const BYTE_UNITS = ['B', 'KiB', 'MiB', 'GiB', 'TiB'];
/** The kind a log is opened on and the field plotted first when they exist (the IMU's roll gyro). */
const PREFERRED_KIND = 'imu';
const PREFERRED_FIELD = 'gyro_x';
/** Half the span drawn for a field that is constant, so that a flat line is a line and not a division by zero. */
const FLAT_HALF_SPAN = 1;

export const shortSha = (sha: string): string => sha.slice(0, SHORT_SHA);

export function formatBytes(n: number): string {
	let v = n;
	let i = 0;
	while (v >= KIB && i < BYTE_UNITS.length - 1) {
		v /= KIB;
		i += 1;
	}
	return i === 0 ? `${n} B` : `${v.toFixed(1)} ${BYTE_UNITS[i]}`;
}

/** A value as the plot's readouts show it: seven significant digits, no trailing zeros. */
export function formatValue(v: number | null): string {
	return v === null ? '—' : String(Number(v.toPrecision(SIGNIFICANT)));
}

/** Seconds from `originUs`, three decimals. */
export function formatSeconds(us: number, originUs: number): string {
	return ((us - originUs) / US_PER_S).toFixed(3);
}

/** The fields of a kind that can be plotted: the time columns are the axis, not a curve. */
export function plottableFields(info: KindInfo | undefined): string[] {
	return (info?.fields ?? []).filter((f) => f !== 't_us' && !f.endsWith('_lo_us'));
}

/** The kinds that can be plotted: with a time axis and at least one field. */
export function plottableKinds(summary: LogSummary): string[] {
	return Object.entries(summary.kinds)
		.filter(([, k]) => k.t_min_us !== null && plottableFields(k).length > 0)
		.map(([name]) => name);
}

export interface ReportRow {
	key: string;
	name: string;
	value: string;
	tone: 'ok' | 'warn' | 'bad' | 'neutral';
}

/** The decoder's report as rows: what was read, what was not, and why. */
export function reportRows(s: LogSummary): ReportRow[] {
	if (s.decoder_error) return [{ key: 'decoder', name: 'decoder', value: `failed: ${s.decoder_error}`, tone: 'bad' }];
	const rows: ReportRow[] = [
		{ key: 'sectors', name: 'sectors', value: `${s.sectors_ok} valid of ${s.sectors_total}`, tone: s.sectors_ok === s.sectors_total ? 'ok' : 'warn' },
		{ key: 'session', name: 'session', value: s.session ?? 'none', tone: 'neutral' },
		{ key: 'records', name: 'records', value: Object.entries(s.records).sort(([a], [b]) => a.localeCompare(b)).map(([k, n]) => `${k} ${n}`).join(', ') || 'none', tone: 'neutral' },
		{ key: 'end', name: 'end record', value: s.ended ? 'yes' : 'NO: the log is cut short', tone: s.ended ? 'ok' : 'warn' }
	];
	if (s.ends_inside_record) rows.push({ key: 'partial', name: 'last sector', value: 'ends inside a record (the partial record is not returned)', tone: 'warn' });
	const dropped = Object.entries(s.gaps.dropped).map(([k, n]) => `${k} ${n}`).join(', ');
	rows.push({ key: 'gaps', name: 'gap records', value: s.gaps.count === 0 ? 'none' : `${s.gaps.count}, dropped ${dropped}`, tone: s.gaps.count === 0 ? 'ok' : 'warn' });
	rows.push({
		key: 'corrupt',
		name: 'corrupt sectors',
		value: s.problems.length === 0 ? 'none' : `${s.problems.length}: ${s.problems.map((p) => `sector ${p.sector} ${p.reason}${p.detail ? ` (${p.detail})` : ''}`).join('; ')}`,
		tone: s.problems.length === 0 ? 'ok' : 'bad'
	});
	if (s.stopped_at !== null) rows.push({ key: 'stopped', name: 'decoding stopped', value: `at sector ${s.stopped_at}: what follows is not trusted`, tone: 'bad' });
	return rows;
}

export interface TimeRange {
	t0_us: number | null;
	t1_us: number | null;
}

/** "from" and "to" in seconds from the start of the kind (blank: the whole range) as the helper's microseconds; an error text if they cannot be. */
export function parseRange(fromText: string, toText: string, originUs: number): TimeRange | { error: string } {
	const one = (text: string): number | null | 'bad' => {
		const t = text.trim();
		if (t === '') return null;
		if (!/^-?(\d+\.?\d*|\.\d+)$/.test(t)) return 'bad';
		return Math.round(originUs + Number(t) * US_PER_S);
	};
	const a = one(fromText);
	const b = one(toText);
	if (a === 'bad' || b === 'bad') return { error: 'from and to are seconds from the start of the log (a number, or blank for the whole range)' };
	if (a !== null && b !== null && b < a) return { error: 'to is before from' };
	return { t0_us: a, t1_us: b };
}

export interface PlotModel {
	/** The SVG path: one vertical stroke from the bucket's minimum to its maximum, for every bucket that holds a value. */
	path: string;
	yMin: number;
	yMax: number;
	dataMin: number | null;
	dataMax: number | null;
	filled: number;
}

/** The min/max band of one field in a width x height box (y up on the screen is larger). */
export function plotModel(min: Array<number | null>, max: Array<number | null>, width: number, height: number): PlotModel {
	let lo = Infinity;
	let hi = -Infinity;
	for (let i = 0; i < min.length; i += 1) {
		const a = min[i];
		const b = max[i];
		if (a === null || a === undefined || b === null || b === undefined) continue;
		lo = Math.min(lo, a);
		hi = Math.max(hi, b);
	}
	if (lo > hi) return { path: '', yMin: 0, yMax: 0, dataMin: null, dataMax: null, filled: 0 };
	const yMin = lo === hi ? lo - FLAT_HALF_SPAN : lo;
	const yMax = lo === hi ? hi + FLAT_HALF_SPAN : hi;
	const y = (v: number): number => height - ((v - yMin) / (yMax - yMin)) * height;
	const x = (i: number): number => ((i + 1 / 2) / min.length) * width;
	let path = '';
	let filled = 0;
	for (let i = 0; i < min.length; i += 1) {
		const a = min[i];
		const b = max[i];
		if (a === null || a === undefined || b === null || b === undefined) continue;
		filled += 1;
		const px = x(i).toFixed(2);
		path += `M${px} ${y(a).toFixed(2)}V${y(b).toFixed(2)}`;
	}
	return { path, yMin, yMax, dataMin: lo, dataMax: hi, filled };
}

/** The import that is going: what the helper's progress events say (bytesTotal is 0 until the first one arrives). */
export interface ImportProgress {
	id: number | null;
	name: string;
	phase: 'copy' | 'decode';
	bytesDone: number;
	bytesTotal: number;
}

export interface LogsState {
	/** The library; null until read. */
	logs: LogEntry[] | null;
	cards: CardFile[];
	library: string | null;
	selected: string | null;
	kind: string | null;
	fields: string[];
	from: string;
	to: string;
	series: SeriesView | null;
	/** The helper's refusal of the last command, verbatim. */
	error: string | null;
	/** What the last import did (the sha, and whether the decoder had problems). */
	notice: string | null;
	/** The import that is going, else null: no other import is started while it is set. */
	importing: ImportProgress | null;
	busy: 'list' | 'series' | null;
}

export interface LogsDeps {
	request(cmd: string, args?: Record<string, unknown>): Promise<HelperReply>;
	onEvent(handler: (event: HelperEvent) => void): () => void;
}

const isObject = (v: unknown): v is Record<string, unknown> => typeof v === 'object' && v !== null && !Array.isArray(v);

/** An entry of the helper's library, or null for something that is not one. */
export function entryOf(v: unknown): LogEntry | null {
	if (!isObject(v) || typeof v.sha256 !== 'string' || typeof v.name !== 'string' || typeof v.size !== 'number' || !isObject(v.summary)) return null;
	return v as unknown as LogEntry;
}

function cardOf(v: unknown): CardFile | null {
	if (!isObject(v) || typeof v.path !== 'string' || typeof v.name !== 'string' || typeof v.size !== 'number') return null;
	return { path: v.path, name: v.name, mount: typeof v.mount === 'string' ? v.mount : '', size: v.size };
}

const messageOf = (e: unknown): string => (e instanceof Error ? e.message : String(e));

export class LogsController {
	private readonly store = writable<LogsState>({
		logs: null, cards: [], library: null, selected: null, kind: null, fields: [], from: '', to: '', series: null, error: null, notice: null, importing: null, busy: null
	});
	readonly state: Readable<LogsState> = { subscribe: this.store.subscribe };

	private off: (() => void) | null = null;
	/** The imports whose done event has been seen: the answer of log_import may arrive after the event of a small file. */
	private readonly finished = new Set<number>();

	constructor(private readonly deps: LogsDeps) {}

	/** Starts following the helper's import events; returns what stops it. A second call does nothing. */
	start(): () => void {
		if (!this.off) this.off = this.deps.onEvent((e) => this.onEvent(e));
		return () => {
			this.off?.();
			this.off = null;
		};
	}

	private onEvent(e: HelperEvent): void {
		const id = typeof e.id === 'number' ? e.id : null;
		if (e.event === 'log_import_progress') {
			if (id !== null && this.finished.has(id)) return;
			const phase = e.phase === 'decode' ? 'decode' : 'copy';
			const num = (v: unknown): number => (typeof v === 'number' && Number.isFinite(v) ? v : 0);
			this.store.update((s) => ({ ...s, importing: { id, name: s.importing?.name ?? '', phase, bytesDone: num(e.bytes_done), bytesTotal: num(e.bytes_total) } }));
		} else if (e.event === 'log_import_done') {
			if (id !== null) this.finished.add(id);
			void this.importDone(e);
		}
	}

	private async importDone(e: HelperEvent): Promise<void> {
		if (e.ok !== true) {
			this.store.update((s) => ({ ...s, importing: null, notice: null, error: typeof e.error === 'string' ? e.error : 'the import failed' }));
			return;
		}
		const entry = entryOf(e);
		const problems = entry?.summary.problems.length ?? 0;
		const notice = entry
			? `${entry.name}: ${shortSha(entry.sha256)}${e.already_imported === true ? ' (already in the library)' : ''}${
					entry.summary.decoder_error ? `; the decoder failed: ${entry.summary.decoder_error}` : problems > 0 ? `; the decoder reports ${problems} problem${problems === 1 ? '' : 's'}` : ''
				}`
			: 'imported';
		this.store.update((s) => ({ ...s, importing: null, error: null, notice }));
		try {
			await this.read();
			if (entry) this.select(entry.sha256);
		} catch (err) {
			this.store.update((s) => ({ ...s, error: messageOf(err) }));
		}
	}

	/** The library and the cards the helper sees. */
	async load(): Promise<void> {
		if (get(this.store).busy) return;
		this.store.update((s) => ({ ...s, busy: 'list' }));
		try {
			await this.read();
			this.store.update((s) => ({ ...s, error: null, busy: null }));
		} catch (e) {
			this.store.update((s) => ({ ...s, error: messageOf(e), busy: null }));
		}
	}

	private async read(): Promise<void> {
		const reply = await this.deps.request('log_list');
		const logs = (Array.isArray(reply.logs) ? reply.logs : []).map(entryOf).filter((e): e is LogEntry => e !== null);
		const cards = (Array.isArray(reply.cards) ? reply.cards : []).map(cardOf).filter((c): c is CardFile => c !== null);
		const going = isObject(reply.importing) && typeof reply.importing.id === 'number' ? reply.importing : null;
		this.store.update((s) => ({
			...s,
			logs,
			cards,
			library: typeof reply.library === 'string' ? reply.library : null,
			importing: s.importing ?? (going && !this.finished.has(going.id as number) ? { id: going.id as number, name: String(going.name ?? ''), phase: 'copy', bytesDone: 0, bytesTotal: 0 } : null)
		}));
	}

	/** Asks the helper to copy a file of the host into the library. The helper answers at once; the progress and the end come as events. A file the
	 *  decoder has problems with is imported all the same; the notice says so. */
	async importPath(path: string): Promise<void> {
		const p = path.trim();
		if (get(this.store).importing) return;
		if (p === '') {
			this.store.update((s) => ({ ...s, error: 'give the path of an .mlog file (absolute), or import one of the card’s files' }));
			return;
		}
		this.store.update((s) => ({ ...s, notice: null, error: null }));
		try {
			const reply = await this.deps.request('log_import', { path: p });
			const id = typeof reply.import_id === 'number' ? reply.import_id : null;
			if (id !== null && this.finished.has(id)) return; // a small file: its done event came first
			const name = p.slice(p.lastIndexOf('/') + 1);
			this.store.update((s) => ({ ...s, importing: { id, name, phase: s.importing?.phase ?? 'copy', bytesDone: s.importing?.bytesDone ?? 0, bytesTotal: s.importing?.bytesTotal ?? 0 } }));
		} catch (e) {
			this.store.update((s) => ({ ...s, error: messageOf(e) }));
		}
	}

	/** Chooses a log of the library: the plot is reset to its IMU gyro if it has one. */
	select(sha: string): void {
		const entry = get(this.store).logs?.find((l) => l.sha256 === sha) ?? null;
		if (!entry) return;
		const kinds = plottableKinds(entry.summary);
		const kind = kinds.includes(PREFERRED_KIND) ? PREFERRED_KIND : (kinds[0] ?? null);
		this.store.update((s) => ({ ...s, selected: sha, series: null, error: null, from: '', to: '', ...this.defaults(entry, kind) }));
	}

	private defaults(entry: LogEntry, kind: string | null): { kind: string | null; fields: string[] } {
		const fields = plottableFields(kind ? entry.summary.kinds[kind] : undefined);
		const first = fields.includes(PREFERRED_FIELD) ? PREFERRED_FIELD : fields[0];
		return { kind, fields: first === undefined ? [] : [first] };
	}

	setKind(kind: string): void {
		const s = get(this.store);
		const entry = s.logs?.find((l) => l.sha256 === s.selected);
		if (!entry || !(kind in entry.summary.kinds)) return;
		this.store.update((st) => ({ ...st, series: null, from: '', to: '', ...this.defaults(entry, kind) }));
	}

	toggleField(field: string, on: boolean): void {
		this.store.update((s) => ({ ...s, fields: on ? (s.fields.includes(field) ? s.fields : [...s.fields, field]) : s.fields.filter((f) => f !== field) }));
	}

	setRange(from: string, to: string): void {
		this.store.update((s) => ({ ...s, from, to }));
	}

	/** Asks the helper for the min/max series of the chosen fields over the range. */
	async plot(): Promise<void> {
		const s = get(this.store);
		const entry = s.logs?.find((l) => l.sha256 === s.selected);
		const info = entry && s.kind ? entry.summary.kinds[s.kind] : undefined;
		if (s.busy || !entry || !s.kind || !info || info.t_min_us === null) return;
		if (s.fields.length === 0) {
			this.store.update((st) => ({ ...st, error: 'choose at least one field to plot' }));
			return;
		}
		const range = parseRange(s.from, s.to, info.t_min_us);
		if ('error' in range) {
			this.store.update((st) => ({ ...st, error: range.error }));
			return;
		}
		this.store.update((st) => ({ ...st, busy: 'series', error: null }));
		try {
			const args: Record<string, unknown> = { sha256: entry.sha256, kind: s.kind, fields: s.fields, buckets: PLOT_BUCKETS };
			if (range.t0_us !== null) args.t0_us = range.t0_us;
			if (range.t1_us !== null) args.t1_us = range.t1_us;
			const reply = await this.deps.request('log_series', args);
			const series = reply.series as SeriesView | undefined;
			if (!series || !isObject(series.fields)) throw new Error('the helper’s answer to log_series is not a series');
			this.store.update((st) => ({ ...st, series, busy: null }));
		} catch (e) {
			this.store.update((st) => ({ ...st, error: messageOf(e), busy: null }));
		}
	}
}
