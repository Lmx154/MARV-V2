// The Logs page's logic (GCS-first unit GX-15): the decoder's report as rows (a corrupt sector is shown with its sector and reason), the
// time range, the min/max band of a plot, and the controller's requests to the helper (the refusals are the helper's own text).
import { describe, expect, it } from 'vitest';
import { get } from 'svelte/store';
import { HelperError, type HelperEvent, type HelperReply } from '$link/transport';
import {
	LogsController,
	PLOT_BUCKETS,
	formatBytes,
	formatSeconds,
	formatValue,
	parseRange,
	plotModel,
	plottableFields,
	plottableKinds,
	reportRows,
	shortSha,
	type KindInfo,
	type LogEntry,
	type LogSummary
} from './logs_state';

const SHA = 'a'.repeat(64);
const SHA2 = 'b'.repeat(64);
const T0 = 1000;
const T1 = 80667;

const summary = (over: Partial<LogSummary> = {}): LogSummary => ({
	sectors_ok: 41,
	sectors_total: 41,
	session: '0x1122334455667788',
	ended: true,
	ends_inside_record: false,
	stopped_at: null,
	clean: true,
	problems: [],
	records: { imu: 600, status: 2, end: 1 },
	kinds: {
		end: { count: 1, t_min_us: 80800, t_max_us: 80800, fields: ['t_us', 'sectors'] },
		imu: { count: 600, t_min_us: T0, t_max_us: T1, fields: ['flags', 't_lo_us', 'gyro_x', 'gyro_y', 't_us'] },
		status: { count: 2, t_min_us: T0, t_max_us: 80800, fields: ['t_us', 'drops_imu'] },
		manual_control: { count: 3, t_min_us: null, t_max_us: null, fields: ['roll'] }
	},
	gaps: { count: 0, dropped: {}, shown: [] },
	end: null,
	header: { image_hash: null, config_hash: null, schema_hash: null, odr_hz: 7500.3, params: 1 },
	...over
});

const entry = (over: Partial<LogEntry> = {}): LogEntry => ({
	sha256: SHA, size: 20992, name: '00000001.mlog', source: '/media/x/marv/log/00000001.mlog', imported_utc: '2026-10-19T00:00:00Z', summary: summary(), ...over
});

class Fake {
	calls: Array<{ cmd: string; args: Record<string, unknown> | undefined }> = [];
	answers = new Map<string, HelperReply | Error>();
	handlers = new Set<(e: HelperEvent) => void>();
	controller = new LogsController({
		request: async (cmd, args) => {
			this.calls.push({ cmd, args });
			const a = this.answers.get(cmd) ?? { ok: true };
			if (a instanceof Error) throw a;
			return a;
		},
		onEvent: (h) => {
			this.handlers.add(h);
			return () => this.handlers.delete(h);
		}
	});
	push(e: HelperEvent) {
		this.handlers.forEach((h) => h(e));
	}
	/** Lets the controller's own follow-up reads (after a done event) finish. */
	settle() {
		return new Promise((r) => setTimeout(r, 0));
	}
	library(...logs: LogEntry[]) {
		this.answers.set('log_list', { ok: true, logs, cards: [{ path: '/m/MARVSD/marv/log/00000001.mlog', name: '00000001.mlog', mount: '/m/MARVSD', size: 20992 }], library: '/run/logs' });
	}
	refuse(cmd: string, error: string) {
		this.answers.set(cmd, new HelperError({ ok: false, error }, cmd));
	}
	get state() {
		return get(this.controller.state);
	}
	sent(cmd: string) {
		return this.calls.filter((c) => c.cmd === cmd);
	}
}

describe('formats', () => {
	it('formats bytes, values and seconds', () => {
		expect(formatBytes(512)).toBe('512 B');
		expect(formatBytes(20992)).toBe('20.5 KiB');
		expect(formatBytes(1024 ** 3)).toBe('1.0 GiB');
		expect(formatValue(40333.5)).toBe('40333.5');
		expect(formatValue(500)).toBe('500');
		expect(formatValue(0.1 + 0.2)).toBe('0.3');
		expect(formatValue(null)).toBe('—');
		expect(formatSeconds(2_500_000, 1_000_000)).toBe('1.500');
		expect(shortSha(SHA)).toBe('aaaaaaaaaaaa');
	});
});

describe('what can be plotted', () => {
	it('leaves the time columns out and the kinds without a time or a field', () => {
		const s = summary();
		expect(plottableFields(s.kinds.imu)).toEqual(['flags', 'gyro_x', 'gyro_y']);
		expect(plottableFields(undefined)).toEqual([]);
		expect(plottableKinds(s)).toEqual(['end', 'imu', 'status']); // manual_control has no time axis
	});
});

describe('reportRows', () => {
	const row = (rows: ReturnType<typeof reportRows>, key: string) => rows.find((r) => r.key === key);

	it('shows a clean log as clean', () => {
		const rows = reportRows(summary());
		expect(row(rows, 'sectors')).toMatchObject({ value: '41 valid of 41', tone: 'ok' });
		expect(row(rows, 'records')?.value).toBe('end 1, imu 600, status 2');
		expect(row(rows, 'end')).toMatchObject({ value: 'yes', tone: 'ok' });
		expect(row(rows, 'gaps')).toMatchObject({ value: 'none', tone: 'ok' });
		expect(row(rows, 'corrupt')).toMatchObject({ value: 'none', tone: 'ok' });
		expect(row(rows, 'stopped')).toBeUndefined();
	});

	it('shows a corrupt sector with its sector number and reason, and where decoding stopped', () => {
		const rows = reportRows(
			summary({ sectors_ok: 5, clean: false, ended: false, stopped_at: 5, problems: [{ sector: 5, reason: 'crc', detail: '' }] })
		);
		expect(row(rows, 'corrupt')).toMatchObject({ value: '1: sector 5 crc', tone: 'bad' });
		expect(row(rows, 'stopped')?.value).toBe('at sector 5: what follows is not trusted');
		expect(row(rows, 'sectors')).toMatchObject({ value: '5 valid of 41', tone: 'warn' });
		expect(row(rows, 'end')).toMatchObject({ value: 'NO: the log is cut short', tone: 'warn' });
	});

	it('shows a hole with its detail, the gap records, a partial last record and a decoder failure', () => {
		const rows = reportRows(
			summary({
				ends_inside_record: true,
				problems: [{ sector: 9, reason: 'hole', detail: 'sequence 10, expected 9' }],
				gaps: { count: 2, dropped: { imu: 4, rate_tick: 1 }, shown: [] }
			})
		);
		expect(row(rows, 'corrupt')?.value).toBe('1: sector 9 hole (sequence 10, expected 9)');
		expect(row(rows, 'gaps')?.value).toBe('2, dropped imu 4, rate_tick 1');
		expect(row(rows, 'partial')?.tone).toBe('warn');
		const failed = reportRows(summary({ decoder_error: 'ValueError: planted' }));
		expect(failed).toEqual([{ key: 'decoder', name: 'decoder', value: 'failed: ValueError: planted', tone: 'bad' }]);
	});
});

describe('parseRange', () => {
	it('reads blank as the whole range and seconds from the origin as microseconds', () => {
		expect(parseRange('', '  ', T0)).toEqual({ t0_us: null, t1_us: null });
		expect(parseRange('0.5', '2', T0)).toEqual({ t0_us: T0 + 500_000, t1_us: T0 + 2_000_000 });
		expect(parseRange('.25', '', T0)).toEqual({ t0_us: T0 + 250_000, t1_us: null });
	});
	it.each([['abc', ''], ['', '1e3'], ['1 2', ''], ['0x1', '']])('does not read %j to %j', (a, b) => {
		expect(parseRange(a, b, T0)).toHaveProperty('error');
	});
	it('refuses a range that ends before it starts', () => {
		expect(parseRange('2', '1', T0)).toEqual({ error: 'to is before from' });
	});
});

describe('plotModel', () => {
	it('draws one stroke per bucket that holds values, from its minimum to its maximum', () => {
		const m = plotModel([0, null, -2], [4, null, 2], 300, 100);
		expect(m).toMatchObject({ dataMin: -2, dataMax: 4, yMin: -2, yMax: 4, filled: 2 });
		// x = (i + 1/2) / 3 * 300; y = 100 - (v - yMin) / 6 * 100
		expect(m.path).toBe('M50.00 66.67V0.00M250.00 100.00V33.33');
	});
	it('gives a constant field a span so that it is a line', () => {
		const m = plotModel([5, 5], [5, 5], 100, 10);
		expect(m).toMatchObject({ yMin: 4, yMax: 6, dataMin: 5, dataMax: 5, filled: 2 });
		expect(m.path).toBe('M25.00 5.00V5.00M75.00 5.00V5.00');
	});
	it('draws nothing when every bucket is empty', () => {
		expect(plotModel([null, null], [null, null], 100, 10)).toEqual({ path: '', yMin: 0, yMax: 0, dataMin: null, dataMax: null, filled: 0 });
	});
});

describe('the library', () => {
	it('reads the helper’s library and cards', async () => {
		const f = new Fake();
		f.library(entry());
		await f.controller.load();
		expect(f.state.logs?.map((l) => l.sha256)).toEqual([SHA]);
		expect(f.state.cards.map((c) => c.name)).toEqual(['00000001.mlog']);
		expect(f.state.library).toBe('/run/logs');
		expect(f.state.error).toBeNull();
	});
	it('ignores an answer that is not a log and shows a failed read', async () => {
		const f = new Fake();
		f.answers.set('log_list', { ok: true, logs: [{ sha256: 1 }, entry()], cards: [{ nope: 1 }] });
		await f.controller.load();
		expect(f.state.logs).toHaveLength(1);
		expect(f.state.cards).toEqual([]);
		f.refuse('log_list', 'boom');
		await f.controller.load();
		expect(f.state.error).toBe('boom');
	});
});

describe('import', () => {
	const started = (id = 3) => ({ ok: true, started: true, import_id: id });
	const doneOf = (e: LogEntry, over: Record<string, unknown> = {}) => ({ event: 'log_import_done', id: 3, ok: true, already_imported: false, ...e, ...over });

	it('sends the trimmed path, shows the helper’s progress, and on the done event reads the library and selects the log with the IMU gyro plotted first', async () => {
		const f = new Fake();
		const stop = f.controller.start();
		f.controller.start();
		expect(f.handlers.size).toBe(1);
		f.library(entry());
		f.answers.set('log_import', started());
		await f.controller.importPath('  /media/x/00000001.mlog ');
		expect(f.sent('log_import')).toEqual([{ cmd: 'log_import', args: { path: '/media/x/00000001.mlog' } }]);
		expect(f.state.importing).toEqual({ id: 3, name: '00000001.mlog', phase: 'copy', bytesDone: 0, bytesTotal: 0 });
		f.push({ event: 'log_import_progress', id: 3, phase: 'copy', bytes_done: 512, bytes_total: 1024 });
		expect(f.state.importing).toMatchObject({ id: 3, name: '00000001.mlog', phase: 'copy', bytesDone: 512, bytesTotal: 1024 });
		f.push({ event: 'log_import_progress', id: 3, phase: 'decode', bytes_done: 1024, bytes_total: 1024 });
		expect(f.state.importing).toMatchObject({ phase: 'decode', bytesDone: 1024 });
		f.push(doneOf(entry()));
		await f.settle();
		expect(f.state).toMatchObject({ importing: null, selected: SHA, kind: 'imu', fields: ['gyro_x'], error: null });
		expect(f.state.notice).toBe(`00000001.mlog: ${shortSha(SHA)}`);
		expect(f.sent('log_list')).toHaveLength(1);
		stop();
		expect(f.handlers.size).toBe(0);
	});
	it('does not start a second import while one goes (the page’s buttons are off too)', async () => {
		const f = new Fake();
		f.controller.start();
		f.answers.set('log_import', started());
		await f.controller.importPath('/a.mlog');
		await f.controller.importPath('/b.mlog');
		expect(f.sent('log_import')).toHaveLength(1);
	});
	it('takes a done event that arrives before the answer of the command (a small file) and does not show the finished import as going', async () => {
		const f = new Fake();
		f.controller.start();
		f.library(entry());
		f.answers.set('log_import', started());
		const real = f.controller as unknown as { deps: { request: (c: string, a?: Record<string, unknown>) => Promise<HelperReply> } };
		const request = real.deps.request;
		real.deps.request = async (cmd, args) => {
			if (cmd === 'log_import') {
				f.push({ event: 'log_import_progress', id: 3, phase: 'copy', bytes_done: 0, bytes_total: 1024 });
				f.push(doneOf(entry()));
			}
			return request(cmd, args);
		};
		await f.controller.importPath('/a.mlog');
		await f.settle();
		expect(f.state.importing).toBeNull();
		expect(f.state.selected).toBe(SHA);
	});
	it('says so when the decoder reports problems and when the file was in the library', async () => {
		const f = new Fake();
		f.controller.start();
		const bad = entry({ summary: summary({ clean: false, problems: [{ sector: 5, reason: 'crc', detail: '' }] }) });
		f.library(bad);
		f.push(doneOf(bad, { already_imported: true }));
		await f.settle();
		expect(f.state.notice).toBe(`00000001.mlog: ${shortSha(SHA)} (already in the library); the decoder reports 1 problem`);
		expect(f.state.selected).toBe(SHA);
	});
	it('shows the helper’s refusal of the command as it words it, and a failed import as its error', async () => {
		const f = new Fake();
		f.controller.start();
		f.refuse('log_import', 'the path is inside the repository (/r): copy the file out of it first (W-1)');
		await f.controller.importPath('/r/x.mlog');
		expect(f.state.error).toBe('the path is inside the repository (/r): copy the file out of it first (W-1)');
		expect(f.state).toMatchObject({ selected: null, importing: null, notice: null });
		f.push({ event: 'log_import_done', id: 4, ok: false, error: 'cancelled', cancelled: true });
		await f.settle();
		expect(f.state).toMatchObject({ error: 'cancelled', importing: null, selected: null });
		expect(f.sent('log_list')).toHaveLength(0);
	});
	it('does not send a blank path', async () => {
		const f = new Fake();
		await f.controller.importPath('   ');
		expect(f.calls).toEqual([]);
		expect(f.state.error).toMatch(/absolute/);
	});
	it('picks up an import that is already going when the page is opened', async () => {
		const f = new Fake();
		f.answers.set('log_list', { ok: true, logs: [], cards: [], library: '/run/logs', importing: { id: 8, name: 'big.mlog' } });
		await f.controller.load();
		expect(f.state.importing).toEqual({ id: 8, name: 'big.mlog', phase: 'copy', bytesDone: 0, bytesTotal: 0 });
	});
});

describe('the plot', () => {
	async function loaded() {
		const f = new Fake();
		f.library(entry(), entry({ sha256: SHA2, summary: summary({ kinds: { status: summary().kinds.status as KindInfo } }) }));
		await f.controller.load();
		f.controller.select(SHA);
		return f;
	}
	const series = { sha256: SHA, kind: 'imu', t0_us: T0, t1_us: T1, bucket_us: 133, buckets: 600, records: 600, n: [1], fields: { gyro_x: { min: [500], max: [40333.5], skipped: 0 } } };

	it('opens a log on its IMU gyro and on the first field of a log without one', async () => {
		const f = await loaded();
		expect(f.state).toMatchObject({ selected: SHA, kind: 'imu', fields: ['gyro_x'] });
		f.controller.select(SHA2);
		expect(f.state).toMatchObject({ selected: SHA2, kind: 'status', fields: ['drops_imu'] });
		f.controller.select('c'.repeat(64));
		expect(f.state.selected).toBe(SHA2);
	});

	it('asks for the chosen fields over the range, in the placeholder bucket count, and keeps the series', async () => {
		const f = await loaded();
		f.controller.toggleField('gyro_y', true);
		f.controller.toggleField('gyro_y', true);
		f.controller.setRange('0.01', '0.05');
		f.answers.set('log_series', { ok: true, series });
		await f.controller.plot();
		expect(f.sent('log_series')).toEqual([
			{ cmd: 'log_series', args: { sha256: SHA, kind: 'imu', fields: ['gyro_x', 'gyro_y'], buckets: PLOT_BUCKETS, t0_us: T0 + 10_000, t1_us: T0 + 50_000 } }
		]);
		expect(f.state.series).toEqual(series);
		expect(f.state.busy).toBeNull();
	});

	it('leaves the range out when blank, and a field off after it is unchecked', async () => {
		const f = await loaded();
		f.controller.toggleField('gyro_x', false);
		await f.controller.plot();
		expect(f.sent('log_series')).toHaveLength(0);
		expect(f.state.error).toBe('choose at least one field to plot');
		f.controller.toggleField('gyro_y', true);
		f.answers.set('log_series', { ok: true, series });
		await f.controller.plot();
		expect(f.sent('log_series')[0]?.args).toEqual({ sha256: SHA, kind: 'imu', fields: ['gyro_y'], buckets: PLOT_BUCKETS });
	});

	it('does not send a range it cannot read, and shows the helper’s refusal of a series', async () => {
		const f = await loaded();
		f.controller.setRange('x', '');
		await f.controller.plot();
		expect(f.sent('log_series')).toHaveLength(0);
		expect(f.state.error).toMatch(/seconds from the start/);
		f.controller.setRange('', '');
		f.refuse('log_series', 'buckets must be 1 to 4096');
		await f.controller.plot();
		expect(f.state.error).toBe('buckets must be 1 to 4096');
		expect(f.state.series).toBeNull();
	});

	it('forgets the series when another kind or log is chosen', async () => {
		const f = await loaded();
		f.answers.set('log_series', { ok: true, series });
		await f.controller.plot();
		expect(f.state.series).not.toBeNull();
		f.controller.setKind('status');
		expect(f.state).toMatchObject({ kind: 'status', fields: ['drops_imu'], series: null, from: '', to: '' });
		f.controller.setKind('nope');
		expect(f.state.kind).toBe('status');
	});
});
