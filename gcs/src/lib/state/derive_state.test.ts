// The Derive panel's logic (GCS-first unit GX-13): nothing is derived until the click, every refusal and lint finding is the helper's own text,
// an edit needs its source text before anything is sent, the progress and the end come from the helper's events, the validation is the existing
// one (the report is fetched and its id checked) and the hand-over to the vehicle stages only what differs.
import { describe, expect, it } from 'vitest';
import { get } from 'svelte/store';
import { HelperError, type HelperEvent, type HelperReply } from '$link/transport';
import {
	DeriveController, deriveOutcome, findingsOf, parseValueInput, reasonsOf, recordOf, showValue, type DeriveEntry, type DeriveState, type DeriveVehicle
} from './derive_state';

const entry = (over: Partial<DeriveEntry>): DeriveEntry => ({
	key: 'mass', value: 0.732, unit: 'kg', shape: null, method: 'measured', source: 'old text', sigma: 'UNKNOWN', status: null, locked: false, lock: null, ...over
});
const ENTRIES: DeriveEntry[] = [
	entry({}),
	entry({ key: 'rotors.m1.position', value: [0.112, 0.1275, -0.0233], shape: 'frd3', unit: 'm' }),
	entry({ key: 'rotors.m1.spin', value: 'ccw', unit: null, shape: null }),
	entry({ key: 'rotors.speed_range', value: [0, 2297], shape: 'range', locked: true, lock: { by: 'Luis', on: '2026-10-17', via: 'manual' } })
];
const IMAGES = [{ hash: 'a'.repeat(64), name: 'marv_sil_quad_fc', kind: 'sil', tree: 'host-gz-quadfc' }, { hash: 'b'.repeat(64), name: 'marv_fc3v2', kind: 'board', tree: 'm33' }];
const CONFIG = 'c'.repeat(64);
const DONE = {
	id: '20261021T000000Z-0', running: false, state: 'done', ok: true, card: 'vehicles/marv_7in_fc3v2.yaml', edits: [], started_utc: 't', finished_utc: 't2', error: null,
	findings: [], step: 'mcfg', tail: [], config_hash: CONFIG, schema_hash: '0x0000000000000001', reports: ['x_rate_report.txt'], images: IMAGES, changes: [], baseline: null
};

class FakeVehicle implements DeriveVehicle {
	why: string | null = null;
	list: Array<{ linkId: string | null; stagedBits: number | null; locked: boolean | null }> = [];
	written: Array<[string, string, number, string]> = [];
	refuse = new Set<string>();
	schema: string | null = '0x0000000000000001';
	whyCannotStage() {
		return this.why;
	}
	schemaHash() {
		return this.schema;
	}
	params() {
		return this.list;
	}
	async stage(linkId: string, type: never, bits: number, source: string) {
		this.written.push([linkId, type, bits, source]);
		return { accepted: !this.refuse.has(linkId), reasons: this.refuse.has(linkId) ? ['not now'] : [] };
	}
}

class Fake {
	calls: Array<{ cmd: string; args: Record<string, unknown> | undefined }> = [];
	answers = new Map<string, HelperReply | Error>();
	handlers = new Set<(e: HelperEvent) => void>();
	vehicle = new FakeVehicle();
	reportBytes: Uint8Array | Error = new Uint8Array();
	controller = new DeriveController({
		request: async (cmd, args) => {
			this.calls.push({ cmd, args });
			const a = this.answers.get(cmd) ?? { ok: true };
			if (a instanceof Error) throw a;
			return a;
		},
		onEvent: (h) => {
			this.handlers.add(h);
			return () => this.handlers.delete(h);
		},
		report: async () => {
			if (this.reportBytes instanceof Error) throw this.reportBytes;
			return this.reportBytes;
		},
		vehicle: this.vehicle
	});
	constructor() {
		this.answers.set('cards', { ok: true, cards: ['vehicles/marv_7in_fc3v2.yaml', 'vehicles/uzh_neurobem_5in.yaml'] });
		this.answers.set('derive_card', { ok: true, card: 'vehicles/marv_7in_fc3v2.yaml', vehicle: 'marv_7in_fc3v2', entries: ENTRIES });
		this.answers.set('derive_status', { ok: true, derive: { current: null } });
		this.controller.start();
	}
	push(e: HelperEvent) {
		this.handlers.forEach((h) => h(e));
	}
	settle() {
		return new Promise((r) => setTimeout(r, 0));
	}
	get state(): DeriveState {
		return get(this.controller.state);
	}
	sent(cmd: string) {
		return this.calls.filter((c) => c.cmd === cmd);
	}
}

const refusal = (reply: HelperReply) => new HelperError(reply, 'derive');

describe('loading', () => {
	it('lists the helper’s cards and the editable entries of the first, and derives nothing', async () => {
		const f = new Fake();
		await f.controller.load();
		expect(f.state.cards).toEqual(['vehicles/marv_7in_fc3v2.yaml', 'vehicles/uzh_neurobem_5in.yaml']);
		expect(f.state.card).toBe('vehicles/marv_7in_fc3v2.yaml');
		expect(f.state.entries?.map((e) => e.key)).toEqual(['mass', 'rotors.m1.position', 'rotors.m1.spin', 'rotors.speed_range']);
		expect(f.sent('derive_card')[0]?.args).toEqual({ card: 'vehicles/marv_7in_fc3v2.yaml' });
		expect(f.sent('derive')).toEqual([]);
	});

	it('asks again for the card the operator picks and drops the edits typed for the other one', async () => {
		const f = new Fake();
		await f.controller.load();
		f.controller.setInput('mass', { value: '0.8', source: 's' });
		await f.controller.setCard('vehicles/uzh_neurobem_5in.yaml');
		expect(f.state.inputs).toEqual({});
		expect(f.sent('derive_card').at(-1)?.args).toEqual({ card: 'vehicles/uzh_neurobem_5in.yaml' });
	});

	it('shows the helper’s refusal of a card verbatim', async () => {
		const f = new Fake();
		f.answers.set('derive_card', refusal({ ok: false, error: "unknown card 'x': a file of the vehicles directory" }));
		await f.controller.load();
		expect(f.state.notices).toEqual(["unknown card 'x': a file of the vehicles directory"]);
	});
});

describe('the edits of the form', () => {
	it('parses a number, a list of the entry’s length and a category, and refuses the rest with the reason', async () => {
		const [mass, pos, spin] = ENTRIES as [DeriveEntry, DeriveEntry, DeriveEntry];
		expect(parseValueInput('0.8', mass)).toEqual({ ok: true, value: 0.8 });
		expect(parseValueInput(' 3.4e-6 ', mass)).toEqual({ ok: true, value: 3.4e-6 });
		expect(parseValueInput('0.1, 0.2 -0.03', pos)).toEqual({ ok: true, value: [0.1, 0.2, -0.03] });
		expect(parseValueInput('cw', spin)).toEqual({ ok: true, value: 'cw' });
		for (const [text, e] of [['', mass], ['heavy', mass], ['1 2', mass], ['1,2', pos], ['1,2,x', pos], ['0x10', mass], ['NaN', mass], ['Infinity', mass], ['1e999', mass]] as const) {
			expect(parseValueInput(text, e).ok, text).toBe(false);
		}
		expect(showValue([1, 2])).toBe('1, 2');
	});

	it('sends an edit only with its value and its source text, and says what is missing', async () => {
		const f = new Fake();
		await f.controller.load();
		const why = () => f.controller.whyNoDerive(f.state, true);
		expect(why()).toBeNull(); // no edit: the card as it is
		f.controller.setInput('mass', { value: '0.8' });
		expect(why()).toBe('mass: the new value needs its source or rationale text');
		f.controller.setInput('mass', { source: '   ' });
		expect(why()).toBe('mass: the new value needs its source or rationale text');
		await f.controller.derive();
		expect(f.sent('derive')).toEqual([]);
		f.controller.setInput('mass', { source: 'scale reading 0.800 kg' });
		expect(why()).toBeNull();
		f.controller.setInput('mass', { value: '' });
		expect(why()).toBe('mass: a source text without a new value');
		f.controller.setInput('mass', { value: 'x', source: 'y' });
		expect(why()).toBe('mass: "x" is not a number or a list of numbers');
		expect(f.controller.whyNoDerive(f.state, false)).toBe('the helper is not connected');
	});

	it('sends the parsed edits in the entry order, with the source texts verbatim', async () => {
		const f = new Fake();
		await f.controller.load();
		f.controller.setInput('rotors.m1.position', { value: '0.11, 0.13, -0.02', source: ' caliper ' });
		f.controller.setInput('mass', { value: '0.8', source: 'scale' });
		f.answers.set('derive', { ok: true, started: true, derive: { ...DONE, running: true, state: 'running', ok: null } });
		await f.controller.derive();
		expect(f.sent('derive')[0]?.args).toEqual({
			card: 'vehicles/marv_7in_fc3v2.yaml',
			edits: [{ key: 'mass', value: 0.8, source: 'scale' }, { key: 'rotors.m1.position', value: [0.11, 0.13, -0.02], source: ' caliper ' }]
		});
		expect(f.state.current?.running).toBe(true);
	});
});

describe('the helper’s refusals', () => {
	it('shows the lint findings of a refused copy verbatim and runs nothing', async () => {
		const f = new Fake();
		await f.controller.load();
		f.controller.setInput('mass', { value: '0.8', source: 'x' });
		const findings = [{ file: 'vehicles/marv_7in_fc3v2.yaml', entry: 'mass', reason: 'source: must be non-empty text' }];
		f.answers.set('derive', refusal({ ok: false, error: 'the card copy does not lint: 1 finding', findings }));
		await f.controller.derive();
		expect(f.state.findings).toEqual(findings);
		expect(f.state.notices).toEqual(['the card copy does not lint: 1 finding']);
		expect(f.state.busy).toBeNull();
		expect(deriveOutcome(f.state.current)).toBe('none');
	});

	it('shows every reason the helper lists for a refusal of the heavy part, and the same click works when they are gone', async () => {
		const f = new Fake();
		await f.controller.load();
		f.answers.set('derive', refusal({ ok: false, error: 'a', refusals: ['the heavy lock is held (pid=1 …): heavy work waits for it (H-3)', 'free memory is low'] }));
		await f.controller.derive();
		expect(f.state.notices).toEqual(['the heavy lock is held (pid=1 …): heavy work waits for it (H-3)', 'free memory is low']);
		f.answers.set('derive', { ok: true, started: true, derive: { ...DONE, running: true, ok: null, state: 'running' } });
		await f.controller.derive();
		expect(f.state.notices).toEqual([]);
		expect(f.sent('derive')).toHaveLength(2);
	});

	it('helpers of the parse', () => {
		expect(findingsOf([{ file: 'a', entry: 'b', reason: 'c' }, 5, null])).toEqual([{ file: 'a', entry: 'b', reason: 'c' }]);
		expect(findingsOf('x')).toEqual([]);
		expect(reasonsOf(new Error('boom'))).toEqual(['boom']);
		expect(recordOf({ id: 1 })).toBeNull();
		expect(recordOf(DONE)?.images).toHaveLength(2);
	});
});

describe('the run and its result', () => {
	it('follows the helper’s events, then reads the whole record and picks the board image', async () => {
		const f = new Fake();
		await f.controller.load();
		f.answers.set('derive', { ok: true, started: true, derive: { ...DONE, running: true, state: 'running', ok: null } });
		await f.controller.derive();
		f.push({ event: 'derive_started', id: DONE.id, card: DONE.card });
		f.push({ event: 'derive_step', id: DONE.id, step: 'flatten', index: 1, steps: 3 });
		expect(f.state.current?.step).toBe('flatten');
		expect(f.state.current?.tail).toEqual(['step 1/3: flatten']);
		expect(f.controller.whyNoDerive(f.state, true)).toBe('a derivation is running: one at a time');
		f.answers.set('derive_status', { ok: true, derive: { current: DONE } });
		f.push({ event: 'derive_done', id: DONE.id, ok: true });
		await f.settle();
		expect(deriveOutcome(f.state.current)).toBe('ok');
		expect(f.state.current?.reports).toEqual(['x_rate_report.txt']);
		expect(f.state.image).toBe('b'.repeat(64));
		expect(f.controller.whyNoDerive(f.state, true)).toBeNull();
	});

	it('reads a report by the name the helper listed', async () => {
		const f = new Fake();
		await f.controller.load();
		f.answers.set('derive_status', { ok: true, derive: { current: DONE } });
		f.push({ event: 'derive_done', id: DONE.id, ok: true });
		await f.settle();
		f.answers.set('derive_report', { ok: true, text: 'a derivation report\n' });
		await f.controller.openReport('x_rate_report.txt');
		expect(f.sent('derive_report')[0]?.args).toEqual({ derive_id: DONE.id, name: 'x_rate_report.txt' });
		expect(f.state.reportText).toBe('a derivation report\n');
	});

	it('says cancelled, failed and refused as the helper’s record says them', () => {
		const rec = recordOf(DONE);
		expect(deriveOutcome({ ...rec!, ok: false, state: 'failed' })).toBe('failed');
		expect(deriveOutcome({ ...rec!, ok: false, state: 'cancelled' })).toBe('cancelled');
		expect(deriveOutcome({ ...rec!, ok: false, state: 'refused' })).toBe('refused');
		expect(deriveOutcome({ ...rec!, running: true })).toBe('running');
	});

	it('cancels through the helper', async () => {
		const f = new Fake();
		await f.controller.cancel();
		expect(f.sent('derive_cancel')).toHaveLength(1);
	});
});

async function withResult(f: Fake) {
	await f.controller.load();
	f.answers.set('derive_status', { ok: true, derive: { current: DONE } });
	f.push({ event: 'derive_done', id: DONE.id, ok: true });
	await f.settle();
}

async function sha256Hex(bytes: Uint8Array): Promise<string> {
	return Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256', bytes as BufferSource)), (b) => b.toString(16).padStart(2, '0')).join('');
}

describe('the validation of the derived config (G7)', () => {
	const report = (verdict: string) =>
		new TextEncoder().encode(JSON.stringify({ format: 'marv-validation-report/1', verdict, image: { sha256: 'b'.repeat(64) }, config: { config_hash: CONFIG }, checks: [] }) + '\n');

	it('is the helper’s validate for (image, config hash), then the report is fetched and its id checked', async () => {
		const f = new Fake();
		await withResult(f);
		f.reportBytes = report('PASS');
		const id = await sha256Hex(f.reportBytes);
		const running = f.controller.validate();
		await f.settle();
		f.push({ event: 'validate_progress', image_hash: 'b'.repeat(64), config_hash: CONFIG, line: 'check 1' });
		f.push({ event: 'validate_progress', image_hash: 'a'.repeat(64), config_hash: CONFIG, line: 'another image' });
		f.push({ event: 'validate_done', image_hash: 'b'.repeat(64), config_hash: CONFIG, verdict: 'PASS', report_id: id });
		await running;
		expect(f.sent('validate')[0]?.args).toEqual({ image_hash: 'b'.repeat(64), config: CONFIG });
		expect(f.state.validation.phase).toBe('done');
		expect(f.state.validation.progress).toEqual(['check 1']);
		expect(f.state.validation.record?.report?.verdict).toBe('PASS');
		expect(f.state.busy).toBeNull();
	});

	it('refuses a report whose id is not the SHA-256 of its bytes, and a validator that wrote none', async () => {
		const f = new Fake();
		await withResult(f);
		f.reportBytes = report('PASS');
		let running = f.controller.validate();
		await f.settle();
		f.push({ event: 'validate_done', image_hash: 'b'.repeat(64), config_hash: CONFIG, verdict: 'PASS', report_id: '0'.repeat(64) });
		await running;
		expect(f.state.validation.phase).toBe('failed');
		expect(f.state.validation.error).toContain('not the SHA-256');
		running = f.controller.validate();
		await f.settle();
		f.push({ event: 'validate_done', image_hash: 'b'.repeat(64), config_hash: CONFIG, verdict: null, report_id: null });
		await running;
		expect(f.state.validation.error).toContain('no report');
	});

	it('is off without a derived config, without an image of the config’s schema or without a pick', async () => {
		const f = new Fake();
		await f.controller.load();
		expect(f.controller.whyNoValidate(f.state, true)).toBe('no derived config');
		await withResult(f);
		expect(f.controller.whyNoValidate(f.state, true)).toBeNull();
		f.controller.setImage('');
		expect(f.controller.whyNoValidate(f.state, true)).toBe('pick an image');
		f.answers.set('derive_status', { ok: true, derive: { current: { ...DONE, images: [] } } });
		f.push({ event: 'derive_done', id: DONE.id, ok: true });
		await f.settle();
		expect(f.controller.whyNoValidate(f.state, true)).toContain('no image of the library');
	});
});

describe('staging on the vehicle', () => {
	const values = (bits: number[]) => ({
		ok: true, derive_id: DONE.id, params: bits.map((b, i) => ({ name: `p${i}`, link_id: `l${i}`, type: 'f32', unit: '1', bits: b, method: 'derived', locked: false, source: `source ${i}` }))
	});

	it('stages the values that differ, with their own source texts, and counts the ones that are the same', async () => {
		const f = new Fake();
		await withResult(f);
		f.answers.set('derive_values', values([0x40000000, 0x3f800000, 0x40400000]));
		f.vehicle.list = [{ linkId: 'l0', stagedBits: 0x3f800000, locked: false }, { linkId: 'l1', stagedBits: 0x3f800000, locked: false }, { linkId: 'l2', stagedBits: 0, locked: true }];
		await f.controller.stage();
		expect(f.vehicle.written).toEqual([['l0', 'f32', 0x40000000, 'source 0']]);
		expect(f.state.staging.staged).toEqual(['p0']);
		expect(f.state.staging.same).toBe(1);
		expect(f.state.staging.notes).toEqual(['p2: locked, not staged']);
		expect(f.state.staging.phase).toBe('done');
	});

	it('reports what the vehicle refuses and what it does not list', async () => {
		const f = new Fake();
		await withResult(f);
		f.answers.set('derive_values', values([0x40000000, 0x40400000]));
		f.vehicle.list = [{ linkId: 'l0', stagedBits: 0, locked: false }];
		f.vehicle.refuse.add('l0');
		await f.controller.stage();
		expect(f.state.staging.notes).toEqual(['p0: refused by the vehicle (not now)', 'p1: the vehicle does not list it']);
		expect(f.state.staging.phase).toBe('failed');
	});

	it('refuses a vehicle of another parameter schema, naming both hashes, and writes nothing; the same config stages on a vehicle of its schema', async () => {
		const f = new Fake();
		await withResult(f);
		f.answers.set('derive_values', values([0x40000000]));
		f.vehicle.list = [{ linkId: 'l0', stagedBits: 0, locked: false }];
		f.vehicle.schema = '0x00000000000000ff';
		const why = f.controller.whyNoStage(f.state);
		expect(why).toContain('0x0000000000000001');
		expect(why).toContain('0x00000000000000ff');
		await f.controller.stage();
		expect(f.sent('derive_values')).toEqual([]);
		expect(f.vehicle.written).toEqual([]);
		expect(f.state.staging.phase).toBe('failed');
		expect(f.state.staging.notes).toEqual([why]);
		f.vehicle.schema = null;
		expect(f.controller.whyNoStage(f.state)).toBe('the vehicle has not reported its identity');
		// the control: the same derived config on a vehicle that reports its schema (compared as numbers of hex digits, whatever the case)
		f.vehicle.schema = '0x0000000000000001';
		expect(f.controller.whyNoStage(f.state)).toBeNull();
		await f.controller.stage();
		expect(f.vehicle.written).toEqual([['l0', 'f32', 0x40000000, 'source 0']]);
		expect(f.state.staging.phase).toBe('done');
	});

	it('is off while the vehicle cannot take parameters and nothing is written', async () => {
		const f = new Fake();
		await withResult(f);
		f.vehicle.why = 'the vehicle is armed: parameters are written only while disarmed';
		expect(f.controller.whyNoStage(f.state)).toBe(f.vehicle.why);
		await f.controller.stage();
		expect(f.sent('derive_values')).toEqual([]);
		expect(f.vehicle.written).toEqual([]);
	});
});
