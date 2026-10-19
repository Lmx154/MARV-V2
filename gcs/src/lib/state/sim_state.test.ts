// The Simulation page's logic (GCS-first unit GX-3): the form becomes the helper's request; a form the helper would refuse is not
// sent; each refusal is shown verbatim next to the field it concerns; every override and the run carry the sil-override label; the
// page never opens a link (it only talks to the helper's run commands).
import { describe, expect, it } from 'vitest';
import { get } from 'svelte/store';
import { HelperError, type HelperReply } from '$link/transport';
import { manifestJson } from '$lib/params/testing/fixture';
import type { ImageList } from '$lib/params/library';
import type { HelperStatus, RunManifest } from './helper_api';
import {
	SIL_BUILD_TREE,
	SIL_LABEL,
	SimController,
	buildRequest,
	parseValue,
	placeMessages,
	reasonsOf,
	runView,
	type RunPlan,
	type SimForm
} from './sim_state';

const CARD = 'vehicles/uzh_neurobem_5in.yaml';
// The helper's texts, as tools/sim/sil_run.py and tools/gs/runs.py write them.
const UNKNOWN_PARAM = "override 'no_such_parameter': not a parameter of the image's parameter set";
const TWICE = 'override rate_kp: given twice';
const BAD_CARD = "unknown card 'vehicles/nope.yaml': a file of the vehicles directory";
const BUSY = 'run 20261019T000000Z-sil-1 is still going: one simulation at a time (H-1)';
const LOCK = 'the heavy lock is held (cmd=ctest): heavy work waits for it (H-3)';

const form = (over: Partial<SimForm> = {}): SimForm => ({ card: '', overrides: [], gui: false, ...over });
const row = (id: number, name: string, value: string) => ({ id, name, value });

describe('parseValue', () => {
	it.each([
		['3.5', 3.5],
		[' -2 ', -2],
		['1e-3', 0.001],
		['.5', 0.5],
		['+7', 7],
		['6.', 6]
	])('reads %j as a number', (text, n) => expect(parseValue(text)).toBe(n));

	it.each(['', ' ', 'abc', '0x10', 'Infinity', 'NaN', '1,5', '1e', '--1', '1 2', '1e999'])('does not read %j', (text) => expect(parseValue(text)).toBeNull());
});

describe('buildRequest: the form as the helper\'s request', () => {
	it('an empty form is a sil run with no card, no override and the gui off', () => {
		expect(buildRequest(form())).toEqual({ request: { kind: 'sil', gui: false }, problems: [] });
	});

	it('maps the card, the gui flag and each override to a JSON number, in order', () => {
		const built = buildRequest(form({ card: CARD, gui: true, overrides: [row(1, ' rate_kp ', '12.5'), row(2, 'rc_channel_roll', '7')] }));
		expect(built.request).toEqual({
			kind: 'sil',
			card: CARD,
			gui: true,
			overrides: [
				{ name: 'rate_kp', value: 12.5 },
				{ name: 'rc_channel_roll', value: 7 }
			]
		});
	});

	it('a blank row is no override; a row with a name but no number, or a number but no name, is a problem of that row and sends nothing', () => {
		expect(buildRequest(form({ overrides: [row(1, '', ''), row(2, ' ', ' ')] })).request).toEqual({ kind: 'sil', gui: false });
		const built = buildRequest(form({ overrides: [row(1, 'rate_kp', 'x'), row(2, '', '3'), row(3, 'att_kp', '1')] }));
		expect(built.request).toBeNull();
		expect(built.problems.map((p) => [p.target, p.notice.text, p.notice.source])).toEqual([
			[{ kind: 'row', id: 1 }, '"x" is not a number', 'form'],
			[{ kind: 'row', id: 2 }, 'name the parameter', 'form']
		]);
	});
});

describe('placeMessages: a refusal is shown where it concerns, as the helper wrote it', () => {
	const f = form({ overrides: [row(1, 'att_kp', '1'), row(2, 'no_such_parameter', '2'), row(3, 'rate_kp', '3'), row(4, 'rate_kp', '4')] });

	it('an unknown parameter goes to its row, the verbatim text', () => {
		const p = placeMessages([UNKNOWN_PARAM], f);
		expect(p.rows).toEqual({ 2: [{ text: UNKNOWN_PARAM, source: 'helper' }] });
		expect(p.card).toEqual([]);
		expect(p.run).toEqual([]);
	});

	it('a repeated parameter goes to the second row of that name; the other override messages to the first', () => {
		expect(Object.keys(placeMessages([TWICE], f).rows)).toEqual(['4']);
		expect(Object.keys(placeMessages(['override rate_kp: the value must be a number'], f).rows)).toEqual(['3']);
	});

	it('a card message goes to the card field', () => {
		expect(placeMessages([BAD_CARD], f).card).toEqual([{ text: BAD_CARD, source: 'helper' }]);
		expect(placeMessages(['card must be a non-empty string'], f).card).toHaveLength(1);
	});

	it('every other reason is the run\'s, all of them, each once', () => {
		const p = placeMessages([BUSY, LOCK, BUSY], f);
		expect(p.run.map((n) => n.text)).toEqual([BUSY, LOCK]);
		expect(p.card).toEqual([]);
		expect(p.rows).toEqual({});
	});

	it('a message about a name that has no row is the run\'s (it is not lost)', () => {
		expect(placeMessages(["override 'gone': not a parameter of the image's parameter set"], f).run).toHaveLength(1);
	});
});

describe('reasonsOf', () => {
	it('lists every refusal of a helper reply, else the one error text', () => {
		const reply: HelperReply = { ok: false, error: BUSY, refusals: [BUSY, LOCK] };
		expect(reasonsOf(new HelperError(reply, 'run_start'))).toEqual([BUSY, LOCK]);
		expect(reasonsOf(new HelperError({ ok: false, error: UNKNOWN_PARAM }, 'run_plan'))).toEqual([UNKNOWN_PARAM]);
		expect(reasonsOf(new Error('the helper connection is not open'))).toEqual(['the helper connection is not open']);
	});
});

const PLAN: RunPlan = {
	kind: 'sil',
	run_id: '20261019T000000Z-sil-1',
	argv: ['python', 'tools/sim/run_quad_fc.py', '--interactive', '--override', 'rate_kp=12.5'],
	options: { overrides: [{ name: 'rate_kp', type: 'f32', value: 12.5 }] },
	label: SIL_LABEL,
	overrides: [{ name: 'rate_kp', type: 'f32', value: 12.5 }],
	out_dir: '/tmp/out',
	udp_port: 14550,
	allowed: true,
	refusals: []
};

type Call = { cmd: string; args: Record<string, unknown> | undefined };

function make(answer: (cmd: string, args?: Record<string, unknown>) => HelperReply | Promise<HelperReply>, images?: ImageList, artifact?: unknown) {
	const calls: Call[] = [];
	let refreshes = 0;
	const sim = new SimController({
		request: async (cmd, args) => {
			calls.push({ cmd, args });
			const reply = await answer(cmd, args);
			if (!reply.ok) throw new HelperError(reply, cmd);
			return reply;
		},
		listImages: async () => images ?? { images: [], unkeyed: [] },
		artifact: async () => artifact,
		refresh: async () => void refreshes++
	});
	return { sim, calls, refreshes: () => refreshes, state: () => get(sim.state) };
}

describe('SimController', () => {
	it('lists the cards the helper gives', async () => {
		const { sim, calls, state } = make(() => ({ ok: true, cards: [CARD, 'vehicles/marv_7in_fc3v2.yaml'] }));
		await sim.loadCards();
		expect(calls).toEqual([{ cmd: 'cards', args: undefined }]);
		expect(state().cards).toEqual([CARD, 'vehicles/marv_7in_fc3v2.yaml']);
		expect(state().cardsError).toBeNull();
	});

	it('plans with the request of the form, keeps the plan with its label, and starts nothing', async () => {
		const { sim, calls, state } = make(() => ({ ok: true, plan: PLAN }));
		sim.setCard(CARD);
		sim.addOverride();
		sim.setOverride(1, { name: 'rate_kp', value: '12.5' });
		await sim.plan();
		expect(calls).toEqual([{ cmd: 'run_plan', args: { kind: 'sil', card: CARD, gui: false, overrides: [{ name: 'rate_kp', value: 12.5 }] } }]);
		expect(state().plan?.label).toBe(SIL_LABEL);
		expect(state().plan?.overrides).toEqual([{ name: 'rate_kp', type: 'f32', value: 12.5 }]);
		expect(state().busy).toBeNull();
	});

	it('the refusals of a plan that is not allowed are shown, all of them (the plan itself is shown too)', async () => {
		const { sim, state } = make(() => ({ ok: true, plan: { ...PLAN, allowed: false, refusals: [BUSY, LOCK] } }));
		await sim.plan();
		expect(state().plan?.allowed).toBe(false);
		expect(state().placed.run.map((n) => n.text)).toEqual([BUSY, LOCK]);
	});

	it('an unknown parameter is refused by the helper and its text is shown by that row, verbatim', async () => {
		const { sim, state } = make(() => ({ ok: false, error: UNKNOWN_PARAM }));
		sim.addOverride();
		sim.addOverride();
		sim.setOverride(1, { name: 'rate_kp', value: '1' });
		sim.setOverride(2, { name: 'no_such_parameter', value: '1' });
		await sim.plan();
		expect(state().plan).toBeNull();
		expect(state().placed.rows[2]).toEqual([{ text: UNKNOWN_PARAM, source: 'helper' }]);
		expect(state().placed.rows[1]).toBeUndefined();
		expect(state().placed.run).toEqual([]);
	});

	it('a form the helper would refuse is not sent: the row says why', async () => {
		const { sim, calls, state } = make(() => ({ ok: true, plan: PLAN }));
		sim.addOverride();
		sim.setOverride(1, { name: 'rate_kp', value: 'twelve' });
		await sim.plan();
		await sim.launch();
		expect(calls).toEqual([]);
		expect(state().placed.rows[1]).toEqual([{ text: '"twelve" is not a number', source: 'form' }]);
	});

	it('launch sends run_start with the same request, clears the plan and reads the status again', async () => {
		const { sim, calls, state, refreshes } = make((cmd) => (cmd === 'run_plan' ? { ok: true, plan: PLAN } : { ok: true, run: {} }));
		sim.setGui(true);
		await sim.plan();
		expect(state().plan).not.toBeNull();
		await sim.launch();
		expect(calls.map((c) => c.cmd)).toEqual(['run_plan', 'run_start']);
		expect(calls[1]?.args).toEqual({ kind: 'sil', gui: true });
		expect(state().plan).toBeNull();
		expect(state().placed).toEqual({ card: [], rows: {}, run: [] });
		expect(refreshes()).toBe(1);
	});

	it('every refusal of a launch is shown (the first one is the error text, the list has them all)', async () => {
		const { sim, state } = make(() => ({ ok: false, error: BUSY, refusals: [BUSY, LOCK] }));
		await sim.launch();
		expect(state().placed.run.map((n) => n.text)).toEqual([BUSY, LOCK]);
	});

	it('a bad card is shown by the card field', async () => {
		const { sim, state } = make(() => ({ ok: false, error: BAD_CARD }));
		sim.setCard('vehicles/nope.yaml');
		await sim.launch();
		expect(state().placed.card).toEqual([{ text: BAD_CARD, source: 'helper' }]);
		expect(state().placed.run).toEqual([]);
	});

	it('stop sends run_stop only and shows its refusal', async () => {
		const ok = make(() => ({ ok: true, run: {}, exit_status: 0 }));
		await ok.sim.stop();
		expect(ok.calls).toEqual([{ cmd: 'run_stop', args: undefined }]);
		expect(ok.refreshes()).toBe(1);
		const none = make(() => ({ ok: false, error: 'no run is going' }));
		await none.sim.stop();
		expect(none.state().placed.run).toEqual([{ text: 'no run is going', source: 'helper' }]);
	});

	it('a change of the form drops the plan and the messages about the form as it was', async () => {
		const { sim, state } = make(() => ({ ok: true, plan: { ...PLAN, allowed: false, refusals: [BUSY] } }));
		await sim.plan();
		expect(state().plan).not.toBeNull();
		sim.setGui(true);
		expect(state().plan).toBeNull();
		expect(state().placed).toEqual({ card: [], rows: {}, run: [] });
	});

	it('one command at a time', async () => {
		let release: (r: HelperReply) => void = () => undefined;
		const { sim, calls, state } = make(() => new Promise<HelperReply>((r) => (release = r)));
		const first = sim.launch();
		expect(state().busy).toBe('launch');
		await sim.launch();
		await sim.stop();
		expect(calls.map((c) => c.cmd)).toEqual(['run_start']);
		release({ ok: true, run: {} });
		await first;
		expect(state().busy).toBeNull();
	});

	it('rows are removed by id and keep their own text', () => {
		const { sim, state } = make(() => ({ ok: true }));
		sim.addOverride();
		sim.addOverride();
		sim.setOverride(1, { name: 'a', value: '1' });
		sim.setOverride(2, { name: 'b', value: '2' });
		sim.removeOverride(1);
		expect(state().form.overrides).toEqual([{ id: 2, name: 'b', value: '2' }]);
	});

	it('offers the parameter names of the SIL image\'s manifest, with their type and lock', async () => {
		const images: ImageList = {
			images: [
				{ hash: 'a'.repeat(64), tree: 'other', name: 'marv_sil_quad_fc', kind: 'sil', params_set: 'x', artifacts: { params_manifest: 'o/m.json' } },
				{ hash: 'b'.repeat(64), tree: SIL_BUILD_TREE, name: 'marv_sil_quad_fc', kind: 'sil', params_set: 'marv_params_fc3v2', artifacts: { params_manifest: 'g/m.json' } }
			],
			unkeyed: []
		};
		const { sim, state } = make(() => ({ ok: true }), images, manifestJson());
		await sim.loadParameters();
		const names = state().parameters?.map((p) => p.name);
		expect(names).toContain('rate_kp');
		expect(state().parameters?.find((p) => p.name === 'arm_throttle_min')).toMatchObject({ type: 'f32', locked: true });
		expect(state().parametersNote).toBeNull();
	});

	it('says so when the library has no SIL manifest (the helper still checks every name)', async () => {
		const { sim, state } = make(() => ({ ok: true }));
		await sim.loadParameters();
		expect(state().parameters).toBeNull();
		expect(state().parametersNote).toContain(SIL_BUILD_TREE);
	});
});

describe('runView: the run on screen and its label', () => {
	const manifest = (over: Partial<RunManifest> = {}): RunManifest => ({
		run_id: '20261019T000000Z-sil-1',
		kind: 'sil',
		state: 'running',
		exit_status: null,
		relaunches: 0,
		label: SIL_LABEL,
		overrides: [{ name: 'rate_kp', type: 'f32', value: 12.5 }],
		...over
	});
	const status = (runs: HelperStatus['runs']): HelperStatus => ({ runs }) as HelperStatus;

	it('a run with an override is labelled sil-override while it runs and after it stopped; a run without is not', () => {
		const running = runView(status({ active: { id: 'x', kind: 'sil' }, last: manifest() }));
		expect(running).toMatchObject({ running: true, label: SIL_LABEL });
		const stopped = runView(status({ active: null, last: manifest({ state: 'stopped', exit_status: 0 }) }));
		expect(stopped).toMatchObject({ running: false, label: SIL_LABEL });
		const plain = runView(status({ active: null, last: manifest({ label: null, overrides: [] }) }));
		expect(plain).toMatchObject({ running: false, label: null });
	});

	it('knows no run before one, and none without a status', () => {
		expect(runView(status({ active: null, last: null }))).toEqual({ running: false, run: null, label: null });
		expect(runView(null)).toEqual({ running: false, run: null, label: null });
	});
});
