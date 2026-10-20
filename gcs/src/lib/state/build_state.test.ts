// The Build panel's logic (GCS-first unit GX-14): nothing is built until the click, every refusal is the helper's own text, the progress
// and the end come from the helper's events, and the library is read again when a build ends.
import { describe, expect, it } from 'vitest';
import { get } from 'svelte/store';
import { HelperError, type HelperEvent, type HelperReply } from '$link/transport';
import { BuildController, TAIL_LINES, buildOutcome, reasonsOf, recordOf, statusText, whyNoBuild, type BuildState } from './build_state';

const ALLOWED = [
	{ preset: 'host-gz-quadfc', tree: 'host-gz-quadfc', configure: 'host-gz-quadfc', cache: {}, targets: [] },
	{ preset: 'm33', tree: 'm33', configure: 'm33', cache: {}, targets: [] }
];
const RUNNING = {
	running: true, preset: 'host-gz-quadfc', tree: 'host-gz-quadfc', describe: 'quad-L2-pass-1-gabc', started_utc: 't', finished_utc: null, ok: null,
	cancelled: false, exit_status: null, error: null, step: null, steps: null, tail: [], log: '/l', images: []
};

class Fake {
	calls: Array<{ cmd: string; args: Record<string, unknown> | undefined }> = [];
	answers = new Map<string, HelperReply | Error>();
	handlers = new Set<(e: HelperEvent) => void>();
	refreshed = 0;
	controller = new BuildController({
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
		refreshImages: async () => void this.refreshed++
	});
	constructor() {
		this.status({ kind: 'steps', steps: 12, text: '12 steps' });
	}
	status(status: unknown, refusals: string[] = [], current: unknown = null) {
		this.answers.set('build_status', { ok: true, build: { allowed: ALLOWED, running: false, current, limits: {}, preset: 'x', status, refusals } });
	}
	push(e: HelperEvent) {
		this.handlers.forEach((h) => h(e));
	}
	settle() {
		return new Promise((r) => setTimeout(r, 0));
	}
	get state() {
		return get(this.controller.state);
	}
	sent(cmd: string) {
		return this.calls.filter((c) => c.cmd === cmd);
	}
}

describe('the status', () => {
	it('reads the allow-list from the helper, picks its first preset and shows the dry run, and builds nothing', async () => {
		const f = new Fake();
		await f.controller.refresh();
		expect(f.state.allowed?.map((p) => p.preset)).toEqual(['host-gz-quadfc', 'm33']);
		expect(f.state.preset).toBe('host-gz-quadfc');
		expect(f.state.status?.text).toBe('12 steps');
		expect(f.sent('build')).toEqual([]);
		expect(f.sent('build_status').map((c) => c.args)).toEqual([{}, { preset: 'host-gz-quadfc' }]);
	});

	it('asks again for the preset the operator picks, and still builds nothing', async () => {
		const f = new Fake();
		await f.controller.refresh();
		f.status({ kind: 'up_to_date', steps: null, text: 'up to date' });
		await f.controller.setPreset('m33');
		expect(f.state.preset).toBe('m33');
		expect(f.state.status?.text).toBe('up to date');
		expect(f.sent('build_status').at(-1)?.args).toEqual({ preset: 'm33' });
		expect(f.sent('build')).toEqual([]);
	});

	it('shows the helper’s refusals beside the status and turns the button off with the first of them', async () => {
		const f = new Fake();
		f.status({ kind: 'up_to_date', steps: null, text: 'up to date' }, ['the heavy lock is held (pid=1 …): heavy work waits for it (H-3)']);
		await f.controller.refresh();
		expect(f.state.refusals).toHaveLength(1);
		expect(whyNoBuild(f.state, true)).toMatch(/^the heavy lock is held/);
	});

	it('keeps an error of the helper as a notice', async () => {
		const f = new Fake();
		f.answers.set('build_status', new Error('the helper went away'));
		await f.controller.refresh();
		expect(f.state.notices).toEqual(['the helper went away']);
		expect(f.state.busy).toBeNull();
	});
});

describe('build and cancel', () => {
	it('sends the preset name and nothing else, on the click', async () => {
		const f = new Fake();
		await f.controller.refresh();
		f.answers.set('build', { ok: true, started: true, build: RUNNING });
		await f.controller.build();
		expect(f.sent('build')).toEqual([{ cmd: 'build', args: { preset: 'host-gz-quadfc' } }]);
		expect(buildOutcome(f.state.current)).toBe('running');
		expect(whyNoBuild(f.state, true)).toMatch(/one build at a time/);
	});

	it('shows every refusal of the helper and asks the status again', async () => {
		const f = new Fake();
		await f.controller.refresh();
		const reasons = ['the heavy lock is held (x): heavy work waits for it (H-3)', 'free memory 1 B is below the build budget 2 B (PLACEHOLDER)'];
		f.answers.set('build', new HelperError({ ok: false, error: reasons[0], refusals: reasons }, 'build'));
		await f.controller.build();
		expect(f.state.notices).toEqual(reasons);
		expect(f.state.busy).toBeNull();
		expect(buildOutcome(f.state.current)).toBe('none');
		expect(reasonsOf(new Error('x'))).toEqual(['x']);
	});

	it('follows the events: started, progress with the step, done; the library is read again and the status asked for again', async () => {
		const f = new Fake();
		await f.controller.refresh();
		f.controller.start();
		f.push({ event: 'build_started', preset: 'host-gz-quadfc', tree: 'host-gz-quadfc', describe: 'quad-L2-pass-1-gabc', started_utc: 't' });
		expect(f.state.current).toMatchObject({ running: true, describe: 'quad-L2-pass-1-gabc' });
		f.push({ event: 'build_progress', preset: 'host-gz-quadfc', line: '[2/12] Building x.o', step: 2, steps: 12 });
		expect(f.state.current).toMatchObject({ step: 2, steps: 12, tail: ['[2/12] Building x.o'] });
		f.push({ event: 'build_progress', preset: 'm33', line: 'another preset', step: 9, steps: 9 });
		expect(f.state.current?.step).toBe(2); // not this build's
		f.status({ kind: 'up_to_date', steps: null, text: 'up to date' });
		f.push({ event: 'build_done', preset: 'host-gz-quadfc', ok: true, cancelled: false, exit_status: 0, error: null, finished_utc: 'u', images: [{ hash: 'a'.repeat(64), name: 'marv_sil_quad_fc', kind: 'sil' }] });
		expect(buildOutcome(f.state.current)).toBe('ok');
		await f.settle();
		await f.settle();
		expect(f.refreshed).toBe(1);
		expect(f.state.status?.text).toBe('up to date');
		expect(f.state.current?.images).toHaveLength(1);
	});

	it('keeps at most TAIL_LINES progress lines and shows a cancelled build as cancelled, a failed one as failed', async () => {
		const f = new Fake();
		await f.controller.refresh();
		f.controller.start();
		f.push({ event: 'build_started', preset: 'host-gz-quadfc', tree: 't', describe: 'd', started_utc: 't' });
		for (let i = 0; i < TAIL_LINES + 5; i++) f.push({ event: 'build_progress', preset: 'host-gz-quadfc', line: `l${i}`, step: i, steps: 99 });
		expect(f.state.current?.tail).toHaveLength(TAIL_LINES);
		expect(f.state.current?.tail.at(-1)).toBe(`l${TAIL_LINES + 4}`);
		f.push({ event: 'build_done', preset: 'host-gz-quadfc', ok: false, cancelled: true, exit_status: -15, error: 'cancelled', images: [] });
		expect(buildOutcome(f.state.current)).toBe('cancelled');
		f.push({ event: 'build_started', preset: 'host-gz-quadfc', tree: 't', describe: 'd', started_utc: 't' });
		f.push({ event: 'build_done', preset: 'host-gz-quadfc', ok: false, cancelled: false, exit_status: 2, error: 'the build step exited with status 2', images: [] });
		expect(buildOutcome(f.state.current)).toBe('failed');
	});

	it('cancels through the helper and shows its refusal', async () => {
		const f = new Fake();
		await f.controller.cancel();
		expect(f.sent('build_cancel')).toHaveLength(1);
		f.answers.set('build_cancel', new HelperError({ ok: false, error: 'no build is running' }, 'build_cancel'));
		await f.controller.cancel();
		expect(f.state.notices).toEqual(['no build is running']);
	});
});

describe('statusText', () => {
	it('says that CMake re-checks its globs only when the helper saw its two steps; a count is shown as the helper gives it', () => {
		expect(statusText(null)).toBe('—');
		expect(statusText({ kind: 'up_to_date', steps: null, text: 'up to date', glob_recheck: true })).toBe('up to date (CMake re-checks its globs at build)');
		expect(statusText({ kind: 'up_to_date', steps: null, text: 'up to date' })).toBe('up to date');
		expect(statusText({ kind: 'steps', steps: 1, text: '1 step', glob_recheck: true })).toBe('1 step');
		expect(statusText({ kind: 'not_configured', steps: null, text: 'not configured' })).toBe('not configured');
	});
});

describe('whyNoBuild and recordOf', () => {
	const base: BuildState = { allowed: ALLOWED, preset: 'm33', status: null, refusals: [], current: null, notices: [], busy: null };
	it('lists the reasons in order and is null when nothing stops the click', () => {
		expect(whyNoBuild(base, false)).toBe('the helper is not connected');
		expect(whyNoBuild({ ...base, busy: 'status' }, true)).toBe('a command is in progress');
		expect(whyNoBuild({ ...base, preset: '' }, true)).toBe('pick a preset');
		expect(whyNoBuild(base, true)).toBeNull();
	});
	it('refuses something that is not a record', () => {
		expect(recordOf(null)).toBeNull();
		expect(recordOf({ preset: 'x' })).toBeNull();
		expect(recordOf(RUNNING)?.preset).toBe('host-gz-quadfc');
	});
});
