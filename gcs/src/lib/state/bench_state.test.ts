// The Bench page's logic (GCS-first unit GX-10): the launch form becomes the helper's requests; the helper's refusals are shown as it words
// them; a form the helper would refuse is not sent; the verdict needs a mode; the motor-capable bench image is the helper's refusal to show.
import { describe, expect, it } from 'vitest';
import { get } from 'svelte/store';
import { HelperError, type HelperReply } from '$link/transport';
import type { HelperStatus, RunManifest } from './helper_api';
import { BENCH_KINDS, BenchController, DEFAULT_SECONDS, benchView, parseSeconds, reasonsOf, type Preflight } from './bench_state';

const RUN = '20261019T000000Z-hil-1';
// The helper's texts, as tools/gs/hil.py writes them.
const NO_ACK = 'the ESC battery acknowledgement is not ticked: unplug the ESC battery, then tick it';
const NO_PROBE = 'no debug probe: no USB device 2e8a:000c with serial X (plug the Debug Probe in, or fix MARV_PROBE_SERIAL)';
const MOTOR = "only after its props-off TB on the exact image and Luis's presence acknowledgement: bench-profile is a motor-capable bench image";

const preflight = (over: Partial<Preflight> = {}): Preflight => ({
	allowed: true,
	refusals: [],
	checks: [{ id: 'probe', state: 'ok', detail: 'debug probe X is on USB' }],
	port: '/dev/serial/by-id/usb-Raspberry_Pi_Pico_X-if00',
	seconds: 300,
	argv: null,
	...over
});

class Fake {
	calls: Array<{ cmd: string; args: Record<string, unknown> | undefined }> = [];
	refreshes = 0;
	answers = new Map<string, HelperReply | Error>();
	deps = {
		request: async (cmd: string, args?: Record<string, unknown>): Promise<HelperReply> => {
			this.calls.push({ cmd, args });
			const a = this.answers.get(cmd) ?? { ok: true };
			if (a instanceof Error) throw a;
			return a;
		},
		refresh: async () => {
			this.refreshes += 1;
		}
	};
	refuse(cmd: string, reply: Record<string, unknown>) {
		const full = { ok: false, ...reply } as HelperReply;
		this.answers.set(cmd, new HelperError(full, cmd));
	}
}

describe('parseSeconds', () => {
	it.each([
		['300', 300],
		[' 45.5 ', 45.5],
		['1e2', 100],
		['.5', 0.5]
	])('reads %j', (text, n) => expect(parseSeconds(text)).toBe(n));
	it.each(['', '0', '-3', 'abc', '1e999', '1 2', '0x10', 'NaN'])('does not read %j', (text) => expect(parseSeconds(text)).toBeNull());
});

describe('reasonsOf', () => {
	it('lists every reason of a refusal, else the one error text', () => {
		expect(reasonsOf(new HelperError({ ok: false, error: 'a', refusals: ['a', 'b'] }, 'hil_start'))).toEqual(['a', 'b']);
		expect(reasonsOf(new HelperError({ ok: false, error: 'only' }, 'flash_swd'))).toEqual(['only']);
		expect(reasonsOf(new Error('boom'))).toEqual(['boom']);
	});
});

describe('benchView', () => {
	const run = (kind: string, state: RunManifest['state'] = 'stopped'): RunManifest => ({ run_id: RUN, kind, state, exit_status: null, relaunches: 0, out_dir: '/runs/x/out' });
	const status = (extra: Partial<HelperStatus>): HelperStatus => ({ runs: { active: null, last: null }, ...extra }) as HelperStatus;

	it('shows the latest run only when it is a HIL run, and whether it is going', () => {
		expect(benchView(null).run).toBeNull();
		expect(benchView(status({ runs: { active: null, last: run('sil') } })).run).toBeNull();
		const ended = benchView(status({ runs: { active: null, last: run('hil') } }));
		expect(ended.run?.run_id).toBe(RUN);
		expect(ended.running).toBe(false);
		expect(ended.outDir).toBe('/runs/x/out');
		expect(benchView(status({ runs: { active: { id: RUN, kind: 'hil' }, last: run('hil', 'running') } })).running).toBe(true);
	});

	it('reads the hold, the saved verdict and the flash from the helper status', () => {
		const verdict = { schema: 1, run_id: RUN, mode: 'ANGLE' as const, text: 'ok', saved_utc: 'now', report: null };
		const v = benchView(status({ hil: { hold: true, hold_run: null, verdict, swd_flash: null } }));
		expect(v.hold).toBe(true);
		expect(v.verdict).toEqual(verdict);
		expect(benchView(null).hold).toBe(false);
	});
});

describe('BenchController: preflight and launch', () => {
	it('sends the acknowledgement and the seconds, keeps the helper\'s checklist and starts nothing', async () => {
		const f = new Fake();
		const pre = preflight({ allowed: false, refusals: [NO_ACK] });
		f.answers.set('hil_preflight', { ok: true, preflight: pre });
		const c = new BenchController(f.deps);
		await c.check();
		expect(f.calls).toEqual([{ cmd: 'hil_preflight', args: { esc_battery_unplugged: false, seconds: 300 } }]);
		expect(get(c.state).preflight).toEqual(pre);
		c.setAck(true);
		expect(get(c.state).preflight).toBeNull(); // the checklist was about the form as it was
		await c.check();
		expect(f.calls[1]).toEqual({ cmd: 'hil_preflight', args: { esc_battery_unplugged: true, seconds: 300 } });
	});

	it('does not send a seconds field the helper would refuse', async () => {
		const f = new Fake();
		const c = new BenchController(f.deps);
		expect(get(c.state).seconds).toBe(DEFAULT_SECONDS);
		c.setSeconds('soon');
		await c.check();
		await c.launch();
		expect(f.calls).toEqual([]);
		expect(get(c.state).notices).toEqual(['"soon" is not a positive number of seconds']);
	});

	it('shows every refusal of a launch verbatim with the checklist of the refusal, and the run status is read again', async () => {
		const f = new Fake();
		const pre = preflight({ allowed: false, refusals: [NO_PROBE, NO_ACK] });
		f.refuse('hil_start', { error: NO_PROBE, refusals: [NO_PROBE, NO_ACK], preflight: pre });
		const c = new BenchController(f.deps);
		await c.launch();
		expect(get(c.state).notices).toEqual([NO_PROBE, NO_ACK]);
		expect(get(c.state).preflight).toEqual(pre);
		expect(f.refreshes).toBe(1);
		// the control: with the helper's agreement the same form launches and shows no refusal
		f.answers.delete('hil_start');
		c.setAck(true);
		await c.launch();
		expect(f.calls.at(-1)).toEqual({ cmd: 'hil_start', args: { esc_battery_unplugged: true, seconds: 300 } });
		expect(get(c.state).notices).toEqual([]);
	});

	it('takes and releases the hold, and stops the run', async () => {
		const f = new Fake();
		const c = new BenchController(f.deps);
		await c.setHold(true);
		await c.setHold(false);
		await c.stop();
		expect(f.calls.map((x) => [x.cmd, x.args])).toEqual([['hil_hold', { on: true }], ['hil_hold', { on: false }], ['run_stop', undefined]]);
		expect(f.refreshes).toBe(3);
	});

	it('one command at a time', async () => {
		const f = new Fake();
		let release: () => void = () => undefined;
		f.deps.request = (cmd: string) => {
			f.calls.push({ cmd, args: undefined });
			return new Promise<HelperReply>((resolve) => (release = () => resolve({ ok: true })));
		};
		const c = new BenchController(f.deps);
		const first = c.setHold(true);
		await c.setHold(false);
		expect(f.calls).toHaveLength(1);
		release();
		await first;
		expect(get(c.state).busy).toBeNull();
	});
});

describe('BenchController: the verdict', () => {
	it('needs the mode before anything is sent, then sends the run, the mode and the text as typed', async () => {
		const f = new Fake();
		const c = new BenchController(f.deps);
		c.setVerdictText('Took off.\nHeld a hover.');
		await c.saveVerdict(RUN);
		expect(f.calls).toEqual([]);
		expect(get(c.state).verdictNotices).toEqual(['choose the flight mode flown: ANGLE or ACRO']);
		c.setMode('ANGLE');
		expect(get(c.state).verdictNotices).toEqual([]);
		await c.saveVerdict(RUN);
		expect(f.calls).toEqual([{ cmd: 'hil_verdict', args: { run_id: RUN, mode: 'ANGLE', text: 'Took off.\nHeld a hover.' } }]);
		expect(f.refreshes).toBe(1);
	});

	it('shows the helper\'s refusal of the verdict as it words it', async () => {
		const f = new Fake();
		f.refuse('hil_verdict', { error: `run ${RUN} already has a verdict (/x/verdict.json): it is evidence and is not overwritten` });
		const c = new BenchController(f.deps);
		c.setMode('ACRO');
		await c.saveVerdict(RUN);
		expect(get(c.state).verdictNotices).toEqual([`run ${RUN} already has a verdict (/x/verdict.json): it is evidence and is not overwritten`]);
	});
});

describe('BenchController: bench images', () => {
	it('lists the hil kind and the two motor-capable kinds, marking which are motor-capable', () => {
		expect(BENCH_KINDS.map((k) => [k.kind, k.motorCapable])).toEqual([['hil', false], ['bench-log', true], ['bench-profile', true]]);
	});

	it('sends the chosen kind and shows the helper\'s refusal of a motor-capable image verbatim; the hil kind is sent without one', async () => {
		const f = new Fake();
		f.refuse('flash_swd', { error: MOTOR });
		const c = new BenchController(f.deps);
		c.setKind('bench-profile');
		await c.flash();
		expect(f.calls).toEqual([{ cmd: 'flash_swd', args: { kind: 'bench-profile' } }]);
		expect(get(c.state).flashNotices).toEqual([MOTOR]);
		c.setKind('hil');
		expect(get(c.state).flashNotices).toEqual([]);
		f.answers.delete('flash_swd');
		await c.flash();
		expect(f.calls.at(-1)).toEqual({ cmd: 'flash_swd', args: { kind: 'hil' } });
		expect(get(c.state).flashNotices).toEqual([]);
	});
});
