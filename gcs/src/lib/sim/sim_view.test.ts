import { describe, expect, it, vi } from 'vitest';
import type { SimSnapshot } from './channel';
import { SIM_TIMING } from './channel';
import { decodeSimRecord } from './record';
import { computeSimView, SimViewController } from './sim_view';
import { buildRecord, FLAG } from './testing/build';

const T0 = 50_000;

function snap(record: Parameters<typeof buildRecord>[0] | null, receivedAt = T0): SimSnapshot {
	return { record: record ? decodeSimRecord(buildRecord(record)) : null, receivedAt: record ? receivedAt : null, count: 12, bad: 0, lost: 0, hz: 19.5, connected: true };
}

const rows = (v: ReturnType<typeof computeSimView>, panel: string) => Object.fromEntries(v.panels.find((p) => p.id === panel)!.rows.map((r) => [r.key, r]));

describe('the SIM panels', () => {
	it('are absent without a record, and absent again once the records stopped for the staleness timeout', () => {
		expect(computeSimView(snap(null), T0)).toMatchObject({ present: false, panels: [] });
		const live = computeSimView(snap({}), T0 + SIM_TIMING.staleMs);
		expect(live.present).toBe(true);
		const gone = computeSimView(snap({}), T0 + SIM_TIMING.staleMs + 1); // the control: one millisecond later they are gone
		expect(gone.present).toBe(false);
		expect(gone.panels).toEqual([]);
	});

	it('are all titled SIM, and a SIL run has no HIL panel', () => {
		const v = computeSimView(snap({}), T0);
		expect(v.panels.map((p) => p.id)).toEqual(['run', 'truth', 'injected']);
		for (const p of v.panels) expect(p.title.startsWith('SIM')).toBe(true);
		expect(rows(v, 'run')['sim-run.kind']!.value).toMatch(/^SIL/);
	});

	it('show the truth with units and the injected inputs', () => {
		const v = computeSimView(snap({ q: [Math.SQRT1_2, Math.SQRT1_2, 0, 0], simTimeNs: 2_500_000_000 }), T0);
		const truth = rows(v, 'truth');
		expect(truth['sim-truth.pos.down']!.value).toBe('-3.50');
		expect(truth['sim-truth.height']!.value).toBe('3.50');
		expect(truth['sim-truth.roll']!.value).toBe('90.0');
		expect(truth['sim-truth.rotor.4']).toMatchObject({ value: '400.0', unit: 'rad/s' });
		expect(rows(v, 'run')['sim-run.time']).toMatchObject({ value: '2.50', unit: 's' });
		expect(rows(v, 'run')['sim-run.pad']!.value).toBe('waiting');
		const inj = rows(v, 'injected');
		expect(inj['sim-injected.cmd.4']).toMatchObject({ value: '2047', bar: 1 });
		expect(inj['sim-injected.applied.3']!.value).toBe('999');
		expect(inj['sim-injected.accel.z']!.unit).toBe('m/s²');
	});

	it('say a world with no plant-model IMU injects none, instead of showing zeros', () => {
		const v = computeSimView(snap({ flags: FLAG.launchPad }), T0);
		expect(rows(v, 'injected')['sim-injected.gyro.x']!.value).toBe('—');
		expect(v.panels.find((p) => p.id === 'injected')!.notes.join(' ')).toMatch(/injects no plant-model IMU/);
	});

	it('add the HIL board panel for a HIL run, with the arm state, the margin and the absent fields named', () => {
		const v = computeSimView(snap({ flags: FLAG.hil | FLAG.imuModel, hil: { arming: 10, marginMin: -12, reports: 4 } }), T0);
		expect(v.panels.map((p) => p.id)).toEqual(['run', 'truth', 'injected', 'hil']);
		const hil = rows(v, 'hil');
		expect(hil['sim-hil.arming']).toMatchObject({ value: 'armed', tone: 'bad' });
		expect(hil['sim-hil.min']).toMatchObject({ value: '-12', unit: 'µs', tone: 'bad' });
		expect(v.panels.find((p) => p.id === 'hil')!.notes.join(' ')).toMatch(/late commands/);
		const none = rows(computeSimView(snap({ flags: FLAG.hil, hil: { statusRecords: 0, reports: 0 } }), T0), 'hil');
		expect(none['sim-hil.arming']!.value).toBe('no status yet');
		expect(none['sim-hil.min']!.value).toBe('—');
	});
});

describe('the SIM view controller', () => {
	it('starts and stops the client once, and recomputes against the clock', () => {
		vi.useFakeTimers();
		let clock = T0;
		let snapshot = snap({}, T0);
		const client = { snapshot: () => snapshot, start: vi.fn(), stop: vi.fn() };
		const c = new SimViewController({ client, now: () => clock });
		const seen: boolean[] = [];
		c.present.subscribe((p) => seen.push(p));
		c.start();
		c.start();
		expect(client.start).toHaveBeenCalledTimes(1);
		expect(seen.at(-1)).toBe(true);
		clock += SIM_TIMING.staleMs + 1;
		vi.advanceTimersByTime(1000);
		expect(seen.at(-1)).toBe(false); // the records stopped: the panels are gone
		snapshot = snap({}, clock);
		vi.advanceTimersByTime(1000);
		expect(seen.at(-1)).toBe(true);
		c.stop();
		expect(client.stop).toHaveBeenCalledTimes(1);
		vi.useRealTimers();
	});
});
