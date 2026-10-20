// The status bar without a browser: computeBar is pure, so every stale transition is a comparison of two instants; the
// ShellController is driven with fake timers and fake dependencies.
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import { get, writable } from 'svelte/store';
import type { HelperConnectionState, HelperEvent } from '$link/transport';
import type { SessionState } from '$link/session';
import type { VehicleIdentity } from '$link/vehicle';
import type { HelperStatus } from './helper_api';
import { computeBar, formatAge, SHELL_TIMING, ShellController, STATUS_POLL_MS, TICK_MS, type BarInputs, type Indicator, type IndicatorId } from './shell';
import { ROUTES, routeOf } from './nav';

const T0 = 1_000_000;
const VEHICLE = SHELL_TIMING.vehicleStaleMs;
const HELPER = SHELL_TIMING.helperStaleMs;

const session = (over: Partial<SessionState> = {}): SessionState => ({
	heartbeat: 'alive',
	vehicle: { systemId: 1, componentId: 1 },
	armed: false,
	systemStatus: 'standby',
	lastFrameAt: T0,
	lastHeartbeatAt: T0,
	rxBytes: 100,
	txBytes: 10,
	frames: 5,
	badFrames: 0,
	...over
});

const identity = (over: Partial<VehicleIdentity> = {}): VehicleIdentity => ({
	schemaHash: '0x0123456789abcdef',
	imageSha256: 'a'.repeat(64),
	configSha256: 'b'.repeat(64),
	validationReportId: null,
	configSource: 'factory',
	configState: 'ok',
	configRefusal: 'none',
	lastActivation: 'none' as VehicleIdentity['lastActivation'],
	stagedDiffers: false,
	...over
});

const status = (over: Partial<HelperStatus> = {}): HelperStatus => ({
	origin: 'http://127.0.0.1:1',
	run_dir: '/tmp/r',
	device: 'udp 127.0.0.1:14550',
	client: true,
	dropped_bytes: 0,
	sil: { running: false, pid: null, exit_status: null, udp_port: 14550, link: null },
	runs: { active: null, last: null },
	host: {
		loadavg: [0.5, 0.4, 0.3],
		cpus: 8,
		mem_total_bytes: 16 * 1024 ** 3,
		mem_available_bytes: 8 * 1024 ** 3,
		heavy_lock: { held: false, holder: null, path: '/tmp/marv-heavy.lock' }
	},
	autoconnect: { enabled: true, suspended: null, owner: null, prefer: 'board', reason: 'no SIL run is active' },
	validation: null,
	gcs_built: true,
	...over
});

const inputs = (over: Partial<BarInputs> = {}): BarInputs => ({
	helper: 'open',
	status: status(),
	statusAt: T0,
	session: session(),
	identity: identity(),
	...over
});

const pick = (bar: Indicator[], id: IndicatorId): Indicator => {
	const found = bar.find((i) => i.id === id);
	if (!found) throw new Error(`no ${id} indicator`);
	return found;
};
const field = (i: Indicator, name: string): string | undefined => i.fields.find((f) => f.name === name)?.value;
const staleOf = (bar: Indicator[]): Record<string, boolean> => Object.fromEntries(bar.map((i) => [i.id, i.stale]));

describe('computeBar: the six indicators', () => {
	it('lists helper, link, identity, arming, run and host, in that order', () => {
		expect(computeBar(inputs(), T0).map((i) => i.id)).toEqual(['helper', 'link', 'identity', 'arming', 'run', 'host']);
	});

	it('is fresh everywhere when everything was just heard', () => {
		expect(staleOf(computeBar(inputs(), T0))).toEqual({ helper: false, link: false, identity: false, arming: false, run: false, host: false });
	});

	it('is stale everywhere before anything was heard (the indicator is not constant)', () => {
		const bar = computeBar(inputs({ helper: 'closed', status: null, statusAt: null, session: null, identity: null }), T0);
		expect(staleOf(bar)).toEqual({ helper: true, link: true, identity: true, arming: true, run: true, host: true });
		expect(pick(bar, 'link').summary).toBe('no heartbeat');
		expect(pick(bar, 'identity').summary).toBe('not reported');
		expect(pick(bar, 'arming').summary).toBe('unknown');
	});
});

describe('computeBar: vehicle-sourced staleness (3 s)', () => {
	it('link and identity go stale just after the vehicle was last heard, not before', () => {
		const at = (now: number) => staleOf(computeBar(inputs(), now));
		expect(at(T0 + VEHICLE)).toMatchObject({ link: false, identity: false });
		expect(at(T0 + VEHICLE + 1)).toMatchObject({ link: true, identity: true });
	});

	it('come back as soon as a frame arrives', () => {
		const old = computeBar(inputs(), T0 + 2 * VEHICLE);
		expect(pick(old, 'link').stale).toBe(true);
		const again = computeBar(inputs({ session: session({ lastFrameAt: T0 + 2 * VEHICLE }) }), T0 + 2 * VEHICLE);
		expect(pick(again, 'link').stale).toBe(false);
		expect(pick(again, 'identity').stale).toBe(false);
	});

	it('arming follows the heartbeat, not any frame', () => {
		const now = T0 + VEHICLE + 1;
		const bar = computeBar(inputs({ session: session({ lastFrameAt: now, lastHeartbeatAt: T0 }) }), now);
		expect(pick(bar, 'link').stale).toBe(false);
		expect(pick(bar, 'arming').stale).toBe(true);
	});

	it('keeps the last value on screen when stale and does not touch the helper-sourced ones', () => {
		const bar = computeBar(inputs({ session: session({ armed: true, systemStatus: 'active' }) }), T0 + VEHICLE + 1);
		expect(pick(bar, 'arming').summary).toBe('armed');
		expect(pick(bar, 'arming').stale).toBe(true);
		expect(staleOf(bar)).toMatchObject({ helper: false, run: false, host: false });
	});

	it('shows the ages of the link', () => {
		const link = pick(computeBar(inputs({ session: session({ lastFrameAt: T0 - 400, lastHeartbeatAt: T0 - 1200 }) }), T0), 'link');
		expect(field(link, 'rx age')).toBe('0.4 s');
		expect(field(link, 'heartbeat age')).toBe('1.2 s');
		expect(formatAge(null)).toBe('never');
	});
});

describe('computeBar: helper-sourced staleness (10 s)', () => {
	it('helper, run and host go stale just after the status was read, not before', () => {
		const at = (now: number) => staleOf(computeBar(inputs(), now));
		expect(at(T0 + HELPER)).toMatchObject({ helper: false, run: false, host: false });
		expect(at(T0 + HELPER + 1)).toMatchObject({ helper: true, run: true, host: true });
	});

	it('do not move the vehicle-sourced indicators', () => {
		const now = T0 + HELPER + 1;
		const bar = computeBar(inputs({ session: session({ lastFrameAt: now, lastHeartbeatAt: now }) }), now);
		expect(staleOf(bar)).toMatchObject({ helper: true, link: false, identity: false, arming: false });
	});

	it('come back when a new status is read', () => {
		const now = T0 + 2 * HELPER;
		expect(pick(computeBar(inputs({ statusAt: now }), now), 'host').stale).toBe(false);
	});

	it('a closed WebSocket is shown as closed and bad, whatever the age of the status', () => {
		const helper = pick(computeBar(inputs({ helper: 'closed' }), T0), 'helper');
		expect(helper.summary).toBe('closed');
		expect(helper.tone).toBe('bad');
	});
});

describe('computeBar: contents', () => {
	it('the identity carries the hashes, the config source and the config state', () => {
		const i = pick(computeBar(inputs({ identity: identity({ configSource: 'active', validationReportId: 'c'.repeat(64) }) }), T0), 'identity');
		expect(i.summary).toBe(`image ${'a'.repeat(12)}`);
		expect(field(i, 'config')).toBe('b'.repeat(12));
		expect(field(i, 'source')).toBe('active');
		expect(field(i, 'state')).toBe('ok');
		expect(field(i, 'report')).toBe('c'.repeat(12));
	});

	it('the identity keeps the last activation and the full hashes (hover text), which the bar shortens', () => {
		const i = pick(
			computeBar(inputs({ identity: identity({ lastActivation: 'refused at boot', validationReportId: 'c'.repeat(64) }) }), T0),
			'identity'
		);
		expect(field(i, 'last activation')).toBe('refused at boot');
		expect(field(i, 'schema')).toBe('0x0123456789abcdef');
		expect(i.summaryTitle).toBe(`image SHA-256 ${'a'.repeat(64)}`);
		expect(i.fields.find((f) => f.name === 'config')?.title).toBe(`config SHA-256 ${'b'.repeat(64)}`);
		expect(i.fields.find((f) => f.name === 'report')?.title).toBe('c'.repeat(64));
		expect(i.fields.find((f) => f.name === 'report')?.title).not.toBe(field(i, 'report'));
	});

	it('a refused config is bad, says why and is the reason the vehicle does not arm', () => {
		const bar = computeBar(inputs({ identity: identity({ configState: 'refused', configRefusal: 'image binding' }) }), T0);
		expect(pick(bar, 'identity').tone).toBe('bad');
		expect(field(pick(bar, 'identity'), 'state')).toBe('refused: image binding');
		expect(field(pick(bar, 'arming'), 'reason')).toContain('image binding');
		expect(field(pick(bar, 'arming'), 'reason')).toContain('does not arm');
	});

	it('arming states with their reasons', () => {
		const arming = (s: SessionState | null) => pick(computeBar(inputs({ session: s }), T0), 'arming');
		expect(arming(session()).summary).toBe('disarmed');
		expect(field(arming(session()), 'reason')).toBe('vehicle standby');
		expect(arming(session({ armed: true, systemStatus: 'active' })).summary).toBe('armed');
		expect(arming(session({ armed: true, systemStatus: 'active' })).tone).toBe('bad');
		expect(field(arming(session({ armed: true, systemStatus: 'active' })), 'reason')).toBe('vehicle active');
		expect(arming(session({ armed: null, systemStatus: null })).summary).toBe('unknown');
		expect(field(arming(null), 'reason')).toBe('no heartbeat received');
	});

	it('the run: none, running with relaunches, ended with its exit status', () => {
		const run = (runs: HelperStatus['runs']) => pick(computeBar(inputs({ status: status({ runs }) }), T0), 'run');
		expect(run({ active: null, last: null }).summary).toBe('none');
		const last = { run_id: 'r-1', kind: 'sil', state: 'running' as const, exit_status: null, relaunches: 2 };
		const going = run({ active: { id: 'r-1', kind: 'sil' }, last });
		expect(going.summary).toBe('sil running');
		expect(field(going, 'relaunches')).toBe('2');
		const ended = run({ active: null, last: { ...last, state: 'exited', exit_status: 3 } });
		expect(ended.summary).toBe('sil exited');
		expect(field(ended, 'exit')).toBe('3');
	});

	it('the host: load, memory and the heavy lock with its holder', () => {
		const free = pick(computeBar(inputs(), T0), 'host');
		expect(free.summary).toBe('heavy lock free');
		expect(field(free, 'load')).toBe('0.50 / 8 cpus');
		expect(field(free, 'mem free')).toBe('8.0 / 16.0 GiB');
		const host = { ...status().host, heavy_lock: { held: true, holder: 'pid=7 user=luis', path: '/x' } };
		const held = pick(computeBar(inputs({ status: status({ host }) }), T0), 'host');
		expect(held.summary).toBe('heavy lock held');
		expect(held.tone).toBe('warn');
		expect(field(held, 'lock holder')).toBe('pid=7 user=luis');
	});

	it('the helper: autoconnect on, off and suspended; the link: the device', () => {
		const ac = (autoconnect: HelperStatus['autoconnect']) => field(pick(computeBar(inputs({ status: status({ autoconnect }) }), T0), 'helper'), 'autoconnect');
		expect(ac({ enabled: true, suspended: null, owner: null, prefer: 'board', reason: null })).toBe('on');
		expect(ac({ enabled: false, suspended: null, owner: null, prefer: null, reason: null })).toBe('off');
		expect(ac({ enabled: true, suspended: 'a HIL run owns the board', owner: null, prefer: null, reason: null })).toBe('suspended: a HIL run owns the board');
		const prefers = (autoconnect: HelperStatus['autoconnect']) => field(pick(computeBar(inputs({ status: status({ autoconnect }) }), T0), 'helper'), 'autoconnect prefers');
		expect(prefers({ enabled: true, suspended: null, owner: 'sil', prefer: 'sil', reason: 'SIL run r1 is active' })).toBe('sil: SIL run r1 is active');
		expect(prefers({ enabled: true, suspended: null, owner: null, prefer: 'board', reason: 'no SIL run is active' })).toBe('board: no SIL run is active');
		expect(prefers({ enabled: false, suspended: null, owner: null, prefer: null, reason: null })).toBe('—');
		expect(field(pick(computeBar(inputs(), T0), 'link'), 'device')).toBe('udp 127.0.0.1:14550');
		expect(field(pick(computeBar(inputs({ status: status({ device: null }) }), T0), 'link'), 'device')).toBe('none');
	});
});

describe('the pages', () => {
	it('has the connection, vehicle, simulation, radio, parameters and flash pages and the placeholders, each with a unit when not built', () => {
		expect(ROUTES.map((r) => r.path)).toEqual(['/', '/vehicle', '/simulation', '/radio', '/params', '/flash', '/bench', '/logs', '/flight-prep', '/sim']);
		expect(ROUTES.filter((r) => r.unit === null).map((r) => r.path)).toEqual(['/', '/vehicle', '/simulation', '/radio', '/params', '/flash', '/sim']);
		// The SIM page (GX-8) is listed only while a simulation publishes.
		expect(ROUTES.filter((r) => r.simOnly).map((r) => r.path)).toEqual(['/sim']);
	});

	it('finds a route from a pathname with a trailing slash or a .html suffix', () => {
		expect(routeOf('/')?.label).toBe('Connection');
		expect(routeOf('/params/')?.path).toBe('/params');
		expect(routeOf('/flight-prep.html')?.path).toBe('/flight-prep');
		expect(routeOf('/nowhere')).toBeNull();
	});
});

describe('ShellController', () => {
	beforeEach(() => {
		vi.useFakeTimers();
		vi.setSystemTime(T0);
	});
	afterEach(() => vi.useRealTimers());

	function make(readStatus: () => Promise<HelperStatus> = async () => status()) {
		const helperState = writable<HelperConnectionState>('open');
		const sessionStore = writable<SessionState>(session({ lastFrameAt: null, lastHeartbeatAt: null, armed: null, systemStatus: null, heartbeat: 'no heartbeat' }));
		const identityStore = writable<VehicleIdentity | null>(null);
		let handler: ((e: HelperEvent) => void) | null = null;
		const calls = { opened: 0, closed: 0, relaunched: 0 };
		const problems: Array<{ source: string; message: string }> = [];
		const reads = { n: 0 };
		const shell = new ShellController({
			readStatus: async () => {
				reads.n++;
				return readStatus();
			},
			onEvent: (h) => {
				handler = h;
				return () => (handler = null);
			},
			helperState,
			session: sessionStore,
			identity: identityStore,
			guard: async (source, what) => {
				try {
					await what();
				} catch (e) {
					problems.push({ source, message: e instanceof Error ? e.message : String(e) });
				}
			},
			deviceOpened: () => calls.opened++,
			deviceClosed: () => calls.closed++,
			relaunched: () => calls.relaunched++
		});
		return { shell, sessionStore, identityStore, helperState, calls, problems, reads, emit: (e: HelperEvent) => handler?.(e) };
	}

	it('reads the status when started and then every poll period', async () => {
		const { shell, reads } = make();
		shell.start();
		await vi.advanceTimersByTimeAsync(0);
		expect(reads.n).toBe(1);
		await vi.advanceTimersByTimeAsync(STATUS_POLL_MS);
		expect(reads.n).toBe(2);
		await vi.advanceTimersByTimeAsync(2 * STATUS_POLL_MS);
		expect(reads.n).toBe(4);
		shell.dispose();
		await vi.advanceTimersByTimeAsync(2 * STATUS_POLL_MS);
		expect(reads.n).toBe(4);
	});

	it('the tick moves a stale flag with no new data: link fresh, then stale 3 s after the last frame, then fresh again on a frame', async () => {
		const { shell, sessionStore } = make();
		shell.start();
		await vi.advanceTimersByTimeAsync(0);
		sessionStore.update((s) => ({ ...s, heartbeat: 'alive', armed: false, systemStatus: 'standby', lastFrameAt: Date.now(), lastHeartbeatAt: Date.now() }));
		expect(pick(get(shell.bar), 'link').stale).toBe(false);
		await vi.advanceTimersByTimeAsync(VEHICLE);
		expect(pick(get(shell.bar), 'link').stale).toBe(false);
		await vi.advanceTimersByTimeAsync(TICK_MS);
		expect(pick(get(shell.bar), 'link').stale).toBe(true);
		expect(pick(get(shell.bar), 'arming').stale).toBe(true);
		sessionStore.update((s) => ({ ...s, lastFrameAt: Date.now(), lastHeartbeatAt: Date.now() }));
		expect(pick(get(shell.bar), 'link').stale).toBe(false);
		expect(pick(get(shell.bar), 'arming').stale).toBe(false);
		shell.dispose();
	});

	it('a helper that stops answering turns the helper-sourced indicators stale after 10 s, and the failure is recorded', async () => {
		let fail = false;
		const { shell, problems } = make(async () => {
			if (fail) throw new Error('/api/status: 503');
			return status();
		});
		shell.start();
		await vi.advanceTimersByTimeAsync(0);
		expect(pick(get(shell.bar), 'host').stale).toBe(false);
		fail = true;
		await vi.advanceTimersByTimeAsync(HELPER);
		expect(pick(get(shell.bar), 'host').stale).toBe(false);
		await vi.advanceTimersByTimeAsync(STATUS_POLL_MS);
		expect(staleOf(get(shell.bar))).toMatchObject({ helper: true, run: true, host: true });
		expect(problems.at(-1)).toEqual({ source: 'status', message: '/api/status: 503' });
		expect(pick(get(shell.bar), 'host').summary).toBe('heavy lock free');
		fail = false;
		await vi.advanceTimersByTimeAsync(STATUS_POLL_MS);
		expect(staleOf(get(shell.bar))).toMatchObject({ helper: false, run: false, host: false });
		shell.dispose();
	});

	it('follows the helper connection and the identity', async () => {
		const { shell, helperState, identityStore } = make();
		shell.start();
		await vi.advanceTimersByTimeAsync(0);
		expect(pick(get(shell.bar), 'helper').summary).toBe('open');
		helperState.set('closed');
		expect(pick(get(shell.bar), 'helper').summary).toBe('closed');
		identityStore.set(identity());
		expect(pick(get(shell.bar), 'identity').summary).toContain('aaaaaaaaaaaa');
		shell.dispose();
	});

	it('device_opened forgets the old vehicle; device_closed forgets it and clears the device; each re-reads the status', async () => {
		const { shell, calls, reads, emit } = make();
		shell.start();
		await vi.advanceTimersByTimeAsync(0);
		emit({ event: 'device_opened', device: 'udp 127.0.0.1:1' });
		expect(calls).toEqual({ opened: 1, closed: 0, relaunched: 0 });
		expect(get(shell.device)).toBe('udp 127.0.0.1:1');
		expect(reads.n).toBe(2);
		emit({ event: 'device_closed', reason: 'user' });
		expect(calls).toEqual({ opened: 1, closed: 1, relaunched: 0 });
		await vi.advanceTimersByTimeAsync(0);
		expect(reads.n).toBe(3);
		shell.dispose();
	});

	it('sil_relaunched forgets the vehicle so that its identity is read again, and re-reads the status', async () => {
		const { shell, calls, reads, emit } = make();
		shell.start();
		await vi.advanceTimersByTimeAsync(0);
		emit({ event: 'sil_relaunched', run_id: 'r', relaunches: 1, pid: 5 });
		expect(calls).toEqual({ opened: 0, closed: 0, relaunched: 1 });
		expect(reads.n).toBe(2);
		shell.dispose();
	});

	it('keeps the latest autoconnect event', async () => {
		const { shell, emit } = make();
		shell.start();
		emit({ event: 'autoconnect', state: 'opened', source: 'sil', device: 'udp 127.0.0.1:2' });
		expect(get(shell.autoconnect)).toEqual({ state: 'opened', source: 'sil', device: 'udp 127.0.0.1:2', reason: null });
		emit({ event: 'autoconnect', state: 'suspended', reason: 'a flash is running' });
		expect(get(shell.autoconnect)).toEqual({ state: 'suspended', source: null, device: null, reason: 'a flash is running' });
		shell.dispose();
	});

	it('start twice does not poll twice', async () => {
		const { shell, reads } = make();
		shell.start();
		shell.start();
		await vi.advanceTimersByTimeAsync(0);
		expect(reads.n).toBe(1);
		shell.dispose();
	});
});
