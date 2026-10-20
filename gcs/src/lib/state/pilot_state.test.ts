// The pilot bridge's GCS side (ground-segment unit GX-9): the pilot_state events become the view, the commands go to the helper only,
// the axis map is checked before it is sent, and the form's Pilot flag becomes the run option.
import { describe, expect, it } from 'vitest';
import { get } from 'svelte/store';
import type { HelperEvent, HelperReply } from '$link/transport';
import { HelperError } from '$link/transport';
import { PilotController, CLOSED, parseAxisMap, viewOf } from './pilot_state';
import { buildRequest, type SimForm } from './sim_state';

const CHANNELS = Array.from({ length: 16 }, (_, i) => 992 + i);

function make(replies: Record<string, HelperReply | Error>) {
	const sent: Array<{ cmd: string; args?: Record<string, unknown> }> = [];
	const handlers = new Set<(e: HelperEvent) => void>();
	const controller = new PilotController({
		request: async (cmd, args) => {
			sent.push({ cmd, args });
			const r = replies[cmd] ?? { ok: true };
			if (r instanceof Error) throw r;
			return r;
		},
		onEvent: (h) => {
			handlers.add(h);
			return () => handlers.delete(h);
		}
	});
	return { controller, sent, push: (e: HelperEvent) => handlers.forEach((h) => h(e)), handlers };
}

describe('viewOf', () => {
	it('reads an open state with its channels', () => {
		expect(viewOf({ state: 'open', reason: null, device: '/dev/input/js0', frames_sent: 12, send_errors: 0, channels: CHANNELS })).toEqual({
			state: 'open', reason: null, device: '/dev/input/js0', framesSent: 12, sendErrors: 0, channels: CHANNELS
		});
	});
	it('does not read an unknown state or bad channels', () => {
		expect(viewOf({ state: 'weird' })).toBeNull();
		expect(viewOf({ state: 'open', channels: ['a'] })?.channels).toBeNull();
	});
});

describe('the events', () => {
	it('pilot_state events set the view, other events do not, and stopping stops following', () => {
		const { controller, push, handlers } = make({});
		const stop = controller.start();
		controller.start();
		expect(handlers.size).toBe(1);
		push({ event: 'device_opened', device: 'x' });
		expect(get(controller.state).view).toEqual(CLOSED);
		push({ event: 'pilot_state', state: 'open', device: '/dev/input/js0', frames_sent: 3, channels: CHANNELS });
		expect(get(controller.state).view.state).toBe('open');
		expect(get(controller.state).view.channels).toEqual(CHANNELS);
		push({ event: 'pilot_state', state: 'lost', reason: 'end of file', channels: null });
		expect(get(controller.state).view).toMatchObject({ state: 'lost', reason: 'end of file', channels: null });
		stop();
		expect(handlers.size).toBe(0);
	});
});

describe('the commands', () => {
	it('open sends the device (and the axis map only when given); the helper\'s refusal is shown verbatim', async () => {
		const { controller, sent } = make({ pilot_open: { ok: true, pilot: { state: 'open', device: '/dev/input/js0', frames_sent: 0, channels: CHANNELS } } });
		await controller.open('/dev/input/js0');
		expect(sent).toEqual([{ cmd: 'pilot_open', args: { device: '/dev/input/js0' } }]);
		expect(get(controller.state).view.state).toBe('open');
		const refusing = make({ pilot_open: new HelperError({ ok: false, error: 'no SIL run with the pilot option is going' }, 'pilot_open') });
		await refusing.controller.open('/dev/input/js0');
		expect(get(refusing.controller.state).error).toBe('no SIL run with the pilot option is going');
		expect(get(refusing.controller.state).view.state).toBe('closed');
	});

	it('an axis map that is not 16 axes is not sent', async () => {
		const { controller, sent } = make({});
		await controller.open('/dev/input/js0', '1 2 3');
		expect(sent).toEqual([]);
		expect(get(controller.state).error).toMatch(/16 axes/);
		await controller.open('/dev/input/js0', `- ${'7 '.repeat(15)}`);
		expect(sent[0]?.args?.axis_map).toEqual([null, ...Array(15).fill(7)]);
	});

	it('close goes to the helper and loadDevices reads the devices and the status', async () => {
		const { controller, sent } = make({
			pilot_devices: { ok: true, devices: ['/dev/input/js0'] },
			status: { ok: true, status: { pilot: { state: 'open', device: '/dev/input/js0', frames_sent: 5, channels: CHANNELS } } },
			pilot_close: { ok: true, pilot: { state: 'closed' } }
		});
		await controller.loadDevices();
		expect(get(controller.state).devices).toEqual(['/dev/input/js0']);
		expect(get(controller.state).view.state).toBe('open');
		await controller.close();
		expect(sent.map((s) => s.cmd)).toEqual(['pilot_devices', 'status', 'pilot_close']);
		expect(get(controller.state).view.state).toBe('closed');
	});
});

describe('parseAxisMap', () => {
	it('blank is the default layout', () => expect(parseAxisMap('  ')).toEqual({ axisMap: null }));
	it('refuses a bad axis', () => expect(parseAxisMap(`${'1 '.repeat(15)}x`)).toEqual({ error: '"x" is not an axis number (0 to 255) or "-"' }));
});

describe('the run request', () => {
	const base: SimForm = { card: '', overrides: [], gui: false };
	it('carries pilot only when it is on', () => {
		expect(buildRequest(base).request).toEqual({ kind: 'sil', gui: false });
		expect(buildRequest({ ...base, pilot: false }).request).toEqual({ kind: 'sil', gui: false });
		expect(buildRequest({ ...base, pilot: true }).request).toEqual({ kind: 'sil', gui: false, pilot: true });
	});
});
