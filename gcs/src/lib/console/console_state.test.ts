// The Console page's logic (ground-segment unit GX-16): the scrollback, the lines sent unchanged, the state from the helper's events and
// status, the helper's refusals shown verbatim, and the channel (reconnect, text only). Fakes only: no socket, no helper.
import { describe, expect, it, vi } from 'vitest';
import { readFileSync } from 'node:fs';
import { get, writable } from 'svelte/store';
import { HelperError, type HelperEvent, type HelperReply } from '$link/transport';
import { ConsoleChannel, consoleUrlOf, type ConsoleSocket } from './console_channel';
import { ConsoleController, CLOSED, LINE_END, LINE_MAX_CHARS, SCROLLBACK_MAX_LINES, feed, viewOf, type ConsoleEntry } from './console_state';

const texts = (entries: readonly ConsoleEntry[]): string[] => entries.map((e) => e.text);

describe('feed', () => {
	it('ends a line at LF, drops the CR before it and keeps the unfinished line for the next chunk', () => {
		let r = feed([], '', 'banner\r\nsecond', 1);
		expect(texts(r.entries)).toEqual(['banner']);
		expect(r.partial).toBe('second');
		r = feed(r.entries, r.partial, ' line\r', r.nextId);
		expect(texts(r.entries)).toEqual(['banner']);
		expect(r.partial).toBe('second line\r');
		r = feed(r.entries, r.partial, '\nthird\n', r.nextId);
		expect(texts(r.entries)).toEqual(['banner', 'second line', 'third']);
		expect(r.partial).toBe('');
		expect(new Set(r.entries.map((e) => e.id)).size).toBe(3);
	});

	it('keeps an empty line and breaks a line that never ends', () => {
		expect(texts(feed([], '', '\n\n', 1).entries)).toEqual(['', '']);
		const r = feed([], '', 'x'.repeat(LINE_MAX_CHARS * 2 + 5), 1);
		expect(r.entries.map((e) => e.text.length)).toEqual([LINE_MAX_CHARS, LINE_MAX_CHARS]);
		expect(r.partial.length).toBe(5);
	});

	it('keeps the last SCROLLBACK_MAX_LINES lines', () => {
		const chunk = Array.from({ length: SCROLLBACK_MAX_LINES + 10 }, (_, i) => `l${i}`).join('\n') + '\n';
		const r = feed([], '', chunk, 1);
		expect(r.entries.length).toBe(SCROLLBACK_MAX_LINES);
		expect(r.entries[0]!.text).toBe('l10');
		expect(r.entries.at(-1)!.text).toBe(`l${SCROLLBACK_MAX_LINES + 9}`);
	});
});

describe('viewOf', () => {
	it('reads the three states and nothing else', () => {
		expect(viewOf({ state: 'open', reason: null, device: '/d' })).toEqual({ state: 'open', reason: null, device: '/d' });
		expect(viewOf({ state: 'lost', reason: 'read error: Input/output error', device: null })).toMatchObject({ state: 'lost' });
		expect(viewOf({ state: 'weird' })).toBeNull();
		expect(viewOf({})).toBeNull();
	});
});

function fakeChannel(initial = true) {
	const handlers = new Set<(t: string) => void>();
	const connected = writable(initial);
	const sent: string[] = [];
	return {
		channel: {
			connected: { subscribe: connected.subscribe },
			start: () => undefined,
			stop: () => undefined,
			send: (t: string) => (get(connected) ? (sent.push(t), true) : false),
			onText: (h: (t: string) => void) => (handlers.add(h), () => handlers.delete(h))
		},
		sent,
		down: (t: string) => handlers.forEach((h) => h(t)),
		connected,
		handlers
	};
}

function make(replies: Record<string, HelperReply | Error> = {}, channel = fakeChannel()) {
	const calls: Array<{ cmd: string; args?: Record<string, unknown> }> = [];
	const events = new Set<(e: HelperEvent) => void>();
	const controller = new ConsoleController({
		request: async (cmd, args) => {
			calls.push({ cmd, args });
			const r = replies[cmd] ?? { ok: true };
			if (r instanceof Error) throw r;
			return r;
		},
		onEvent: (h) => (events.add(h), () => events.delete(h)),
		channel: channel.channel
	});
	return { controller, calls, channel, push: (e: HelperEvent) => events.forEach((h) => h(e)), events };
}

const OPEN = { event: 'console_state', state: 'open', reason: null, device: '/dev/serial/by-id/usb-X-if00' };

describe('the controller', () => {
	it('follows the channel and the console_state events, and stops following', () => {
		const { controller, channel, push, events } = make();
		const stop = controller.start();
		controller.start();
		expect(events.size).toBe(1);
		expect(channel.handlers.size).toBe(1);
		push({ event: 'device_opened', device: 'x' });
		expect(get(controller.state).view).toEqual(CLOSED);
		push(OPEN);
		channel.down('hello\r\nwor');
		channel.down('ld\r\n');
		const s = get(controller.state);
		expect(s.view).toEqual({ state: 'open', reason: null, device: OPEN.device });
		expect(s.entries.map((e) => [e.kind, e.text])).toEqual([['note', `console opened: ${OPEN.device}`], ['rx', 'hello'], ['rx', 'world']]);
		push({ event: 'console_state', state: 'lost', reason: 'read error: Input/output error', device: null });
		expect(get(controller.state).view).toMatchObject({ state: 'lost', reason: 'read error: Input/output error' });
		expect(get(controller.state).entries.at(-1)).toMatchObject({ kind: 'note', text: 'console lost: read error: Input/output error' });
		stop();
		expect(events.size).toBe(0);
		expect(channel.handlers.size).toBe(0);
	});

	it('sends the typed text unchanged plus the line end, and keeps what it sent in the scrollback', () => {
		const { controller, channel, push } = make();
		controller.start();
		push(OPEN);
		for (const typed of ['stop', '  odd   spacing \t', 'MIXED case 123 !?', '', 'café']) expect(controller.sendLine(typed)).toBe(true);
		expect(channel.sent).toEqual(['stop' + LINE_END, '  odd   spacing \t' + LINE_END, 'MIXED case 123 !?' + LINE_END, LINE_END, 'café' + LINE_END]);
		expect(LINE_END).toBe('\n');
		expect(get(controller.state).entries.filter((e) => e.kind === 'tx').map((e) => e.text)).toEqual(['stop', '  odd   spacing \t', 'MIXED case 123 !?', '', 'café']);
	});

	it('sends nothing unless the console is open and the channel is connected', () => {
		const { controller, channel, push } = make();
		controller.start();
		expect(controller.sendLine('x')).toBe(false); // closed
		push(OPEN);
		channel.connected.set(false);
		expect(controller.sendLine('x')).toBe(false); // channel down
		channel.connected.set(true);
		expect(controller.sendLine('x')).toBe(true); // control
		push({ event: 'console_state', state: 'closed', reason: 'closed by request', device: null });
		expect(controller.sendLine('y')).toBe(false);
		expect(channel.sent).toEqual(['x\n']);
	});

	it('opens and closes through the helper with the device as chosen and takes the state from the answer', async () => {
		const open = { ok: true, console: { state: 'open', reason: null, device: '/d-if00', devices: ['/d-if00'], blocked: [] } };
		const closed = { ok: true, console: { state: 'closed', reason: 'closed by request', device: null, devices: ['/d-if00'], blocked: [] } };
		const { controller, calls } = make({ console_open: open, console_close: closed });
		await controller.open('/d-if00');
		expect(calls).toEqual([{ cmd: 'console_open', args: { device: '/d-if00' } }]);
		expect(get(controller.state)).toMatchObject({ view: { state: 'open', device: '/d-if00' }, devices: ['/d-if00'], error: null, busy: false });
		await controller.close();
		expect(calls[1]).toEqual({ cmd: 'console_close', args: {} });
		expect(get(controller.state).view.state).toBe('closed');
	});

	it("shows the helper's refusal as it words it, and every reason of a list", async () => {
		const one = make({ console_open: new Error('a HIL run (r1) owns the board port: stop the run first (H-1)') });
		await one.controller.open('/d-if00');
		expect(get(one.controller.state).error).toBe('a HIL run (r1) owns the board port: stop the run first (H-1)');
		expect(get(one.controller.state)).toMatchObject({ view: CLOSED, busy: false });
		const reply = { ok: false, error: 'first', refusals: ['first', 'second'] };
		const many = make({ console_open: new HelperError(reply, 'console_open') });
		await many.controller.open('/d-if00');
		expect(get(many.controller.state).error).toBe('first; second');
		const ok = make({});
		await ok.controller.open('/d-if00'); // control: no refusal, no error
		expect(get(ok.controller.state).error).toBeNull();
	});

	it('takes the devices, the blockers and the state from the status block, and ignores something else', () => {
		const { controller } = make();
		controller.applyStatus({ state: 'closed', reason: null, device: null, devices: ['/a-if00', '/b-if00'], blocked: ['the board is held for a HIL run (hil_hold): release the hold first'] });
		expect(get(controller.state)).toMatchObject({ devices: ['/a-if00', '/b-if00'], blocked: ['the board is held for a HIL run (hil_hold): release the hold first'] });
		controller.applyStatus(undefined);
		controller.applyStatus({ state: 'open', device: '/a-if00', devices: [], blocked: [] });
		expect(get(controller.state)).toMatchObject({ view: { state: 'open', device: '/a-if00' }, blocked: [] });
		expect(get(controller.state).entries).toEqual([]); // a status read adds no scrollback
	});

	it('answers one command at a time', async () => {
		let release!: () => void;
		const gate = new Promise<void>((r) => (release = r));
		const calls: string[] = [];
		const controller = new ConsoleController({
			request: async (cmd) => (calls.push(cmd), await gate, { ok: true }),
			onEvent: () => () => undefined,
			channel: fakeChannel().channel
		});
		const first = controller.open('/d-if00');
		await controller.open('/d-if00');
		release();
		await first;
		expect(calls).toEqual(['console_open']);
	});
});

class FakeSocket implements ConsoleSocket {
	readyState = 0;
	sent: string[] = [];
	onopen: ConsoleSocket['onopen'] = null;
	onclose: ConsoleSocket['onclose'] = null;
	onerror: ConsoleSocket['onerror'] = null;
	onmessage: ConsoleSocket['onmessage'] = null;
	send(data: string): void {
		this.sent.push(data);
	}
	close(): void {
		this.readyState = 3;
	}
	open(): void {
		this.readyState = 1;
		this.onopen?.({});
	}
	drop(): void {
		this.readyState = 3;
		this.onclose?.({});
	}
}

describe('the channel', () => {
	it('is the helper\'s /ws/console with the same host and token', () => {
		expect(consoleUrlOf('ws://127.0.0.1:8765/ws?token=abc')).toBe('ws://127.0.0.1:8765/ws/console?token=abc');
	});

	it('hands text frames up and sends text as it is, only while open; ignores a binary frame', () => {
		const sockets: FakeSocket[] = [];
		const ch = new ConsoleChannel({ url: 'ws://x/ws/console', factory: () => (sockets.push(new FakeSocket()), sockets.at(-1)!) });
		const got: string[] = [];
		ch.onText((t) => got.push(t));
		ch.start();
		ch.start();
		expect(sockets.length).toBe(1);
		expect(ch.send('early\n')).toBe(false);
		sockets[0]!.open();
		expect(get(ch.connected)).toBe(true);
		sockets[0]!.onmessage?.({ data: 'out put\r\n' });
		sockets[0]!.onmessage?.({ data: new ArrayBuffer(4) });
		expect(got).toEqual(['out put\r\n']);
		expect(ch.send('  stop \n')).toBe(true);
		expect(sockets[0]!.sent).toEqual(['  stop \n']);
		ch.stop();
		expect(get(ch.connected)).toBe(false);
		expect(ch.send('late\n')).toBe(false);
	});

	it('connects again after the socket closes, until stopped', () => {
		vi.useFakeTimers();
		try {
			const sockets: FakeSocket[] = [];
			const ch = new ConsoleChannel({ url: 'ws://x/ws/console', retryMs: 100, factory: () => (sockets.push(new FakeSocket()), sockets.at(-1)!) });
			ch.start();
			sockets[0]!.open();
			sockets[0]!.drop();
			expect(get(ch.connected)).toBe(false);
			vi.advanceTimersByTime(99);
			expect(sockets.length).toBe(1);
			vi.advanceTimersByTime(1);
			expect(sockets.length).toBe(2);
			sockets[1]!.open();
			expect(get(ch.connected)).toBe(true);
			ch.stop();
			sockets[1]!.drop();
			vi.advanceTimersByTime(1000);
			expect(sockets.length).toBe(2);
		} finally {
			vi.useRealTimers();
		}
	});
});

describe('the page is a terminal (decision 0038)', () => {
	it('names no motor command, button or shortcut of its own', () => {
		const dir = new URL('.', import.meta.url);
		for (const file of [new URL('console_state.ts', dir), new URL('console_channel.ts', dir), new URL('../../routes/console/+page.svelte', dir)]) {
			const source = readFileSync(file, 'utf8');
			expect(source.toLowerCase(), file.pathname).not.toMatch(/motor|props|dshot|throttle/);
		}
	});
});
