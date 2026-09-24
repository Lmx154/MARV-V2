import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import fixture from '$lib/fixtures/schema.json';
import { MockFc } from './mock';
import { normalizeSchema, parseServer } from './protocol';
import { age, busyBadge, linkHolder, parseResources, terminateBlocked, terminateQuestion } from './resources';
import type { ResourceHolder, RigResource, Schema, ServerMsg } from './types';

const bridge: ResourceHolder = { pid: 4321, name: 'marv_bridge', cmdline: 'marv_bridge --port /dev/ttyACM1', started: 1000, self: false, child_of_launcher: false };
const fc: RigResource = { id: 'fc_usb', label: 'Flight controller USB', path: '/dev/ttyACM1', holders: [bridge] };

describe('resources messages', () => {
	it('parses the list loosely', () => {
		const m = parseServer(
			JSON.stringify({
				type: 'resources',
				resources: [
					{ id: 'fc_usb', label: 'Flight controller USB', path: '/dev/ttyACM1', holders: [{ ...bridge, extra: 1 }, { name: 'no pid' }] },
					{ id: 'sim', path: null, holders: 'junk' },
					{ label: 'no id' },
					'junk'
				]
			})
		);
		expect(m).toEqual({
			type: 'resources',
			resources: [fc, { id: 'sim', label: 'sim', path: null, holders: [] }]
		});
		expect(parseResources(null)).toEqual([]);
		expect(parseResources([{ id: 'gcs', holders: [{ pid: 7, self: 'yes', started: 'x' }] }])[0].holders[0]).toEqual({
			pid: 7,
			name: '',
			cmdline: '',
			started: null,
			self: false,
			child_of_launcher: false
		});
	});

	it('reads the link holder and makes the busy badge', () => {
		const m = parseServer('{"type":"link","mode":"auto","connected":false,"via":"busy","target":"/dev/serial/by-id/x","holder":{"name":"marv_bridge","pid":4321}}');
		expect(m?.type === 'link' && m.link.holder).toEqual({ name: 'marv_bridge', pid: 4321 });
		expect(busyBadge(m?.type === 'link' ? m.link : null)).toBe('FC busy — held by marv_bridge (pid 4321)');
		expect(busyBadge({ mode: 'auto', connected: false, via: 'busy', target: null, holder: linkHolder({ pid: 0 }) })).toBe('FC busy — held by another process');
		expect(busyBadge({ mode: 'auto', connected: false, via: 'busy', target: null })).toBe('FC busy — held by another process');
		expect(busyBadge({ mode: 'auto', connected: true, via: 'usb', target: '/dev/ttyACM1', holder: null })).toBeNull();
		expect(busyBadge(null)).toBeNull();
		expect(linkHolder('x')).toBeNull();
	});
});

describe('terminate control', () => {
	it('is disabled for the backend itself, while closed, and while one is pending', () => {
		expect(terminateBlocked(bridge, true, null)).toBeNull();
		expect(terminateBlocked({ ...bridge, self: true }, true, null)).toMatch(/backend itself/);
		expect(terminateBlocked(bridge, false, null)).toMatch(/not connected/);
		expect(terminateBlocked(bridge, true, 99)).toMatch(/pid 99/);
	});

	it('names the process and what it holds in the confirm dialog', () => {
		const q = terminateQuestion(fc, bridge);
		expect(q).toContain('marv_bridge (pid 4321)');
		expect(q).toContain('Flight controller USB /dev/ttyACM1');
		expect(q).toContain('SIGKILL');
		expect(terminateQuestion(fc, { ...bridge, child_of_launcher: true })).toMatch(/^Stop the sim/);
	});

	it('formats ages', () => {
		expect(age(null, 0)).toBe('—');
		expect(age(1000, 1030_000)).toBe('30 s');
		expect(age(1000, (1000 + 3600 + 720) * 1000)).toBe('1 h 12 min');
		expect(age(1000, (1000 + 2 * 86400 + 3 * 3600) * 1000)).toBe('2 d 3 h');
	});
});

describe('mock resources', () => {
	let msgs: ServerMsg[];
	let fcMock: MockFc;
	beforeEach(() => {
		vi.useFakeTimers();
		msgs = [];
		fcMock = new MockFc(normalizeSchema(fixture as Schema), '');
		fcMock.onmessage = (ev) => {
			const m = parseServer(ev.data);
			if (m) msgs.push(m);
		};
	});
	afterEach(() => {
		fcMock.close();
		vi.useRealTimers();
	});
	const last = (): Extract<ServerMsg, { type: 'resources' }> | undefined =>
		msgs.filter((m): m is Extract<ServerMsg, { type: 'resources' }> => m.type === 'resources').at(-1);

	it('lists holders, refuses itself, and terminates a holder', () => {
		fcMock.send(JSON.stringify({ type: 'resources_request' }));
		vi.advanceTimersByTime(100);
		const list = last()!.resources;
		const self = list.flatMap((r) => r.holders).find((h) => h.self)!;
		const other = list.flatMap((r) => r.holders).find((h) => !h.self)!;
		expect(self && other).toBeTruthy();
		fcMock.send(JSON.stringify({ type: 'terminate', pid: self.pid }));
		vi.advanceTimersByTime(100);
		expect(msgs.filter((m) => m.type === 'error').at(-1)).toMatchObject({ type: 'error', request: 'terminate', error: expect.stringMatching(/this marv_gcs/) });
		fcMock.send(JSON.stringify({ type: 'terminate', pid: other.pid }));
		vi.advanceTimersByTime(1000);
		expect(last()!.resources.flatMap((r) => r.holders).some((h) => h.pid === other.pid)).toBe(false);
	});
});
