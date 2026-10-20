// The SIL pilot bridge in the GCS (ground-segment unit GX-9): open and close the RadioMaster's USB joystick and show the state the
// helper pushes (tools/gs/pilot.py: pilot_state events, at most 30 Hz). The helper reads the joystick and sends the frames to the
// simulator; the browser touches no device (it never reads the Gamepad API). Manual control is not on the vehicle link (PB-1): this
// controller speaks to the helper only, and nothing here reaches the vehicle client.
import { get, writable, type Readable } from 'svelte/store';
import type { HelperEvent, HelperReply } from '$link/transport';

export type PilotMode = 'closed' | 'open' | 'lost';

export interface PilotView {
	state: PilotMode;
	/** Why it is not open (the helper's text), else null. */
	reason: string | null;
	device: string | null;
	framesSent: number;
	sendErrors: number;
	/** The 16 raw 11-bit channel values being sent while open, else null. */
	channels: number[] | null;
}

export interface PilotState {
	view: PilotView;
	/** The joystick devices the helper lists; null until read. */
	devices: string[] | null;
	/** The helper's refusal of the last command, verbatim. */
	error: string | null;
	busy: boolean;
}

export const CLOSED: PilotView = { state: 'closed', reason: null, device: null, framesSent: 0, sendErrors: 0, channels: null };

const text = (v: unknown): string | null => (typeof v === 'string' ? v : null);
const count = (v: unknown): number => (typeof v === 'number' && Number.isFinite(v) ? v : 0);

/** The view a pilot_state event (or the status's `pilot` block) stands for; null for something that is not one. */
export function viewOf(e: Record<string, unknown>): PilotView | null {
	const state = e.state;
	if (state !== 'closed' && state !== 'open' && state !== 'lost') return null;
	const channels = Array.isArray(e.channels) && e.channels.every((c) => typeof c === 'number') ? (e.channels as number[]) : null;
	return { state, reason: text(e.reason), device: text(e.device), framesSent: count(e.frames_sent), sendErrors: count(e.send_errors), channels };
}

/** "0 1 2 ... 15" or "- 7 ..." as the helper's axis_map: 16 axis numbers, '-' for none; blank is the default layout (channel n is axis n-1). */
export function parseAxisMap(textValue: string): { axisMap: Array<number | null> | null } | { error: string } {
	const parts = textValue.trim().split(/[\s,]+/).filter((p) => p !== '');
	if (parts.length === 0) return { axisMap: null };
	if (parts.length !== 16) return { error: `the axis map lists 16 axes (one per channel), not ${parts.length}` };
	const axisMap: Array<number | null> = [];
	for (const p of parts) {
		if (p === '-') axisMap.push(null);
		else if (/^[0-9]+$/.test(p) && Number(p) < 256) axisMap.push(Number(p));
		else return { error: `"${p}" is not an axis number (0 to 255) or "-"` };
	}
	return { axisMap };
}

export interface PilotDeps {
	request(cmd: string, args?: Record<string, unknown>): Promise<HelperReply>;
	onEvent(handler: (event: HelperEvent) => void): () => void;
}

export class PilotController {
	private readonly store = writable<PilotState>({ view: CLOSED, devices: null, error: null, busy: false });
	readonly state: Readable<PilotState> = { subscribe: this.store.subscribe };
	private off: (() => void) | null = null;

	constructor(private readonly deps: PilotDeps) {}

	/** Starts following the helper's pilot_state events; returns what stops it. A second call does nothing. */
	start(): () => void {
		if (!this.off) {
			this.off = this.deps.onEvent((e) => {
				if (e.event !== 'pilot_state') return;
				const view = viewOf(e);
				if (view) this.store.update((s) => ({ ...s, view }));
			});
		}
		return () => {
			this.off?.();
			this.off = null;
		};
	}

	/** The joystick devices and, for a page opened while the bridge is going, the bridge's state from the helper's status. */
	async loadDevices(): Promise<void> {
		try {
			const reply = await this.deps.request('pilot_devices');
			const devices = Array.isArray(reply.devices) ? reply.devices.filter((d): d is string => typeof d === 'string') : [];
			const status = (await this.deps.request('status')).status as { pilot?: Record<string, unknown> } | undefined;
			const view = status?.pilot ? viewOf(status.pilot) : null;
			this.store.update((s) => ({ ...s, devices, view: view ?? s.view, error: null }));
		} catch (e) {
			this.store.update((s) => ({ ...s, devices: null, error: e instanceof Error ? e.message : String(e) }));
		}
	}

	async open(device: string, axisMapText = ''): Promise<void> {
		if (get(this.store).busy) return;
		const parsed = parseAxisMap(axisMapText);
		if ('error' in parsed) {
			this.store.update((s) => ({ ...s, error: parsed.error }));
			return;
		}
		await this.command('pilot_open', { device, ...(parsed.axisMap ? { axis_map: parsed.axisMap } : {}) });
	}

	async close(): Promise<void> {
		if (get(this.store).busy) return;
		await this.command('pilot_close');
	}

	private async command(cmd: string, args: Record<string, unknown> = {}): Promise<void> {
		this.store.update((s) => ({ ...s, busy: true }));
		try {
			const reply = await this.deps.request(cmd, args);
			const view = typeof reply.pilot === 'object' && reply.pilot !== null ? viewOf(reply.pilot as Record<string, unknown>) : null;
			this.store.update((s) => ({ ...s, view: view ?? s.view, error: null, busy: false }));
		} catch (e) {
			this.store.update((s) => ({ ...s, error: e instanceof Error ? e.message : String(e), busy: false }));
		}
	}
}
