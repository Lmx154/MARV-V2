// The Console page's logic (ground-segment unit GX-16, decision 0038): the board's console output as scrollback, the operator's lines sent
// as typed, and the helper's console slot opened and closed. The page is a terminal and nothing more: what is typed goes to the board
// unchanged (plus the line end the board's console parser reads, LF: fw/hal/target/decode/src/console.cpp), and this module knows no
// command of the board. The helper decides every refusal (tools/gs/console.py) and its text is shown as it is.
import { get, writable, type Readable } from 'svelte/store';
import { HelperError, type HelperEvent, type HelperReply } from '$link/transport';
import type { TextChannel } from './console_channel';

export type ConsoleMode = 'closed' | 'open' | 'lost';

export interface ConsoleView {
	state: ConsoleMode;
	/** Why it is not open (the helper's text), else null. */
	reason: string | null;
	device: string | null;
}

/** rx: a line the board printed. tx: a line the operator sent (what went up, as sent). note: the page says the console opened or closed. */
export type EntryKind = 'rx' | 'tx' | 'note';

export interface ConsoleEntry {
	id: number;
	kind: EntryKind;
	text: string;
}

export interface ConsoleState {
	view: ConsoleView;
	/** The console interfaces the helper lists under the by-id directory. */
	devices: string[];
	/** Why a console cannot be opened now (a HIL run, the hold or a flash owns the board); empty when it can. */
	blocked: string[];
	entries: ConsoleEntry[];
	/** The line the board has begun and not ended. */
	partial: string;
	/** The helper's refusal of the last command, verbatim. */
	error: string | null;
	busy: boolean;
	/** Whether the console channel (/ws/console) is connected. */
	connected: boolean;
}

/** The line end sent after a typed line: the board's console parser ends a line at LF. */
export const LINE_END = '\n';
/** PLACEHOLDER (harness): lines of scrollback kept; the board prints one stats line a second, so this is about half an hour. */
export const SCROLLBACK_MAX_LINES = 2000;
/** PLACEHOLDER (harness): the longest line the page holds before it breaks it (a device that prints no line end). */
export const LINE_MAX_CHARS = 4096;

export const CLOSED: ConsoleView = { state: 'closed', reason: null, device: null };

const text = (v: unknown): string | null => (typeof v === 'string' ? v : null);

/** The view a console_state event (or the status's `console` block) stands for; null for something that is not one. */
export function viewOf(e: Record<string, unknown>): ConsoleView | null {
	const state = e.state;
	if (state !== 'closed' && state !== 'open' && state !== 'lost') return null;
	return { state, reason: text(e.reason), device: text(e.device) };
}

const strings = (v: unknown): string[] => (Array.isArray(v) ? v.filter((x): x is string => typeof x === 'string') : []);

/**
 * Appends output to the scrollback. A line ends at LF (a CR before it is dropped, as is any other CR); what follows the last LF is the
 * partial line, kept for the next chunk, so a line split between chunks, or a CR LF split between them, is one line.
 */
export function feed(entries: readonly ConsoleEntry[], partial: string, chunk: string, nextId: number): { entries: ConsoleEntry[]; partial: string; nextId: number } {
	const parts = (partial + chunk).split('\n');
	let rest = parts.pop() as string;
	const out = entries.slice();
	for (const part of parts) out.push({ id: nextId++, kind: 'rx', text: part.replace(/\r/g, '') });
	while (rest.length > LINE_MAX_CHARS) {
		out.push({ id: nextId++, kind: 'rx', text: rest.slice(0, LINE_MAX_CHARS).replace(/\r/g, '') });
		rest = rest.slice(LINE_MAX_CHARS);
	}
	return { entries: out.length > SCROLLBACK_MAX_LINES ? out.slice(-SCROLLBACK_MAX_LINES) : out, partial: rest, nextId };
}

export interface ConsoleDeps {
	request(cmd: string, args?: Record<string, unknown>): Promise<HelperReply>;
	onEvent(handler: (event: HelperEvent) => void): () => void;
	channel: TextChannel;
}

export class ConsoleController {
	private readonly store = writable<ConsoleState>({ view: CLOSED, devices: [], blocked: [], entries: [], partial: '', error: null, busy: false, connected: false });
	readonly state: Readable<ConsoleState> = { subscribe: this.store.subscribe };
	private nextId = 1;
	private stops: Array<() => void> = [];

	constructor(private readonly deps: ConsoleDeps) {}

	/** Starts following the console channel and the helper's console_state events; returns what stops them. A second call does nothing. */
	start(): () => void {
		if (this.stops.length === 0) {
			this.stops = [
				this.deps.onEvent((e) => {
					if (e.event !== 'console_state') return;
					const view = viewOf(e);
					if (!view) return;
					this.store.update((s) => ({ ...s, view }));
					this.note(view.state === 'open' ? `console opened: ${view.device ?? ''}` : `console ${view.state}${view.reason ? `: ${view.reason}` : ''}`);
				}),
				this.deps.channel.onText((chunk) => this.output(chunk)),
				this.deps.channel.connected.subscribe((connected) => this.store.update((s) => ({ ...s, connected })))
			];
		}
		return () => {
			for (const stop of this.stops) stop();
			this.stops = [];
		};
	}

	/** The console block of the helper's status (devices, why it is blocked, the state): called when the status is read. */
	applyStatus(block: unknown): void {
		if (typeof block !== 'object' || block === null) return;
		const b = block as Record<string, unknown>;
		const view = viewOf(b);
		this.store.update((s) => ({ ...s, view: view ?? s.view, devices: strings(b.devices), blocked: strings(b.blocked) }));
	}

	private output(chunk: string): void {
		this.store.update((s) => {
			const fed = feed(s.entries, s.partial, chunk, this.nextId);
			this.nextId = fed.nextId;
			return { ...s, entries: fed.entries, partial: fed.partial };
		});
	}

	private note(line: string): void {
		this.append({ id: this.nextId++, kind: 'note', text: line });
	}

	private append(entry: ConsoleEntry): void {
		this.store.update((s) => ({ ...s, entries: [...s.entries, entry].slice(-SCROLLBACK_MAX_LINES) }));
	}

	async open(device: string): Promise<void> {
		await this.command('console_open', { device });
	}

	async close(): Promise<void> {
		await this.command('console_close');
	}

	private async command(cmd: string, args: Record<string, unknown> = {}): Promise<void> {
		if (get(this.store).busy) return;
		this.store.update((s) => ({ ...s, busy: true }));
		try {
			const reply = await this.deps.request(cmd, args);
			const block = typeof reply.console === 'object' && reply.console !== null ? (reply.console as Record<string, unknown>) : null;
			this.store.update((s) => ({ ...s, error: null, busy: false }));
			if (block) this.applyStatus(block);
		} catch (e) {
			const refusals = e instanceof HelperError ? strings(e.reply.refusals) : [];
			const message = e instanceof Error ? e.message : String(e);
			this.store.update((s) => ({ ...s, error: refusals.length > 1 ? refusals.join('; ') : message, busy: false }));
		}
	}

	/** Sends the typed text to the board as it is, plus the line end; false (nothing sent) unless the console is open and the channel is connected. */
	sendLine(line: string): boolean {
		const s = get(this.store);
		if (s.view.state !== 'open' || !s.connected) return false;
		if (!this.deps.channel.send(line + LINE_END)) return false;
		this.append({ id: this.nextId++, kind: 'tx', text: line });
		return true;
	}
}
