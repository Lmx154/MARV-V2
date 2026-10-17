// The link layer of the GCS: bytes from a transport -> the codec's parser -> a typed message bus; messages out through the
// codec; the heartbeat watchdog. Everything above this file (the vehicle client in vehicle.ts and the pages) works with the
// typed bus or with the domain types of vehicle.ts. The default codec is the MAVLink codec; any other codec that passes the
// conformance suite can be given.
import { derived, get, writable, type Readable } from 'svelte/store';
import type { Codec, Envelope, FrameParser } from './codec';
import type { Message } from './generated/messages';
import { mavlinkCodec } from './mavlink';
import type { ByteLink } from './transport';

type Kind = Message['kind'];
export type MessageOf<K extends Kind> = Extract<Message, { kind: K }>;
export type Handler<K extends Kind> = (message: MessageOf<K>, envelope: Envelope) => void;

/** The typed message bus: a subscription per message kind. A handler that throws does not stop the others. */
export class MessageBus {
	private readonly byKind = new Map<Kind, Set<(m: Message, e: Envelope) => void>>();

	on<K extends Kind>(kind: K, handler: Handler<K>): () => void {
		let set = this.byKind.get(kind);
		if (!set) this.byKind.set(kind, (set = new Set()));
		const wrapped = handler as unknown as (m: Message, e: Envelope) => void;
		set.add(wrapped);
		return () => set.delete(wrapped);
	}

	emit(message: Message, envelope: Envelope): void {
		for (const h of [...(this.byKind.get(message.kind) ?? [])]) {
			try {
				h(message, envelope);
			} catch (e) {
				console.error('message handler failed', e);
			}
		}
	}
}

/** The GCS's own address on the link: MAVLink convention (system 255 is a ground station, component 190 = MAV_COMP_ID_MISSIONPLANNER). */
export const GCS_SYSTEM_ID = 255;
export const GCS_COMPONENT_ID = 190;
/** MAV_MODE_FLAG_SAFETY_ARMED (common.xml). */
const MODE_FLAG_SAFETY_ARMED = 128;
/** PLACEHOLDER: the display timeout of "no heartbeat" is 3 missed heartbeats at the MAVLink convention of 1 Hz; it is replaced by the GS-6 derivation (heartbeat rate and false-alarm budget) when the vehicle sets its rate. */
export const HEARTBEAT_TIMEOUT_MS = 3000;

export type HeartbeatState = 'no heartbeat' | 'alive';

export interface SessionState {
	heartbeat: HeartbeatState;
	/** The vehicle's address, learnt from its first heartbeat; null until then. */
	vehicle: { systemId: number; componentId: number } | null;
	/** From the heartbeat's base mode; null until a heartbeat arrived. */
	armed: boolean | null;
	rxBytes: number;
	txBytes: number;
	frames: number;
	badFrames: number;
}

export interface SessionOptions {
	codec?: Codec;
	heartbeatTimeoutMs?: number;
}

export class LinkSession {
	readonly bus = new MessageBus();
	private readonly codec: Codec;
	private readonly parser: FrameParser;
	private readonly timeoutMs: number;
	private readonly stateStore = writable<SessionState>(initialState());
	private timer: ReturnType<typeof setTimeout> | null = null;
	private sequence = 0;
	private unsubscribe: (() => void) | null;
	readonly state: Readable<SessionState> = { subscribe: this.stateStore.subscribe };
	readonly heartbeat: Readable<HeartbeatState> = derived(this.stateStore, (s) => s.heartbeat);

	constructor(private readonly link: ByteLink, options: SessionOptions = {}) {
		this.codec = options.codec ?? mavlinkCodec;
		this.parser = this.codec.createParser();
		this.timeoutMs = options.heartbeatTimeoutMs ?? HEARTBEAT_TIMEOUT_MS;
		this.unsubscribe = link.subscribe((bytes) => this.onBytes(bytes));
	}

	/** The address messages to the vehicle carry; null until its heartbeat was seen. */
	target(): { systemId: number; componentId: number } | null {
		return get(this.stateStore).vehicle;
	}

	send(message: Message): void {
		const envelope: Envelope = { systemId: GCS_SYSTEM_ID, componentId: GCS_COMPONENT_ID, sequence: this.sequence };
		this.sequence = (this.sequence + 1) & 0xff;
		const bytes = this.codec.encode(envelope, message);
		this.link.send(bytes);
		this.stateStore.update((s) => ({ ...s, txBytes: s.txBytes + bytes.length }));
	}

	/** Forget the vehicle (the connection changed). */
	reset(): void {
		if (this.timer) clearTimeout(this.timer);
		this.timer = null;
		this.stateStore.update((s) => ({ ...initialState(), rxBytes: s.rxBytes, txBytes: s.txBytes }));
	}

	dispose(): void {
		this.unsubscribe?.();
		this.unsubscribe = null;
		if (this.timer) clearTimeout(this.timer);
		this.timer = null;
	}

	private onBytes(bytes: Uint8Array): void {
		let frames = 0;
		let bad = 0;
		const delivered: Array<{ message: Message; envelope: Envelope }> = [];
		for (const byte of bytes) {
			const status = this.parser.push(byte);
			if (status === 'ok') {
				frames++;
				delivered.push({ message: this.parser.frame.message, envelope: this.parser.frame.envelope });
			} else if (status !== 'pending' && status !== 'unsupported') {
				bad++;
			}
		}
		this.stateStore.update((s) => ({ ...s, rxBytes: s.rxBytes + bytes.length, frames: s.frames + frames, badFrames: s.badFrames + bad }));
		for (const { message, envelope } of delivered) {
			if (message.kind === 'Heartbeat') this.onHeartbeat(message, envelope);
			this.bus.emit(message, envelope);
		}
	}

	private onHeartbeat(message: MessageOf<'Heartbeat'>, envelope: Envelope): void {
		const known = get(this.stateStore).vehicle;
		if (known && (known.systemId !== envelope.systemId || known.componentId !== envelope.componentId)) return;
		this.stateStore.update((s) => ({
			...s,
			heartbeat: 'alive',
			vehicle: { systemId: envelope.systemId, componentId: envelope.componentId },
			armed: (message.base_mode & MODE_FLAG_SAFETY_ARMED) !== 0
		}));
		if (this.timer) clearTimeout(this.timer);
		this.timer = setTimeout(() => {
			this.timer = null;
			this.stateStore.update((s) => ({ ...s, heartbeat: 'no heartbeat' }));
		}, this.timeoutMs);
	}
}

function initialState(): SessionState {
	return { heartbeat: 'no heartbeat', vehicle: null, armed: null, rxBytes: 0, txBytes: 0, frames: 0, badFrames: 0 };
}
