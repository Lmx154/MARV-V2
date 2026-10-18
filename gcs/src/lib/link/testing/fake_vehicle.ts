// A fake vehicle for the tests: a TypeScript stub that answers the GUI's parameter protocol with frames built by the codec
// (the behaviour decision 0053 Q1 and Q2 give the real vehicle; unit 6 builds that one, and the SIL replaces this stub in the
// end-to-end run). It keeps a staged and an active value per parameter, refuses a write that the ruling refuses (wrong type,
// locked, armed, not finite, empty source) with the unchanged value and a STATUSTEXT, and stages a plain write with the
// default source text until a source SET replaces it.
import type { Envelope } from '../codec';
import { MarvParamSourceOp, type MarvParamMethod, type MarvParamOrigin, type MarvParamSigmaKind, type Message } from '../generated/messages';
import { MavlinkParser, encodeMavlink } from '../mavlink';
import type { ByteLink } from '../transport';
import { bitsToF32, paramValueBits, paramValueFields } from '../bits';

export interface FakeParam {
	id: number;
	linkId: string;
	type: 'f32' | 'i32';
	bits: number;
	locked?: boolean;
	origin?: number;
	method?: number;
	sigmaKind?: number;
	sigma?: number;
	source?: string;
}

export interface FakeVehicleOptions {
	params: FakeParam[];
	schemaHash?: bigint;
	imageSha256?: Uint8Array;
	configSha256?: Uint8Array;
	armed?: boolean;
	/** Answer REQUEST_MESSAGE for the identity and the version (default true). */
	answersIdentity?: boolean;
	capabilities?: bigint;
}

export const FAKE_SYSTEM_ID = 1;
export const FAKE_COMPONENT_ID = 1;
export const DEFAULT_SOURCE = 'set over the link; no source stated';
const PARAM_TYPE_INT32 = 6;
const PARAM_TYPE_REAL32 = 9;
const REQUEST_MESSAGE = 512;
const CHUNK = 64;

interface Slot extends FakeParam {
	staged: number;
	active: number;
	text: string;
	pending: Uint8Array | null;
}

const enc = new TextEncoder();
const pad = (text: string, n: number): Uint8Array => {
	const out = new Uint8Array(n);
	out.set(enc.encode(text).subarray(0, n));
	return out;
};

export class FakeVehicle {
	readonly slots: Slot[];
	armed: boolean;
	readonly schemaHash: bigint;
	readonly sent: Message[] = [];
	readonly received: Message[] = [];
	/** Indices whose PARAM_VALUE or MARV_PARAM_RECORD is dropped the first time (listing retry tests). */
	dropFirstValueFor = new Set<number>();
	dropFirstRecordFor = new Set<number>();
	/** Indices never answered (an incomplete listing). */
	neverSendFor = new Set<number>();
	/** Ignore source SET messages (a vehicle that does not store sources). */
	ignoreSourceSet = false;
	/** Ignore the next N source SET messages (a write that is lost once). */
	dropSourceSets = 0;
	/** Answer a PARAM_SET first with a PARAM_VALUE of the value it replaces (a listing reply that was still on its way). */
	staleValueBeforeEcho = false;
	/** Answer a PARAM_SET first with a PARAM_VALUE carrying these bits (a stale reply that happens to carry the requested value). */
	staleBitsBeforeAnswer: number | null = null;
	/** Delay the PARAM_VALUE answering a PARAM_SET by this long. */
	answerDelayMs = 0;
	private handler: ((bytes: Uint8Array) => void) | null = null;
	private readonly parser = new MavlinkParser();
	private sequence = 0;
	private readonly options: FakeVehicleOptions;

	constructor(options: FakeVehicleOptions) {
		this.options = options;
		this.schemaHash = options.schemaHash ?? 0x3ae00eb50938acban;
		this.armed = options.armed ?? false;
		this.slots = options.params.map((p) => ({
			...p,
			staged: p.bits,
			active: p.bits,
			text: p.source ?? 'factory source of ' + p.linkId,
			pending: null
		}));
	}

	/** The GCS's end of the loopback. */
	readonly link: ByteLink = {
		send: (bytes) => this.fromGcs(bytes),
		subscribe: (h) => {
			this.handler = h;
			return () => {
				this.handler = null;
			};
		}
	};

	heartbeat(): void {
		this.emit({ kind: 'Heartbeat', type: 2, autopilot: 0, base_mode: this.armed ? 128 : 0, custom_mode: 0, system_status: 4 });
	}

	emit(message: Message): void {
		this.sent.push(message);
		const envelope: Envelope = { systemId: FAKE_SYSTEM_ID, componentId: FAKE_COMPONENT_ID, sequence: this.sequence++ & 0xff };
		const bytes = encodeMavlink(envelope, message);
		queueMicrotask(() => this.handler?.(bytes));
	}

	/** Splits the bytes delivered to the GCS at a given point (a frame cut across two deliveries). */
	emitSplit(message: Message, at: number): void {
		this.sent.push(message);
		const bytes = encodeMavlink({ systemId: FAKE_SYSTEM_ID, componentId: FAKE_COMPONENT_ID, sequence: this.sequence++ & 0xff }, message);
		queueMicrotask(() => this.handler?.(bytes.subarray(0, at)));
		queueMicrotask(() => this.handler?.(bytes.subarray(at)));
	}

	private fromGcs(bytes: Uint8Array): void {
		for (const b of bytes) {
			if (this.parser.push(b) === 'ok') {
				const m = this.parser.frame.message;
				this.received.push(m);
				this.handle(m);
			}
		}
	}

	private status(text: string): void {
		this.emit({ kind: 'Statustext', severity: 4, text: pad(text, 50), id: 0, chunk_seq: 0 });
	}

	private paramValue(s: Slot): Message {
		return {
			kind: 'ParamValue', param_id: pad(s.linkId, 16), ...paramValueFields(s.staged),
			param_type: s.type === 'f32' ? PARAM_TYPE_REAL32 : PARAM_TYPE_INT32, param_count: this.slots.length, param_index: s.id
		};
	}

	private record(s: Slot): Message {
		const bytes = enc.encode(s.text);
		const first = bytes.subarray(0, CHUNK);
		return {
			kind: 'MarvParamRecord', active_bits: s.active, staged_bits: s.staged, sigma: s.sigma ?? 0, param_index: s.id,
			param_count: this.slots.length, source_length: bytes.length, origin: (s.origin ?? 0) as MarvParamOrigin, method: (s.method ?? 4) as MarvParamMethod,
			sigma_kind: (s.sigmaKind ?? 1) as MarvParamSigmaKind, lock: s.locked ? 1 : 0, source_chunk_length: first.length, source_chunk: padBytes(first)
		};
	}

	private sendParam(s: Slot): void {
		if (this.neverSendFor.has(s.id)) return;
		if (this.dropFirstValueFor.delete(s.id) === false) this.emit(this.paramValue(s));
		if (this.dropFirstRecordFor.delete(s.id) === false) this.emit(this.record(s));
	}

	private handle(m: Message): void {
		switch (m.kind) {
			case 'CommandLong':
				if (m.command !== REQUEST_MESSAGE || this.options.answersIdentity === false) return;
				if (m.param1 === 15200) this.sendIdentity();
				if (m.param1 === 148) this.sendVersion();
				return;
			case 'ParamRequestList':
				for (const s of this.slots) this.sendParam(s);
				return;
			case 'ParamRequestRead': {
				const s = this.slots.find((x) => x.id === m.param_index);
				if (s) this.sendParam(s);
				return;
			}
			case 'ParamSet':
				return this.set(m);
			case 'MarvParamSource':
				return this.source(m);
			case 'MarvConfigCommand':
				this.emit({ kind: 'MarvConfigAck', target_system: 255, target_component: 190, action: m.action, result: 4 });
				return;
			default:
				return;
		}
	}

	sendIdentity(): void {
		this.emit({
			kind: 'MarvIdentity', param_schema_hash: this.schemaHash, image_sha256: this.options.imageSha256 ?? new Uint8Array(32).fill(0xab),
			config_sha256: this.options.configSha256 ?? new Uint8Array(32).fill(0xcd), validation_report_id: new Uint8Array(32), config_source: 0,
			config_state: 0, config_refusal: 0, last_activation_result: 0,
			staged_differs: this.slots.some((s) => s.staged !== s.active) ? 1 : 0
		});
	}

	sendVersion(): void {
		this.emit({
			kind: 'AutopilotVersion', capabilities: this.options.capabilities ?? 16n, flight_sw_version: 0, middleware_sw_version: 0, os_sw_version: 0,
			board_version: 0, flight_custom_version: new Uint8Array(8), middleware_custom_version: new Uint8Array(8), os_custom_version: new Uint8Array(8),
			vendor_id: 0, product_id: 0, uid: 0n, uid2: new Uint8Array(18)
		});
	}

	private set(m: Extract<Message, { kind: 'ParamSet' }>): void {
		const id = new TextDecoder().decode(m.param_id.subarray(0, m.param_id.indexOf(0) < 0 ? 16 : m.param_id.indexOf(0)));
		const s = this.slots.find((x) => x.linkId === id);
		if (!s) return this.status(`refused: no parameter ${id}`);
		if (this.staleBitsBeforeAnswer !== null) this.emit({ ...this.paramValue(s), ...paramValueFields(this.staleBitsBeforeAnswer) });
		const type = m.param_type === PARAM_TYPE_REAL32 ? 'f32' : m.param_type === PARAM_TYPE_INT32 ? 'i32' : null;
		const bits = paramValueBits(m);
		let why: string | null = null;
		if (type !== s.type) why = `${id}: type differs from the record`;
		else if (s.locked) why = `${id}: locked, keeps its compiled value`;
		else if (this.armed) why = 'armed: parameters are written only while disarmed';
		else if (s.type === 'f32' && !Number.isFinite(bitsToF32(bits))) why = `${id}: not finite`;
		if (why) {
			this.status('refused: ' + why);
			this.answer(s);
			return;
		}
		if (this.staleValueBeforeEcho) this.emit(this.paramValue(s));
		s.staged = bits;
		s.text = DEFAULT_SOURCE;
		s.origin = 2;
		s.method = 6;
		s.sigmaKind = 3;
		s.sigma = 0;
		this.answer(s);
	}

	private answer(s: Slot): void {
		const message = this.paramValue(s);
		if (this.answerDelayMs > 0) setTimeout(() => this.emit(message), this.answerDelayMs);
		else this.emit(message);
	}

	private source(m: Extract<Message, { kind: 'MarvParamSource' }>): void {
		const s = this.slots.find((x) => x.id === m.param_index);
		if (!s) return;
		if (m.param_schema_hash !== this.schemaHash) return this.status('refused: schema hash differs');
		if (m.operation === MarvParamSourceOp.Read) {
			const bytes = enc.encode(s.text);
			const chunk = bytes.subarray(m.offset, m.offset + CHUNK);
			this.emit({
				kind: 'MarvParamSource', param_schema_hash: this.schemaHash, param_index: s.id, offset: m.offset, total_length: bytes.length,
				target_system: 255, target_component: 190, operation: MarvParamSourceOp.Data, chunk_length: chunk.length, chunk: padBytes(chunk)
			});
		} else if (m.operation === MarvParamSourceOp.Set && !this.ignoreSourceSet && this.dropSourceSets-- <= 0) {
			if (m.offset === 0) s.pending = new Uint8Array(m.total_length);
			if (!s.pending || m.offset + m.chunk_length > s.pending.length) return;
			s.pending.set(m.chunk.subarray(0, m.chunk_length), m.offset);
			if (m.offset + m.chunk_length === s.pending.length) {
				s.text = new TextDecoder().decode(s.pending);
				s.pending = null;
			}
		}
	}
}

function padBytes(bytes: Uint8Array): Uint8Array {
	const out = new Uint8Array(CHUNK);
	out.set(bytes);
	return out;
}
