// The ground log of the helper (tools/gs/groundlog.py, ground spec GS-8) as the tests need it: a writer of the file layout and a reader of
// its received chunks, so that a recorded link can be fed to a LinkSession in place of a vehicle. Layout, little-endian: header "MARVGL01",
// u64 wall_ns, u64 mono_ns; records u8 kind (1 received, 2 sent, 3 event), u64 mono_ns, u32 length, payload.
import { encodeMavlink } from '../mavlink';
import type { Message } from '../generated/messages';
import type { ByteLink } from '../transport';

const MAGIC = 'MARVGL01';
const HEADER_BYTES = 24;
const RECORD_BYTES = 13;
const KIND_RX = 1;
const NS_PER_MS = 1_000_000n;

export interface LoggedChunk {
	/** When the chunk was read from the device (ms on the log's monotonic clock). */
	atMs: number;
	bytes: Uint8Array;
}

/** The frames of `messages` from one vehicle, in order (sequence numbers counting up). */
export function framesOf(messages: readonly Message[]): Uint8Array[] {
	return messages.map((message, i) => encodeMavlink({ systemId: 1, componentId: 1, sequence: i & 0xff }, message));
}

/** A ground log holding the received chunks. */
export function buildGroundLog(chunks: readonly LoggedChunk[]): Uint8Array {
	const total = HEADER_BYTES + chunks.reduce((n, c) => n + RECORD_BYTES + c.bytes.length, 0);
	const out = new Uint8Array(total);
	const view = new DataView(out.buffer);
	out.set(new TextEncoder().encode(MAGIC));
	let at = HEADER_BYTES;
	for (const c of chunks) {
		view.setUint8(at, KIND_RX);
		view.setBigUint64(at + 1, BigInt(Math.round(c.atMs)) * NS_PER_MS, true);
		view.setUint32(at + 9, c.bytes.length, true);
		out.set(c.bytes, at + RECORD_BYTES);
		at += RECORD_BYTES + c.bytes.length;
	}
	return out;
}

/** The received chunks of a ground log, with their times; a truncated last record is ignored, as the helper's reader does. */
export function readGroundLog(log: Uint8Array): LoggedChunk[] {
	if (new TextDecoder().decode(log.subarray(0, MAGIC.length)) !== MAGIC) throw new Error('not a ground log');
	const view = new DataView(log.buffer, log.byteOffset, log.byteLength);
	const out: LoggedChunk[] = [];
	let at = HEADER_BYTES;
	while (at + RECORD_BYTES <= log.length) {
		const kind = view.getUint8(at);
		const t = view.getBigUint64(at + 1, true);
		const length = view.getUint32(at + 9, true);
		if (at + RECORD_BYTES + length > log.length) break;
		if (kind === KIND_RX) out.push({ atMs: Number(t / NS_PER_MS), bytes: log.slice(at + RECORD_BYTES, at + RECORD_BYTES + length) });
		at += RECORD_BYTES + length;
	}
	return out;
}

/** A link that has only what was recorded: bytes are pushed into it, and what the client sends goes nowhere. */
export class RecordedLink implements ByteLink {
	private handler: ((bytes: Uint8Array) => void) | null = null;
	readonly sent: Uint8Array[] = [];
	send(bytes: Uint8Array): void {
		this.sent.push(bytes);
	}
	subscribe(handler: (bytes: Uint8Array) => void): () => void {
		this.handler = handler;
		return () => {
			this.handler = null;
		};
	}
	push(bytes: Uint8Array): void {
		this.handler?.(bytes);
	}
}
