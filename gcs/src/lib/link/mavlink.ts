// The MAVLink v2 codec with the MARV dialect for the GCS: framing, the checksum and the byte-stream parser. The messages and
// the wire table are generated (generated/messages.ts); nothing here depends on a MAVLink library.
import type { Codec, Envelope, Frame, FrameParser, Status } from './codec';
import { DIALECT, MESSAGES, type Message, type MessageInfo } from './generated/messages';
import { x25, x25Accumulate } from './x25';
import { decodePayload, encodePayload } from './wire';

export const STX = 0xfd;
const HEADER_BYTES = 10;
const CHECKSUM_BYTES = 2;
const SIGNATURE_BYTES = 13;
const INCOMPAT_SIGNED = 0x01;
const BY_KIND = new Map<string, MessageInfo>(MESSAGES.map((m) => [m.kind, m]));
const BY_ID = new Map<number, MessageInfo>(MESSAGES.map((m) => [m.id, m]));

export function encodeMavlink(envelope: Envelope, message: Message): Uint8Array {
	const info = BY_KIND.get(message.kind);
	if (!info) throw new RangeError(`no wire table for ${message.kind}`);
	const payload = encodePayload(info, message);
	const frame = new Uint8Array(HEADER_BYTES + payload.length + CHECKSUM_BYTES);
	frame.set([STX, payload.length, 0, 0, envelope.sequence, envelope.systemId, envelope.componentId]);
	frame.set([info.id & 0xff, (info.id >> 8) & 0xff, (info.id >> 16) & 0xff], 7);
	frame.set(payload, HEADER_BYTES);
	const crc = x25Accumulate(x25(frame, 1, HEADER_BYTES + payload.length), info.crcExtra);
	frame[HEADER_BYTES + payload.length] = crc & 0xff;
	frame[HEADER_BYTES + payload.length + 1] = crc >> 8;
	return frame;
}

const enum State {
	Idle,
	Header,
	Payload,
	Checksum,
	Signature
}

export class MavlinkParser implements FrameParser {
	frame: Frame = { envelope: { systemId: 0, componentId: 0, sequence: 0 }, message: { kind: 'ParamRequestList', target_system: 0, target_component: 0 } };
	private state = State.Idle;
	private buffer = new Uint8Array(HEADER_BYTES + 255 + CHECKSUM_BYTES + SIGNATURE_BYTES);
	private count = 0;
	private signatureLeft = 0;
	private failed: Status | null = null;

	push(byte: number): Status {
		switch (this.state) {
			case State.Idle:
				if (byte === STX) this.begin(byte);
				return 'pending';
			case State.Header:
				this.buffer[this.count++] = byte;
				if (this.count === HEADER_BYTES) {
					if ((this.buffer[2] as number) & ~INCOMPAT_SIGNED) {
						this.state = State.Idle; // an incompatibility flag this parser does not know: drop the frame
						return 'pending';
					}
					this.state = this.buffer[1] === 0 ? State.Checksum : State.Payload;
				}
				return 'pending';
			case State.Payload:
				this.buffer[this.count++] = byte;
				if (this.count === HEADER_BYTES + (this.buffer[1] as number)) this.state = State.Checksum;
				return 'pending';
			case State.Checksum:
				this.buffer[this.count++] = byte;
				if (this.count < HEADER_BYTES + (this.buffer[1] as number) + CHECKSUM_BYTES) return 'pending';
				return this.finishChecksum(byte);
			case State.Signature:
				this.count++;
				if (--this.signatureLeft > 0) return 'pending';
				this.state = State.Idle;
				return this.failed ?? this.deliver();
		}
	}

	reset(): void {
		this.state = State.Idle;
		this.count = 0;
	}

	private begin(byte: number): void {
		this.buffer[0] = byte;
		this.count = 1;
		this.state = State.Header;
		this.failed = null;
	}

	private finishChecksum(lastByte: number): Status {
		const length = this.buffer[1] as number;
		const id = (this.buffer[7] as number) | ((this.buffer[8] as number) << 8) | ((this.buffer[9] as number) << 16);
		const entry = DIALECT[id];
		let status: Status | null = null;
		if (!entry) {
			status = 'bad_crc';
		} else {
			const crc = x25Accumulate(x25(this.buffer, 1, HEADER_BYTES + length), entry.crcExtra);
			const wire = (this.buffer[HEADER_BYTES + length] as number) | ((this.buffer[HEADER_BYTES + length + 1] as number) << 8);
			if (crc !== wire) status = 'bad_crc';
		}
		if ((this.buffer[2] as number) & INCOMPAT_SIGNED) {
			this.failed = status;
			this.state = State.Signature;
			this.signatureLeft = SIGNATURE_BYTES;
			return 'pending';
		}
		this.state = State.Idle;
		if (status === 'bad_crc') {
			if (lastByte === STX) this.begin(lastByte); // the checksum's last byte may itself start the next frame
			return status;
		}
		return this.deliver();
	}

	private deliver(): Status {
		const length = this.buffer[1] as number;
		const id = (this.buffer[7] as number) | ((this.buffer[8] as number) << 8) | ((this.buffer[9] as number) << 16);
		const info = BY_ID.get(id);
		if (!info) return 'unsupported';
		if (length > info.maxLength) return 'bad_length';
		this.frame = {
			envelope: { systemId: this.buffer[5] as number, componentId: this.buffer[6] as number, sequence: this.buffer[4] as number },
			message: decodePayload(info, this.buffer.slice(HEADER_BYTES, HEADER_BYTES + length))
		};
		return 'ok';
	}
}

export const mavlinkCodec: Codec = {
	name: 'mavlink',
	encode: encodeMavlink,
	createParser: () => new MavlinkParser()
};
