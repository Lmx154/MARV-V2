// The trivial test codec (ground segment GS-3: "a second, trivial codec (for tests) passes the same conformance suite").
// Frame: 'T', system, component, sequence, JSON length (2 bytes, little endian), the message as JSON (a bigint as
// {"$bigint"}, a byte array as {"$bytes"}), CRC-16/CCITT-FALSE over everything after 'T'. It is not a wire format.
import type { Codec, Envelope, Frame, FrameParser, Status } from '$link/codec';
import type { Message } from '$link/generated/messages';

const MAGIC = 0x54;
const HEADER = 6;

function crc16(bytes: ArrayLike<number>, start: number, end: number): number {
	let crc = 0xffff;
	for (let i = start; i < end; i++) {
		crc ^= (bytes[i] as number) << 8;
		for (let bit = 0; bit < 8; bit++) crc = (crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1) & 0xffff;
	}
	return crc;
}

function encode(envelope: Envelope, message: Message): Uint8Array {
	const json = new TextEncoder().encode(
		JSON.stringify(message, (_k, v) =>
			typeof v === 'bigint' ? { $bigint: v.toString() } : v instanceof Uint8Array ? { $bytes: Array.from(v) } : v
		)
	);
	const frame = new Uint8Array(HEADER + json.length + 2);
	frame.set([MAGIC, envelope.systemId, envelope.componentId, envelope.sequence, json.length & 0xff, json.length >> 8]);
	frame.set(json, HEADER);
	const crc = crc16(frame, 1, HEADER + json.length);
	frame[HEADER + json.length] = crc & 0xff;
	frame[HEADER + json.length + 1] = crc >> 8;
	return frame;
}

class TrivialParser implements FrameParser {
	frame: Frame = { envelope: { systemId: 0, componentId: 0, sequence: 0 }, message: { kind: 'ParamRequestList', target_system: 0, target_component: 0 } };
	private buffer: number[] = [];

	push(byte: number): Status {
		if (this.buffer.length === 0 && byte !== MAGIC) return 'pending';
		this.buffer.push(byte);
		if (this.buffer.length < HEADER) return 'pending';
		const length = (this.buffer[4] as number) | ((this.buffer[5] as number) << 8);
		if (this.buffer.length < HEADER + length + 2) return 'pending';
		const bytes = this.buffer;
		this.buffer = [];
		const want = (bytes[HEADER + length] as number) | ((bytes[HEADER + length + 1] as number) << 8);
		if (crc16(bytes, 1, HEADER + length) !== want) return 'bad_crc';
		const text = new TextDecoder().decode(Uint8Array.from(bytes.slice(HEADER, HEADER + length)));
		const message = JSON.parse(text, (_k, v) =>
			v && typeof v === 'object' && '$bigint' in v ? BigInt(v.$bigint) : v && typeof v === 'object' && '$bytes' in v ? Uint8Array.from(v.$bytes) : v
		) as Message;
		this.frame = { envelope: { systemId: bytes[1] as number, componentId: bytes[2] as number, sequence: bytes[3] as number }, message };
		return 'ok';
	}
}

export const trivialCodec: Codec = {
	name: 'trivial',
	encode,
	createParser: () => new TrivialParser()
};
