// The link codec interface of the GCS (core section 8; ground segment GS-3). Everything above the codec uses these types and
// the generated messages; only the codec (this directory) knows MAVLink. A codec is an `encode` function and a parser;
// tests/regression/quad/L00/link/ts holds every codec to the same conformance suite.
import type { Message } from './generated/messages';

export interface Envelope {
	systemId: number;
	componentId: number;
	sequence: number;
}

export interface Frame {
	envelope: Envelope;
	message: Message;
}

export type Status =
	/** A frame was decoded; `frame` holds it. */
	| 'ok'
	/** More bytes are needed. */
	| 'pending'
	/** The checksum is wrong, or the message id is not in the dialect. */
	| 'bad_crc'
	/** The payload is longer than the message allows, or bytes follow a complete frame. */
	| 'bad_length'
	/** A valid frame of a message this build does not carry. */
	| 'unsupported';

export interface FrameParser {
	push(byte: number): Status;
	/** The last frame decoded; valid after `push` returned 'ok'. */
	readonly frame: Frame;
}

export interface Codec {
	readonly name: string;
	encode(envelope: Envelope, message: Message): Uint8Array;
	createParser(): FrameParser;
}

/** Decodes exactly one frame from `bytes` with a fresh parser (the same rules as decode_frame in fw/link). */
export function decodeFrame(parser: FrameParser, bytes: Uint8Array): { status: Status; frame?: Frame } {
	let status: Status = 'pending';
	for (const byte of bytes) {
		if (status !== 'pending') return { status: 'bad_length' };
		status = parser.push(byte);
		if (status !== 'pending' && status !== 'ok') return { status };
	}
	return status === 'ok' ? { status, frame: parser.frame } : { status };
}
