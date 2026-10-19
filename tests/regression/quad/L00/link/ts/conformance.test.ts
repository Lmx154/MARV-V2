// The link codec conformance suite, TypeScript side (ground segment GS-3; core section 8 rows IF-1, IF-5, IF-11; decision
// 0053). The same properties as tests/regression/quad/L00/link/codec_conformance_test.cpp: every codec (the MAVLink codec
// and the trivial test codec) round-trips every sample, delivers frames from a stream with garbage between them, rejects a
// frame with any one byte changed and tells a truncated frame (pending) from an extended one (bad_length); the MAVLink
// codec alone is held to its framing and to the C++ codec through the committed vectors.
import { readFileSync, writeFileSync } from 'node:fs';
import { describe, expect, it } from 'vitest';
import { decodeFrame, type Codec, type Envelope } from '$link/codec';
import { MESSAGES } from '$link/generated/messages';
import { encodeMavlink, MavlinkParser, mavlinkCodec } from '$link/mavlink';
import { envelopeFor, samples } from './samples';
import { trivialCodec } from './trivial_codec';

const VECTORS = new URL('../vectors/', import.meta.url);
const toHex = (b: Uint8Array) => Array.from(b, (x) => x.toString(16).padStart(2, '0')).join('');
const fromHex = (s: string) => Uint8Array.from(s.match(/../g) ?? [], (h) => parseInt(h, 16));

interface VectorLine {
	name: string;
	envelope: Envelope;
	frame: Uint8Array;
}

function readVectors(file: string): VectorLine[] {
	return readFileSync(new URL(file, VECTORS), 'utf8')
		.split('\n')
		.filter((l) => l && !l.startsWith('#'))
		.map((l) => {
			const [name, system, component, sequence, hex] = l.split(' ') as [string, string, string, string, string];
			return { name, envelope: { systemId: +system, componentId: +component, sequence: +sequence }, frame: fromHex(hex) };
		});
}

describe.each<Codec>([mavlinkCodec, trivialCodec])('codec conformance: $name', (codec) => {
	it('round-trips every sample with its envelope', () => {
		samples.forEach((s, i) => {
			const { status, frame } = decodeFrame(codec.createParser(), codec.encode(envelopeFor(i), s.message));
			expect(status, s.name).toBe('ok');
			expect(frame?.envelope, s.name).toEqual(envelopeFor(i));
			expect(frame?.message, s.name).toEqual(s.message);
		});
	});

	it('delivers the frames of a stream with garbage between them', () => {
		const parts: number[] = [];
		samples.forEach((s, i) => parts.push(...codec.encode(envelopeFor(i), s.message), 0x00, 0x55));
		const parser = codec.createParser();
		let arrived = 0;
		for (const byte of parts) {
			if (parser.push(byte) === 'ok') {
				expect(parser.frame.envelope).toEqual(envelopeFor(arrived));
				expect(parser.frame.message, samples[arrived]?.name).toEqual(samples[arrived]?.message);
				arrived++;
			}
		}
		expect(arrived).toBe(samples.length);
	});

	it('does not decode a frame with any one byte changed', () => {
		samples.forEach((s, i) => {
			const good = codec.encode(envelopeFor(i), s.message);
			for (let at = 0; at < good.length; at++) {
				const bad = good.slice();
				bad[at] = (bad[at] as number) ^ 0x5a;
				expect(decodeFrame(codec.createParser(), bad).status, `${s.name} byte ${at}`).not.toBe('ok');
			}
		});
	});

	it('calls a truncated frame pending and an extended one bad_length', () => {
		samples.forEach((s, i) => {
			const good = codec.encode(envelopeFor(i), s.message);
			expect(decodeFrame(codec.createParser(), good.slice(0, -1)).status, s.name).toBe('pending');
			const longer = new Uint8Array(good.length + 1);
			longer.set(good);
			expect(decodeFrame(codec.createParser(), longer).status, s.name).toBe('bad_length');
		});
	});
});

const crcOf = (bytes: number[], extra: number) => {
	let crc = 0xffff;
	for (const b of [...bytes, extra]) {
		let t = (b ^ (crc & 0xff)) & 0xff;
		t = (t ^ (t << 4)) & 0xff;
		crc = ((crc >> 8) ^ (t << 8) ^ (t << 3) ^ (t >> 4)) & 0xffff;
	}
	return crc;
};

function handFrame(id: number, payloadLength: number, crcExtra: number): Uint8Array {
	const body = [payloadLength, 0, 0, 7, 1, 2, id & 0xff, (id >> 8) & 0xff, id >> 16, ...Array<number>(payloadLength).fill(1)];
	const crc = crcOf(body, crcExtra);
	return Uint8Array.from([0xfd, ...body, crc & 0xff, crc >> 8]);
}

describe('MAVLink framing', () => {
	const HEARTBEAT = 0;
	const HEARTBEAT_BYTES = 9;
	const HEARTBEAT_CRC_EXTRA = 50;
	const SYSTEM_TIME = 2; // a common.xml message outside link/scope.txt
	const SYSTEM_TIME_BYTES = 12;
	const SYSTEM_TIME_CRC_EXTRA = 137;
	const ABSENT = 14999;

	it('carries the envelope and the message id in the header', () => {
		const bytes = encodeMavlink({ systemId: 7, componentId: 190, sequence: 33 }, { ...samples[10]!.message });
		expect(bytes[0]).toBe(0xfd);
		expect(bytes[2]).toBe(0);
		expect([bytes[4], bytes[5], bytes[6]]).toEqual([33, 7, 190]);
		expect(bytes[7]! | (bytes[8]! << 8) | (bytes[9]! << 16)).toBe(MESSAGES.find((m) => m.kind === 'MarvIdentity')!.id);
	});

	it('trims the zero tail of the payload to one byte', () => {
		const bytes = encodeMavlink({ systemId: 1, componentId: 1, sequence: 0 }, samples[3]!.message);
		expect(bytes.length).toBe(10 + 1 + 2);
		expect(bytes[1]).toBe(1);
	});

	it('decodes a frame built by hand and calls one byte more than the message has bad_length', () => {
		const exact = decodeFrame(new MavlinkParser(), handFrame(HEARTBEAT, HEARTBEAT_BYTES, HEARTBEAT_CRC_EXTRA));
		expect(exact.status).toBe('ok');
		expect(exact.frame?.envelope).toEqual({ systemId: 1, componentId: 2, sequence: 7 });
		expect(decodeFrame(new MavlinkParser(), handFrame(HEARTBEAT, HEARTBEAT_BYTES + 1, HEARTBEAT_CRC_EXTRA)).status).toBe('bad_length');
	});

	it('calls an id not in the dialect bad_crc and a valid frame of a message it does not carry unsupported', () => {
		expect(decodeFrame(new MavlinkParser(), handFrame(ABSENT, 4, 0)).status).toBe('bad_crc');
		expect(decodeFrame(new MavlinkParser(), handFrame(SYSTEM_TIME, SYSTEM_TIME_BYTES, SYSTEM_TIME_CRC_EXTRA)).status).toBe('unsupported');
	});

	it('rejects a correct checksum made with the wrong CRC_EXTRA', () => {
		expect(decodeFrame(new MavlinkParser(), handFrame(HEARTBEAT, HEARTBEAT_BYTES, HEARTBEAT_CRC_EXTRA + 1)).status).toBe('bad_crc');
	});
});

describe('MAVLink vectors', () => {
	const mavlinkFrames = () => samples.map((s, i) => `${s.name} ${envelopeFor(i).systemId} ${envelopeFor(i).componentId} ${envelopeFor(i).sequence} ${toHex(encodeMavlink(envelopeFor(i), s.message))}`);

	it('writes vectors/ts_frames.txt, the frames the C++ codec decodes (MARV_UPDATE_VECTORS=1), and it is current', () => {
		const text =
			'# MAVLink v2 frames produced by the TypeScript codec (gcs/src/lib/link) for the samples of samples.ts.\n' +
			'# name system component sequence frame-hex\n' +
			mavlinkFrames().join('\n') +
			'\n';
		if (process.env.MARV_UPDATE_VECTORS === '1') writeFileSync(new URL('ts_frames.txt', VECTORS), text);
		expect(readFileSync(new URL('ts_frames.txt', VECTORS), 'utf8')).toBe(text);
	});

	it('decodes every frame the C++ codec wrote to the sample and encodes it to the same bytes', () => {
		const lines = readVectors('cpp_frames.txt');
		expect(lines.length).toBe(samples.length);
		lines.forEach((line, i) => {
			const sample = samples[i]!;
			expect(line.name).toBe(sample.name);
			const { status, frame } = decodeFrame(new MavlinkParser(), line.frame);
			expect(status, line.name).toBe('ok');
			expect(frame?.envelope, line.name).toEqual(line.envelope);
			expect(frame?.message, line.name).toEqual(sample.message);
			expect(encodeMavlink(line.envelope, sample.message), line.name).toEqual(line.frame);
		});
	});
});
