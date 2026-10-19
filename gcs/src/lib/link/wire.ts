// Table-driven payload encode and decode for the generated messages (MAVLink v2 serialisation: fields in wire order, little
// endian, trailing zero bytes of the payload trimmed to at least one byte, a shorter payload read as zero padded).
import { MAVLINK_VERSION, type FieldInfo, type Message, type MessageInfo } from './generated/messages';

type FieldValue = number | bigint | Uint8Array | number[] | bigint[];
type Fields = Record<string, FieldValue>;

/** Suffix of the raw-bits companion of a float field: a float as a JavaScript number cannot hold every 32-bit pattern. */
const RAW_BITS = '_bits';

/** Bytes of one element of each wire type. */
const ELEMENT_BYTES: Record<FieldInfo['type'], number> = { u8: 1, i8: 1, u16: 2, i16: 2, u32: 4, i32: 4, u64: 8, i64: 8, f32: 4, f64: 8, version: 1 };

/** A byte or char array is a Uint8Array; an array of any other type is an array of numbers (bigints for 64-bit elements). */
const isByteArray = (f: FieldInfo) => f.length > 0 && f.type === 'u8';

export function encodePayload(info: MessageInfo, message: Message): Uint8Array {
	const bytes = new Uint8Array(info.maxLength);
	const view = new DataView(bytes.buffer);
	const fields = message as unknown as Fields;
	for (const f of info.fields) {
		if (f.type === 'version') {
			view.setUint8(f.offset, MAVLINK_VERSION);
			continue;
		}
		const value = fields[f.name];
		const rawBits = f.type === 'f32' && f.length === 0 ? fields[f.name + RAW_BITS] : undefined;
		if (typeof rawBits === 'number') {
			view.setUint32(f.offset, rawBits >>> 0, true);
		} else if (isByteArray(f)) {
			const array = value as Uint8Array;
			if (array.length !== f.length) throw new RangeError(`${info.name}.${f.name} needs ${f.length} bytes`);
			bytes.set(array, f.offset);
		} else if (f.length > 0) {
			const array = value as (number | bigint)[];
			if (array.length !== f.length) throw new RangeError(`${info.name}.${f.name} needs ${f.length} elements`);
			array.forEach((v, i) => writeScalar(view, f, v, f.offset + i * ELEMENT_BYTES[f.type]));
		} else {
			writeScalar(view, f, value as number | bigint);
		}
	}
	let length = bytes.length;
	while (length > 1 && bytes[length - 1] === 0) length--;
	return bytes.subarray(0, length);
}

export function decodePayload(info: MessageInfo, payload: Uint8Array): Message {
	const padded = new Uint8Array(info.maxLength);
	padded.set(payload.subarray(0, info.maxLength));
	const view = new DataView(padded.buffer);
	const fields: Fields = {};
	for (const f of info.fields) {
		if (f.type === 'version') continue;
		if (isByteArray(f)) fields[f.name] = padded.slice(f.offset, f.offset + f.length);
		else if (f.length > 0) fields[f.name] = Array.from({ length: f.length }, (_, i) => readScalar(view, f, f.offset + i * ELEMENT_BYTES[f.type])) as number[] | bigint[];
		else fields[f.name] = readScalar(view, f);
	}
	const message = { kind: info.kind, ...fields };
	for (const f of info.fields) {
		// non-enumerable: the decoded message compares equal to one built from the numeric view alone
		if (f.type === 'f32' && f.length === 0) Object.defineProperty(message, f.name + RAW_BITS, { value: view.getUint32(f.offset, true), enumerable: false });
	}
	return message as unknown as Message;
}

function writeScalar(view: DataView, f: FieldInfo, value: number | bigint, offset: number = f.offset): void {
	switch (f.type) {
		case 'u8': return view.setUint8(offset, value as number);
		case 'i8': return view.setInt8(offset, value as number);
		case 'u16': return view.setUint16(offset, value as number, true);
		case 'i16': return view.setInt16(offset, value as number, true);
		case 'u32': return view.setUint32(offset, value as number, true);
		case 'i32': return view.setInt32(offset, value as number, true);
		case 'u64': return view.setBigUint64(offset, value as bigint, true);
		case 'i64': return view.setBigInt64(offset, value as bigint, true);
		case 'f32': return view.setFloat32(offset, value as number, true);
		case 'f64': return view.setFloat64(offset, value as number, true);
		case 'version': return;
	}
}

function readScalar(view: DataView, f: FieldInfo, offset: number = f.offset): number | bigint {
	switch (f.type) {
		case 'u8': return view.getUint8(offset);
		case 'i8': return view.getInt8(offset);
		case 'u16': return view.getUint16(offset, true);
		case 'i16': return view.getInt16(offset, true);
		case 'u32': return view.getUint32(offset, true);
		case 'i32': return view.getInt32(offset, true);
		case 'u64': return view.getBigUint64(offset, true);
		case 'i64': return view.getBigInt64(offset, true);
		case 'f32': return view.getFloat32(offset, true);
		case 'f64': return view.getFloat64(offset, true);
		case 'version': return 0;
	}
}
