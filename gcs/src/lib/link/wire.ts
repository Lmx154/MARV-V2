// Table-driven payload encode and decode for the generated messages (MAVLink v2 serialisation: fields in wire order, little
// endian, trailing zero bytes of the payload trimmed to at least one byte, a shorter payload read as zero padded).
import { MAVLINK_VERSION, type FieldInfo, type Message, type MessageInfo } from './generated/messages';

type FieldValue = number | bigint | Uint8Array;
type Fields = Record<string, FieldValue>;

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
		if (f.length > 0) {
			const array = value as Uint8Array;
			if (array.length !== f.length) throw new RangeError(`${info.name}.${f.name} needs ${f.length} bytes`);
			bytes.set(array, f.offset);
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
		fields[f.name] = f.length > 0 ? padded.slice(f.offset, f.offset + f.length) : readScalar(view, f);
	}
	return { kind: info.kind, ...fields } as unknown as Message;
}

function writeScalar(view: DataView, f: FieldInfo, value: number | bigint): void {
	switch (f.type) {
		case 'u8': return view.setUint8(f.offset, value as number);
		case 'i8': return view.setInt8(f.offset, value as number);
		case 'u16': return view.setUint16(f.offset, value as number, true);
		case 'i16': return view.setInt16(f.offset, value as number, true);
		case 'u32': return view.setUint32(f.offset, value as number, true);
		case 'i32': return view.setInt32(f.offset, value as number, true);
		case 'u64': return view.setBigUint64(f.offset, value as bigint, true);
		case 'i64': return view.setBigInt64(f.offset, value as bigint, true);
		case 'f32': return view.setFloat32(f.offset, value as number, true);
		case 'f64': return view.setFloat64(f.offset, value as number, true);
		case 'version': return;
	}
}

function readScalar(view: DataView, f: FieldInfo): number | bigint {
	switch (f.type) {
		case 'u8': return view.getUint8(f.offset);
		case 'i8': return view.getInt8(f.offset);
		case 'u16': return view.getUint16(f.offset, true);
		case 'i16': return view.getInt16(f.offset, true);
		case 'u32': return view.getUint32(f.offset, true);
		case 'i32': return view.getInt32(f.offset, true);
		case 'u64': return view.getBigUint64(f.offset, true);
		case 'i64': return view.getBigInt64(f.offset, true);
		case 'f32': return view.getFloat32(f.offset, true);
		case 'f64': return view.getFloat64(f.offset, true);
		case 'version': return 0;
	}
}
