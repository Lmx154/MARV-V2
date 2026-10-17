// Byte-wise parameter values (decision 0053 Q1): a parameter travels as the raw 32 bits of its type. An f32 parameter is the
// IEEE-754 binary32 pattern, an i32 parameter the two's-complement pattern; the two are never converted into each other.
// "bits" here is always an unsigned 32-bit integer (0 .. 2^32 - 1).
export type ParamType = 'f32' | 'i32';

const scratch = new DataView(new ArrayBuffer(4));

export function f32ToBits(value: number): number {
	scratch.setFloat32(0, value);
	return scratch.getUint32(0);
}

export function bitsToF32(bits: number): number {
	scratch.setUint32(0, bits >>> 0);
	return scratch.getFloat32(0);
}

export function i32ToBits(value: number): number {
	if (!Number.isInteger(value) || value < -(2 ** 31) || value > 2 ** 31 - 1) throw new RangeError(`${value} is not an int32`);
	scratch.setInt32(0, value);
	return scratch.getUint32(0);
}

export function bitsToI32(bits: number): number {
	scratch.setUint32(0, bits >>> 0);
	return scratch.getInt32(0);
}

export function bitsToValue(bits: number, type: ParamType): number {
	return type === 'f32' ? bitsToF32(bits) : bitsToI32(bits);
}

export function bitsHex(bits: number): string {
	return '0x' + (bits >>> 0).toString(16).padStart(8, '0');
}

/** The shortest decimal text that reads back as the same binary32 value (at most 9 significant digits). */
export function formatF32(bits: number): string {
	const x = bitsToF32(bits);
	if (!Number.isFinite(x)) return Number.isNaN(x) ? 'NaN' : x > 0 ? 'inf' : '-inf';
	if (x === 0) return Object.is(x, -0) ? '-0' : '0';
	for (let digits = 1; digits <= 9; digits++) {
		const text = x.toPrecision(digits);
		if (Math.fround(Number(text)) === x) return String(Number(text));
	}
	return String(x);
}

export function formatValue(bits: number, type: ParamType): string {
	return type === 'f32' ? formatF32(bits) : String(bitsToI32(bits));
}

export type ParsedValue = { ok: true; bits: number } | { ok: false; error: string };

const INT_TEXT = /^[+-]?\d+$/;
const FLOAT_TEXT = /^[+-]?(\d+\.?\d*|\.\d+)([eE][+-]?\d+)?$/;

/** Text typed by the operator to bits. An i32 accepts decimal integers only, an f32 a finite decimal that fits binary32. */
export function parseValueText(text: string, type: ParamType): ParsedValue {
	const t = text.trim();
	if (type === 'i32') {
		if (!INT_TEXT.test(t)) return { ok: false, error: 'an int32 parameter takes a whole number' };
		const v = Number(t);
		if (v < -(2 ** 31) || v > 2 ** 31 - 1) return { ok: false, error: 'outside the int32 range' };
		return { ok: true, bits: i32ToBits(v) };
	}
	if (!FLOAT_TEXT.test(t)) return { ok: false, error: 'a float32 parameter takes a decimal number' };
	const bits = f32ToBits(Number(t));
	if (!Number.isFinite(bitsToF32(bits))) return { ok: false, error: 'outside the float32 range' };
	return { ok: true, bits };
}

/** The PARAM_SET / PARAM_VALUE value field is a JavaScript number holding the 4 bytes as a float. */
export function bitsToWireNumber(bits: number): number {
	return bitsToF32(bits);
}

export function wireNumberToBits(value: number): number {
	return f32ToBits(value);
}

/** True when the 32 bits survive the trip through that float field (a signalling-NaN pattern does not: it is quietened). */
export function carriedExactly(bits: number): boolean {
	return wireNumberToBits(bitsToWireNumber(bits)) === bits >>> 0;
}
