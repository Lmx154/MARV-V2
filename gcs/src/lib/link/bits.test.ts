import { describe, expect, it } from 'vitest';
import {
	bitsHex, bitsToF32, bitsToI32, bitsToWireNumber, carriedExactly, f32ToBits, formatF32, formatValue, i32ToBits, parseValueText,
	wireNumberToBits
} from './bits';

describe('byte-wise value encoding', () => {
	it('keeps REAL32 as its IEEE-754 bits', () => {
		expect(f32ToBits(1)).toBe(0x3f800000);
		expect(f32ToBits(-2)).toBe(0xc0000000);
		expect(f32ToBits(0.732)).toBe(0x3f3b645a);
		expect(bitsToF32(0x3f3b645a)).toBe(Math.fround(0.732));
	});

	it('keeps INT32 as two\'s complement bits and never reads them as a float', () => {
		expect(i32ToBits(1)).toBe(1);
		expect(i32ToBits(-1)).toBe(0xffffffff);
		expect(i32ToBits(-(2 ** 31))).toBe(0x80000000);
		expect(bitsToI32(0xffffffff)).toBe(-1);
		// the bit pattern of int 1 is a float denormal, of int -1 a NaN: neither may leak into the integer
		expect(bitsToF32(1)).not.toBe(1);
		expect(Number.isNaN(bitsToF32(0xffffffff))).toBe(true);
		expect(bitsToI32(0xffffffff)).toBe(-1);
	});

	it('round-trips every INT32 corner and a sweep through the 4-byte float field of the write', () => {
		const values = [0, 1, -1, 2, -2, 255, 256, 65535, 2 ** 31 - 1, -(2 ** 31), 0x7f7fffff, -0x00800000, 1234567, -7654321];
		for (const v of values) {
			const bits = i32ToBits(v);
			expect(bitsToI32(bits)).toBe(v);
			if (carriedExactly(bits)) expect(wireNumberToBits(bitsToWireNumber(bits))).toBe(bits);
		}
	});

	it('round-trips REAL32 bit patterns through the write field, including infinities and the quiet NaN', () => {
		for (const bits of [0, 0x80000000, 1, 0x3f800000, 0x7f7fffff, 0xff7fffff, 0x7f800000, 0xff800000, 0x7fc00000, 0x00800000]) {
			expect(wireNumberToBits(bitsToWireNumber(bits))).toBe(bits);
		}
	});

	it('says which bit patterns the float field cannot carry (a signalling NaN is quietened on the way)', () => {
		expect(carriedExactly(0x7f800001)).toBe(false);
		expect(carriedExactly(0xff800001)).toBe(false);
		expect(carriedExactly(0xffffffff)).toBe(true); // int -1
		expect(carriedExactly(0x7fc00001)).toBe(true);
	});

	it('rejects an int outside the int32 range and a non-integer', () => {
		expect(() => i32ToBits(2 ** 31)).toThrow(RangeError);
		expect(() => i32ToBits(1.5)).toThrow(RangeError);
	});
});

describe('value text', () => {
	it('parses an int32 as a whole number only', () => {
		expect(parseValueText('-1', 'i32')).toEqual({ ok: true, bits: 0xffffffff });
		expect(parseValueText(' 7 ', 'i32')).toEqual({ ok: true, bits: 7 });
		expect(parseValueText('1.0', 'i32').ok).toBe(false);
		expect(parseValueText('1e3', 'i32').ok).toBe(false);
		expect(parseValueText('2147483648', 'i32').ok).toBe(false);
		expect(parseValueText('', 'i32').ok).toBe(false);
	});

	it('parses a float32 as a finite decimal that fits binary32', () => {
		expect(parseValueText('0.5', 'f32')).toEqual({ ok: true, bits: 0x3f000000 });
		expect(parseValueText('1e-3', 'f32').ok).toBe(true);
		expect(parseValueText('-.25', 'f32')).toEqual({ ok: true, bits: 0xbe800000 });
		expect(parseValueText('1e39', 'f32').ok).toBe(false);
		expect(parseValueText('NaN', 'f32').ok).toBe(false);
		expect(parseValueText('0x10', 'f32').ok).toBe(false);
		expect(parseValueText('1,5', 'f32').ok).toBe(false);
	});

	it('formats the shortest text that reads back as the same float32', () => {
		expect(formatF32(f32ToBits(0.732))).toBe('0.732');
		expect(formatF32(f32ToBits(12.5))).toBe('12.5');
		expect(formatF32(f32ToBits(0.1))).toBe('0.1');
		expect(formatValue(i32ToBits(-1), 'i32')).toBe('-1');
		expect(formatValue(i32ToBits(-1), 'f32')).toBe('NaN');
		expect(bitsHex(0x3f)).toBe('0x0000003f');
		for (const text of ['0.732', '1e-7', '123456.7', '3.4028235e38', '1.1754944e-38']) {
			const r = parseValueText(text, 'f32');
			if (!r.ok) throw new Error(text);
			expect(f32ToBits(Number(formatF32(r.bits)))).toBe(r.bits);
		}
	});
});
