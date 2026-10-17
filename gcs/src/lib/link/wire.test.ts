import { describe, expect, it } from 'vitest';
import { paramValueBits, paramValueFields } from './bits';
import { mavlinkCodec } from './mavlink';
import { decodeFrame } from './codec';

// INT32 parameters travel as their 4 bytes in the float field of PARAM_SET / PARAM_VALUE (decision 0053 Q1). A JavaScript
// float cannot hold a signalling NaN, so the codec carries the raw bits beside the numeric view.
const BOUNDARIES = [
	0x7f7fffff, 0x7f800000, 0x7f800001, 0x7f800002, 0x7fa00000, 0x7fbffffe, 0x7fbfffff, 0x7fc00000, 0x7fc00001, 0x7fffffff,
	0xff7fffff, 0xff800000, 0xff800001, 0xff800002, 0xffa00000, 0xffbffffe, 0xffbfffff, 0xffc00000, 0xffc00001, 0xffffffff,
	0, 1, 0x80000000, 0x80000001, 0x3f800000
];

describe('raw bits of a float field', () => {
	const codec = mavlinkCodec;
	const envelope = { systemId: 1, componentId: 2, sequence: 3 };
	const id = new TextEncoder().encode('rotor_yaw_s_2713');

	it.each(BOUNDARIES)('round-trips PARAM_SET and PARAM_VALUE with the pattern %i', (bits) => {
		const set = codec.encode(envelope, { kind: 'ParamSet', target_system: 1, target_component: 1, param_id: id, ...paramValueFields(bits), param_type: 6 });
		const gotSet = decodeFrame(codec.createParser(), set).frame?.message;
		expect(gotSet?.kind).toBe('ParamSet');
		if (gotSet?.kind === 'ParamSet') expect(paramValueBits(gotSet)).toBe(bits);
		const value = codec.encode(envelope, { kind: 'ParamValue', param_id: id, ...paramValueFields(bits), param_type: 6, param_count: 1, param_index: 0 });
		const gotValue = decodeFrame(codec.createParser(), value).frame?.message;
		expect(gotValue?.kind).toBe('ParamValue');
		if (gotValue?.kind === 'ParamValue') expect(paramValueBits(gotValue)).toBe(bits);
	});

	it('encodes the numeric view alone as before', () => {
		const a = codec.encode(envelope, { kind: 'ParamValue', param_id: id, param_value: 0.25, param_type: 9, param_count: 1, param_index: 0 });
		const b = codec.encode(envelope, { kind: 'ParamValue', param_id: id, ...paramValueFields(0x3e800000), param_type: 9, param_count: 1, param_index: 0 });
		expect(b).toEqual(a);
	});
});
