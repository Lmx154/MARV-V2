import { describe, expect, it } from 'vitest';
import { decodeSimRecord, SIM_RECORD_BYTES, SimRecordError } from './record';
import { buildRecord, FLAG } from './testing/build';

describe('the sim record decoder', () => {
	it('reads every field of version 1 at the layout of sim_telemetry.hpp', () => {
		const r = decodeSimRecord(buildRecord({ seq: 7, simTimeNs: 5_000_000_123, tick: 4242, rtf: 0.75, flags: FLAG.imuModel | FLAG.launchPad | FLAG.padReleased }));
		expect(r.version).toBe(1);
		expect(r.seq).toBe(7);
		expect(r.simTimeNs).toBe(5_000_000_123); // above 2^32: the high word counts
		expect(r.tick).toBe(4242);
		expect(r.rtf).toBe(0.75);
		expect(r.flags).toEqual({ imuModel: true, launchPad: true, ground: false, hil: false, padHeld: false, padReleased: true });
		expect(r.posNed).toEqual([1.5, -2.5, -3.5]);
		expect(r.velNed[1]).toBeCloseTo(0.2, 12);
		expect(r.q).toEqual([1, 0, 0, 0]);
		expect(r.omegaFrd[2]).toBeCloseTo(0.03, 12);
		expect(r.specificForceFrd).toEqual([0, 0, -9.81]);
		expect(r.rotorSpeed).toEqual([100, 200, 300, 400]);
		expect(r.imu.gyro[0]).toBeCloseTo(0.011, 6);
		expect(r.imu.accel[2]).toBeCloseTo(-9.7, 5);
		expect(r.imu.flags).toBe(0x1c0);
		expect(r.dshotCommanded).toEqual([48, 500, 1000, 2047]);
		expect(r.dshotApplied).toEqual([48, 499, 999, 2000]);
		expect(r.touchdowns).toBe(2);
		expect(r.lastTouchdownDownMs).toBe(1.25);
	});

	it('has the HIL block only with the hil flag, and names the board\'s arming state', () => {
		expect(decodeSimRecord(buildRecord()).hil).toBeNull(); // control: a SIL record
		const r = decodeSimRecord(buildRecord({ flags: FLAG.hil, hil: { arming: 10, mode: 2, deadline: 4, marginMin: -30 } }));
		expect(r.hil).not.toBeNull();
		expect(r.hil?.armingState).toBe(10);
		expect(r.hil?.armingName).toBe('armed');
		expect(r.hil?.mode).toBe('angle');
		expect(r.hil?.deadlineMisses).toBe(4);
		expect(r.hil?.marginMinUs).toBe(-30);
		expect(r.hil?.channels).toHaveLength(16);
		expect(r.hil?.channels[8]).toBeCloseTo(0.5, 6);
		expect(r.hil?.sticks[3]).toBeCloseTo(0.4, 6);
	});

	it('does not know the arming state before the first status record, and calls an unknown code unknown', () => {
		const none = decodeSimRecord(buildRecord({ flags: FLAG.hil, hil: { statusRecords: 0 } }));
		expect(none.hil?.armingState).toBeNull();
		expect(none.hil?.armingName).toBeNull();
		expect(decodeSimRecord(buildRecord({ flags: FLAG.hil, hil: { arming: 200 } })).hil?.armingName).toBe('unknown');
		expect(decodeSimRecord(buildRecord({ flags: FLAG.hil, hil: { mode: 9 } })).hil?.mode).toBe('unknown');
	});

	it('refuses what is not a version 1 record: magic, version, length', () => {
		const good = buildRecord();
		expect(() => decodeSimRecord(good)).not.toThrow(); // control
		expect(() => decodeSimRecord(good.slice(0, SIM_RECORD_BYTES - 1))).toThrow(SimRecordError);
		expect(() => decodeSimRecord(new Uint8Array(0))).toThrow(SimRecordError);
		expect(() => decodeSimRecord(buildRecord({ patch: (v) => v.setUint8(0, 0x58) }))).toThrow(/magic/);
		expect(() => decodeSimRecord(buildRecord({ patch: (v) => v.setUint16(4, 2, true) }))).toThrow(/version 2/);
		expect(() => decodeSimRecord(buildRecord({ patch: (v) => v.setUint16(6, SIM_RECORD_BYTES - 4, true) }))).toThrow(/length/);
		const longer = new Uint8Array(SIM_RECORD_BYTES + 8);
		longer.set(good);
		new DataView(longer.buffer).setUint16(6, longer.length, true);
		expect(decodeSimRecord(longer).seq).toBe(0); // a later version appends fields; this one reads what it knows
		longer[6] = 0; // the length field no longer matches the datagram
		expect(() => decodeSimRecord(longer)).toThrow(SimRecordError);
	});

	it('reads a record at an offset inside a larger buffer', () => {
		const inner = buildRecord({ seq: 3 });
		const outer = new Uint8Array(inner.length + 10);
		outer.set(inner, 5);
		expect(decodeSimRecord(outer.subarray(5, 5 + inner.length)).seq).toBe(3);
	});
});
