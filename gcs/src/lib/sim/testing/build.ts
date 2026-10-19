// A sim record builder for the tests: writes version 1's layout (sim/gz/plugin/src/sim_telemetry.hpp) field by field, independently of
// the decoder, so that a test of the decoder is a test against the layout and not against itself.
import { SIM_RECORD_BYTES } from '../record';

export interface BuildOptions {
	seq?: number;
	flags?: number;
	simTimeNs?: number;
	tick?: number;
	rtf?: number;
	hil?: Partial<{ arming: number; statusRecords: number; frames: number; counter: number; mode: number; deadline: number; faults: number; reports: number; lagEvents: number; sticks: number[]; lq: number; marginMin: number; marginMax: number; lagUs: number }>;
	q?: number[];
	/** Replaces bytes of the finished record, for the refusal tests. */
	patch?: (v: DataView) => void;
}

/** Flag bits. */
export const FLAG = { imuModel: 1, launchPad: 2, ground: 4, hil: 8, padHeld: 16, padReleased: 32 } as const;

export function buildRecord(o: BuildOptions = {}): Uint8Array {
	const bytes = new Uint8Array(SIM_RECORD_BYTES);
	const v = new DataView(bytes.buffer);
	[..."MSIM"].forEach((c, i) => (bytes[i] = c.charCodeAt(0)));
	v.setUint16(4, 1, true);
	v.setUint16(6, SIM_RECORD_BYTES, true);
	v.setUint32(8, o.seq ?? 0, true);
	v.setUint32(12, o.flags ?? FLAG.imuModel | FLAG.launchPad | FLAG.ground, true);
	const t = o.simTimeNs ?? 2_500_000_000;
	v.setUint32(16, t % 2 ** 32, true);
	v.setUint32(20, Math.floor(t / 2 ** 32), true);
	v.setUint32(24, o.tick ?? 16000, true);
	v.setFloat64(32, o.rtf ?? 0.98, true);
	const doubles = [1.5, -2.5, -3.5, 0.1, 0.2, -0.3, ...(o.q ?? [1, 0, 0, 0]), 0.01, 0.02, 0.03, 0, 0, -9.81, 100, 200, 300, 400];
	doubles.forEach((d, i) => v.setFloat64(40 + 8 * i, d, true));
	[0.011, -0.012, 0.013, 0.1, 0.2, -9.7].forEach((f, i) => v.setFloat32(200 + 4 * i, f, true));
	v.setUint32(224, 0x1c0, true);
	[48, 500, 1000, 2047, 48, 499, 999, 2000].forEach((d, i) => v.setUint16(228 + 2 * i, d, true));
	v.setUint32(244, 2, true);
	v.setFloat32(248, 1.25, true);
	const h = o.hil ?? {};
	[h.arming ?? 7, h.statusRecords ?? 3, h.frames ?? 9, h.counter ?? 8, h.mode ?? 2, h.deadline ?? 0, h.faults ?? 0, h.reports ?? 5, h.lagEvents ?? 1].forEach((x, i) => v.setUint32(252 + 4 * i, x, true));
	(h.sticks ?? [0.1, -0.2, 0.3, 0.4]).forEach((x, i) => v.setFloat32(288 + 4 * i, x, true));
	for (let i = 0; i < 16; i++) v.setFloat32(304 + 4 * i, i / 16, true);
	[h.lq ?? 0.9, h.marginMin ?? 1200, h.marginMax ?? 2400, h.lagUs ?? 3000].forEach((x, i) => v.setFloat32(368 + 4 * i, x, true));
	o.patch?.(v);
	return bytes;
}
