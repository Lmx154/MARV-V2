// Three records the lockstep plugin itself published (the world of tests/regression/quad/L10/gz/test_sim_channel.py: the first host step, the
// middle and the last of a flight that lifts off), kept with the fields the Python reference decoder (tools/sim/sim_channel.py) read from
// the same bytes. The TypeScript decoder must read the same numbers: the three implementations of the layout (the plugin's writer, this
// decoder, the Python decoder) are checked against each other on real bytes, not on bytes this repository wrote by hand.
import { describe, expect, it } from 'vitest';
import fixture from './testing/plugin_records.json';
import { decodeSimRecord } from './record';

const bytes = (hex: string): Uint8Array => Uint8Array.from(hex.match(/../g)!.map((h) => Number.parseInt(h, 16)));

describe('the plugin\'s own records', () => {
	for (const [i, f] of fixture.entries()) {
		it(`record ${i}: every field equals what the Python reference decoder read`, () => {
			const r = decodeSimRecord(bytes(f.hex));
			const d = f.decoded;
			expect(r.seq).toBe(d.seq);
			expect(r.simTimeNs).toBe(d.sim_time_ns);
			expect(r.tick).toBe(d.tick);
			expect(r.rtf).toBe(d.rtf);
			expect([...r.posNed]).toEqual(d.pos_ned_m);
			expect([...r.velNed]).toEqual(d.vel_ned_m_s);
			expect([...r.q]).toEqual(d.q_wxyz);
			expect([...r.omegaFrd]).toEqual(d.omega_frd_rad_s);
			expect([...r.specificForceFrd]).toEqual(d.specific_force_frd_m_s2);
			expect([...r.rotorSpeed]).toEqual(d.rotor_speed_rad_s);
			expect([...r.imu.gyro]).toEqual(d.imu_gyro_rad_s);
			expect([...r.imu.accel]).toEqual(d.imu_accel_m_s2);
			expect(r.imu.flags).toBe(d.imu_flags);
			expect([...r.dshotCommanded]).toEqual(d.dshot_commanded);
			expect([...r.dshotApplied]).toEqual(d.dshot_applied);
			expect(r.touchdowns).toBe(d.touchdowns);
			expect(r.lastTouchdownDownMs).toBe(d.last_touchdown_down_m_s);
			expect(r.flags).toEqual({
				imuModel: d.flags.imu_model,
				launchPad: d.flags.launch_pad,
				ground: d.flags.ground,
				hil: d.flags.hil,
				padHeld: d.flags.pad_held,
				padReleased: d.flags.pad_released
			});
			expect(r.hil).toBeNull(); // a SIL record
		});
	}

	it('are a SIL flight: the pad holds the first record and has released by the last, with DShot above the minimum', () => {
		const first = decodeSimRecord(bytes(fixture[0]!.hex));
		const last = decodeSimRecord(bytes(fixture[fixture.length - 1]!.hex));
		expect(first.flags.padHeld).toBe(true);
		expect(last.flags.padReleased).toBe(true);
		expect(Math.max(...last.dshotCommanded)).toBeGreaterThan(48);
		expect(last.rtf).toBeGreaterThan(0);
	});
});
