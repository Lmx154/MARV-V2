import { describe, expect, it } from 'vitest';
import { LocalFrame } from './geo';

// The numbers of tests/test_geo.cpp: the Gazebo world's origin; 1e-4 deg at 47.376 deg, 408 m is 11.11853 m north,
// 7.55251 m east (WGS84 radii, computed in double).
const f = new LocalFrame({ lat_e7: 473763880, lon_e7: 85477780, alt_m: 408 });

describe('LocalFrame (port of marv::LocalFrame)', () => {
	it('metres per 1e-4 deg north and east match test_geo.cpp', () => {
		let v = f.toNed({ lat_e7: 473764880, lon_e7: 85477780, alt_m: 408 });
		expect(Math.abs(v[0] - 11.11853)).toBeLessThan(0.001);
		expect(Math.abs(v[1])).toBeLessThan(1e-6);
		v = f.toNed({ lat_e7: 473763880, lon_e7: 85478780, alt_m: 410 });
		expect(Math.abs(v[1] - 7.55251)).toBeLessThan(0.001);
		expect(v[2]).toBe(-2);
	});

	it('round trips within one e7 step', () => {
		const v = f.toNed(f.toGeo([123.4, -56.7, -8.9]));
		expect(Math.abs(v[0] - 123.4)).toBeLessThan(0.012);
		expect(Math.abs(v[1] + 56.7)).toBeLessThan(0.012);
		expect(Math.abs(v[2] + 8.9)).toBeLessThan(1e-4);
	});

	it('degrees and metres above home convert both ways', () => {
		const p = f.latLonOf([11.11853, 7.55251, -30]);
		expect(p.lat).toBeCloseTo(47.376488, 7);
		expect(p.lon).toBeCloseTo(8.547878, 7);
		expect(p.alt_m).toBeCloseTo(30, 6);
		const n = f.nedOf({ lat: 47.376388, lon: 8.547778, alt_m: 12 });
		expect(n[0]).toBeCloseTo(0, 9);
		expect(n[1]).toBeCloseTo(0, 9);
		expect(n[2]).toBeCloseTo(-12, 9);
	});
});
