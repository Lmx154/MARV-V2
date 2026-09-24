/**
 * Local NED about a geodetic origin, the same flat earth as the FC's marv::LocalFrame (fsw/include/marv/fsw/geo.hpp):
 * WGS84 meridian and prime-vertical radii at the origin's latitude, plus its altitude; e7 differences first.
 */
import type { GeoPoint, LatLonAlt } from './types';

const A = 6378137;
const E2 = 6.69437999014e-3;
const RAD_PER_E7 = (1e-7 * Math.PI) / 180;

export type Ned = [number, number, number];

export const toE7 = (deg: number): number => Math.round(deg * 1e7);
export const fromE7 = (e7: number): number => e7 / 1e7;

/** std::lround: half away from zero. */
const lround = (x: number): number => Math.sign(x) * Math.round(Math.abs(x));

export class LocalFrame {
	private readonly mPerE7N: number;
	private readonly mPerE7E: number;

	constructor(readonly origin: GeoPoint) {
		const lat = origin.lat_e7 * RAD_PER_E7;
		const s = Math.sin(lat);
		const d = 1 - E2 * s * s;
		this.mPerE7N = ((A * (1 - E2)) / (d * Math.sqrt(d)) + origin.alt_m) * RAD_PER_E7;
		this.mPerE7E = (A / Math.sqrt(d) + origin.alt_m) * Math.cos(lat) * RAD_PER_E7;
	}

	toNed(p: GeoPoint): Ned {
		const o = this.origin;
		return [(p.lat_e7 - o.lat_e7) * this.mPerE7N, (p.lon_e7 - o.lon_e7) * this.mPerE7E, -(p.alt_m - o.alt_m)];
	}

	toGeo(ned: Ned): GeoPoint {
		const o = this.origin;
		return { lat_e7: o.lat_e7 + lround(ned[0] / this.mPerE7N), lon_e7: o.lon_e7 + lround(ned[1] / this.mPerE7E), alt_m: o.alt_m - ned[2] };
	}

	/** NED of a point given in degrees and metres above the origin. */
	nedOf(p: LatLonAlt): Ned {
		return this.toNed({ lat_e7: toE7(p.lat), lon_e7: toE7(p.lon), alt_m: this.origin.alt_m + p.alt_m });
	}

	/** Degrees and metres above the origin of a NED point. */
	latLonOf(ned: Ned): LatLonAlt {
		const g = this.toGeo(ned);
		return { lat: fromE7(g.lat_e7), lon: fromE7(g.lon_e7), alt_m: g.alt_m - this.origin.alt_m };
	}
}
