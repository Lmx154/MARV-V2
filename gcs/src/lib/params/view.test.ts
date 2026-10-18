// The parameter page's pure logic: sorting, counting, the factory/active/staged comparison and the "About this parameter" text.
import { describe, expect, it } from 'vitest';
import { f32ToBits } from '$link/bits';
import type { VehicleParam } from '$link/vehicle';
import { parseManifest, parseProvenance } from './manifest';
import { mergeParams } from './merge';
import { manifestJson, provenanceJson, SPECS } from './testing/fixture';
import { aboutParam, countRows, LOCK_RULING, nextSort, sortRows } from './view';

const manifest = parseManifest(manifestJson());
const provenance = parseProvenance(provenanceJson());

function vehicleParam(linkId: string, patch: Partial<VehicleParam> = {}): VehicleParam {
	const spec = SPECS.findIndex((s) => s.linkId === linkId);
	const m = manifest.params[spec]!;
	return {
		index: spec, linkId, type: m.type, activeBits: m.defaultBits, stagedBits: m.defaultBits, sigma: m.sigma, origin: m.origin, method: m.method,
		sigmaKind: m.sigmaKind, locked: m.locked, hasRecord: true, sourceLength: m.source.length, source: m.source, sourcePartial: m.source, ...patch
	};
}
const onVehicle = (patches: Record<string, Partial<VehicleParam>> = {}): VehicleParam[] => manifest.params.map((m) => vehicleParam(m.linkId, patches[m.linkId]));
const names = (rows: { name: string }[]): string[] => rows.map((r) => r.name);

describe('factory, active and staged', () => {
	it('shows the factory value without a vehicle, with no active or staged value', () => {
		const rows = mergeParams(manifest, provenance, null).rows;
		expect(rows.every((r) => r.factoryBits !== null && r.activeBits === null && r.stagedBits === null && !r.changed && !r.differs)).toBe(true);
	});

	it('marks an active value that left the factory value, and a staged one that left the active value', () => {
		const merged = mergeParams(manifest, provenance, onVehicle({
			mass: { activeBits: f32ToBits(0.8), stagedBits: f32ToBits(0.8) },
			rate_kp: { stagedBits: f32ToBits(20) }
		}));
		const row = (n: string) => merged.rows.find((r) => r.name === n)!;
		expect(row('mass')).toMatchObject({ changed: true, differs: false });
		expect(row('rate_kp')).toMatchObject({ changed: false, differs: true });
		expect(row('inertia_xx')).toMatchObject({ changed: false, differs: false });
		expect(countRows(merged.rows)).toEqual({ total: 6, locked: 1, derived: 2, staged: 1, changed: 1 });
	});
});

describe('sorting', () => {
	const rows = mergeParams(manifest, provenance, onVehicle({ mass: { activeBits: f32ToBits(0.8), stagedBits: f32ToBits(0.8) } })).rows;

	it('sorts by name and by number, ascending then descending', () => {
		expect(names(sortRows(rows, { key: 'name', direction: 'asc' }))).toEqual(['arm_throttle_min', 'estimator_gyro_bias_x', 'inertia_xx', 'mass', 'rate_kp', 'rotor_yaw_sign_m1']);
		expect(names(sortRows(rows, { key: 'staged', direction: 'asc' }))[0]).toBe('rotor_yaw_sign_m1'); // -1
		expect(names(sortRows(rows, { key: 'staged', direction: 'desc' }))[0]).toBe('rate_kp'); // 12.5
	});

	it('keeps unknown values last in either direction and the manifest order for ties', () => {
		const known = rows.filter((r) => r.sigmaKind === 'Known');
		expect(known).toHaveLength(2);
		for (const direction of ['asc', 'desc'] as const) {
			const sorted = sortRows(rows, { key: 'sigma', direction });
			expect(names(sorted).slice(0, 2).sort()).toEqual(names(known).sort());
		}
		expect(names(sortRows(rows, { key: 'sigma', direction: 'asc' })).slice(2)).toEqual(['mass', 'rotor_yaw_sign_m1', 'arm_throttle_min', 'rate_kp']);
		expect(sortRows(rows, null)).toBe(rows);
	});

	it('cycles ascending, descending, off', () => {
		const a = nextSort(null, 'name');
		expect(a).toEqual({ key: 'name', direction: 'asc' });
		const d = nextSort(a, 'name');
		expect(d).toEqual({ key: 'name', direction: 'desc' });
		expect(nextSort(d, 'name')).toBeNull();
		expect(nextSort(d, 'unit')).toEqual({ key: 'unit', direction: 'asc' });
	});
});

describe('about this parameter', () => {
	const rows = mergeParams(manifest, provenance, null).rows;
	const about = (n: string) => aboutParam(rows.find((r) => r.name === n)!);

	it('gives the derivation text of a derived value only', () => {
		expect(about('inertia_xx').derivation).toBe(SPECS[1]!.source);
		expect(about('mass').derivation).toBeNull();
	});

	it('says what the link id is', () => {
		expect(about('mass')).toMatchObject({ linkId: 'mass', index: expect.stringContaining('#0') });
		expect(about('mass').linkIdNote).toContain('full name');
		expect(about('rotor_yaw_sign_m1').linkId).toBe('rotor_yaw_s_2713');
		expect(about('rotor_yaw_sign_m1').linkIdNote).toContain('SHA-256');
	});

	it('quotes the lock and its ruling', () => {
		expect(about('arm_throttle_min').lock).toBe(`Locked by Luis on 2026-10-17 (via manual). ${LOCK_RULING}`);
		expect(about('mass').lock).toBeNull();
	});
});
