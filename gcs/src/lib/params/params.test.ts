// The manifest, the merge of manifest and vehicle, the staging rules and the manifest lookup in the image library.
import { describe, expect, it } from 'vitest';
import { f32ToBits, i32ToBits } from '$link/bits';
import type { VehicleParam } from '$link/vehicle';
import { findManifest, loadSelected, type ImageEntry, type ImageList } from './library';
import { parseManifest, parseProvenance } from './manifest';
import { matchesFilter, mergeParams, NO_GROUP, type ParamRow } from './merge';
import { checkStage, DERIVED_WARNING, LOCK_TOOLTIP_DEFAULT, lockTooltip, SOURCE_REQUIRED_MESSAGE, whyReadOnly, type StageContext } from './staging';
import { manifestJson, provenanceJson, SCHEMA, SPECS } from './testing/fixture';

const manifest = parseManifest(manifestJson());
const provenance = parseProvenance(provenanceJson());

function vehicleParam(linkId: string, patch: Partial<VehicleParam> = {}): VehicleParam {
	const spec = SPECS.findIndex((s) => s.linkId === linkId);
	const m = manifest.params[spec < 0 ? 0 : spec]!;
	return {
		index: spec, linkId, type: m.type, activeBits: m.defaultBits, stagedBits: m.defaultBits, sigma: m.sigma, origin: m.origin, method: m.method,
		sigmaKind: m.sigmaKind, locked: m.locked, hasRecord: true, sourceLength: m.source.length, source: m.source, sourcePartial: m.source, ...patch
	};
}

const allOnVehicle = (): VehicleParam[] => manifest.params.map((m) => vehicleParam(m.linkId));
const put = (v: VehicleParam[], p: VehicleParam): VehicleParam[] => v.map((x) => (x.linkId === p.linkId ? p : x));
const ctx: StageContext = { connected: true, identityKnown: true, manifestMatches: true, bytewise: true, armed: false };
const rowOf = (merged: ReturnType<typeof mergeParams>, name: string): ParamRow => merged.rows.find((r) => r.name === name)!;

describe('the manifest', () => {
	it('reads the generator\'s files', () => {
		expect(manifest.schemaHash).toBe(SCHEMA);
		expect(manifest.params).toHaveLength(SPECS.length);
		expect(manifest.params[2]).toMatchObject({ name: 'rotor_yaw_sign_m1', type: 'i32', defaultBits: 0xffffffff, linkId: 'rotor_yaw_s_2713' });
		expect(provenance.entries.find((e) => e.name === 'arm_throttle_min')?.lock).toEqual({ by: 'Luis', on: '2026-10-17', via: 'manual' });
	});

	it('says what is wrong with a file it cannot use', () => {
		const bad = manifestJson() as { params: Record<string, Record<string, unknown>> };
		bad.params.mass!.type = 'f64';
		expect(() => parseManifest(bad)).toThrow('neither f32 nor i32');
		const wrongCount = manifestJson() as { count: number };
		wrongCount.count = 99;
		expect(() => parseManifest(wrongCount)).toThrow('count is 99');
		expect(() => parseManifest({ schema_hash: '0x1', count: 0 })).toThrow('params');
		const unknownMethod = manifestJson() as { params: Record<string, Record<string, unknown>> };
		unknownMethod.params.mass!.method = 'guessed';
		expect(() => parseManifest(unknownMethod)).toThrow('method');
	});
});

describe('the merge of manifest and vehicle', () => {
	it('groups by source file in provenance order and merges the values of both sides', () => {
		const merged = mergeParams(manifest, provenance, allOnVehicle());
		expect(merged.groups.map((g) => g.name)).toEqual(['card.yaml', 'mixer.yaml', 'register.yaml']);
		expect(merged.issues).toEqual([]);
		const mass = rowOf(merged, 'mass');
		expect(mass).toMatchObject({ status: 'matched', activeBits: f32ToBits(0.732), stagedBits: f32ToBits(0.732), differs: false, factoryBits: f32ToBits(0.732), locked: false });
		expect(rowOf(merged, 'arm_throttle_min')).toMatchObject({ locked: true, lockInfo: { by: 'Luis', on: '2026-10-17' } });
		expect(rowOf(merged, 'rotor_yaw_sign_m1')).toMatchObject({ activeBits: 0xffffffff, type: 'i32' });
	});

	it('marks a staged value that differs from the active one', () => {
		const v = put(allOnVehicle(), vehicleParam('rate_kp', { stagedBits: f32ToBits(20) }));
		const merged = mergeParams(manifest, provenance, v);
		expect(rowOf(merged, 'rate_kp')).toMatchObject({ differs: true, activeBits: f32ToBits(12.5), stagedBits: f32ToBits(20) });
		expect(merged.rows.filter((r) => r.differs).map((r) => r.name)).toEqual(['rate_kp']);
	});

	it('takes provenance and source from the vehicle once it reports them', () => {
		const v = put(allOnVehicle(), vehicleParam('mass', { origin: 'Manual', method: 'scenario', source: 'set over the link', sigmaKind: 'Choice' }));
		const mass = rowOf(mergeParams(manifest, provenance, v), 'mass');
		expect(mass).toMatchObject({ origin: 'Manual', method: 'scenario', source: 'set over the link', sourceFromVehicle: true, sigmaKind: 'Choice' });
	});

	it('keeps the manifest\'s source while the vehicle\'s is not complete', () => {
		const v = put(allOnVehicle(), vehicleParam('inertia_xx', { source: null, sourcePartial: 'the long' }));
		const row = rowOf(mergeParams(manifest, provenance, v), 'inertia_xx');
		expect(row.sourceFromVehicle).toBe(false);
		expect(row.source).toBe(manifest.params[1]!.source);
		expect(row.sourceLength).toBe(manifest.params[1]!.source.length);
	});

	it('marks a manifest parameter the vehicle lacks and a vehicle parameter the manifest lacks', () => {
		const v = allOnVehicle().filter((p) => p.linkId !== 'mass');
		v.push(vehicleParam('rate_kp', { index: 40, linkId: 'surprise', type: 'f32' }));
		const merged = mergeParams(manifest, provenance, v);
		expect(rowOf(merged, 'mass')).toMatchObject({ status: 'manifest only', activeBits: null, factoryBits: f32ToBits(0.732) });
		const extra = rowOf(merged, 'surprise');
		expect(extra).toMatchObject({ status: 'vehicle only', group: NO_GROUP, factoryBits: null });
		expect(merged.issues.join(' ')).toContain('not in the manifest');
		expect(merged.issues.join(' ')).toContain('not on the vehicle');
	});

	it('marks a type or index disagreement, and treats a lock the manifest has as a lock', () => {
		const v = put(put(put(allOnVehicle(), vehicleParam('mass', { type: 'i32' })), vehicleParam('rate_kp', { index: 9 })), vehicleParam('arm_throttle_min', { locked: false }));
		const merged = mergeParams(manifest, provenance, v);
		expect(rowOf(merged, 'mass').status).toBe('mismatch');
		expect(rowOf(merged, 'rate_kp').status).toBe('mismatch');
		expect(rowOf(merged, 'arm_throttle_min')).toMatchObject({ locked: true, status: 'matched' });
		expect(rowOf(merged, 'arm_throttle_min').notes.join(' ')).toContain('treated as locked');
	});

	it('shows the manifest alone when no vehicle is connected', () => {
		const merged = mergeParams(manifest, provenance, null);
		expect(merged.rows).toHaveLength(SPECS.length);
		expect(rowOf(merged, 'mass')).toMatchObject({ status: 'matched', activeBits: null, stagedBits: null, differs: false });
		expect(merged.issues).toEqual([]);
	});

	it('flags provenance of another schema, and filters by words in any column', () => {
		const other = parseProvenance(provenanceJson(SPECS, '0x1111111111111111'));
		expect(mergeParams(manifest, other, null).issues[0]).toContain('different schemas');
		const merged = mergeParams(manifest, provenance, allOnVehicle());
		expect(merged.rows.filter((r) => matchesFilter(r, 'derived')).map((r) => r.name)).toEqual(['inertia_xx', 'rotor_yaw_sign_m1']);
		expect(merged.rows.filter((r) => matchesFilter(r, 'kg mass')).map((r) => r.name)).toEqual(['mass']);
		expect(merged.rows.filter((r) => matchesFilter(r, 'weighed'))).toHaveLength(1);
		expect(merged.rows.filter((r) => matchesFilter(r, ''))).toHaveLength(SPECS.length);
	});
});

describe('which rows can be edited', () => {
	const merged = mergeParams(manifest, provenance, allOnVehicle());
	const mass = rowOf(merged, 'mass');
	const locked = rowOf(merged, 'arm_throttle_min');

	it('allows a matched, unlocked row of a connected, disarmed vehicle with a matching manifest', () => {
		expect(whyReadOnly(mass, ctx)).toBeNull();
	});

	it('makes a locked row read-only with the ruling as its tooltip', () => {
		expect(whyReadOnly(locked, ctx)).toBe('locked by Luis 2026-10-17 — change only by a firmware build');
		expect(lockTooltip(locked)).toBe(LOCK_TOOLTIP_DEFAULT);
		const result = checkStage(locked, ctx, '0.5', 'a source', true);
		expect(result).toEqual({ ok: false, error: LOCK_TOOLTIP_DEFAULT });
	});

	it('makes every row read-only when the vehicle, the identity, the manifest or the capability is missing, or the vehicle is armed', () => {
		expect(whyReadOnly(mass, { ...ctx, connected: false })).toContain('heartbeat');
		expect(whyReadOnly(mass, { ...ctx, identityKnown: false })).toContain('identity');
		expect(whyReadOnly(mass, { ...ctx, manifestMatches: false })).toContain('schema');
		expect(whyReadOnly(mass, { ...ctx, bytewise: false })).toContain('byte-wise');
		expect(whyReadOnly(mass, { ...ctx, bytewise: null })).toContain('capabilities');
		expect(whyReadOnly(mass, { ...ctx, armed: true })).toContain('armed');
		expect(whyReadOnly(mass, { ...ctx, armed: null })).toContain('not known');
	});

	it('makes rows the manifest and the vehicle disagree on read-only', () => {
		const m = mergeParams(manifest, provenance, [vehicleParam('mass', { type: 'i32' }), vehicleParam('surprise', { index: 30, type: 'f32', linkId: 'surprise' })]);
		expect(whyReadOnly(rowOf(m, 'mass'), ctx)).toContain('disagree');
		expect(whyReadOnly(rowOf(m, 'surprise'), ctx)).toContain('not in the manifest');
		expect(whyReadOnly(rowOf(mergeParams(manifest, provenance, null), 'mass'), { ...ctx, connected: false })).not.toBeNull();
	});
});

describe('the rules of a stage request', () => {
	const merged = mergeParams(manifest, provenance, allOnVehicle());
	const kp = rowOf(merged, 'rate_kp');
	const inertia = rowOf(merged, 'inertia_xx');

	it('requires a source text', () => {
		for (const empty of ['', '  ', '\n']) {
			expect(checkStage(kp, ctx, '15', empty, false)).toEqual({ ok: false, error: SOURCE_REQUIRED_MESSAGE });
		}
		expect(checkStage(kp, ctx, '15', 'bench log', false)).toEqual({ ok: true, request: { bits: f32ToBits(15), source: 'bench log', needsDerivedAck: false } });
	});

	it('parses by the row\'s type: an int32 row takes whole numbers, as bits', () => {
		const yaw = rowOf(merged, 'rotor_yaw_sign_m1');
		expect(checkStage(yaw, ctx, '1', 's', true)).toMatchObject({ ok: true, request: { bits: i32ToBits(1) } });
		expect(checkStage(yaw, ctx, '1.0', 's', true)).toMatchObject({ ok: false });
		expect(checkStage(kp, ctx, 'abc', 's', false)).toMatchObject({ ok: false });
	});

	it('refuses an unchanged value', () => {
		expect(checkStage(kp, ctx, '12.5', 's', false)).toMatchObject({ ok: false, error: expect.stringContaining('unchanged') });
	});

	it('warns about editing a derived value and wants it acknowledged', () => {
		expect(inertia.method).toBe('derived');
		expect(checkStage(inertia, ctx, '0.005', 'measured with a pendulum', false)).toMatchObject({ ok: false, error: expect.stringContaining('derived') });
		expect(checkStage(inertia, ctx, '0.005', 'measured with a pendulum', true)).toMatchObject({ ok: true, request: { needsDerivedAck: true } });
		expect(DERIVED_WARNING).toContain('derived');
		expect(checkStage(kp, ctx, '15', 's', false)).toMatchObject({ ok: true, request: { needsDerivedAck: false } });
	});
});

describe('finding the manifest of the connected image', () => {
	const entry = (hash: string, tree: string, withManifest = true): ImageEntry => ({
		hash, tree, name: 'marv_sil_quad_fc', kind: 'sil', params_set: 'marv_params_fc3v2',
		artifacts: withManifest ? { image: 'x', params_manifest: 'm', params_provenance: 'p' } : { image: 'x' }
	});
	const list: ImageList = { images: [entry('a'.repeat(64), 'host-gz'), entry('b'.repeat(64), 'm33'), entry('c'.repeat(64), 'bare', false)], unkeyed: [] };
	const calls: string[] = [];
	const fetchArtifact = async (hash: string, name: string): Promise<unknown> => {
		calls.push(`${hash[0]}:${name}`);
		return name === 'params_manifest' ? manifestJson(SPECS, hash[0] === 'b' ? '0x2222222222222222' : SCHEMA) : provenanceJson(SPECS, hash[0] === 'b' ? '0x2222222222222222' : SCHEMA);
	};

	it('prefers the image hash', async () => {
		calls.length = 0;
		const found = await findManifest(list, 'b'.repeat(64), SCHEMA, fetchArtifact);
		expect(found?.how).toBe('image hash');
		expect(found?.entry.tree).toBe('m33');
		expect(calls).toEqual(['b:params_manifest', 'b:params_provenance']);
	});

	it('falls back to the schema hash', async () => {
		const found = await findManifest(list, 'f'.repeat(64), SCHEMA, fetchArtifact);
		expect(found?.how).toBe('schema hash');
		expect(found?.entry.tree).toBe('host-gz');
	});

	it('finds nothing for an unknown image and schema, and for an image without a manifest', async () => {
		expect(await findManifest(list, 'f'.repeat(64), '0x9999999999999999', fetchArtifact)).toBeNull();
		expect(await findManifest(list, 'c'.repeat(64), null, fetchArtifact)).toBeNull();
	});

	it('loads the manifest of a picked build tree', async () => {
		const loaded = await loadSelected(list.images[1]!, fetchArtifact);
		expect(loaded.how).toBe('selected');
		expect(loaded.manifest.schemaHash).toBe('0x2222222222222222');
	});
});
