// The G7 gate in the GUI: the staged view, the report id (SHA-256 of the report bytes, checked with WebCrypto) and every refusal of
// whyCannotActivate.
import { describe, expect, it } from 'vitest';
import { f32ToBits } from '$link/bits';
import type { VehicleParam } from '$link/vehicle';
import { parseManifest } from './manifest';
import { manifestJson, SCHEMA, SPECS } from './testing/fixture';
import {
	buildStagedView,
	makeRecord,
	paramsWithoutFullSource,
	parseReport,
	reportIdMatches,
	sha256Hex,
	stagedFingerprint,
	whyCannotActivate,
	whyCannotBuildView,
	type ActivationFacts,
	type ValidationRecord
} from './validation';

const manifest = parseManifest(manifestJson());
const IMAGE = 'a1'.repeat(32);
const OTHER_IMAGE = 'b2'.repeat(32);
const CONFIG = 'c3'.repeat(32);
const OTHER_CONFIG = 'd4'.repeat(32);
const encoder = new TextEncoder();

function vehicleParams(patches: Record<string, Partial<VehicleParam>> = {}): VehicleParam[] {
	return manifest.params.map((m) => ({
		index: m.id, linkId: m.linkId, type: m.type, activeBits: m.defaultBits, stagedBits: m.defaultBits, sigma: m.sigma, sigmaBits: f32ToBits(m.sigma),
		origin: m.origin, method: m.method, sigmaKind: m.sigmaKind, locked: m.locked, hasRecord: true, sourceLength: m.source.length, source: m.source,
		sourcePartial: m.source, ...patches[m.linkId]
	}));
}

function reportBytes(over: { verdict?: string; image?: string; config?: string; format?: string } = {}): Uint8Array {
	return encoder.encode(
		JSON.stringify({
			format: over.format ?? 'marv-validation-report/1',
			verdict: over.verdict ?? 'PASS',
			image: { sha256: over.image ?? IMAGE },
			config: { config_hash: over.config ?? CONFIG },
			checks: [{ id: 'records', title: 'record checks', result: 'PASS', values: ['137 records'], failures: [] }]
		}) + '\n'
	);
}

async function record(bytes = reportBytes(), over: Partial<ValidationRecord> = {}): Promise<ValidationRecord> {
	const made = await makeRecord(IMAGE, CONFIG, await sha256Hex(bytes), bytes, 'fp');
	return { ...made, ...over };
}

const facts = async (over: Partial<ActivationFacts> = {}): Promise<ActivationFacts> => ({
	record: await record(),
	vehicleImageSha256: IMAGE,
	stagedConfigHash: CONFIG,
	fingerprint: 'fp',
	...over
});

describe('the staged view', () => {
	it('is the mcfg JSON view of the staged records in id order, with bits for value and sigma', () => {
		const params = vehicleParams({ inertia_xx: { stagedBits: f32ToBits(0.005) }, rotor_yaw_s_2713: { stagedBits: 0xffffffff } });
		const view = buildStagedView(manifest, params);
		expect(view).toMatchObject({ format_version: 1, sequence: 0, image_sha256: '0'.repeat(64), report_id: '0'.repeat(64) });
		expect(view.sections).toHaveLength(1);
		const section = view.sections[0]!;
		expect(section).toMatchObject({ kind: 'quad', schema_hash: SCHEMA, record_count: SPECS.length });
		expect(section.records.map((r) => r.source)).toEqual(SPECS.map((s) => s.source));
		expect(section.records[1]).toEqual({
			type: 'f32', value_bits: '0x' + f32ToBits(0.005).toString(16).padStart(8, '0'), sigma_bits: '0x' + f32ToBits(0.00107).toString(16).padStart(8, '0'),
			sigma_kind: 'Known', method: 'derived', origin: 'DefaultFromCard', locked: false, source: SPECS[1]!.source
		});
		expect(section.records[2]).toMatchObject({ type: 'i32', value_bits: '0xffffffff', sigma_bits: '0x00000000', sigma_kind: 'Exact' });
		expect(section.records[3]!.locked).toBe(true);
	});

	it('is refused while a record, a value or a full source text is missing', () => {
		expect(whyCannotBuildView(manifest, vehicleParams())).toBeNull();
		expect(whyCannotBuildView(manifest, vehicleParams().slice(1))).toContain('has not reported');
		expect(whyCannotBuildView(manifest, vehicleParams({ mass: { hasRecord: false } }))).toContain('record of mass');
		expect(whyCannotBuildView(manifest, vehicleParams({ mass: { stagedBits: null } }))).toContain('record of mass');
		expect(whyCannotBuildView(manifest, vehicleParams({ mass: { linkId: 'other' } }))).toContain('manifest');
		const partial = vehicleParams({ mass: { source: null } });
		expect(whyCannotBuildView(manifest, partial)).toContain('full source text of mass');
		expect(() => buildStagedView(manifest, partial)).toThrow('full source text of mass');
		expect(whyCannotBuildView(manifest, partial, false)).toBeNull();
		expect(paramsWithoutFullSource(manifest, partial)).toEqual([0]);
	});

	it('changes its fingerprint with any staged bit, sigma, provenance or source', () => {
		const base = stagedFingerprint(vehicleParams());
		expect(stagedFingerprint(vehicleParams())).toBe(base);
		for (const patch of [{ stagedBits: 1 }, { sigmaBits: 1 }, { origin: 'Manual' as const }, { method: 'scenario' as const }, { sigmaKind: 'Choice' as const }, { source: 'other' }, { source: null }]) {
			expect(stagedFingerprint(vehicleParams({ mass: patch }))).not.toBe(base);
		}
		expect(stagedFingerprint(vehicleParams({ mass: { activeBits: 7 } }))).toBe(base);
	});
});

describe('the report id', () => {
	it('is the SHA-256 of the report bytes', async () => {
		const bytes = reportBytes();
		const id = await sha256Hex(bytes);
		expect(id).toHaveLength(64);
		expect(await reportIdMatches(bytes, id)).toBe(true);
		expect(await sha256Hex(encoder.encode('abc'))).toBe('ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad');
	});

	it('does not match another report, an altered byte, another case or a short id', async () => {
		const bytes = reportBytes();
		const id = await sha256Hex(bytes);
		expect(await reportIdMatches(reportBytes({ verdict: 'FAIL' }), id)).toBe(false);
		const altered = bytes.slice();
		altered[altered.length - 2] = (altered[altered.length - 2] ?? 0) ^ 1;
		expect(await reportIdMatches(altered, id)).toBe(false);
		expect(await reportIdMatches(bytes, id.toUpperCase())).toBe(false);
		expect(await reportIdMatches(bytes, id.slice(0, 32))).toBe(false);
		expect(await reportIdMatches(bytes, '')).toBe(false);
	});

	it('reads the verdict, the hashes and the checks of a report, and refuses a document that is not one', () => {
		expect(parseReport(reportBytes())).toMatchObject({ verdict: 'PASS', imageSha256: IMAGE, configHash: CONFIG, checks: [{ id: 'records', result: 'PASS', values: ['137 records'] }] });
		expect(() => parseReport(encoder.encode('not json'))).toThrow('not JSON');
		expect(() => parseReport(encoder.encode('[]'))).toThrow('not a JSON object');
		expect(() => parseReport(reportBytes({ format: 'other/1' }))).toThrow('format');
	});
});

describe('whyCannotActivate', () => {
	it('allows a PASS report of this image and this staged config', async () => {
		expect(whyCannotActivate(await facts())).toBeNull();
	});

	it('refuses without a report', async () => {
		expect(whyCannotActivate(await facts({ record: null }))).toContain('no validation report');
	});

	it('refuses a report that says FAIL, or says nothing', async () => {
		expect(whyCannotActivate(await facts({ record: await record(reportBytes({ verdict: 'FAIL' })) }))).toContain('says FAIL');
		expect(whyCannotActivate(await facts({ record: await record(reportBytes({ verdict: '' })) }))).toContain('no verdict');
	});

	it('refuses a report that cannot be read', async () => {
		expect(whyCannotActivate(await facts({ record: await record(encoder.encode('garbage')) }))).toContain('cannot be read');
	});

	it('refuses a report whose id is not the SHA-256 of its bytes', async () => {
		const forged = await record(reportBytes(), { reportId: '0'.repeat(64) });
		expect(whyCannotActivate(await facts({ record: forged }))).toContain('SHA-256');
		const notHex = await record(reportBytes(), { reportId: 'zz', computedId: 'zz' });
		expect(whyCannotActivate(await facts({ record: notHex }))).toContain('SHA-256');
		const swapped = await record(reportBytes({ verdict: 'PASS' }), { reportId: await sha256Hex(reportBytes({ verdict: 'FAIL' })) });
		expect(whyCannotActivate(await facts({ record: swapped }))).toContain('SHA-256');
	});

	it('refuses a report for another image hash, in the report or in the request', async () => {
		expect(whyCannotActivate(await facts({ record: await record(reportBytes({ image: OTHER_IMAGE })) }))).toContain('image');
		expect(whyCannotActivate(await facts({ vehicleImageSha256: OTHER_IMAGE }))).toContain('image');
		expect(whyCannotActivate(await facts({ record: await record(reportBytes(), { imageHash: OTHER_IMAGE }) }))).toContain('image');
		expect(whyCannotActivate(await facts({ vehicleImageSha256: null }))).toContain('image hash');
	});

	it('refuses a report for another config hash, in the report or in the request', async () => {
		expect(whyCannotActivate(await facts({ record: await record(reportBytes({ config: OTHER_CONFIG })) }))).toContain('config');
		expect(whyCannotActivate(await facts({ stagedConfigHash: OTHER_CONFIG }))).toContain('config');
		expect(whyCannotActivate(await facts({ record: await record(reportBytes(), { configHash: OTHER_CONFIG }) }))).toContain('config');
		expect(whyCannotActivate(await facts({ stagedConfigHash: null }))).toContain('has not been computed');
	});

	it('refuses when the staged config changed since the validation', async () => {
		expect(whyCannotActivate(await facts({ fingerprint: 'fp changed' }))).toContain('changed since it was validated');
	});
});
