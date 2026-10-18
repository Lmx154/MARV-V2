// The G7 gate in the GUI (core G7, decision 0053 Q2 and unit 9): the staged config the vehicle holds is rebuilt here as the JSON
// view of tools/config/mcfg.py (the format's single source), the helper validates that view (tools/validate/validate_config.py)
// and the GCS activates only with a passing report for exactly this (image hash, config hash) pair. A report's id is the SHA-256 of
// the report's bytes; this file checks it again with WebCrypto rather than trust the helper's word for it.
import { bitsHex } from '$link/bits';
import type { VehicleParam } from '$link/vehicle';
import type { Manifest } from './manifest';

/** The section of the quad parameter sets (tools/validate/validate_config.py SECTION): the image library holds no other kind. */
export const CONFIG_SECTION_KIND = 'quad';
/** tools/config/mcfg.py FORMAT_VERSION. */
const FORMAT_VERSION = 1;
const ZERO_DIGEST = '0'.repeat(64);
export const REPORT_FORMAT = 'marv-validation-report/1';

export interface ConfigViewRecord {
	type: string;
	value_bits: string;
	sigma_bits: string;
	sigma_kind: string;
	method: string;
	origin: string;
	locked: boolean;
	source: string;
}

/** tools/config/mcfg.py view(): bits, enumerations and source texts; names and decoded values are not needed to read it back. */
export interface ConfigView {
	format_version: number;
	sequence: number;
	image_sha256: string;
	report_id: string;
	sections: Array<{ kind: string; schema_hash: string; record_count: number; records: ConfigViewRecord[] }>;
}

/** Why the staged config cannot be built into a view (a record or a value is not known yet; with `needSources` also a source text read in full), or null. */
export function whyCannotBuildView(manifest: Manifest, params: VehicleParam[], needSources = true): string | null {
	const byIndex = new Map(params.map((p) => [p.index, p]));
	for (const m of manifest.params) {
		const p = byIndex.get(m.id);
		if (!p) return `the vehicle has not reported ${m.name}`;
		if (p.linkId !== m.linkId) return `parameter ${m.id} is ${p.linkId ?? 'unknown'} on the vehicle and ${m.linkId} in the manifest`;
		if (!p.hasRecord || p.stagedBits === null || p.sigmaBits === null || p.type === null) return `the provenance record of ${m.name} has not arrived`;
		if (p.origin === null || p.method === null || p.sigmaKind === null || p.locked === null) return `the provenance record of ${m.name} is incomplete`;
		if (needSources && p.source === null) return `the full source text of ${m.name} has not been read`;
	}
	return null;
}

/** The parameters whose full source text is still to be read (a record carries only the first chunk). */
export function paramsWithoutFullSource(manifest: Manifest, params: VehicleParam[]): number[] {
	const byIndex = new Map(params.map((p) => [p.index, p]));
	return manifest.params.filter((m) => byIndex.get(m.id)?.source === null).map((m) => m.id);
}

/** The staged config as the mcfg JSON view: one section of the manifest's schema, the records in id order with the STAGED values. */
export function buildStagedView(manifest: Manifest, params: VehicleParam[]): ConfigView {
	const why = whyCannotBuildView(manifest, params);
	if (why) throw new Error(why);
	const byIndex = new Map(params.map((p) => [p.index, p]));
	const records = manifest.params.map((m): ConfigViewRecord => {
		const p = byIndex.get(m.id) as VehicleParam;
		return {
			type: p.type as string,
			value_bits: bitsHex(p.stagedBits as number),
			sigma_bits: bitsHex(p.sigmaBits as number),
			sigma_kind: p.sigmaKind as string,
			method: p.method as string,
			origin: p.origin as string,
			locked: p.locked as boolean,
			source: p.source as string
		};
	});
	return {
		format_version: FORMAT_VERSION,
		sequence: 0,
		image_sha256: ZERO_DIGEST,
		report_id: ZERO_DIGEST,
		sections: [{ kind: CONFIG_SECTION_KIND, schema_hash: manifest.schemaHash, record_count: records.length, records }]
	};
}

/**
 * Everything the activation depends on, as one string: the staged bits, sigma, provenance and source of every parameter the vehicle
 * reported. It is compared for equality only, so that any change of the staged config after a validation is noticed at once
 * (without waiting for the helper to hash it again). A source text not read in full counts as null: a validation reads all of them,
 * so a later null means the record was rewritten.
 */
export function stagedFingerprint(params: VehicleParam[]): string {
	return JSON.stringify(
		[...params]
			.sort((a, b) => a.index - b.index)
			.map((p) => [p.index, p.linkId, p.stagedBits, p.sigmaBits, p.sigmaKind, p.method, p.origin, p.locked, p.sourceLength, p.source])
	);
}

// ---- the report ---------------------------------------------------------------------------------------------------------

export interface ReportCheck {
	id: string;
	title: string;
	result: string;
	values: string[];
	failures: string[];
}

export interface ValidationReport {
	format: string;
	verdict: string;
	imageSha256: string;
	configHash: string;
	checks: ReportCheck[];
}

const isObject = (x: unknown): x is Record<string, unknown> => typeof x === 'object' && x !== null && !Array.isArray(x);
const strings = (x: unknown): string[] => (Array.isArray(x) ? x.filter((v): v is string => typeof v === 'string') : []);

/** Reads the report bytes (tools/validate/validate_config.py's canonical JSON); throws with the reason for anything else. */
export function parseReport(bytes: Uint8Array): ValidationReport {
	let doc: unknown;
	try {
		doc = JSON.parse(new TextDecoder('utf-8', { fatal: true }).decode(bytes));
	} catch {
		throw new Error('the report is not JSON');
	}
	if (!isObject(doc)) throw new Error('the report is not a JSON object');
	if (doc.format !== REPORT_FORMAT) throw new Error(`the report's format is ${JSON.stringify(doc.format)}, not ${REPORT_FORMAT}`);
	const image = isObject(doc.image) ? doc.image : {};
	const config = isObject(doc.config) ? doc.config : {};
	const checks = Array.isArray(doc.checks) ? doc.checks : [];
	return {
		format: doc.format,
		verdict: typeof doc.verdict === 'string' ? doc.verdict : '',
		imageSha256: typeof image.sha256 === 'string' ? image.sha256 : '',
		configHash: typeof config.config_hash === 'string' ? config.config_hash : '',
		checks: checks.filter(isObject).map((c) => ({
			id: String(c.id ?? ''),
			title: String(c.title ?? ''),
			result: String(c.result ?? ''),
			values: strings(c.values),
			failures: strings(c.failures)
		}))
	};
}

const hexOf = (bytes: ArrayBuffer): string => Array.from(new Uint8Array(bytes), (b) => b.toString(16).padStart(2, '0')).join('');

export async function sha256Hex(bytes: Uint8Array): Promise<string> {
	return hexOf(await crypto.subtle.digest('SHA-256', bytes as BufferSource));
}

/** True when `id` is the SHA-256 (lower-case hex) of exactly these bytes: the report id of the store's binding. */
export async function reportIdMatches(bytes: Uint8Array, id: string): Promise<boolean> {
	return /^[0-9a-f]{64}$/.test(id) && (await sha256Hex(bytes)) === id;
}

/** A validation as the GUI holds it: the report bytes, what was checked and the staged config it was run for. */
export interface ValidationRecord {
	/** The image hash and config hash the validation was asked for. */
	imageHash: string;
	configHash: string;
	/** The id the helper announced; the report is trusted only when this equals the SHA-256 of its bytes (checked again by whyCannotActivate). */
	reportId: string;
	bytes: Uint8Array;
	/** The id computed here from `bytes`. */
	computedId: string;
	/** null when the bytes are not a report. */
	report: ValidationReport | null;
	parseError: string | null;
	/** stagedFingerprint() at the moment the view was built. */
	fingerprint: string;
}

export async function makeRecord(imageHash: string, configHash: string, reportId: string, bytes: Uint8Array, fingerprint: string): Promise<ValidationRecord> {
	let report: ValidationReport | null = null;
	let parseError: string | null = null;
	try {
		report = parseReport(bytes);
	} catch (e) {
		parseError = e instanceof Error ? e.message : String(e);
	}
	return { imageHash, configHash, reportId, bytes, computedId: await sha256Hex(bytes), report, parseError, fingerprint };
}

export interface ActivationFacts {
	record: ValidationRecord | null;
	/** The image hash the vehicle reports (MARV_IDENTITY). */
	vehicleImageSha256: string | null;
	/** The config hash the helper computed for the staged view as it is now; null when it has not been computed for the current staged config. */
	stagedConfigHash: string | null;
	/** stagedFingerprint() of the staged config as it is now. */
	fingerprint: string;
}

/** null when the staged config may be activated, else the reason it may not (the Activate button's tooltip). */
export function whyCannotActivate(f: ActivationFacts): string | null {
	const r = f.record;
	if (!r) return 'no validation report: validate the staged config first';
	if (!r.report) return `the validation report cannot be read: ${r.parseError ?? 'unknown error'}`;
	if (r.report.verdict !== 'PASS') return `the validation report says ${r.report.verdict || 'no verdict'}: only a PASS report allows activation`;
	if (!/^[0-9a-f]{64}$/.test(r.reportId) || r.reportId !== r.computedId) return 'the report id is not the SHA-256 of the report: the report cannot be trusted';
	if (!f.vehicleImageSha256) return 'the vehicle has not reported its image hash';
	if (r.report.imageSha256 !== f.vehicleImageSha256 || r.imageHash !== f.vehicleImageSha256) {
		return `the report is for image ${r.report.imageSha256.slice(0, 12)}, the vehicle runs image ${f.vehicleImageSha256.slice(0, 12)}`;
	}
	if (r.fingerprint !== f.fingerprint) return 'the staged config changed since it was validated: validate it again';
	if (f.stagedConfigHash === null) return 'the config hash of the staged config has not been computed';
	if (r.report.configHash !== f.stagedConfigHash || r.configHash !== f.stagedConfigHash) {
		return `the report is for config ${r.report.configHash.slice(0, 12)}, the staged config is ${f.stagedConfigHash.slice(0, 12)}`;
	}
	return null;
}
