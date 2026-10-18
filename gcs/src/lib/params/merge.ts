// The parameter table: the manifest (what the image declares) merged with what the vehicle reports. Keyed by the link id (the
// GCS keys by name, decision 0053 Q1); a parameter on one side only is kept and marked.
import { bitsToValue } from '$link/bits';
import type { ParamType } from '$link/bits';
import type { Method, Origin, SigmaKind, VehicleParam } from '$link/vehicle';
import type { LockInfo, Manifest, ManifestParam, Provenance } from './manifest';

/** How the row relates to the vehicle. */
export type RowStatus = 'matched' | 'manifest only' | 'vehicle only' | 'mismatch';

export interface ParamRow {
	/** The link id (unique within a schema). */
	key: string;
	/** The full name: the manifest's, else the link id. */
	name: string;
	linkId: string;
	index: number | null;
	type: ParamType;
	unit: string;
	group: string;
	method: Method | null;
	origin: Origin | null;
	sigmaKind: SigmaKind | null;
	sigma: number | null;
	/** The factory source text of the manifest, or the vehicle's when it reports one. */
	source: string | null;
	/** The source text is the vehicle's (it can differ from the factory text once staged). */
	sourceFromVehicle: boolean;
	sourceLength: number | null;
	locked: boolean;
	lockInfo: LockInfo | null;
	factoryBits: number | null;
	activeBits: number | null;
	stagedBits: number | null;
	/** Staged differs from active. */
	differs: boolean;
	/** The active value is not the factory value (the manifest's default); false while either is unknown. */
	changed: boolean;
	status: RowStatus;
	/** What does not agree between the two sides. */
	notes: string[];
}

export interface Group {
	name: string;
	rows: ParamRow[];
}

export interface Merged {
	rows: ParamRow[];
	groups: Group[];
	issues: string[];
}

export const NO_GROUP = 'not in the manifest';

export function mergeParams(manifest: Manifest | null, provenance: Provenance | null, vehicle: VehicleParam[] | null): Merged {
	const issues: string[] = [];
	if (manifest && provenance && manifest.schemaHash !== provenance.schemaHash) {
		issues.push(`the manifest (${manifest.schemaHash}) and the provenance file (${provenance.schemaHash}) are of different schemas`);
	}
	const lockOf = new Map<string, LockInfo>();
	const groupOf = new Map<string, string>();
	const order = new Map<string, number>();
	provenance?.entries.forEach((e, i) => {
		for (const c of e.components) {
			groupOf.set(c, e.sourceFile);
			order.set(c, i);
			if (e.lock) lockOf.set(c, e.lock);
		}
	});
	const byLinkId = new Map<string, VehicleParam>();
	for (const v of vehicle ?? []) if (v.linkId !== null && v.type !== null) byLinkId.set(v.linkId, v);

	const rows: ParamRow[] = [];
	const seen = new Set<string>();
	for (const m of manifest?.params ?? []) {
		const v = byLinkId.get(m.linkId);
		seen.add(m.linkId);
		rows.push(rowOf(m, v, groupOf.get(m.name) ?? m.sourceFile, lockOf.get(m.name) ?? null, vehicle !== null));
	}
	for (const v of byLinkId.values()) {
		if (seen.has(v.linkId as string)) continue;
		rows.push(vehicleOnlyRow(v));
	}
	const orderOf = (r: ParamRow): number => order.get(r.name) ?? Number.MAX_SAFE_INTEGER;
	rows.sort((a, b) => orderOf(a) - orderOf(b) || (a.index ?? 0) - (b.index ?? 0) || a.key.localeCompare(b.key));
	if (manifest && vehicle && vehicle.length > 0) {
		const extra = rows.filter((r) => r.status === 'vehicle only').length;
		const missing = rows.filter((r) => r.status === 'manifest only').length;
		if (extra) issues.push(`${extra} parameter(s) on the vehicle are not in the manifest`);
		if (missing) issues.push(`${missing} parameter(s) of the manifest are not on the vehicle`);
	}
	return { rows, groups: groupRows(rows), issues };
}

function rowOf(m: ManifestParam, v: VehicleParam | undefined, group: string, lockInfo: LockInfo | null, haveVehicle: boolean): ParamRow {
	const notes: string[] = [];
	let status: RowStatus = haveVehicle ? 'manifest only' : 'matched';
	if (v && v.type !== null) {
		status = 'matched';
		if (v.type !== m.type) {
			status = 'mismatch';
			notes.push(`the vehicle reports ${v.type}, the manifest ${m.type}`);
		}
		if (v.index !== m.id) {
			status = 'mismatch';
			notes.push(`the vehicle's index is ${v.index}, the manifest's id ${m.id}`);
		}
		if (v.locked === false && m.locked) notes.push('the manifest locks this parameter, the vehicle reports it unlocked; treated as locked');
	}
	const hasVehicle = v !== undefined;
	const active = hasVehicle ? v.activeBits : null;
	const staged = hasVehicle ? v.stagedBits : null;
	return {
		key: m.linkId,
		name: m.name,
		linkId: m.linkId,
		index: m.id,
		type: m.type,
		unit: m.unit,
		group,
		method: hasVehicle && v.method !== null ? v.method : m.method,
		origin: hasVehicle && v.origin !== null ? v.origin : m.origin,
		sigmaKind: hasVehicle && v.sigmaKind !== null ? v.sigmaKind : m.sigmaKind,
		sigma: hasVehicle && v.sigma !== null ? v.sigma : m.sigma,
		source: hasVehicle && v.source !== null ? v.source : m.source,
		sourceFromVehicle: hasVehicle && v.source !== null,
		sourceLength: hasVehicle ? v.sourceLength : null,
		locked: m.locked || (hasVehicle && v.locked === true),
		lockInfo,
		factoryBits: m.defaultBits,
		activeBits: active,
		stagedBits: staged,
		differs: active !== null && staged !== null && active !== staged,
		changed: active !== null && active !== m.defaultBits,
		status,
		notes
	};
}

function vehicleOnlyRow(v: VehicleParam): ParamRow {
	return {
		key: v.linkId as string,
		name: v.linkId as string,
		linkId: v.linkId as string,
		index: v.index,
		type: v.type as ParamType,
		unit: '',
		group: NO_GROUP,
		method: v.method,
		origin: v.origin,
		sigmaKind: v.sigmaKind,
		sigma: v.sigma,
		source: v.source,
		sourceFromVehicle: true,
		sourceLength: v.sourceLength,
		locked: v.locked === true,
		lockInfo: null,
		factoryBits: null,
		activeBits: v.activeBits,
		stagedBits: v.stagedBits,
		differs: v.activeBits !== null && v.stagedBits !== null && v.activeBits !== v.stagedBits,
		changed: false,
		status: 'vehicle only',
		notes: ['not in the manifest of this image: read-only']
	};
}

function groupRows(rows: ParamRow[]): Group[] {
	const groups: Group[] = [];
	const byName = new Map<string, Group>();
	for (const r of rows) {
		let g = byName.get(r.group);
		if (!g) {
			g = { name: r.group, rows: [] };
			byName.set(r.group, g);
			groups.push(g);
		}
		g.rows.push(r);
	}
	return groups;
}

/** The numeric value of bits in the row's type, for display of a sigma comparison and sorting. */
export function valueOf(bits: number | null, type: ParamType): number | null {
	return bits === null ? null : bitsToValue(bits, type);
}

/** True when any of the row's visible text contains every word of the filter (case-insensitive). */
export function matchesFilter(row: ParamRow, filter: string): boolean {
	const words = filter.toLowerCase().split(/\s+/).filter(Boolean);
	if (words.length === 0) return true;
	const hay = [row.name, row.linkId, row.group, row.unit, row.method ?? '', row.origin ?? '', row.sigmaKind ?? '', row.source ?? ''].join('\n').toLowerCase();
	return words.every((w) => hay.includes(w));
}
