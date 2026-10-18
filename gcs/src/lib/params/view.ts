// What the parameter page does with the merged rows: sorting, counting and the "About this parameter" text. Pure functions, so
// that they are tested without a browser.
import { bitsToValue } from '$link/bits';
import type { ParamRow } from './merge';

export type SortKey = 'name' | 'index' | 'factory' | 'active' | 'staged' | 'unit' | 'method' | 'origin' | 'sigma';
export type SortDirection = 'asc' | 'desc';
export interface Sort {
	key: SortKey;
	direction: SortDirection;
}

function compareNullable<T>(a: T | null, b: T | null, compare: (x: T, y: T) => number): number {
	if (a === null && b === null) return 0;
	if (a === null) return 1; // unknown values go last in either direction (the caller flips only known pairs)
	if (b === null) return -1;
	return compare(a, b);
}

const text = (a: string, b: string): number => a.localeCompare(b, 'en', { numeric: true });
const num = (a: number, b: number): number => (a < b ? -1 : a > b ? 1 : 0);

function keyOf(row: ParamRow, key: SortKey): string | number | null {
	switch (key) {
		case 'name': return row.name;
		case 'index': return row.index;
		case 'factory': return row.factoryBits === null ? null : bitsToValue(row.factoryBits, row.type);
		case 'active': return row.activeBits === null ? null : bitsToValue(row.activeBits, row.type);
		case 'staged': return row.stagedBits === null ? null : bitsToValue(row.stagedBits, row.type);
		case 'unit': return row.unit || null;
		case 'method': return row.method;
		case 'origin': return row.origin;
		case 'sigma': return row.sigmaKind === 'Known' ? row.sigma : null;
	}
}

/** A sorted copy; unknown values (no vehicle value, no sigma) stay last whichever way the column is sorted. Ties keep the manifest's order. */
export function sortRows(rows: ParamRow[], sort: Sort | null): ParamRow[] {
	if (!sort) return rows;
	const sign = sort.direction === 'asc' ? 1 : -1;
	return rows
		.map((row, i) => ({ row, i }))
		.sort((x, y) => {
			const a = keyOf(x.row, sort.key);
			const b = keyOf(y.row, sort.key);
			const known = a !== null && b !== null;
			const c = compareNullable(a, b, (p, q) => (typeof p === 'number' && typeof q === 'number' ? num(p, q) : text(String(p), String(q))));
			return (known ? sign * c : c) || x.i - y.i;
		})
		.map((e) => e.row);
}

/** Click on a column header: ascending, then descending, then the manifest's order. */
export function nextSort(current: Sort | null, key: SortKey): Sort | null {
	if (!current || current.key !== key) return { key, direction: 'asc' };
	return current.direction === 'asc' ? { key, direction: 'desc' } : null;
}

export interface Counts {
	total: number;
	locked: number;
	derived: number;
	/** Staged differs from active. */
	staged: number;
	/** Active differs from the factory value. */
	changed: number;
}

export function countRows(rows: ParamRow[]): Counts {
	return {
		total: rows.length,
		locked: rows.filter((r) => r.locked).length,
		derived: rows.filter((r) => r.method === 'derived').length,
		staged: rows.filter((r) => r.differs).length,
		changed: rows.filter((r) => r.changed).length
	};
}

const SIGMA_MEANING: Record<string, string> = {
	Known: 'a stated 1-sigma uncertainty',
	Exact: 'exact: no uncertainty (a constant by definition or a choice of the model)',
	Unknown: 'UNKNOWN: the value has no sourced uncertainty',
	Choice: 'a design choice: no measurement uncertainty applies'
};

export interface About {
	/** The link id, how it relates to the full name, and the index. */
	linkId: string;
	linkIdNote: string;
	index: string;
	/** Derived values: the provenance's source text, which starts with the derivation rule (the generator writes "<rule>; <source>"). */
	derivation: string | null;
	sigma: string;
	/** The lock ruling, or null when the parameter is not locked. */
	lock: string | null;
}

export const LOCK_RULING =
	'A locked parameter keeps its compiled value and provenance: staging and activation refuse a difference, so it changes only by a firmware build (decision 0053 Q2).';

export function aboutParam(row: ParamRow): About {
	const sameAsName = row.linkId === row.name;
	return {
		linkId: row.linkId,
		linkIdNote: sameAsName
			? 'the full name (at most 16 characters)'
			: /^.{11}_[0-9a-f]{4}$/.test(row.linkId)
				? 'the name cut to 11 characters, "_" and the first 4 hex digits of the SHA-256 of the name (or a mav_id override)'
				: 'a mav_id override of the record',
		index: row.index === null ? 'not in the manifest' : `#${row.index} (the parameter index on the link; valid within one schema)`,
		derivation: row.method === 'derived' ? row.source : null,
		sigma: row.sigmaKind === null ? 'not reported' : SIGMA_MEANING[row.sigmaKind] ?? row.sigmaKind,
		lock: row.locked
			? `${row.lockInfo ? `Locked by ${row.lockInfo.by} on ${row.lockInfo.on}${row.lockInfo.via ? ` (via ${row.lockInfo.via})` : ''}. ` : 'Locked. '}${LOCK_RULING}`
			: null
	};
}
