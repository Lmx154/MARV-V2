/**
 * Flight profiles (ADR-0012): the profiled params as one row per param with a column per profile, the profile message,
 * and the requested-vs-reported profile the Mission view shows.
 */
import type { ClientMsg, MissionStatus, ParamSpec, ProfileDef, Schema, Telemetry } from './types';

/** One profiled param: its cells in Schema.profiles order. */
export interface ProfiledRow {
	family: number;
	kind: number;
	/** The param's id without its .profile suffix. */
	id: string;
	label: string;
	unit?: string;
	cells: ParamSpec[];
}

/** Every profiled param of every kind, grouped into rows; a row missing a profile's cell is left out. */
export function profiledRows(schema: Schema): ProfiledRow[] {
	const out: ProfiledRow[] = [];
	schema.families.forEach((f, fi) =>
		f.kinds.forEach((k, ki) => {
			const rows = new Map<string, (ParamSpec | undefined)[]>();
			for (const p of k.params) {
				if (p.profile === undefined) continue;
				const col = schema.profiles.findIndex((x) => x.id === p.profile);
				const base = p.id.endsWith(`.${p.profile}`) ? p.id.slice(0, -(p.profile.length + 1)) : p.id;
				if (col < 0) continue;
				const cells = rows.get(base) ?? Array<ParamSpec | undefined>(schema.profiles.length).fill(undefined);
				cells[col] = p;
				rows.set(base, cells);
			}
			for (const [id, cells] of rows) {
				if (!cells.every((c): c is ParamSpec => c !== undefined)) continue;
				out.push({ family: fi, kind: ki, id, label: cells[0].label, unit: cells[0].unit, cells });
			}
		})
	);
	return out;
}

/** The live profile switch: an index into Schema.profiles. */
export function profileMsg(profile: number): Extract<ClientMsg, { type: 'profile' }> {
	return { type: 'profile', profile };
}

/** The profile the FC reports applying: telemetry's, else the executor's. */
export function reportedProfile(telem: Telemetry | null, mission: MissionStatus | null): number | null {
	return telem?.profile ?? mission?.profile ?? null;
}

export interface ProfileStatus {
	requested: string;
	reported: string;
	/** Both known and different. */
	mismatch: boolean;
}

/** Labels of the requested and reported profiles, and whether they differ. */
export function profileStatus(profiles: readonly ProfileDef[], requested: number | null, reported: number | null): ProfileStatus {
	const label = (i: number | null): string => (i === null ? '—' : (profiles[i]?.label ?? `#${i}`));
	return { requested: label(requested), reported: label(reported), mismatch: requested !== null && reported !== null && requested !== reported };
}
