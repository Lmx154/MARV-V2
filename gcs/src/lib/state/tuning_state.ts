// The Tuning page's state (Luis, 2026-10-20: a good UI so the user tunes, since it varies from airframe to airframe): the parameters of
// four groups, laid out from the manifest-and-vehicle rows, and the one write that stages the changed values.
//
// What the page never does: it has no write path of its own. Every value goes through VehicleClient.stage (the parameter write, then the source
// text), the rules of lib/params/staging.ts (locks, parse, unchanged, derived) and the gates of gating.ts; validation and activation are
// the Parameters page's flow (the staged panel). No limit or default lives here: every value on screen comes from the manifest and the
// vehicle, and a parameter this image does not have is shown as such. The names below are the register's names, not numbers.
import { formatValue } from '$link/bits';
import type { ParamType } from '$link/bits';
import type { StageResult } from '$link/vehicle';
import type { ParamRow } from '$lib/params/merge';
import { checkStage, type StageContext } from '$lib/params/staging';

export const NOT_A_PARAMETER = 'not a parameter in this image';
export const NO_PARAMETERS_YET = 'the parameters are not loaded yet';
/** The text every written record starts with; the operator's reason follows. */
export const SOURCE_PREFIX = 'GCS Tuning page: ';
export const SOURCE_NEEDED = 'Say why: a source text is stored with every value';
export const DERIVED_NEEDED = 'acknowledge that a hand edit departs from the derivation of a derived value';
export const CONFIRM_NEEDED = 'confirm the change of the safety parameters';

export interface TuningItemDef {
	label: string;
	/** The register's name of the parameter; null where the image has no parameter for what the owner asked for. */
	param: string | null;
	/** What the item is when it is not self-evident. */
	note?: string;
}

export interface TuningGroupDef {
	id: string;
	title: string;
	about: string;
	/** A change needs the explicit confirmation step (motor and output limits are safety numbers). */
	safety: boolean;
	items: readonly TuningItemDef[];
}

const AXES = ['roll', 'pitch', 'yaw'] as const;
const MOTORS = ['m1', 'm2', 'm3', 'm4'] as const;
const cap = (s: string): string => s.charAt(0).toUpperCase() + s.slice(1);
const perAxis = (label: (axis: string) => string, param: (axis: string) => string): TuningItemDef[] => AXES.map((a) => ({ label: label(cap(a)), param: param(a) }));

export const TUNING_GROUPS: readonly TuningGroupDef[] = [
	{
		id: 'rate',
		title: 'Rate PID and feed-forward',
		about: 'The rate loop: the gains per axis and the reference feed-forward (decision 0058).',
		safety: false,
		items: [
			...perAxis((a) => `${a} rate P`, (a) => `rate_kp_${a}`),
			...perAxis((a) => `${a} rate I`, (a) => `rate_ki_${a}`),
			...perAxis((a) => `${a} rate D`, (a) => `rate_kd_${a}`),
			{ label: 'Reference feed-forward on', param: 'rate_ref_ff_enable', note: '1 = on (model following); any other value = off' },
			{ label: 'Feed-forward k (clamp, fraction of the hover torque authority)', param: 'rate_ff_clamp_k' },
			{ label: 'Feed-forward filter time constant', param: 'rate_ff_filter_tau' },
			{ label: 'Gyroscopic feed-forward on', param: 'rate_ff_enable', note: '1 = on; any other value = off' },
			{ label: 'Integrator limit', param: null, note: 'the integrator is held by the torque limit (anti-windup), not by a parameter' }
		]
	},
	{
		id: 'angle',
		title: 'Angle mode',
		about: 'The angle loop and what the sticks command: the angle gain, the tilt limit, the stick rates and expo.',
		safety: false,
		items: [
			{ label: 'Angle P gain', param: 'att_kp' },
			{ label: 'Maximum tilt angle', param: 'angle_tilt_max' },
			...perAxis((a) => `${a} stick centre rate`, (a) => `rates_centre_${a}`),
			...perAxis((a) => `${a} stick expo`, (a) => `rates_expo_${a}`),
			...perAxis((a) => `${a} maximum rate`, (a) => `rate_max_${a}`)
		]
	},
	{
		id: 'motor',
		title: 'Motor and mixer',
		about: 'Idle speed, output limits, motor order and direction. These are safety numbers: a change needs your explicit confirmation.',
		safety: true,
		items: [
			{ label: 'Idle speed', param: 'idle_speed' },
			{ label: 'Output limit, minimum rotor speed', param: 'rotor_speed_min' },
			{ label: 'Output limit, maximum rotor speed', param: 'rotor_speed_max' },
			...MOTORS.map((m) => ({ label: `Motor ${m.slice(1)} output pad (motor order)`, param: `motor_pad_${m}` })),
			...MOTORS.map((m) => ({ label: `Motor ${m.slice(1)} spin direction (torque sign)`, param: `rotor_yaw_sign_${m}` }))
		]
	},
	{
		id: 'filters',
		title: 'Filters',
		about: 'The gyro low-pass and the D-term low-pass.',
		safety: false,
		items: [
			{ label: 'Gyro low-pass cutoff', param: 'gyro_lpf_cutoff_hz' },
			...perAxis((a) => `${a} D-term low-pass time constant`, (a) => `rate_d_filter_tau_${a}`)
		]
	}
];

export interface TuningItem {
	label: string;
	note: string | null;
	/** The register's name; null when the owner's item has no parameter. */
	name: string | null;
	row: ParamRow | null;
	/** Why there is no row (NOT_A_PARAMETER, NO_PARAMETERS_YET); null when there is one. */
	absent: string | null;
}

export interface TuningGroup {
	id: string;
	title: string;
	about: string;
	safety: boolean;
	items: TuningItem[];
}

/** The groups from the merged rows (the manifest and what the vehicle reports): an item whose parameter the image does not have says so. */
export function groupsOf(rows: readonly ParamRow[], defs: readonly TuningGroupDef[] = TUNING_GROUPS): TuningGroup[] {
	const byName = new Map(rows.map((r) => [r.name, r]));
	return defs.map((g) => ({
		id: g.id,
		title: g.title,
		about: g.about,
		safety: g.safety,
		items: g.items.map((i): TuningItem => {
			const row = i.param === null ? null : (byName.get(i.param) ?? null);
			const absent = row ? null : i.param === null || rows.length > 0 ? NOT_A_PARAMETER : NO_PARAMETERS_YET;
			return { label: i.label, note: i.note ?? null, name: i.param, row, absent };
		})
	}));
}

/** What the vehicle holds now (the active value), as text. */
export const activeText = (row: ParamRow): string => (row.activeBits === null ? '—' : formatValue(row.activeBits, row.type));
export const stagedText = (row: ParamRow): string => (row.stagedBits === null ? '—' : formatValue(row.stagedBits, row.type));

/** The value text typed by the operator, by parameter name. */
export type Edits = Readonly<Record<string, string>>;

export interface Change {
	name: string;
	label: string;
	linkId: string;
	type: ParamType;
	unit: string;
	safety: boolean;
	derived: boolean;
	bits: number;
	/** The value the vehicle holds now, as text. */
	from: string;
	/** The value to stage, as text. */
	to: string;
	/** The source text of the record: the prefix, then the operator's reason. */
	source: string;
}

export interface Problem {
	name: string;
	label: string;
	error: string;
}

export interface WritePlan {
	changes: Change[];
	/** Typed values that cannot be sent, each with why. */
	problems: Problem[];
	/** The changes of the safety group: shown for confirmation, each with old and new value. */
	safety: Change[];
	/** Names the safety changes with their new values: a confirmation holds only for the set it was given for. */
	confirmKey: string;
	needsDerivedAck: boolean;
	sourceMissing: boolean;
}

/**
 * What a write would send: the typed values that change something, each checked by the same rules as a stage on the Parameters page.
 * A typed value equal to the staged one is no change; a value on a locked or otherwise read-only entry is a problem, not a change.
 */
export function planWrite(groups: readonly TuningGroup[], ctx: StageContext, edits: Edits, reason: string): WritePlan {
	const changes: Change[] = [];
	const problems: Problem[] = [];
	const source = SOURCE_PREFIX + reason.trim();
	for (const g of groups) {
		for (const i of g.items) {
			const text = i.name === null ? undefined : edits[i.name];
			if (!i.row || i.name === null || text === undefined || text.trim() === '') continue;
			const check = checkStage(i.row, ctx, text, source, true);
			if (!check.ok) {
				if (!check.error.startsWith('unchanged')) problems.push({ name: i.name, label: i.label, error: check.error });
				continue;
			}
			changes.push({
				name: i.name,
				label: i.label,
				linkId: i.row.linkId,
				type: i.row.type,
				unit: i.row.unit,
				safety: g.safety,
				derived: check.request.needsDerivedAck,
				bits: check.request.bits,
				from: activeText(i.row),
				to: formatValue(check.request.bits, i.row.type),
				source
			});
		}
	}
	const safety = changes.filter((c) => c.safety);
	return {
		changes,
		problems,
		safety,
		confirmKey: safety.map((c) => `${c.name}=${c.bits}`).join(';'),
		needsDerivedAck: changes.some((c) => c.derived),
		sourceMissing: reason.trim() === ''
	};
}

/** null when the plan may be written, else why not (the link gate is separate: gating.ts whyCannotWrite). */
export function whyCannotSend(plan: WritePlan, derivedAck: boolean, confirmedKey: string | null): string | null {
	if (plan.problems.length > 0) return `${plan.problems[0]!.label}: ${plan.problems[0]!.error}`;
	if (plan.changes.length === 0) return 'no value was changed';
	if (plan.sourceMissing) return SOURCE_NEEDED;
	if (plan.needsDerivedAck && !derivedAck) return DERIVED_NEEDED;
	if (plan.safety.length > 0 && confirmedKey !== plan.confirmKey) return CONFIRM_NEEDED;
	return null;
}

export interface WrittenRecord {
	name: string;
	accepted: boolean;
	/** The vehicle's refusal texts or the failure that stopped the write. */
	why: string[];
	/** The accepted value's source text did not read back. */
	sourceFailed: boolean;
}

export interface WriteReport {
	/** Why nothing was sent; null when the write started. */
	refused: string | null;
	written: WrittenRecord[];
	/** Changes not sent because an earlier one failed. */
	unsent: string[];
}

export interface WriteRequest {
	plan: WritePlan;
	/** gating.ts whyCannotWrite for the live link: armed, no vehicle, schema mismatch. */
	linkBlocker: string | null;
	derivedAck: boolean;
	confirmedKey: string | null;
	stage(linkId: string, type: ParamType, bits: number, source: string): Promise<StageResult>;
}

const messageOf = (e: unknown): string => (e instanceof Error ? e.message : String(e));

/**
 * The one write: exactly the changed records, in page order, each staged with its source text. A failure (a vehicle refusal or a throw)
 * stops the write; what was staged stays staged (Revert staged on the staged panel discards it).
 */
export async function writeChanges(req: WriteRequest): Promise<WriteReport> {
	const refused = req.linkBlocker ?? whyCannotSend(req.plan, req.derivedAck, req.confirmedKey);
	if (refused !== null) return { refused, written: [], unsent: [] };
	const written: WrittenRecord[] = [];
	const changes = req.plan.changes;
	for (let k = 0; k < changes.length; k++) {
		const c = changes[k]!;
		try {
			const r = await req.stage(c.linkId, c.type, c.bits, c.source);
			written.push({ name: c.name, accepted: r.accepted, why: r.accepted ? [] : r.reasons.length > 0 ? r.reasons : [r.note ?? 'the vehicle did not accept the value'], sourceFailed: r.accepted && r.sourceStored === false });
			if (!r.accepted) return { refused: null, written, unsent: changes.slice(k + 1).map((x) => x.name) };
		} catch (e) {
			written.push({ name: c.name, accepted: false, why: [messageOf(e)], sourceFailed: false });
			return { refused: null, written, unsent: changes.slice(k + 1).map((x) => x.name) };
		}
	}
	return { refused: null, written, unsent: [] };
}
