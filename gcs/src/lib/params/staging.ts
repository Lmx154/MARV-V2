// The rules of editing a parameter in the GUI (decision 0053 Q1, Q2): when a row may be edited at all, and what a stage
// request needs before any byte is sent. The vehicle enforces the same rules again; these keep the operator from sending
// what would be refused and say why a row is read-only.
import { formatValue, parseValueText } from '$link/bits';
import type { ParamRow } from './merge';

export interface StageContext {
	/** The vehicle's heartbeat is alive. */
	connected: boolean;
	/** The vehicle's identity message arrived. */
	identityKnown: boolean;
	/** The manifest on screen is of the schema the vehicle reports. */
	manifestMatches: boolean;
	/** The vehicle advertises byte-wise parameter encoding; null = not known yet. */
	bytewise: boolean | null;
	/** null = not known. */
	armed: boolean | null;
}

export const LOCK_TOOLTIP_DEFAULT = 'locked by Luis 2026-10-17 — change only by a firmware build';
export const DERIVED_WARNING =
	'This value is derived from other inputs (see its source). A hand edit departs from the derivation: it is staged as a manual value, and the derivation no longer explains it.';
export const SOURCE_REQUIRED_MESSAGE = 'A source text is required: say where this value comes from.';

export function lockTooltip(row: ParamRow): string {
	return row.lockInfo ? `locked by ${row.lockInfo.by} ${row.lockInfo.on} — change only by a firmware build` : LOCK_TOOLTIP_DEFAULT;
}

/** null when the row may be edited, else the reason it may not. */
export function whyReadOnly(row: ParamRow, ctx: StageContext): string | null {
	if (row.locked) return lockTooltip(row);
	if (row.status !== 'matched') return row.status === 'vehicle only' ? 'not in the manifest of this image' : `the vehicle and the manifest disagree (${row.status})`;
	if (!ctx.connected) return 'no heartbeat from the vehicle';
	if (!ctx.identityKnown) return 'the vehicle has not reported its identity';
	if (!ctx.manifestMatches) return 'the manifest on screen is not of the vehicle\'s parameter schema';
	if (ctx.bytewise === null) return 'the vehicle has not reported its capabilities';
	if (!ctx.bytewise) return 'the vehicle does not advertise byte-wise parameter encoding';
	if (ctx.armed !== false) return ctx.armed ? 'the vehicle is armed: parameters are written only while disarmed' : 'the arming state is not known';
	if (row.stagedBits === null) return 'the vehicle has not reported this parameter';
	return null;
}

export interface StageRequest {
	bits: number;
	source: string;
	/** The row's method is derived: the operator must acknowledge the departure. */
	needsDerivedAck: boolean;
}

export type StageCheck = { ok: true; request: StageRequest } | { ok: false; error: string };

/** Everything that must hold before a PARAM_SET is sent: the row is editable, the text parses, it changes the value, a source is given. */
export function checkStage(row: ParamRow, ctx: StageContext, valueText: string, sourceText: string, derivedAck: boolean): StageCheck {
	const readOnly = whyReadOnly(row, ctx);
	if (readOnly) return { ok: false, error: readOnly };
	const parsed = parseValueText(valueText, row.type);
	if (!parsed.ok) return { ok: false, error: parsed.error };
	if (parsed.bits === row.stagedBits) return { ok: false, error: `unchanged: already staged as ${formatValue(parsed.bits, row.type)}` };
	if (sourceText.trim().length === 0) return { ok: false, error: SOURCE_REQUIRED_MESSAGE };
	const needsDerivedAck = row.method === 'derived';
	if (needsDerivedAck && !derivedAck) return { ok: false, error: 'acknowledge the warning about editing a derived value' };
	return { ok: true, request: { bits: parsed.bits, source: sourceText, needsDerivedAck } };
}
