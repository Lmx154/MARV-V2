// The Tuning page's state: the groups laid out from a manifest, the rows the owner asked for that the image does not have, the refusals
// (armed, no vehicle, schema mismatch, locked, missing reason), the explicit confirmation of a safety parameter, and that a write sends
// exactly the changed records, each with its source text.
import { describe, expect, it } from 'vitest';
import { f32ToBits } from '$link/bits';
import type { SessionState } from '$link/session';
import type { StageResult, VehicleParam, VehicleState } from '$link/vehicle';
import { stageContextOf, whyCannotWrite } from './gating';
import { mergeParams } from '$lib/params/merge';
import { parseManifest, parseProvenance } from '$lib/params/manifest';
import { manifestJson, provenanceJson, SCHEMA, SPECS } from '$lib/params/testing/fixture';
import type { StageContext } from '$lib/params/staging';
import {
	CONFIRM_NEEDED, DERIVED_NEEDED, groupsOf, NO_PARAMETERS_YET, NOT_A_PARAMETER, planWrite, SOURCE_NEEDED, SOURCE_PREFIX, TUNING_GROUPS, whyCannotSend,
	writeChanges, type Edits, type WriteRequest
} from './tuning_state';

type Spec = (typeof SPECS)[number];
const spec = (name: string, type: 'f32' | 'i32', value: number, extra: Partial<Spec> = {}): Spec => ({
	name, type, unit: '1', linkId: name.slice(0, 16), value, method: 'scenario', sigmaKind: 'Choice', source: `the source of ${name}`, file: 'register.yaml', ...extra
});
const LOCKED = { lock: true };

// The image has most of what the owner listed, not the second pad of the motor map and no integrator limit (the link id is the name cut to 16 here).
const SPEC_LIST: Spec[] = [
	spec('rate_kp_roll', 'f32', 0.5, { method: 'derived' }),
	spec('rate_kp_pitch', 'f32', 0.6, { method: 'derived' }),
	spec('rate_ff_clamp_k', 'f32', 0.5),
	spec('att_kp', 'f32', 8, { method: 'derived' }),
	spec('angle_tilt_max', 'f32', 0.5),
	spec('idle_speed', 'f32', 100, { method: 'derived' }),
	spec('rotor_speed_max', 'f32', 2000, LOCKED),
	spec('motor_pad_m1', 'i32', 3, LOCKED),
	spec('rotor_yaw_sign_m1', 'i32', 1),
	spec('gyro_lpf_cutoff_hz', 'f32', 200, { method: 'derived', unit: 'Hz' })
];
const manifest = parseManifest(manifestJson(SPEC_LIST));
const provenance = parseProvenance(provenanceJson(SPEC_LIST));

function vehicleParams(patch: Record<string, Partial<VehicleParam>> = {}): VehicleParam[] {
	return manifest.params.map((m, i) => ({
		index: i, linkId: m.linkId, type: m.type, activeBits: m.defaultBits, stagedBits: m.defaultBits, sigma: m.sigma, sigmaBits: f32ToBits(m.sigma), origin: m.origin, method: m.method,
		sigmaKind: m.sigmaKind, locked: m.locked, hasRecord: true, sourceLength: m.source.length, source: m.source, sourcePartial: m.source, ...(patch[m.name] ?? {})
	}));
}
const rowsOf = (patch: Record<string, Partial<VehicleParam>> = {}) => mergeParams(manifest, provenance, vehicleParams(patch)).rows;

const ctx: StageContext = { connected: true, identityKnown: true, manifestMatches: true, bytewise: true, armed: false };
const groups = () => groupsOf(rowsOf());
const accepted = (bits: number): StageResult => ({ accepted: true, sentBits: bits, echoBits: bits, reasons: [], sourceStored: true, sourceState: 'stored', sourceAttempts: 1, source: '', note: null, at: 0 });
const refused = (bits: number, reasons: string[]): StageResult => ({ ...accepted(bits), accepted: false, echoBits: null, reasons, sourceStored: null, sourceState: 'none' });

interface Sent { linkId: string; type: string; bits: number; source: string }
function request(plan: ReturnType<typeof planWrite>, over: Partial<WriteRequest> = {}): { req: WriteRequest; sent: Sent[] } {
	const sent: Sent[] = [];
	const req: WriteRequest = {
		plan, linkBlocker: null, derivedAck: true, confirmedKey: plan.confirmKey,
		stage: async (linkId, type, bits, source) => {
			sent.push({ linkId, type, bits, source });
			return accepted(bits);
		},
		...over
	};
	return { req, sent };
}

describe('the groups', () => {
	it('are the owner\'s four, in order, with the motor group alone a safety group', () => {
		const g = groups();
		expect(g.map((x) => x.id)).toEqual(['rate', 'angle', 'motor', 'filters']);
		expect(g.map((x) => x.safety)).toEqual([false, false, true, false]);
		expect(TUNING_GROUPS.filter((x) => x.safety).map((x) => x.id)).toEqual(['motor']);
	});

	it('take the rows of the manifest and the vehicle, with unit, provenance and lock', () => {
		const rate = groups().find((x) => x.id === 'rate')!;
		const kp = rate.items.find((i) => i.name === 'rate_kp_roll')!;
		expect(kp.absent).toBeNull();
		expect(kp.row).toMatchObject({ name: 'rate_kp_roll', unit: '1', method: 'derived', origin: 'DefaultFromCard', locked: false, activeBits: f32ToBits(0.5) });
		const motor = groups().find((x) => x.id === 'motor')!;
		expect(motor.items.find((i) => i.name === 'rotor_speed_max')!.row).toMatchObject({ locked: true, lockInfo: { by: 'Luis' } });
	});

	it('say "not a parameter in this image" for what the image has no parameter for', () => {
		const rate = groups().find((x) => x.id === 'rate')!;
		const integrator = rate.items.find((i) => i.label === 'Integrator limit')!;
		expect(integrator).toMatchObject({ name: null, row: null, absent: NOT_A_PARAMETER });
		expect(rate.items.find((i) => i.name === 'rate_kp_yaw')).toMatchObject({ row: null, absent: NOT_A_PARAMETER });
		expect(groups().find((x) => x.id === 'motor')!.items.find((i) => i.name === 'motor_pad_m2')).toMatchObject({ row: null, absent: NOT_A_PARAMETER });
	});

	it('say the parameters are not loaded, rather than missing, while there are no rows', () => {
		const empty = groupsOf([]);
		expect(empty.flatMap((g) => g.items).filter((i) => i.name !== null).every((i) => i.absent === NO_PARAMETERS_YET)).toBe(true);
		expect(empty.find((g) => g.id === 'rate')!.items.find((i) => i.name === null)!.absent).toBe(NOT_A_PARAMETER);
	});
});

describe('what a write would send', () => {
	it('is the typed values that change something, with old and new value and the reason as the source text', () => {
		const plan = planWrite(groups(), ctx, { rate_kp_roll: '0.75', att_kp: ' 9 ', rate_kp_pitch: '' }, 'tuned on the bench');
		expect(plan.changes.map((c) => [c.name, c.from, c.to, c.derived])).toEqual([['rate_kp_roll', '0.5', '0.75', true], ['att_kp', '8', '9', true]]);
		expect(plan.changes.every((c) => c.source === `${SOURCE_PREFIX}tuned on the bench`)).toBe(true);
		expect(plan.problems).toEqual([]);
		expect(whyCannotSend(plan, true, null)).toBeNull();
	});

	it('leaves out a value equal to the staged one', () => {
		const plan = planWrite(groups(), ctx, { rate_kp_roll: '0.5' }, 'x');
		expect(plan.changes).toEqual([]);
		expect(whyCannotSend(plan, true, null)).toBe('no value was changed');
	});

	it('makes a typed value on a locked entry a problem, not a change', () => {
		const plan = planWrite(groups(), ctx, { rotor_speed_max: '2500', motor_pad_m1: '1' }, 'x');
		expect(plan.changes).toEqual([]);
		expect(plan.problems.map((p) => p.name)).toEqual(['rotor_speed_max', 'motor_pad_m1']);
		expect(plan.problems[0]!.error).toContain('locked by Luis');
		expect(whyCannotSend(plan, true, null)).toContain('locked by Luis');
	});

	it('makes text that does not parse a problem', () => {
		const plan = planWrite(groups(), ctx, { rate_kp_roll: 'abc', rotor_yaw_sign_m1: '0.5' }, 'x');
		expect(plan.changes).toEqual([]);
		expect(plan.problems.map((p) => p.name)).toEqual(['rate_kp_roll', 'rotor_yaw_sign_m1']);
	});

	it('needs a reason, and the acknowledgement while a derived value is edited', () => {
		const plan = planWrite(groups(), ctx, { rate_kp_roll: '0.75' }, '  ');
		expect(whyCannotSend(plan, true, null)).toBe(SOURCE_NEEDED);
		const plan2 = planWrite(groups(), ctx, { rate_kp_roll: '0.75' }, 'x');
		expect(whyCannotSend(plan2, false, null)).toBe(DERIVED_NEEDED);
		const plain = planWrite(groups(), ctx, { rate_ff_clamp_k: '0.4' }, 'x');
		expect(plain.needsDerivedAck).toBe(false);
		expect(whyCannotSend(plain, false, null)).toBeNull();
	});
});

describe('the write', () => {
	it('sends exactly the changed records, in order, each with its source text, and nothing else', async () => {
		const plan = planWrite(groups(), ctx, { rate_kp_roll: '0.75', rate_kp_pitch: '0.6', rate_ff_clamp_k: '0.4', angle_tilt_max: '' }, 'tuned');
		const { req, sent } = request(plan);
		const report = await writeChanges(req);
		expect(report).toEqual({ refused: null, written: [
			{ name: 'rate_kp_roll', accepted: true, why: [], sourceFailed: false },
			{ name: 'rate_ff_clamp_k', accepted: true, why: [], sourceFailed: false }
		], unsent: [] });
		expect(sent).toEqual([
			{ linkId: 'rate_kp_roll', type: 'f32', bits: f32ToBits(0.75), source: `${SOURCE_PREFIX}tuned` },
			{ linkId: 'rate_ff_clamp_k', type: 'f32', bits: f32ToBits(0.4), source: `${SOURCE_PREFIX}tuned` }
		]);
	});

	it('stops at the first refusal of the vehicle, says why and lists what was not sent', async () => {
		const plan = planWrite(groups(), ctx, { rate_kp_roll: '0.75', rate_ff_clamp_k: '0.4', att_kp: '9' }, 'x');
		const sent: string[] = [];
		const { req } = request(plan, {
			stage: async (linkId, _type, bits) => {
				sent.push(linkId);
				return linkId === 'rate_ff_clamp_k' ? refused(bits, ['refused: out of range']) : accepted(bits);
			}
		});
		const report = await writeChanges(req);
		expect(sent).toEqual(['rate_kp_roll', 'rate_ff_clamp_k']);
		expect(report.written.map((w) => [w.name, w.accepted, w.why])).toEqual([['rate_kp_roll', true, []], ['rate_ff_clamp_k', false, ['refused: out of range']]]);
		expect(report.unsent).toEqual(['att_kp']);
	});

	it('reports a failure of the link and stops', async () => {
		const plan = planWrite(groups(), ctx, { rate_kp_roll: '0.75', att_kp: '9' }, 'x');
		const { req } = request(plan, { stage: async () => { throw new Error('the vehicle did not answer'); } });
		const report = await writeChanges(req);
		expect(report.written).toEqual([{ name: 'rate_kp_roll', accepted: false, why: ['the vehicle did not answer'], sourceFailed: false }]);
		expect(report.unsent).toEqual(['att_kp']);
	});

	it('sends nothing without a reason, an acknowledgement or a change', async () => {
		for (const [edits, reason, ack] of [[{ rate_kp_roll: '0.75' }, '', true], [{ rate_kp_roll: '0.75' }, 'x', false], [{}, 'x', true]] as Array<[Edits, string, boolean]>) {
			const { req, sent } = request(planWrite(groups(), ctx, edits, reason), { derivedAck: ack });
			expect((await writeChanges(req)).refused).not.toBeNull();
			expect(sent).toEqual([]);
		}
	});
});

describe('refusals of the link', () => {
	const sessionOf = (patch: Partial<SessionState> = {}): SessionState => ({
		heartbeat: 'alive', vehicle: { systemId: 1, componentId: 1 }, armed: false, systemStatus: 'standby', lastFrameAt: 0, lastHeartbeatAt: 0, rxBytes: 0, txBytes: 0, frames: 0, badFrames: 0, ...patch
	});
	const vehicleOf = (patch: Partial<VehicleState> = {}): VehicleState => ({
		identity: { schemaHash: SCHEMA } as VehicleState['identity'], bytewise: true, count: SPEC_LIST.length, params: vehicleParams(), listing: 'complete', missing: [], notices: [], results: {}, configAck: null, ...patch
	});
	const why = (s: SessionState | null, v: VehicleState | null, hash: string | null = SCHEMA) => whyCannotWrite(stageContextOf(s, v, hash), v?.listing, { manifest: hash, vehicle: v?.identity?.schemaHash ?? null });

	it('allow a write to a connected, disarmed vehicle of the schema on screen (the control)', () => {
		expect(why(sessionOf(), vehicleOf())).toBeNull();
	});

	it('refuse while armed, and while the arming state is not known, and send nothing', async () => {
		expect(why(sessionOf({ armed: true }), vehicleOf())).toContain('armed');
		expect(why(sessionOf({ armed: null }), vehicleOf())).toContain('arming state is not known');
		const plan = planWrite(groups(), ctx, { rate_kp_roll: '0.75' }, 'x');
		const { req, sent } = request(plan, { linkBlocker: why(sessionOf({ armed: true }), vehicleOf()) });
		expect((await writeChanges(req)).refused).toContain('armed');
		expect(sent).toEqual([]);
	});

	it('refuse with no vehicle connected', () => {
		expect(why(null, null)).toContain('no vehicle is connected');
		expect(why(sessionOf({ heartbeat: 'no heartbeat' }), vehicleOf())).toContain('no vehicle is connected');
		expect(why(sessionOf(), vehicleOf({ identity: null }))).toContain('identity');
	});

	it('refuse on a schema mismatch and name both hashes', () => {
		const message = why(sessionOf(), vehicleOf(), '0x1111111111111111');
		expect(message).toContain('schema mismatch');
		expect(message).toContain('0x1111111111111111');
		expect(message).toContain(SCHEMA);
		expect(why(sessionOf(), vehicleOf(), null)).toContain('no manifest');
	});

	it('refuse before the parameters are read', () => {
		expect(why(sessionOf(), vehicleOf({ listing: 'idle' }))).toContain('read the parameters');
		expect(why(sessionOf(), vehicleOf({ listing: 'loading' }))).toContain('being read');
	});
});

describe('the motor and mixer group', () => {
	it('needs the explicit confirmation of the changed safety parameters, each with old and new value', async () => {
		const plan = planWrite(groups(), ctx, { idle_speed: '120', rotor_yaw_sign_m1: '-1', rate_kp_roll: '0.75' }, 'x');
		expect(plan.safety.map((c) => [c.name, c.from, c.to])).toEqual([['idle_speed', '100', '120'], ['rotor_yaw_sign_m1', '1', '-1']]);
		expect(whyCannotSend(plan, true, null)).toBe(CONFIRM_NEEDED);
		const unconfirmed = request(plan, { confirmedKey: null });
		expect((await writeChanges(unconfirmed.req)).refused).toBe(CONFIRM_NEEDED);
		expect(unconfirmed.sent).toEqual([]);
		const confirmed = request(plan);
		expect((await writeChanges(confirmed.req)).refused).toBeNull();
		expect(confirmed.sent.map((s) => s.linkId)).toEqual(['rate_kp_roll', 'idle_speed', 'rotor_yaw_sign_m']);
	});

	it('does not carry a confirmation over to a different set of changes', () => {
		const first = planWrite(groups(), ctx, { idle_speed: '120' }, 'x');
		const second = planWrite(groups(), ctx, { idle_speed: '130' }, 'x');
		expect(second.confirmKey).not.toBe(first.confirmKey);
		expect(whyCannotSend(second, true, first.confirmKey)).toBe(CONFIRM_NEEDED);
		expect(whyCannotSend(first, true, first.confirmKey)).toBeNull();
	});

	it('needs no confirmation when only other groups change (the control)', () => {
		const plan = planWrite(groups(), ctx, { att_kp: '9' }, 'x');
		expect(plan.safety).toEqual([]);
		expect(whyCannotSend(plan, true, null)).toBeNull();
	});

	it('shows the locked output limits and motor order read only: a typed value on them is refused with the lock', () => {
		const plan = planWrite(groups(), ctx, { rotor_speed_max: '2500', motor_pad_m1: '0', idle_speed: '120' }, 'x');
		expect(plan.problems.map((p) => p.name)).toEqual(['rotor_speed_max', 'motor_pad_m1']);
		expect(plan.changes.map((c) => c.name)).toEqual(['idle_speed']);
	});
});
