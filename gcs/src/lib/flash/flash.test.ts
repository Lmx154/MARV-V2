// The flash gate's decisions (ground-segment unit 15): Flash is off unless the helper, a device, a live disarmed vehicle with an identity,
// a picked image and a plan whose gate passes for exactly this vehicle all hold; and the read-back comparison.
import { describe, expect, it } from 'vitest';
import type { VehicleIdentity } from '$link/vehicle';
import { boardFacts, checkIdentity, planKey, whyCannotFlash, type FlashGateFacts, type FlashPlan } from './flash';

const identity: VehicleIdentity = {
	schemaHash: '0x3ae00eb50938acba',
	imageSha256: 'a'.repeat(64),
	configSha256: 'c'.repeat(64),
	validationReportId: null,
	configSource: 'factory',
	configState: 'ok',
	configRefusal: 'none',
	lastActivation: 'none',
	stagedDiffers: false
};
const IMAGE = 'b'.repeat(64);

const plan = (over: Partial<FlashPlan['gate']> = {}): FlashPlan => ({
	image: { hash: IMAGE, tree: 'm33', name: 'marv_fc3v2', schema_hash: identity.schemaHash },
	config: { which: 'factory', hash: 'd'.repeat(64), expected_source: 'factory', why: 'the board runs its factory table' },
	gate: { passes: true, reason: null, report_id: 'e'.repeat(64), verdict: 'PASS', ...over },
	can_validate: true,
	allowed: true,
	refusal: null
});

const facts = (over: Partial<FlashGateFacts> = {}): FlashGateFacts => ({
	helper: 'open',
	device: 'serial /dev/ttyACM1 115200',
	heartbeat: 'alive',
	armed: false,
	identity,
	image: IMAGE,
	plan: plan(),
	planFor: planKey(IMAGE, boardFacts(identity, false) as NonNullable<ReturnType<typeof boardFacts>>),
	planError: null,
	busy: false,
	...over
});

describe('whyCannotFlash', () => {
	it('allows the flash when every condition holds', () => {
		expect(whyCannotFlash(facts())).toBeNull();
	});

	it('is off without a passing report, naming the report problem', () => {
		const why = whyCannotFlash(facts({ plan: plan({ passes: false, reason: 'no validation report for image bbbbbbbbbbbb with config dddddddddddd', verdict: null, report_id: null }) }));
		expect(why).toContain('no validation report');
		expect(whyCannotFlash(facts({ plan: plan({ passes: false, reason: null, verdict: 'FAIL' }) }))).toContain('no passing validation report');
	});

	it('is off while armed, with an unknown arming state, and without a live heartbeat', () => {
		expect(whyCannotFlash(facts({ armed: true }))).toContain('armed');
		expect(whyCannotFlash(facts({ armed: null }))).toContain('arming state');
		expect(whyCannotFlash(facts({ armed: undefined }))).toContain('arming state');
		expect(whyCannotFlash(facts({ heartbeat: 'no heartbeat' }))).toContain('no heartbeat');
	});

	it('is off without a helper, a device, an identity or an image, and while busy', () => {
		expect(whyCannotFlash(facts({ helper: 'closed' }))).toContain('helper');
		expect(whyCannotFlash(facts({ device: null }))).toContain('no device');
		expect(whyCannotFlash(facts({ identity: null }))).toContain('identity');
		expect(whyCannotFlash(facts({ image: null }))).toContain('pick an image');
		expect(whyCannotFlash(facts({ busy: true }))).toContain('in progress');
		expect(whyCannotFlash(facts({ planError: 'boom' }))).toContain('boom');
	});

	it('is off when the plan is for another image or for a vehicle that has changed since', () => {
		expect(whyCannotFlash(facts({ plan: null }))).toContain('not been checked');
		expect(whyCannotFlash(facts({ image: 'f'.repeat(64) }))).toContain('not been checked');
		const changed = { ...identity, configSha256: '1'.repeat(64) };
		expect(whyCannotFlash(facts({ identity: changed }))).toContain('changed since');
		expect(whyCannotFlash(facts({ identity: { ...identity, configSource: 'active' } }))).toContain('changed since');
	});
});

describe('checkIdentity', () => {
	const expected = { image_hash: IMAGE, config_hash: 'c'.repeat(64), config_source: 'factory' as const };

	it('is equal only when the image hash, the config hash and the config source are the expected ones', () => {
		expect(checkIdentity({ ...identity, imageSha256: IMAGE }, expected).every((c) => c.ok)).toBe(true);
		const wrongImage = checkIdentity(identity, expected);
		expect(wrongImage.filter((c) => !c.ok).map((c) => c.what)).toEqual(['image hash']);
		const wrongConfig = checkIdentity({ ...identity, imageSha256: IMAGE, configSha256: '0'.repeat(64), configSource: 'active' }, expected);
		expect(wrongConfig.filter((c) => !c.ok).map((c) => c.what)).toEqual(['config hash', 'config source']);
	});
});
