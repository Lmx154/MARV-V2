// The G7 gate for flashing, as the Flash page applies it (core G7, decision 0053 Q2 and Q6, ground-segment unit 15): an image is
// flashed only when a passing validation report is stored for (image hash, the config the board will run with it), and only to
// a vehicle that reports itself disarmed. The helper plans the flash and checks the report store (tools/gs/flash.py); this file
// is the page's own check of what the helper said against what the vehicle says now, and the comparison of the identity the
// vehicle reports after the flash with the identity the plan expects.
import type { VehicleIdentity } from '$link/vehicle';

/** What the helper needs of the vehicle to decide which config the board will run (tools/gs/flash.py parse_board). */
export interface BoardFacts {
	schema_hash: string;
	config_hash: string;
	config_source: 'factory' | 'active';
	armed: boolean | null;
}

/** The helper's plan (tools/gs/flash.py make_plan). */
export interface FlashPlan {
	image: { hash: string; tree: string; name: string; schema_hash: string };
	config: { which: 'factory' | 'active'; hash: string; expected_source: 'factory' | 'active'; why: string };
	gate: { passes: boolean; reason: string | null; report_id: string | null; verdict: string | null };
	can_validate: boolean;
	allowed: boolean;
	refusal: string | null;
}

export interface FlashExpected {
	image_hash: string;
	config_hash: string;
	config_source: 'factory' | 'active';
}

export function boardFacts(identity: VehicleIdentity | null, armed: boolean | null): BoardFacts | null {
	if (!identity) return null;
	return { schema_hash: identity.schemaHash, config_hash: identity.configSha256, config_source: identity.configSource, armed };
}

/** What a plan was made for: the image and the vehicle's schema, config and config source (not its arming state, which is live). */
export function planKey(imageHash: string, facts: BoardFacts): string {
	return JSON.stringify([imageHash, facts.schema_hash, facts.config_hash, facts.config_source]);
}

export interface FlashGateFacts {
	/** The helper WebSocket state: 'open' when the GCS can talk to the helper. */
	helper: string;
	device: string | null;
	heartbeat: string | undefined;
	/** From the heartbeat's base mode; null until one arrived. */
	armed: boolean | null | undefined;
	identity: VehicleIdentity | null;
	/** The image picked from the library. */
	image: string | null;
	plan: FlashPlan | null;
	/** planKey() of what `plan` was made for. */
	planFor: string | null;
	planError: string | null;
	/** A plan, validation or flash is in progress. */
	busy: boolean;
}

/** null when Flash may be pressed, else why not (the button's tooltip and the text next to it). */
export function whyCannotFlash(f: FlashGateFacts): string | null {
	if (f.helper !== 'open') return 'the helper is not connected';
	if (!f.device) return 'no device is open (Connection page)';
	if (f.heartbeat !== 'alive') return 'no heartbeat from the vehicle: its arming state is unknown';
	if (f.armed === true) return 'the vehicle is armed: never flash while armed';
	if (f.armed !== false) return 'the vehicle has not reported its arming state';
	if (!f.identity) return 'the vehicle has not reported its identity';
	if (f.busy) return 'a check, a validation or a flash is in progress';
	if (!f.image) return 'pick an image';
	if (f.planError) return `the gate could not be checked: ${f.planError}`;
	const facts = boardFacts(f.identity, f.armed);
	if (!f.plan || !facts || f.plan.image.hash !== f.image) return 'the gate has not been checked for this image';
	if (f.planFor !== planKey(f.image, facts)) return 'the vehicle changed since the gate was checked: check it again';
	if (!f.plan.gate.passes) return f.plan.gate.reason ?? 'no passing validation report for this image and config';
	return null;
}

export interface IdentityCheck {
	what: string;
	expected: string;
	got: string;
	ok: boolean;
}

/** The identity the vehicle reports after the flash against the plan's expectation: image hash, config hash and config source. */
export function checkIdentity(identity: VehicleIdentity, expected: FlashExpected): IdentityCheck[] {
	return [
		{ what: 'image hash', expected: expected.image_hash, got: identity.imageSha256, ok: identity.imageSha256 === expected.image_hash },
		{ what: 'config hash', expected: expected.config_hash, got: identity.configSha256, ok: identity.configSha256 === expected.config_hash },
		{ what: 'config source', expected: expected.config_source, got: identity.configSource, ok: identity.configSource === expected.config_source }
	];
}
