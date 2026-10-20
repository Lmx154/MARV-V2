// When a button may be pressed, and the reason when it may not (shown as the tooltip and next to the button).
import type { HeartbeatState, SessionState } from '$link/session';
import type { Listing, VehicleState } from '$link/vehicle';
import type { StageContext } from '$lib/params/staging';

export interface LinkFacts {
	/** The helper WebSocket: 'open' when the GCS can talk to the helper. */
	helper: string;
	device: string | null;
	heartbeat: HeartbeatState | undefined;
	listing: Listing | undefined;
}

/** null when "Read parameters from the vehicle" may be used, else why not. */
export function whyCannotRead(f: LinkFacts): string | null {
	if (f.helper !== 'open') return 'the helper is not connected';
	if (!f.device) return 'no device is open (Connection page)';
	if (f.heartbeat !== 'alive') return 'no heartbeat from the vehicle: there is nothing to read from';
	if (f.listing === 'loading') return 'the parameters are being read';
	return null;
}

/** The facts a stage request is gated on, read from the session, the vehicle client and the schema hash of the manifest on screen. */
export function stageContextOf(session: SessionState | null, vehicle: VehicleState | null, manifestSchemaHash: string | null): StageContext {
	return {
		connected: session?.heartbeat === 'alive',
		identityKnown: vehicle?.identity != null,
		manifestMatches: Boolean(vehicle?.identity && manifestSchemaHash !== null && manifestSchemaHash === vehicle.identity.schemaHash),
		bytewise: vehicle?.bytewise ?? null,
		armed: session?.armed ?? null
	};
}

export interface SchemaHashes {
	/** The manifest on screen; null when none is loaded. */
	manifest: string | null;
	/** The vehicle's, from its identity; null until it reported it. */
	vehicle: string | null;
}

/** null when parameter values may be written to the vehicle now, else why not (the vehicle enforces the same rules again). */
export function whyCannotWrite(ctx: StageContext, listing: Listing | undefined, hashes: SchemaHashes): string | null {
	if (!ctx.connected) return 'no vehicle is connected: there is no heartbeat';
	if (!ctx.identityKnown) return 'the vehicle has not reported its identity';
	if (!ctx.manifestMatches) {
		return hashes.manifest === null
			? 'no manifest of the vehicle\'s parameter schema is on screen'
			: `schema mismatch: the manifest on screen is ${hashes.manifest}, the vehicle reports ${hashes.vehicle}`;
	}
	if (ctx.armed !== false) return ctx.armed ? 'the vehicle is armed: parameters are written only while disarmed' : 'the arming state is not known';
	if (ctx.bytewise === null) return 'the vehicle has not reported its capabilities';
	if (!ctx.bytewise) return 'the vehicle does not advertise byte-wise parameter encoding';
	if (listing === 'loading') return 'the parameters are being read';
	if (listing !== 'complete' && listing !== 'incomplete') return 'read the parameters from the vehicle first';
	return null;
}

/** null when Revert staged and Reset to factory may be used, else why not. */
export function whyCannotCommand(ctx: StageContext): string | null {
	if (!ctx.connected) return 'no heartbeat from the vehicle';
	if (!ctx.identityKnown) return 'the vehicle has not reported its identity';
	if (ctx.armed !== false) return ctx.armed ? 'the vehicle is armed: config commands are accepted only while disarmed' : 'the arming state is not known';
	return null;
}

export interface ValidateFacts {
	hasHelper: boolean;
	ctx: StageContext;
	manifestLoaded: boolean;
	listing: Listing | undefined;
	/** Why the staged view cannot be built from the parameters read so far (whyCannotBuildView), or null. */
	unreadable: string | null;
}

/** null when the staged config may be validated, else why not. */
export function whyCannotValidate(f: ValidateFacts): string | null {
	if (!f.hasHelper) return 'no helper';
	if (!f.ctx.connected) return 'no heartbeat from the vehicle';
	if (!f.ctx.identityKnown) return 'the vehicle has not reported its identity';
	if (!f.ctx.manifestMatches || !f.manifestLoaded) return 'no manifest of the vehicle\'s parameter schema is on screen';
	if (f.listing !== 'complete') return 'read the parameters from the vehicle first';
	return f.unreadable;
}
