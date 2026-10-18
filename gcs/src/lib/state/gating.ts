// When a button may be pressed, and the reason when it may not (shown as the tooltip and next to the button).
import type { HeartbeatState } from '$link/session';
import type { Listing } from '$link/vehicle';

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
