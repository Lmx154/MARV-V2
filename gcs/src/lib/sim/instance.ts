// The SIM side of the app (ground-segment unit GX-8): the sim channel client and its view, created on first use in the browser. Separate
// from lib/state/app.ts on purpose: the app holds the vehicle client, and nothing that holds one may hold the sim channel (invariant S-1,
// tools/ci/check_s1.py). The helper's address and token come from the same page meta the app reads (helper_api.ts), which is
// neutral: it says where the helper is, not what the vehicle sent.
import { readHelperInfo } from '$lib/state/helper_api';
import { SimChannelClient } from './channel';
import { SimViewController } from './sim_view';

/** The helper's path for the sim channel (tools/gs/helper.py). */
export const SIM_WS_PATH = '/ws/sim';

let instance: SimViewController | null = null;

/** The URL of the sim channel for a helper's /ws URL (same host, same token). */
export function simUrlOf(wsUrl: string): string {
	const url = new URL(wsUrl);
	url.pathname = SIM_WS_PATH;
	return url.toString();
}

/** The SIM view controller, or null without a helper. Not started: the layout starts it. */
export function simView(): SimViewController | null {
	if (instance) return instance;
	if (typeof document === 'undefined') return null;
	const meta = document.querySelector('meta[name="marv-helper"]')?.getAttribute('content') ?? null;
	const info = readHelperInfo(meta, location.search, location.origin);
	if (!info) return null;
	instance = new SimViewController({ client: new SimChannelClient({ url: simUrlOf(info.wsUrl) }) });
	return instance;
}
