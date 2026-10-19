// The GCS's pages (GCS-first unit GX-2): one list that the navigation and the placeholder pages read. A placeholder page names the
// unit that builds it (docs/spec/20-ground-segment.md, the GCS-first units), so that nothing in the navigation is a dead link.

export interface NavRoute {
	path: string;
	label: string;
	/** The unit that builds the page; null for a page that exists. */
	unit: string | null;
	/** What the page will hold, shown on a placeholder. */
	will: string;
}

export const ROUTES: readonly NavRoute[] = [
	{ path: '/', label: 'Connection', unit: null, will: 'The helper, the device, the link.' },
	{ path: '/vehicle', label: 'Vehicle', unit: null, will: 'The live view of the vehicle: attitude, sensors, estimators, rates, motors, radio, arming, faults and link health.' },
	{ path: '/simulation', label: 'Simulation', unit: null, will: 'Define, launch and stop a simulation run; its parameters and overrides, labelled.' },
	{ path: '/radio', label: 'Radio', unit: 'GX-7', will: 'The receiver channels, their mapping, the switch thresholds and the display checks.' },
	{ path: '/params', label: 'Parameters', unit: null, will: 'The vehicle parameters.' },
	{ path: '/flash', label: 'Flash', unit: null, will: 'Images and configs through the validation gate.' },
	{ path: '/bench', label: 'Bench', unit: 'GX-10, GX-16', will: 'The console, the hardware-in-the-loop launcher and the bench images.' },
	{ path: '/logs', label: 'Logs', unit: 'GX-15', will: 'The onboard logs: import, viewer and replay.' },
	{ path: '/flight-prep', label: 'Flight prep', unit: 'GX-17', will: 'The pre-flight checklist and the diffs of the configs.' }
];

/** The route a pathname belongs to (a trailing slash or a .html suffix does not matter); null if none. */
export function routeOf(pathname: string): NavRoute | null {
	const clean = pathname.replace(/\.html$/, '').replace(/(.)\/+$/, '$1') || '/';
	return ROUTES.find((r) => r.path === clean) ?? null;
}
