/** Shapes of the GCS backend's HTTP and WebSocket API (gcs decision (b), (d)). */

export interface ParamSpec {
	id: string;
	label: string;
	unit?: string;
	default: number;
	min: number;
	max: number;
	step: number;
	digits?: number;
	/** What it is / its physical limit. */
	note: string;
	/** Where the default comes from. */
	source?: string;
	/** Absolute index into Setup::values[]. */
	index: number;
}

/** A named group of a block's parameters: an expanded block is a list of parts. */
export interface BlockPart {
	id: string;
	label: string;
	params: string[];
	note?: string;
}

export interface KindDef {
	id: string;
	label: string;
	summary: string;
	/** The vehicle kinds (by id) this kind serves; a vehicle kind lists itself. */
	vehicles: string[];
	params: ParamSpec[];
	parts?: BlockPart[];
}

export interface FamilyDef {
	id: string;
	kinds: KindDef[];
}

export interface FactorySetup {
	id: number;
	label: string;
	/** Kind index per family, in family order (the wire's kind[7]). */
	kinds: number[];
	values: number[];
}

/** GET /api/schema. */
export interface Schema {
	schema_hash: number;
	families: FamilyDef[];
	factory: FactorySetup[];
}

/** kSetupHeader, as the backend forwards it. */
export interface SetupHeader {
	schema_hash: number;
	param_count: number;
	kind: number[];
	running_crc: number;
	staged_crc: number;
	stored_crc: number;
	stored_valid: boolean;
	armed: boolean;
}

export interface LinkInfo {
	mode: string;
	connected: boolean;
}

/** A geodetic point as the FC reports it (home): 1e-7 deg, altitude above mean sea level. */
export interface GeoPoint {
	lat_e7: number;
	lon_e7: number;
	alt_m: number;
}

/** Decimal degrees and metres above home: the vehicle's position, a waypoint, a mission target. */
export interface LatLonAlt {
	lat: number;
	lon: number;
	alt_m: number;
}

export type MissionMode = 'disarmed' | 'armed' | 'climb' | 'hold' | 'mission' | 'rth' | 'land';

/** The backend's mission executor (server message mission_state). */
export interface MissionStatus {
	state: MissionMode;
	/** 0-based; -1 when no waypoint is being flown. */
	wp_index: number;
	wp_count: number;
	target: LatLonAlt | null;
	dist_m: number | null;
	climb_alt_m: number | null;
	/** Where the executor returns to (set at arm), or null. */
	home: { lat: number; lon: number } | null;
	/** Why the state is what it is, e.g. "landed", "telemetry stale"; empty when none. */
	reason: string;
}

export interface Telemetry {
	p_ned: [number, number, number];
	q: [number, number, number, number];
	preset: number;
	armed: boolean;
	/** The controller's learned hover thrust, fraction of full collective (UAV). */
	thrust_hover: number;
	/** Air brake deployment 0..1 (rocket). */
	brake: number;
	/** Null until the FC reports a valid home. */
	home: GeoPoint | null;
	/** The vehicle's position, when the backend reports it. */
	geo: LatLonAlt | null;
	/** The last actuator command, one value per motor; null when not reported. */
	motor: [number, number, number, number] | null;
}

export type ClientMsg =
	| { type: 'request_setup' }
	| { type: 'set_param'; index: number; value: number }
	| { type: 'set_kind'; family: number; kind: number }
	| { type: 'load_factory'; id: number }
	| { type: 'save' }
	| { type: 'reset' }
	| { type: 'reboot' }
	| { type: 'flash' }
	| { type: 'arm' }
	| { type: 'disarm' }
	| { type: 'climb'; alt_m: number }
	| { type: 'mission_start'; waypoints: LatLonAlt[] }
	| { type: 'rth' }
	| { type: 'land' };

/** Server messages are parsed loosely (see link.ts); this is what the page acts on. */
export type ServerMsg =
	| { type: 'link'; link: LinkInfo; header: SetupHeader | null }
	| { type: 'setup'; header: SetupHeader; values: number[] | null }
	| { type: 'param'; index: number; value: number }
	| { type: 'telemetry'; telemetry: Telemetry }
	| { type: 'mission_state'; mission: MissionStatus }
	| { type: 'flash_log'; line: string }
	| { type: 'error'; request: string; error: string; family?: number };
