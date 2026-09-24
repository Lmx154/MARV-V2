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
	/** How the backend reaches the FC now: 'usb', 'sim-bridge', or null (none open). */
	via: string | null;
	/** The device path or UDP address it is open on. */
	target: string | null;
	/** via 'busy': the process holding the FC's USB port (pid 0: unknown). */
	holder?: { name: string; pid: number } | null;
}

/** One process holding a rig resource (GET /api/resources, the resources message). */
export interface ResourceHolder {
	pid: number;
	name: string;
	cmdline: string;
	/** Start time, unix seconds. */
	started: number | null;
	/** The backend itself: never terminated. */
	self: boolean;
	/** A process of the sim the Development tab launched: terminating it stops that sim. */
	child_of_launcher: boolean;
}

export interface RigResource {
	id: string;
	label: string;
	path: string | null;
	holders: ResourceHolder[];
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

/** An airframe's physical specs as the sim reports them (GET /api/sim/airframes). */
export interface AirframeSpecs {
	mass_kg: number;
	ixx: number;
	iyy: number;
	izz: number;
	arm_m: number;
	rotor_count: number;
	/** Thrust per rotor = motor_constant * omega^2 (N s^2/rad^2). */
	motor_constant: number;
	/** Yaw torque per rotor = moment_constant * thrust (m). */
	moment_constant: number;
	/** rad/s */
	max_rot_velocity: number;
	time_constant_up: number;
	time_constant_down: number;
	/** Full thrust of one rotor, motor_constant * max_rot_velocity^2 (N). */
	t_max_n: number;
	thrust_to_weight: number;
	hover_thrust_frac: number;
	/** Rotor positions (x forward, y, m) when the backend reports them. */
	rotors?: [number, number][];
}

export interface Airframe {
	id: string;
	label: string;
	frame: string;
	/** Where the specs come from, e.g. the model file. */
	source: string;
	specs: AirframeSpecs;
}

/** Where the firmware runs: compiled for this computer (host SITL), or on the connected flight controller fed simulated
 * sensor data over USB (FC in the loop). */
export type SimTarget = 'host' | 'fc';

/** The world the sim is launched into. Wind direction is where it blows FROM, degrees clockwise from north. */
export interface SimEnv {
	wind_speed_ms: number;
	wind_dir_deg: number;
	/** Standard deviation of Gazebo's white-noise gusts. */
	gust_sigma_ms: number;
	lat: number;
	lon: number;
	elevation_m: number;
	temperature_c: number | null;
	pressure_pa: number | null;
}

/** The backend's launcher (server message sim_status). env is shown as the backend reports it. */
export interface SimStatus {
	running: boolean;
	airframe: string | null;
	env: Record<string, unknown> | null;
	target: string | null;
	gui: boolean;
	started_at: number | string | null;
	pid: number | null;
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
	| { type: 'mission_start'; waypoints: LatLonAlt[]; speed_mps?: number }
	| { type: 'rth' }
	| { type: 'land' }
	| { type: 'sim_launch'; airframe: string; env: SimEnv; gui: boolean; target: SimTarget }
	| { type: 'sim_stop' }
	| { type: 'resources_request' }
	| { type: 'resources_subscribe'; on: boolean }
	| { type: 'terminate'; pid: number };

/** Server messages are parsed loosely (see link.ts); this is what the page acts on. */
export type ServerMsg =
	| { type: 'link'; link: LinkInfo; header: SetupHeader | null }
	| { type: 'setup'; header: SetupHeader; values: number[] | null }
	| { type: 'param'; index: number; value: number }
	| { type: 'telemetry'; telemetry: Telemetry }
	| { type: 'mission_state'; mission: MissionStatus }
	| { type: 'flash_log'; line: string }
	| { type: 'sim_status'; sim: SimStatus }
	| { type: 'sim_log'; line: string }
	| { type: 'resources'; resources: RigResource[] }
	| { type: 'error'; request: string; error: string; family?: number };
