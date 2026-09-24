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

export interface Telemetry {
	p_ned: [number, number, number];
	q: [number, number, number, number];
	preset: number;
	armed: boolean;
	/** The controller's learned hover thrust, fraction of full collective (UAV). */
	thrust_hover: number;
	/** Air brake deployment 0..1 (rocket). */
	brake: number;
}

export type ClientMsg =
	| { type: 'request_setup' }
	| { type: 'set_param'; index: number; value: number }
	| { type: 'set_kind'; family: number; kind: number }
	| { type: 'load_factory'; id: number }
	| { type: 'save' }
	| { type: 'reset' }
	| { type: 'reboot' }
	| { type: 'flash' };

/** Server messages are parsed loosely (see link.ts); this is what the page acts on. */
export type ServerMsg =
	| { type: 'link'; link: LinkInfo; header: SetupHeader | null }
	| { type: 'setup'; header: SetupHeader; values: number[] | null }
	| { type: 'param'; index: number; value: number }
	| { type: 'telemetry'; telemetry: Telemetry }
	| { type: 'flash_log'; line: string }
	| { type: 'error'; request: string; error: string; family?: number };
