// The vehicle client: the identity, the parameter list with provenance, the staging of a value with its source text, source
// text reads, config commands and the refusal notices, over the typed bus of a LinkSession. This is the only place the GUI's
// parameter protocol is spelled out (decision 0053 Q1); pages work with the domain types exported here and never see a wire
// message.
//
// What the vehicle must do for this client (the SIL's link service implements it; the fake vehicle in testing/ is the executable statement):
//   - answer a PARAM_REQUEST_LIST (and a PARAM_REQUEST_READ by index) with one PARAM_VALUE and one MARV_PARAM_RECORD per
//     parameter; PARAM_VALUE carries the STAGED value, the record both values;
//   - answer a PARAM_SET with a PARAM_VALUE (the staged value after the set, the unchanged one when refused) and, when
//     refused, a STATUSTEXT with the reason; the client does not take that PARAM_VALUE as the verdict (nothing correlates it with
//     the set): it reads the parameter back by index after the refusal grace and judges by the first reply after that request;
//   - answer MARV_PARAM_SOURCE READ with MARV_PARAM_SOURCE DATA chunks;
//   - answer a REQUEST_MESSAGE command for MARV_IDENTITY and AUTOPILOT_VERSION with that message.
import { get, writable, type Readable } from 'svelte/store';
import { f32ToBits, paramValueBits, paramValueFields, type ParamType } from './bits';
import { MarvConfigAction, MarvParamSourceOp, MESSAGES, type Message } from './generated/messages';
import { type LinkSession } from './session';
import { TelemetryClient, type TelemetryOptions } from './telemetry';

// ---- domain types -------------------------------------------------------------------------------------------------------

export type ConfigSource = 'factory' | 'active';
export type ConfigState = 'ok' | 'refused';
export type ConfigRefusal = 'none' | 'format' | 'schema' | 'image binding' | 'report binding' | 'validation' | 'unknown';
export type ActivationResult = 'none' | 'activated' | 'refused at boot' | 'write failed' | 'unknown';
export type Origin = 'DefaultFromCard' | 'DefaultFromRegister' | 'Manual' | 'Identified';
export type Method = 'published' | 'measured' | 'identified' | 'datasheet' | 'derived' | 'design-budget' | 'scenario';
export type SigmaKind = 'Known' | 'Exact' | 'Unknown' | 'Choice';
export type ConfigAction = 'activate' | 'revert' | 'reset-factory';
/** The answer to a maintenance grant request: 'granted' once the flight tick holds it (arming is refused from then until the vehicle restarts). */
export type MaintenanceGrant = 'granted' | 'denied: armed' | 'unsupported' | 'failed' | 'no answer';
export type ConfigCommandResult = 'accepted' | 'denied: armed' | 'denied: report' | 'denied: hash' | 'denied: unavailable' | 'failed' | 'unknown';

export interface VehicleIdentity {
	/** kParamSchemaHash as '0x' and 16 hex digits (the manifest's form). */
	schemaHash: string;
	imageSha256: string;
	configSha256: string;
	/** null when the vehicle reports none (all zero). */
	validationReportId: string | null;
	configSource: ConfigSource;
	configState: ConfigState;
	configRefusal: ConfigRefusal;
	lastActivation: ActivationResult;
	stagedDiffers: boolean;
}

export interface VehicleParam {
	index: number;
	/** The id the parameter has on the link; null until its value message arrived. */
	linkId: string | null;
	type: ParamType | null;
	activeBits: number | null;
	stagedBits: number | null;
	sigma: number | null;
	/** The raw 32 bits of sigma (the config file stores the bits, not the printed number). */
	sigmaBits: number | null;
	origin: Origin | null;
	method: Method | null;
	sigmaKind: SigmaKind | null;
	locked: boolean | null;
	/** True once the provenance record arrived. */
	hasRecord: boolean;
	sourceLength: number | null;
	/** The whole source text once it is complete, else null. */
	source: string | null;
	/** What has been received of the source text so far. */
	sourcePartial: string;
}

export interface Notice {
	at: number;
	severity: number;
	text: string;
}

/** 'none': no source was written (the value was refused); 'writing': a write is in flight; 'stored' / 'failed': the read-back result. */
export type SourceState = 'none' | 'writing' | 'stored' | 'failed';

export interface StageResult {
	/** The vehicle echoed the value that was sent. */
	accepted: boolean;
	sentBits: number;
	echoBits: number | null;
	/** Refusal texts the vehicle sent with the answer. */
	reasons: string[];
	/** The source text read back from the vehicle equals what was sent (null: not checked, as the value was refused). */
	sourceStored: boolean | null;
	sourceState: SourceState;
	/** How many times the source text was written (an accepted value whose source did not read back is written again). */
	sourceAttempts: number;
	/** The source text that was sent, kept so that a failed write can be retried. */
	source: string;
	note: string | null;
	at: number;
}

export type Listing = 'idle' | 'loading' | 'complete' | 'incomplete';

export interface VehicleState {
	identity: VehicleIdentity | null;
	/** The vehicle advertises byte-wise parameter encoding; null until its version message arrived. */
	bytewise: boolean | null;
	count: number | null;
	params: VehicleParam[];
	listing: Listing;
	missing: number[];
	notices: Notice[];
	results: Record<string, StageResult>;
	configAck: { action: ConfigAction; result: ConfigCommandResult; at: number } | null;
}

// ---- wire constants (common.xml unless noted) --------------------------------------------------------------------------

const PARAM_TYPE_INT32 = 6;
const PARAM_TYPE_REAL32 = 9;
/** MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE. */
const CAPABILITY_PARAM_ENCODE_BYTEWISE = 16n;
/** MAV_CMD_REQUEST_MESSAGE. */
const CMD_REQUEST_MESSAGE = 512;
/** MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, its param1 value 3 (REBOOT_SHUTDOWN_ACTION: reboot to the bootloader; this autopilot only grants the maintenance and keeps running) and the MAV_RESULT values. */
const CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246;
const REBOOT_ACTION_BOOTLOADER = 3;
const RESULT_ACCEPTED = 0;
const RESULT_DENIED = 2;
const RESULT_UNSUPPORTED = 3;
const RESULT_IN_PROGRESS = 5;
/** A PARAM_VALUE index meaning "none" (the standard's 65535). */
const INDEX_NONE = 0xffff;

const idOf = (kind: Message['kind']): number => {
	const info = MESSAGES.find((m) => m.kind === kind);
	if (!info) throw new Error(`no ${kind} in the dialect`);
	return info.id;
};
const chunkSize = (kind: Message['kind']): number => {
	const f = MESSAGES.find((m) => m.kind === kind)?.fields.find((x) => x.name === 'chunk' || x.name === 'source_chunk');
	if (!f) throw new Error(`no chunk field in ${kind}`);
	return f.length;
};
const SOURCE_CHUNK = chunkSize('MarvParamSource');
const PARAM_ID_BYTES = 16;
const STATUS_TEXT_BYTES = 50;

// ---- harness timing (PLACEHOLDER: protocol timeouts of this client, not design numbers) ---------------------------------

export interface Timing {
	/** Wait for one answer (an echo, a source chunk, a record, an acknowledgement). */
	replyMs: number;
	/** Wait for the final answer of the maintenance grant (the vehicle asks its flight tick, which answers within a few ticks). */
	grantMs: number;
	/** After this long without a value or record during a listing, the missing indices are asked for one by one. */
	listingIdleMs: number;
	/** How many such rounds before the listing is reported incomplete. */
	listingRounds: number;
	/** How long a refused write waits for its refusal text (and an echo that is not the sent value waits for a better one). */
	refusalGraceMs: number;
	/** How many times the source text is written before the write is reported failed (the first write included). */
	sourceAttempts: number;
}
export const DEFAULT_TIMING: Timing = { replyMs: 1000, grantMs: 5000, listingIdleMs: 1500, listingRounds: 3, refusalGraceMs: 150, sourceAttempts: 2 };
/** PLACEHOLDER: how many notices are kept for display. */
export const NOTICE_LIMIT = 100;
export const NO_SOURCE_ERROR = 'a source text is required';

// ---- helpers -------------------------------------------------------------------------------------------------------------

const hex = (bytes: Uint8Array): string => Array.from(bytes, (b) => b.toString(16).padStart(2, '0')).join('');
const isZero = (bytes: Uint8Array): boolean => bytes.every((b) => b === 0);
const decoder = new TextDecoder('utf-8');
const encoder = new TextEncoder();

function textOf(bytes: Uint8Array): string {
	const end = bytes.indexOf(0);
	return decoder.decode(end < 0 ? bytes : bytes.subarray(0, end));
}

export function encodeLinkId(linkId: string): Uint8Array {
	const bytes = encoder.encode(linkId);
	if (bytes.length > PARAM_ID_BYTES) throw new RangeError(`link id ${linkId} is longer than ${PARAM_ID_BYTES} bytes`);
	const out = new Uint8Array(PARAM_ID_BYTES);
	out.set(bytes);
	return out;
}

function pick<T extends string>(table: readonly T[], index: number, fallback: T): T {
	return table[index] ?? fallback;
}
const SOURCES: ConfigSource[] = ['factory', 'active'];
const STATES: ConfigState[] = ['ok', 'refused'];
const REFUSALS: ConfigRefusal[] = ['none', 'format', 'schema', 'image binding', 'report binding', 'validation'];
const ACTIVATIONS: ActivationResult[] = ['none', 'activated', 'refused at boot', 'write failed'];
const ORIGINS: Origin[] = ['DefaultFromCard', 'DefaultFromRegister', 'Manual', 'Identified'];
const METHODS: Method[] = ['published', 'measured', 'identified', 'datasheet', 'derived', 'design-budget', 'scenario'];
const SIGMA_KINDS: SigmaKind[] = ['Known', 'Exact', 'Unknown', 'Choice'];
const COMMAND_RESULTS: ConfigCommandResult[] = ['accepted', 'denied: armed', 'denied: report', 'denied: hash', 'denied: unavailable', 'failed'];
const ACTION_CODES: Record<ConfigAction, MarvConfigAction> = {
	activate: MarvConfigAction.Activate,
	revert: MarvConfigAction.RevertLastGood,
	'reset-factory': MarvConfigAction.ResetFactory
};

function emptyParam(index: number): VehicleParam {
	return {
		index, linkId: null, type: null, activeBits: null, stagedBits: null, sigma: null, sigmaBits: null, origin: null, method: null, sigmaKind: null,
		locked: null, hasRecord: false, sourceLength: null, source: null, sourcePartial: ''
	};
}

function initialState(): VehicleState {
	return { identity: null, bytewise: null, count: null, params: [], listing: 'idle', missing: [], notices: [], results: {}, configAck: null };
}

interface Waiter {
	cancel(): void;
}

// ---- the client ----------------------------------------------------------------------------------------------------------

export class VehicleClient {
	private readonly store = writable<VehicleState>(initialState());
	readonly state: Readable<VehicleState> = { subscribe: this.store.subscribe };
	private readonly rows = new Map<number, VehicleParam>();
	private readonly sourceBytes = new Map<number, Uint8Array>();
	private readonly textParts = new Map<number, string[]>();
	private identity: VehicleIdentity | null = null;
	private identityRaw: bigint | null = null;
	private bytewise: boolean | null = null;
	private count: number | null = null;
	private listing: Listing = 'idle';
	private missing: number[] = [];
	private notices: Notice[] = [];
	private results: Record<string, StageResult> = {};
	private configAck: VehicleState['configAck'] = null;
	private chain: Promise<unknown> = Promise.resolve();
	private lastActivity = 0;
	private readonly unsubscribers: Array<() => void> = [];
	private readonly timing: Timing;
	private readonly now: () => number;
	private requestedAtHeartbeat = false;
	private readonly noticeListeners = new Set<(text: string) => void>();
	/** The telemetry streams (the live view, GCS-first unit GX-5): a stream is asked for by the view that shows it. */
	readonly telemetry: TelemetryClient;

	constructor(private readonly session: LinkSession, options: { timing?: Partial<Timing>; now?: () => number; telemetry?: TelemetryOptions } = {}) {
		this.timing = { ...DEFAULT_TIMING, ...options.timing };
		this.now = options.now ?? (() => Date.now());
		this.telemetry = new TelemetryClient(session, { now: this.now, ...options.telemetry });
		const bus = session.bus;
		this.unsubscribers.push(
			bus.on('Heartbeat', () => {
				if (this.requestedAtHeartbeat) return;
				this.requestedAtHeartbeat = true;
				this.requestIdentity();
			}),
			bus.on('MarvIdentity', (m) => {
				this.identityRaw = m.param_schema_hash;
				this.identity = {
					schemaHash: '0x' + m.param_schema_hash.toString(16).padStart(16, '0'),
					imageSha256: hex(m.image_sha256),
					configSha256: hex(m.config_sha256),
					validationReportId: isZero(m.validation_report_id) ? null : hex(m.validation_report_id),
					configSource: pick(SOURCES, m.config_source, 'factory'),
					configState: pick(STATES, m.config_state, 'refused'),
					configRefusal: pick(REFUSALS, m.config_refusal, 'unknown'),
					lastActivation: pick(ACTIVATIONS, m.last_activation_result, 'unknown'),
					stagedDiffers: m.staged_differs !== 0
				};
				this.publish();
			}),
			bus.on('AutopilotVersion', (m) => {
				this.bytewise = (m.capabilities & CAPABILITY_PARAM_ENCODE_BYTEWISE) !== 0n;
				this.publish();
			}),
			bus.on('ParamValue', (m) => this.onValue(m)),
			bus.on('MarvParamRecord', (m) => this.onRecord(m)),
			bus.on('MarvParamSource', (m) => this.onSourceData(m)),
			bus.on('Statustext', (m) => this.onStatusText(m)),
			bus.on('MarvConfigAck', (m) => {
				const action = (Object.keys(ACTION_CODES) as ConfigAction[]).find((k) => ACTION_CODES[k] === m.action);
				if (!action) return;
				this.configAck = { action, result: pick(COMMAND_RESULTS, m.result, 'unknown'), at: this.now() };
				this.publish();
			})
		);
	}

	dispose(): void {
		for (const u of this.unsubscribers) u();
		this.unsubscribers.length = 0;
		this.telemetry.dispose();
	}

	/** Forget everything learnt from the vehicle (the connection changed). */
	reset(): void {
		this.rows.clear();
		this.sourceBytes.clear();
		this.textParts.clear();
		this.identity = null;
		this.identityRaw = null;
		this.bytewise = null;
		this.count = null;
		this.listing = 'idle';
		this.missing = [];
		this.results = {};
		this.configAck = null;
		this.requestedAtHeartbeat = false;
		this.telemetry.reset();
		this.publish();
	}

	// ---- requests -----------------------------------------------------------------------------------------------------

	/** Asks for the identity and the capabilities (a one-shot request of each message). */
	requestIdentity(): void {
		const target = this.session.target();
		if (!target) return;
		for (const kind of ['MarvIdentity', 'AutopilotVersion'] as const) {
			this.session.send({
				kind: 'CommandLong', target_system: target.systemId, target_component: target.componentId, command: CMD_REQUEST_MESSAGE,
				confirmation: 0, param1: idOf(kind), param2: 0, param3: 0, param4: 0, param5: 0, param6: 0, param7: 0
			});
		}
	}

	/**
	 * Reads the whole parameter list (values and provenance records). Resolves with the listing state when complete; resolves
	 * 'incomplete' (with `missing` set) when indices stayed missing after the retry rounds.
	 */
	async requestAll(): Promise<Listing> {
		const target = this.requireTarget();
		this.listing = 'loading';
		this.missing = [];
		this.lastActivity = this.now();
		this.publish();
		this.session.send({ kind: 'ParamRequestList', target_system: target.systemId, target_component: target.componentId });
		for (let round = 0; round <= this.timing.listingRounds; round++) {
			const done = await this.waitIdleOrComplete();
			if (done) {
				this.listing = 'complete';
				this.missing = [];
				this.publish();
				return this.listing;
			}
			if (round === this.timing.listingRounds) break;
			for (const index of this.missingIndices()) {
				this.session.send({
					kind: 'ParamRequestRead', target_system: target.systemId, target_component: target.componentId,
					param_id: new Uint8Array(PARAM_ID_BYTES), param_index: index
				});
			}
		}
		this.listing = 'incomplete';
		this.missing = this.missingIndices();
		this.publish();
		return this.listing;
	}

	/** The whole source text of a parameter, fetching the chunks the record did not carry. */
	readSource(index: number): Promise<string> {
		return this.enqueue(() => this.fetchSource(index));
	}

	/**
	 * Stages a value with its source text: PARAM_SET, then (when the vehicle echoed the value) the source text in chunks, then a
	 * read-back of the source. A missing source text is refused here before any byte is sent.
	 */
	stage(linkId: string, type: ParamType, bits: number, source: string): Promise<StageResult> {
		return this.enqueue(() => this.doStage(linkId, type, bits, source));
	}

	/** Sends a config command and resolves with the vehicle's answer. */
	async configCommand(action: ConfigAction, args: { reportId?: Uint8Array; expectedConfigSha256?: Uint8Array } = {}): Promise<ConfigCommandResult> {
		const target = this.requireTarget();
		if (this.identityRaw === null) throw new Error('the vehicle identity is not known yet');
		const code = ACTION_CODES[action];
		const wait = this.waitFor('MarvConfigAck', (m) => m.action === code, this.timing.replyMs);
		this.session.send({
			kind: 'MarvConfigCommand', param_schema_hash: this.identityRaw, report_id: args.reportId ?? new Uint8Array(32),
			expected_config_sha256: args.expectedConfigSha256 ?? new Uint8Array(32), target_system: target.systemId,
			target_component: target.componentId, action: code
		});
		const ack = await wait.promise;
		if (!ack) throw new Error('the vehicle did not answer the config command');
		return pick(COMMAND_RESULTS, ack.result, 'unknown');
	}

	/**
	 * Asks for the maintenance grant a flash needs (the vehicle's USB reset interface honours picotool only after it): the vehicle
	 * answers in progress, then accepted, or denied while armed. A grant is permanent until the vehicle restarts.
	 */
	async requestMaintenanceGrant(): Promise<MaintenanceGrant> {
		const target = this.requireTarget();
		const wait = this.waitFor('CommandAck', (m) => m.command === CMD_PREFLIGHT_REBOOT_SHUTDOWN && m.result !== RESULT_IN_PROGRESS, this.timing.grantMs);
		this.session.send({
			kind: 'CommandLong', target_system: target.systemId, target_component: target.componentId, command: CMD_PREFLIGHT_REBOOT_SHUTDOWN,
			confirmation: 0, param1: REBOOT_ACTION_BOOTLOADER, param2: 0, param3: 0, param4: 0, param5: 0, param6: 0, param7: 0
		});
		const ack = await wait.promise;
		if (!ack) return 'no answer';
		if (ack.result === RESULT_ACCEPTED) return 'granted';
		if (ack.result === RESULT_DENIED) return 'denied: armed';
		return ack.result === RESULT_UNSUPPORTED ? 'unsupported' : 'failed';
	}

	// ---- internals: stage ----------------------------------------------------------------------------------------------

	private async doStage(linkId: string, type: ParamType, bits: number, source: string): Promise<StageResult> {
		if (source.trim().length === 0) throw new Error(NO_SOURCE_ERROR);
		const target = this.requireTarget();
		if (this.listing === 'loading') throw new Error('the parameter list is still being read: stage after it is complete');
		const row = [...this.rows.values()].find((r) => r.linkId === linkId);
		if (!row) throw new Error(`the vehicle has no parameter ${linkId}`);
		if (row.type !== type) throw new Error(`${linkId} is ${row.type}, not ${type}`);
		if (row.locked) throw new Error(`${linkId} is locked`);
		if (this.identityRaw === null) throw new Error('the vehicle identity is not known yet');
		const armed = get(this.session.state).armed;
		if (armed) throw new Error('the vehicle is armed: parameters are written only while disarmed');
		const startedAt = this.now();
		const reasons: string[] = [];
		const heard = (text: string): number => reasons.push(text);
		this.noticeListeners.add(heard);
		const base = { sentBits: bits, source, sourceAttempts: 0, at: startedAt };
		let result: StageResult;
		try {
			this.session.send({
				kind: 'ParamSet', target_system: target.systemId, target_component: target.componentId,
				param_id: encodeLinkId(linkId), ...paramValueFields(bits), param_type: type === 'f32' ? PARAM_TYPE_REAL32 : PARAM_TYPE_INT32
			});
			// The immediate PARAM_VALUE is only a hint: without a correlation id it may be a reply that was already on its way.
			// The verdict is an explicit read-back: wait out the refusal grace (collecting the STATUSTEXT reasons), ask for the
			// parameter, and take the first reply that arrives after that request. The vehicle serves the link in order, so that
			// reply reflects the write.
			let hint: Extract<Message, { kind: 'ParamValue' }> | null = null;
			const watch = this.session.bus.on('ParamValue', (m) => {
				if (textOf(m.param_id) === linkId) hint = m;
			});
			await this.sleep(this.timing.refusalGraceMs);
			watch();
			const answer = await this.readBack(target, linkId, row.index);
			if (!answer) {
				const hinted = hint as Extract<Message, { kind: 'ParamValue' }> | null;
				result = {
					...base, accepted: false, echoBits: hinted ? paramValueBits(hinted) : null, reasons: [...reasons], sourceStored: null, sourceState: 'none',
					note: 'the vehicle did not answer the read-back after the write'
				};
			} else {
				const echoBits = paramValueBits(answer);
				const echoType = answer.param_type === PARAM_TYPE_REAL32 ? 'f32' : answer.param_type === PARAM_TYPE_INT32 ? 'i32' : null;
				result = { ...base, accepted: echoBits === bits && echoType === type, echoBits, reasons: [...reasons], sourceStored: null, sourceState: 'none', note: null };
			}
		} finally {
			this.noticeListeners.delete(heard);
		}
		this.results = { ...this.results, [linkId]: result };
		this.publish();
		if (result.accepted) result = await this.storeSource(linkId, row.index, result);
		return result;
	}

	/**
	 * Writes the source text of an accepted value (again when it did not read back) and records the outcome with a visible state.
	 * A source SET has no per-chunk ack, so success is the read-back only.
	 */
	private async storeSource(linkId: string, index: number, start: StageResult): Promise<StageResult> {
		let result = start;
		const bytes = encoder.encode(start.source);
		for (let attempt = result.sourceAttempts + 1, made = 0; made < this.timing.sourceAttempts; attempt++, made++) {
			result = { ...result, sourceState: 'writing', sourceStored: null, sourceAttempts: attempt, note: attempt > 1 ? `writing the source text again (attempt ${attempt})` : null };
			this.results = { ...this.results, [linkId]: result };
			this.publish();
			const stored = await this.writeSource(index, bytes, start.source);
			result = {
				...result,
				sourceStored: stored,
				sourceState: stored ? 'stored' : 'failed',
				note: stored ? null : 'the source text read back from the vehicle differs from the one sent'
			};
			if (stored) break;
		}
		this.results = { ...this.results, [linkId]: result };
		this.publish();
		return result;
	}

	/**
	 * Writes the source text of a value the vehicle accepted again, after the first write did not read back (the value stays
	 * staged with the placeholder source until this succeeds).
	 */
	retrySource(linkId: string): Promise<StageResult> {
		return this.enqueue(async () => {
			const last = this.results[linkId];
			if (!last || !last.accepted) throw new Error(`${linkId} has no accepted write whose source could be retried`);
			if (last.sourceState === 'stored') return last;
			const row = [...this.rows.values()].find((r) => r.linkId === linkId);
			if (!row) throw new Error(`the vehicle has no parameter ${linkId}`);
			this.requireTarget();
			return this.storeSource(linkId, row.index, last);
		});
	}

	/** Asks for one parameter and resolves with the first PARAM_VALUE for it that arrives after the request was sent. */
	private readBack(target: { systemId: number; componentId: number }, linkId: string, index: number): Promise<Extract<Message, { kind: 'ParamValue' }> | null> {
		this.session.send({
			kind: 'ParamRequestRead', target_system: target.systemId, target_component: target.componentId,
			param_id: new Uint8Array(PARAM_ID_BYTES), param_index: index
		});
		return this.waitFor('ParamValue', (m) => textOf(m.param_id) === linkId, this.timing.replyMs).promise;
	}

	private async writeSource(index: number, bytes: Uint8Array, text: string): Promise<boolean> {
		const target = this.requireTarget();
		const hash = this.identityRaw as bigint;
		for (let offset = 0; offset < bytes.length; offset += SOURCE_CHUNK) {
			const piece = bytes.subarray(offset, offset + SOURCE_CHUNK);
			const chunk = new Uint8Array(SOURCE_CHUNK);
			chunk.set(piece);
			this.session.send({
				kind: 'MarvParamSource', param_schema_hash: hash, param_index: index, offset, total_length: bytes.length,
				target_system: target.systemId, target_component: target.componentId, operation: MarvParamSourceOp.Set,
				chunk_length: piece.length, chunk
			});
		}
		// Read the record again, then the text: the check that the vehicle stored what was sent.
		const record = this.waitFor('MarvParamRecord', (m) => m.param_index === index, this.timing.replyMs);
		this.session.send({
			kind: 'ParamRequestRead', target_system: target.systemId, target_component: target.componentId,
			param_id: new Uint8Array(PARAM_ID_BYTES), param_index: index
		});
		if (!(await record.promise)) return false;
		try {
			return (await this.fetchSource(index)) === text;
		} catch {
			return false;
		}
	}

	// ---- internals: source reads ----------------------------------------------------------------------------------------

	private async fetchSource(index: number): Promise<string> {
		const row = this.rows.get(index);
		if (!row || row.sourceLength === null) throw new Error('the parameter record is not known yet');
		if (row.source !== null) return row.source;
		const target = this.requireTarget();
		const hash = this.identityRaw;
		if (hash === null) throw new Error('the vehicle identity is not known yet');
		const total = row.sourceLength;
		for (let attempts = 0; (this.sourceBytes.get(index)?.length ?? 0) < total; ) {
			const have = this.sourceBytes.get(index)?.length ?? 0;
			const wait = this.waitFor('MarvParamSource', (m) => m.operation === MarvParamSourceOp.Data && m.param_index === index && m.offset === have, this.timing.replyMs);
			this.session.send({
				kind: 'MarvParamSource', param_schema_hash: hash, param_index: index, offset: have, total_length: 0,
				target_system: target.systemId, target_component: target.componentId, operation: MarvParamSourceOp.Read,
				chunk_length: 0, chunk: new Uint8Array(SOURCE_CHUNK)
			});
			const answer = await wait.promise;
			if (!answer) {
				if (++attempts > 2) throw new Error('the vehicle did not send the source text');
			}
		}
		return this.rows.get(index)?.source ?? '';
	}

	private onSourceData(m: Extract<Message, { kind: 'MarvParamSource' }>): void {
		if (m.operation !== MarvParamSourceOp.Data) return;
		const row = this.rows.get(m.param_index);
		if (!row) return;
		const have = this.sourceBytes.get(m.param_index) ?? new Uint8Array(0);
		if (m.offset !== have.length) return;
		const next = new Uint8Array(have.length + m.chunk_length);
		next.set(have);
		next.set(m.chunk.subarray(0, m.chunk_length), have.length);
		this.sourceBytes.set(m.param_index, next);
		this.updateSource(row, next);
	}

	private updateSource(row: VehicleParam, bytes: Uint8Array): void {
		const complete = row.sourceLength !== null && bytes.length >= row.sourceLength;
		const text = decoder.decode(bytes);
		this.rows.set(row.index, { ...row, sourcePartial: text, source: complete ? text : null });
		this.touch();
		this.publish();
	}

	// ---- internals: incoming messages ------------------------------------------------------------------------------------

	private onValue(m: Extract<Message, { kind: 'ParamValue' }>): void {
		const linkId = textOf(m.param_id);
		const type: ParamType | null = m.param_type === PARAM_TYPE_REAL32 ? 'f32' : m.param_type === PARAM_TYPE_INT32 ? 'i32' : null;
		let index = m.param_index;
		if (index === INDEX_NONE) {
			const found = [...this.rows.values()].find((r) => r.linkId === linkId);
			if (!found) return;
			index = found.index;
		}
		if (type === null) return;
		this.count = m.param_count;
		const row = this.rows.get(index) ?? emptyParam(index);
		this.rows.set(index, { ...row, linkId, type, stagedBits: paramValueBits(m) });
		this.touch();
		this.publish();
	}

	private onRecord(m: Extract<Message, { kind: 'MarvParamRecord' }>): void {
		const row = this.rows.get(m.param_index) ?? emptyParam(m.param_index);
		this.count = m.param_count;
		const first = m.source_chunk.subarray(0, Math.min(m.source_chunk_length, m.source_length));
		const known = this.sourceBytes.get(m.param_index);
		const keep = known !== undefined && row.sourceLength === m.source_length && known.length >= first.length;
		if (!keep) this.sourceBytes.set(m.param_index, first.slice());
		const bytes = this.sourceBytes.get(m.param_index) as Uint8Array;
		const complete = bytes.length >= m.source_length;
		const text = decoder.decode(bytes);
		this.rows.set(m.param_index, {
			...row,
			activeBits: m.active_bits,
			stagedBits: m.staged_bits,
			sigma: m.sigma,
			sigmaBits: m.sigma_bits ?? f32ToBits(m.sigma),
			origin: pick(ORIGINS, m.origin, 'Manual'),
			method: pick(METHODS, m.method, 'scenario'),
			sigmaKind: pick(SIGMA_KINDS, m.sigma_kind, 'Choice'),
			locked: m.lock !== 0,
			hasRecord: true,
			sourceLength: m.source_length,
			sourcePartial: text,
			source: complete ? text : null
		});
		this.touch();
		this.publish();
	}

	private onStatusText(m: Extract<Message, { kind: 'Statustext' }>): void {
		let text: string;
		if (m.id === 0) {
			text = textOf(m.text);
		} else {
			const parts = this.textParts.get(m.id) ?? [];
			parts[m.chunk_seq] = decoder.decode(m.text.subarray(0, m.text.indexOf(0) < 0 ? STATUS_TEXT_BYTES : m.text.indexOf(0)));
			this.textParts.set(m.id, parts);
			if (m.text.indexOf(0) < 0) return;
			this.textParts.delete(m.id);
			text = parts.map((p) => p ?? '').join('');
		}
		this.notices = [...this.notices, { at: this.now(), severity: m.severity, text }].slice(-NOTICE_LIMIT);
		for (const l of this.noticeListeners) l(text);
		this.publish();
	}

	// ---- internals: plumbing ------------------------------------------------------------------------------------------

	private requireTarget(): { systemId: number; componentId: number } {
		const target = this.session.target();
		if (!target) throw new Error('no vehicle: no heartbeat has been seen');
		return target;
	}

	private touch(): void {
		this.lastActivity = this.now();
	}

	private missingIndices(): number[] {
		const count = this.count;
		if (count === null) return [0];
		const out: number[] = [];
		for (let i = 0; i < count; i++) {
			const r = this.rows.get(i);
			if (!r || r.linkId === null || !r.hasRecord) out.push(i);
		}
		return out;
	}

	/** Resolves true when every index has its value and record, false after the listing went idle. */
	private waitIdleOrComplete(): Promise<boolean> {
		return new Promise((resolve) => {
			const check = (): void => {
				if (this.count !== null && this.missingIndices().length === 0) return finish(true);
				if (this.now() - this.lastActivity >= this.timing.listingIdleMs) return finish(false);
				timer = setTimeout(check, Math.max(1, this.timing.listingIdleMs - (this.now() - this.lastActivity)));
			};
			const finish = (ok: boolean): void => {
				if (timer) clearTimeout(timer);
				off.forEach((f) => f());
				resolve(ok);
			};
			let timer: ReturnType<typeof setTimeout> | null = null;
			const off = [this.session.bus.on('ParamValue', check), this.session.bus.on('MarvParamRecord', check)];
			check();
		});
	}

	private waitFor<K extends Message['kind']>(kind: K, predicate: (m: Extract<Message, { kind: K }>) => boolean, ms: number): Waiter & { promise: Promise<Extract<Message, { kind: K }> | null> } {
		let cancel = (): void => undefined;
		const promise = new Promise<Extract<Message, { kind: K }> | null>((resolve) => {
			const off = this.session.bus.on(kind, (m) => {
				if (!predicate(m)) return;
				cancel();
				resolve(m);
			});
			const timer = setTimeout(() => {
				cancel();
				resolve(null);
			}, ms);
			cancel = () => {
				off();
				clearTimeout(timer);
			};
		});
		return { promise, cancel: () => cancel() };
	}

	private sleep(ms: number): Promise<void> {
		return new Promise((resolve) => setTimeout(resolve, ms));
	}

	private enqueue<T>(fn: () => Promise<T>): Promise<T> {
		const run = this.chain.then(fn, fn);
		this.chain = run.catch(() => undefined);
		return run;
	}

	private publish(): void {
		this.store.set({
			identity: this.identity,
			bytewise: this.bytewise,
			count: this.count,
			params: [...this.rows.values()].sort((a, b) => a.index - b.index),
			listing: this.listing,
			missing: this.missing,
			notices: this.notices,
			results: this.results,
			configAck: this.configAck
		});
	}
}
