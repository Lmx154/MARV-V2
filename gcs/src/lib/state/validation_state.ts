// The validation and activation of the staged config as the Parameters page drives them: read every full source text, build the
// mcfg view, have the helper validate it (progress lines as they come), fetch the report and verify its id, and activate only when
// whyCannotActivate says nothing. One controller per page; the dependencies are injected so that the flow runs without a browser.
import { derived, get, writable, type Readable } from 'svelte/store';
import type { HelperEvent, HelperReply } from '$link/transport';
import type { ConfigAction, ConfigCommandResult, VehicleState } from '$link/vehicle';
import type { Manifest } from '$lib/params/manifest';
import {
	buildStagedView,
	makeRecord,
	paramsWithoutFullSource,
	stagedFingerprint,
	whyCannotActivate,
	type ValidationRecord
} from '$lib/params/validation';

export type ValidationPhase = 'idle' | 'reading' | 'validating' | 'done' | 'failed';

export interface ValidationState {
	phase: ValidationPhase;
	/** The helper's progress lines of the run in progress or the last one. */
	progress: string[];
	record: ValidationRecord | null;
	error: string | null;
	/** The config hash the helper computed, and the fingerprint of the staged config it was computed for. */
	computed: { fingerprint: string; configHash: string } | null;
	/** What the last config command (activate, revert, reset) answered. */
	command: { action: ConfigAction; result: ConfigCommandResult | 'error'; text: string } | null;
	busy: boolean;
}

export interface ValidationDeps {
	request(cmd: string, args?: Record<string, unknown>): Promise<HelperReply>;
	onEvent(handler: (event: HelperEvent) => void): () => void;
	report(imageHash: string, configHash: string): Promise<Uint8Array>;
	vehicle: {
		state: Readable<VehicleState>;
		readSource(index: number): Promise<string>;
		configCommand(action: ConfigAction, args?: { reportId?: Uint8Array; expectedConfigSha256?: Uint8Array }): Promise<ConfigCommandResult>;
	};
	/** The manifest of the connected image, or null. */
	manifest(): Manifest | null;
}

const bytesOfHex = (hex: string): Uint8Array => Uint8Array.from(hex.match(/../g) ?? [], (h) => parseInt(h, 16));
const messageOf = (e: unknown): string => (e instanceof Error ? e.message : String(e));

export class ValidationController {
	private readonly store = writable<ValidationState>({ phase: 'idle', progress: [], record: null, error: null, computed: null, command: null, busy: false });
	readonly state: Readable<ValidationState> = { subscribe: this.store.subscribe };
	/** null until the vehicle is listed. */
	readonly fingerprint: Readable<string>;

	constructor(private readonly deps: ValidationDeps) {
		this.fingerprint = derived(deps.vehicle.state, (v) => stagedFingerprint(v.params));
	}

	/** Why the Activate button is off now (the synchronous gate); null when it may be pressed. */
	whyCannotActivate(vehicle: VehicleState, s: ValidationState, fingerprint: string): string | null {
		return whyCannotActivate({
			record: s.record,
			vehicleImageSha256: vehicle.identity?.imageSha256 ?? null,
			stagedConfigHash: s.computed && s.computed.fingerprint === fingerprint ? s.computed.configHash : null,
			fingerprint
		});
	}

	private set(patch: Partial<ValidationState>): void {
		this.store.update((s) => ({ ...s, ...patch }));
	}

	/** Reads every source text the records did not carry in full, then returns the staged view and its fingerprint. */
	private async stagedView() {
		const manifest = this.deps.manifest();
		if (!manifest) throw new Error('no manifest of the connected image is loaded');
		const vehicle = this.deps.vehicle;
		if (get(vehicle.state).identity === null) throw new Error('the vehicle has not reported its identity');
		for (const index of paramsWithoutFullSource(manifest, get(vehicle.state).params)) await vehicle.readSource(index);
		const params = get(vehicle.state).params;
		return { view: buildStagedView(manifest, params), fingerprint: stagedFingerprint(params) };
	}

	/** Validates the staged config for the vehicle's image: the helper runs the validator; the report is fetched and checked here. */
	async validate(): Promise<void> {
		if (get(this.store).busy) return;
		this.set({ busy: true, phase: 'reading', progress: [], error: null, record: null, command: null });
		let off: () => void = () => undefined;
		try {
			const { view, fingerprint } = await this.stagedView();
			const identity = get(this.deps.vehicle.state).identity;
			if (!identity) throw new Error('the vehicle has not reported its identity');
			const imageHash = identity.imageSha256;
			this.set({ phase: 'validating' });
			let configHash = '';
			let done: ((e: HelperEvent) => void) | null = null;
			const finished = new Promise<HelperEvent>((resolve) => (done = resolve));
			off = this.deps.onEvent((event) => {
				if (event.image_hash !== imageHash || (configHash !== '' && event.config_hash !== configHash)) return;
				if (event.event === 'validate_progress') this.store.update((s) => ({ ...s, progress: [...s.progress, String(event.line)] }));
				else if (event.event === 'validate_done') done?.(event);
			});
			const reply = await this.deps.request('validate', { image_hash: imageHash, view });
			configHash = String(reply.config_hash);
			this.set({ computed: { fingerprint, configHash } });
			const end = await finished;
			if (typeof end.report_id !== 'string' || typeof end.verdict !== 'string') {
				throw new Error('the validator produced no report (see the progress lines)');
			}
			const bytes = await this.deps.report(imageHash, configHash);
			const record = await makeRecord(imageHash, configHash, end.report_id, bytes, fingerprint);
			this.set({ record, phase: 'done' });
			if (record.report === null) throw new Error(`the report cannot be read: ${record.parseError}`);
			if (record.reportId !== record.computedId) throw new Error('the report id announced by the helper is not the SHA-256 of the report');
		} catch (e) {
			this.set({ error: messageOf(e), phase: get(this.store).record ? 'done' : 'failed' });
		} finally {
			off();
			this.set({ busy: false });
		}
	}

	/**
	 * Activates: the staged config is rebuilt and hashed again, the gate is asked again with those fresh facts, and only then does the
	 * command go out, carrying the report id and the config hash the report is for.
	 */
	async activate(): Promise<void> {
		if (get(this.store).busy) return;
		this.set({ busy: true, command: null, error: null });
		try {
			const { view, fingerprint } = await this.stagedView();
			const identity = get(this.deps.vehicle.state).identity;
			const record = get(this.store).record;
			const reply = await this.deps.request('config_report', { image_hash: identity?.imageSha256 ?? '', view });
			const configHash = String(reply.config_hash);
			this.set({ computed: { fingerprint, configHash } });
			const why = whyCannotActivate({ record, vehicleImageSha256: identity?.imageSha256 ?? null, stagedConfigHash: configHash, fingerprint });
			if (why !== null || record === null) {
				this.set({ command: { action: 'activate', result: 'error', text: `not sent: ${why}` } });
				return;
			}
			const result = await this.deps.vehicle.configCommand('activate', {
				reportId: bytesOfHex(record.reportId),
				expectedConfigSha256: bytesOfHex(configHash)
			});
			this.set({ command: { action: 'activate', result, text: result === 'accepted' ? 'accepted: the vehicle restarts to apply it' : result } });
		} catch (e) {
			this.set({ command: { action: 'activate', result: 'error', text: messageOf(e) } });
		} finally {
			this.set({ busy: false });
		}
	}

	/** Revert-staged and reset-to-factory: no report is involved; the vehicle answers. */
	async command(action: 'revert' | 'reset-factory'): Promise<ConfigCommandResult | 'error'> {
		if (get(this.store).busy) return 'error';
		this.set({ busy: true, command: null });
		try {
			const result = await this.deps.vehicle.configCommand(action);
			this.set({ command: { action, result, text: result }, record: null, computed: null, phase: 'idle', progress: [] });
			return result;
		} catch (e) {
			this.set({ command: { action, result: 'error', text: messageOf(e) } });
			return 'error';
		} finally {
			this.set({ busy: false });
		}
	}
}
