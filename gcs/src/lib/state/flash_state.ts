// The Flash page's flow (ground-segment unit 15, decision 0053 Q6): check the gate with the helper, offer the existing validation
// for a missing report, and flash through the gate: re-check, ask the vehicle for the maintenance grant, have the helper run
// picotool, then wait for the vehicle to come back and compare the identity it reports with the identity the plan expects.
// The dependencies are injected so that the flow runs without a browser.
import { derived, get, writable, type Readable } from 'svelte/store';
import type { HelperEvent, HelperReply } from '$link/transport';
import type { MaintenanceGrant, VehicleState } from '$link/vehicle';
import type { SessionState } from '$link/session';
import {
	boardFacts,
	checkIdentity,
	planKey,
	whyCannotFlash,
	type BoardFacts,
	type FlashExpected,
	type FlashPlan,
	type IdentityCheck
} from '$lib/flash/flash';

export type FlashPhase = 'idle' | 'checking' | 'validating' | 'granting' | 'flashing' | 'reconnecting' | 'done' | 'failed';

export interface FlashResult {
	/** The vehicle reports the expected image hash, config hash and config source. */
	verified: boolean;
	checks: IdentityCheck[];
	/** What the vehicle says of the config it runs (a stored config bound to the previous image is refused until it is activated again). */
	configState: string;
	configRefusal: string;
}

export interface FlashState {
	phase: FlashPhase;
	busy: boolean;
	image: string | null;
	plan: FlashPlan | null;
	/** planKey() of what `plan` was made for. */
	planFor: string | null;
	planError: string | null;
	/** The helper's lines of the validation and of the flash in progress or the last one. */
	validateProgress: string[];
	progress: string[];
	result: FlashResult | null;
	error: string | null;
	/** The vehicle holds the maintenance grant: arming is refused until it restarts. */
	inMaintenance: boolean;
}

export interface FlashTiming {
	/** PLACEHOLDER harness value: the helper bounds its own steps (convert 60 s, load 180 s, port 60 s); this bounds the wait for its flash_done. */
	flashDoneMs: number;
	/** PLACEHOLDER harness value: how long the vehicle may take to report its identity after the flash. */
	identityMs: number;
	/** PLACEHOLDER harness value: the identity is asked for again this often while waiting. */
	identityRetryMs: number;
}

export const DEFAULT_FLASH_TIMING: FlashTiming = { flashDoneMs: 400_000, identityMs: 60_000, identityRetryMs: 2_000 };

export interface FlashDeps {
	request(cmd: string, args?: Record<string, unknown>): Promise<HelperReply>;
	onEvent(handler: (event: HelperEvent) => void): () => void;
	vehicle: {
		state: Readable<VehicleState>;
		requestMaintenanceGrant(): Promise<MaintenanceGrant>;
		requestIdentity(): void;
	};
	session: { state: Readable<SessionState> };
	/** The helper WebSocket state and the device it has open. */
	helperState: Readable<string>;
	device: Readable<string | null>;
	timing?: Partial<FlashTiming>;
}

const messageOf = (e: unknown): string => (e instanceof Error ? e.message : String(e));
const MAINTENANCE_NOTE = 'The vehicle holds the maintenance grant and refuses to arm until it restarts (power-cycle it).';

export class FlashController {
	private readonly store = writable<FlashState>({
		phase: 'idle', busy: false, image: null, plan: null, planFor: null, planError: null, validateProgress: [], progress: [], result: null,
		error: null, inMaintenance: false
	});
	readonly state: Readable<FlashState> = { subscribe: this.store.subscribe };
	/** null = Flash may be pressed, else why not; follows the vehicle, the heartbeat and the plan. */
	readonly whyCannot: Readable<string | null>;
	private readonly timing: FlashTiming;

	constructor(private readonly deps: FlashDeps) {
		this.timing = { ...DEFAULT_FLASH_TIMING, ...deps.timing };
		this.whyCannot = derived(
			[this.store, deps.vehicle.state, deps.session.state, deps.helperState, deps.device],
			([s, v, session, helper, device]) =>
				whyCannotFlash({
					helper,
					device,
					heartbeat: session.heartbeat,
					armed: session.armed,
					identity: v.identity,
					image: s.image,
					plan: s.plan,
					planFor: s.planFor,
					planError: s.planError,
					busy: s.busy
				})
		);
	}

	private set(patch: Partial<FlashState>): void {
		this.store.update((s) => ({ ...s, ...patch }));
	}

	private facts(): BoardFacts | null {
		return boardFacts(get(this.deps.vehicle.state).identity, get(this.deps.session.state).armed);
	}

	/** Picks (or clears) the image; the plan of the previous pick no longer applies. */
	setImage(hash: string | null): void {
		if (get(this.store).busy) return;
		this.set({ image: hash, plan: null, planFor: null, planError: null, result: null, error: null, validateProgress: [], progress: [], phase: 'idle' });
	}

	/** Asks the helper what flashing the picked image onto this vehicle would run and whether the gate passes. */
	async check(): Promise<void> {
		const s = get(this.store);
		if (s.busy || !s.image) return;
		this.set({ busy: true, phase: 'checking' });
		try {
			await this.plan(s.image);
		} finally {
			this.set({ busy: false, phase: 'idle' });
		}
	}

	private async plan(image: string): Promise<FlashPlan | null> {
		const facts = this.facts();
		if (!facts) {
			this.set({ plan: null, planFor: null, planError: 'the vehicle has not reported its identity' });
			return null;
		}
		try {
			const reply = await this.deps.request('flash_plan', { image_hash: image, board: facts });
			const plan = reply.plan as FlashPlan;
			this.set({ plan, planFor: planKey(image, facts), planError: null });
			return plan;
		} catch (e) {
			this.set({ plan: null, planFor: null, planError: messageOf(e) });
			return null;
		}
	}

	/** Validates the config the board will run for the picked image (the helper's existing validation), then checks the gate again. */
	async validate(): Promise<void> {
		const s = get(this.store);
		if (s.busy || !s.image || !s.plan) return;
		const image = s.image;
		const plan = s.plan;
		this.set({ busy: true, phase: 'validating', validateProgress: [], error: null, result: null });
		let off: () => void = () => undefined;
		try {
			let done: ((e: HelperEvent) => void) | null = null;
			const finished = new Promise<HelperEvent>((resolve) => (done = resolve));
			off = this.deps.onEvent((event) => {
				if (event.image_hash !== image || event.config_hash !== plan.config.hash) return;
				if (event.event === 'validate_progress') this.store.update((st) => ({ ...st, validateProgress: [...st.validateProgress, String(event.line)] }));
				else if (event.event === 'validate_done') done?.(event);
			});
			await this.deps.request('validate', { image_hash: image, config: plan.config.which === 'factory' ? 'factory' : plan.config.hash });
			const end = await finished;
			if (typeof end.report_id !== 'string') this.set({ error: 'the validator produced no report (see the progress lines)' });
			await this.plan(image);
		} catch (e) {
			this.set({ error: messageOf(e) });
		} finally {
			off();
			this.set({ busy: false, phase: 'idle' });
		}
	}

	private fail(error: string, extra: Partial<FlashState> = {}): void {
		this.set({ phase: 'failed', error, busy: false, ...extra });
	}

	/**
	 * Flashes the picked image: the synchronous gate first, then the helper's plan again (the report store is read afresh), then the
	 * maintenance grant, the helper's flash, and the read-back of the identity over the link.
	 */
	async flash(): Promise<void> {
		const before = get(this.store);
		if (before.busy) return;
		const why = get(this.whyCannot);
		if (why !== null || !before.image) {
			this.set({ error: `not sent: ${why ?? 'pick an image'}`, phase: 'failed' });
			return;
		}
		const image = before.image;
		this.set({ busy: true, phase: 'checking', error: null, result: null, progress: [] });
		let off: () => void = () => undefined;
		try {
			const plan = await this.plan(image);
			const facts = this.facts();
			if (!plan || !facts) return this.fail(`not sent: ${get(this.store).planError ?? 'the gate could not be checked'}`);
			if (facts.armed !== false) return this.fail('not sent: the vehicle is not reported disarmed');
			if (!plan.gate.passes) return this.fail(`not sent: ${plan.gate.reason ?? 'no passing validation report'}`);

			this.set({ phase: 'granting' });
			const grant = await this.deps.vehicle.requestMaintenanceGrant();
			if (grant !== 'granted') {
				return this.fail(
					grant === 'denied: armed'
						? 'the vehicle refused the maintenance grant: it is armed. Nothing was flashed.'
						: `the vehicle did not grant maintenance (${grant}). Nothing was flashed.`
				);
			}
			this.set({ inMaintenance: true });

			// From here on the vehicle is expected to disappear and come back: the identity is read afresh.
			let sawReset = false;
			const unsubscribeState = this.deps.vehicle.state.subscribe((v) => {
				if (v.identity === null) sawReset = true;
			});
			let finished: (e: HelperEvent) => void = () => undefined;
			const flashDone = new Promise<HelperEvent>((resolve) => (finished = resolve));
			const offEvents = this.deps.onEvent((event) => {
				if (event.image_hash !== image) return;
				if (event.event === 'flash_progress') this.store.update((st) => ({ ...st, progress: [...st.progress, `${String(event.stage)}: ${String(event.line)}`] }));
				else if (event.event === 'flash_done') finished(event);
			});
			off = () => {
				unsubscribeState();
				offEvents();
			};
			this.set({ phase: 'flashing' });
			let reply: HelperReply;
			try {
				reply = await this.deps.request('flash', { image_hash: image, board: facts });
			} catch (e) {
				return this.fail(`the helper refused to flash: ${messageOf(e)}. ${MAINTENANCE_NOTE}`);
			}
			const expected = reply.expected as FlashExpected;
			const timer = new Promise<null>((resolve) => setTimeout(() => resolve(null), this.timing.flashDoneMs));
			const end = await Promise.race([flashDone, timer]);
			if (end === null) return this.fail('the helper did not report the end of the flash in time');
			if (end.ok !== true) {
				return this.fail(`the flash failed at ${String(end.stage)}: ${String(end.error)}`);
			}

			this.set({ phase: 'reconnecting' });
			const identity = await this.identityAfterReset(() => sawReset);
			if (!identity) return this.fail(`the flash finished but the vehicle did not report its identity within ${this.timing.identityMs / 1000} s`);
			const checks = checkIdentity(identity, expected);
			const verified = checks.every((c) => c.ok);
			this.set({
				phase: verified ? 'done' : 'failed',
				error: verified ? null : `the vehicle reports ${checks.filter((c) => !c.ok).map((c) => `${c.what} ${c.got.slice(0, 12)} (expected ${c.expected.slice(0, 12)})`).join(', ')}`,
				result: { verified, checks, configState: identity.configState, configRefusal: identity.configRefusal },
				inMaintenance: false
			});
		} catch (e) {
			this.fail(messageOf(e));
		} finally {
			off();
			this.set({ busy: false });
		}
	}

	/** The identity the vehicle reports once it was reset (the link closed and opened again) and has spoken again; null at the timeout. */
	private async identityAfterReset(sawReset: () => boolean): Promise<VehicleState['identity']> {
		const deadline = Date.now() + this.timing.identityMs;
		let lastAsk = 0;
		for (;;) {
			const identity = get(this.deps.vehicle.state).identity;
			if (identity && sawReset()) return identity;
			const now = Date.now();
			if (now >= deadline) return null;
			if (now - lastAsk >= this.timing.identityRetryMs) {
				lastAsk = now;
				try {
					this.deps.vehicle.requestIdentity();
				} catch {
					// no heartbeat yet: the vehicle client asks at its first one
				}
			}
			await new Promise((resolve) => setTimeout(resolve, Math.min(50, this.timing.identityRetryMs)));
		}
	}
}
