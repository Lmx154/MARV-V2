// The Radio page (GCS-first unit GX-7): the receiver's 16 channels, the channel each control uses, the switch thresholds against the
// live channel, the display checks and the lock display. computeRadio() is pure (the same snapshot, rows and `now` give the same view),
// so every value, warning and check is tested without a browser.
//
// What the page never does: it has no write path of its own. A remap is a parameter value staged through the vehicle client
// (VehicleClient.stage, the page's Stage button), validated against the G7 gate and activated by the Parameters page's own flow
// (ValidationController); the page only warns about a collision (rule R-1), the vehicle's composition validate refuses it for real.
import { readable, writable, type Readable } from 'svelte/store';
import { bitsToValue } from '$link/bits';
import type { TelemetryClient, TelemetrySnapshot } from '$link/telemetry';
import type { ParamRow } from '$lib/params/merge';
import { SHELL_TIMING, type ShellTiming, type Tone } from './shell';

export type FnId = 'roll' | 'pitch' | 'throttle' | 'yaw' | 'arm' | 'angle' | 'bench';

export interface FnSpec {
	id: FnId;
	label: string;
	/** The parameter that holds the 1-based channel number. */
	param: string;
	/** The parameter that holds the switch threshold (normalised), or null for a stick. */
	thresholdParam: string | null;
	kind: 'stick' | 'switch';
	/** 0 is allowed and means "no such control" (the angle switch: acro always). */
	noneAllowed: boolean;
}

/** The controls of the radio, in the order the page lists them. The bench-profile switch exists only in the bench-profile image. */
export const FUNCTIONS: readonly FnSpec[] = [
	{ id: 'roll', label: 'Roll', param: 'rc_channel_roll', thresholdParam: null, kind: 'stick', noneAllowed: false },
	{ id: 'pitch', label: 'Pitch', param: 'rc_channel_pitch', thresholdParam: null, kind: 'stick', noneAllowed: false },
	{ id: 'throttle', label: 'Throttle', param: 'rc_channel_throttle', thresholdParam: null, kind: 'stick', noneAllowed: false },
	{ id: 'yaw', label: 'Yaw', param: 'rc_channel_yaw', thresholdParam: null, kind: 'stick', noneAllowed: false },
	{ id: 'arm', label: 'Arm switch', param: 'rc_channel_arm', thresholdParam: 'arm_switch_threshold', kind: 'switch', noneAllowed: false },
	{ id: 'angle', label: 'Angle switch', param: 'rc_channel_angle', thresholdParam: 'angle_switch_threshold', kind: 'switch', noneAllowed: true },
	{ id: 'bench', label: 'Bench-profile switch', param: 'bench_profile_rc_channel', thresholdParam: 'bench_profile_switch_threshold', kind: 'switch', noneAllowed: false }
];

const FN_BY_ID = new Map(FUNCTIONS.map((f) => [f.id, f]));
export const fnLabel = (id: FnId): string => FN_BY_ID.get(id)?.label ?? id;

/** The throttle-low check reads this parameter; the arming rule compares the throttle with it (fw/arming: throttle must be below). */
export const ARM_THROTTLE_PARAM = 'arm_throttle_min';
/** The radio's other locked entries listed beside the channels (the failsafe times are listed by the prefix). */
const LOCK_LIST_NAMES: readonly string[] = [ARM_THROTTLE_PARAM, 'airmode_activate_throttle'];
const LOCK_LIST_PREFIXES: readonly string[] = ['failsafe_'];

export const CHANNELS = 16;

/** The receiver's contract (fw/prim constants kCrsfUsCentre, kRcUsHalfSpan, kRcUsThrottleMin, kRcUsThrottleSpan): x = (us - 1500) / 512, throttle = (us - 988) / 1024. */
const RC_US_CENTRE = 1500;
const RC_US_HALF_SPAN = 512;
const RC_US_THROTTLE_MIN = 988;
const RC_US_THROTTLE_SPAN = 1024;
const PERCENT = 100;

/**
 * PLACEHOLDER (a display check, no flight effect): a stick is "centred" when its normalised value is within this of zero. Reason: no
 * sourced stick-centre scatter of the RadioMaster exists yet, and Betaflight's rc_deadband is 0 by default (it does not give one).
 * Replaced by: a tolerance from the scatter measured on the TB (bench with Luis's RadioMaster, GX-7 UNKNOWN steps). 0.05 is 26 us.
 */
export const CENTRE_TOLERANCE = 0.05;
/**
 * PLACEHOLDER (a labelled harness value, display only): a channel counts as moved (detection, the unmapped-activity hint) when its
 * range since the page was opened, or since "Detect" was pressed, reaches this many microseconds. Reason: a switch travels the full
 * 1024 us (988 to 2012) and a deliberate stick move at least half that; 400 us is well above the centre scatter and the cross-talk of a
 * hand on a stick. Replaced by: a value from the scatter and travel measured on the TB with the RadioMaster.
 */
export const MOVED_US = 400;
/** How often the page is recomputed from the telemetry (ms, a labelled harness value: the display's frame period). */
export const RADIO_TICK_MS = 100;

const DASH = '—';
const clamp = (v: number, lo: number, hi: number): number => Math.min(hi, Math.max(lo, v));
/** x of the receive contract for a channel value in microseconds. */
export const normalised = (us: number): number => clamp((us - RC_US_CENTRE) / RC_US_HALF_SPAN, -1, 1);
/** The throttle of the receive contract for a channel value in microseconds. */
export const throttleOf = (us: number): number => clamp((us - RC_US_THROTTLE_MIN) / RC_US_THROTTLE_SPAN, 0, 1);

// ---- the channel map and rule R-1 -----------------------------------------------------------------------------------------------

export type ChannelMapping = Partial<Record<FnId, number>>;

export interface Finding {
	kind: 'collision' | 'range';
	functions: FnId[];
	text: string;
}

/**
 * Rule R-1 (the vehicle's composition validate and receiver start-up are the authority): the stick, arm and angle channels (and the
 * bench-profile channel, when the image has one) are pairwise distinct, each in 1 .. 16, the angle channel also 0 = none (not compared).
 * The page uses this to warn before a value is staged; it refuses nothing.
 */
export function findingsOf(map: ChannelMapping): Finding[] {
	const out: Finding[] = [];
	const used = new Map<number, FnId[]>();
	for (const spec of FUNCTIONS) {
		const channel = map[spec.id];
		if (channel === undefined) continue;
		if (!Number.isInteger(channel) || channel < (spec.noneAllowed ? 0 : 1) || channel > CHANNELS) {
			out.push({ kind: 'range', functions: [spec.id], text: `${spec.label}: ${channel} is outside ${spec.noneAllowed ? 0 : 1} .. ${CHANNELS}${spec.noneAllowed ? ' (0 = none)' : ''}` });
			continue;
		}
		if (channel === 0) continue;
		used.set(channel, [...(used.get(channel) ?? []), spec.id]);
	}
	for (const [channel, functions] of [...used].sort((a, b) => a[0] - b[0])) {
		if (functions.length < 2) continue;
		out.push({
			kind: 'collision',
			functions,
			text: `Channel ${channel} is used by ${functions.map(fnLabel).join(' and ')}: the stick, arm and angle channels must be pairwise distinct (R-1), the vehicle refuses this.`
		});
	}
	return out;
}

/** The findings about one control if it were set to `channel` in `map` (what the page warns about before it stages the value). */
export function candidateFindings(map: ChannelMapping, fn: FnId, channel: number): Finding[] {
	return findingsOf({ ...map, [fn]: channel }).filter((f) => f.functions.includes(fn));
}

// ---- excursions and detection ----------------------------------------------------------------------------------------------

/** The least and the greatest value each channel has had since the tracker began (null: never seen). */
export interface Excursions {
	min: ReadonlyArray<number | null>;
	max: ReadonlyArray<number | null>;
	frames: number;
}

export const emptyExcursions = (): Excursions => ({ min: Array(CHANNELS).fill(null), max: Array(CHANNELS).fill(null), frames: 0 });

export function observe(e: Excursions, channelsUs: ReadonlyArray<number | null>): Excursions {
	const min = [...e.min];
	const max = [...e.max];
	for (let i = 0; i < CHANNELS; i++) {
		const us = channelsUs[i];
		if (us === null || us === undefined) continue;
		min[i] = min[i] === null ? us : Math.min(min[i] as number, us);
		max[i] = max[i] === null ? us : Math.max(max[i] as number, us);
	}
	return { min, max, frames: e.frames + 1 };
}

/** How far channel `channel` (1-based) has moved (us); 0 if never seen. */
export const rangeUs = (e: Excursions, channel: number): number => {
	const lo = e.min[channel - 1];
	const hi = e.max[channel - 1];
	return lo === null || lo === undefined || hi === null || hi === undefined ? 0 : hi - lo;
};

/** The channels (1-based) that have moved at least `threshold`. */
export const movedChannels = (e: Excursions, threshold: number = MOVED_US): number[] =>
	Array.from({ length: CHANNELS }, (_, i) => i + 1).filter((c) => rangeUs(e, c) >= threshold);

/** "Move the switch you want for X": the channel that moved since Detect was pressed. */
export type Detection =
	| { phase: 'idle' }
	| { phase: 'listening'; fn: FnId; track: Excursions }
	| { phase: 'found'; fn: FnId; channel: number; moved: number; track: Excursions }
	/** Two or more channels moved in the same frame: move only the one wanted, and detect again. */
	| { phase: 'ambiguous'; fn: FnId; channels: number[]; track: Excursions };

export const IDLE: Detection = { phase: 'idle' };

export function beginDetection(fn: FnId): Detection {
	return { phase: 'listening', fn, track: emptyExcursions() };
}

/** One RC frame: a listening detection is decided by the first frame after which exactly one channel (or several) has moved; the others do not change. */
export function stepDetection(d: Detection, channelsUs: ReadonlyArray<number | null>): Detection {
	if (d.phase !== 'listening') return d;
	const track = observe(d.track, channelsUs);
	const moved = movedChannels(track);
	if (moved.length === 0) return { ...d, track };
	const [only] = moved;
	if (moved.length === 1 && only !== undefined) return { phase: 'found', fn: d.fn, channel: only, moved: rangeUs(track, only), track };
	return { phase: 'ambiguous', fn: d.fn, channels: moved, track };
}

// ---- the view ---------------------------------------------------------------------------------------------------------------

export interface Bar {
	channel: number;
	/** Null: the vehicle does not report this channel. */
	us: number | null;
	/** x of the receive contract, -1 .. 1; null with `us`. */
	x: number | null;
	/** The bar position, 0 .. 1 (x mapped); null with `us`. */
	bar: number | null;
	/** The controls that read this channel on the running config. */
	roles: FnId[];
	/** The controls that would read it once the staged config is active (differs from `roles` only for a staged remap). */
	stagedRoles: FnId[];
	/** How far the channel has moved since the page was opened (us). */
	movedUs: number;
	/** The channel has moved (at least MOVED_US) since the page was opened or reset. */
	active: boolean;
	/** While detecting: the channel moved since Detect was pressed (the highlight). */
	detected: boolean;
}

export interface MapRow {
	id: FnId;
	label: string;
	param: string;
	/** The image has the parameter and the vehicle reported it. */
	present: boolean;
	/** The channel on the running config; null if not known. */
	active: number | null;
	/** The channel in the staged config. */
	staged: number | null;
	/** Staged differs from active. */
	differs: boolean;
	locked: boolean;
	lockText: string | null;
	/** The parameter row, for staging; null if the vehicle did not report it. */
	row: ParamRow | null;
	/** The threshold of the switch: the active value (normalised). */
	threshold: number | null;
	/** The live value of the channel the control reads, from the running config. */
	liveUs: number | null;
}

export type SwitchPosition = 'at or above' | 'below' | 'unknown' | 'none';

export interface SwitchRow {
	id: FnId;
	label: string;
	channel: number | null;
	threshold: number | null;
	thresholdUs: number | null;
	x: number | null;
	us: number | null;
	position: SwitchPosition;
	/** The sentence the page shows. */
	text: string;
	tone: Tone;
}

export type CheckStatus = 'ok' | 'fail' | 'unknown';

export interface Check {
	id: 'centre.roll' | 'centre.pitch' | 'centre.yaw' | 'throttle.low';
	label: string;
	status: CheckStatus;
	detail: string;
}

export interface LockRow {
	name: string;
	locked: boolean;
	/** The ruling: who and when, and the way to change it. */
	text: string;
}

export interface RcState {
	heard: boolean;
	stale: boolean;
	ageMs: number | null;
	hz: number | null;
	/** 0..1; null: the receiver sent no link statistics. */
	linkQuality: number | null;
	channelCount: number;
}

export interface RadioView {
	rc: RcState;
	bars: Bar[];
	map: MapRow[];
	/** The mapping of the running config is known (the parameters were read from the vehicle). */
	mapKnown: boolean;
	findingsActive: Finding[];
	findingsStaged: Finding[];
	switches: SwitchRow[];
	checks: Check[];
	locks: LockRow[];
	detection: Detection;
	/** Warnings about channels that moved and are mapped to nothing while a switch's own channel has not moved. */
	hints: string[];
}

export interface RadioInput {
	snapshot: TelemetrySnapshot;
	rows: readonly ParamRow[];
	now: number;
	activity: Excursions;
	detection: Detection;
	timing?: ShellTiming;
}

const rowOf = (rows: readonly ParamRow[], name: string): ParamRow | null => rows.find((r) => r.name === name || r.linkId === name) ?? null;
const valueOf = (row: ParamRow | null, bits: number | null): number | null => (row && bits !== null ? bitsToValue(bits, row.type) : null);

export function lockTextOf(row: ParamRow): string {
	return row.lockInfo ? `locked by ${row.lockInfo.by} ${row.lockInfo.on} (${row.lockInfo.via}); change only by a firmware build` : 'locked; change only by a firmware build';
}

const fmt = (v: number, decimals: number): string => v.toFixed(decimals);

export function computeRadio(input: RadioInput): RadioView {
	const { snapshot, rows, now, activity, detection } = input;
	const timing = input.timing ?? SHELL_TIMING;
	const rc = snapshot.rc;
	const stamp = snapshot.stamps.rc;
	const ageMs = stamp.at === null ? null : Math.max(0, now - stamp.at);
	const stale = ageMs === null || ageMs > timing.vehicleStaleMs;
	const usOf = (channel: number | null): number | null => (channel === null || channel < 1 || channel > CHANNELS ? null : (rc?.channelsUs[channel - 1] ?? null));

	const map: MapRow[] = FUNCTIONS.map((spec): MapRow => {
		const row = rowOf(rows, spec.param);
		const present = row !== null && row.type === 'i32' && row.status !== 'manifest only';
		const active = present ? valueOf(row, row.activeBits) : null;
		const staged = present ? valueOf(row, row.stagedBits) : null;
		const thresholdRow = spec.thresholdParam ? rowOf(rows, spec.thresholdParam) : null;
		const threshold = thresholdRow && thresholdRow.status !== 'manifest only' ? valueOf(thresholdRow, thresholdRow.activeBits) : null;
		return {
			id: spec.id,
			label: spec.label,
			param: spec.param,
			present,
			active,
			staged,
			differs: active !== null && staged !== null && active !== staged,
			locked: row?.locked ?? false,
			lockText: row?.locked ? lockTextOf(row) : null,
			row: present ? row : null,
			threshold,
			liveUs: usOf(active)
		};
	});
	const mapKnown = FUNCTIONS.filter((f) => f.id !== 'bench').every((f) => map.find((m) => m.id === f.id)?.active !== null);
	const mapOf = (pick: (m: MapRow) => number | null): ChannelMapping => {
		const out: ChannelMapping = {};
		for (const m of map) {
			const v = pick(m);
			if (v !== null) out[m.id] = v;
		}
		return out;
	};
	const activeMap = mapOf((m) => m.active);
	const stagedMap = mapOf((m) => m.staged);

	const detectedChannels = detection.phase === 'idle' ? new Set<number>() : new Set(movedChannels(detection.track));
	const bars: Bar[] = Array.from({ length: CHANNELS }, (_, i): Bar => {
		const channel = i + 1;
		const us = rc?.channelsUs[i] ?? null;
		const x = us === null ? null : normalised(us);
		return {
			channel,
			us,
			x,
			bar: x === null ? null : (x + 1) / 2,
			roles: map.filter((m) => m.active === channel).map((m) => m.id),
			stagedRoles: map.filter((m) => m.staged === channel).map((m) => m.id),
			movedUs: rangeUs(activity, channel),
			active: rangeUs(activity, channel) >= MOVED_US,
			detected: detectedChannels.has(channel)
		};
	});

	const switches: SwitchRow[] = map
		.filter((m) => FN_BY_ID.get(m.id)?.kind === 'switch' && (m.id !== 'bench' || m.present))
		.map((m): SwitchRow => {
			const label = m.label;
			const none = m.active === 0;
			const us = m.liveUs;
			const x = us === null ? null : normalised(us);
			const thresholdUs = m.threshold === null ? null : Math.round(RC_US_CENTRE + m.threshold * RC_US_HALF_SPAN);
			let position: SwitchPosition = 'unknown';
			let text = `${label}: not known (${m.active === null || m.threshold === null ? 'read the parameters from the vehicle' : 'the channel is not reported'})`;
			let tone: Tone = 'neutral';
			if (none) {
				position = 'none';
				text = `${label}: no channel (0): the vehicle never selects it`;
			} else if (m.active !== null && m.threshold !== null && x !== null && us !== null) {
				position = x >= m.threshold ? 'at or above' : 'below';
				text = `${label} (channel ${m.active}): ${us} µs, x ${fmt(x, 3)}, ${position} the threshold ${fmt(m.threshold, 3)} (${thresholdUs} µs): the switch is ${position === 'at or above' ? 'ON' : 'OFF'} now`;
				tone = stale ? 'neutral' : position === 'at or above' ? 'warn' : 'ok';
			}
			return { id: m.id, label, channel: m.active, threshold: m.threshold, thresholdUs, x, us, position, text, tone };
		});

	const checks: Check[] = [];
	for (const id of ['roll', 'pitch', 'yaw'] as const) {
		const m = map.find((x) => x.id === id) as MapRow;
		const us = m.liveUs;
		const label = `${fnLabel(id)} stick centred`;
		if (m.active === null || us === null) {
			checks.push({ id: `centre.${id}`, label, status: 'unknown', detail: m.active === null ? 'the channel is not known (read the parameters)' : `channel ${m.active} is not reported` });
			continue;
		}
		const x = normalised(us);
		const ok = Math.abs(x) <= CENTRE_TOLERANCE;
		checks.push({
			id: `centre.${id}`,
			label,
			status: ok ? 'ok' : 'fail',
			detail: `channel ${m.active}: x ${fmt(x, 3)} (${us} µs); limit ±${CENTRE_TOLERANCE} (PLACEHOLDER)`
		});
	}
	{
		const m = map.find((x) => x.id === 'throttle') as MapRow;
		const limit = valueOf(rowOf(rows, ARM_THROTTLE_PARAM), rowOf(rows, ARM_THROTTLE_PARAM)?.activeBits ?? null);
		const label = 'Throttle low';
		if (m.active === null || m.liveUs === null) {
			checks.push({ id: 'throttle.low', label, status: 'unknown', detail: m.active === null ? 'the channel is not known (read the parameters)' : `channel ${m.active} is not reported` });
		} else if (limit === null) {
			checks.push({ id: 'throttle.low', label, status: 'unknown', detail: `${ARM_THROTTLE_PARAM} is not known (read the parameters)` });
		} else {
			const t = throttleOf(m.liveUs);
			const ok = t < limit;
			checks.push({
				id: 'throttle.low',
				label,
				status: ok ? 'ok' : 'fail',
				detail: `channel ${m.active}: throttle ${fmt(t, 3)} (${m.liveUs} µs); arming needs it below ${ARM_THROTTLE_PARAM} ${fmt(limit, 3)} (${Math.round(RC_US_THROTTLE_MIN + limit * RC_US_THROTTLE_SPAN)} µs)`
			});
		}
	}

	const locks: LockRow[] = rows
		.filter((r) => FUNCTIONS.some((f) => f.param === r.name || f.thresholdParam === r.name) || LOCK_LIST_NAMES.includes(r.name) || LOCK_LIST_PREFIXES.some((p) => r.name.startsWith(p)))
		.map((r): LockRow => ({ name: r.name, locked: r.locked, text: r.locked ? lockTextOf(r) : 'not locked' }));

	const mapped = new Set<number>([...Object.values(activeMap), ...Object.values(stagedMap)].filter((c) => c >= 1));
	const hints: string[] = [];
	if (mapKnown) {
		const silentSwitches = map.filter((m) => (m.id === 'arm' || m.id === 'angle') && m.active !== null && m.active >= 1 && rangeUs(activity, m.active) < MOVED_US);
		for (const channel of movedChannels(activity)) {
			if (mapped.has(channel) || silentSwitches.length === 0) continue;
			hints.push(
				`Channel ${channel} moved by ${rangeUs(activity, channel)} µs and no control uses it, while ${silentSwitches.map((m) => `the ${m.label.toLowerCase()} (channel ${m.active})`).join(' and ')} has not moved since the page was opened. If channel ${channel} is your switch, press Detect on that control and stage the channel.`
			);
		}
	}

	return {
		rc: { heard: stamp.at !== null, stale, ageMs, hz: stale ? null : stamp.hz, linkQuality: rc?.linkQuality ?? null, channelCount: rc?.channelCount ?? 0 },
		bars,
		map,
		mapKnown,
		findingsActive: findingsOf(activeMap),
		findingsStaged: findingsOf(stagedMap),
		switches,
		checks,
		locks,
		detection,
		hints
	};
}

export const percentOf = (quality: number | null): string => (quality === null ? DASH : `${Math.round(quality * PERCENT)} %`);

// ---- the controller ---------------------------------------------------------------------------------------------------------

export interface RadioDeps {
	telemetry: Pick<TelemetryClient, 'snapshot' | 'start' | 'stop' | 'maintain'>;
	/** The merged parameter rows (manifest and vehicle); empty until the parameters were read. */
	rows?: Readable<readonly ParamRow[]>;
	now?: () => number;
	timing?: ShellTiming;
}

/** Keeps the page true: asks for the streams while the page is open, follows the RC frames for the detection and the activity, and recomputes the view against the clock. */
export class RadioController {
	private readonly now: () => number;
	private readonly store = writable<RadioView | null>(null);
	private timer: ReturnType<typeof setInterval> | null = null;
	private unsubscribe: (() => void) | null = null;
	private rows: readonly ParamRow[] = [];
	private activity = emptyExcursions();
	private detection: Detection = IDLE;
	private seen = 0;
	readonly view: Readable<RadioView | null> = { subscribe: this.store.subscribe };

	constructor(private readonly deps: RadioDeps) {
		this.now = deps.now ?? (() => Date.now());
	}

	start(): void {
		if (this.timer) return;
		this.unsubscribe = (this.deps.rows ?? readable<readonly ParamRow[]>([])).subscribe((r) => {
			this.rows = r;
			this.recompute();
		});
		this.deps.telemetry.start();
		this.timer = setInterval(() => this.tick(), RADIO_TICK_MS);
		this.recompute();
	}

	stop(): void {
		if (!this.timer) return;
		clearInterval(this.timer);
		this.timer = null;
		this.unsubscribe?.();
		this.unsubscribe = null;
		this.deps.telemetry.stop();
	}

	tick(): void {
		this.deps.telemetry.maintain();
		this.recompute();
	}

	/** "Move the switch you want for <fn>". */
	detect(fn: FnId): void {
		this.detection = beginDetection(fn);
		this.recompute();
	}

	cancelDetection(): void {
		this.detection = IDLE;
		this.recompute();
	}

	/** Forget which channels have moved. */
	resetActivity(): void {
		this.activity = emptyExcursions();
		this.recompute();
	}

	private recompute(): void {
		const snapshot = this.deps.telemetry.snapshot();
		const count = snapshot.stamps.rc.count;
		if (count < this.seen) this.activity = emptyExcursions();
		if (count !== this.seen && snapshot.rc) {
			this.activity = observe(this.activity, snapshot.rc.channelsUs);
			this.detection = stepDetection(this.detection, snapshot.rc.channelsUs);
		}
		this.seen = count;
		this.store.set(computeRadio({ snapshot, rows: this.rows, now: this.now(), activity: this.activity, detection: this.detection, timing: this.deps.timing }));
	}
}
