<script lang="ts">
	import { onMount, untrack } from 'svelte';
	import ChannelBars from './ChannelBars.svelte';
	import FlightModes from './FlightModes.svelte';
	import {
		DEADBAND_MAX,
		STICKS,
		axisValue,
		calCentre,
		calCentrePhase,
		calFinish,
		calSample,
		calStart,
		normalizeStick,
		radioApi,
		sameConfig,
		validateConfig,
		type Calibration,
		type ProfileCfg,
		type RadioApi,
		type RadioConfig,
		type RadioDevice,
		type RadioLive,
		type StickName
	} from '$lib/gcs/radio';
	import type { ProfileDef } from '$lib/gcs/types';

	interface Props {
		profiles: ProfileDef[];
		/** The last radio message. */
		live: RadioLive | null;
		/** The last radio_devices message (hot-plug), null before one. */
		hotplug: RadioDevice[] | null;
		/** The backend's refusal of radio_subscribe. */
		error: string | null;
		wsOpen: boolean;
		visible: boolean;
		mock: string | null;
		onsubscribe: (device: string | null) => void;
	}
	let { profiles, live, hotplug, error, wsOpen, visible, mock, onsubscribe }: Props = $props();

	type Page = 'radio' | 'sticks' | 'modes';
	const PAGES: { id: Page; label: string }[] = [
		{ id: 'radio', label: 'Radio calibration' },
		{ id: 'sticks', label: 'Sticks & arming' },
		{ id: 'modes', label: 'Flight modes' }
	];
	/** Frames older than this read as stale. */
	const STALE_MS = 1000;

	let api = $state.raw<RadioApi | null>(null);
	let devices = $state.raw<RadioDevice[]>([]);
	let device = $state('');
	let saved = $state.raw<RadioConfig | null>(null);
	let source = $state<'stored' | 'default'>('default');
	let draft = $state<RadioConfig | null>(null);
	let apiError = $state<string | null>(null);
	let note = $state<string | null>(null);
	let busy = $state(false);
	let page = $state<Page>('radio');
	let cal = $state.raw<Calibration | null>(null);
	let calErrors = $state<string[]>([]);
	let lastAt = $state(0);
	let now = $state(0);
	let loadedOnce = false;

	const dev = $derived(devices.find((d) => d.name === device) ?? null);
	const cur = $derived(live && live.device === device && now - lastAt < STALE_MS ? live : null);
	const nAxes = $derived(dev?.axes || cur?.axes.length || 8);
	const nButtons = $derived(dev?.buttons ?? cur?.buttons.length ?? 0);
	const ids = $derived(profiles.map((p) => p.id));
	const dirty = $derived(draft !== null && !sameConfig(draft, saved));
	const problems = $derived(draft ? validateConfig(draft, ids, dev?.axes ?? 0, dev?.buttons ?? 0) : []);
	const profileLabel = (id: string | null): string => (id === null ? '—' : (profiles.find((p) => p.id === id)?.label ?? id));

	const axisRole = $derived.by(() => {
		const r: Record<number, string> = {};
		if (!draft) return r;
		for (const k of STICKS) r[draft.sticks[k].axis] = k;
		if (draft.arm.source === 'axis') r[draft.arm.index] = r[draft.arm.index] ? `${r[draft.arm.index]}+arm` : 'arm';
		if (draft.profile.source === 'axis') r[draft.profile.axis] = r[draft.profile.axis] ? `${r[draft.profile.axis]}+modes` : 'flight modes';
		return r;
	});
	const buttonRole = $derived.by(() => {
		const r: Record<number, string> = {};
		if (!draft) return r;
		if (draft.arm.source === 'button') r[draft.arm.button] = 'arm';
		if (draft.arm.disarm_button !== null) r[draft.arm.disarm_button] = 'disarm';
		if (draft.profile.source === 'buttons') for (const [k, id] of Object.entries(draft.profile.buttons)) r[Number(k)] = r[Number(k)] ? `${r[Number(k)]}+${profileLabel(id)}` : profileLabel(id);
		return r;
	});
	const marks = $derived.by(() => {
		const r: Record<number, { min: number; center: number; max: number }> = {};
		if (draft) for (const k of STICKS) r[draft.sticks[k].axis] = draft.sticks[k];
		return r;
	});

	onMount(() => {
		void radioApi(mock).then((a) => (api = a));
		const tick = setInterval(() => (now = performance.now()), 250);
		return () => clearInterval(tick);
	});

	// Frames: their time, and the calibration's samples.
	$effect(() => {
		const l = live;
		if (!l) return;
		untrack(() => {
			now = lastAt = performance.now();
			if (cal && l.device === device) cal = calSample(cal, l.axes);
		});
	});

	$effect(() => {
		const list = hotplug;
		if (!list) return;
		untrack(() => {
			devices = list;
			if (!device && list.length && api) void select(list[0].name);
		});
	});

	$effect(() => {
		if (visible && api && !loadedOnce) {
			loadedOnce = true;
			void untrack(() => refreshDevices());
		}
	});

	// The stream, while this tab shows a device.
	$effect(() => {
		if (!visible || !wsOpen || !device) return;
		const d = device;
		untrack(() => onsubscribe(d));
		return () => untrack(() => onsubscribe(null));
	});

	async function guarded(what: () => Promise<void>): Promise<void> {
		apiError = null;
		busy = true;
		try {
			await what();
		} catch (e) {
			apiError = e instanceof Error ? e.message : String(e);
		} finally {
			busy = false;
		}
	}

	async function refreshDevices(): Promise<void> {
		await guarded(async () => {
			if (!api) return;
			devices = await api.devices();
			if (!device && devices.length) await select(devices[0].name);
		});
	}

	async function load(name: string): Promise<void> {
		if (!api) return;
		const r = await api.config(name);
		saved = r.config;
		source = r.source;
		draft = structuredClone(r.config);
	}

	async function select(name: string): Promise<void> {
		if (dirty && !confirm(`Discard the unsaved changes to ${device}?`)) return;
		device = name;
		cal = null;
		calErrors = [];
		note = null;
		await guarded(() => load(name));
	}

	function save(): void {
		if (!draft || problems.length) return;
		const d = $state.snapshot(draft) as RadioConfig;
		void guarded(async () => {
			if (!api) return;
			saved = await api.save(d);
			source = 'stored';
			draft = structuredClone(saved);
			devices = devices.map((x) => (x.name === d.device_name ? { ...x, has_config: true } : x));
			note = 'Saved: the backend maps this device with it now.';
		});
	}

	function revert(): void {
		note = null;
		void guarded(() => load(device));
	}

	function resetDefault(): void {
		if (!confirm(`Delete the stored mapping of ${device} and go back to the default?`)) return;
		note = null;
		void guarded(async () => {
			if (!api) return;
			await api.reset(device);
			await load(device);
			devices = devices.map((x) => (x.name === device ? { ...x, has_config: false } : x));
			note = 'Stored mapping deleted: the default applies.';
		});
	}

	function setStick<K extends keyof RadioConfig['sticks'][StickName]>(k: StickName, key: K, v: RadioConfig['sticks'][StickName][K]): void {
		if (draft) draft.sticks[k][key] = v;
	}

	function calibrate(): void {
		calErrors = [];
		cal = calStart(nAxes);
	}

	function calDone(): void {
		if (!cal || !draft) return;
		const r = calFinish(cal, $state.snapshot(draft) as RadioConfig);
		draft = r.config;
		calErrors = r.errors;
		cal = null;
		note = r.errors.length ? null : 'Calibration captured: review the sticks, then Save.';
	}

	const numIn = (e: Event): number => Number((e.currentTarget as HTMLInputElement).value);
	const f2 = (v: number | undefined): string => (v === undefined || !Number.isFinite(v) ? '—' : v.toFixed(2));
</script>

<section class="ctl mono" aria-label="Controller">
	<div class="head">
		<label>
			Device
			<select value={device} disabled={busy} onchange={(e) => void select(e.currentTarget.value)}>
				{#if !device}<option value="" disabled>{devices.length ? 'choose…' : 'none plugged in'}</option>{/if}
				{#if device && !dev}<option value={device}>{device} (unplugged)</option>{/if}
				{#each devices as d (d.name)}<option value={d.name}>{d.name} — {d.axes} axes, {d.buttons} buttons{d.has_config ? '' : ' (default)'}</option>{/each}
			</select>
		</label>
		<button type="button" class="btn sm" disabled={busy || !api} onclick={() => void refreshDevices()}>Rescan</button>
		{#if dev}<span class="k">{dev.path}</span>{/if}
		{#if draft}<span class="tag" class:stored={source === 'stored'}>{source === 'stored' ? 'stored mapping' : 'default mapping'}</span>{/if}
		{#if dirty}<span class="unsaved">● unsaved changes</span>{/if}
		<span class="spacer"></span>
		<button type="button" class="btn" disabled={!draft || !dirty || problems.length > 0 || busy} onclick={save} title={problems.length ? 'Fix the problems listed first' : 'PUT /api/radio/config'}>Save</button>
		<button type="button" class="btn" disabled={!draft || !dirty || busy} onclick={revert}>Revert</button>
		<button type="button" class="btn" disabled={!device || busy} onclick={resetDefault}>Reset to default</button>
	</div>
	{#if apiError}<p class="err" role="alert">{apiError}</p>{/if}
	{#if error}<p class="err" role="alert">{error}</p>{/if}
	{#if note}<p class="hint" role="status">{note}</p>{/if}
	{#if problems.length}
		<ul class="problems" role="alert">
			{#each problems as p (p)}<li>{p}</li>{/each}
		</ul>
	{/if}

	<div class="preview">
		<span class="k">live{dirty ? ' (saved mapping)' : ''}:</span>
		{#if cur}
			<span>roll {f2(cur.normalized.roll)}</span>
			<span>pitch {f2(cur.normalized.pitch)}</span>
			<span>throttle {f2(cur.normalized.throttle)}</span>
			<span>yaw {f2(cur.normalized.yaw)}</span>
			<span class:armed={cur.arm}>{cur.arm ? 'ARM' : 'disarm'}</span>
			<span>profile <b>{profileLabel(cur.profile)}</b></span>
		{:else}
			<span class="k">{!wsOpen ? 'backend not connected' : !device ? 'no device chosen' : 'no frames from the device'}</span>
		{/if}
	</div>

	{#if draft}
		<div class="layout">
			<nav class="side" aria-label="Controller pages">
				{#each PAGES as p (p.id)}
					<button type="button" class:on={page === p.id} onclick={() => (page = p.id)}>{p.label}</button>
				{/each}
			</nav>

			<div class="page">
				{#if page === 'radio'}
					<div class="line">
						<h2>Radio calibration</h2>
						{#if !cal}
							<button type="button" class="btn" disabled={!cur} onclick={calibrate} title={cur ? '' : 'Needs live frames from the device'}>Calibrate</button>
						{:else}
							<button type="button" class="btn sm" onclick={() => (cal = null)}>Cancel</button>
						{/if}
					</div>
					{#if cal?.phase === 'extremes'}
						<div class="step">
							<b>1.</b> Move all sticks and switches to their extremes, several times. The shaded range grows as it captures.
							<button type="button" class="btn sm" onclick={() => (cal = cal && calCentrePhase(cal))}>Next: centre</button>
						</div>
					{:else if cal?.phase === 'centre'}
						<div class="step">
							<b>2.</b> Centre the sticks (throttle at mid-stick: the centre is hold height), leave them still, then Done.
							<span class="k">{STICKS.map((k) => `${k} ${cal ? (calCentre(cal, draft!.sticks[k].axis) ?? '—') : '—'}`).join(', ')}</span>
							<button type="button" class="btn sm" onclick={calDone}>Done</button>
						</div>
					{/if}
					{#if calErrors.length}
						<ul class="problems" role="alert">{#each calErrors as p (p)}<li>{p}</li>{/each}</ul>
					{/if}
					<ChannelBars
						axes={nAxes}
						buttons={nButtons}
						raw={cur?.axes ?? null}
						pressed={cur?.buttons ?? null}
						{axisRole}
						{buttonRole}
						{marks}
						bandAxis={draft.profile.source === 'axis' ? draft.profile.axis : null}
						bands={draft.profile.bands}
						{cal}
					/>
					<p class="hint">Bars: raw value (−32768…32767) from the centre line; black ticks: the mapping's min, center, max; orange: flight-mode band edges.</p>
				{:else if page === 'sticks'}
					<h2>Sticks</h2>
					<table class="specs">
						<thead><tr><th>stick</th><th>axis</th><th>min</th><th>center</th><th>max</th><th>reverse</th><th>deadband</th><th>raw</th><th>out</th></tr></thead>
						<tbody>
							{#each STICKS as k (k)}
								{@const s = draft.sticks[k]}
								{@const r = cur?.axes[s.axis]}
								{@const out = r === undefined ? NaN : normalizeStick(r, s)}
								<tr>
									<td><b>{k}</b></td>
									<td>
										<select value={s.axis} onchange={(e) => setStick(k, 'axis', Number(e.currentTarget.value))}>
											{#each Array.from({ length: Math.max(nAxes, s.axis + 1) }, (_, i) => i) as i (i)}<option value={i}>CH{i + 1} (a{i})</option>{/each}
										</select>
									</td>
									<td><input type="number" step="1" value={s.min} onchange={(e) => setStick(k, 'min', numIn(e))} /></td>
									<td><input type="number" step="1" value={s.center} onchange={(e) => setStick(k, 'center', numIn(e))} /></td>
									<td><input type="number" step="1" value={s.max} onchange={(e) => setStick(k, 'max', numIn(e))} /></td>
									<td><input type="checkbox" checked={s.reverse} onchange={(e) => setStick(k, 'reverse', e.currentTarget.checked)} /></td>
									<td><input type="number" class="short" min="0" max={DEADBAND_MAX} step="0.01" value={s.deadband} onchange={(e) => setStick(k, 'deadband', numIn(e))} /></td>
									<td class="k">{r ?? '—'}</td>
									<td>
										<span class="mini"><span class="mini-fill" style:left="{Math.min(50, 50 + 50 * (Number.isFinite(out) ? out : 0))}%" style:width="{50 * Math.abs(Number.isFinite(out) ? out : 0)}%"></span></span>
										{f2(out)}
									</td>
								</tr>
							{/each}
						</tbody>
					</table>
					<p class="hint">
						Out: this mapping applied to the live value (deadband is the fraction of each half-travel that reads 0).
						{draft.throttle_centre_hold ? 'Throttle centre holds height; up climbs, down descends.' : ''}
					</p>

					<h2>Arming</h2>
					<div class="line">
						<label>
							Arm with
							<select value={draft.arm.source} onchange={(e) => draft && (draft.arm.source = e.currentTarget.value === 'button' ? 'button' : 'axis')}>
								<option value="axis">a switch channel</option>
								<option value="button">a button</option>
							</select>
						</label>
						{#if draft.arm.source === 'axis'}
							<label>
								Channel
								<select value={draft.arm.index} onchange={(e) => draft && (draft.arm.index = Number(e.currentTarget.value))}>
									{#each Array.from({ length: Math.max(nAxes, draft.arm.index + 1) }, (_, i) => i) as i (i)}<option value={i}>CH{i + 1} (a{i}){axisRole[i] && axisRole[i] !== 'arm' ? ` — ${axisRole[i]}` : ''}</option>{/each}
								</select>
							</label>
							<label>
								on above
								<input type="number" class="short" min="-0.99" max="0.99" step="0.05" value={draft.arm.on_above} onchange={(e) => draft && (draft.arm.on_above = numIn(e))} />
							</label>
							{@const sw = cur?.axes[draft.arm.index]}
							<span class="k">switch now: {sw === undefined ? '—' : `${f2(axisValue(sw))} → ${axisValue(sw) > draft.arm.on_above ? 'ON' : 'off'}`}</span>
						{:else}
							<label>
								Button
								<select value={draft.arm.button} onchange={(e) => draft && (draft.arm.button = Number(e.currentTarget.value))}>
									{#each Array.from({ length: Math.max(nButtons, draft.arm.button + 1) }, (_, i) => i) as i (i)}<option value={i}>{i}</option>{/each}
								</select>
							</label>
						{/if}
						<label>
							Disarm button
							<select value={draft.arm.disarm_button ?? -1} onchange={(e) => draft && (draft.arm.disarm_button = Number(e.currentTarget.value) < 0 ? null : Number(e.currentTarget.value))}>
								<option value={-1}>none</option>
								{#each Array.from({ length: Math.max(nButtons, (draft.arm.disarm_button ?? -1) + 1) }, (_, i) => i) as i (i)}<option value={i}>{i}</option>{/each}
							</select>
						</label>
						<label>
							<input type="checkbox" checked={draft.arm.require_throttle_low} onchange={(e) => draft && (draft.arm.require_throttle_low = e.currentTarget.checked)} />
							arm only with throttle low
						</label>
						<span class:armed={cur?.arm}>{cur ? (cur.arm ? 'ARM' : 'disarm') : ''}</span>
					</div>
				{:else}
					<h2>Flight modes</h2>
					<FlightModes
						cfg={draft.profile}
						{profiles}
						axes={nAxes}
						buttons={nButtons}
						raw={cur?.axes ?? null}
						pressed={cur?.buttons ?? null}
						{axisRole}
						serverProfile={cur?.profile ?? null}
						onchange={(p: ProfileCfg) => draft && (draft.profile = p)}
					/>
				{/if}
			</div>
		</div>
	{:else if !busy}
		<p class="hint">{devices.length ? 'Choose a device.' : 'No joystick or radio is plugged into the ground station. Plug one in (it appears here when it does) or Rescan.'}</p>
	{/if}
</section>

<style>
	.ctl {
		display: flex;
		flex-direction: column;
		gap: 0.6rem;
		margin-top: 0.75rem;
		font-size: 0.8rem;
		min-width: 0;
	}
	.ctl h2 {
		margin: 0.4rem 0 0.2rem;
	}
	.head,
	.line {
		display: flex;
		flex-wrap: wrap;
		align-items: center;
		gap: 0.5rem 1rem;
	}
	.spacer {
		flex: 1;
	}
	.tag {
		border: 1px solid var(--border);
		border-radius: 999px;
		padding: 0.05rem 0.55rem;
		color: var(--muted);
	}
	.tag.stored {
		color: var(--ok);
		border-color: var(--ok);
	}
	.unsaved {
		color: var(--warn);
	}
	.preview {
		display: flex;
		flex-wrap: wrap;
		gap: 0.3rem 1.2rem;
		padding: 0.4rem 0.6rem;
		border: 1px solid var(--border);
		border-radius: 8px;
		background: var(--surface);
		font-variant-numeric: tabular-nums;
	}
	.armed {
		color: var(--bad);
		font-weight: 700;
	}
	.layout {
		display: grid;
		grid-template-columns: 11rem 1fr;
		gap: 1rem;
		align-items: start;
	}
	.side {
		display: flex;
		flex-direction: column;
		border: 1px solid var(--border);
		border-radius: 8px;
		overflow: hidden;
		background: var(--surface);
	}
	.side button {
		background: none;
		border: none;
		border-bottom: 1px solid var(--border);
		color: var(--muted);
		text-align: left;
		padding: 0.5rem 0.7rem;
		font: inherit;
		cursor: pointer;
	}
	.side button:last-child {
		border-bottom: none;
	}
	.side button.on {
		color: var(--text);
		background: var(--accent-bg);
		box-shadow: inset 3px 0 0 var(--accent);
	}
	.page {
		display: flex;
		flex-direction: column;
		gap: 0.5rem;
		min-width: 0;
		padding: 0.6rem 0.8rem;
		border: 1px solid var(--border);
		border-radius: 8px;
		background: var(--surface);
	}
	.step {
		display: flex;
		flex-wrap: wrap;
		align-items: center;
		gap: 0.4rem 0.8rem;
		padding: 0.4rem 0.6rem;
		border: 1px solid var(--accent);
		border-radius: 6px;
		background: var(--accent-bg);
	}
	.problems {
		margin: 0;
		padding-left: 1.2rem;
		color: var(--bad);
	}
	.mini {
		position: relative;
		display: inline-block;
		width: 4rem;
		height: 0.6rem;
		vertical-align: middle;
		border: 1px solid var(--border);
		border-radius: 3px;
		background: var(--surface-2);
		overflow: hidden;
	}
	.mini-fill {
		position: absolute;
		top: 0;
		bottom: 0;
		background: var(--ok);
	}
	.ctl :global(.k) {
		color: var(--muted);
	}
	.ctl :global(.hint) {
		margin: 0;
		color: var(--muted);
		max-width: none;
	}
	.ctl :global(.err) {
		margin: 0;
		color: var(--bad);
		max-width: none;
	}
	.ctl :global(select),
	.ctl :global(input[type='number']) {
		background: var(--surface-2);
		color: var(--text);
		border: 1px solid var(--border);
		border-radius: 6px;
		padding: 0.15rem 0.35rem;
		font: inherit;
		margin-left: 0.3rem;
	}
	.ctl :global(input[type='number']) {
		width: 6rem;
	}
	.ctl :global(input.short) {
		width: 4.5rem;
	}
	.ctl :global(.specs) {
		width: 100%;
		border-collapse: collapse;
		font-size: 0.75rem;
		font-variant-numeric: tabular-nums;
		white-space: nowrap;
	}
	.ctl :global(.specs th),
	.ctl :global(.specs td) {
		padding: 0.2rem 0.45rem;
		border-bottom: 1px solid var(--border);
		text-align: left;
	}
	.ctl :global(.specs th) {
		color: var(--muted);
		font-weight: 600;
	}
	.ctl :global(.btn) {
		background: var(--surface-2);
		color: var(--text);
		border: 1px solid var(--border);
		border-radius: 6px;
		padding: 0.35rem 0.7rem;
		font: inherit;
		cursor: pointer;
	}
	.ctl :global(.btn:hover) {
		border-color: var(--border-strong);
	}
	.ctl :global(.btn:disabled) {
		opacity: 0.5;
		cursor: default;
	}
	.ctl :global(.btn.sm) {
		padding: 0.15rem 0.45rem;
	}
</style>
