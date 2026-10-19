<script lang="ts">
	import { onMount } from 'svelte';
	import { readable } from 'svelte/store';
	import { app } from '$lib/state/app';
	import { whyCannotRead } from '$lib/state/gating';
	import {
		candidateFindings, fnLabel, MOVED_US, percentOf, type ChannelMapping, type FnId, type MapRow, type RadioView
	} from '$lib/state/radio_state';
	import { formatHz, formatStreamAge } from '$lib/state/live_state';
	import type { ValidationState } from '$lib/state/validation_state';
	import type { SessionState } from '$link/session';
	import type { VehicleState } from '$link/vehicle';
	import { whyCannotBuildView } from '$lib/params/validation';
	import { checkStage, type StageContext } from '$lib/params/staging';
	import type { ParamRow } from '$lib/params/merge';
	import StagedPanel from '$lib/components/StagedPanel.svelte';
	import Badge from '$lib/components/ui/Badge.svelte';
	import Panel from '$lib/components/ui/Panel.svelte';

	const gcs = app();
	const radio = gcs.radio;
	const view = radio?.view ?? readable<RadioView | null>(null);
	const manifest = gcs.manifest;
	const vehicle = gcs.vehicle?.state ?? readable<VehicleState | null>(null);
	const session = gcs.session?.state ?? readable<SessionState | null>(null);
	const helperState = gcs.helper?.state ?? readable('closed');
	const device = gcs.device;
	const paramRows = gcs.paramRows;
	const validation = gcs.validation;
	const validationState = validation?.state ?? readable<ValidationState | null>(null);
	const fingerprint = validation?.fingerprint ?? readable('');

	// The streams are asked for while the page is open and stopped when it is left (and when the tab is closed, best effort).
	onMount(() => {
		void gcs.ensureImages();
		radio?.start();
		const stop = (): void => radio?.stop();
		window.addEventListener('pagehide', stop);
		return () => {
			window.removeEventListener('pagehide', stop);
			radio?.stop();
		};
	});

	let source = $state('');
	let typed = $state<Partial<Record<FnId, string>>>({});
	let outcome = $state<Partial<Record<FnId, { ok: boolean; text: string }>>>({});
	let busy = $state<FnId | null>(null);
	let autoRead = false;

	const ctx = $derived<StageContext>({
		connected: $session?.heartbeat === 'alive',
		identityKnown: $vehicle?.identity != null,
		manifestMatches: Boolean($vehicle?.identity && $manifest.loaded && $manifest.loaded.manifest.schemaHash === $vehicle.identity.schemaHash),
		bytewise: $vehicle?.bytewise ?? null,
		armed: $session?.armed ?? null
	});
	const readBlocked = $derived(whyCannotRead({ helper: $helperState, device: $device, heartbeat: $session?.heartbeat, listing: $vehicle?.listing }));

	// The parameters are read once when the page finds a live vehicle without them: reading changes nothing on the vehicle.
	$effect(() => {
		if (!autoRead && readBlocked === null && ($vehicle?.params.length ?? 0) === 0 && $vehicle?.listing === 'idle' && $vehicle.identity) {
			autoRead = true;
			void gcs.loadParams();
		}
	});

	/** The channel number typed for a control, or the staged one. */
	function chosen(m: MapRow): string {
		return typed[m.id] ?? (m.staged === null ? '' : String(m.staged));
	}

	/** The channel map the next stage would produce, for the R-1 warning: the staged config. */
	function stagedMap(v: RadioView): ChannelMapping {
		const out: ChannelMapping = {};
		for (const m of v.map) if (m.staged !== null) out[m.id] = m.staged;
		return out;
	}

	function warningsFor(v: RadioView, m: MapRow): string[] {
		const text = chosen(m).trim();
		if (!/^-?\d+$/.test(text)) return text === '' ? [] : ['not a whole number'];
		return candidateFindings(stagedMap(v), m.id, Number(text)).map((f) => f.text);
	}

	function whyCannotStage(m: MapRow): string | null {
		if (!m.row) return m.present ? 'the vehicle has not reported this parameter' : 'this image does not have this parameter';
		const check = checkStage(m.row, ctx, chosen(m).trim(), source, false);
		return check.ok ? null : check.error;
	}

	async function stage(m: MapRow): Promise<void> {
		if (!gcs.vehicle || !m.row) return;
		const check = checkStage(m.row, ctx, chosen(m).trim(), source, false);
		if (!check.ok) return;
		busy = m.id;
		try {
			const result = await gcs.vehicle.stage(m.row.linkId, m.row.type, check.request.bits, check.request.source);
			outcome[m.id] = result.accepted
				? { ok: true, text: `staged ${chosen(m).trim()}${result.sourceStored === false ? ' (the source text did not read back)' : ''}` }
				: { ok: false, text: `refused by the vehicle: ${result.reasons.join('; ') || result.note || 'no reason sent'}` };
			if (result.accepted) delete typed[m.id];
		} catch (e) {
			outcome[m.id] = { ok: false, text: e instanceof Error ? e.message : String(e) };
		} finally {
			busy = null;
		}
	}

	const staged = $derived($paramRows.filter((r: ParamRow) => r.differs));
	const whyValidate = $derived.by((): string | null => {
		if (!validation) return 'no helper';
		if (!ctx.connected) return 'no heartbeat from the vehicle';
		if (!ctx.identityKnown) return 'the vehicle has not reported its identity';
		if (!ctx.manifestMatches || !$manifest.loaded) return 'no manifest of the vehicle\'s parameter schema is on screen';
		if ($vehicle?.listing !== 'complete') return 'read the parameters from the vehicle first';
		return whyCannotBuildView($manifest.loaded.manifest, $vehicle?.params ?? [], false);
	});
	const whyActivate = $derived(
		validation && $vehicle && $validationState ? validation.whyCannotActivate($vehicle, $validationState, $fingerprint) : 'no validation report: validate the staged config first'
	);
	const whyCommands = $derived.by((): string | null => {
		if (!ctx.connected) return 'no heartbeat from the vehicle';
		if (!ctx.identityKnown) return 'the vehicle has not reported its identity';
		if (ctx.armed !== false) return ctx.armed ? 'the vehicle is armed: config commands are accepted only while disarmed' : 'the arming state is not known';
		return null;
	});
	const idleValidation: ValidationState = { phase: 'idle', progress: [], record: null, error: null, computed: null, command: null, busy: false };

	async function reloadAfter(action: 'revert' | 'reset-factory'): Promise<void> {
		const result = await validation?.command(action);
		if (result === 'accepted') await gcs.loadParams();
	}

	const us = (v: number | null): string => (v === null ? '—' : String(v));
	const x3 = (v: number | null): string => (v === null ? '—' : v.toFixed(3));
	const label = (id: FnId): string => fnLabel(id);
</script>

<svelte:head><title>Radio · MARV GCS</title></svelte:head>

<h1>Radio</h1>
<p class="muted lead">
	The receiver's 16 channels as the vehicle reports them, the channel each control reads from the vehicle's own parameters, and the
	switch thresholds against the live channel. A remap is staged here but goes through the same validation and activation as every
	parameter; the vehicle's own check is the authority and this page only warns.
</p>

{#if !radio}
	<p class="muted" role="status">No helper: there is no link to read.</p>
{:else if $view}
	<p class="muted small" role="status" data-summary>
		{#if !$view.rc.heard}no RC frame has been heard from the vehicle
		{:else if $view.rc.stale}<span class="warn">stale</span>: no RC frame for {formatStreamAge($view.rc.ageMs)}; the values shown are the last ones
		{:else}live · {formatHz($view.rc.hz)} · link quality {percentOf($view.rc.linkQuality)} · {$view.rc.channelCount} channels{/if}
		{#if $view.rc.heard && $view.rc.channelCount === 0}· the vehicle reports no channels{/if}
	</p>

	{#each $view.hints as h (h)}<p class="hint" role="alert" data-hint>{h}</p>{/each}
	{#if !$view.mapKnown}
		<div class="buttons">
			<button type="button" disabled={readBlocked !== null} title={readBlocked ?? 'Request the parameter list from the vehicle'} onclick={() => void gcs.loadParams()}>
				{$vehicle?.listing === 'loading' ? 'Reading…' : 'Read parameters from the vehicle'}
			</button>
			<span class="muted small">{readBlocked ?? 'the mapping, the thresholds and the checks come from the vehicle\'s parameters'}</span>
		</div>
	{/if}

	<div class="grid">
		<div class="wide">
			<Panel title="Channels">
				<div class="state" data-stale={$view.rc.stale}>
					{#if $view.rc.stale}<span class="stale">stale</span>{:else}<Badge text="live" tone="ok" />{/if}
					<span class="muted small">a channel is highlighted once it moved at least {MOVED_US} µs since the page was opened</span>
					<button type="button" class="link" onclick={() => radio.resetActivity()}>forget what moved</button>
				</div>
				<table class="bars mono" aria-label="Channels">
					<thead>
						<tr><th scope="col">Ch</th><th scope="col">Used by</th><th scope="col" class="r">µs</th><th scope="col" class="r">x</th><th scope="col">Position</th><th scope="col" class="r">moved</th></tr>
					</thead>
					<tbody>
						{#each $view.bars as b (b.channel)}
							<tr data-channel={b.channel} data-used={b.roles.length > 0} data-detected={b.detected} data-moved={b.active} class:detected={b.detected} class:moved={b.active && !b.detected}>
								<th scope="row">{b.channel}</th>
								<td>
									{#each b.roles as r (r)}<span class="role">{label(r)}</span>{/each}
									{#each b.stagedRoles.filter((r) => !b.roles.includes(r)) as r (r)}<span class="role staged" title="after the staged config is activated">→ {label(r)}</span>{/each}
								</td>
								<td class="r" data-cell="us">{us(b.us)}</td>
								<td class="r" data-cell="x">{x3(b.x)}</td>
								<td>
									<span class="bar centred" aria-hidden="true">
										{#if b.bar !== null}<span class="fill" style={`left:${Math.min(b.bar, 0.5) * 100}%;width:${Math.abs(b.bar - 0.5) * 100}%`}></span>{/if}
									</span>
								</td>
								<td class="r muted">{b.movedUs > 0 ? `${b.movedUs} µs` : ''}</td>
							</tr>
						{/each}
					</tbody>
				</table>
			</Panel>
		</div>

		<div class="wide">
			<Panel title="Mapping">
				<p class="muted small">
					The channel each control reads, on the running config and in the staged config. To change one: type a channel (or press Detect and move
					the switch), give the source text, press Stage; then validate and activate below. A channel shared by two controls is refused by the vehicle.
				</p>
				<div class="src">
					<label for="radio-source">Source text</label>
					<input id="radio-source" type="text" bind:value={source} placeholder="where this value comes from, e.g. who asked for it and why" />
				</div>
				{#if $view.detection.phase !== 'idle'}
					{@const d = $view.detection}
					<div class="detect" role="status" data-detection={d.phase}>
						{#if d.phase === 'listening'}
							Move the switch you want for <strong>{label(d.fn)}</strong>…
						{:else if d.phase === 'found'}
							Channel <strong data-found-channel>{d.channel}</strong> moved ({d.moved} µs) for <strong>{label(d.fn)}</strong>.
							<button type="button" onclick={() => { typed[d.fn] = String(d.channel); radio.cancelDetection(); }}>Use channel {d.channel} for {label(d.fn)}</button>
							<button type="button" onclick={() => radio.detect(d.fn)}>Detect again</button>
						{:else}
							Channels {d.channels.join(' and ')} moved at once: move only the one you want.
							<button type="button" onclick={() => radio.detect(d.fn)}>Detect again</button>
						{/if}
						<button type="button" onclick={() => radio.cancelDetection()}>Cancel</button>
					</div>
				{/if}
				<table class="map" aria-label="Mapping">
					<thead>
						<tr>
							<th scope="col">Control</th><th scope="col" class="r">Running</th><th scope="col" class="r">Staged</th><th scope="col" class="r">Live µs</th>
							<th scope="col">Lock</th><th scope="col">New channel</th><th scope="col"></th>
						</tr>
					</thead>
					<tbody>
						{#each $view.map as m (m.id)}
							{@const warnings = warningsFor($view, m)}
							{@const why = whyCannotStage(m)}
							{#if m.id !== 'bench' || m.present}
								<tr data-fn={m.id}>
									<th scope="row">
										{m.label}
										<span class="muted mono small">{m.param}</span>
										{#if m.id === 'bench'}<span class="muted small">(bench image only)</span>{/if}
									</th>
									<td class="r mono" data-cell="active">{m.active === null ? '—' : m.active === 0 ? '0 (none)' : m.active}</td>
									<td class="r mono" data-cell="staged" class:diff={m.differs}>{m.staged === null ? '—' : m.staged}</td>
									<td class="r mono">{m.active === 0 ? '' : us(m.liveUs)}</td>
									<td>{#if m.locked}<Badge text="locked" tone="warn" />{/if}</td>
									<td>
										<input
											class="num mono"
											type="text"
											inputmode="numeric"
											aria-label={`${m.label} channel`}
											disabled={!m.present}
											value={chosen(m)}
											oninput={(e) => { typed[m.id] = e.currentTarget.value; }}
										/>
										<button type="button" aria-label={`Detect ${m.label}`} disabled={!m.present} onclick={() => radio.detect(m.id)}>Detect</button>
									</td>
									<td>
										<button type="button" class="primary" aria-label={`Stage ${m.label}`} disabled={why !== null || busy !== null} title={why ?? 'Stage this channel on the vehicle'} onclick={() => void stage(m)}>Stage</button>
									</td>
								</tr>
								{#if warnings.length > 0 || outcome[m.id] || (why !== null && chosen(m).trim() !== '' && typed[m.id] !== undefined)}
									<tr class="note" data-fn-note={m.id}>
										<td colspan="7">
											{#each warnings as w (w)}<span class="warn" data-r1>⚠ {w}</span>{/each}
											{#if outcome[m.id]}<span class={outcome[m.id]?.ok ? 'ok' : 'bad'} role="status">{outcome[m.id]?.text}</span>{/if}
											{#if why !== null && typed[m.id] !== undefined}<span class="muted small">cannot stage: {why}</span>{/if}
										</td>
									</tr>
								{/if}
							{/if}
						{/each}
					</tbody>
				</table>
				{#if $view.findingsActive.length > 0}
					{#each $view.findingsActive as f (f.text)}<p class="bad" role="alert">The running config: {f.text}</p>{/each}
				{/if}
				{#if $view.findingsStaged.length > 0}
					{#each $view.findingsStaged as f (f.text)}<p class="warn" role="alert" data-r1-staged>The staged config: {f.text}</p>{/each}
				{/if}
				{#each $view.map.filter((m) => m.locked && m.present) as m (m.id)}
					<p class="muted small">{m.label} ({m.param}) is {m.lockText}.</p>
				{/each}
			</Panel>
		</div>

		<Panel title="Switch thresholds">
			<p class="muted small">Each switch is on at or above its threshold (the arming rule). The position is from the channel's live value now.</p>
			{#each $view.switches as s (s.id)}
				<div class="switch" data-switch={s.id} data-position={s.position}>
					<span class="name">{s.label}</span>
					<span class="mono tone-{s.tone}">{s.text}</span>
					{#if s.x !== null && s.threshold !== null}
						<span class="gauge" aria-hidden="true">
							<span class="mark" style={`left:${((s.threshold + 1) / 2) * 100}%`}></span>
							<span class="dot" class:on={s.position === 'at or above'} style={`left:${((s.x + 1) / 2) * 100}%`}></span>
						</span>
					{/if}
				</div>
			{/each}
			{#if $view.switches.length === 0}<p class="muted">The switches are not known: read the parameters from the vehicle.</p>{/if}
		</Panel>

		<Panel title="Display checks">
			<p class="muted small">
				A check reads the channels right now: release the sticks and lower the throttle. The centre tolerance is a PLACEHOLDER of this
				display (no flight effect); the throttle limit is the vehicle's own arming limit.
			</p>
			<ul class="checks">
				{#each $view.checks as c (c.id)}
					<li data-check={c.id} data-status={c.status}>
						<Badge text={c.status === 'ok' ? 'ok' : c.status === 'fail' ? 'not ok' : 'unknown'} tone={c.status === 'ok' ? 'ok' : c.status === 'fail' ? 'bad' : 'neutral'} />
						<strong>{c.label}</strong> <span class="mono small muted">{c.detail}</span>
					</li>
				{/each}
			</ul>
		</Panel>

		<Panel title="Locked radio parameters">
			<p class="muted small">Read only. A lock is compiled into the image: it changes only by a firmware build, never from here.</p>
			{#if $view.locks.length === 0}
				<p class="muted">Not known: read the parameters from the vehicle.</p>
			{:else}
				<ul class="locks mono">
					{#each $view.locks as l (l.name)}
						<li data-lock={l.name} data-locked={l.locked}>
							<span class="lname">{l.name}</span>
							{#if l.locked}<Badge text="locked" tone="warn" />{/if}
							<span class="muted small">{l.text}</span>
						</li>
					{/each}
				</ul>
			{/if}
		</Panel>

		<div class="wide">
			<StagedPanel
				rows={staged}
				validation={$validationState ?? idleValidation}
				{whyValidate}
				{whyActivate}
				{whyCommands}
				onvalidate={() => void validation?.validate()}
				onactivate={() => void validation?.activate()}
				onrevert={() => void reloadAfter('revert')}
				onreset={() => void reloadAfter('reset-factory')}
			/>
		</div>
	</div>
{/if}

<style>
	.lead {
		margin: 0.2rem 0 0.75rem;
	}

	.small {
		font-size: 0.8rem;
	}

	.grid {
		display: grid;
		grid-template-columns: repeat(auto-fit, minmax(28rem, 1fr));
		gap: 0.75rem;
		align-items: start;
	}

	.wide {
		grid-column: 1 / -1;
	}

	.buttons,
	.state,
	.src,
	.detect {
		display: flex;
		flex-wrap: wrap;
		gap: 0.5rem;
		align-items: center;
		margin: 0.5rem 0;
	}

	.src input {
		flex: 1;
		min-width: 18rem;
	}

	.detect {
		padding: 0.4rem 0.6rem;
		border: 1px solid var(--accent);
		background: var(--accent-bg);
		border-radius: 4px;
	}

	.hint {
		margin: 0.4rem 0;
		padding: 0.4rem 0.6rem;
		border: 1px solid var(--warn);
		border-radius: 4px;
		color: var(--warn);
	}

	.stale {
		color: var(--warn);
		font-size: 0.65rem;
		text-transform: uppercase;
		letter-spacing: 0.06em;
		border: 1px solid var(--warn);
		border-radius: 3px;
		padding: 0.1rem 0.45rem;
	}

	.state[data-stale='true'] ~ .bars {
		opacity: 0.55;
	}

	.link {
		background: none;
		border: none;
		color: var(--muted);
		text-decoration: underline;
		cursor: pointer;
		font-size: 0.8rem;
	}

	table {
		width: 100%;
		border-collapse: collapse;
		font-size: 0.85rem;
	}

	th,
	td {
		padding: 0.15rem 0.5rem;
		text-align: left;
		border-bottom: 1px solid var(--border);
		vertical-align: middle;
	}

	.r {
		text-align: right;
	}

	thead th {
		color: var(--muted);
		font-size: 0.7rem;
		text-transform: uppercase;
		letter-spacing: 0.06em;
		font-weight: 600;
	}

	.bars tbody th {
		font-weight: 600;
	}

	tr.detected {
		background: var(--accent-bg);
		outline: 2px solid var(--accent);
		outline-offset: -2px;
	}

	tr.moved {
		background: var(--lock-bg);
	}

	.role {
		display: inline-block;
		margin-right: 0.35rem;
		padding: 0 0.4rem;
		border: 1px solid var(--border-strong);
		border-radius: 3px;
		font-size: 0.7rem;
	}

	.role.staged {
		border-style: dashed;
		color: var(--accent);
	}

	.bar {
		position: relative;
		display: inline-block;
		width: 12rem;
		height: 0.6rem;
		background: var(--surface-2);
		border: 1px solid var(--border);
		border-radius: 2px;
		vertical-align: middle;
	}

	.bar.centred::after {
		content: '';
		position: absolute;
		left: 50%;
		top: -1px;
		bottom: -1px;
		width: 1px;
		background: var(--border-strong);
	}

	.fill {
		position: absolute;
		top: 0;
		bottom: 0;
		background: var(--accent);
	}

	.num {
		width: 4rem;
		text-align: right;
	}

	.diff {
		color: var(--accent);
		font-weight: 600;
	}

	tr.note td {
		border-bottom: 1px solid var(--border);
		font-size: 0.8rem;
	}

	tr.note span {
		margin-right: 1rem;
	}

	.warn {
		color: var(--warn);
	}

	.bad {
		color: var(--bad);
	}

	.ok {
		color: var(--ok);
	}

	.switch {
		display: grid;
		gap: 0.15rem;
		margin: 0 0 0.75rem;
	}

	.switch .name {
		font-weight: 600;
	}

	.tone-ok {
		color: var(--ok);
	}

	.tone-warn {
		color: var(--warn);
	}

	.gauge {
		position: relative;
		display: block;
		width: 16rem;
		height: 0.6rem;
		background: var(--surface-2);
		border: 1px solid var(--border);
		border-radius: 2px;
	}

	.gauge .mark {
		position: absolute;
		top: -3px;
		bottom: -3px;
		width: 2px;
		background: var(--bad);
	}

	.gauge .dot {
		position: absolute;
		top: 0;
		bottom: 0;
		width: 0.5rem;
		margin-left: -0.25rem;
		background: var(--muted);
		border-radius: 50%;
	}

	.gauge .dot.on {
		background: var(--accent);
	}

	.checks,
	.locks {
		list-style: none;
		margin: 0;
		padding: 0;
	}

	.checks li {
		margin: 0 0 0.4rem;
	}

	.locks li {
		display: flex;
		flex-wrap: wrap;
		gap: 0.5rem;
		align-items: baseline;
		padding: 0.1rem 0;
		border-bottom: 1px solid var(--border);
	}

	.lname {
		min-width: 15rem;
	}
</style>
