<script lang="ts">
	import { onMount } from 'svelte';
	import { readable } from 'svelte/store';
	import { app } from '$lib/state/app';
	import { SIL_LABEL, runView, type Notice, type SimState } from '$lib/state/sim_state';
	import type { HelperConnectionState } from '$link/transport';
	import type { HelperStatus } from '$lib/state/helper_api';
	import Badge from '$lib/components/ui/Badge.svelte';
	import Panel from '$lib/components/ui/Panel.svelte';

	const gcs = app();
	const sim = gcs.sim;
	const simState = sim?.state ?? readable<SimState | null>(null);
	const status = gcs.status ?? readable<HelperStatus | null>(null);
	const helperState = gcs.helper?.state ?? readable<HelperConnectionState>('closed');

	onMount(() => {
		void sim?.loadCards();
		void sim?.loadParameters();
	});

	const form = $derived($simState?.form ?? { card: '', overrides: [], gui: false });
	const placed = $derived($simState?.placed ?? { card: [], rows: {}, run: [] });
	const plan = $derived($simState?.plan ?? null);
	const busy = $derived($simState?.busy ?? null);
	const view = $derived(runView($status));
	const run = $derived(view.run);
	const hasOverrides = $derived(form.overrides.some((r) => r.name.trim() !== '' || r.value.trim() !== ''));
	const autoconnect = $derived($status?.autoconnect ?? null);

	const whyNoLaunch = $derived.by(() => {
		if ($helperState !== 'open') return 'the helper is not connected';
		if (busy) return 'a command is in progress';
		if (view.running) return 'a run is going: one simulation at a time';
		return null;
	});
	const whyNoStop = $derived(busy ? 'a command is in progress' : !view.running ? 'no run is going' : null);

	const noticeTone = (n: Notice): string => (n.source === 'helper' ? 'helper' : 'form');
</script>

<svelte:head><title>Simulation · MARV GCS</title></svelte:head>

<h1>Simulation</h1>
<p class="muted lead">
	Define, launch and stop a software-in-the-loop run. The page talks to the helper only; when the run's link is up, the helper's autoconnect
	opens it and the status bar shows the vehicle. A parameter override changes the firmware's value in the simulation only: it is labelled
	<span class="mono">{SIL_LABEL}</span> here and in the run, and it never goes into a config for a board.
</p>

<div class="grid">
	<Panel title="Run definition" label="sil">
		<div class="row">
			<label for="sim-card">Card</label>
			<select id="sim-card" value={form.card} disabled={busy !== null || view.running} onchange={(e) => sim?.setCard(e.currentTarget.value)}>
				<option value="">the run script's default</option>
				{#each $simState?.cards ?? [] as c (c)}<option value={c}>{c}</option>{/each}
			</select>
		</div>
		{#if $simState?.cardsError}<p class="bad small" role="alert" data-notice="cards">{$simState.cardsError}</p>{/if}
		{#each placed.card as n, i (i)}<p class="notice bad small" role="alert" data-notice="card" data-source={noticeTone(n)}>{n.text}</p>{/each}

		<h3 class="sub">Parameter overrides</h3>
		<datalist id="sim-parameters">
			{#each $simState?.parameters ?? [] as p (p.name)}<option value={p.name}>{p.type}{p.unit ? ` ${p.unit}` : ''}{p.locked ? ', locked' : ''}</option>{/each}
		</datalist>
		{#if $simState?.parametersNote}<p class="muted small" data-notice="parameters">{$simState.parametersNote}</p>{/if}
		{#if form.overrides.length === 0}<p class="muted small">No override: the firmware runs with the values of its config.</p>{/if}
		<ul class="rows" aria-label="Parameter overrides">
			{#each form.overrides as r, i (r.id)}
				<li class="orow" data-label={SIL_LABEL}>
					<div class="row">
						<input
							type="text"
							list="sim-parameters"
							aria-label={`Override ${i + 1} parameter`}
							placeholder="parameter name"
							spellcheck="false"
							autocomplete="off"
							class="mono name"
							value={r.name}
							disabled={busy !== null || view.running}
							oninput={(e) => sim?.setOverride(r.id, { name: e.currentTarget.value })}
						/>
						<input
							type="text"
							inputmode="decimal"
							aria-label={`Override ${i + 1} value`}
							placeholder="value"
							spellcheck="false"
							autocomplete="off"
							class="mono value"
							value={r.value}
							disabled={busy !== null || view.running}
							oninput={(e) => sim?.setOverride(r.id, { value: e.currentTarget.value })}
						/>
						<Badge text={SIL_LABEL} tone="warn" />
						<button type="button" aria-label={`Remove override ${i + 1}`} disabled={busy !== null || view.running} onclick={() => sim?.removeOverride(r.id)}>Remove</button>
					</div>
					{#each placed.rows[r.id] ?? [] as n, k (k)}<p class="notice bad small" role="alert" data-notice="override" data-source={noticeTone(n)}>{n.text}</p>{/each}
				</li>
			{/each}
		</ul>
		<div class="buttons">
			<button type="button" disabled={busy !== null || view.running} onclick={() => sim?.addOverride()}>Add override</button>
		</div>

		<div class="row">
			<input id="sim-gui" type="checkbox" checked={form.gui} disabled={busy !== null || view.running} onchange={(e) => sim?.setGui(e.currentTarget.checked)} />
			<label for="sim-gui" class="plain">GUI: the gz window and the display-only visuals</label>
		</div>

		<div class="buttons">
			<button type="button" disabled={busy !== null || $helperState !== 'open'} onclick={() => void sim?.plan()}>{busy === 'plan' ? 'Planning…' : 'Plan'}</button>
			<button type="button" class="primary" disabled={whyNoLaunch !== null} title={whyNoLaunch ?? 'Start the run'} onclick={() => void sim?.launch()}>{busy === 'launch' ? 'Launching…' : 'Launch'}</button>
			<button type="button" disabled={whyNoStop !== null} title={whyNoStop ?? 'Stop the run and its processes'} onclick={() => void sim?.stop()}>{busy === 'stop' ? 'Stopping…' : 'Stop'}</button>
			{#if hasOverrides}<Badge text={`${SIL_LABEL} run`} tone="warn" />{/if}
		</div>
		{#each placed.run as n, i (i)}<p class="notice bad small" role="alert" data-notice="run" data-source={noticeTone(n)}>{n.text}</p>{/each}
		{#if autoconnect && !autoconnect.enabled}
			<p class="warn small">Autoconnect is off: the vehicle will not connect by itself; open the link from the Connection page.</p>
		{/if}
	</Panel>

	<div class="stack">
		{#if plan}
			<Panel title="Plan" label="nothing started">
				<dl class="facts mono" aria-label="Plan">
					<dt>run id</dt><dd>{plan.run_id}</dd>
					<dt>label</dt><dd>{#if plan.label}<Badge text={plan.label} tone="warn" />{:else}none: no override{/if}</dd>
					<dt>allowed</dt><dd>{#if plan.allowed}<Badge text="yes" tone="ok" />{:else}<Badge text="refused" tone="bad" />{/if}</dd>
					<dt>command</dt><dd class="wrap" data-plan="argv">{plan.argv.join(' ')}</dd>
					<dt>output</dt><dd class="wrap">{plan.out_dir}</dd>
					{#if plan.udp_port !== null}<dt>link</dt><dd>udp 127.0.0.1:{plan.udp_port}</dd>{/if}
				</dl>
				{#if plan.overrides.length > 0}
					<table class="mono small" aria-label="Planned overrides">
						<thead><tr><th>parameter</th><th>type</th><th>value</th><th></th></tr></thead>
						<tbody>
							{#each plan.overrides as o (o.name)}
								<tr><td>{o.name}</td><td>{o.type}</td><td>{o.value}</td><td><Badge text={SIL_LABEL} tone="warn" /></td></tr>
							{/each}
						</tbody>
					</table>
				{/if}
			</Panel>
		{/if}

		<Panel title="Run" label="helper">
			{#if !run}
				<p class="muted" data-run="none">No run in this helper session.</p>
			{:else}
				<dl class="facts mono" aria-label="Run">
					<dt>state</dt>
					<dd>
						<Badge text={view.running ? 'running' : run.state} tone={view.running ? 'ok' : 'neutral'} />
						{#if view.label}<Badge text={view.label} tone="warn" />{/if}
					</dd>
					<dt>run id</dt><dd>{run.run_id}</dd>
					<dt>kind</dt><dd>{run.kind}</dd>
					{#if run.options?.card}<dt>card</dt><dd>{run.options.card}</dd>{/if}
					{#if run.options && 'gui' in run.options}<dt>gui</dt><dd>{run.options.gui ? 'on' : 'off'}</dd>{/if}
					<dt>relaunches</dt><dd>{run.relaunches}</dd>
					<dt>exit</dt><dd>{run.exit_status === null ? '—' : run.exit_status}</dd>
				</dl>
				{#if (run.overrides?.length ?? 0) > 0}
					<table class="mono small" aria-label="Run overrides">
						<thead><tr><th>parameter</th><th>type</th><th>value</th><th></th></tr></thead>
						<tbody>
							{#each run.overrides ?? [] as o (o.name)}
								<tr data-label={SIL_LABEL}><td>{o.name}</td><td>{o.type}</td><td>{o.value}</td><td><Badge text={SIL_LABEL} tone="warn" /></td></tr>
							{/each}
						</tbody>
					</table>
					<p class="muted small">These values were forced in the simulation's firmware. They are not in any config for a board.</p>
				{/if}
			{/if}
		</Panel>
	</div>
</div>

<style>
	.lead {
		margin: 0.2rem 0 0.75rem;
	}

	.grid {
		display: grid;
		grid-template-columns: repeat(auto-fit, minmax(30rem, 1fr));
		gap: 0.75rem;
		align-items: start;
	}

	.stack {
		display: flex;
		flex-direction: column;
		gap: 0.75rem;
	}

	.row,
	.buttons {
		display: flex;
		flex-wrap: wrap;
		gap: 0.5rem;
		align-items: center;
		margin: 0 0 0.5rem;
	}

	label {
		color: var(--muted);
		font-size: 0.75rem;
		text-transform: uppercase;
		letter-spacing: 0.06em;
	}

	label.plain {
		text-transform: none;
		letter-spacing: normal;
		font-size: 0.85rem;
	}

	.sub {
		margin: 0.75rem 0 0.4rem;
	}

	.rows {
		list-style: none;
		margin: 0 0 0.5rem;
		padding: 0;
	}

	.orow {
		margin: 0 0 0.25rem;
		padding-left: 0.5rem;
		border-left: 3px solid var(--warn);
	}

	.name {
		width: 16rem;
	}

	.value {
		width: 9rem;
	}

	.notice {
		margin: 0 0 0.35rem;
		overflow-wrap: anywhere;
	}

	.small {
		font-size: 0.8rem;
	}

	.facts {
		display: grid;
		grid-template-columns: max-content 1fr;
		gap: 0.2rem 1rem;
		margin: 0 0 0.5rem;
		font-size: 0.85rem;
	}

	.facts dt {
		color: var(--muted);
	}

	.facts dd {
		margin: 0;
	}

	.wrap {
		overflow-wrap: anywhere;
	}

	table {
		border-collapse: collapse;
		margin: 0 0 0.5rem;
	}

	th,
	td {
		text-align: left;
		padding: 0.15rem 0.75rem 0.15rem 0;
	}

	th {
		color: var(--muted);
		font-weight: 400;
	}
</style>
