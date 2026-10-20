<script lang="ts">
	import { readable } from 'svelte/store';
	import { onMount } from 'svelte';
	import { app } from '$lib/state/app';
	import {
		formatBytes,
		plottableFields,
		plottableKinds,
		reportRows,
		shortSha,
		type LogsState
	} from '$lib/state/logs_state';
	import type { HelperConnectionState } from '$link/transport';
	import LogPlot from '$lib/components/LogPlot.svelte';
	import Badge from '$lib/components/ui/Badge.svelte';
	import Panel from '$lib/components/ui/Panel.svelte';

	const gcs = app();
	const logs = gcs.logs;
	const empty: LogsState = { logs: null, cards: [], library: null, selected: null, kind: null, fields: [], from: '', to: '', series: null, error: null, notice: null, busy: null };
	const st = logs?.state ?? readable<LogsState>(empty);
	const helperState = gcs.helper?.state ?? readable<HelperConnectionState>('closed');

	let importPath = $state('');

	onMount(() => {
		void logs?.load();
	});

	const entry = $derived($st.logs?.find((l) => l.sha256 === $st.selected) ?? null);
	const kindInfo = $derived(entry && $st.kind ? entry.summary.kinds[$st.kind] : undefined);
	const origin = $derived(kindInfo?.t_min_us ?? 0);
	const whyNoImport = $derived($helperState !== 'open' ? 'the helper is not connected' : $st.busy ? 'a command is in progress' : null);
	const whyNoPlot = $derived(
		$helperState !== 'open' ? 'the helper is not connected' : $st.busy ? 'a command is in progress' : !entry ? 'choose a log' : $st.fields.length === 0 ? 'choose a field' : null
	);
</script>

<svelte:head><title>Logs · MARV GCS</title></svelte:head>

<h1>Logs</h1>
<p class="muted lead">
	The onboard logger's .mlog files, pulled from the SD card. The helper copies a file into its library (never into the repository), records its SHA-256,
	reads it with the logger's own decoder and sends the plot only the minimum and maximum of each field per time bucket. Retrieval over the link (IF-7) is later.
</p>

<div class="grid">
	<div class="stack">
		<Panel title="Import" label="helper">
			<h4>Mounted cards</h4>
			{#if $st.cards.length === 0}
				<p class="muted small" data-logs="no-cards">No .mlog file on a mounted card (<span class="mono">&lt;mount&gt;/marv/log</span>). Mount the card, then rescan.</p>
			{:else}
				<ul class="files" aria-label="Card files">
					{#each $st.cards as c (c.path)}
						<li data-card={c.name}>
							<span class="mono">{c.name}</span>
							<span class="muted small">{c.mount} · {formatBytes(c.size)}</span>
							<button type="button" aria-label={`Import ${c.name}`} disabled={whyNoImport !== null} title={whyNoImport ?? 'Copy it into the library'} onclick={() => void logs?.importPath(c.path)}>Import</button>
						</li>
					{/each}
				</ul>
			{/if}
			<div class="buttons">
				<button type="button" disabled={$helperState !== 'open' || $st.busy !== null} onclick={() => void logs?.load()}>Rescan</button>
			</div>
			<h4>A path</h4>
			<div class="row">
				<label for="log-path">Absolute path of an .mlog file</label>
				<input id="log-path" type="text" class="mono path" spellcheck="false" autocomplete="off" placeholder="/home/…/00000001.mlog" bind:value={importPath} />
				<button type="button" class="primary" disabled={whyNoImport !== null || importPath.trim() === ''} title={whyNoImport ?? 'Copy it into the library'} onclick={() => void logs?.importPath(importPath)}>
					{$st.busy === 'import' ? 'Importing…' : 'Import path'}
				</button>
			</div>
			{#if $st.error}<p class="notice bad small" role="alert" data-notice="logs">{$st.error}</p>{/if}
			{#if $st.notice}<p class="notice small" role="status" data-notice="import">{$st.notice}</p>{/if}
		</Panel>

		<Panel title="Library" label="helper">
			{#if $st.logs === null}
				<p class="muted small">Not read yet.</p>
			{:else if $st.logs.length === 0}
				<p class="muted small" data-logs="empty">No log imported in this helper session.</p>
			{:else}
				<table class="mono small" aria-label="Imported logs">
					<thead><tr><th>file</th><th>sha256</th><th>size</th><th>decoder</th><th></th></tr></thead>
					<tbody>
						{#each $st.logs as l (l.sha256)}
							<tr data-sha={l.sha256} data-selected={l.sha256 === $st.selected}>
								<td>{l.name}</td>
								<td title={l.sha256}>{shortSha(l.sha256)}</td>
								<td>{formatBytes(l.size)}</td>
								<td>
									{#if l.summary.decoder_error}<Badge text="failed" tone="bad" />
									{:else if l.summary.clean}<Badge text="clean" tone="ok" />
									{:else}<Badge text={`${l.summary.problems.length} problem${l.summary.problems.length === 1 ? '' : 's'}`} tone="bad" />{/if}
								</td>
								<td><button type="button" aria-label={`View ${l.name} ${shortSha(l.sha256)}`} onclick={() => logs?.select(l.sha256)}>View</button></td>
							</tr>
						{/each}
					</tbody>
				</table>
				{#if $st.library}<p class="muted small wrap">Library: <span class="mono">{$st.library}</span></p>{/if}
			{/if}
		</Panel>
	</div>

	<div class="stack">
		<Panel title="Decoder report" label="helper">
			{#if !entry}
				<p class="muted small">Choose a log of the library.</p>
			{:else}
				<dl class="facts mono" aria-label="Decoder report" data-sha={entry.sha256}>
					<dt>file</dt><dd data-report="name">{entry.name}</dd>
					<dt>sha256</dt><dd class="wrap" data-report="sha256">{entry.sha256}</dd>
					<dt>size</dt><dd data-report="size">{entry.size} B</dd>
					{#each reportRows(entry.summary) as r (r.key)}
						<dt>{r.name}</dt><dd class={`wrap tone-${r.tone}`} data-report={r.key} data-tone={r.tone}>{r.value}</dd>
					{/each}
				</dl>
				{#if entry.summary.gaps.shown.length > 0}
					<table class="mono small" aria-label="Gap records">
						<thead><tr>{#each Object.keys(entry.summary.gaps.shown[0] ?? {}) as k (k)}<th>{k}</th>{/each}</tr></thead>
						<tbody>
							{#each entry.summary.gaps.shown as g, i (i)}<tr>{#each Object.values(g) as v, j (j)}<td>{v}</td>{/each}</tr>{/each}
						</tbody>
					</table>
				{/if}
			{/if}
		</Panel>

		<Panel title="Plot" label="helper">
			{#if !entry}
				<p class="muted small">Choose a log of the library.</p>
			{:else if plottableKinds(entry.summary).length === 0}
				<p class="muted small" data-logs="nothing-to-plot">The decoder read no record with a time and a numeric field.</p>
			{:else}
				<div class="row">
					<label for="plot-kind">Record kind</label>
					<select id="plot-kind" value={$st.kind ?? ''} disabled={$st.busy !== null} onchange={(e) => logs?.setKind(e.currentTarget.value)}>
						{#each plottableKinds(entry.summary) as k (k)}<option value={k}>{k}</option>{/each}
					</select>
				</div>
				<fieldset class="fields">
					<legend>Fields</legend>
					{#each plottableFields(kindInfo) as f (f)}
						<label class="plain"><input type="checkbox" checked={$st.fields.includes(f)} onchange={(e) => logs?.toggleField(f, e.currentTarget.checked)} /> {f}</label>
					{/each}
				</fieldset>
				<div class="row">
					<label for="plot-from">From (s)</label>
					<input id="plot-from" type="text" inputmode="decimal" class="mono secs" placeholder="start" value={$st.from} oninput={(e) => logs?.setRange(e.currentTarget.value, $st.to)} />
					<label for="plot-to">To (s)</label>
					<input id="plot-to" type="text" inputmode="decimal" class="mono secs" placeholder="end" value={$st.to} oninput={(e) => logs?.setRange($st.from, e.currentTarget.value)} />
					<button type="button" class="primary" disabled={whyNoPlot !== null} title={whyNoPlot ?? 'Ask the helper for the minimum and maximum per bucket'} onclick={() => void logs?.plot()}>
						{$st.busy === 'series' ? 'Plotting…' : 'Plot'}
					</button>
				</div>
				{#if $st.series && $st.series.sha256 === entry.sha256}
					<p class="muted small" data-series="summary">
						{$st.series.kind}: {$st.series.records} records in {$st.series.buckets} buckets of {$st.series.bucket_us} µs
					</p>
					{#each Object.entries($st.series.fields) as [name, f] (name)}
						<LogPlot field={name} min={f.min} max={f.max} t0Us={$st.series.t0_us} t1Us={$st.series.t1_us} originUs={origin} />
						{#if f.skipped > 0}<p class="muted small">{name}: {f.skipped} non-finite value{f.skipped === 1 ? '' : 's'} left out</p>{/if}
					{/each}
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

	h4 {
		margin: 0.6rem 0 0.25rem;
		color: var(--muted);
		font-size: 0.65rem;
		text-transform: uppercase;
		letter-spacing: 0.06em;
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
		color: inherit;
	}

	.path {
		flex: 1 1 18rem;
	}

	.secs {
		width: 6rem;
	}

	.files {
		list-style: none;
		margin: 0 0 0.5rem;
		padding: 0;
	}

	.files li {
		display: flex;
		flex-wrap: wrap;
		gap: 0.5rem;
		align-items: baseline;
		margin: 0 0 0.3rem;
	}

	.fields {
		display: flex;
		flex-wrap: wrap;
		gap: 0.25rem 1rem;
		margin: 0 0 0.5rem;
		padding: 0.25rem 0.5rem;
		border: 1px solid var(--border);
	}

	.fields legend {
		color: var(--muted);
		font-size: 0.75rem;
		text-transform: uppercase;
		letter-spacing: 0.06em;
	}

	table {
		border-collapse: collapse;
		width: 100%;
		margin: 0 0 0.5rem;
	}

	th,
	td {
		text-align: left;
		padding: 0.15rem 0.5rem 0.15rem 0;
		border-bottom: 1px solid var(--border);
	}

	th {
		color: var(--muted);
		font-weight: 600;
	}

	tr[data-selected='true'] td {
		color: var(--accent);
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

	.tone-ok {
		color: var(--ok);
	}

	.tone-warn {
		color: var(--warn);
	}

	.tone-bad {
		color: var(--bad);
	}
</style>
