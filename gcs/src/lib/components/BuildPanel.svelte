<script lang="ts">
	import { onMount } from 'svelte';
	import { readable } from 'svelte/store';
	import { app } from '$lib/state/app';
	import { buildOutcome, statusText, whyNoBuild, type BuildState } from '$lib/state/build_state';
	import { shortSha } from '$lib/state/logs_state';
	import type { HelperConnectionState } from '$link/transport';
	import Badge from '$lib/components/ui/Badge.svelte';
	import Panel from '$lib/components/ui/Panel.svelte';

	const gcs = app();
	const build = gcs.build;
	const bstate = build?.state ?? readable<BuildState | null>(null);
	const manifest = gcs.manifest;
	const helperState = gcs.helper?.state ?? readable<HelperConnectionState>('closed');

	// The status is a dry run: opening the page builds nothing (H-3). A build starts only on the Build button.
	onMount(() => void build?.refresh());

	const s = $derived($bstate);
	const current = $derived(s?.current ?? null);
	const outcome = $derived(buildOutcome(current));
	const preset = $derived(s?.allowed?.find((p) => p.preset === s.preset) ?? null);
	const why = $derived(s ? whyNoBuild(s, $helperState === 'open') : 'no helper');
	const whyNoCancel = $derived(s?.busy ? 'a command is in progress' : !current?.running ? 'no build is running' : null);
	const entries = $derived(preset ? ($manifest.images?.images.filter((e) => e.tree === preset.tree) ?? []) : []);
	const tone = (o: typeof outcome): 'ok' | 'bad' | 'warn' | 'neutral' => (o === 'ok' ? 'ok' : o === 'failed' ? 'bad' : o === 'cancelled' ? 'warn' : 'neutral');
</script>

<Panel title="Build" label="helper">
	<p class="muted small">
		The helper builds one of its allow-listed presets when you click Build, under the host's heavy lock; a launch never builds. It records
		<span class="mono">git describe --dirty</span> with the image it produces.
	</p>
	<div class="row">
		<label for="build-preset">Preset</label>
		<select id="build-preset" value={s?.preset ?? ''} disabled={!s?.allowed || s.busy !== null || current?.running} onchange={(e) => void build?.setPreset(e.currentTarget.value)}>
			{#if !s?.allowed}<option value="">no helper</option>{/if}
			{#each s?.allowed ?? [] as p (p.preset)}<option value={p.preset}>{p.preset}</option>{/each}
		</select>
		<button type="button" disabled={s?.busy !== null || $helperState !== 'open'} onclick={() => void build?.refresh()}>Check status</button>
	</div>
	<dl class="facts mono" aria-label="Build status">
		<dt>tree</dt><dd>{preset ? preset.tree : '—'}</dd>
		<dt>status</dt><dd data-build="status" data-kind={s?.status?.kind ?? ''}>{statusText(s?.status ?? null)}</dd>
	</dl>
	<div class="buttons">
		<button type="button" class="primary" disabled={why !== null} title={why ?? 'Build this preset under the heavy lock'} aria-describedby="why-build" onclick={() => void build?.build()}>
			{s?.busy === 'build' ? 'Starting…' : current?.running ? 'Building…' : 'Build'}
		</button>
		<button type="button" disabled={whyNoCancel !== null} title={whyNoCancel ?? 'Stop the build and every process of it'} onclick={() => void build?.cancel()}>Cancel</button>
		<span id="why-build" class="why" data-build="why">{why ?? ''}</span>
	</div>
	{#each s?.notices ?? [] as n, i (i)}<p class="bad small" role="alert" data-notice="build">{n}</p>{/each}
	{#if current}
		<dl class="facts mono" aria-label="Build">
			<dt>state</dt><dd data-build="state"><Badge text={outcome} tone={tone(outcome)} /></dd>
			<dt>preset</dt><dd>{current.preset}</dd>
			<dt>describe</dt><dd class="wrap" data-build="describe">{current.describe}</dd>
			{#if current.steps !== null}<dt>step</dt><dd data-build="step">{current.step}/{current.steps}</dd>{/if}
			{#if !current.running}<dt>exit</dt><dd>{current.exit_status === null ? '—' : current.exit_status}{current.error ? ` (${current.error})` : ''}</dd>{/if}
		</dl>
		{#if current.tail.length > 0}
			<ul class="progress mono small" aria-label="Build progress">
				{#each current.tail as line, i (i)}<li>{line}</li>{/each}
			</ul>
		{/if}
	{/if}
	{#if entries.length > 0}
		<table class="mono small" aria-label="Images of the build tree">
			<thead><tr><th>image</th><th>hash</th><th>describe</th></tr></thead>
			<tbody>
				{#each entries as e (e.hash)}
					<tr data-image={e.hash}><td>{e.name}</td><td>{shortSha(e.hash)}</td><td>{e.build ? e.build.describe : 'not built by the GCS'}</td></tr>
				{/each}
			</tbody>
		</table>
	{/if}
</Panel>

<style>
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

	.why {
		font-size: 0.75rem;
		color: var(--muted);
	}

	.progress {
		list-style: none;
		margin: 0.3rem 0;
		padding: 0;
		max-height: 10rem;
		overflow: auto;
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
