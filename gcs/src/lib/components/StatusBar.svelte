<script lang="ts">
	import { readable } from 'svelte/store';
	import { app } from '$lib/state/app';
	import type { Indicator } from '$lib/state/shell';
	import Badge from './ui/Badge.svelte';

	const gcs = typeof document === 'undefined' ? null : app();
	const bar = gcs?.bar ?? readable<Indicator[]>([]);
</script>

<!-- aria-live is off: the ages move twice a second, a screen reader is not told of each. -->
<div class="bar" role="status" aria-label="Status bar" aria-live="off">
	{#each $bar as ind (ind.id)}
		<div class="ind" role="group" aria-label={ind.label} data-indicator={ind.id} data-stale={ind.stale}>
			<span class="k">{ind.label}</span>
			<Badge text={ind.summary} tone={ind.stale ? 'neutral' : ind.tone} />
			{#if ind.stale}<span class="stale" title="not heard from recently: the value shown is the last one">stale</span>{/if}
			{#each ind.fields as f (f.name)}
				<span class="f mono"><span class="fk">{f.name}</span> {f.value}</span>
			{/each}
		</div>
	{/each}
</div>

<style>
	.bar {
		display: flex;
		flex-wrap: wrap;
		gap: 0.25rem 1.25rem;
		align-items: baseline;
		font-size: 0.75rem;
		padding-bottom: 0.4rem;
	}

	.ind {
		display: flex;
		flex-wrap: wrap;
		gap: 0.1rem 0.5rem;
		align-items: baseline;
	}

	.ind[data-stale='true'] .f {
		opacity: 0.55;
	}

	.k,
	.fk {
		color: var(--muted);
		text-transform: uppercase;
		letter-spacing: 0.06em;
		font-size: 0.65rem;
	}

	.fk {
		margin-right: 0.15rem;
	}

	.stale {
		color: var(--warn);
		font-size: 0.65rem;
		text-transform: uppercase;
		letter-spacing: 0.06em;
	}
</style>
