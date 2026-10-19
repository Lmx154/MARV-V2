<script lang="ts">
	import { onMount } from 'svelte';
	import { readable } from 'svelte/store';
	import { app } from '$lib/state/app';
	import type { LiveView } from '$lib/state/live_state';
	import LivePanel from '$lib/components/LivePanel.svelte';

	const gcs = app();
	const live = gcs.live;
	const view = live?.view ?? readable<LiveView | null>(null);

	// The streams are asked for while the page is open and stopped when it is left (and when the tab is closed, best effort).
	onMount(() => {
		live?.start();
		const stop = (): void => live?.stop();
		window.addEventListener('pagehide', stop);
		return () => {
			window.removeEventListener('pagehide', stop);
			live?.stop();
		};
	});

	const stalePanels = $derived($view ? $view.panels.filter((p) => p.stale).length : 0);
</script>

<svelte:head><title>Vehicle · MARV GCS</title></svelte:head>

<h1>Vehicle</h1>
<p class="muted lead">
	The vehicle's own telemetry, over the link, as the vehicle sends it: the same panels for a simulation and for a board. The page asks the
	vehicle for its streams while it is open and stops them when you leave. A panel goes <span class="mono">stale</span> when one of its streams
	has not been heard for a while; it keeps its last values on screen, marked.
</p>

{#if !live}
	<p class="muted" role="status">No helper: there is no link to read.</p>
{:else if $view}
	<p class="muted small" role="status" data-summary>
		{stalePanels === 0 ? 'every panel is live' : `${stalePanels} of ${$view.panels.length} panels stale`}
		{#if $view.requests.refused > 0}· the vehicle refused {$view.requests.refused} stream request(s) (last answer: {$view.requests.lastRefusal}){/if}
	</p>
	<div class="grid">
		{#each $view.panels as p (p.id)}
			<LivePanel panel={p} />
		{/each}
	</div>
	{#if !$view.channelMapKnown}
		<div class="buttons">
			<button type="button" onclick={() => void gcs.loadParams()}>Read parameters from the vehicle</button>
			<span class="muted small">to show the sticks by their channels</span>
		</div>
	{/if}
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
		grid-template-columns: repeat(auto-fit, minmax(24rem, 1fr));
		gap: 0.75rem;
		align-items: start;
	}

	.buttons {
		display: flex;
		flex-wrap: wrap;
		gap: 0.5rem;
		align-items: center;
		margin: 0.75rem 0;
	}
</style>
