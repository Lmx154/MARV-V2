<script lang="ts">
	import { onMount } from 'svelte';
	import { readable } from 'svelte/store';
	import { simView } from '$lib/sim/instance';
	import type { SimView } from '$lib/sim/sim_view';
	import SimPanel from '$lib/sim/SimPanel.svelte';

	const sim = simView();
	const view = sim?.view ?? readable<SimView | null>(null);

	// The layout starts the channel; a page opened first starts it too (a second start does nothing).
	onMount(() => {
		sim?.start();
	});
</script>

<svelte:head><title>SIM · MARV GCS</title></svelte:head>

<h1>SIM</h1>
<p class="muted lead">
	Simulator data, not the vehicle's: the truth the simulator holds and what it injects into the firmware, on the sim channel the simulator
	publishes beside the link. These panels exist only while a simulation is publishing and are labelled SIM; the vehicle's own view is on the
	Vehicle page.
</p>

{#if !sim}
	<p class="muted" role="status">No helper: there is no sim channel to read.</p>
{:else if $view && $view.present}
	<div class="grid" data-sim-present="true">
		{#each $view.panels as p (p.id)}
			<SimPanel panel={p} />
		{/each}
	</div>
{:else}
	<p class="muted" role="status" data-sim-present="false">
		No simulation is publishing the sim channel{$view?.stream.connected ? '' : ' (the channel to the helper is not open)'}. Launch one from the
		Simulation page.
	</p>
{/if}

<style>
	.lead {
		margin: 0.2rem 0 0.75rem;
	}

	.grid {
		display: grid;
		grid-template-columns: repeat(auto-fit, minmax(24rem, 1fr));
		gap: 0.75rem;
		align-items: start;
	}
</style>
