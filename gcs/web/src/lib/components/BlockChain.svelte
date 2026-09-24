<script lang="ts">
	import type { Schema } from '$lib/gcs/types';
	import type { Rejected } from '$lib/gcs/setup';
	import BlockCard from './BlockCard.svelte';

	let {
		schema,
		kinds,
		value,
		reference,
		rejected,
		pending,
		disabled,
		onkind,
		onparam,
		onreset
	}: {
		schema: Schema;
		/** Staged kind index per family. */
		kinds: number[];
		value: (index: number) => number;
		/** Per family, the values "modified" is measured against for the block's current kind. */
		reference: (family: number) => Record<string, number>;
		rejected: Record<number, Rejected>;
		pending: (index: number) => boolean;
		disabled: boolean;
		onkind: (family: number, kind: number) => void;
		onparam: (index: number, value: number) => void;
		onreset: (family: number) => void;
	} = $props();
</script>

<div class="chain" role="list" aria-label="Block chain, in the order one flight-software tick runs">
	{#each schema.families as family, i (family.id)}
		{#if i > 0}<span class="arrow" aria-hidden="true">→</span>{/if}
		<div class="cell" role="listitem">
			<BlockCard
				{family}
				kind={kinds[i] ?? 0}
				{value}
				reference={reference(i)}
				{rejected}
				{pending}
				{disabled}
				onkind={(kind) => onkind(i, kind)}
				{onparam}
				onreset={() => onreset(i)}
			/>
		</div>
	{/each}
</div>
<p class="loop mono">↻ the actuators drive the vehicle: the loop closes through the plant, which the sensors sample again.</p>

<style>
	.chain {
		display: flex;
		flex-wrap: wrap;
		align-items: flex-start;
		gap: 0.6rem 0.3rem;
		min-width: 0;
	}
	.cell {
		display: contents;
	}
	.arrow {
		align-self: center;
		color: var(--muted);
		font-size: 1.1rem;
		flex: 0 0 auto;
	}
	.loop {
		margin: 0.4rem 0 0;
		font-size: 0.75rem;
		color: var(--muted);
	}
</style>
