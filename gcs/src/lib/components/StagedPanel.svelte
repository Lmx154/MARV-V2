<script lang="ts">
	import { formatValue } from '$link/bits';
	import type { ParamRow } from '$lib/params/merge';
	import Panel from './ui/Panel.svelte';

	let {
		rows,
		onrevert,
		onreset
	}: {
		rows: ParamRow[];
		onrevert: () => void;
		onreset: () => void;
	} = $props();

	/** The activation needs a validation report for the (image, config) pair: unit 9. It stays off in this unit. */
	const ACTIVATE_READY = false;
	/** Revert and reset are wired to the vehicle's config commands, which the vehicle answers from unit 6. */
	const CONFIG_COMMANDS_READY = false;

	const fmt = (r: ParamRow, bits: number | null): string => (bits === null ? '—' : formatValue(bits, r.type));
</script>

<Panel title={`Staged changes (${rows.length})`} label="staged vs active">
	{#if rows.length === 0}
		<p class="muted">Nothing is staged: the staged config equals the active one.</p>
	{:else}
		<ul class="list mono">
			{#each rows as r (r.key)}
				<li><span class="n">{r.name}</span> <span class="v">{fmt(r, r.activeBits)} → <strong>{fmt(r, r.stagedBits)}</strong> {r.unit}</span></li>
			{/each}
		</ul>
	{/if}
	<div class="buttons">
		<button type="button" class="primary" disabled={!ACTIVATE_READY} aria-describedby="why-activate">Activate</button>
		<span id="why-activate" class="why">needs a validation report (unit 9)</span>
	</div>
	<div class="buttons">
		<button type="button" disabled={!CONFIG_COMMANDS_READY || rows.length === 0} onclick={onrevert} aria-describedby="why-cmds">Revert staged</button>
		<button type="button" disabled={!CONFIG_COMMANDS_READY} onclick={onreset} aria-describedby="why-cmds">Reset to factory</button>
		<span id="why-cmds" class="why">needs the vehicle's config commands (unit 6)</span>
	</div>
</Panel>

<style>
	.list {
		list-style: none;
		margin: 0 0 0.5rem;
		padding: 0;
		font-size: 0.8rem;
		max-height: 14rem;
		overflow: auto;
	}

	.list li {
		display: flex;
		justify-content: space-between;
		gap: 1rem;
		padding: 0.1rem 0;
		border-bottom: 1px solid var(--border);
	}

	.n {
		overflow-wrap: anywhere;
	}

	.v {
		white-space: nowrap;
	}

	.buttons {
		display: flex;
		gap: 0.5rem;
		align-items: center;
		flex-wrap: wrap;
		margin-top: 0.4rem;
	}

	.why {
		font-size: 0.75rem;
		color: var(--muted);
	}
</style>
