<script lang="ts">
	import type { SimPanelView, SimRow } from './sim_view';
	import Panel from '$lib/components/ui/Panel.svelte';
	import Badge from '$lib/components/ui/Badge.svelte';

	let { panel }: { panel: SimPanelView } = $props();

	const groups = $derived.by(() => {
		const out: Array<{ name: string | null; rows: SimRow[] }> = [];
		for (const row of panel.rows) {
			const last = out[out.length - 1];
			if (last && last.name === row.group) last.rows.push(row);
			else out.push({ name: row.group, rows: [row] });
		}
		return out;
	});
</script>

<!-- A SIM panel: simulator data, labelled as such in its title and its badge. It exists only while a simulation publishes. -->
<div class="sim" role="group" aria-label={panel.title} data-sim-panel={panel.id}>
	<Panel title={panel.title}>
		<div class="state"><Badge text="SIM" tone="warn" /></div>
		{#each groups as g (g.name ?? '')}
			{#if g.name}<h4>{g.name}</h4>{/if}
			<dl class="facts mono">
				{#each g.rows as r (r.key)}
					<dt>{r.name}</dt>
					<dd data-row={r.key} data-tone={r.tone}>
						<span class="v tone-{r.tone}">{r.value}</span>{#if r.unit}<span class="u">{r.unit}</span>{/if}
						{#if r.bar !== null}
							<span class="bar" aria-hidden="true"><span class="fill" style={`left:0;width:${r.bar * 100}%`}></span></span>
						{/if}
					</dd>
				{/each}
			</dl>
		{/each}
		{#each panel.notes as n (n)}<p class="muted small">{n}</p>{/each}
	</Panel>
</div>

<style>
	.state {
		margin: 0 0 0.5rem;
	}

	h4 {
		margin: 0.6rem 0 0.25rem;
		color: var(--muted);
		font-size: 0.65rem;
		text-transform: uppercase;
		letter-spacing: 0.06em;
		font-weight: 600;
	}

	.facts {
		display: grid;
		grid-template-columns: max-content 1fr;
		gap: 0.15rem 1rem;
		margin: 0 0 0.25rem;
		font-size: 0.85rem;
	}

	.facts dt {
		color: var(--muted);
	}

	.facts dd {
		margin: 0;
		display: flex;
		align-items: baseline;
		gap: 0.1rem;
		flex-wrap: wrap;
	}

	.u {
		color: var(--muted);
		margin-left: 0.25em;
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

	.bar {
		position: relative;
		display: inline-block;
		width: 6rem;
		height: 0.5rem;
		margin-left: 0.5rem;
		align-self: center;
		background: var(--surface-2);
		border: 1px solid var(--border);
		border-radius: 2px;
	}

	.fill {
		position: absolute;
		top: 0;
		bottom: 0;
		background: var(--accent);
	}

	.small {
		font-size: 0.8rem;
	}
</style>
