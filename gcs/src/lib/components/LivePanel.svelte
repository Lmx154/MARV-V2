<script lang="ts">
	import { formatHz, formatStreamAge, type PanelView, type Row } from '$lib/state/live_state';
	import Badge from './ui/Badge.svelte';
	import Panel from './ui/Panel.svelte';

	let { panel }: { panel: PanelView } = $props();

	/** The rows in order, with the group heading before the first row of each group. */
	const groups = $derived.by(() => {
		const out: Array<{ name: string | null; rows: Row[] }> = [];
		for (const row of panel.rows) {
			const last = out[out.length - 1];
			if (last && last.name === row.group) last.rows.push(row);
			else out.push({ name: row.group, rows: [row] });
		}
		return out;
	});
</script>

<!-- Same pattern as the status bar: a group with its own data-stale. A stale panel keeps its last values on screen, marked. -->
<div class="live" role="group" aria-label={panel.title} data-panel={panel.id} data-stale={panel.stale} data-heard={panel.heard}>
	<Panel title={panel.title}>
		<div class="state">
			{#if panel.stale}
				<span class="stale" title="not heard from recently: the values shown are the last ones">stale</span>
			{:else}
				<Badge text="live" tone="ok" />
			{/if}
			{#each panel.streams as s (s.id)}
				<span class="stream mono" data-stream={s.id} data-stale={s.stale} title={`stream ${s.id}: last heard ${formatStreamAge(s.ageMs)} ago`}>
					{s.id} {s.stale ? formatStreamAge(s.ageMs) : formatHz(s.hz)}
				</span>
			{/each}
		</div>
		{#each groups as g (g.name ?? '')}
			{#if g.name}<h4>{g.name}</h4>{/if}
			<dl class="facts mono">
				{#each g.rows as r (r.key)}
					<dt>{r.name}</dt>
					<dd data-row={r.key} data-tone={r.tone}>
						<span class="v tone-{r.tone}">{r.value}</span>{#if r.unit}<span class="u">{r.unit}</span>{/if}
						{#if r.bar !== null}
							<span class="bar" class:centred={r.centred} aria-hidden="true">
								<span class="fill" style={r.centred ? `left:${Math.min(r.bar, 0.5) * 100}%;width:${Math.abs(r.bar - 0.5) * 100}%` : `left:0;width:${r.bar * 100}%`}></span>
							</span>
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
		display: flex;
		flex-wrap: wrap;
		gap: 0.2rem 0.75rem;
		align-items: baseline;
		margin: 0 0 0.5rem;
		font-size: 0.75rem;
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

	.stream {
		color: var(--muted);
	}

	.stream[data-stale='true'] {
		color: var(--warn);
	}

	.live[data-stale='true'] .facts {
		opacity: 0.55;
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

	.small {
		font-size: 0.8rem;
	}
</style>
