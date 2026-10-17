<!-- Copied unchanged from the avionics toolbox, src/lib/components/ui/Readout.svelte, toolbox commit 88658504d69757eda042961d7f02d8319e568e64 (MARV V2 ground segment, decision 0053 Q5). -->
<script lang="ts">
	let {
		label,
		value,
		unit,
		tone = 'neutral',
		size = 'md'
	}: {
		label: string;
		value: string;
		unit?: string;
		tone?: 'ok' | 'warn' | 'bad' | 'neutral';
		size?: 'md' | 'lg';
	} = $props();
</script>

<div class="readout tone-{tone} size-{size}">
	<span class="readout-label mono">{label}</span>
	<span class="readout-value mono">{value}{#if unit}<span class="readout-unit">{unit}</span>{/if}</span>
</div>

<style>
	.readout {
		position: relative;
		padding-left: 0.75rem;
		border-left: 3px solid var(--muted);
		display: flex;
		flex-direction: column;
		gap: 0.2rem;
	}

	.tone-ok {
		border-left-color: var(--ok);
	}

	.tone-warn {
		border-left-color: var(--warn);
	}

	.tone-bad {
		border-left-color: var(--bad);
	}

	.readout-label {
		font-size: 0.7rem;
		text-transform: uppercase;
		letter-spacing: 0.08em;
		color: var(--muted);
		font-weight: 600;
	}

	.readout-value {
		font-size: 1.2rem;
		font-weight: 600;
	}

	.size-lg .readout-value {
		font-size: 1.8rem;
	}

	.readout-unit {
		margin-left: 0.3rem;
		font-size: 0.75em;
		color: var(--muted);
		font-weight: 500;
	}
</style>
