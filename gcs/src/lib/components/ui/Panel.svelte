<!-- Copied unchanged from the avionics toolbox, src/lib/components/ui/Panel.svelte, toolbox commit 88658504d69757eda042961d7f02d8319e568e64 (MARV V2 ground segment, decision 0053 Q5). -->
<script lang="ts">
	import type { Snippet } from 'svelte';
	import SectionLabel from './SectionLabel.svelte';

	let {
		title,
		label,
		children
	}: { title?: string; label?: string; children: Snippet } = $props();
</script>

<div class="panel">
	{#if label}<SectionLabel text={label} />{/if}
	{#if title}<h3 class="panel-title">{title}</h3>{/if}
	<div class="panel-body">
		{@render children()}
	</div>
</div>

<style>
	.panel {
		position: relative;
		background: var(--surface);
		border: 1px solid var(--border);
		border-radius: 6px;
		padding: 1rem;
		isolation: isolate;
	}

	.panel::before {
		content: '';
		position: absolute;
		inset: 0;
		border-radius: inherit;
		pointer-events: none;
		z-index: -1;
		background-image:
			linear-gradient(var(--border) 1px, transparent 1px),
			linear-gradient(90deg, var(--border) 1px, transparent 1px);
		background-size: 16px 16px;
		opacity: 0.05;
	}

	.panel-title {
		margin: 0.15rem 0 0.75rem;
	}

	.panel-body > :global(*:first-child) {
		margin-top: 0;
	}
</style>
