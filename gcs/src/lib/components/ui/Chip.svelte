<!-- Copied unchanged from the avionics toolbox, src/lib/components/ui/Chip.svelte, toolbox commit 88658504d69757eda042961d7f02d8319e568e64 (MARV V2 ground segment, decision 0053 Q5). -->
<script lang="ts">
	let {
		label,
		color,
		active = false,
		href,
		onclick
	}: {
		label: string;
		color?: string;
		active?: boolean;
		href?: string;
		onclick?: () => void;
	} = $props();

	const style = $derived(color ? `--chip-color: var(${color})` : undefined);
</script>

{#if href}
	<a class="chip mono" class:active {style} {href}>
		{#if color}<span class="dot" aria-hidden="true"></span>{/if}
		{label}
	</a>
{:else}
	<button type="button" class="chip mono" class:active {style} {onclick}>
		{#if color}<span class="dot" aria-hidden="true"></span>{/if}
		{label}
	</button>
{/if}

<style>
	.chip {
		display: inline-flex;
		align-items: center;
		gap: 0.4rem;
		padding: 0.25rem 0.6rem;
		border: 1px solid var(--border);
		border-radius: 999px;
		background: transparent;
		color: var(--text);
		font-size: 0.75rem;
		line-height: 1;
		cursor: pointer;
	}

	.chip:hover {
		border-color: var(--border-strong);
		text-decoration: none;
	}

	.chip.active {
		border-color: var(--accent);
		background: var(--accent-bg);
	}

	.dot {
		width: 0.5rem;
		height: 0.5rem;
		border-radius: 50%;
		background: var(--chip-color, var(--muted));
	}
</style>
