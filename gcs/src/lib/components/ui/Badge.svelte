<!-- Copied unchanged from the avionics toolbox, src/lib/components/ui/Badge.svelte, toolbox commit 88658504d69757eda042961d7f02d8319e568e64 (MARV V2 ground segment, decision 0053 Q5). -->
<script lang="ts">
	let {
		text,
		tone = 'neutral'
	}: { text: string; tone?: 'neutral' | 'ok' | 'warn' | 'bad' } = $props();
</script>

<span class="badge mono tone-{tone}">{text}</span>

<style>
	.badge {
		display: inline-block;
		padding: 0.1rem 0.45rem;
		border: 1px solid var(--border-strong);
		border-radius: 3px;
		font-size: 0.7rem;
		text-transform: uppercase;
		letter-spacing: 0.05em;
		color: var(--muted);
	}

	.tone-ok {
		border-color: var(--ok);
		color: var(--ok);
	}

	.tone-warn {
		border-color: var(--warn);
		color: var(--warn);
	}

	.tone-bad {
		border-color: var(--bad);
		color: var(--bad);
	}
</style>
