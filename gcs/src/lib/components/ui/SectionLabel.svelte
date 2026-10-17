<!-- Copied unchanged from the avionics toolbox, src/lib/components/ui/SectionLabel.svelte, toolbox commit 88658504d69757eda042961d7f02d8319e568e64 (MARV V2 ground segment, decision 0053 Q5). -->
<script lang="ts">
	let { text }: { text: string } = $props();
</script>

<span class="section-label mono">{text}</span>

<style>
	.section-label {
		display: block;
		font-size: 0.7rem;
		text-transform: uppercase;
		letter-spacing: 0.08em;
		color: var(--muted);
		font-weight: 600;
	}
</style>
