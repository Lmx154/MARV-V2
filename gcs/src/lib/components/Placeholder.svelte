<script lang="ts">
	import { ROUTES } from '$lib/state/nav';

	let { path }: { path: string } = $props();

	const route = $derived(ROUTES.find((r) => r.path === path) ?? null);
</script>

<svelte:head><title>{route?.label ?? 'Page'} · MARV GCS</title></svelte:head>

{#if route}
	<h1>{route.label}</h1>
	<p class="muted" data-placeholder={route.path}>Not built yet: {route.unit} builds this page. It will hold: {route.will}</p>
{/if}
