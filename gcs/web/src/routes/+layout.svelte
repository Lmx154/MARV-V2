<script lang="ts">
	import '../app.css';

	let { children } = $props();
</script>

<svelte:head><title>MARV ground control</title></svelte:head>

{@render children()}
