<script lang="ts">
	import '../app.css';
	import { onMount } from 'svelte';
	import { page } from '$app/state';
	import { app } from '$lib/state/app';
	import { ROUTES, routeOf } from '$lib/state/nav';
	import { readSetting, writeSetting } from '$lib/state/storage';
	import type { Problem } from '$lib/state/problems';
	import { readable } from 'svelte/store';
	import StatusBar from '$lib/components/StatusBar.svelte';
	import { simView } from '$lib/sim/instance';

	let { children } = $props();

	const gcs = typeof document === 'undefined' ? null : app();
	const problems = gcs?.problems ?? readable<Problem[]>([]);
	// The sim channel (GX-8) is separate from the vehicle's app: the layout only asks whether a simulation is publishing, to list the SIM page.
	const sim = typeof document === 'undefined' ? null : simView();
	const simPresent = sim?.present ?? readable(false);

	let theme = $state<'system' | 'light' | 'dark'>('system');
	let headerEl = $state<HTMLElement | null>(null);

	onMount(() => {
		void gcs?.connect();
		sim?.start();
		const stopShell = gcs?.startShell();
		const saved = readSetting('theme');
		if (saved === 'light' || saved === 'dark') theme = saved;
		// Pages with a sticky table header sit below this one: they read its height from --header-h.
		if (!headerEl) {
			return () => {
				stopShell?.();
				sim?.stop();
			};
		}
		const el = headerEl;
		const publish = (): void => document.documentElement.style.setProperty('--header-h', `${el.offsetHeight}px`);
		publish();
		const observer = new ResizeObserver(publish);
		observer.observe(el);
		return () => {
			observer.disconnect();
			stopShell?.();
			sim?.stop();
		};
	});

	function setTheme(next: 'system' | 'light' | 'dark'): void {
		theme = next;
		if (next === 'system') {
			writeSetting('theme', null);
			delete document.documentElement.dataset.theme;
		} else {
			writeSetting('theme', next);
			document.documentElement.dataset.theme = next;
		}
	}

	const current = $derived(routeOf(page.url.pathname)?.path ?? null);
</script>

<a class="skip" href="#main">Skip to the page</a>

<header bind:this={headerEl}>
	<div class="top">
		<span class="brand mono">MARV GCS</span>
		<nav aria-label="Pages">
			{#each ROUTES.filter((x) => !x.simOnly || $simPresent) as r (r.path)}
				<a href={r.path} aria-current={current === r.path ? 'page' : undefined} class:planned={r.unit !== null} title={r.unit ? `built by ${r.unit}` : undefined}>{r.label}</a>
			{/each}
		</nav>
		<div class="theme" role="group" aria-label="Theme">
			{#each ['system', 'light', 'dark'] as const as t (t)}
				<button type="button" class:on={theme === t} aria-pressed={theme === t} onclick={() => setTheme(t)}>{t}</button>
			{/each}
		</div>
	</div>
	<StatusBar />
</header>

<main id="main">
	{#each $problems as p (p.source)}
		<p class="error" role="alert"><span class="src mono">{p.source}</span> {p.message}</p>
	{/each}
	{@render children()}
</main>

<style>
	header {
		display: flex;
		flex-direction: column;
		gap: 0.4rem;
		padding: 0.5rem 1rem 0;
		background: var(--surface);
		border-bottom: 1px solid var(--border);
		position: sticky;
		top: 0;
		z-index: 5;
	}

	.top {
		display: flex;
		align-items: center;
		gap: 1.25rem;
		flex-wrap: wrap;
	}

	.brand {
		font-weight: 700;
		letter-spacing: 0.08em;
		color: var(--accent);
	}

	.theme {
		margin-left: auto;
		display: flex;
	}

	.theme button {
		border-radius: 0;
		padding: 0.05rem 0.5rem;
		font-size: 0.75rem;
	}

	.theme button:first-child {
		border-radius: 4px 0 0 4px;
	}

	.theme button:last-child {
		border-radius: 0 4px 4px 0;
	}

	.theme button.on {
		border-color: var(--accent);
		background: var(--accent-bg);
	}

	nav {
		display: flex;
		flex-wrap: wrap;
		gap: 0.1rem;
	}

	nav a {
		padding: 0.35rem 0.7rem;
		border-bottom: 2px solid transparent;
		color: var(--text);
		font-weight: 600;
	}

	nav a:hover {
		text-decoration: none;
		background: var(--surface-2);
	}

	nav a[aria-current='page'] {
		border-bottom-color: var(--accent);
		color: var(--accent);
	}

	nav a.planned:not([aria-current='page']) {
		color: var(--muted);
		font-weight: 400;
	}

	main {
		padding: 0.75rem 1rem 2rem;
		max-width: 110rem;
	}

	.error {
		margin: 0 0 0.4rem;
		padding: 0.4rem 0.6rem;
		border: 1px solid var(--bad);
		border-radius: 4px;
		color: var(--bad);
		max-width: none;
	}

	.error .src {
		text-transform: uppercase;
		font-size: 0.65rem;
		letter-spacing: 0.06em;
		margin-right: 0.4rem;
	}

	.skip {
		position: absolute;
		left: -999px;
	}

	.skip:focus {
		left: 0.5rem;
		top: 0.5rem;
		z-index: 10;
		background: var(--surface);
		padding: 0.3rem;
	}
</style>
