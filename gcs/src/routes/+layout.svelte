<script lang="ts">
	import '../app.css';
	import { onMount } from 'svelte';
	import { page } from '$app/state';
	import { app } from '$lib/state/app';
	import type { SessionState } from '$link/session';
	import type { Problem } from '$lib/state/problems';
	import type { VehicleState } from '$link/vehicle';
	import { readable } from 'svelte/store';
	import Badge from '$lib/components/ui/Badge.svelte';
	import IdentityStrip from '$lib/components/IdentityStrip.svelte';

	let { children } = $props();

	const gcs = typeof document === 'undefined' ? null : app();
	const none = readable<string | null>(null);
	const helperState = gcs?.helper?.state ?? readable('closed');
	const device = gcs?.device ?? none;
	const problems = gcs?.problems ?? readable<Problem[]>([]);
	const sessionState = gcs?.session?.state ?? readable<SessionState | null>(null);
	const vehicleState = gcs?.vehicle?.state ?? readable<VehicleState | null>(null);

	let theme = $state<'system' | 'light' | 'dark'>('system');
	let headerEl = $state<HTMLElement | null>(null);

	onMount(() => {
		const saved = localStorage.theme;
		if (saved === 'light' || saved === 'dark') theme = saved;
		void gcs?.connect();
		// Pages with a sticky table header sit below this one: they read its height from --header-h.
		if (!headerEl) return;
		const el = headerEl;
		const publish = (): void => document.documentElement.style.setProperty('--header-h', `${el.offsetHeight}px`);
		publish();
		const observer = new ResizeObserver(publish);
		observer.observe(el);
		return () => observer.disconnect();
	});

	function setTheme(next: 'system' | 'light' | 'dark'): void {
		theme = next;
		if (next === 'system') {
			localStorage.removeItem('theme');
			delete document.documentElement.dataset.theme;
		} else {
			localStorage.theme = next;
			document.documentElement.dataset.theme = next;
		}
	}

	const armed = $derived($sessionState?.armed);
	const heartbeat = $derived($sessionState?.heartbeat ?? 'no heartbeat');
	const pathname = $derived(page.url.pathname);
</script>

<a class="skip" href="#main">Skip to the page</a>

<header bind:this={headerEl}>
	<div class="top">
		<span class="brand mono">MARV GCS</span>
		<div class="states" role="status" aria-label="Connection state">
			<span class="item"><span class="k">helper</span> <Badge text={$helperState} tone={$helperState === 'open' ? 'ok' : 'bad'} /></span>
			<span class="item"><span class="k">device</span> <span class="mono">{$device ?? 'none'}</span></span>
			<span class="item"><span class="k">vehicle</span> <Badge text={heartbeat} tone={heartbeat === 'alive' ? 'ok' : 'warn'} /></span>
			<span class="item">
				<span class="k">arming</span>
				{#if armed === true}<Badge text="armed" tone="bad" />{:else if armed === false}<Badge text="disarmed" tone="ok" />{:else}<Badge text="unknown" />{/if}
			</span>
		</div>
		<div class="theme" role="group" aria-label="Theme">
			{#each ['system', 'light', 'dark'] as const as t (t)}
				<button type="button" class:on={theme === t} aria-pressed={theme === t} onclick={() => setTheme(t)}>{t}</button>
			{/each}
		</div>
	</div>
	<IdentityStrip identity={$vehicleState?.identity ?? null} />
	<nav aria-label="Pages">
		<a href="/" aria-current={pathname === '/' ? 'page' : undefined}>Connection</a>
		<a href="/params" aria-current={pathname === '/params' ? 'page' : undefined}>Parameters</a>
		<span class="soon" aria-disabled="true" title="coming in a later unit">Plots <small>coming in a later unit</small></span>
		<span class="soon" aria-disabled="true" title="coming in a later unit">Flash <small>coming in a later unit</small></span>
	</nav>
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

	.states {
		display: flex;
		gap: 1rem;
		flex-wrap: wrap;
		align-items: center;
		font-size: 0.8rem;
	}

	.k {
		color: var(--muted);
		text-transform: uppercase;
		letter-spacing: 0.06em;
		font-size: 0.65rem;
		margin-right: 0.2rem;
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
		gap: 0.25rem;
		margin-top: 0.2rem;
	}

	nav a,
	nav .soon {
		padding: 0.35rem 0.8rem;
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

	.soon {
		color: var(--muted);
		cursor: not-allowed;
	}

	.soon small {
		font-weight: 400;
		font-size: 0.7rem;
		margin-left: 0.3rem;
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
