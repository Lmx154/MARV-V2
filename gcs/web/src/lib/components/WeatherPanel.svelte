<script lang="ts">
	import { browser } from '$app/environment';
	import { fetchWeather, geocode, placeLabel, simEnvFromWeather, type Place, type Weather } from '$lib/gcs/open-meteo';
	import type { SimEnv } from '$lib/gcs/types';

	let { onapply }: { onapply: (env: Partial<SimEnv>, provenance: string) => void } = $props();

	let query = $state('');
	let candidates = $state<Place[] | null>(null);
	let picked = $state<Place | null>(null);
	let weather = $state<Weather | null>(null);
	let busy = $state<'search' | 'weather' | null>(null);
	let error = $state<string | null>(null);
	let request = 0;

	const uid = $props.id();
	const derived = $derived(weather && picked ? simEnvFromWeather(weather, picked) : null);
	const fmt = (v: number | undefined | null, digits: number): string => (v === undefined || v === null || !Number.isFinite(v) ? '—' : v.toFixed(digits));
	const message = (e: unknown): string => (e instanceof Error ? e.message : String(e));

	async function lookUp(e: SubmitEvent): Promise<void> {
		e.preventDefault();
		if (!browser || !query.trim()) return;
		const id = ++request;
		busy = 'search';
		error = null;
		candidates = null;
		picked = null;
		weather = null;
		try {
			const found = await geocode(query);
			if (id !== request) return;
			candidates = found;
		} catch (err) {
			if (id === request) error = `Look-up failed: ${message(err)}`;
		} finally {
			if (id === request) busy = null;
		}
	}

	async function pick(place: Place): Promise<void> {
		if (!browser) return;
		const id = ++request;
		picked = place;
		weather = null;
		busy = 'weather';
		error = null;
		try {
			const w = await fetchWeather(place.latitude, place.longitude);
			if (id === request) weather = w;
		} catch (err) {
			if (id === request) error = `Weather request failed: ${message(err)}`;
		} finally {
			if (id === request) busy = null;
		}
	}
</script>

<section class="weather" aria-label="Local weather">
	<h3>Use local weather</h3>
	<form class="search" onsubmit={lookUp}>
		<label class="mono" for={`${uid}-q`}>Place</label>
		<input id={`${uid}-q`} type="text" bind:value={query} placeholder="e.g. Austin" autocomplete="off" />
		<button type="submit" class="btn mono" disabled={busy !== null || !query.trim()}>Look up</button>
	</form>

	{#if busy === 'search'}<p class="status mono" role="status">Looking up “{query.trim()}”…</p>{/if}
	{#if error}<p class="error mono" role="alert">{error}</p>{/if}

	{#if candidates}
		{#if candidates.length === 0}
			<p class="status mono">No place found.</p>
		{:else}
			<ul class="candidates">
				{#each candidates as c, i (i)}
					<li>
						<button type="button" class="candidate" class:active={picked === c} disabled={busy === 'weather'} onclick={() => pick(c)}>
							{placeLabel(c)} <span class="coords mono">{c.latitude.toFixed(2)}, {c.longitude.toFixed(2)}{Number.isFinite(c.elevation) ? ` · ${Math.round(c.elevation)} m` : ''}</span>
						</button>
					</li>
				{/each}
			</ul>
		{/if}
	{/if}

	{#if busy === 'weather' && picked}<p class="status mono" role="status">Fetching the weather at {placeLabel(picked)}…</p>{/if}

	{#if weather && derived}
		<table class="readings mono">
			<tbody>
				<tr><th scope="row">time</th><td>{weather.time.replace('T', ' ')} {weather.timezone}</td></tr>
				<tr><th scope="row">wind 10 m</th><td>{fmt(weather.windSpeed10, 1)} m/s from {fmt(weather.windDir10, 0)}°</td></tr>
				<tr><th scope="row">wind 80 / 120 / 180 m</th><td>{fmt(weather.wind80, 1)} / {fmt(weather.wind120, 1)} / {fmt(weather.wind180, 1)} m/s (not simulated)</td></tr>
				<tr><th scope="row">gusts 10 m</th><td>{fmt(weather.gust10, 1)} m/s</td></tr>
				<tr><th scope="row">temperature 2 m</th><td>{fmt(weather.temperature2m, 1)} °C</td></tr>
				<tr><th scope="row">surface pressure</th><td>{fmt(weather.surfacePressure, 1)} hPa at {fmt(weather.elevation, 0)} m</td></tr>
			</tbody>
		</table>
		<table class="readings derived mono">
			<tbody>
				<tr><th scope="row">mean wind</th><td>{fmt(derived.env.wind_speed_ms, 1)} m/s from {fmt(derived.env.wind_dir_deg, 0)}°</td></tr>
				<tr><th scope="row">gust σ = (gust − mean)/3</th><td>{fmt(derived.env.gust_sigma_ms, 2)} m/s</td></tr>
				<tr><th scope="row">origin</th><td>{fmt(derived.env.lat, 4)}, {fmt(derived.env.lon, 4)}, {fmt(derived.env.elevation_m, 0)} m</td></tr>
				<tr><th scope="row">temperature, pressure</th><td>{fmt(derived.env.temperature_c, 1)} °C, {fmt(derived.env.pressure_pa, 0)} Pa</td></tr>
			</tbody>
		</table>
		<button type="button" class="btn mono apply" onclick={() => derived && onapply(derived.env, derived.provenance)}>Use for the sim world</button>
	{/if}

	<p class="attribution">
		<a href="https://open-meteo.com/" target="_blank" rel="noopener">Weather data by Open-Meteo.com</a>
		(<a href="https://creativecommons.org/licenses/by/4.0/" target="_blank" rel="noopener">CC BY 4.0</a>)
	</p>
</section>

<style>
	.weather {
		display: flex;
		flex-direction: column;
		gap: 0.4rem;
		padding: 0.5rem 0.7rem;
		border: 1px dashed var(--border-strong);
		border-radius: 8px;
		background: var(--surface);
		font-size: 0.78rem;
		min-width: 0;
	}
	h3 {
		margin: 0;
		font-size: 0.85rem;
	}
	.search {
		display: flex;
		align-items: center;
		gap: 0.4rem;
		flex-wrap: wrap;
	}
	.search label {
		font-size: 0.72rem;
		color: var(--muted);
	}
	.search input {
		flex: 1 1 7rem;
		min-width: 0;
		background: var(--surface-2);
		color: var(--text);
		border: 1px solid var(--border);
		border-radius: 6px;
		padding: 0.2rem 0.4rem;
		font: inherit;
	}
	.btn {
		background: var(--surface-2);
		color: var(--text);
		border: 1px solid var(--border);
		border-radius: 6px;
		padding: 0.2rem 0.55rem;
		font-size: 0.72rem;
		cursor: pointer;
	}
	.btn:disabled {
		opacity: 0.5;
		cursor: default;
	}
	.apply {
		align-self: flex-start;
		border-color: var(--accent);
		color: var(--accent);
	}
	.status {
		margin: 0;
		color: var(--muted);
		font-size: 0.72rem;
	}
	.error {
		margin: 0;
		color: var(--bad);
		font-size: 0.72rem;
	}
	.candidates {
		list-style: none;
		margin: 0;
		padding: 0;
		display: flex;
		flex-direction: column;
		gap: 0.2rem;
	}
	.candidate {
		width: 100%;
		text-align: left;
		background: transparent;
		color: var(--text);
		border: 1px solid var(--border);
		border-radius: 6px;
		padding: 0.2rem 0.4rem;
		font: inherit;
		cursor: pointer;
	}
	.candidate.active {
		border-color: var(--accent);
		background: var(--accent-bg);
	}
	.coords {
		color: var(--muted);
		font-size: 0.68rem;
	}
	.readings {
		width: 100%;
		border-collapse: collapse;
		font-size: 0.72rem;
		font-variant-numeric: tabular-nums;
	}
	.readings th,
	.readings td {
		padding: 0.12rem 0.3rem;
		border-bottom: 1px solid var(--border);
		text-align: left;
		vertical-align: top;
	}
	.readings th {
		color: var(--muted);
		font-weight: 600;
	}
	.derived td {
		color: var(--accent);
	}
	.attribution {
		margin: 0;
		font-size: 0.68rem;
		color: var(--muted);
	}
	.attribution a {
		color: inherit;
	}
</style>
