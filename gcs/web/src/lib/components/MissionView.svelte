<script lang="ts">
	import { onMount } from 'svelte';
	import MissionMap from './MissionMap.svelte';
	import { LocalFrame, fromE7 } from '$lib/gcs/geo';
	import {
		ALT_MAX_M,
		ALT_MIN_M,
		MAX_RANGE_M,
		MAX_WAYPOINTS,
		climbError,
		disarmNeedsConfirm,
		enabledControls,
		exportPresets,
		importPresets,
		missionError,
		moveWaypoint,
		parseWaypoint,
		propsSpinning,
		readPresets,
		reasonText,
		upsertPreset,
		waypointError,
		writePresets,
		type MissionMsg,
		type MissionPreset,
		type MissionRequest
	} from '$lib/gcs/mission';
	import { euler } from '$lib/gcs/protocol';
	import type { LatLonAlt, MissionStatus, Telemetry } from '$lib/gcs/types';

	interface Props {
		telem: Telemetry | null;
		mission: MissionStatus | null;
		/** Backend open and FC connected. */
		connected: boolean;
		errors: Partial<Record<MissionRequest, string>>;
		visible: boolean;
		onsend: (m: MissionMsg) => void;
	}
	let { telem, mission, connected, errors, visible, onsend }: Props = $props();

	const DEG = 180 / Math.PI;

	let waypoints = $state<LatLonAlt[]>([]);
	let newLat = $state('');
	let newLon = $state('');
	let newAlt = $state('20');
	let addError = $state<string | null>(null);
	let climbAlt = $state(10);
	let presets = $state<MissionPreset[]>([]);
	let presetName = $state('');
	let note = $state<string | null>(null);

	const storage = (): Storage | null => {
		try {
			return window.localStorage;
		} catch {
			return null;
		}
	};
	onMount(() => (presets = readPresets(storage())));

	const home = $derived(telem?.home ?? null);
	/** The executor's home once armed, else the FC's. */
	const homeLL = $derived(mission?.home ? { ...mission.home, alt_m: 0 } : home ? { lat: fromE7(home.lat_e7), lon: fromE7(home.lon_e7), alt_m: 0 } : null);
	/** The backend's geo, else the estimate's NED about home through the FC's frame. */
	const position = $derived.by((): LatLonAlt | null => {
		if (!telem) return null;
		if (telem.geo) return telem.geo;
		if (!home || !telem.p_ned.every(Number.isFinite)) return null;
		return new LocalFrame(home).latLonOf(telem.p_ned);
	});
	const altitude = $derived(position && Number.isFinite(position.alt_m) ? position.alt_m : telem ? -telem.p_ned[2] : NaN);
	const vehicle = $derived(position ? { lat: position.lat, lon: position.lon, yaw_deg: telem ? euler(telem.q).yaw * DEG : 0 } : null);
	const mode = $derived(mission?.state ?? null);
	const on = $derived(enabledControls({ state: mode, connected, climbAlt, waypoints, home: homeLL }));
	const startWhy = $derived(mode !== 'hold' ? 'enabled in hold, after the climb' : missionError(waypoints, homeLL));
	const reason = $derived(reasonText(mission));
	const spinning = $derived(propsSpinning(telem));
	const full = $derived(waypoints.length >= MAX_WAYPOINTS);

	const fmt = (v: number | null | undefined, d = 1): string => (v !== null && v !== undefined && Number.isFinite(v) ? v.toFixed(d) : '—');

	function addTyped(): void {
		if (full) return void (addError = `at most ${MAX_WAYPOINTS} waypoints`);
		const w = parseWaypoint(newLat, newLon, newAlt, homeLL);
		if (typeof w === 'string') {
			addError = w;
			return;
		}
		addError = null;
		waypoints.push(w);
		newLat = '';
		newLon = '';
	}

	function addPicked(lat: number, lon: number): void {
		if (full) return void (addError = `at most ${MAX_WAYPOINTS} waypoints`);
		const w = parseWaypoint(lat.toFixed(7), lon.toFixed(7), newAlt, homeLL);
		if (typeof w === 'string') {
			addError = w;
			return;
		}
		addError = null;
		waypoints.push(w);
	}

	function edit(i: number, key: keyof LatLonAlt, v: string): void {
		waypoints[i][key] = v.trim() === '' ? NaN : Number(v);
	}

	function disarm(): void {
		if (disarmNeedsConfirm(mode, altitude) && !confirm(`Disarm now? The vehicle is ${mode ?? 'in an unknown state'} at ${fmt(altitude)} m: the motors stop and it falls.`)) return;
		onsend({ type: 'disarm' });
	}

	function savePreset(): void {
		const name = presetName.trim();
		if (!name) return void (note = 'Preset: give it a name.');
		const why = missionError(waypoints);
		if (why) return void (note = `Preset not saved: ${why}.`);
		presets = upsertPreset(presets, { name, waypoints: waypoints.map((w) => ({ ...w })) });
		const err = writePresets(storage(), presets);
		note = err ? `Preset ${name} kept for this session only: ${err}.` : `Preset ${name} saved in this browser.`;
	}

	function loadPreset(name: string): void {
		const p = presets.find((x) => x.name === name);
		if (!p) return;
		waypoints = p.waypoints.map((w) => ({ ...w }));
		presetName = p.name;
		note = `Preset ${p.name} loaded: ${p.waypoints.length} waypoints.`;
	}

	function deletePreset(): void {
		const name = presetName.trim();
		if (!presets.some((p) => p.name === name) || !confirm(`Delete preset ${name}?`)) return;
		presets = presets.filter((p) => p.name !== name);
		const err = writePresets(storage(), presets);
		note = err ? `Preset ${name} deleted for this session only: ${err}.` : `Preset ${name} deleted.`;
	}

	function exportJson(): void {
		const url = URL.createObjectURL(new Blob([JSON.stringify(exportPresets(presets), null, '\t') + '\n'], { type: 'application/json' }));
		const a = document.createElement('a');
		a.href = url;
		a.download = 'marv-mission-presets.json';
		a.click();
		URL.revokeObjectURL(url);
	}

	async function importJson(e: Event): Promise<void> {
		const input = e.currentTarget as HTMLInputElement;
		const f = input.files?.[0];
		input.value = '';
		if (!f) return;
		let parsed: unknown;
		try {
			parsed = JSON.parse(await f.text());
		} catch {
			note = `Import: ${f.name} is not JSON.`;
			return;
		}
		const r = importPresets(parsed);
		for (const p of r.presets) presets = upsertPreset(presets, p);
		const err = writePresets(storage(), presets);
		note = `Import ${f.name}: ${r.presets.length} presets` + (r.rejected.length ? `; invalid, not imported: ${r.rejected.join(', ')}` : '') + (err ? `; not stored: ${err}` : '') + '.';
	}
</script>

<section class="mission" aria-label="Mission">
	<div class="status mono" aria-label="Mission status">
		<span><span class="k">state</span> <b>{mode ?? '—'}</b></span>
		{#if reason}<span><span class="k">reason</span> {reason}</span>{/if}
		<span><span class="k">waypoint</span> {mission && mission.wp_count ? `${mission.wp_index >= 0 ? mission.wp_index + 1 : '—'}/${mission.wp_count}` : '—'}</span>
		<span><span class="k">to target</span> {fmt(mission?.dist_m)} m</span>
		<span><span class="k">altitude</span> {fmt(altitude)} m</span>
		<span><span class="k">armed</span> {telem ? (telem.armed ? 'yes' : 'no') : '—'}</span>
		{#if spinning && telem?.motor}<span class="warn" role="status">props spinning <span class="k">motor</span> {telem.motor.map((v) => fmt(v, 2)).join(' ')}</span>{/if}
		{#if mission?.climb_alt_m != null}<span><span class="k">safe alt</span> {fmt(mission.climb_alt_m)} m</span>{/if}
		{#if !home}<span class="warn">no home from the FC</span>{/if}
	</div>

	<div class="controls mono" aria-label="Mission controls">
		<div class="ctl">
			<button type="button" class="btn go" disabled={!on.arm} onclick={() => onsend({ type: 'arm' })} title="Props spin at idle; no takeoff">ARM</button>
			{#if errors.arm}<span class="err" role="alert">{errors.arm}</span>{/if}
		</div>
		<div class="ctl">
			<button type="button" class="btn stop" disabled={!on.disarm} onclick={disarm}>DISARM</button>
			{#if errors.disarm}<span class="err" role="alert">{errors.disarm}</span>{/if}
		</div>
		<div class="ctl">
			<button type="button" class="btn" disabled={!on.climb} onclick={() => onsend({ type: 'climb', alt_m: climbAlt })} title="Take off and climb to the safe altitude, then hold">CLIMB</button>
			<label>to <input type="number" min={ALT_MIN_M} max={ALT_MAX_M} step="any" bind:value={climbAlt} class:bad={climbError(climbAlt) !== null} aria-label="Safe altitude (m above home)" /> m</label>
			{#if climbError(climbAlt)}<span class="err">{climbError(climbAlt)}</span>{/if}
			{#if errors.climb}<span class="err" role="alert">{errors.climb}</span>{/if}
		</div>
		<div class="ctl">
			<button type="button" class="btn" disabled={!on.mission_start} onclick={() => onsend({ type: 'mission_start', waypoints: waypoints.map((w) => ({ ...w })) })} title={startWhy ?? 'Fly the waypoints, then return and hold over home'}>START MISSION</button>
			{#if errors.mission_start}<span class="err" role="alert">{errors.mission_start}</span>{/if}
		</div>
		<div class="ctl">
			<button type="button" class="btn" disabled={!on.rth} onclick={() => onsend({ type: 'rth' })}>RETURN TO HOME</button>
			{#if errors.rth}<span class="err" role="alert">{errors.rth}</span>{/if}
		</div>
		<div class="ctl">
			<button type="button" class="btn" disabled={!on.land} onclick={() => onsend({ type: 'land' })}>LAND</button>
			{#if errors.land}<span class="err" role="alert">{errors.land}</span>{/if}
		</div>
	</div>

	<div class="body">
		<MissionMap
			home={homeLL}
			{vehicle}
			{waypoints}
			target={mission?.target ?? null}
			active={mode === 'mission' ? (mission?.wp_index ?? -1) : -1}
			{visible}
			onpick={addPicked}
		/>

		<div class="editor mono">
			<h2>Waypoints</h2>
			<p class="hint">
				Click the map, or type decimal degrees. Altitude is {ALT_MIN_M}..{ALT_MAX_M} m above home; up to {MAX_WAYPOINTS} waypoints, each within {MAX_RANGE_M / 1000} km of home. The route starts at home and
				ends holding over it.
			</p>
			<div class="add">
				<input placeholder="lat" bind:value={newLat} aria-label="New waypoint latitude" />
				<input placeholder="lon" bind:value={newLon} aria-label="New waypoint longitude" />
				<input placeholder="alt" bind:value={newAlt} aria-label="New waypoint altitude (m above home)" class="alt" />
				<button type="button" class="btn" disabled={full} onclick={addTyped}>Add</button>
			</div>
			{#if addError}<p class="err" role="alert">{addError}</p>{/if}

			<ol class="rows">
				{#each waypoints as w, i (i)}
					{@const why = waypointError(w, homeLL)}
					<li class:active={mode === 'mission' && mission?.wp_index === i}>
						<span class="n">{i + 1}</span>
						<input type="number" step="any" value={w.lat} onchange={(e) => edit(i, 'lat', e.currentTarget.value)} aria-label="Latitude {i + 1}" />
						<input type="number" step="any" value={w.lon} onchange={(e) => edit(i, 'lon', e.currentTarget.value)} aria-label="Longitude {i + 1}" />
						<input type="number" step="any" value={w.alt_m} onchange={(e) => edit(i, 'alt_m', e.currentTarget.value)} aria-label="Altitude {i + 1}" class="alt" />
						<button type="button" class="btn sm" disabled={i === 0} onclick={() => (waypoints = moveWaypoint(waypoints, i, -1))} aria-label="Move up">↑</button>
						<button type="button" class="btn sm" disabled={i === waypoints.length - 1} onclick={() => (waypoints = moveWaypoint(waypoints, i, 1))} aria-label="Move down">↓</button>
						<button type="button" class="btn sm" onclick={() => waypoints.splice(i, 1)} aria-label="Delete">✕</button>
						{#if why}<span class="err row-err">{why}</span>{/if}
					</li>
				{/each}
			</ol>
			{#if waypoints.length}<button type="button" class="btn sm" onclick={() => (waypoints = [])}>Clear all</button>{/if}

			<h2>Presets</h2>
			<div class="add">
				<input placeholder="preset name" bind:value={presetName} aria-label="Preset name" list="mission-presets" />
				<datalist id="mission-presets">{#each presets as p (p.name)}<option value={p.name}></option>{/each}</datalist>
				<button type="button" class="btn" onclick={savePreset}>Save</button>
				<button type="button" class="btn" disabled={!presets.some((p) => p.name === presetName.trim())} onclick={deletePreset}>Delete</button>
			</div>
			<div class="add">
				<select value="" onchange={(e) => loadPreset(e.currentTarget.value)} aria-label="Load preset">
					<option value="" disabled>load a preset…</option>
					{#each presets as p (p.name)}<option value={p.name}>{p.name} ({p.waypoints.length})</option>{/each}
				</select>
				<button type="button" class="btn" disabled={!presets.length} onclick={exportJson}>Export JSON</button>
				<label class="btn">Import JSON<input type="file" accept="application/json,.json" hidden onchange={importJson} /></label>
			</div>
			{#if note}<p class="hint" role="status">{note}</p>{/if}
		</div>
	</div>
</section>

<style>
	.mission {
		display: flex;
		flex-direction: column;
		gap: 0.75rem;
	}
	.status {
		display: flex;
		flex-wrap: wrap;
		gap: 0.3rem 1.5rem;
		padding: 0.4rem 0.6rem;
		border: 1px solid var(--border);
		border-radius: 8px;
		background: var(--surface);
		font-size: 0.8rem;
		font-variant-numeric: tabular-nums;
	}
	.k {
		color: var(--muted);
		margin-right: 0.3rem;
	}
	.warn {
		color: var(--warn);
	}
	.controls {
		display: flex;
		flex-wrap: wrap;
		gap: 0.5rem 1rem;
		font-size: 0.8rem;
	}
	.ctl {
		display: flex;
		align-items: center;
		gap: 0.4rem;
	}
	.body {
		display: grid;
		grid-template-columns: minmax(0, 3fr) minmax(18rem, 2fr);
		gap: 1rem;
	}
	@media (max-width: 60rem) {
		.body {
			grid-template-columns: minmax(0, 1fr);
		}
	}
	.editor {
		display: flex;
		flex-direction: column;
		gap: 0.5rem;
		font-size: 0.8rem;
		min-width: 0;
	}
	.editor h2 {
		margin: 0.3rem 0 0;
	}
	.hint {
		margin: 0;
		color: var(--muted);
		max-width: none;
	}
	.add {
		display: flex;
		flex-wrap: wrap;
		gap: 0.4rem;
		align-items: center;
	}
	input,
	select {
		background: var(--surface-2);
		color: var(--text);
		border: 1px solid var(--border);
		border-radius: 6px;
		padding: 0.2rem 0.4rem;
		font: inherit;
		width: 8.5rem;
	}
	input.alt,
	.ctl input {
		width: 4.5rem;
	}
	input.bad {
		border-color: var(--bad);
	}
	.rows {
		list-style: none;
		margin: 0;
		padding: 0;
		display: flex;
		flex-direction: column;
		gap: 0.3rem;
	}
	.rows li {
		display: flex;
		flex-wrap: wrap;
		align-items: center;
		gap: 0.3rem;
		padding: 0.2rem;
		border-radius: 6px;
	}
	.rows li.active {
		background: var(--accent-bg);
	}
	.n {
		width: 1.5rem;
		text-align: right;
		color: var(--muted);
	}
	.row-err {
		flex-basis: 100%;
		padding-left: 1.8rem;
	}
	.btn {
		background: var(--surface-2);
		color: var(--text);
		border: 1px solid var(--border);
		border-radius: 6px;
		padding: 0.35rem 0.7rem;
		font-size: 0.8rem;
		cursor: pointer;
	}
	.btn:hover {
		border-color: var(--border-strong);
	}
	.btn:disabled {
		opacity: 0.5;
		cursor: default;
	}
	.btn.sm {
		padding: 0.15rem 0.45rem;
	}
	.btn.go:not(:disabled) {
		border-color: var(--ok);
		color: var(--ok);
	}
	.btn.stop:not(:disabled) {
		border-color: var(--bad);
		color: var(--bad);
	}
	.err {
		margin: 0;
		color: var(--bad);
		max-width: none;
	}
</style>
