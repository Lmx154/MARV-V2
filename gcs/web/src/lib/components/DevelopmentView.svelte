<script lang="ts">
	import WeatherPanel from './WeatherPanel.svelte';
	import { DEFAULT_ENV, compass, envError, launchMsg, launcherControls, windToward } from '$lib/gcs/sim';
	import type { Airframe, ClientMsg, SimEnv, SimStatus, SimTarget } from '$lib/gcs/types';

	interface Props {
		airframes: Airframe[] | null;
		airframesError: string | null;
		/** The selected airframe's id ('' for none). */
		selected: string;
		status: SimStatus | null;
		log: string[];
		/** The backend's refusal of the last sim_launch / sim_stop. */
		error: string | null;
		pending: boolean;
		wsOpen: boolean;
		onselect: (id: string) => void;
		onreload: () => void;
		onsend: (m: Extract<ClientMsg, { type: 'sim_launch' | 'sim_stop' }>) => void;
	}
	let { airframes, airframesError, selected, status, log, error, pending, wsOpen, onselect, onreload, onsend }: Props = $props();

	let env = $state<SimEnv>({ ...DEFAULT_ENV });
	/** Where the environment came from, until a field is edited by hand. */
	let provenance = $state<string | null>(null);
	let target = $state<SimTarget>('sitl');
	let gui = $state(true);
	let logEl = $state<HTMLPreElement | null>(null);

	const airframe = $derived(airframes?.find((a) => a.id === selected) ?? null);
	const why = $derived(envError(env));
	const ctl = $derived(launcherControls({ wsOpen, status, airframe: airframe !== null, envError: why, pending }));
	const toward = $derived(windToward(env.wind_speed_ms, env.wind_dir_deg));
	const reported = $derived(status?.env ? Object.entries(status.env) : []);
	const fmt = (v: number | null | undefined, d = 3): string => (typeof v === 'number' && Number.isFinite(v) ? (Math.abs(v) >= 1e-3 || v === 0 ? v.toFixed(d) : v.toExponential(3)) : '—');
	const shown = (v: unknown): string => (v === null || v === undefined ? '—' : typeof v === 'number' ? String(v) : typeof v === 'string' ? v : JSON.stringify(v));
	const started = $derived.by(() => {
		const t = status?.started_at;
		if (typeof t === 'number') return new Date(t < 1e12 ? t * 1000 : t).toLocaleTimeString();
		return t ?? '—';
	});

	$effect(() => {
		void log.length;
		if (logEl) logEl.scrollTop = logEl.scrollHeight;
	});

	function useWeather(patch: Partial<SimEnv>, from: string): void {
		env = { ...env, ...patch };
		provenance = from;
	}

	function launch(): void {
		if (!airframe || !ctl.launch) return;
		onsend(launchMsg(airframe.id, env, gui, target));
	}

	function stop(): void {
		if (!confirm('Stop the sim? The simulated vehicle and its flight software stop.')) return;
		onsend({ type: 'sim_stop' });
	}
</script>

<section class="dev mono" aria-label="Development">
	<section class="panel" aria-label="Airframes">
		<div class="head">
			<h2>Sim airframe</h2>
			<button type="button" class="btn sm" onclick={onreload}>Reload</button>
			{#if airframesError}<span class="err" role="alert">{airframesError}</span>{/if}
		</div>
		{#if airframes === null && !airframesError}
			<p class="hint">Loading the catalog…</p>
		{:else if airframes && airframes.length === 0}
			<p class="hint">The backend lists no airframes.</p>
		{:else if airframes}
			<div class="scroll">
				<table class="specs">
					<thead>
						<tr>
							<th></th><th>airframe</th><th>frame</th><th>mass kg</th><th>Ixx, Iyy, Izz kg m²</th><th>arm m</th><th>rotors</th><th>k N s²</th><th>k_m m</th>
							<th>ω max rad/s</th><th>τ up, down s</th><th>T max/rotor N</th><th>T/W</th><th>hover</th><th>source</th>
						</tr>
					</thead>
					<tbody>
						{#each airframes as a (a.id)}
							{@const s = a.specs}
							<tr class:on={a.id === selected} onclick={() => onselect(a.id)}>
								<td><input type="radio" name="sim-airframe" checked={a.id === selected} onchange={() => onselect(a.id)} aria-label="Select {a.label}" /></td>
								<td><b>{a.label}</b> <span class="k">{a.id}</span></td>
								<td>{a.frame || '—'}</td>
								<td>{fmt(s.mass_kg, 3)}</td>
								<td>{fmt(s.ixx, 4)}, {fmt(s.iyy, 4)}, {fmt(s.izz, 4)}</td>
								<td>{fmt(s.arm_m, 3)}</td>
								<td>{fmt(s.rotor_count, 0)}</td>
								<td>{fmt(s.motor_constant)}</td>
								<td>{fmt(s.moment_constant, 3)}</td>
								<td>{fmt(s.max_rot_velocity, 0)}</td>
								<td>{fmt(s.time_constant_up, 4)}, {fmt(s.time_constant_down, 4)}</td>
								<td>{fmt(s.t_max_n, 2)}</td>
								<td>{fmt(s.thrust_to_weight, 2)}</td>
								<td>{fmt(s.hover_thrust_frac, 3)}</td>
								<td class="k">{a.source || '—'}</td>
							</tr>
						{/each}
					</tbody>
				</table>
			</div>
		{/if}
	</section>

	<div class="cols">
		<section class="panel" aria-label="Environment">
			<h2>World environment</h2>
			<div class="grid">
				<label>wind speed <span><input type="number" min="0" step="0.1" bind:value={env.wind_speed_ms} oninput={() => (provenance = null)} /> m/s</span></label>
				<label>wind from <span><input type="number" min="0" max="360" step="1" bind:value={env.wind_dir_deg} oninput={() => (provenance = null)} /> deg ({compass(env.wind_dir_deg)})</span></label>
				<label>white-noise gusts σ <span><input type="number" min="0" step="0.05" bind:value={env.gust_sigma_ms} oninput={() => (provenance = null)} /> m/s</span></label>
				<label>latitude <span><input type="number" min="-90" max="90" step="any" bind:value={env.lat} oninput={() => (provenance = null)} /> deg</span></label>
				<label>longitude <span><input type="number" min="-180" max="180" step="any" bind:value={env.lon} oninput={() => (provenance = null)} /> deg</span></label>
				<label>elevation <span><input type="number" step="any" bind:value={env.elevation_m} oninput={() => (provenance = null)} /> m</span></label>
				<label>temperature <span><input type="number" step="any" placeholder="optional" bind:value={env.temperature_c} oninput={() => (provenance = null)} /> °C</span></label>
				<label>pressure <span><input type="number" step="any" placeholder="optional" bind:value={env.pressure_pa} oninput={() => (provenance = null)} /> Pa</span></label>
			</div>
			<p class="hint">
				Direction is where the wind blows FROM, clockwise from north: the air moves {fmt(toward.n, 1)} m/s north, {fmt(toward.e, 1)} m/s east. Gusts are
				Gazebo's wind noise, white (uncorrelated), not a Dryden or Gauss–Markov turbulence model. Wind shear with height is not supported by Gazebo: the
				wind is the same at every altitude. Temperature and pressure may be ignored by the backend; the running world below shows what it used.
			</p>
			<p class="hint">{provenance ? `From ${provenance}` : 'Entered by hand.'}</p>
			{#if why}<p class="err">{why}</p>{/if}
		</section>
		<WeatherPanel onapply={useWeather} />
	</div>

	<section class="panel" aria-label="Launcher">
		<h2>Launch</h2>
		<div class="controls">
			<label><input type="radio" name="sim-target" value="sitl" bind:group={target} /> SITL (flight software on this PC)</label>
			<label><input type="radio" name="sim-target" value="pico" bind:group={target} /> Pico-in-the-loop</label>
			<label><input type="checkbox" bind:checked={gui} /> open Gazebo window</label>
		</div>
		<div class="controls">
			<button type="button" class="btn go" disabled={!ctl.launch} onclick={launch} title={ctl.why ?? `Launch ${airframe?.label ?? ''}`}>LAUNCH</button>
			<button type="button" class="btn stop" disabled={!ctl.stop} onclick={stop}>STOP</button>
			{#if error}<span class="err" role="alert">{error}</span>{:else if ctl.why && !status?.running}<span class="hint">{ctl.why}</span>{/if}
		</div>
		<div class="status" aria-label="Sim status">
			<span><span class="k">sim</span> <b class:ok={status?.running}>{status ? (status.running ? 'running' : 'stopped') : '—'}</b></span>
			<span><span class="k">airframe</span> {status?.airframe ?? '—'}</span>
			<span><span class="k">target</span> {status?.target ?? '—'}</span>
			<span><span class="k">window</span> {status ? (status.gui ? 'yes' : 'no') : '—'}</span>
			<span><span class="k">pid</span> {status?.pid ?? '—'}</span>
			<span><span class="k">started</span> {started}</span>
		</div>
		{#if reported.length}
			<div class="status" aria-label="World as the backend reports it">
				<span class="k">world used</span>
				{#each reported as [k, v] (k)}<span><span class="k">{k}</span> {shown(v)}</span>{/each}
			</div>
		{/if}
		<pre class="log" bind:this={logEl}>{log.length ? log.join('\n') : 'no sim output yet'}</pre>
	</section>
</section>

<style>
	.dev {
		display: flex;
		flex-direction: column;
		gap: 0.75rem;
		font-size: 0.8rem;
	}
	.panel {
		display: flex;
		flex-direction: column;
		gap: 0.5rem;
		min-width: 0;
	}
	.panel h2 {
		margin: 0;
	}
	.head,
	.controls {
		display: flex;
		flex-wrap: wrap;
		align-items: center;
		gap: 0.5rem 1rem;
	}
	.cols {
		display: grid;
		grid-template-columns: minmax(0, 3fr) minmax(18rem, 2fr);
		gap: 1rem;
		align-items: start;
	}
	@media (max-width: 60rem) {
		.cols {
			grid-template-columns: minmax(0, 1fr);
		}
	}
	.scroll {
		overflow-x: auto;
		border: 1px solid var(--border);
		border-radius: 8px;
		background: var(--surface);
	}
	.specs {
		width: 100%;
		border-collapse: collapse;
		font-size: 0.75rem;
		font-variant-numeric: tabular-nums;
		white-space: nowrap;
	}
	.specs th,
	.specs td {
		padding: 0.25rem 0.45rem;
		border-bottom: 1px solid var(--border);
		text-align: left;
	}
	.specs th {
		color: var(--muted);
		font-weight: 600;
	}
	.specs tbody tr {
		cursor: pointer;
	}
	.specs tr.on {
		background: var(--accent-bg);
	}
	.grid {
		display: grid;
		grid-template-columns: repeat(auto-fill, minmax(15rem, 1fr));
		gap: 0.4rem 1rem;
	}
	.grid label {
		display: flex;
		justify-content: space-between;
		align-items: center;
		gap: 0.4rem;
		color: var(--muted);
	}
	.grid label span {
		color: var(--text);
	}
	input[type='number'] {
		background: var(--surface-2);
		color: var(--text);
		border: 1px solid var(--border);
		border-radius: 6px;
		padding: 0.2rem 0.4rem;
		font: inherit;
		width: 6.5rem;
	}
	.status {
		display: flex;
		flex-wrap: wrap;
		gap: 0.3rem 1.5rem;
		padding: 0.4rem 0.6rem;
		border: 1px solid var(--border);
		border-radius: 8px;
		background: var(--surface);
		font-variant-numeric: tabular-nums;
	}
	.k {
		color: var(--muted);
		margin-right: 0.3rem;
	}
	.ok {
		color: var(--ok);
	}
	.hint {
		margin: 0;
		color: var(--muted);
		max-width: none;
	}
	.err {
		margin: 0;
		color: var(--bad);
		max-width: none;
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
	.log {
		margin: 0;
		height: 14rem;
		overflow: auto;
		padding: 0.5rem 0.7rem;
		border: 1px solid var(--border);
		border-radius: 8px;
		background: var(--surface);
		font-size: 0.75rem;
		white-space: pre-wrap;
	}
</style>
