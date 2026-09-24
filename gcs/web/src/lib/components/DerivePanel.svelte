<script lang="ts">
	import { X3_LOOP_GAINS, blankInputs, derive, gainsError, inputsError, prefillInputs, squareX, stagePlan, type DeriveInputs, type LoopGains } from '$lib/gcs/derive';
	import type { Airframe, Schema } from '$lib/gcs/types';

	interface Props {
		schema: Schema;
		value: (index: number) => number;
		/** Staging refused (no FC, schema mismatch). */
		disabled: boolean;
		/** The airframe selected on the Development tab, offered for a prefill; never applied by itself. */
		airframe: Airframe | null;
		onparam: (index: number, value: number) => void;
	}
	let { schema, value, disabled, airframe, onparam }: Props = $props();

	let inputs = $state<DeriveInputs>(blankInputs());
	let gains = $state<LoopGains>(structuredClone(X3_LOOP_GAINS));
	let arm = $state<number | null>(null);
	let note = $state<string | null>(null);

	const why = $derived(inputsError(inputs) ?? gainsError(gains));
	const result = $derived(why ? null : derive(inputs, gains));
	const rows = $derived(result ? stagePlan(schema, result, value) : []);
	const toStage = $derived(rows.filter((r) => r.index >= 0 && r.inRange && r.changed));
	const refused = $derived(rows.filter((r) => !r.inRange));
	const AXES = ['x', 'y', 'z'] as const;
	const fmt = (v: number): string => (!Number.isFinite(v) ? '—' : v === 0 ? '0' : Math.abs(v) < 1e-2 ? v.toExponential(4) : v.toPrecision(4));

	function prefill(): void {
		if (!airframe) return;
		inputs = prefillInputs(airframe);
		arm = null;
		note = `Prefilled from ${airframe.label} (${airframe.source || airframe.id}). Nothing is staged until you press Stage.`;
	}

	function fillSquare(): void {
		if (arm === null || !(arm > 0)) return;
		inputs.rotors = squareX(arm);
	}

	function stage(): void {
		for (const r of toStage) onparam(r.index, r.value);
		note = `Staged ${toStage.length} derived values` + (refused.length ? `; out of range, not staged: ${refused.map((r) => r.key).join(', ')}` : '') + '. Apply runs them.';
	}
</script>

<details class="derive mono">
	<summary>Derive from airframe specs</summary>
	<p class="hint">
		Optional: enter your airframe's physical specs to compute the hover thrust, m g / (4 k ω²), and the rate-loop gains, K = K_phys I / τ_max with the
		quad-x allocation's full torques τ roll = ½ T_max Σ|y|, pitch = ½ T_max Σ|x|, yaw = ½ T_max k_m · 4, T_max = k ω². The values are staged like any edit.
	</p>
	<div class="row">
		<button type="button" class="btn" disabled={!airframe} onclick={prefill} title={airframe ? `Copy ${airframe.label}'s specs into the inputs` : 'Select an airframe on the Development tab'}>
			Prefill from sim airframe{airframe ? ` (${airframe.label})` : ''}
		</button>
	</div>

	<div class="grid">
		<fieldset>
			<legend>Airframe</legend>
			<label>mass <span><input type="number" step="any" bind:value={inputs.mass_kg} /> kg</span></label>
			<label>Ixx <span><input type="number" step="any" bind:value={inputs.ixx} /> kg m²</span></label>
			<label>Iyy <span><input type="number" step="any" bind:value={inputs.iyy} /> kg m²</span></label>
			<label>Izz <span><input type="number" step="any" bind:value={inputs.izz} /> kg m²</span></label>
			<label>thrust constant k <span><input type="number" step="any" bind:value={inputs.motor_constant} /> N s²</span></label>
			<label>max rotor speed ω <span><input type="number" step="any" bind:value={inputs.max_rot_velocity} /> rad/s</span></label>
			<label>moment constant k_m <span><input type="number" step="any" bind:value={inputs.moment_constant} /> m</span></label>
		</fieldset>
		<fieldset>
			<legend>Rotor positions (x forward, y; m)</legend>
			{#each inputs.rotors as _, i (i)}
				<label>rotor {i} <span><input type="number" step="any" bind:value={inputs.rotors[i][0]} aria-label="rotor {i} x" /> <input type="number" step="any" bind:value={inputs.rotors[i][1]} aria-label="rotor {i} y" /></span></label>
			{/each}
			<label>or square X, arm <span><input type="number" step="any" bind:value={arm} /> m <button type="button" class="btn sm" disabled={arm === null || !(arm > 0)} onclick={fillSquare}>Set</button></span></label>
		</fieldset>
		<fieldset>
			<legend>Target loop gains (bandwidth choices, X3 defaults)</legend>
			{#each AXES as ax, k (ax)}
				<label>rate {ax}: P, I, D
					<span>
						<input type="number" step="any" bind:value={gains.rate_p[k]} aria-label="rate P {ax} (1/s)" />
						<input type="number" step="any" bind:value={gains.rate_i[k]} aria-label="rate I {ax} (1/s²)" />
						<input type="number" step="any" bind:value={gains.rate_d[k]} aria-label="rate D {ax}" />
					</span>
				</label>
			{/each}
			<label>rate integrator limit <span><input type="number" step="any" bind:value={gains.rate_int_accel} /> rad/s²</span></label>
			<label>yaw torque limit <span><input type="number" step="any" bind:value={gains.yaw_torque_nm} /> N m</span></label>
			<button type="button" class="btn sm" onclick={() => (gains = structuredClone(X3_LOOP_GAINS))}>X3 defaults</button>
		</fieldset>
	</div>

	{#if why}
		<p class="hint">{why}</p>
	{:else if result}
		<p class="hint">
			T_max {fmt(result.t_max_n)} N per rotor; τ_max roll {fmt(result.tau_max[0])}, pitch {fmt(result.tau_max[1])}, yaw {fmt(result.tau_max[2])} N m.
		</p>
		<table class="plan">
			<thead><tr><th>param</th><th>staged</th><th>derived</th><th></th></tr></thead>
			<tbody>
				{#each rows as r (r.key)}
					<tr class:changed={r.changed} class:bad={!r.inRange}>
						<td>{r.key}</td>
						<td>{fmt(r.current)}</td>
						<td>{fmt(r.value)} {r.unit}</td>
						<td>{r.index < 0 ? 'not in this schema' : !r.inRange ? 'out of range' : r.changed ? 'will change' : 'same'}</td>
					</tr>
				{/each}
			</tbody>
		</table>
		<div class="row">
			<button type="button" class="btn go" disabled={disabled || toStage.length === 0} onclick={stage} title={disabled ? 'Needs a connected FC with this schema' : 'Send the derived values as edits'}>
				Stage {toStage.length} derived values
			</button>
		</div>
	{/if}
	{#if note}<p class="hint" role="status">{note}</p>{/if}
</details>

<style>
	.derive {
		border: 1px solid var(--border);
		border-radius: 8px;
		background: var(--surface);
		padding: 0.5rem 0.7rem;
		font-size: 0.8rem;
	}
	summary {
		cursor: pointer;
		font-weight: 600;
	}
	.derive > :not(summary) {
		margin-top: 0.5rem;
	}
	.grid {
		display: grid;
		grid-template-columns: repeat(auto-fill, minmax(20rem, 1fr));
		gap: 0.75rem;
	}
	fieldset {
		border: 1px solid var(--border);
		border-radius: 6px;
		margin: 0;
		padding: 0.4rem 0.6rem;
		display: flex;
		flex-direction: column;
		gap: 0.3rem;
	}
	legend {
		color: var(--muted);
		padding: 0 0.3rem;
	}
	label {
		display: flex;
		justify-content: space-between;
		align-items: center;
		gap: 0.4rem;
		color: var(--muted);
	}
	label span {
		color: var(--text);
		display: flex;
		align-items: center;
		gap: 0.3rem;
	}
	input {
		background: var(--surface-2);
		color: var(--text);
		border: 1px solid var(--border);
		border-radius: 6px;
		padding: 0.15rem 0.35rem;
		font: inherit;
		width: 5.5rem;
	}
	.row {
		display: flex;
		gap: 0.5rem;
		align-items: center;
		flex-wrap: wrap;
	}
	.hint {
		margin: 0;
		color: var(--muted);
		max-width: none;
	}
	.plan {
		border-collapse: collapse;
		font-size: 0.75rem;
		font-variant-numeric: tabular-nums;
	}
	.plan th,
	.plan td {
		padding: 0.15rem 0.6rem 0.15rem 0;
		border-bottom: 1px solid var(--border);
		text-align: left;
	}
	.plan th {
		color: var(--muted);
		font-weight: 600;
	}
	.plan tr.changed td:nth-child(3) {
		color: var(--accent);
	}
	.plan tr.bad td {
		color: var(--bad);
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
		align-self: flex-start;
	}
	.btn.go:not(:disabled) {
		border-color: var(--ok);
		color: var(--ok);
	}
</style>
