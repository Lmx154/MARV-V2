<script lang="ts">
	import { formatValue } from '$link/bits';
	import type { ParamRow } from '$lib/params/merge';
	import type { ValidationState } from '$lib/state/validation_state';
	import Badge from './ui/Badge.svelte';
	import Panel from './ui/Panel.svelte';

	let {
		rows,
		validation,
		whyValidate,
		whyActivate,
		whyCommands,
		onvalidate,
		onactivate,
		onrevert,
		onreset
	}: {
		rows: ParamRow[];
		validation: ValidationState;
		/** Why Validate is off, or null. */
		whyValidate: string | null;
		/** Why Activate is off (the gate: whyCannotActivate), or null. */
		whyActivate: string | null;
		/** Why Revert staged and Reset to factory are off, or null. */
		whyCommands: string | null;
		onvalidate: () => void;
		onactivate: () => void;
		onrevert: () => void;
		onreset: () => void;
	} = $props();

	const fmt = (r: ParamRow, bits: number | null): string => (bits === null ? '—' : formatValue(bits, r.type));
	const report = $derived(validation.record?.report ?? null);
	const validateOff = $derived(whyValidate !== null || validation.busy);
</script>

<Panel title={`Staged changes (${rows.length})`} label="staged vs active">
	{#if rows.length === 0}
		<p class="muted">Nothing is staged: the staged config equals the active one.</p>
	{:else}
		<ul class="list mono">
			{#each rows as r (r.key)}
				<li><span class="n">{r.name}</span> <span class="v">{fmt(r, r.activeBits)} → <strong>{fmt(r, r.stagedBits)}</strong> {r.unit}</span></li>
			{/each}
		</ul>
	{/if}

	<div class="buttons">
		<button type="button" disabled={validateOff} title={whyValidate ?? 'Run the validator on the staged config for the vehicle\'s image'} aria-describedby="why-validate" onclick={onvalidate}>
			{validation.phase === 'reading' ? 'Reading sources…' : validation.phase === 'validating' ? 'Validating…' : 'Validate'}
		</button>
		<span id="why-validate" class="why">{whyValidate ?? ''}</span>
	</div>

	{#if validation.progress.length > 0 && (validation.phase === 'validating' || report === null)}
		<ul class="progress mono" aria-label="Validation progress">
			{#each validation.progress as line, i (i)}<li>{line}</li>{/each}
		</ul>
	{/if}
	{#if validation.error}<p class="bad" role="alert">{validation.error}</p>{/if}

	{#if validation.record && report}
		<div class="report" aria-label="Validation report">
			<p>
				<Badge text={report.verdict || 'no verdict'} tone={report.verdict === 'PASS' ? 'ok' : 'bad'} />
				<span class="mono small">report {validation.record.reportId.slice(0, 16)}…</span>
			</p>
			<p class="mono small wrap">image {validation.record.imageHash.slice(0, 16)}… · config {validation.record.configHash.slice(0, 16)}…</p>
			{#each report.checks as c (c.id)}
				<details open={c.result !== 'PASS'}>
					<summary><Badge text={c.result} tone={c.result === 'PASS' ? 'ok' : 'bad'} /> <strong>{c.id}</strong> <span class="muted">{c.title}</span></summary>
					{#if c.failures.length > 0}
						<ul class="mono small bad">{#each c.failures as f, i (i)}<li>{f}</li>{/each}</ul>
					{/if}
					<ul class="mono small values">{#each c.values as v, i (i)}<li>{v}</li>{/each}</ul>
				</details>
			{/each}
		</div>
	{/if}

	<div class="buttons">
		<button type="button" class="primary" disabled={whyActivate !== null || validation.busy} title={whyActivate ?? 'Activate the staged config: the vehicle restarts to apply it'} aria-describedby="why-activate" onclick={onactivate}>Activate</button>
		<span id="why-activate" class="why">{whyActivate ?? ''}</span>
	</div>
	<div class="buttons">
		<button type="button" disabled={whyCommands !== null || validation.busy || rows.length === 0} title={whyCommands ?? 'Discard the staged changes'} aria-describedby="why-cmds" onclick={onrevert}>Revert staged</button>
		<button type="button" disabled={whyCommands !== null || validation.busy} title={whyCommands ?? 'Replace the staged config by the factory config'} aria-describedby="why-cmds" onclick={onreset}>Reset to factory</button>
		<span id="why-cmds" class="why">{whyCommands ?? ''}</span>
	</div>
	{#if validation.command}
		<p class="small" class:bad={validation.command.result !== 'accepted'} role="status">{validation.command.action}: {validation.command.text}</p>
	{/if}
</Panel>

<style>
	.list {
		list-style: none;
		margin: 0 0 0.5rem;
		padding: 0;
		font-size: 0.8rem;
		max-height: 14rem;
		overflow: auto;
	}

	.list li {
		display: flex;
		justify-content: space-between;
		gap: 1rem;
		padding: 0.1rem 0;
		border-bottom: 1px solid var(--border);
	}

	.n {
		overflow-wrap: anywhere;
	}

	.v {
		white-space: nowrap;
	}

	.buttons {
		display: flex;
		gap: 0.5rem;
		align-items: center;
		flex-wrap: wrap;
		margin-top: 0.4rem;
	}

	.progress,
	.values {
		list-style: none;
		margin: 0.3rem 0;
		padding: 0;
		max-height: 10rem;
		overflow: auto;
	}

	.report {
		margin-top: 0.5rem;
	}

	.report details {
		margin: 0.2rem 0;
	}

	.wrap {
		overflow-wrap: anywhere;
	}

	.why {
		font-size: 0.75rem;
		color: var(--muted);
	}
</style>
