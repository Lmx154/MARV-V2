<script lang="ts">
	import type { ParamSpec } from '$lib/gcs/types';
	import type { Rejected } from '$lib/gcs/setup';

	let {
		spec,
		value,
		reference,
		rejected,
		pending,
		disabled,
		onchange
	}: {
		spec: ParamSpec;
		value: number;
		/** The value "modified" is measured against and reset restores (the factory value of this kind). */
		reference: number;
		/** The last edit the FC did not take, if any. */
		rejected?: Rejected;
		/** An edit is in flight, awaiting the FC's echo. */
		pending: boolean;
		disabled: boolean;
		onchange: (v: number) => void;
	} = $props();

	const digits = $derived.by(() => {
		if (spec.digits !== undefined) return spec.digits;
		const str = String(spec.step);
		const i = str.indexOf('.');
		return i === -1 ? 0 : str.length - i - 1;
	});
	const modified = $derived(Math.fround(value) !== Math.fround(reference));
	const fmt = (v: number): string => (!Number.isFinite(v) ? '—' : digits > 6 ? v.toExponential(4) : v.toFixed(digits));
	const show = (v: number): string => `${fmt(v)}${spec.unit ? ` ${spec.unit}` : ''}`;
	const uid = $props.id();
	const id = `param-${uid}`;
</script>

<div class="param" class:modified class:rejected={!!rejected}>
	<div class="head">
		<label class="label mono" for={id}>
			{#if modified}<span class="dot" aria-hidden="true"></span>{/if}{spec.label}
		</label>
		<span class="readout mono" class:pending>{show(value)}</span>
		<button
			type="button"
			class="reset mono"
			title={`Reset to ${show(reference)}`}
			aria-label={`Reset ${spec.label}`}
			disabled={disabled || !modified}
			onclick={() => onchange(reference)}>↺</button
		>
	</div>
	<input
		{id}
		class="slider"
		type="range"
		min={spec.min}
		max={spec.max}
		step={spec.step}
		{value}
		{disabled}
		oninput={(e) => onchange(Number((e.currentTarget as HTMLInputElement).value))}
	/>
	<div class="minmax mono"><span>{spec.min}</span><span>{spec.max}</span></div>
	{#if rejected}
		<p class="refused mono" role="status">
			{rejected.timeout ? 'no reply for' : 'refused'}
			{show(rejected.requested)}: the FC holds {show(rejected.held)}
		</p>
	{/if}
	{#if spec.note}<p class="note">{spec.note}</p>{/if}
	{#if spec.source}<p class="source">default from: {spec.source}</p>{/if}
</div>

<style>
	.param {
		display: flex;
		flex-direction: column;
		gap: 0.2rem;
		padding: 0.45rem 0.5rem;
		border-radius: 6px;
		min-width: 0;
	}
	.param.modified {
		background: var(--accent-bg);
	}
	.param.rejected {
		outline: 1px solid var(--bad);
	}
	.head {
		display: flex;
		align-items: baseline;
		gap: 0.4rem;
	}
	.label {
		flex: 1;
		min-width: 0;
		font-size: 0.7rem;
		text-transform: uppercase;
		letter-spacing: 0.06em;
		color: var(--muted);
		font-weight: 600;
	}
	.modified .label {
		color: var(--accent);
	}
	.dot {
		display: inline-block;
		width: 0.45rem;
		height: 0.45rem;
		border-radius: 50%;
		background: var(--accent);
		margin-right: 0.35rem;
		vertical-align: middle;
	}
	.readout {
		font-size: 0.8rem;
		color: var(--text);
		font-variant-numeric: tabular-nums;
		white-space: nowrap;
	}
	.readout.pending {
		color: var(--muted);
	}
	.reset {
		background: transparent;
		border: 1px solid var(--border);
		border-radius: 4px;
		color: var(--text);
		font-size: 0.7rem;
		line-height: 1;
		padding: 0.1rem 0.3rem;
		cursor: pointer;
	}
	.reset:disabled {
		opacity: 0.35;
		cursor: default;
	}
	.slider {
		width: 100%;
		margin: 0;
	}
	.minmax {
		display: flex;
		justify-content: space-between;
		font-size: 0.65rem;
		color: var(--muted);
	}
	.note,
	.source,
	.refused {
		margin: 0;
		font-size: 0.75rem;
		color: var(--muted);
	}
	.refused {
		color: var(--bad);
	}
	.source {
		opacity: 0.8;
		font-style: italic;
	}
</style>
