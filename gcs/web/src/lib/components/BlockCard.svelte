<script module lang="ts">
	/** Family -> layer colour token of the toolbox's design. */
	const FAMILY_COLOUR: Record<string, string> = {
		vehicle: 'var(--l-design)',
		sensors: 'var(--l-hardware)',
		estimator: 'var(--l-validation)',
		guidance: 'var(--l-software)',
		controller: 'var(--l-software)',
		allocation: 'var(--l-firmware)',
		actuators: 'var(--l-firmware)'
	};
</script>

<script lang="ts">
	import type { FamilyDef, KindDef } from '$lib/gcs/types';
	import type { Rejected } from '$lib/gcs/setup';
	import ParamSlider from './ParamSlider.svelte';

	let {
		family,
		kind,
		options,
		error,
		value,
		reference,
		rejected,
		pending,
		disabled,
		onkind,
		onparam,
		onreset
	}: {
		family: FamilyDef;
		/** Kind index the FC reports staged for this family. */
		kind: number;
		/** The kinds offered: those that serve the staged vehicle (every class on the vehicle card). */
		options: { kind: number; def: KindDef }[];
		/** The FC's refusal of the last kind asked for, if any. */
		error: string | undefined;
		/** Value shown for an absolute param index. */
		value: (index: number) => number;
		/** Values "modified" is measured against: the factory values of this kind. */
		reference: Record<string, number>;
		rejected: Record<number, Rejected>;
		pending: (index: number) => boolean;
		disabled: boolean;
		onkind: (kind: number) => void;
		onparam: (index: number, value: number) => void;
		onreset: () => void;
	} = $props();

	let expanded = $state(false);
	const uid = $props.id();

	const def = $derived(family.kinds[kind]);
	const modifiedCount = $derived(def ? def.params.filter((p) => Math.fround(value(p.index)) !== Math.fround(reference[p.id] ?? p.default)).length : 0);
	const groups = $derived.by(() => {
		if (!def) return [];
		const byId = new Map(def.params.map((p) => [p.id, p]));
		const used = new Set<string>();
		const out = (def.parts ?? []).map((part) => ({
			id: part.id,
			label: part.label,
			note: part.note,
			params: part.params.flatMap((id) => {
				const spec = byId.get(id);
				if (!spec) return [];
				used.add(id);
				return [spec];
			})
		}));
		const rest = def.params.filter((p) => !used.has(p.id));
		if (rest.length) out.push({ id: 'rest', label: 'Parameters', note: undefined, params: rest });
		return out.filter((g) => g.params.length);
	});
</script>

<div class="card" class:expanded style={`--family:${FAMILY_COLOUR[family.id] ?? 'var(--muted)'}`}>
	<div class="family mono">{family.id}</div>
	{#key `${kind}:${error ?? ''}`}
		<select
			class="kind"
			aria-label={`${family.id} block`}
			value={kind}
			{disabled}
			onchange={(e) => onkind(Number((e.currentTarget as HTMLSelectElement).value))}
		>
			{#each options as o (o.def.id)}
				<option value={o.kind}>{o.def.label}</option>
			{/each}
		</select>
	{/key}
	{#if error}<p class="err mono" role="alert">{error}</p>{/if}
	{#if def}<p class="summary" class:clamped={!expanded} title={expanded ? undefined : def.summary}>{def.summary}</p>{/if}
	<div class="foot mono">
		<span class="count">{def?.params.length ?? 0} params · <span class:accent={modifiedCount > 0}>{modifiedCount} modified</span></span>
		{#if def && def.params.length > 0}
			<button type="button" class="toggle mono" aria-expanded={expanded} aria-controls={`${uid}-params`} onclick={() => (expanded = !expanded)}>
				{expanded ? '▾ collapse' : '▸ expand'}
			</button>
		{/if}
	</div>

	{#if expanded && def}
		<div class="params" id={`${uid}-params`}>
			{#each groups as g (g.id)}
				<fieldset class="part">
					<legend class="mono">{g.label}</legend>
					{#if g.note}<p class="part-note">{g.note}</p>{/if}
					<div class="grid">
						{#each g.params as spec (spec.id)}
							<ParamSlider
								{spec}
								value={value(spec.index)}
								reference={reference[spec.id] ?? spec.default}
								rejected={rejected[spec.index]}
								pending={pending(spec.index)}
								{disabled}
								onchange={(v) => onparam(spec.index, v)}
							/>
						{/each}
					</div>
				</fieldset>
			{/each}
			<div class="actions">
				<button type="button" class="reset-block mono" disabled={disabled || modifiedCount === 0} onclick={onreset}>reset block</button>
			</div>
		</div>
	{/if}
</div>

<style>
	.card {
		flex: 1 1 14rem;
		min-width: 14rem;
		max-width: 100%;
		box-sizing: border-box;
		display: flex;
		flex-direction: column;
		gap: 0.4rem;
		padding: 0.6rem 0.7rem;
		border: 1px solid var(--border);
		border-top: 3px solid var(--family);
		border-radius: 8px;
		background: var(--surface);
	}
	.card.expanded {
		flex-basis: 100%;
	}
	.family {
		font-variant: small-caps;
		text-transform: lowercase;
		letter-spacing: 0.08em;
		font-weight: 700;
		color: var(--family);
		font-size: 0.85rem;
	}
	.kind {
		width: 100%;
		background: var(--surface-2);
		color: var(--text);
		border: 1px solid var(--border);
		border-radius: 6px;
		padding: 0.2rem 0.4rem;
		font: inherit;
		font-size: 0.85rem;
	}
	.err {
		margin: 0;
		font-size: 0.75rem;
		color: var(--bad);
	}
	.summary {
		margin: 0;
		font-size: 0.78rem;
		color: var(--muted);
	}
	.summary.clamped {
		display: -webkit-box;
		-webkit-line-clamp: 3;
		line-clamp: 3;
		-webkit-box-orient: vertical;
		overflow: hidden;
	}
	.foot {
		display: flex;
		align-items: center;
		justify-content: space-between;
		gap: 0.5rem;
		font-size: 0.72rem;
		color: var(--muted);
		margin-top: auto;
	}
	.accent {
		color: var(--accent);
	}
	.toggle,
	.reset-block {
		background: var(--surface-2);
		color: var(--text);
		border: 1px solid var(--border);
		border-radius: 6px;
		padding: 0.15rem 0.5rem;
		font-size: 0.72rem;
		cursor: pointer;
	}
	.reset-block:disabled {
		opacity: 0.5;
		cursor: default;
	}
	.params {
		display: flex;
		flex-direction: column;
		gap: 0.6rem;
		margin-top: 0.3rem;
	}
	.part {
		border: 1px solid var(--border);
		border-radius: 6px;
		padding: 0.3rem 0.4rem 0.4rem;
		margin: 0;
		min-width: 0;
	}
	.part legend {
		font-size: 0.72rem;
		color: var(--family);
		padding: 0 0.3rem;
		font-weight: 600;
	}
	.part-note {
		margin: 0 0.5rem 0.3rem;
		font-size: 0.75rem;
		color: var(--muted);
	}
	.grid {
		display: grid;
		grid-template-columns: repeat(auto-fill, minmax(min(15rem, 100%), 1fr));
		gap: 0.3rem;
	}
	.actions {
		display: flex;
		justify-content: flex-end;
	}
</style>
