<script lang="ts">
	import type { Schema } from '$lib/gcs/types';
	import { profiledRows, type ProfiledRow } from '$lib/gcs/profiles';
	import type { Rejected } from '$lib/gcs/setup';

	let {
		schema,
		value,
		reference,
		rejected,
		pending,
		disabled,
		onparam
	}: {
		schema: Schema;
		value: (index: number) => number;
		/** Per family, the values "modified" is measured against for the block's current kind. */
		reference: (family: number) => Record<string, number>;
		rejected: Record<number, Rejected>;
		pending: (index: number) => boolean;
		disabled: boolean;
		onparam: (index: number, value: number) => void;
	} = $props();

	const rows = $derived(profiledRows(schema));
	const block = (r: ProfiledRow): string => `${schema.families[r.family]?.id ?? ''} · ${schema.families[r.family]?.kinds[r.kind]?.label ?? ''}`;
</script>

{#if rows.length && schema.profiles.length}
	<section class="profiles" aria-label="Profiles">
		<h2 class="mono">Profiles</h2>
		<p class="hint">
			One column per flight profile; the pilot or the mission picks the active one live. Edits are staged like any other; Apply runs them (refused while armed).
		</p>
		<div class="scroll">
			<table class="mono">
				<thead>
					<tr>
						<th scope="col">parameter</th>
						{#each schema.profiles as p (p.id)}<th scope="col">{p.label}</th>{/each}
					</tr>
				</thead>
				<tbody>
					{#each rows as r (`${r.family}.${r.kind}.${r.id}`)}
						{@const ref = reference(r.family)}
						<tr>
							<th scope="row" title={r.cells[0].note}>
								{r.label}{#if r.unit}<span class="unit"> {r.unit}</span>{/if}
								<span class="block">{block(r)}</span>
							</th>
							{#each r.cells as spec, i (spec.id)}
								{@const v = value(spec.index)}
								{@const bad = rejected[spec.index]}
								<td class:modified={Math.fround(v) !== Math.fround(ref[spec.id] ?? spec.default)} class:rejected={!!bad}>
									<input
										type="number"
										min={spec.min}
										max={spec.max}
										step={spec.step}
										value={v}
										{disabled}
										class:pending={pending(spec.index)}
										aria-label={`${r.label} (${schema.profiles[i].label})`}
										title={bad ? `${bad.timeout ? 'no reply for' : 'refused'} ${bad.requested}: the FC holds ${bad.held}` : `${spec.min}..${spec.max}${r.unit ? ` ${r.unit}` : ''}`}
										onchange={(e) => onparam(spec.index, Number((e.currentTarget as HTMLInputElement).value))}
									/>
								</td>
							{/each}
						</tr>
					{/each}
				</tbody>
			</table>
		</div>
	</section>
{/if}

<style>
	.profiles {
		display: flex;
		flex-direction: column;
		gap: 0.4rem;
		padding: 0.6rem 0.7rem;
		border: 1px solid var(--border);
		border-radius: 8px;
		background: var(--surface);
	}
	h2 {
		margin: 0;
		font-size: 0.9rem;
	}
	.hint {
		margin: 0;
		font-size: 0.78rem;
		color: var(--muted);
		max-width: none;
	}
	.scroll {
		overflow-x: auto;
	}
	table {
		border-collapse: collapse;
		font-size: 0.78rem;
	}
	th,
	td {
		padding: 0.2rem 0.5rem;
		text-align: left;
		border-bottom: 1px solid var(--border);
	}
	thead th {
		color: var(--muted);
		font-weight: 600;
	}
	.unit,
	.block {
		color: var(--muted);
	}
	.block {
		display: block;
		font-size: 0.68rem;
		font-weight: 400;
	}
	td.modified {
		background: var(--accent-bg);
	}
	td.rejected {
		outline: 1px solid var(--bad);
	}
	input {
		width: 5.5rem;
		background: var(--surface-2);
		color: var(--text);
		border: 1px solid var(--border);
		border-radius: 6px;
		padding: 0.15rem 0.3rem;
		font: inherit;
	}
	input.pending {
		color: var(--muted);
	}
</style>
