<script lang="ts">
	import { formatValue } from '$link/bits';
	import type { StageResult } from '$link/vehicle';
	import type { ParamRow } from '$lib/params/merge';
	import { checkStage, DERIVED_WARNING, lockTooltip, whyReadOnly, type StageContext } from '$lib/params/staging';
	import Badge from './ui/Badge.svelte';

	let {
		row,
		ctx,
		result,
		onstage,
		onreadsource
	}: {
		row: ParamRow;
		ctx: StageContext;
		result: StageResult | undefined;
		onstage: (row: ParamRow, bits: number, source: string) => Promise<StageResult>;
		onreadsource: (row: ParamRow) => Promise<void>;
	} = $props();

	const uid = $props.id();
	let open = $state(false);
	let valueText = $state('');
	let sourceText = $state('');
	let ack = $state(false);
	let error = $state<string | null>(null);
	let sending = $state(false);
	let reading = $state(false);

	const readOnly = $derived(whyReadOnly(row, ctx));
	const fmt = (bits: number | null): string => (bits === null ? '—' : formatValue(bits, row.type));
	const sigmaText = $derived(
		row.sigmaKind === null ? '—' : row.sigmaKind === 'Known' ? `± ${row.sigma ?? '?'}` : row.sigmaKind.toLowerCase()
	);
	const needsRead = $derived(row.sourceLength !== null && !row.sourceFromVehicle);

	function toggle(): void {
		open = !open;
		if (open) {
			valueText = fmt(row.stagedBits ?? row.factoryBits);
			if (needsRead) void read();
		}
	}

	async function read(): Promise<void> {
		reading = true;
		try {
			await onreadsource(row);
		} finally {
			reading = false;
		}
	}

	async function stage(e: SubmitEvent): Promise<void> {
		e.preventDefault();
		const check = checkStage(row, ctx, valueText, sourceText, ack);
		if (!check.ok) {
			error = check.error;
			return;
		}
		error = null;
		sending = true;
		try {
			const r = await onstage(row, check.request.bits, check.request.source);
			if (r.accepted) {
				sourceText = '';
				ack = false;
			}
		} catch (err) {
			error = err instanceof Error ? err.message : String(err);
		} finally {
			sending = false;
		}
	}
</script>

<tr class:differs={row.differs} class:locked={row.locked} class:open>
	<th scope="row" class="name">
		<button type="button" class="expander mono" aria-expanded={open} aria-controls={`${uid}-detail`} onclick={toggle}>
			<span aria-hidden="true">{open ? '▾' : '▸'}</span>
			<span class="full">{row.name}</span>
		</button>
		<span class="linkid mono" title="the id of this parameter on the link">{row.linkId}{row.index !== null ? ` · #${row.index}` : ''}</span>
	</th>
	<td class="num mono">{fmt(row.activeBits)}</td>
	<td class="num mono staged" class:diff={row.differs}>{fmt(row.stagedBits)}{#if row.differs}<span class="sr"> (differs from active)</span>{/if}</td>
	<td class="mono unit">{row.unit || '—'}</td>
	<td class="mono">{row.method ?? '—'}</td>
	<td class="mono">{row.origin ?? '—'}</td>
	<td class="mono">{sigmaText}</td>
	<td class="flags">
		{#if row.locked}<span title={lockTooltip(row)}><Badge text="locked" tone="warn" /></span>{/if}
		{#if row.status !== 'matched'}<Badge text={row.status} tone={row.status === 'mismatch' ? 'bad' : 'neutral'} />{/if}
		{#if row.method === 'derived'}<span title="derived value: see the warning when editing"><Badge text="derived" /></span>{/if}
		{#if result && !result.accepted}<Badge text="refused" tone="bad" />{:else if result?.accepted}<Badge text="staged" tone="ok" />{/if}
	</td>
</tr>
{#if open}
	<tr class="detail" id={`${uid}-detail`}>
		<td colspan="8">
			<div class="body">
				<section>
					<h4>Source</h4>
					<p class="src">{row.source ?? '(the vehicle has not sent the source text)'}</p>
					<p class="muted small">
						{#if row.sourceFromVehicle}text from the vehicle{:else}factory text from the manifest{/if}
						{#if row.sourceLength !== null}· {row.sourceLength} bytes on the vehicle{/if}
						{#if needsRead}
							· <button type="button" class="link" disabled={reading} onclick={() => void read()}>{reading ? 'reading…' : 'read from the vehicle'}</button>
						{/if}
					</p>
					<dl class="facts mono">
						<dt>type</dt><dd>{row.type}</dd>
						<dt>factory value</dt><dd>{fmt(row.factoryBits)}</dd>
						<dt>group</dt><dd>{row.group}</dd>
					</dl>
					{#each row.notes as n (n)}<p class="warn small">{n}</p>{/each}
				</section>
				<section>
					<h4>Stage a value</h4>
					{#if readOnly}
						<p class="muted">Read-only: {readOnly}</p>
					{:else}
						<form onsubmit={stage}>
							<div class="field">
								<label for={`${uid}-value`}>Value{row.unit ? ` (${row.unit})` : ''}</label>
								<input id={`${uid}-value`} class="mono" type="text" bind:value={valueText} autocomplete="off" spellcheck="false" />
							</div>
							<div class="field wide">
								<label for={`${uid}-source`}>Source (required)</label>
								<textarea id={`${uid}-source`} rows="3" bind:value={sourceText} placeholder="where does this value come from?"></textarea>
							</div>
							{#if row.method === 'derived'}
								<div class="derived" role="note">
									<strong>Derived value.</strong> {DERIVED_WARNING}
									<label class="check"><input type="checkbox" bind:checked={ack} /> I understand</label>
								</div>
							{/if}
							<div class="actions">
								<button type="submit" class="primary" disabled={sending}>{sending ? 'Staging…' : 'Stage'}</button>
								{#if error}<span class="bad" role="alert">{error}</span>{/if}
							</div>
						</form>
					{/if}
					{#if result}
						<p class="result" class:bad={!result.accepted} class:ok={result.accepted} role="status">
							{#if result.accepted}
								Vehicle echoed {fmt(result.echoBits)}.
								{#if result.sourceStored === true}Source stored.{:else if result.sourceStored === false}Source not confirmed by the vehicle.{/if}
							{:else}
								Refused: the vehicle kept {fmt(result.echoBits)}.
								{#if result.note}{result.note}{/if}
							{/if}
							{#each result.reasons as r (r)}<span class="reason"> “{r}”</span>{/each}
							{#if result.accepted && result.note}<span class="warn"> {result.note}</span>{/if}
						</p>
					{/if}
				</section>
			</div>
		</td>
	</tr>
{/if}

<style>
	tr > * {
		padding: 0.15rem 0.5rem;
		border-bottom: 1px solid var(--border);
		vertical-align: baseline;
		text-align: left;
		font-weight: 400;
	}

	tr.locked {
		background: var(--lock-bg);
	}

	tr.open {
		background: var(--accent-bg);
	}

	.name {
		min-width: 14rem;
	}

	.expander {
		background: none;
		border: none;
		padding: 0.1rem 0.2rem;
		text-align: left;
		display: inline-flex;
		gap: 0.35rem;
		color: var(--text);
		font-size: 0.8rem;
	}

	.expander:hover:not(:disabled) {
		color: var(--accent);
	}

	.full {
		overflow-wrap: anywhere;
	}

	.linkid {
		display: block;
		margin-left: 1.15rem;
		font-size: 0.65rem;
		color: var(--muted);
	}

	.num {
		text-align: right;
		white-space: nowrap;
	}

	td.staged.diff {
		background: var(--diff-bg);
		font-weight: 700;
		color: var(--accent);
	}

	td.unit {
		color: var(--muted);
		font-size: 0.75rem;
	}

	td {
		font-size: 0.8rem;
	}

	.flags {
		white-space: nowrap;
		display: flex;
		gap: 0.25rem;
	}

	.sr {
		position: absolute;
		width: 1px;
		height: 1px;
		overflow: hidden;
		clip: rect(0 0 0 0);
	}

	.detail > td {
		padding: 0.5rem 0.75rem 0.75rem 1.7rem;
		background: var(--surface);
	}

	.body {
		display: grid;
		grid-template-columns: minmax(0, 3fr) minmax(0, 2fr);
		gap: 1.5rem;
	}

	@media (max-width: 62rem) {
		.body {
			grid-template-columns: minmax(0, 1fr);
		}
	}

	h4 {
		margin: 0 0 0.25rem;
		font-size: 0.7rem;
		text-transform: uppercase;
		letter-spacing: 0.08em;
		color: var(--muted);
	}

	.src {
		margin: 0;
		white-space: pre-wrap;
		overflow-wrap: anywhere;
		max-height: 14rem;
		overflow: auto;
		font-size: 0.8rem;
		max-width: none;
	}

	.small {
		font-size: 0.75rem;
		margin: 0.25rem 0 0;
	}

	.link {
		background: none;
		border: none;
		padding: 0;
		color: var(--accent);
		text-decoration: underline;
	}

	.facts {
		display: grid;
		grid-template-columns: max-content 1fr;
		gap: 0.1rem 1rem;
		margin: 0.5rem 0 0;
		font-size: 0.75rem;
	}

	.facts dt {
		color: var(--muted);
	}

	.facts dd {
		margin: 0;
	}

	form {
		display: flex;
		flex-direction: column;
		gap: 0.5rem;
	}

	.field {
		display: flex;
		flex-direction: column;
		gap: 0.15rem;
	}

	label {
		font-size: 0.7rem;
		text-transform: uppercase;
		letter-spacing: 0.06em;
		color: var(--muted);
	}

	.derived {
		border: 1px solid var(--warn);
		border-radius: 4px;
		padding: 0.4rem 0.5rem;
		font-size: 0.8rem;
		color: var(--warn);
	}

	.check {
		display: block;
		margin-top: 0.3rem;
		text-transform: none;
		letter-spacing: normal;
		font-size: 0.8rem;
		color: var(--text);
	}

	.actions {
		display: flex;
		gap: 0.75rem;
		align-items: center;
		flex-wrap: wrap;
	}

	.result {
		margin: 0.5rem 0 0;
		font-size: 0.8rem;
		max-width: none;
	}

	.reason {
		font-style: italic;
	}
</style>
