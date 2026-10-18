<script lang="ts">
	import { formatSigma, formatValue, valueTitle } from '$link/bits';
	import type { StageResult } from '$link/vehicle';
	import type { ParamRow } from '$lib/params/merge';
	import { aboutParam } from '$lib/params/view';
	import { checkStage, DERIVED_WARNING, lockTooltip, whyReadOnly, type StageContext } from '$lib/params/staging';
	import Badge from './ui/Badge.svelte';

	let {
		row,
		ctx,
		result,
		dense = false,
		onstage,
		onreadsource,
		onretry
	}: {
		row: ParamRow;
		ctx: StageContext;
		result: StageResult | undefined;
		dense?: boolean;
		onstage: (row: ParamRow, bits: number, source: string) => Promise<StageResult>;
		onreadsource: (row: ParamRow) => Promise<void>;
		onretry: (row: ParamRow) => Promise<void>;
	} = $props();

	const uid = $props.id();
	let open = $state(false);
	let valueText = $state('');
	let sourceText = $state('');
	let ack = $state(false);
	let error = $state<string | null>(null);
	let sending = $state(false);
	let reading = $state(false);
	let retrying = $state(false);
	let expander = $state<HTMLButtonElement | null>(null);

	const readOnly = $derived(whyReadOnly(row, ctx));
	const fmt = (bits: number | null): string => (bits === null ? '—' : formatValue(bits, row.type));
	const title = (bits: number | null): string | undefined => (bits === null ? undefined : valueTitle(bits, row.type));
	const sigmaText = $derived(
		row.sigmaKind === null ? '—' : row.sigmaKind === 'Known' ? (row.sigma === null ? '± ?' : `± ${formatSigma(row.sigma)}`) : row.sigmaKind.toLowerCase()
	);
	const sigmaTitle = $derived(
		row.sigmaKind === 'Known' && row.sigma !== null ? `sigma ${row.sigma} (sent as an f32)` : row.sigmaKind ? `sigma kind ${row.sigmaKind}` : undefined
	);
	const about = $derived(aboutParam(row));
	const needsRead = $derived(row.sourceLength !== null && !row.sourceFromVehicle);

	function toggle(): void {
		open = !open;
		if (open) {
			valueText = fmt(row.stagedBits ?? row.factoryBits);
			if (needsRead) void read();
		}
	}

	function close(): void {
		open = false;
		expander?.focus();
	}

	function onkeydown(e: KeyboardEvent): void {
		if (e.key === 'Escape' && open) {
			e.preventDefault();
			e.stopPropagation();
			close();
		}
	}

	async function retry(): Promise<void> {
		retrying = true;
		try {
			await onretry(row);
		} finally {
			retrying = false;
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

<tr class:differs={row.differs} class:locked={row.locked} class:open class:dense {onkeydown}>
	<th scope="row" class="name">
		<button
			type="button"
			class="expander mono"
			data-param-expander
			bind:this={expander}
			aria-expanded={open}
			aria-controls={`${uid}-detail`}
			aria-label={`${row.name}: ${open ? 'close' : 'open'} the details`}
			onclick={toggle}
		>
			<span aria-hidden="true">{open ? '▾' : '▸'}</span>
			<span class="full">{row.name}</span>
		</button>
		<span class="linkid mono" title="the id of this parameter on the link">{row.linkId}{row.index !== null ? ` · #${row.index}` : ''}</span>
	</th>
	<td class="num mono factory" title={title(row.factoryBits)}>{fmt(row.factoryBits)}</td>
	<td class="num mono active" class:chg={row.changed} title={title(row.activeBits)}>{fmt(row.activeBits)}{#if row.changed}<span class="sr"> (differs from the factory value)</span>{/if}</td>
	<td class="num mono staged" class:diff={row.differs} title={title(row.stagedBits)}>{fmt(row.stagedBits)}{#if row.differs}<span class="sr"> (differs from active)</span>{/if}</td>
	<td class="mono unit">{row.unit || '—'}</td>
	<td class="mono">{row.method ?? '—'}</td>
	<td class="mono">{row.origin ?? '—'}</td>
	<td class="mono" title={sigmaTitle}>{sigmaText}</td>
	<td class="flags"><div class="flagbox">
		{#if row.locked}<span title={lockTooltip(row)}><Badge text="locked" tone="warn" /></span>{/if}
		{#if row.status !== 'matched'}<Badge text={row.status} tone={row.status === 'mismatch' ? 'bad' : 'neutral'} />{/if}
		{#if row.method === 'derived'}<span title="derived value: see the warning when editing"><Badge text="derived" /></span>{/if}
		{#if result && !result.accepted}<Badge text="refused" tone="bad" />
		{:else if result?.accepted && result.sourceState === 'failed'}<span title="the value is staged, the source text was not stored"><Badge text="source failed" tone="bad" /></span>
		{:else if result?.accepted && result.sourceState === 'writing'}<Badge text="writing source" tone="warn" />
		{:else if result?.accepted}<Badge text="staged" tone="ok" />{/if}
	</div></td>
</tr>
{#if open}
	<tr class="detail" id={`${uid}-detail`} {onkeydown}>
		<td colspan="9">
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
						<dt>factory value</dt><dd title={title(row.factoryBits)}>{fmt(row.factoryBits)}</dd>
						<dt>group</dt><dd>{row.group}</dd>
					</dl>
					{#each row.notes as n (n)}<p class="warn small">{n}</p>{/each}
					<h4 class="about-h">About this parameter</h4>
					<dl class="facts mono about" aria-label={`About ${row.name}`}>
						<dt>link id</dt><dd>{about.linkId} <span class="muted">— {about.linkIdNote}</span></dd>
						<dt>index</dt><dd>{about.index}</dd>
						<dt>sigma</dt><dd>{about.sigma}</dd>
						{#if about.derivation}
							<dt>derivation</dt><dd class="wrapline">{about.derivation} <span class="muted">(the provenance text: the rule, then “; ” and the source)</span></dd>
						{/if}
						{#if about.lock}<dt>lock</dt><dd class="wrapline">{about.lock}</dd>{/if}
					</dl>
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
								{#if result.sourceState === 'writing'}Writing the source text{result.sourceAttempts > 1 ? ` (attempt ${result.sourceAttempts})` : ''}…
								{:else if result.sourceState === 'stored'}Source stored.
								{:else if result.sourceState === 'failed'}The value is staged but its source text was not stored ({result.sourceAttempts} write{result.sourceAttempts === 1 ? '' : 's'}); the vehicle holds the placeholder source.
									<button type="button" class="link" disabled={retrying} onclick={() => void retry()}>{retrying ? 'retrying…' : 'retry the source write'}</button>
								{/if}
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

	.flagbox {
		white-space: nowrap;
		display: flex;
		gap: 0.25rem;
	}

	td.factory {
		color: var(--muted);
	}

	td.active.chg {
		background: var(--diff-bg);
		color: var(--accent);
	}

	tr.dense > * {
		padding-top: 0;
		padding-bottom: 0;
		line-height: 1.2;
	}

	tr.dense .linkid {
		display: inline;
		margin-left: 0.5rem;
	}

	tr.dense .expander {
		padding-top: 0;
		padding-bottom: 0;
	}

	.about-h {
		margin-top: 0.75rem;
	}

	.wrapline {
		white-space: pre-wrap;
		overflow-wrap: anywhere;
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
