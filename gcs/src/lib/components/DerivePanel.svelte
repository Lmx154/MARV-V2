<script lang="ts">
	import { onMount } from 'svelte';
	import { readable } from 'svelte/store';
	import { app } from '$lib/state/app';
	import { deriveOutcome, showValue, type DeriveState } from '$lib/state/derive_state';
	import { shortSha } from '$lib/state/logs_state';
	import type { HelperConnectionState } from '$link/transport';
	import Badge from '$lib/components/ui/Badge.svelte';
	import Panel from '$lib/components/ui/Panel.svelte';

	const gcs = app();
	const derive = gcs.derive;
	const dstate = derive?.state ?? readable<DeriveState | null>(null);
	const helperState = gcs.helper?.state ?? readable<HelperConnectionState>('closed');

	// Opening the page asks the helper for the cards and the editable entries; it derives nothing (the Derive button does, and only that).
	onMount(() => void derive?.load());

	const s = $derived($dstate);
	const open = $derived($helperState === 'open');
	const current = $derived(s?.current ?? null);
	const outcome = $derived(deriveOutcome(current));
	const whyNoDerive = $derived(s && derive ? derive.whyNoDerive(s, open) : 'no helper');
	const whyNoCancel = $derived(s?.busy ? 'a command is in progress' : !current?.running ? 'no derivation is running' : null);
	const whyNoValidate = $derived(s && derive ? derive.whyNoValidate(s, open) : 'no helper');
	const whyNoStage = $derived(s && derive ? derive.whyNoStage(s) : 'no helper');
	const tone = (o: typeof outcome): 'ok' | 'bad' | 'warn' | 'neutral' => (o === 'ok' ? 'ok' : o === 'failed' || o === 'refused' ? 'bad' : o === 'cancelled' ? 'warn' : 'neutral');
	const hex = (bits: string) => bits.replace(/^0x/, '');
</script>

<Panel title="Derive a config" label="helper">
	<p class="muted small">
		Derive builds a <span class="mono">.mcfg</span> from a copy of a vehicle card with your edits, by the build the repository runs for the flight set (flatten, parameter
		generator, config encoder). Every edit needs its source or rationale text, which becomes the entry's source. The repository's card is never written: the copy
		lives in the helper's run directory. Locked entries change only by the lock editor.
	</p>
	<div class="row">
		<label for="derive-card">Card</label>
		<select id="derive-card" value={s?.card ?? ''} disabled={!s?.cards || s.busy !== null || current?.running} onchange={(e) => void derive?.setCard(e.currentTarget.value)}>
			{#if !s?.cards}<option value="">no helper</option>{/if}
			{#each s?.cards ?? [] as c (c)}<option value={c}>{c}</option>{/each}
		</select>
	</div>
	{#if s?.entries}
		<table class="mono small" aria-label="Card entries">
			<thead><tr><th scope="col">entry</th><th scope="col">value</th><th scope="col">unit</th><th scope="col">method</th><th scope="col">new value</th><th scope="col">source of the new value</th></tr></thead>
			<tbody>
				{#each s.entries as e (e.key)}
					<tr data-entry={e.key}>
						<th scope="row">{e.key}</th>
						<td>{showValue(e.value)}</td>
						<td>{e.unit ?? '—'}</td>
						<td class="method" title={e.source ?? ''}>{e.method ?? '—'}</td>
						{#if e.locked}
							<td colspan="2"><Badge text="locked" tone="warn" /> <span class="small">{e.lock?.by ?? ''} {e.lock?.on ?? ''}: changes only by the lock editor</span></td>
						{:else}
							<td>
								<input type="text" aria-label={`New value of ${e.key}`} placeholder={showValue(e.value)} value={s.inputs[e.key]?.value ?? ''} disabled={current?.running} oninput={(ev) => derive?.setInput(e.key, { value: ev.currentTarget.value })} />
							</td>
							<td>
								<input type="text" class="source" aria-label={`Source of the new value of ${e.key}`} placeholder="where this value comes from" value={s.inputs[e.key]?.source ?? ''} disabled={current?.running} oninput={(ev) => derive?.setInput(e.key, { source: ev.currentTarget.value })} />
							</td>
						{/if}
					</tr>
				{/each}
			</tbody>
		</table>
	{/if}
	<div class="buttons">
		<button type="button" class="primary" disabled={whyNoDerive !== null} title={whyNoDerive ?? 'Derive the config from this card copy under the heavy lock'} aria-describedby="why-derive" onclick={() => void derive?.derive()}>
			{s?.busy === 'derive' ? 'Starting…' : current?.running ? 'Deriving…' : 'Derive'}
		</button>
		<button type="button" disabled={whyNoCancel !== null} title={whyNoCancel ?? 'Stop the derivation and every process of it'} onclick={() => void derive?.cancel()}>Cancel</button>
		<span id="why-derive" class="why" data-derive="why">{whyNoDerive ?? ''}</span>
	</div>
	{#each s?.notices ?? [] as n, i (i)}<p class="bad small" role="alert" data-notice="derive">{n}</p>{/each}
	{#if (s?.findings.length ?? 0) > 0}
		<div class="findings" role="alert" aria-label="Lint findings">
			<p class="bad small">The card copy does not lint; nothing was derived.</p>
			<ul class="mono small">
				{#each s?.findings ?? [] as f, i (i)}<li data-finding={f.entry}>{f.file}: {f.entry}: {f.reason}</li>{/each}
			</ul>
		</div>
	{/if}
	{#if current}
		<dl class="facts mono" aria-label="Derivation">
			<dt>state</dt><dd data-derive="state"><Badge text={outcome} tone={tone(outcome)} /></dd>
			<dt>id</dt><dd>{current.id}</dd>
			{#if current.step}<dt>step</dt><dd data-derive="step">{current.step}</dd>{/if}
			{#if current.error}<dt>error</dt><dd>{current.error}</dd>{/if}
			{#if current.config_hash}<dt>config hash</dt><dd class="wrap" data-derive="config-hash">{current.config_hash}</dd>{/if}
			{#if current.schema_hash}<dt>parameter schema</dt><dd>{current.schema_hash}</dd>{/if}
		</dl>
		{#if current.running && current.tail.length > 0}
			<ul class="progress mono small" aria-label="Derivation progress">
				{#each current.tail as line, i (i)}<li>{line}</li>{/each}
			</ul>
		{/if}
	{/if}
	{#if current?.ok}
		<h4>Reports</h4>
		<div class="buttons" role="group" aria-label="Derivation reports">
			{#each current.reports as r (r)}
				<button type="button" class:active={s?.reportName === r} disabled={s?.busy !== null} onclick={() => void derive?.openReport(r)}>{r}</button>
			{/each}
		</div>
		{#if s?.reportText !== null && s?.reportName}<pre class="report mono small" aria-label={`Report ${s.reportName}`}>{s.reportText}</pre>{/if}
		<h4>Values that differ from the library image's factory config</h4>
		{#if current.changes === null}
			<p class="muted small">No image of the library has this config's parameter schema, so there is nothing to compare with.</p>
		{:else if current.changes.length === 0}
			<p class="muted small" data-derive="changes">No value differs from {current.baseline?.name} ({current.baseline?.tree}).</p>
		{:else}
			<table class="mono small" aria-label="Changed values" data-derive="changes">
				<thead><tr><th scope="col">parameter</th><th scope="col">method</th><th scope="col">from (bits)</th><th scope="col">to (bits)</th></tr></thead>
				<tbody>
					{#each current.changes as c (c.name)}
						<tr data-change={c.name}><th scope="row">{c.name}</th><td>{c.method}{c.locked ? ' (locked)' : ''}</td><td>{hex(c.from_bits)}</td><td>{hex(c.to_bits)}</td></tr>
					{/each}
				</tbody>
			</table>
		{/if}
		<h4>Hand the config to the validation and activation flow</h4>
		<div class="row">
			<label for="derive-image">Image</label>
			<select id="derive-image" value={s?.image ?? ''} disabled={s?.busy !== null} onchange={(e) => derive?.setImage(e.currentTarget.value)}>
				<option value="">pick an image</option>
				{#each current.images as i (i.hash)}<option value={i.hash}>{i.name} ({i.tree}) {shortSha(i.hash)}</option>{/each}
			</select>
			<button type="button" disabled={whyNoValidate !== null} title={whyNoValidate ?? 'Run the validator on the derived config for this image (G7)'} aria-describedby="why-derive-validate" onclick={() => void derive?.validate()}>
				{s?.validation.phase === 'validating' ? 'Validating…' : 'Validate'}
			</button>
			<span id="why-derive-validate" class="why">{whyNoValidate ?? ''}</span>
		</div>
		{#if (s?.validation.progress.length ?? 0) > 0}
			<ul class="progress mono small" aria-label="Validation progress">
				{#each s?.validation.progress ?? [] as line, i (i)}<li>{line}</li>{/each}
			</ul>
		{/if}
		{#if s?.validation.record?.report}
			<p data-derive="verdict">
				Verdict <Badge text={s.validation.record.report.verdict} tone={s.validation.record.report.verdict === 'PASS' ? 'ok' : 'bad'} />
				<span class="small mono">report {shortSha(s.validation.record.reportId)}…</span>
			</p>
		{/if}
		{#if s?.validation.error}<p class="bad small" role="alert">{s.validation.error}</p>{/if}
		<div class="buttons">
			<button type="button" disabled={whyNoStage !== null} title={whyNoStage ?? 'Stage the values that differ on the vehicle, each with its source text'} aria-describedby="why-derive-stage" onclick={() => void derive?.stage()}>
				{s?.staging.phase === 'staging' ? 'Staging…' : 'Stage on the vehicle'}
			</button>
			<a href="/params">Parameters page</a>
			<span id="why-derive-stage" class="why">{whyNoStage ?? ''}</span>
		</div>
		{#if s && s.staging.phase !== 'idle'}
			<p class="small mono" role="status" data-derive="staged">
				staged {s.staging.staged.length}{s.staging.staged.length > 0 ? ` (${s.staging.staged.join(', ')})` : ''}; {s.staging.same} already the same.
				The Parameters page validates and activates the staged config.
			</p>
			{#each s.staging.notes as n, i (i)}<p class="bad small">{n}</p>{/each}
		{/if}
	{/if}
</Panel>

<style>
	.row,
	.buttons {
		display: flex;
		flex-wrap: wrap;
		gap: 0.5rem;
		align-items: center;
		margin: 0 0 0.5rem;
	}

	label {
		color: var(--muted);
		font-size: 0.75rem;
		text-transform: uppercase;
		letter-spacing: 0.06em;
	}

	h4 {
		margin: 0.75rem 0 0.35rem;
		font-size: 0.85rem;
	}

	.facts {
		display: grid;
		grid-template-columns: max-content 1fr;
		gap: 0.2rem 1rem;
		margin: 0 0 0.5rem;
		font-size: 0.85rem;
	}

	.facts dt {
		color: var(--muted);
	}

	.facts dd {
		margin: 0;
	}

	.wrap {
		overflow-wrap: anywhere;
	}

	.why {
		font-size: 0.75rem;
		color: var(--muted);
	}

	.progress,
	.findings ul {
		list-style: none;
		margin: 0.3rem 0;
		padding: 0;
		max-height: 10rem;
		overflow: auto;
	}

	.report {
		max-height: 18rem;
		overflow: auto;
		white-space: pre;
		border: 1px solid var(--border);
		padding: 0.5rem;
	}

	table {
		border-collapse: collapse;
		margin: 0 0 0.5rem;
	}

	th,
	td {
		text-align: left;
		padding: 0.15rem 0.75rem 0.15rem 0;
		vertical-align: top;
	}

	th {
		color: var(--muted);
		font-weight: 400;
	}

	.method {
		max-width: 12rem;
		overflow: hidden;
		text-overflow: ellipsis;
		white-space: nowrap;
	}

	input.source {
		min-width: 22rem;
	}

	button.active {
		outline: 1px solid var(--accent, currentColor);
	}
</style>
