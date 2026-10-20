<script lang="ts">
	import { derived, readable } from 'svelte/store';
	import { app } from '$lib/state/app';
	import { stageContextOf, whyCannotCommand, whyCannotRead, whyCannotValidate, whyCannotWrite } from '$lib/state/gating';
	import type { SessionState } from '$link/session';
	import type { VehicleState } from '$link/vehicle';
	import type { ValidationState } from '$lib/state/validation_state';
	import { whyCannotBuildView } from '$lib/params/validation';
	import { DERIVED_WARNING, lockTooltip, whyReadOnly } from '$lib/params/staging';
	import type { ParamRow } from '$lib/params/merge';
	import { valueTitle } from '$link/bits';
	import {
		activeText, groupsOf, planWrite, stagedText, whyCannotSend, writeChanges, type TuningItem, type WriteReport
	} from '$lib/state/tuning_state';
	import StagedPanel from '$lib/components/StagedPanel.svelte';
	import Badge from '$lib/components/ui/Badge.svelte';
	import Panel from '$lib/components/ui/Panel.svelte';

	const gcs = app();
	const manifest = gcs.manifest;
	const vehicle = gcs.vehicle?.state ?? readable<VehicleState | null>(null);
	const session = gcs.session?.state ?? readable<SessionState | null>(null);
	const helperState = gcs.helper?.state ?? readable('closed');
	const device = gcs.device;
	const validation = gcs.validation;
	const validationState = validation?.state ?? readable<ValidationState | null>(null);
	const fingerprint = validation?.fingerprint ?? readable('');
	const paramRows = gcs.paramRows;

	let edits = $state<Record<string, string>>({});
	let reason = $state('');
	let derivedAck = $state(false);
	let confirmedFor = $state<string | null>(null);
	let writing = $state(false);
	let report = $state<WriteReport | null>(null);
	let autoRead = $state(false);

	const ctx = $derived(stageContextOf($session, $vehicle, $manifest.loaded?.manifest.schemaHash ?? null));
	const groups = $derived(groupsOf($paramRows));
	const plan = $derived(planWrite(groups, ctx, edits, reason));
	const linkBlock = $derived(
		whyCannotWrite(ctx, $vehicle?.listing, { manifest: $manifest.loaded?.manifest.schemaHash ?? null, vehicle: $vehicle?.identity?.schemaHash ?? null })
	);
	const planBlock = $derived(whyCannotSend(plan, derivedAck, confirmedFor));
	const blocked = $derived(linkBlock ?? planBlock);
	const confirmed = $derived(plan.safety.length > 0 && confirmedFor === plan.confirmKey);
	const readBlocked = $derived(whyCannotRead({ helper: $helperState, device: $device, heartbeat: $session?.heartbeat, listing: $vehicle?.listing }));

	// As on the Radio page: the parameters are read once when the page finds a live vehicle without them (reading changes nothing on the vehicle).
	$effect(() => {
		if (($vehicle?.params.length ?? 0) > 0) autoRead = false;
		else if (!autoRead && readBlocked === null && $vehicle?.listing === 'idle' && $vehicle.identity) {
			autoRead = true;
			void gcs.loadParams();
		}
	});

	const staged = $derived($paramRows.filter((r: ParamRow) => r.differs));

	const whyValidate = $derived(
		whyCannotValidate({
			hasHelper: validation !== null,
			ctx,
			manifestLoaded: $manifest.loaded !== null,
			listing: $vehicle?.listing,
			unreadable: $manifest.loaded ? whyCannotBuildView($manifest.loaded.manifest, $vehicle?.params ?? [], false) : null
		})
	);
	const whyActivate = $derived(
		validation && $vehicle && $validationState ? validation.whyCannotActivate($vehicle, $validationState, $fingerprint) : 'no validation report: validate the staged config first'
	);
	const whyCommands = $derived(whyCannotCommand(ctx));
	const idleValidation: ValidationState = { phase: 'idle', progress: [], record: null, error: null, computed: null, command: null, busy: false };

	async function reloadAfter(action: 'revert' | 'reset-factory'): Promise<void> {
		const result = await validation?.command(action);
		if (result === 'accepted') await gcs.loadParams();
	}

	async function write(): Promise<void> {
		const vehicleClient = gcs.vehicle;
		if (!vehicleClient || writing) return;
		writing = true;
		try {
			const done = await writeChanges({
				plan,
				linkBlocker: linkBlock,
				derivedAck,
				confirmedKey: confirmedFor,
				stage: (linkId, type, bits, source) => vehicleClient.stage(linkId, type, bits, source)
			});
			report = done;
			for (const w of done.written) if (w.accepted) delete edits[w.name];
			if (done.refused === null && done.unsent.length === 0 && done.written.every((w) => w.accepted)) {
				reason = '';
				derivedAck = false;
			}
			confirmedFor = null;
		} finally {
			writing = false;
		}
	}

	const labelOf = (name: string): string => groups.flatMap((g) => g.items).find((i) => i.name === name)?.label ?? name;
	const changed = (i: TuningItem): boolean => i.name !== null && plan.changes.some((c) => c.name === i.name);
	const problemOf = (i: TuningItem): string | null => plan.problems.find((p) => p.name === i.name)?.error ?? null;
</script>

<svelte:head><title>Tuning · MARV GCS</title></svelte:head>

<h1>Tuning</h1>
<p class="muted lead">
	The tuning parameters of the vehicle in four groups, as the vehicle reports them. A value you type is staged on the vehicle with your reason as its
	source text; validation and activation are the same as on the Parameters page (the staged changes on the right). Nothing is written while the vehicle is armed.
</p>

<div class="toolbar">
	<button
		type="button"
		class="primary"
		disabled={readBlocked !== null}
		title={readBlocked ? `Unavailable: ${readBlocked}` : 'Request the parameter list and the provenance records from the vehicle'}
		onclick={() => void gcs.loadParams()}
	>
		{$vehicle?.listing === 'loading' ? 'Reading…' : 'Read parameters from the vehicle'}
	</button>
	<span class="muted small" data-link-gate role="status">{linkBlock ?? 'the vehicle is connected and disarmed: values can be written'}</span>
</div>

<div class="layout">
	<div class="groups">
		{#each groups as g (g.id)}
			<Panel title={g.title} label={g.safety ? 'safety parameters' : undefined}>
				<section data-group={g.id}>
					<p class="muted small">{g.about}</p>
					{#if g.safety}
						<p class="safety" role="note" data-safety-note>
							Safety numbers. They are shown here; a change needs your explicit confirmation below before it is written. Locked entries change only by a
							firmware build.
						</p>
						<p class="muted small" data-motor-check>
							Motor order and direction are checked with the console motor test, props off (decision 0038), on the <a href="/console">Console</a> page. This
							page shows the motor map and sends no motor command.
						</p>
					{/if}
					<table aria-label={g.title}>
						<thead>
							<tr>
								<th scope="col">Parameter</th>
								<th scope="col" class="r">Current</th>
								<th scope="col" class="r">Staged</th>
								<th scope="col">New value</th>
								<th scope="col">Unit</th>
								<th scope="col">Provenance</th>
								<th scope="col">Lock</th>
							</tr>
						</thead>
						<tbody>
							{#each g.items as i (i.label)}
								{#if i.row === null}
									<tr class="absent" data-item={i.name ?? i.label} data-absent>
										<th scope="row">{i.label}{#if i.name}<span class="name mono">{i.name}</span>{/if}</th>
										<td colspan="6" class="muted">{i.absent}{#if i.note}: {i.note}{/if}</td>
									</tr>
								{:else}
									{@const row = i.row}
									{@const readOnly = whyReadOnly(row, ctx)}
									<tr class:differs={row.differs} class:locked={row.locked} class:pending={changed(i)} data-item={i.name} data-locked={row.locked}>
										<th scope="row">
											{i.label}
											<span class="name mono">{row.name}</span>
											{#if i.note}<span class="muted small">{i.note}</span>{/if}
										</th>
										<td class="r mono" data-cell="current" title={row.activeBits === null ? undefined : valueTitle(row.activeBits, row.type)}>{activeText(row)}</td>
										<td class="r mono" class:diff={row.differs} data-cell="staged" title={row.stagedBits === null ? undefined : valueTitle(row.stagedBits, row.type)}>
											{stagedText(row)}{#if row.differs}<span class="visually-hidden"> (differs from the current value)</span>{/if}
										</td>
										<td>
											<input
												class="mono"
												type="text"
												inputmode="decimal"
												autocomplete="off"
												spellcheck="false"
												aria-label={`New value of ${row.name}`}
												placeholder={readOnly ? '' : activeText(row)}
												disabled={readOnly !== null}
												title={readOnly ?? undefined}
												value={edits[row.name] ?? ''}
												oninput={(e) => {
													const v = e.currentTarget.value;
													if (v === '') delete edits[row.name];
													else edits[row.name] = v;
												}}
											/>
											{#if problemOf(i) && !readOnly}<span class="bad small" role="alert">{problemOf(i)}</span>{/if}
										</td>
										<td class="mono">{row.unit || '—'}</td>
										<td class="small">
											<span class="mono">{row.method ?? '—'} · {row.origin ?? '—'}</span>
											<details>
												<summary>source</summary>
												<p class="src">{row.source ?? '(the vehicle has not sent the source text)'}</p>
												{#if row.method === 'derived'}<p class="muted">{DERIVED_WARNING}</p>{/if}
											</details>
										</td>
										<td class="small" data-cell="lock">
											{#if row.locked}
												<Badge text="locked" tone="warn" />
												<span class="muted" data-lock-reason>{lockTooltip(row)}</span>
											{:else}
												<span class="muted">not locked</span>
											{/if}
										</td>
									</tr>
								{/if}
							{/each}
						</tbody>
					</table>
				</section>
			</Panel>
		{/each}

		<Panel title="Write to the vehicle" label="staged on the vehicle">
			{#if plan.changes.length === 0}
				<p class="muted" data-changes="0">Type a new value in a row above. Only the values you change are written.</p>
			{:else}
				<ul class="changes mono" aria-label="Values to write" data-changes={plan.changes.length}>
					{#each plan.changes as c (c.name)}
						<li data-change={c.name} data-safety={c.safety}>
							<span>{c.label} <span class="muted">({c.name})</span></span>
							<span>{c.from} → <strong>{c.to}</strong> {c.unit}</span>
						</li>
					{/each}
				</ul>
			{/if}

			<div class="field">
				<label for="tuning-reason">Why (stored with each value as its source text)</label>
				<textarea id="tuning-reason" rows="2" bind:value={reason} placeholder="e.g. bench hover on this frame; the response was sluggish"></textarea>
			</div>

			{#if plan.needsDerivedAck}
				<div class="derived" role="note">
					<strong>Derived values.</strong> {DERIVED_WARNING}
					<label class="check"><input type="checkbox" bind:checked={derivedAck} /> I understand</label>
				</div>
			{/if}

			{#if plan.safety.length > 0}
				<div class="confirm" role="group" aria-label="Confirm safety parameters" data-confirm>
					<strong>Safety parameters will change.</strong>
					<ul class="mono">
						{#each plan.safety as c (c.name)}
							<li data-confirm-item={c.name}>{c.name}: {c.from} → <strong>{c.to}</strong> {c.unit}</li>
						{/each}
					</ul>
					<label class="check">
						<input type="checkbox" checked={confirmed} onchange={(e) => (confirmedFor = e.currentTarget.checked ? plan.confirmKey : null)} />
						I confirm that I change {plan.safety.map((c) => c.name).join(', ')} as shown
					</label>
				</div>
			{/if}

			<div class="buttons">
				<button type="button" class="primary" disabled={blocked !== null || writing} title={blocked ?? 'Stage the changed values on the vehicle'} aria-describedby="write-why" onclick={() => void write()}>
					{writing ? 'Writing…' : 'Write to vehicle'}
				</button>
				<span id="write-why" class="muted small" data-write-why>{blocked ?? ''}</span>
			</div>

			{#if report}
				<div class="result" role="status" data-result>
					{#if report.refused}
						<p class="bad">not written: {report.refused}</p>
					{:else}
						<ul class="mono">
							{#each report.written as w (w.name)}
								<li data-written={w.name} data-accepted={w.accepted}>
									{labelOf(w.name)}: {w.accepted ? 'staged' : `refused: ${w.why.join('; ')}`}{w.sourceFailed ? ' (the source text did not read back)' : ''}
								</li>
							{/each}
						</ul>
						{#if report.unsent.length > 0}<p class="bad">not sent after the failure: {report.unsent.map(labelOf).join(', ')}</p>{/if}
					{/if}
				</div>
			{/if}
		</Panel>
	</div>

	<aside>
		<StagedPanel
			rows={staged}
			validation={$validationState ?? idleValidation}
			{whyValidate}
			{whyActivate}
			{whyCommands}
			onvalidate={() => void validation?.validate()}
			onactivate={() => void validation?.activate()}
			onrevert={() => void reloadAfter('revert')}
			onreset={() => void reloadAfter('reset-factory')}
		/>
		{#if $vehicle?.identity && $manifest.loaded && !ctx.manifestMatches}
			<p class="bad" role="alert">
				The manifest on screen has schema {$manifest.loaded.manifest.schemaHash}, the vehicle reports {$vehicle.identity.schemaHash}: nothing can be written.
			</p>
		{/if}
	</aside>
</div>

<style>
	.lead {
		margin: 0.2rem 0 0.75rem;
	}

	.small {
		font-size: 0.75rem;
	}

	.toolbar,
	.buttons {
		display: flex;
		gap: 0.5rem;
		align-items: center;
		flex-wrap: wrap;
		margin-bottom: 0.5rem;
	}

	.layout {
		display: grid;
		grid-template-columns: minmax(0, 1fr) 21rem;
		gap: 1rem;
		align-items: start;
	}

	@media (max-width: 72rem) {
		.layout {
			grid-template-columns: minmax(0, 1fr);
		}
	}

	.groups {
		display: flex;
		flex-direction: column;
		gap: 0.75rem;
	}

	aside {
		position: sticky;
		top: calc(var(--header-h, 7rem) + 0.5rem);
		display: flex;
		flex-direction: column;
		gap: 0.75rem;
	}

	table {
		width: 100%;
		border-collapse: collapse;
		background: var(--surface);
		border: 1px solid var(--border);
	}

	thead th {
		text-align: left;
		font-size: 0.65rem;
		text-transform: uppercase;
		letter-spacing: 0.08em;
		color: var(--muted);
		padding: 0.25rem 0.5rem;
		border-bottom: 1px solid var(--border-strong);
		background: var(--surface-2);
	}

	tbody th,
	tbody td {
		padding: 0.25rem 0.5rem;
		border-bottom: 1px solid var(--border);
		text-align: left;
		vertical-align: top;
		font-weight: normal;
	}

	.r {
		text-align: right;
	}

	.name {
		display: block;
		font-size: 0.7rem;
		color: var(--muted);
	}

	tr.absent th {
		color: var(--muted);
	}

	tr.pending {
		background: var(--accent-bg);
	}

	td.diff {
		color: var(--warn);
	}

	td input {
		width: 8rem;
	}

	.src {
		margin: 0.2rem 0;
		max-width: 28rem;
		overflow-wrap: anywhere;
	}

	.safety,
	.confirm,
	.derived {
		border: 1px solid var(--warn);
		border-radius: 4px;
		padding: 0.4rem 0.6rem;
		margin: 0.5rem 0;
	}

	.confirm ul,
	.changes,
	.result ul {
		list-style: none;
		margin: 0.3rem 0;
		padding: 0;
		font-size: 0.8rem;
	}

	.changes li {
		display: flex;
		justify-content: space-between;
		gap: 1rem;
		padding: 0.1rem 0;
		border-bottom: 1px solid var(--border);
	}

	.check {
		display: flex;
		gap: 0.4rem;
		align-items: center;
		margin-top: 0.3rem;
		text-transform: none;
		letter-spacing: normal;
		font-size: 0.8rem;
		color: var(--text);
	}

	.field {
		display: flex;
		flex-direction: column;
		gap: 0.2rem;
		margin: 0.5rem 0;
	}

	.field label {
		color: var(--muted);
		font-size: 0.75rem;
		text-transform: uppercase;
		letter-spacing: 0.06em;
	}
</style>
