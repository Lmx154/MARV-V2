<script lang="ts">
	import { onMount } from 'svelte';
	import { readable } from 'svelte/store';
	import { app } from '$lib/state/app';
	import { boardFacts, planKey } from '$lib/flash/flash';
	import { entryKey, entryLabel } from '$lib/params/library';
	import type { SessionState } from '$link/session';
	import type { VehicleState } from '$link/vehicle';
	import Badge from '$lib/components/ui/Badge.svelte';
	import BuildPanel from '$lib/components/BuildPanel.svelte';
	import Panel from '$lib/components/ui/Panel.svelte';

	const gcs = app();
	const flash = gcs.flash;
	const manifest = gcs.manifest;
	const fstate = flash?.state ?? readable(null);
	const whyCannot = flash?.whyCannot ?? readable<string | null>('no helper');
	const sessionState = gcs.session?.state ?? readable<SessionState | null>(null);
	const vehicleState = gcs.vehicle?.state ?? readable<VehicleState | null>(null);

	let pickedKey = $state('');
	/** The plan key last asked for, so that a failed check is not repeated in a loop. */
	let tried = '';

	onMount(() => void gcs.ensureImages());

	const boards = $derived($manifest.images?.images.filter((e) => e.kind === 'board') ?? []);
	const entry = $derived(boards.find((e) => entryKey(e) === pickedKey) ?? null);
	const identity = $derived($vehicleState?.identity ?? null);
	const wanted = $derived.by(() => {
		const facts = boardFacts(identity, $sessionState?.armed ?? null);
		return entry && facts ? planKey(entry.hash, facts) : '';
	});

	$effect(() => {
		flash?.setImage(entry?.hash ?? null);
		tried = '';
	});

	// The gate is asked for whenever the image or the vehicle's config changes.
	$effect(() => {
		if (wanted && wanted !== tried && !$fstate?.busy) {
			tried = wanted;
			void flash?.check();
		}
	});

	const plan = $derived($fstate?.plan ?? null);
	const busy = $derived($fstate?.busy ?? false);
	const whyValidate = $derived.by(() => {
		if (!entry) return 'pick an image';
		if (busy) return 'a check, a validation or a flash is in progress';
		if (!plan) return 'the gate has not been checked';
		if (plan.gate.passes) return 'a passing report is stored already';
		if (!plan.can_validate) return 'the helper holds no copy of the config the board will run: stage and validate it from the Parameters page';
		return null;
	});
	const armed = $derived($sessionState?.armed);
</script>

<svelte:head><title>Flash · MARV GCS</title></svelte:head>

<h1>Flash</h1>
<p class="muted lead">
	An image is flashed only with a passing validation report for the image and the config the board will run with it (G7). The helper runs
	picotool; the vehicle must be disarmed and grants maintenance first; the image hash it reports afterwards is checked here.
</p>

<div class="grid">
	<BuildPanel />

	<Panel title="Image" label="library">
		<div class="row">
			<label for="flash-image">Board image</label>
			<select id="flash-image" bind:value={pickedKey} disabled={busy}>
				<option value="">pick an image</option>
				{#each boards as e (entryKey(e))}<option value={entryKey(e)}>{entryLabel(e)}</option>{/each}
			</select>
			<button type="button" onclick={() => void gcs.refreshImages()} disabled={$manifest.imagesState === 'loading' || busy}>Rescan images</button>
		</div>
		{#if boards.length === 0 && $manifest.imagesState === 'ready'}
			<p class="muted">No board image in the library: build the m33 preset and rescan.</p>
		{/if}
		{#if entry}
			<dl class="facts mono">
				<dt>image hash</dt><dd class="wrap">{entry.hash}</dd>
				<dt>build tree</dt><dd>{entry.tree}</dd>
				<dt>vehicle runs</dt><dd class="wrap">{identity ? identity.imageSha256 : '—'}</dd>
			</dl>
		{/if}
	</Panel>

	<Panel title="Gate" label="G7">
		{#if !entry}
			<p class="muted">Pick an image.</p>
		{:else if $fstate?.planError}
			<p class="bad" role="alert">The gate could not be checked: {$fstate.planError}</p>
		{:else if !plan}
			<p class="muted">{identity ? 'Checking the gate…' : 'Connect to the vehicle: the config it will run depends on what the vehicle holds.'}</p>
		{:else}
			<dl class="facts mono">
				<dt>config it will run</dt><dd>{plan.config.which} <span class="wrap">{plan.config.hash}</span></dd>
				<dt>why</dt><dd>{plan.config.why}</dd>
				<dt>validation report</dt>
				<dd>
					{#if plan.gate.passes}<Badge text="PASS" tone="ok" /> <span class="small">report {plan.gate.report_id?.slice(0, 16)}…</span>
					{:else}<Badge text={plan.gate.verdict ?? 'none'} tone="bad" /> <span class="small">{plan.gate.reason}</span>{/if}
				</dd>
				<dt>vehicle</dt>
				<dd>
					{#if armed === true}<Badge text="armed" tone="bad" />{:else if armed === false}<Badge text="disarmed" tone="ok" />{:else}<Badge text="unknown" tone="warn" />{/if}
				</dd>
			</dl>
			<div class="buttons">
				<button type="button" disabled={whyValidate !== null} title={whyValidate ?? 'Run the validator on the config the board will run, for this image'} aria-describedby="why-validate" onclick={() => void flash?.validate()}>
					{$fstate?.phase === 'validating' ? 'Validating…' : 'Validate'}
				</button>
				<span id="why-validate" class="why">{whyValidate ?? ''}</span>
			</div>
			{#if ($fstate?.validateProgress.length ?? 0) > 0}
				<ul class="progress mono small" aria-label="Validation progress">
					{#each $fstate?.validateProgress ?? [] as line, i (i)}<li>{line}</li>{/each}
				</ul>
			{/if}
		{/if}
	</Panel>

	<Panel title="Flash" label="picotool">
		<div class="buttons">
			<button type="button" class="primary" disabled={$whyCannot !== null} title={$whyCannot ?? 'Flash the image onto the vehicle'} aria-describedby="why-flash" onclick={() => void flash?.flash()}>
				{#if $fstate?.phase === 'checking'}Checking the gate…{:else if $fstate?.phase === 'granting'}Asking for maintenance…{:else if $fstate?.phase === 'flashing'}Flashing…{:else if $fstate?.phase === 'reconnecting'}Waiting for the vehicle…{:else}Flash{/if}
			</button>
			<span id="why-flash" class="why">{$whyCannot ?? ''}</span>
		</div>
		{#if $fstate?.error}<p class="bad" role="alert">{$fstate.error}</p>{/if}
		{#if $fstate?.inMaintenance}<p class="warn small" role="status">The vehicle holds the maintenance grant and refuses to arm until it restarts.</p>{/if}
		{#if ($fstate?.progress.length ?? 0) > 0}
			<ul class="progress mono small" aria-label="Flash progress">
				{#each $fstate?.progress ?? [] as line, i (i)}<li>{line}</li>{/each}
			</ul>
		{/if}
		{#if $fstate?.result}
			<div class="result" aria-label="Read-back">
				<p>
					<Badge text={$fstate.result.verified ? 'verified' : 'MISMATCH'} tone={$fstate.result.verified ? 'ok' : 'bad'} />
					<span class="small">read back over the link after the flash</span>
				</p>
				<table class="mono small">
					<thead><tr><th>what</th><th>expected</th><th>vehicle reports</th><th></th></tr></thead>
					<tbody>
						{#each $fstate.result.checks as c (c.what)}
							<tr><td>{c.what}</td><td class="wrap">{c.expected}</td><td class="wrap">{c.got}</td><td><Badge text={c.ok ? 'equal' : 'differs'} tone={c.ok ? 'ok' : 'bad'} /></td></tr>
						{/each}
					</tbody>
				</table>
				<p class="small muted">
					config state {$fstate.result.configState}{$fstate.result.configRefusal !== 'none' ? ` (${$fstate.result.configRefusal})` : ''}: a stored config bound to the previous image
					is refused until it is activated again for this one.
				</p>
			</div>
		{/if}
	</Panel>
</div>

<style>
	.lead {
		margin: 0.2rem 0 0.75rem;
	}

	.grid {
		display: grid;
		grid-template-columns: repeat(auto-fit, minmax(30rem, 1fr));
		gap: 0.75rem;
		align-items: start;
	}

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

	.progress {
		list-style: none;
		margin: 0.3rem 0;
		padding: 0;
		max-height: 10rem;
		overflow: auto;
	}

	.result table {
		border-collapse: collapse;
		width: 100%;
	}

	.result th,
	.result td {
		text-align: left;
		padding: 0.2rem 0.5rem 0.2rem 0;
		border-bottom: 1px solid var(--border);
		vertical-align: top;
	}

	.warn {
		color: var(--warn);
	}
</style>
