<script lang="ts">
	import { derived, readable } from 'svelte/store';
	import { app } from '$lib/state/app';
	import type { SessionState } from '$link/session';
	import type { VehicleState } from '$link/vehicle';
	import { matchesFilter, mergeParams, type ParamRow as Row } from '$lib/params/merge';
	import type { StageContext } from '$lib/params/staging';
	import { entryLabel } from '$lib/params/library';
	import ParamRow from '$lib/components/ParamRow.svelte';
	import StagedPanel from '$lib/components/StagedPanel.svelte';
	import Badge from '$lib/components/ui/Badge.svelte';
	import Chip from '$lib/components/ui/Chip.svelte';

	const gcs = app();
	const manifest = gcs.manifest;
	const vehicle = gcs.vehicle?.state ?? readable<VehicleState | null>(null);
	const session = gcs.session?.state ?? readable<SessionState | null>(null);

	let filter = $state('');
	let onlyDiffs = $state(false);
	let onlyLocked = $state(false);
	let onlyDerived = $state(false);
	let selectedHash = $state('');

	const merged = derived([manifest, vehicle], ([m, v]) =>
		mergeParams(m.loaded?.manifest ?? null, m.loaded?.provenance ?? null, v && v.params.length > 0 ? v.params : null)
	);

	const ctx = $derived<StageContext>({
		connected: $session?.heartbeat === 'alive',
		identityKnown: $vehicle?.identity != null,
		manifestMatches: Boolean($vehicle?.identity && $manifest.loaded && $manifest.loaded.manifest.schemaHash === $vehicle.identity.schemaHash),
		bytewise: $vehicle?.bytewise ?? null,
		armed: $session?.armed ?? null
	});

	const visible = $derived(
		$merged.groups
			.map((g) => ({
				name: g.name,
				rows: g.rows.filter(
					(r: Row) => matchesFilter(r, filter) && (!onlyDiffs || r.differs) && (!onlyLocked || r.locked) && (!onlyDerived || r.method === 'derived')
				)
			}))
			.filter((g) => g.rows.length > 0)
	);
	const shown = $derived(visible.reduce((n, g) => n + g.rows.length, 0));
	const staged = $derived($merged.rows.filter((r: Row) => r.differs));
	const progress = $derived.by(() => {
		const v = $vehicle;
		if (!v || v.count === null) return null;
		return { have: v.params.filter((p) => p.linkId !== null && p.hasRecord).length, count: v.count };
	});

	async function stage(row: Row, bits: number, source: string) {
		if (!gcs.vehicle) throw new Error('no vehicle link');
		return gcs.vehicle.stage(row.linkId, row.type, bits, source);
	}

	async function readSource(row: Row): Promise<void> {
		if (row.index === null || !gcs.vehicle) return;
		try {
			await gcs.vehicle.readSource(row.index);
		} catch {
			// the row keeps showing the manifest's text; the button offers a retry
		}
	}

	async function pick(hash: string): Promise<void> {
		selectedHash = hash;
		const entry = $manifest.images?.images.find((e) => e.hash === hash && e.artifacts.params_manifest) ?? null;
		await gcs.selectImage(entry);
	}

	const withManifest = $derived($manifest.images?.images.filter((e) => e.artifacts.params_manifest) ?? []);
</script>

<div class="head">
	<h1>Parameters</h1>
	{#if $manifest.loaded}
		<span class="muted">
			manifest {$manifest.loaded.manifest.schemaHash} · {$manifest.loaded.manifest.count} parameters · from
			<span class="mono">{entryLabel($manifest.loaded.entry)}</span> (by {$manifest.loaded.how})
		</span>
	{/if}
</div>

<div class="toolbar">
	<label for="image-pick">Parameter set</label>
	<select id="image-pick" value={selectedHash} onchange={(e) => void pick(e.currentTarget.value)}>
		<option value="">automatic (the connected image)</option>
		{#each withManifest as e (e.hash + e.tree)}
			<option value={e.hash}>{entryLabel(e)}</option>
		{/each}
	</select>
	<button type="button" onclick={() => void gcs.refreshImages()}>Rescan images</button>
	<button type="button" class="primary" disabled={$session?.heartbeat !== 'alive' || $vehicle?.listing === 'loading'} onclick={() => void gcs.loadParams()}>
		{$vehicle?.listing === 'loading' ? 'Reading…' : 'Read parameters from the vehicle'}
	</button>
	{#if progress}
		<span class="mono muted" role="status">{progress.have} / {progress.count}</span>
	{/if}
	{#if $vehicle?.listing === 'incomplete'}<Badge text={`incomplete: ${$vehicle.missing.length} missing`} tone="bad" />{/if}
	{#if $manifest.loading}<span class="muted">loading the manifest…</span>{/if}
</div>

{#if $manifest.error}<p class="warn" role="alert">{$manifest.error}</p>{/if}
{#each $merged.issues as issue (issue)}<p class="warn">{issue}</p>{/each}
{#if $vehicle?.identity && $manifest.loaded && !ctx.manifestMatches}
	<p class="bad" role="alert">
		The manifest on screen has schema {$manifest.loaded.manifest.schemaHash}, the vehicle reports {$vehicle.identity.schemaHash}: all rows are read-only.
	</p>
{/if}

<div class="filters">
	<label for="filter" class="visually-hidden">Filter parameters</label>
	<input id="filter" type="search" placeholder="filter by name, unit, method, source text…" bind:value={filter} />
	<Chip label="staged differs" active={onlyDiffs} onclick={() => (onlyDiffs = !onlyDiffs)} />
	<Chip label="locked" active={onlyLocked} onclick={() => (onlyLocked = !onlyLocked)} />
	<Chip label="derived" active={onlyDerived} onclick={() => (onlyDerived = !onlyDerived)} />
	<span class="muted mono">{shown} / {$merged.rows.length} shown</span>
</div>

<div class="layout">
	<div class="tables">
		{#if $merged.rows.length === 0}
			<p class="muted">No parameters: connect a vehicle (Connection) or pick a parameter set above to browse the manifest.</p>
		{/if}
		{#each visible as g (g.name)}
			<section>
				<h2 class="mono">{g.name} <span class="muted">({g.rows.length})</span></h2>
				<table>
					<thead>
						<tr>
							<th scope="col">Name / link id</th>
							<th scope="col" class="r">Active</th>
							<th scope="col" class="r">Staged</th>
							<th scope="col">Unit</th>
							<th scope="col">Method</th>
							<th scope="col">Origin</th>
							<th scope="col">σ</th>
							<th scope="col">Flags</th>
						</tr>
					</thead>
					<tbody>
						{#each g.rows as r (r.key)}
							<ParamRow row={r} {ctx} result={$vehicle?.results[r.linkId]} onstage={stage} onreadsource={readSource} />
						{/each}
					</tbody>
				</table>
			</section>
		{/each}
	</div>
	<aside>
		<StagedPanel
			rows={staged}
			onrevert={() => void gcs.vehicle?.configCommand('revert')}
			onreset={() => void gcs.vehicle?.configCommand('reset-factory')}
		/>
		{#if ($vehicle?.notices.length ?? 0) > 0}
			<div class="notices">
				<h3>Vehicle messages</h3>
				<ul class="mono">
					{#each [...($vehicle?.notices ?? [])].reverse().slice(0, 20) as n (n.at + n.text)}
						<li>{n.text}</li>
					{/each}
				</ul>
			</div>
		{/if}
	</aside>
</div>

<style>
	.head {
		display: flex;
		gap: 1rem;
		align-items: baseline;
		flex-wrap: wrap;
		margin-bottom: 0.5rem;
	}

	.toolbar,
	.filters {
		display: flex;
		gap: 0.5rem;
		align-items: center;
		flex-wrap: wrap;
		margin-bottom: 0.5rem;
	}

	label {
		color: var(--muted);
		font-size: 0.75rem;
		text-transform: uppercase;
		letter-spacing: 0.06em;
	}

	.filters input {
		flex: 1 1 18rem;
		max-width: 32rem;
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

	aside {
		position: sticky;
		top: 8rem;
		display: flex;
		flex-direction: column;
		gap: 0.75rem;
	}

	section {
		margin-bottom: 1rem;
	}

	h2 {
		margin: 0.5rem 0 0.25rem;
		font-size: 0.85rem;
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

	.r {
		text-align: right;
	}

	.notices ul {
		list-style: none;
		margin: 0.25rem 0 0;
		padding: 0;
		font-size: 0.75rem;
		max-height: 12rem;
		overflow: auto;
	}

	.notices li {
		padding: 0.1rem 0;
		border-bottom: 1px solid var(--border);
	}
</style>
