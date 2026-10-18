<script lang="ts">
	import { onMount } from 'svelte';
	import { derived, readable } from 'svelte/store';
	import { app } from '$lib/state/app';
	import { whyCannotRead } from '$lib/state/gating';
	import type { SessionState } from '$link/session';
	import type { VehicleState } from '$link/vehicle';
	import { matchesFilter, mergeParams, type ParamRow as Row } from '$lib/params/merge';
	import type { StageContext } from '$lib/params/staging';
	import { entryKey, entryLabel, explainManifest } from '$lib/params/library';
	import { countRows, nextSort, sortRows, type Sort, type SortKey } from '$lib/params/view';
	import ParamRow from '$lib/components/ParamRow.svelte';
	import StagedPanel from '$lib/components/StagedPanel.svelte';
	import Badge from '$lib/components/ui/Badge.svelte';
	import Chip from '$lib/components/ui/Chip.svelte';

	const gcs = app();
	const manifest = gcs.manifest;
	const vehicle = gcs.vehicle?.state ?? readable<VehicleState | null>(null);
	const session = gcs.session?.state ?? readable<SessionState | null>(null);
	const helperState = gcs.helper?.state ?? readable('closed');
	const device = gcs.device;

	let filter = $state('');
	let filterInput = $state<HTMLInputElement | null>(null);
	let onlyDiffs = $state(false);
	let onlyLocked = $state(false);
	let onlyDerived = $state(false);
	let sort = $state<Sort | null>(null);
	let dense = $state(false);
	let grouped = $state(true);
	let collapsed = $state<Record<string, boolean>>({});

	onMount(() => {
		dense = localStorage.paramDensity === 'compact';
		void gcs.ensureImages();
	});

	function setDense(next: boolean): void {
		dense = next;
		if (next) localStorage.paramDensity = 'compact';
		else localStorage.removeItem('paramDensity');
	}

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

	const haveVehicle = $derived(($vehicle?.params.length ?? 0) > 0);
	const counts = $derived(countRows($merged.rows));
	const filtered = $derived(
		$merged.rows.filter((r: Row) => matchesFilter(r, filter) && (!onlyDiffs || r.differs) && (!onlyLocked || r.locked) && (!onlyDerived || r.method === 'derived'))
	);
	const visible = $derived.by(() => {
		if (!grouped) return filtered.length ? [{ name: 'all parameters', rows: sortRows(filtered, sort) }] : [];
		return $merged.groups
			.map((g) => ({ name: g.name, rows: sortRows(g.rows.filter((r) => filtered.includes(r)), sort) }))
			.filter((g) => g.rows.length > 0);
	});
	const staged = $derived($merged.rows.filter((r: Row) => r.differs));
	const progress = $derived.by(() => {
		const v = $vehicle;
		if (!v || v.count === null) return null;
		return { have: v.params.filter((p) => p.linkId !== null && p.hasRecord).length, count: v.count };
	});
	const readBlocked = $derived(
		whyCannotRead({ helper: $helperState, device: $device, heartbeat: $session?.heartbeat, listing: $vehicle?.listing })
	);
	const why = $derived(
		explainManifest(
			{ loaded: $manifest.loaded, choice: $manifest.choice, error: $manifest.error, loading: $manifest.loading },
			$vehicle?.identity ?? null
		)
	);
	const withManifest = $derived($manifest.images?.images.filter((e) => e.artifacts.params_manifest) ?? []);
	const pickedKey = $derived($manifest.choice.kind === 'picked' ? $manifest.choice.key : '');
	const pickedMissing = $derived(pickedKey !== '' && !withManifest.some((e) => entryKey(e) === pickedKey));

	async function stage(row: Row, bits: number, source: string) {
		if (!gcs.vehicle) throw new Error('no vehicle link');
		return gcs.vehicle.stage(row.linkId, row.type, bits, source);
	}

	async function retry(row: Row): Promise<void> {
		if (!gcs.vehicle) return;
		try {
			await gcs.vehicle.retrySource(row.linkId);
		} catch {
			// the row shows the failed state; the button offers another retry
		}
	}

	async function readSource(row: Row): Promise<void> {
		if (row.index === null || !gcs.vehicle) return;
		try {
			await gcs.vehicle.readSource(row.index);
		} catch {
			// the row keeps showing the manifest's text; the button offers a retry
		}
	}

	async function pick(key: string): Promise<void> {
		const entry = withManifest.find((e) => entryKey(e) === key) ?? null;
		await gcs.selectImage(entry);
	}

	function sortBy(key: SortKey): void {
		sort = nextSort(sort, key);
	}

	const ariaSort = (key: SortKey): 'ascending' | 'descending' | 'none' =>
		sort?.key === key ? (sort.direction === 'asc' ? 'ascending' : 'descending') : 'none';
	const arrow = (key: SortKey): string => (sort?.key === key ? (sort.direction === 'asc' ? ' ▴' : ' ▾') : '');

	function setAll(value: boolean): void {
		collapsed = Object.fromEntries(visible.map((g) => [g.name, value]));
	}

	function onTableKey(e: KeyboardEvent): void {
		if (e.key !== 'ArrowDown' && e.key !== 'ArrowUp') return;
		const target = e.target as HTMLElement;
		if (!target.matches('[data-nav]')) return;
		const all = [...(e.currentTarget as HTMLElement).querySelectorAll<HTMLElement>('[data-nav]')];
		const next = all[all.indexOf(target) + (e.key === 'ArrowDown' ? 1 : -1)];
		if (next) {
			e.preventDefault();
			next.focus();
		}
	}

	function onWindowKey(e: KeyboardEvent): void {
		const target = e.target as HTMLElement | null;
		const typing = target?.matches('input, textarea, select') ?? false;
		if (e.key === '/' && !typing && !e.ctrlKey && !e.metaKey && !e.altKey) {
			e.preventDefault();
			filterInput?.focus();
		}
	}
</script>

<svelte:head><title>Parameters · MARV GCS</title></svelte:head>
<svelte:window onkeydown={onWindowKey} />

<div class="head">
	<h1>Parameters</h1>
	{#if $manifest.loaded}
		<span class="muted">{$manifest.loaded.manifest.schemaHash} · {$manifest.loaded.manifest.count} parameters</span>
	{/if}
</div>

<div class="toolbar">
	<label for="image-pick">Parameter set</label>
	<select id="image-pick" value={pickedKey} onchange={(e) => void pick(e.currentTarget.value)}>
		<option value="">automatic (the connected image)</option>
		{#if pickedMissing}<option value={pickedKey} disabled>(picked set is no longer in the library)</option>{/if}
		{#each withManifest as e (entryKey(e))}
			<option value={entryKey(e)}>{entryLabel(e)}</option>
		{/each}
	</select>
	<button type="button" onclick={() => void gcs.refreshImages()} disabled={$manifest.imagesState === 'loading'}>
		{$manifest.imagesState === 'loading' ? 'Scanning…' : 'Rescan images'}
	</button>
	<span class="muted mono small" role="status">
		{#if $manifest.imagesState === 'error'}<span class="bad">image list failed: {$manifest.imagesError}</span>
		{:else if $manifest.imagesState === 'ready'}{withManifest.length} with a parameter manifest
		{:else if $manifest.imagesState === 'loading'}reading the image library…{/if}
	</span>
	<button
		type="button"
		class="primary"
		disabled={readBlocked !== null}
		title={readBlocked ? `Unavailable: ${readBlocked}` : 'Request the parameter list and the provenance records from the vehicle'}
		aria-describedby="read-why"
		onclick={() => void gcs.loadParams()}
	>
		{$vehicle?.listing === 'loading' ? 'Reading…' : 'Read parameters from the vehicle'}
	</button>
	<span id="read-why" class="muted small">{readBlocked && $vehicle?.listing !== 'loading' ? readBlocked : ''}</span>
	{#if progress}
		<span class="mono muted" role="status">{progress.have} / {progress.count}</span>
	{/if}
	{#if $vehicle?.listing === 'incomplete'}<Badge text={`incomplete: ${$vehicle.missing.length} missing`} tone="bad" />{/if}
</div>

<p class="inuse small" class:muted={$manifest.loaded !== null} role="status"><span class="k">manifest in use</span> {why}</p>

{#if $manifest.error}<p class="warn" role="alert">{$manifest.error}</p>{/if}
{#each $merged.issues as issue (issue)}<p class="warn">{issue}</p>{/each}
{#if $vehicle?.identity && $manifest.loaded && !ctx.manifestMatches}
	<p class="bad" role="alert">
		The manifest on screen has schema {$manifest.loaded.manifest.schemaHash}, the vehicle reports {$vehicle.identity.schemaHash}: all rows are read-only.
	</p>
{/if}

<div class="filters">
	<label for="filter" class="visually-hidden">Filter parameters</label>
	<input
		id="filter"
		type="search"
		placeholder="filter by name, unit, method, source text…  ( / )"
		bind:value={filter}
		bind:this={filterInput}
		onkeydown={(e) => { if (e.key === 'Escape') filter = ''; }}
	/>
	<Chip label={`staged differs (${counts.staged})`} active={onlyDiffs} onclick={() => (onlyDiffs = !onlyDiffs)} />
	<Chip label={`locked (${counts.locked})`} active={onlyLocked} onclick={() => (onlyLocked = !onlyLocked)} />
	<Chip label={`derived (${counts.derived})`} active={onlyDerived} onclick={() => (onlyDerived = !onlyDerived)} />
	{#if haveVehicle}<span class="muted mono small" title="active value differs from the factory value">{counts.changed} changed from factory</span>{/if}
	<span class="muted mono" role="status">{filtered.length} / {counts.total} shown</span>
	<span class="spacer"></span>
	<div class="seg" role="group" aria-label="Table options">
		<button type="button" aria-pressed={dense} class:on={dense} onclick={() => setDense(!dense)} title="Compact rows">compact</button>
		<button type="button" aria-pressed={grouped} class:on={grouped} onclick={() => (grouped = !grouped)} title="Group the rows by source file">grouped</button>
		<button type="button" disabled={!grouped} onclick={() => setAll(true)} title="Collapse every group">collapse all</button>
		<button type="button" disabled={!grouped} onclick={() => setAll(false)} title="Expand every group">expand all</button>
	</div>
</div>

<div class="layout">
	<div class="tables">
		{#if $merged.rows.length === 0}
			<p class="muted">No parameters: connect a vehicle (Connection) or pick a parameter set above to browse the manifest.</p>
		{:else}
			<!-- the arrow keys only move focus between the buttons inside; every action stays on a button -->
			<!-- svelte-ignore a11y_no_noninteractive_element_interactions -->
			<div class="tablewrap" role="group" aria-label="Parameter table: up and down arrows move between rows, Enter opens a row, Escape closes it" onkeydown={onTableKey}>
<table class:dense aria-label="Parameters">
				<thead>
					<tr>
						<th scope="col" aria-sort={ariaSort('name')}>
							<button type="button" class="sort" onclick={() => sortBy('name')}>Name{arrow('name')}</button> /
							<button type="button" class="sort" aria-label={`Sort by link index, ${ariaSort('index')}`} onclick={() => sortBy('index')}>link id #{arrow('index')}</button>
						</th>
						<th scope="col" class="r" aria-sort={ariaSort('factory')}><button type="button" class="sort" onclick={() => sortBy('factory')}>Factory{arrow('factory')}</button></th>
						<th scope="col" class="r" aria-sort={ariaSort('active')}><button type="button" class="sort" onclick={() => sortBy('active')}>Active{arrow('active')}</button></th>
						<th scope="col" class="r" aria-sort={ariaSort('staged')}><button type="button" class="sort" onclick={() => sortBy('staged')}>Staged{arrow('staged')}</button></th>
						<th scope="col" aria-sort={ariaSort('unit')}><button type="button" class="sort" onclick={() => sortBy('unit')}>Unit{arrow('unit')}</button></th>
						<th scope="col" aria-sort={ariaSort('method')}><button type="button" class="sort" onclick={() => sortBy('method')}>Method{arrow('method')}</button></th>
						<th scope="col" aria-sort={ariaSort('origin')}><button type="button" class="sort" onclick={() => sortBy('origin')}>Origin{arrow('origin')}</button></th>
						<th scope="col" aria-sort={ariaSort('sigma')}><button type="button" class="sort" onclick={() => sortBy('sigma')}>σ{arrow('sigma')}</button></th>
						<th scope="col">Flags</th>
					</tr>
				</thead>
				{#each visible as g (g.name)}
					<tbody>
						{#if grouped}
							<tr class="group">
								<th scope="colgroup" colspan="9">
									<button
										type="button"
										class="gbtn mono"
										data-nav
										aria-expanded={!collapsed[g.name]}
										aria-label={`${g.name}, ${g.rows.length} parameters: ${collapsed[g.name] ? 'expand' : 'collapse'}`}
										onclick={() => (collapsed[g.name] = !collapsed[g.name])}
									>
										<span aria-hidden="true">{collapsed[g.name] ? '▸' : '▾'}</span> {g.name} <span class="muted">({g.rows.length})</span>
									</button>
								</th>
							</tr>
						{/if}
						{#if !grouped || !collapsed[g.name]}
							{#each g.rows as r (r.key)}
								<ParamRow row={r} {ctx} {dense} result={$vehicle?.results[r.linkId]} onstage={stage} onreadsource={readSource} onretry={retry} />
							{/each}
						{/if}
					</tbody>
				{/each}
			</table>
</div>
			{#if visible.length === 0}<p class="muted">No parameter matches the filter.</p>{/if}
		{/if}
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
		top: calc(var(--header-h, 7rem) + 0.5rem);
		display: flex;
		flex-direction: column;
		gap: 0.75rem;
	}

	.small {
		font-size: 0.75rem;
	}

	.spacer {
		flex: 1;
	}

	.k {
		text-transform: uppercase;
		letter-spacing: 0.06em;
		font-size: 0.65rem;
		color: var(--muted);
		margin-right: 0.4rem;
	}

	.inuse {
		margin: 0 0 0.5rem;
		max-width: none;
	}

	.seg {
		display: flex;
	}

	.seg button {
		border-radius: 0;
		padding: 0.05rem 0.5rem;
		font-size: 0.75rem;
	}

	.seg button:first-child {
		border-radius: 4px 0 0 4px;
	}

	.seg button:last-child {
		border-radius: 0 4px 4px 0;
	}

	.seg button.on {
		border-color: var(--accent);
		background: var(--accent-bg);
	}

	.tablewrap {
		display: contents;
	}

	.sort {
		background: none;
		border: none;
		padding: 0;
		font: inherit;
		text-transform: inherit;
		letter-spacing: inherit;
		color: inherit;
		cursor: pointer;
	}

	.sort:hover {
		color: var(--accent);
	}

	tr.group > th {
		background: var(--surface-2);
		border-bottom: 1px solid var(--border-strong);
		padding: 0.15rem 0.4rem;
		text-transform: none;
		letter-spacing: normal;
	}

	.gbtn {
		background: none;
		border: none;
		padding: 0.1rem 0.2rem;
		font-size: 0.8rem;
		color: var(--text);
	}

	table {
		width: 100%;
		border-collapse: collapse;
		background: var(--surface);
		border: 1px solid var(--border);
	}

	thead th {
		position: sticky;
		top: var(--header-h, 7rem);
		z-index: 2;
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
