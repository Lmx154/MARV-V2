<script lang="ts">
	import { onMount } from 'svelte';
	import { readable } from 'svelte/store';
	import { app } from '$lib/state/app';
	import { formatBytes } from '$lib/state/logs_state';
	import { IDLE_STATUS, progressText, type ReplayLog, type ReplayMode, type ReplayState } from '$lib/state/replay_state';
	import type { HelperConnectionState } from '$link/transport';
	import Badge from '$lib/components/ui/Badge.svelte';
	import Panel from '$lib/components/ui/Panel.svelte';

	const gcs = app();
	const replay = gcs.replay;
	const empty: ReplayState = { logs: null, status: IDLE_STATUS, error: null, busy: false };
	const st = replay?.state ?? readable<ReplayState>(empty);
	const helperState = gcs.helper?.state ?? readable<HelperConnectionState>('closed');

	onMount(() => {
		void replay?.load();
	});

	const playing = $derived($st.status.state === 'playing');
	const whyNoPlay = $derived($helperState !== 'open' ? 'the helper is not connected' : playing ? 'a replay is going: stop it first' : null);
	const tone = (s: string): 'ok' | 'warn' | 'bad' | 'neutral' => (s === 'playing' ? 'ok' : s === 'failed' ? 'bad' : s === 'stopped' ? 'warn' : 'neutral');
	const label = (l: ReplayLog): string => (l.current ? `${l.name} (this session)` : l.name);
</script>

<Panel title="Replay" label="helper">
	<p class="muted small">
		A recorded ground log, played back through the path a live vehicle's bytes take: the Vehicle page shows the recorded session. Nothing is sent to
		a vehicle, and a replay is refused while a device or a run is open.
	</p>
	{#if $st.logs === null}
		<p class="muted small">Not read yet.</p>
	{:else if $st.logs.length === 0}
		<p class="muted small" data-replay="no-logs">The helper holds no ground log.</p>
	{:else}
		<table class="mono small" aria-label="Ground logs">
			<thead><tr><th>run</th><th>started</th><th>received</th><th>span</th><th></th></tr></thead>
			<tbody>
				{#each $st.logs as l (l.path)}
					<tr data-log={l.name}>
						<td title={l.path}>{label(l)}</td>
						{#if l.error}
							<td colspan="3" class="tone-bad">{l.error}</td>
							<td></td>
						{:else}
							<td>{l.wall_utc}</td>
							<td>{formatBytes(l.rx_bytes ?? 0)}</td>
							<td>{(l.duration_s ?? 0).toFixed(1)} s</td>
							<td class="actions">
								<button type="button" aria-label={`Replay ${l.name} at recorded pace`} disabled={whyNoPlay !== null || !l.rx_records} title={whyNoPlay ?? 'Play the received bytes at the pace they were recorded'} onclick={() => void replay?.play(l.path, 'paced' satisfies ReplayMode)}>Recorded pace</button>
								<button type="button" aria-label={`Replay ${l.name} as fast as possible`} disabled={whyNoPlay !== null || !l.rx_records} title={whyNoPlay ?? 'Play the received bytes as fast as the page takes them'} onclick={() => void replay?.play(l.path, 'fast' satisfies ReplayMode)}>As fast as possible</button>
							</td>
						{/if}
					</tr>
				{/each}
			</tbody>
		</table>
	{/if}
	<div class="buttons">
		<button type="button" disabled={$helperState !== 'open' || $st.busy} onclick={() => void replay?.load()}>Rescan</button>
		<button type="button" disabled={!playing} onclick={() => void replay?.stop()}>Stop replay</button>
		<a href="/vehicle">Open the Vehicle page</a>
	</div>
	{#if $st.status.state !== 'idle'}
		<p class="status small" role="status" aria-label="Replay status" data-replay={$st.status.state}>
			<Badge text={$st.status.state} tone={tone($st.status.state)} />
			<span class="mono">{progressText($st.status)}</span>
			{#if $st.status.reason}<span class="muted">· {$st.status.reason}</span>{/if}
			{#if $st.status.tx_dropped_bytes > 0}<span class="muted">· {$st.status.tx_dropped_bytes} B this page sent were dropped (a replay sends nothing)</span>{/if}
		</p>
	{/if}
	{#if $st.error}<p class="notice bad small" role="alert" data-notice="replay">{$st.error}</p>{/if}
</Panel>

<style>
	.small {
		font-size: 0.8rem;
	}

	.buttons {
		display: flex;
		flex-wrap: wrap;
		gap: 0.5rem;
		align-items: center;
		margin: 0 0 0.5rem;
	}

	.status {
		display: flex;
		flex-wrap: wrap;
		gap: 0.5rem;
		align-items: baseline;
		margin: 0 0 0.35rem;
	}

	.actions {
		display: flex;
		flex-wrap: wrap;
		gap: 0.35rem;
	}

	.notice {
		margin: 0 0 0.35rem;
		overflow-wrap: anywhere;
	}

	table {
		border-collapse: collapse;
		width: 100%;
		margin: 0 0 0.5rem;
	}

	th,
	td {
		text-align: left;
		padding: 0.15rem 0.5rem 0.15rem 0;
		border-bottom: 1px solid var(--border);
	}

	th {
		color: var(--muted);
		font-weight: 600;
	}

	.tone-bad {
		color: var(--bad);
	}
</style>
