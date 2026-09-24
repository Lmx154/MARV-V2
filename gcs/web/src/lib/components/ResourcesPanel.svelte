<script lang="ts">
	import { age, terminateBlocked, terminateQuestion } from '$lib/gcs/resources';
	import type { ResourceHolder, RigResource } from '$lib/gcs/types';

	interface Props {
		/** The last scan, null before the first. */
		resources: RigResource[] | null;
		/** The backend's refusal of the last terminate. */
		error: string | null;
		/** The pid a terminate was sent for, until a scan no longer lists it. */
		pending: number | null;
		wsOpen: boolean;
		onrefresh: () => void;
		onterminate: (pid: number) => void;
	}
	let { resources, error, pending, wsOpen, onrefresh, onterminate }: Props = $props();

	const now = $derived.by(() => {
		void resources;
		return Date.now();
	});

	function terminate(r: RigResource, h: ResourceHolder): void {
		if (terminateBlocked(h, wsOpen, pending) !== null) return;
		if (!confirm(terminateQuestion(r, h))) return;
		onterminate(h.pid);
	}
</script>

<section class="panel mono" aria-label="Resources">
	<div class="head">
		<h2>Resources</h2>
		<button type="button" class="btn sm" disabled={!wsOpen} onclick={onrefresh}>Refresh</button>
		<span class="hint">who holds the flight controller's port, the rig lock, the sim and the GCS ports; refreshed every 2 s while this tab is open</span>
	</div>
	{#if error}<p class="err" role="alert">{error}</p>{/if}
	{#if resources === null}
		<p class="hint">{wsOpen ? 'Scanning…' : 'The backend is not connected.'}</p>
	{:else}
		<div class="scroll">
			<table class="specs">
				<thead>
					<tr><th>resource</th><th>path</th><th>pid</th><th>process</th><th>running for</th><th>command line</th><th></th></tr>
				</thead>
				<tbody>
					{#each resources as r (r.id)}
						{#if r.holders.length === 0}
							<tr>
								<td><b>{r.label}</b></td>
								<td class="k">{r.path ?? '—'}</td>
								<td colspan="5" class="k">{r.id === 'fc_usb' && !r.path ? 'no flight controller plugged in' : 'free'}</td>
							</tr>
						{:else}
							{#each r.holders as h, i (h.pid)}
								{@const why = terminateBlocked(h, wsOpen, pending)}
								<tr>
									<td>{#if i === 0}<b>{r.label}</b>{/if}</td>
									<td class="k">{i === 0 ? (r.path ?? '—') : ''}</td>
									<td>{h.pid}</td>
									<td>{h.name}{#if h.self} <span class="k">(this GCS)</span>{/if}{#if h.child_of_launcher} <span class="k">(launched sim)</span>{/if}</td>
									<td>{age(h.started, now)}</td>
									<td class="cmd" title={h.cmdline}>{h.cmdline}</td>
									<td>
										<button type="button" class="btn sm stop" disabled={why !== null} title={why ?? `SIGTERM ${h.name} (pid ${h.pid}), SIGKILL 5 s later if needed`} onclick={() => terminate(r, h)}>
											{pending === h.pid ? 'Terminating…' : h.child_of_launcher ? 'Stop sim' : 'Terminate'}
										</button>
									</td>
								</tr>
							{/each}
						{/if}
					{/each}
				</tbody>
			</table>
		</div>
	{/if}
</section>

<style>
	.panel {
		display: flex;
		flex-direction: column;
		gap: 0.5rem;
		min-width: 0;
		margin-top: 0.75rem;
		font-size: 0.8rem;
	}
	.panel h2 {
		margin: 0;
	}
	.head {
		display: flex;
		flex-wrap: wrap;
		align-items: center;
		gap: 0.5rem 1rem;
	}
	.scroll {
		overflow-x: auto;
		border: 1px solid var(--border);
		border-radius: 8px;
		background: var(--surface);
	}
	.specs {
		width: 100%;
		border-collapse: collapse;
		font-size: 0.75rem;
		font-variant-numeric: tabular-nums;
		white-space: nowrap;
	}
	.specs th,
	.specs td {
		padding: 0.25rem 0.45rem;
		border-bottom: 1px solid var(--border);
		text-align: left;
	}
	.specs th {
		color: var(--muted);
		font-weight: 600;
	}
	.cmd {
		max-width: 28rem;
		overflow: hidden;
		text-overflow: ellipsis;
	}
	.k {
		color: var(--muted);
	}
	.hint {
		margin: 0;
		color: var(--muted);
		max-width: none;
	}
	.err {
		margin: 0;
		color: var(--bad);
		max-width: none;
	}
	.btn {
		background: var(--surface-2);
		color: var(--text);
		border: 1px solid var(--border);
		border-radius: 6px;
		padding: 0.35rem 0.7rem;
		font-size: 0.8rem;
		cursor: pointer;
	}
	.btn:hover {
		border-color: var(--border-strong);
	}
	.btn:disabled {
		opacity: 0.5;
		cursor: default;
	}
	.btn.sm {
		padding: 0.15rem 0.45rem;
	}
	.btn.stop:not(:disabled) {
		border-color: var(--bad);
		color: var(--bad);
	}
</style>
