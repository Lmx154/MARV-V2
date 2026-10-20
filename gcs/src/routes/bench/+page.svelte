<script lang="ts">
	import { readable } from 'svelte/store';
	import { app } from '$lib/state/app';
	import { BENCH_KINDS, CHECK_TITLES, VERDICT_MODES, benchView, type BenchState } from '$lib/state/bench_state';
	import type { HelperConnectionState } from '$link/transport';
	import type { HelperStatus } from '$lib/state/helper_api';
	import Badge from '$lib/components/ui/Badge.svelte';
	import Panel from '$lib/components/ui/Panel.svelte';

	const gcs = app();
	const bench = gcs.bench;
	const benchState = bench?.state ?? readable<BenchState | null>(null);
	const status = gcs.status ?? readable<HelperStatus | null>(null);
	const helperState = gcs.helper?.state ?? readable<HelperConnectionState>('closed');

	const ack = $derived($benchState?.ack ?? false);
	const seconds = $derived($benchState?.seconds ?? '');
	const pre = $derived($benchState?.preflight ?? null);
	const notices = $derived($benchState?.notices ?? []);
	const busy = $derived($benchState?.busy ?? null);
	const form = $derived($benchState?.verdictForm ?? { mode: '', text: '' });
	const verdictNotices = $derived($benchState?.verdictNotices ?? []);
	const kind = $derived($benchState?.kind ?? 'hil');
	const flashNotices = $derived($benchState?.flashNotices ?? []);
	const view = $derived(benchView($status));
	const run = $derived(view.run);
	const swd = $derived(view.swd);
	const verdict = $derived(view.verdict);

	const whyNoLaunch = $derived.by(() => {
		if ($helperState !== 'open') return 'the helper is not connected';
		if (busy) return 'a command is in progress';
		if (view.running) return 'a HIL run is going';
		return null;
	});
	const whyNoStop = $derived(busy ? 'a command is in progress' : !view.running ? 'no HIL run is going' : null);
	const whyNoVerdict = $derived.by(() => {
		if (!run) return 'no HIL run in this helper session';
		if (view.running) return 'the run is still going: give the verdict after it ends';
		if (verdict) return 'a verdict is already saved for this run';
		if (busy) return 'a command is in progress';
		return null;
	});
	const whyNoFlash = $derived.by(() => {
		if ($helperState !== 'open') return 'the helper is not connected';
		if (busy) return 'a command is in progress';
		if (swd?.running) return 'a flash is running';
		return null;
	});
	const tone = (state: string): 'ok' | 'bad' | 'neutral' => (state === 'ok' ? 'ok' : state === 'refused' ? 'bad' : 'neutral');
</script>

<svelte:head><title>Bench · MARV GCS</title></svelte:head>

<h1>Bench</h1>
<p class="muted lead">
	Hardware in the loop: the board flies the Gazebo world (stage S4b), the plugin gives it the simulated IMU and the board's own receiver takes your
	radio. The helper checks the bench before it starts anything and names every reason it refuses. The console is built by GX-16.
</p>

<div class="grid">
	<div class="stack">
		<Panel title="HIL launch (S4b)" label="helper">
			<div class="row">
				<input id="hil-ack" type="checkbox" checked={ack} disabled={busy !== null} onchange={(e) => bench?.setAck(e.currentTarget.checked)} />
				<label for="hil-ack" class="plain">ESC battery unplugged</label>
			</div>
			<div class="row">
				<label for="hil-seconds">Seconds</label>
				<input id="hil-seconds" type="text" inputmode="decimal" class="mono secs" value={seconds} disabled={busy !== null} oninput={(e) => bench?.setSeconds(e.currentTarget.value)} />
			</div>
			<div class="buttons">
				<button type="button" disabled={busy !== null || $helperState !== 'open'} onclick={() => void bench?.setHold(!view.hold)}>
					{view.hold ? 'Release the hold' : 'Hold the board'}
				</button>
				<button type="button" disabled={busy !== null || $helperState !== 'open'} onclick={() => void bench?.check()}>{busy === 'preflight' ? 'Checking…' : 'Preflight'}</button>
				<button type="button" class="primary" disabled={whyNoLaunch !== null} title={whyNoLaunch ?? 'Preflight, then start the S4b run'} onclick={() => void bench?.launch()}>
					{busy === 'launch' ? 'Launching…' : 'Launch S4b'}
				</button>
				<button type="button" disabled={whyNoStop !== null} title={whyNoStop ?? 'Stop the run and its processes'} onclick={() => void bench?.stop()}>
					{busy === 'stop' ? 'Stopping…' : 'Stop'}
				</button>
				{#if view.hold}<Badge text="board held" tone="warn" />{/if}
			</div>
			<p class="muted small">
				The hold closes the helper's serial port and keeps autoconnect off the board; the run ends it. Radio on, arm switch down, throttle low before you launch.
			</p>
			{#each notices as n, i (i)}<p class="notice bad small" role="alert" data-notice="launch">{n}</p>{/each}
			{#if pre}
				<ul class="checks" aria-label="Preflight checks">
					{#each pre.checks as c (c.id)}
						<li data-check={c.id} data-state={c.state}>
							<Badge text={c.state} tone={tone(c.state)} />
							<span class="title">{CHECK_TITLES[c.id] ?? c.id}</span>
							<span class="detail muted small">{c.detail}</span>
						</li>
					{/each}
				</ul>
				<p class="small" data-preflight={pre.allowed ? 'allowed' : 'refused'}>
					{#if pre.allowed}<Badge text="allowed" tone="ok" /> every check holds{pre.port ? `: port ${pre.port}, ${pre.seconds} s` : ''}{:else}<Badge text="refused" tone="bad" /> {pre.refusals.length} reason{pre.refusals.length === 1 ? '' : 's'} above{/if}
				</p>
			{/if}
		</Panel>

		<Panel title="Bench images (SWD)" label="helper">
			<p class="muted small">
				Bench images never fly and are flashed through the debug probe, outside the config gate. Only the HIL image is flashed from here; the
				helper never builds.
			</p>
			<div class="row">
				<label for="bench-kind">Image kind</label>
				<select id="bench-kind" value={kind} disabled={busy !== null || swd?.running} onchange={(e) => bench?.setKind(e.currentTarget.value)}>
					{#each BENCH_KINDS as k (k.kind)}<option value={k.kind}>{k.label}</option>{/each}
				</select>
			</div>
			<div class="buttons">
				<button type="button" disabled={whyNoFlash !== null} title={whyNoFlash ?? 'Flash the image over SWD'} onclick={() => void bench?.flash()}>
					{busy === 'flash' ? 'Starting…' : 'Flash over SWD'}
				</button>
			</div>
			{#each flashNotices as n, i (i)}<p class="notice bad small" role="alert" data-notice="flash">{n}</p>{/each}
			{#if swd}
				<dl class="facts mono" aria-label="SWD flash">
					<dt>kind</dt><dd>{swd.kind}</dd>
					<dt>state</dt>
					<dd>{#if swd.running}<Badge text="running" tone="warn" />{:else if swd.ok}<Badge text="done" tone="ok" />{:else}<Badge text="failed" tone="bad" />{/if}</dd>
					<dt>image</dt><dd class="wrap">{swd.elf}</dd>
					<dt>sha256</dt><dd class="wrap">{swd.elf_sha256}</dd>
					{#if swd.error}<dt>error</dt><dd class="wrap">{swd.error}</dd>{/if}
				</dl>
				{#if swd.tail.length > 0}<pre class="tail mono small" aria-label="Flash output">{swd.tail.join('\n')}</pre>{/if}
			{/if}
		</Panel>
	</div>

	<div class="stack">
		<Panel title="Run" label="helper">
			{#if !run}
				<p class="muted" data-run="none">No HIL run in this helper session.</p>
			{:else}
				<dl class="facts mono" aria-label="HIL run">
					<dt>state</dt>
					<dd><Badge text={view.running ? 'running' : run.state} tone={view.running ? 'ok' : 'neutral'} /></dd>
					<dt>run id</dt><dd>{run.run_id}</dd>
					{#if run.options && 'port' in run.options}<dt>port</dt><dd class="wrap">{run.options.port}</dd>{/if}
					{#if run.options && 'seconds' in run.options}<dt>seconds</dt><dd>{run.options.seconds}</dd>{/if}
					<dt>exit</dt><dd>{run.exit_status === null ? '—' : run.exit_status}</dd>
					{#if view.outDir}<dt>files</dt><dd class="wrap" data-run="out-dir">{view.outDir}</dd>{/if}
				</dl>
			{/if}
		</Panel>

		<Panel title="Verdict" label="helper">
			<fieldset class="modes" disabled={whyNoVerdict !== null}>
				<legend>Flight mode flown</legend>
				{#each VERDICT_MODES as m (m)}
					<label class="plain"><input type="radio" name="verdict-mode" value={m} checked={form.mode === m} onchange={() => bench?.setMode(m)} /> {m}</label>
				{/each}
			</fieldset>
			<div class="row col">
				<label for="verdict-text">Verdict</label>
				<textarea id="verdict-text" rows="5" value={form.text} disabled={whyNoVerdict !== null} placeholder="what happened: took off, held a hover, how it felt" oninput={(e) => bench?.setVerdictText(e.currentTarget.value)}></textarea>
			</div>
			<div class="buttons">
				<button type="button" class="primary" disabled={whyNoVerdict !== null} title={whyNoVerdict ?? 'Save the verdict next to the run report'} onclick={() => run && void bench?.saveVerdict(run.run_id)}>
					{busy === 'verdict' ? 'Saving…' : 'Save verdict'}
				</button>
			</div>
			{#each verdictNotices as n, i (i)}<p class="notice bad small" role="alert" data-notice="verdict">{n}</p>{/each}
			{#if verdict}
				<dl class="facts mono" aria-label="Saved verdict">
					<dt>run id</dt><dd>{verdict.run_id}</dd>
					<dt>mode</dt><dd data-verdict="mode">{verdict.mode}</dd>
					<dt>text</dt><dd class="wrap pre" data-verdict="text">{verdict.text}</dd>
					<dt>report</dt><dd class="wrap">{verdict.report ?? 'none found'}</dd>
					<dt>saved</dt><dd>{verdict.saved_utc}</dd>
				</dl>
				<p class="muted small">Saved in the run's directory next to the report{view.outDir ? `: ${view.outDir}` : ''}.</p>
			{/if}
		</Panel>
	</div>
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

	.stack {
		display: flex;
		flex-direction: column;
		gap: 0.75rem;
	}

	.row,
	.buttons {
		display: flex;
		flex-wrap: wrap;
		gap: 0.5rem;
		align-items: center;
		margin: 0 0 0.5rem;
	}

	.row.col {
		flex-direction: column;
		align-items: stretch;
	}

	label {
		color: var(--muted);
		font-size: 0.75rem;
		text-transform: uppercase;
		letter-spacing: 0.06em;
	}

	label.plain {
		text-transform: none;
		letter-spacing: normal;
		font-size: 0.85rem;
		color: inherit;
	}

	.secs {
		width: 7rem;
	}

	.modes {
		display: flex;
		gap: 1rem;
		margin: 0 0 0.5rem;
		padding: 0.25rem 0.5rem;
		border: 1px solid var(--border);
	}

	.modes legend {
		color: var(--muted);
		font-size: 0.75rem;
		text-transform: uppercase;
		letter-spacing: 0.06em;
	}

	.checks {
		list-style: none;
		margin: 0.5rem 0;
		padding: 0;
	}

	.checks li {
		display: flex;
		flex-wrap: wrap;
		gap: 0.5rem;
		align-items: baseline;
		margin: 0 0 0.3rem;
	}

	.checks .title {
		min-width: 14rem;
	}

	.detail {
		overflow-wrap: anywhere;
	}

	.notice {
		margin: 0 0 0.35rem;
		overflow-wrap: anywhere;
	}

	.small {
		font-size: 0.8rem;
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

	.pre {
		white-space: pre-wrap;
	}

	.tail {
		max-height: 12rem;
		overflow: auto;
		white-space: pre-wrap;
		margin: 0;
	}

	textarea {
		width: 100%;
		box-sizing: border-box;
	}
</style>
