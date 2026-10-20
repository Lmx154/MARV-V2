<script lang="ts">
	import { readable } from 'svelte/store';
	import { tick } from 'svelte';
	import { app } from '$lib/state/app';
	import { CLOSED, type ConsoleState } from '$lib/console/console_state';
	import type { HelperConnectionState } from '$link/transport';
	import type { HelperStatus } from '$lib/state/helper_api';
	import Badge from '$lib/components/ui/Badge.svelte';
	import Panel from '$lib/components/ui/Panel.svelte';

	const gcs = app();
	const ctl = gcs.console;
	const consoleState = ctl?.state ?? readable<ConsoleState | null>(null);
	const status = gcs.status ?? readable<HelperStatus | null>(null);
	const helperState = gcs.helper?.state ?? readable<HelperConnectionState>('closed');

	const view = $derived($consoleState?.view ?? CLOSED);
	const devices = $derived($consoleState?.devices ?? []);
	const blocked = $derived($consoleState?.blocked ?? []);
	const entries = $derived($consoleState?.entries ?? []);
	const partial = $derived($consoleState?.partial ?? '');
	const error = $derived($consoleState?.error ?? null);
	const busy = $derived($consoleState?.busy ?? false);
	const connected = $derived($consoleState?.connected ?? false);
	const isOpen = $derived(view.state === 'open');

	let selected = $state('');
	let line = $state('');
	let logEl = $state<HTMLElement | null>(null);
	let follow = $state(true);

	// The helper's status carries the console block (the interfaces it lists, why it is blocked): read it whenever the status is read.
	$effect(() => {
		ctl?.applyStatus($status?.console);
	});
	$effect(() => {
		if (!devices.includes(selected)) selected = isOpen && view.device ? view.device : (devices[0] ?? '');
	});
	// Follow the output unless the operator scrolled up.
	$effect(() => {
		entries.length;
		partial;
		void tick().then(() => {
			if (follow && logEl) logEl.scrollTop = logEl.scrollHeight;
		});
	});

	const NEAR_BOTTOM_PX = 8; // harness value: how close to the end counts as "at the end"
	function onScroll(): void {
		if (logEl) follow = logEl.scrollHeight - logEl.scrollTop - logEl.clientHeight <= NEAR_BOTTOM_PX;
	}

	const whyNoOpen = $derived.by(() => {
		if ($helperState !== 'open') return 'the helper is not connected';
		if (isOpen) return 'the console is open';
		if (busy) return 'a command is in progress';
		if (blocked.length > 0) return blocked[0];
		if (selected === '') return 'no console interface under the by-id directory';
		return null;
	});
	const whyNoClose = $derived(!isOpen ? 'the console is not open' : busy ? 'a command is in progress' : null);
	const whyNoSend = $derived(!isOpen ? 'the console is not open' : !connected ? 'the console channel is not connected' : null);

	function send(event: SubmitEvent): void {
		event.preventDefault();
		if (ctl?.sendLine(line)) line = '';
	}
	const tone = (state: string): 'ok' | 'bad' | 'neutral' => (state === 'open' ? 'ok' : state === 'lost' ? 'bad' : 'neutral');
</script>

<svelte:head><title>Console · MARV GCS</title></svelte:head>

<h1>Console</h1>
<p class="muted lead">
	The board's console as text, through the helper. The page is a terminal: what you type goes to the board unchanged, plus the line end. The link
	is a different interface of the board and stays connected.
</p>

<Panel title="Console device" label="helper">
	<div class="row">
		<label for="console-device">Device</label>
		<select id="console-device" class="mono" value={selected} disabled={isOpen || busy} onchange={(e) => (selected = e.currentTarget.value)}>
			{#each devices as d (d)}<option value={d}>{d}</option>{/each}
			{#if devices.length === 0}<option value="">no console interface found</option>{/if}
		</select>
		<button type="button" disabled={whyNoOpen !== null} title={whyNoOpen ?? 'Open the console'} onclick={() => void ctl?.open(selected)}>Open</button>
		<button type="button" disabled={whyNoClose !== null} title={whyNoClose ?? 'Close the console'} onclick={() => void ctl?.close()}>Close</button>
		<span data-console="state"><Badge text={view.state} tone={tone(view.state)} /></span>
	</div>
	{#if view.reason && !isOpen}<p class="muted small" data-console="reason">{view.reason}</p>{/if}
	{#if blocked.length > 0 && !isOpen}
		<ul class="blocked small" aria-label="Why the console cannot be opened">
			{#each blocked as b, i (i)}<li data-console="blocked">{b}</li>{/each}
		</ul>
	{/if}
	{#if error}<p class="notice bad small" role="alert" data-console="error">{error}</p>{/if}
</Panel>

<Panel title="Output" label="board">
	<!-- svelte-ignore a11y_no_noninteractive_tabindex (a scrolling region must be reachable by keyboard to be scrolled) -->
	<div class="log mono" role="log" aria-label="Console output" tabindex="0" bind:this={logEl} onscroll={onScroll}>
		{#each entries as e (e.id)}<div class="line" data-kind={e.kind}>{e.kind === 'tx' ? '> ' : ''}{e.text}</div>{/each}
		{#if partial !== ''}<div class="line" data-kind="partial">{partial}</div>{/if}
	</div>
	<form class="send" onsubmit={send}>
		<label for="console-input">Input</label>
		<input id="console-input" type="text" class="mono" aria-label="Console input" autocomplete="off" autocapitalize="off" spellcheck="false" bind:value={line} disabled={whyNoSend !== null} placeholder={whyNoSend ?? 'a line for the board, then Enter'} />
		<button type="submit" disabled={whyNoSend !== null} title={whyNoSend ?? 'Send the line to the board'}>Send</button>
	</form>
</Panel>

<style>
	.lead {
		margin: 0.2rem 0 0.75rem;
	}

	.row,
	.send {
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

	select {
		min-width: 24rem;
		max-width: 100%;
	}

	.send input {
		flex: 1 1 24rem;
	}

	.log {
		height: 28rem;
		overflow: auto;
		margin: 0 0 0.5rem;
		padding: 0.4rem 0.5rem;
		border: 1px solid var(--border);
		border-radius: 4px;
		background: var(--surface-2);
		font-size: 0.8rem;
	}

	.line {
		white-space: pre-wrap;
		overflow-wrap: anywhere;
	}

	.line[data-kind='tx'] {
		color: var(--accent);
	}

	.line[data-kind='note'] {
		color: var(--muted);
		font-style: italic;
	}

	.blocked {
		margin: 0 0 0.5rem;
		padding-left: 1.2rem;
		color: var(--warn);
	}

	.notice {
		margin: 0 0 0.35rem;
		overflow-wrap: anywhere;
	}

	.small {
		font-size: 0.8rem;
	}
</style>
