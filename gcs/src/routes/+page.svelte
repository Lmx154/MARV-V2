<script lang="ts">
	import { onMount } from 'svelte';
	import { readable } from 'svelte/store';
	import { app } from '$lib/state/app';
	import type { SessionState } from '$link/session';
	import Badge from '$lib/components/ui/Badge.svelte';
	import Panel from '$lib/components/ui/Panel.svelte';

	const gcs = app();
	const helperState = gcs.helper?.state ?? readable('closed');
	const bytes = gcs.helper?.bytes ?? readable({ rxBytes: 0, txBytes: 0 });
	const status = gcs.status;
	const sessionState = gcs.session?.state ?? readable<SessionState | null>(null);

	let ports = $state<string[]>([]);
	let serialPath = $state('');
	let baud = $state(115200);
	let udpPort = $state(14550);
	let busy = $state(false);

	async function listPorts(): Promise<void> {
		try {
			ports = (await gcs.api?.ports()) ?? [];
			if (!serialPath && ports[0]) serialPath = ports[0];
		} catch {
			ports = [];
		}
	}

	async function act(f: () => Promise<void>): Promise<void> {
		busy = true;
		try {
			await f();
		} finally {
			busy = false;
		}
	}

	onMount(() => {
		void listPorts();
		const timer = setInterval(() => void gcs.refreshStatus(), 2000);
		return () => clearInterval(timer);
	});

	const isOpen = $derived($helperState === 'open');
	const hasDevice = $derived(Boolean($status?.device));
</script>

<svelte:head><title>Connection · MARV GCS</title></svelte:head>

<h1>Connection</h1>
<p class="muted lead">The helper owns the serial port and the SIL socket; this page asks it to open one. One device at a time.</p>

<div class="grid">
	<Panel title="Device" label="open">
		<form class="row" onsubmit={(e) => { e.preventDefault(); void act(() => gcs.openSerial(serialPath, baud)); }}>
			<label for="serial-path">Serial port</label>
			<input id="serial-path" type="text" list="ports" bind:value={serialPath} placeholder="/dev/ttyACM1" autocomplete="off" />
			<datalist id="ports">{#each ports as p (p)}<option value={p}></option>{/each}</datalist>
			<label for="baud">Baud</label>
			<input id="baud" type="number" bind:value={baud} min="1" class="narrow" />
			<button type="button" onclick={() => void listPorts()}>Rescan</button>
			<button type="submit" class="primary" disabled={!isOpen || hasDevice || busy || !serialPath}>Open serial</button>
		</form>
		<form class="row" onsubmit={(e) => { e.preventDefault(); void act(() => gcs.openUdp(udpPort)); }}>
			<label for="udp-port">SIL UDP port</label>
			<input id="udp-port" type="number" bind:value={udpPort} min="1" max="65535" class="narrow" />
			<button type="submit" class="primary" disabled={!isOpen || hasDevice || busy}>Open SIL UDP</button>
		</form>
		<div class="row">
			<button type="button" disabled={!isOpen || !hasDevice || busy} onclick={() => void act(() => gcs.closeDevice())}>Close device</button>
			<span class="muted">Ports found: {ports.length ? ports.join(', ') : 'none'}</span>
		</div>
	</Panel>

	<Panel title="SIL run" label="simulation">
		<div class="row">
			<button type="button" class="primary" disabled={!isOpen || busy || $status?.sil.running} onclick={() => void act(() => gcs.silStart())}>Start SIL run</button>
			<button type="button" disabled={!isOpen || busy || !$status?.sil.running} onclick={() => void act(() => gcs.silStop())}>Stop SIL run</button>
		</div>
		<dl class="facts mono">
			<dt>state</dt>
			<dd>{#if $status?.sil.running}<Badge text="running" tone="ok" />{:else}<Badge text="stopped" />{/if}</dd>
			<dt>pid</dt><dd>{$status?.sil.pid ?? '—'}</dd>
			<dt>exit status</dt><dd>{$status?.sil.exit_status ?? '—'}</dd>
			<dt>UDP port</dt><dd>{$status?.sil.udp_port ?? 'not set (the helper was started without --sil-udp-port)'}</dd>
		</dl>
		<p class="muted note">The SIL speaks the link only from unit 6; until then a run produces no vehicle.</p>
	</Panel>

	<Panel title="Helper" label="status">
		<dl class="facts mono">
			<dt>WebSocket</dt><dd><Badge text={$helperState} tone={isOpen ? 'ok' : 'bad'} /></dd>
			<dt>device</dt><dd>{$status?.device ?? 'none'}</dd>
			<dt>run directory</dt><dd class="wrap">{$status?.run_dir ?? '—'}</dd>
			<dt>bytes from device</dt><dd>{$bytes.rxBytes}</dd>
			<dt>bytes to device</dt><dd>{$bytes.txBytes}</dd>
			<dt>dropped bytes (helper queue)</dt><dd class:bad={($status?.dropped_bytes ?? 0) > 0}>{$status?.dropped_bytes ?? '—'}</dd>
			<dt>GCS built</dt><dd>{$status?.gcs_built ? 'yes' : 'no'}</dd>
		</dl>
		<div class="row"><button type="button" onclick={() => void gcs.refreshStatus()}>Refresh</button></div>
	</Panel>

	<Panel title="Link" label="vehicle">
		<dl class="facts mono">
			<dt>heartbeat</dt>
			<dd><Badge text={$sessionState?.heartbeat ?? 'no heartbeat'} tone={$sessionState?.heartbeat === 'alive' ? 'ok' : 'warn'} /></dd>
			<dt>vehicle address</dt>
			<dd>{$sessionState?.vehicle ? `system ${$sessionState.vehicle.systemId}, component ${$sessionState.vehicle.componentId}` : '—'}</dd>
			<dt>frames decoded</dt><dd>{$sessionState?.frames ?? 0}</dd>
			<dt>bad frames</dt><dd class:bad={($sessionState?.badFrames ?? 0) > 0}>{$sessionState?.badFrames ?? 0}</dd>
		</dl>
	</Panel>
</div>

<style>
	.lead {
		margin: 0.2rem 0 0.75rem;
	}

	.grid {
		display: grid;
		grid-template-columns: repeat(auto-fit, minmax(26rem, 1fr));
		gap: 0.75rem;
		align-items: start;
	}

	.row {
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

	.narrow {
		width: 6rem;
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

	.note {
		font-size: 0.8rem;
		margin: 0;
	}
</style>
