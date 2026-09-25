<script lang="ts">
	import { onMount } from 'svelte';
	import BlockChain from '$lib/components/BlockChain.svelte';
	import ControllerView from '$lib/components/ControllerView.svelte';
	import DerivePanel from '$lib/components/DerivePanel.svelte';
	import DevelopmentView from '$lib/components/DevelopmentView.svelte';
	import MissionView from '$lib/components/MissionView.svelte';
	import ProfilesEditor from '$lib/components/ProfilesEditor.svelte';
	import ResourcesPanel from '$lib/components/ResourcesPanel.svelte';
	import { connect, loadAirframes, loadLink, loadSchema, mockFlag, type Connection } from '$lib/gcs/link';
	import { isMissionRequest, type MissionMsg, type MissionRequest } from '$lib/gcs/mission';
	import { euler } from '$lib/gcs/protocol';
	import type { RadioDevice, RadioLive } from '$lib/gcs/radio';
	import { busyBadge } from '$lib/gcs/resources';
	import {
		applyHeader,
		editParam,
		emptySetup,
		expireParam,
		exportSetup,
		matchingFactory,
		paramCount,
		paramEcho,
		planImport,
		referenceValues,
		refusedFamily,
		requestError,
		sameF32,
		schemaMatches,
		shown,
		statusWords,
		vehicleId,
		type ErrorSlot,
		type SetupState
	} from '$lib/gcs/setup';
	import type { Airframe, ClientMsg, LinkInfo, MissionStatus, RigResource, Schema, ServerMsg, SimStatus, Telemetry } from '$lib/gcs/types';

	/** An edit with no echo after this is dropped (the backend retries every 500 ms). */
	const ECHO_TIMEOUT_MS = 1500;
	const DEG = 180 / Math.PI;
	/** A sim_launch / sim_stop with no answer after this is reported as unanswered. */
	const SIM_TIMEOUT_MS = 15000;

	let schema = $state.raw<Schema | null>(null);
	let loadError = $state<string | null>(null);
	let link = $state<LinkInfo | null>(null);
	let wsOpen = $state(false);
	let st = $state<SetupState>(emptySetup());
	let telem = $state.raw<Telemetry | null>(null);
	let flashLog = $state<string[]>([]);
	let flashing = $state(false);
	let note = $state<string | null>(null);
	/** The backend's last error per slot, shown next to the control that caused it. */
	let errors = $state<Partial<Record<ErrorSlot, string>>>({});
	/** Family -> the FC's refusal of the kind last asked for, shown on that family's card. */
	let kindErrors = $state<Record<number, string>>({});
	let mission = $state.raw<MissionStatus | null>(null);
	/** The backend's last refusal per mission control, shown beside it. */
	let missionErrors = $state<Partial<Record<MissionRequest | 'profile', string>>>({});
	/** The tab chosen; until then Mission while the FC is connected. */
	let tab = $state<'mission' | 'setup' | 'development' | 'controller' | null>(null);
	let airframes = $state.raw<Airframe[] | null>(null);
	let airframesError = $state<string | null>(null);
	/** The Development tab's selected airframe id. */
	let simAirframe = $state('');
	let simStatus = $state.raw<SimStatus | null>(null);
	let simLog = $state<string[]>([]);
	let simError = $state<string | null>(null);
	let simPending = $state(false);
	let simTimer: ReturnType<typeof setTimeout> | undefined;
	/** The Development tab's Resources panel: the last scan, the refusal of the last terminate, the pid being terminated. */
	let resources = $state.raw<RigResource[] | null>(null);
	let resourcesError = $state<string | null>(null);
	let terminating = $state<number | null>(null);
	let terminateTimer: ReturnType<typeof setTimeout> | undefined;
	/** The Controller tab: the last radio frame, the last hot-plug device list, the refusal of radio_subscribe. */
	let radioLive = $state.raw<RadioLive | null>(null);
	let radioDevices = $state.raw<RadioDevice[] | null>(null);
	let radioError = $state<string | null>(null);
	let mockMode = $state<string | null>(null);
	let conn: Connection | null = null;
	const sentAt = new Map<number, number>();

	const header = $derived(st.header);
	const match = $derived(schema !== null && header !== null && schemaMatches(schema, header));
	const loaded = $derived(schema !== null && st.values.length === paramCount(schema));
	const editable = $derived(match && loaded);
	const kinds = $derived(header?.kind ?? []);
	const vehicle = $derived(schema ? vehicleId(schema, kinds) : undefined);
	const armed = $derived(header?.armed ?? telem?.armed ?? false);
	const factoryNow = $derived(schema && loaded ? matchingFactory(schema, kinds, st.values) : null);
	const words = $derived(header ? statusWords(header) : []);
	const view = $derived(tab ?? (header ? 'mission' : 'setup'));
	const att = $derived(telem ? euler(telem.q) : null);
	const busy = $derived(busyBadge(link));
	const selectedAirframe = $derived(airframes?.find((a) => a.id === simAirframe) ?? null);
	const presetLabel = $derived.by(() => {
		if (!telem || !schema) return '—';
		if (telem.preset === 0xff) return 'custom';
		return schema.factory.find((f) => f.id === telem!.preset)?.label ?? `#${telem.preset}`;
	});

	const value = (i: number): number => shown(st, i);
	const pending = (i: number): boolean => i in st.pending || i in st.queued;
	const reference = (family: number): Record<string, number> => (schema ? referenceValues(schema, family, kinds[family] ?? 0) : {});

	function send(m: ClientMsg): void {
		conn?.send(m);
	}

	function sendMission(m: MissionMsg | Extract<ClientMsg, { type: 'profile' }>): void {
		delete missionErrors[m.type];
		send(m);
	}

	function simDone(): void {
		simPending = false;
		clearTimeout(simTimer);
	}

	function sendSim(m: Extract<ClientMsg, { type: 'sim_launch' | 'sim_stop' }>): void {
		simError = null;
		simPending = true;
		clearTimeout(simTimer);
		simTimer = setTimeout(() => {
			simPending = false;
			simError = `${m.type}: no answer from the backend`;
		}, SIM_TIMEOUT_MS);
		send(m);
	}

	function terminate(pid: number): void {
		resourcesError = null;
		terminating = pid;
		clearTimeout(terminateTimer);
		terminateTimer = setTimeout(() => (terminating = null), 10000);
		send({ type: 'terminate', pid });
	}

	// A resource scan every 2 s while the Development tab is shown (the backend sends them to subscribed pages only).
	$effect(() => {
		if (!wsOpen || view !== 'development') return;
		send({ type: 'resources_subscribe', on: true });
		return () => send({ type: 'resources_subscribe', on: false });
	});

	async function reloadAirframes(): Promise<void> {
		airframesError = null;
		try {
			airframes = await loadAirframes(mockMode);
			if (!airframes.some((a) => a.id === simAirframe)) simAirframe = '';
		} catch (e) {
			airframesError = e instanceof Error ? e.message : String(e);
		}
	}

	function sendParam(index: number, v: number): void {
		sentAt.set(index, performance.now());
		send({ type: 'set_param', index, value: v });
	}

	function onMessage(m: ServerMsg): void {
		switch (m.type) {
			case 'link': {
				const was = link?.connected ?? false;
				link = m.link;
				if (!m.link.connected) {
					st.header = null;
					mission = null;
				}
				else if (m.header) applyHeader(st, m.header, null);
				if (m.link.connected && !was) send({ type: 'request_setup' });
				flashing = false;
				break;
			}
			case 'setup': {
				const saving = st.save === 'pending';
				applyHeader(st, m.header, m.values);
				if (saving) note = st.save === 'landed' ? 'Save landed: flash holds the staged setup.' : 'Save refused: flash still differs from staged (refused while armed).';
				if (m.values) sentAt.clear();
				flashing = false;
				break;
			}
			case 'param': {
				sentAt.delete(m.index);
				const next = paramEcho(st, m.index, m.value);
				if (next !== null) sendParam(m.index, next);
				break;
			}
			case 'telemetry':
				telem = m.telemetry;
				break;
			case 'mission_state':
				mission = m.mission;
				break;
			case 'flash_log':
				flashLog = [...flashLog.slice(-499), m.line];
				break;
			case 'sim_status':
				simStatus = m.sim;
				simDone();
				break;
			case 'sim_log':
				simLog = [...simLog.slice(-999), m.line];
				break;
			case 'resources':
				resources = m.resources;
				if (terminating !== null && !m.resources.some((r) => r.holders.some((h) => h.pid === terminating))) {
					terminating = null;
					clearTimeout(terminateTimer);
				}
				break;
			case 'radio':
				radioLive = m.radio;
				break;
			case 'radio_devices':
				radioDevices = m.devices;
				break;
			case 'error': {
				if (m.request === 'radio_subscribe') {
					radioError = `${m.request}: ${m.error}`;
					break;
				}
				if (m.request === 'terminate' || m.request === 'resources_request' || m.request === 'resources_subscribe') {
					resourcesError = `${m.request}: ${m.error}`;
					terminating = null;
					clearTimeout(terminateTimer);
					break;
				}
				if (m.request === 'sim_launch' || m.request === 'sim_stop') {
					simError = `${m.request}: ${m.error}`;
					simDone();
					break;
				}
				const family = refusedFamily(m);
				if (isMissionRequest(m.request) || m.request === 'profile') missionErrors[m.request] = m.error;
				else if (family !== null) kindErrors[family] = m.error;
				else errors[requestError(st, m.request)] = `${m.request}: ${m.error}`;
				if (m.request === 'flash') flashing = false;
				break;
			}
		}
	}

	onMount(() => {
		const mock = mockFlag(new URL(location.href));
		mockMode = mock;
		let stopped = false;
		const expire = setInterval(() => {
			const now = performance.now();
			for (const [i, t] of sentAt) {
				if (now - t < ECHO_TIMEOUT_MS) continue;
				sentAt.delete(i);
				expireParam(st, i);
			}
		}, 250);
		(async () => {
			try {
				const s = await loadSchema(mock);
				if (stopped) return;
				schema = s;
				link = await loadLink(mock);
				if (stopped) return;
				void reloadAirframes();
				conn = connect(mock, s, { message: onMessage, open: (o) => {
						wsOpen = o;
						if (!o) {
							mission = null;
							simStatus = null;
							simDone();
							terminating = null;
						}
					}
				});
			} catch (e) {
				loadError = e instanceof Error ? e.message : String(e);
			}
		})();
		return () => {
			stopped = true;
			clearInterval(expire);
			clearTimeout(simTimer);
			clearTimeout(terminateTimer);
			conn?.close();
		};
	});

	function setKind(family: number, kind: number): void {
		delete errors.params;
		delete kindErrors[family];
		send({ type: 'set_kind', family, kind });
	}

	function setParam(index: number, v: number): void {
		delete errors.params;
		const now = editParam(st, index, v);
		if (now !== null) sendParam(index, now);
	}

	function resetBlock(family: number): void {
		const def = schema?.families[family]?.kinds[kinds[family] ?? 0];
		if (!def) return;
		const ref = reference(family);
		for (const p of def.params) if (!sameF32(value(p.index), ref[p.id] ?? p.default)) setParam(p.index, ref[p.id] ?? p.default);
	}

	function loadFactory(id: number): void {
		note = null;
		delete errors.factory;
		kindErrors = {};
		send({ type: 'load_factory', id });
	}

	function apply(): void {
		if (!confirm('Apply rebuilds the flight software from the staged setup. It STOPS THE MOTORS. Continue?')) return;
		note = null;
		delete errors.actions;
		send({ type: 'reset' });
	}

	function save(): void {
		note = null;
		delete errors.actions;
		st.save = 'pending';
		send({ type: 'save' });
	}

	function reboot(): void {
		if (!confirm('Reboot runs the setup stored in flash and discards staged changes. It STOPS THE MOTORS. Continue?')) return;
		note = null;
		delete errors.actions;
		send({ type: 'reboot' });
	}

	function flash(): void {
		if (!confirm('Build and flash the firmware over SWD? The link closes while it runs and the motors stop.')) return;
		flashLog = [];
		delete errors.actions;
		flashing = true;
		send({ type: 'flash' });
	}

	function exportJson(): void {
		if (!schema) return;
		const file = exportSetup(schema, kinds, st.values);
		const url = URL.createObjectURL(new Blob([JSON.stringify(file, null, '\t') + '\n'], { type: 'application/json' }));
		const a = document.createElement('a');
		a.href = url;
		a.download = `marv-setup-${factoryNow?.label ?? 'custom'}-${new Date().toISOString().slice(0, 19).replace(/[:T]/g, '-')}.json`;
		a.click();
		URL.revokeObjectURL(url);
	}

	async function importJson(e: Event): Promise<void> {
		const input = e.currentTarget as HTMLInputElement;
		const f = input.files?.[0];
		input.value = '';
		if (!f || !schema) return;
		let parsed: unknown;
		try {
			parsed = JSON.parse(await f.text());
		} catch {
			note = `Import: ${f.name} is not JSON.`;
			return;
		}
		const plan = planImport(schema, parsed);
		let k = 0;
		let n = 0;
		for (const x of plan.kinds) {
			if (kinds[x.family] === x.kind) continue;
			setKind(x.family, x.kind);
			k++;
		}
		for (const p of plan.params) {
			if (sameF32(value(p.index), p.value)) continue;
			setParam(p.index, p.value);
			n++;
		}
		const parts = [`Import ${f.name}: ${n} values and ${k} kinds sent`];
		if (plan.unknown.length) parts.push(`unknown, not applied: ${plan.unknown.join(', ')}`);
		if (plan.invalid.length) parts.push(`out of range, not applied: ${plan.invalid.join(', ')}`);
		note = parts.join('; ') + '.';
	}
</script>

<article class="content lab">
	<header class="top">
		<h1>MARV ground control</h1>
		<div class="conn mono">
			<span class="pill" class:ok={wsOpen} class:bad={!wsOpen}>backend {wsOpen ? 'open' : 'closed'}</span>
			<span class="pill">link {link?.mode || '—'}</span>
			{#if busy}
				<button type="button" class="pill bad busy" onclick={() => (tab = 'development')} title="Another process has the flight controller's USB port open, so this GCS does not open it. Development tab, Resources: see it and terminate it.">
					{busy} — see Development › Resources
				</button>
			{:else if link?.via}
				<span class="pill" class:ok={header} class:bad={!header}>
					FC via {link.via === 'usb' ? `USB (${link.target})` : link.via === 'sim-bridge' ? 'sim bridge' : link.via}{header ? '' : link.connected ? ' (waiting for setup)' : ' (not answering)'}
				</span>
			{:else if header}
				<span class="pill ok">FC connected</span>
			{:else}
				<span class="pill bad">no FC — plug in a flight controller or launch a sim</span>
			{/if}
			{#if header}<span class="pill" class:bad={armed} class:ok={!armed}>{armed ? 'ARMED' : 'disarmed'}</span>{/if}
		</div>
		{#if errors.link}<p class="err mono" role="alert">{errors.link}</p>{/if}
	</header>

	{#if loadError}<p class="banner bad mono" role="alert">Cannot load the parameter schema: {loadError}</p>{/if}
	{#if schema && header && !match}
		<p class="banner bad mono" role="alert">
			Schema mismatch: the FC reports hash {header.schema_hash.toString(16)} with {header.param_count} params; this GCS has
			{schema.schema_hash.toString(16)} with {paramCount(schema)}. Edits are refused. Flash the matching firmware or rebuild the GCS.
		</p>
	{/if}

	<section class="strip mono" aria-label="Telemetry">
		<span><span class="k">pos N,E,D</span>
			{#if telem}{telem.p_ned.map((v) => v.toFixed(2)).join(', ')} m{:else}—{/if}</span>
		<span><span class="k">roll, pitch, yaw</span>
			{#if att}{(att.roll * DEG).toFixed(1)}, {(att.pitch * DEG).toFixed(1)}, {(att.yaw * DEG).toFixed(1)} deg{:else}—{/if}</span>
		{#if vehicle === 'rocket'}
			<span><span class="k">brake</span> {telem && Number.isFinite(telem.brake) ? telem.brake.toFixed(2) : '—'}</span>
		{:else}
			<span><span class="k">hover thrust</span> {telem && Number.isFinite(telem.thrust_hover) ? telem.thrust_hover.toFixed(3) : '—'}</span>
		{/if}
		<span><span class="k">preset</span> {presetLabel}</span>
		<span><span class="k">armed</span> {telem ? (telem.armed ? 'yes' : 'no') : '—'}</span>
	</section>

	<div class="tabs mono" role="tablist">
		<button type="button" role="tab" aria-selected={view === 'mission'} class:on={view === 'mission'} onclick={() => (tab = 'mission')}>Mission</button>
		<button type="button" role="tab" aria-selected={view === 'setup'} class:on={view === 'setup'} onclick={() => (tab = 'setup')}>Setup</button>
		<button type="button" role="tab" aria-selected={view === 'controller'} class:on={view === 'controller'} onclick={() => (tab = 'controller')}>Controller</button>
		<button type="button" role="tab" aria-selected={view === 'development'} class:on={view === 'development'} onclick={() => (tab = 'development')}>
			Development{simStatus?.running ? ' · sim running' : ''}
		</button>
	</div>

	<div hidden={view !== 'mission'}>
		<MissionView {telem} {mission} profiles={schema?.profiles ?? []} connected={wsOpen && header !== null} errors={missionErrors} visible={view === 'mission'} onsend={sendMission} />
	</div>

	{#if schema}
		<section class="config" aria-label="Setup" hidden={view !== 'setup'}>
			<div class="row">
				<label class="mono preset">
					Factory setup
					<select
						value={factoryNow?.id ?? -1}
						disabled={!editable}
						onchange={(e) => loadFactory(Number((e.currentTarget as HTMLSelectElement).value))}
					>
						<option value={-1} disabled>custom</option>
						{#each schema.factory as f (f.id)}<option value={f.id}>{f.label}</option>{/each}
					</select>
				</label>
				<span class="config-label mono" class:custom={!factoryNow}>{factoryNow ? `staged = factory ${factoryNow.label}` : 'staged: custom'}</span>
				{#each words as w (w.text)}<span class="word mono {w.tone}">{w.text}</span>{/each}
				{#if errors.factory}<span class="err mono" role="alert">{errors.factory}</span>{/if}
			</div>

			<div class="row actions">
				<button type="button" class="btn mono" disabled={!editable || armed} onclick={apply} title={armed ? 'Refused while armed' : 'Rebuild the flight software from the staged setup'}>Apply</button>
				<button type="button" class="btn mono" disabled={!editable || armed} onclick={save} title={armed ? 'Refused while armed' : 'Store the staged setup in flash'}>Save to flash</button>
				<button type="button" class="btn mono" disabled={!header} onclick={reboot} title="Run the stored setup">Reboot</button>
				<button type="button" class="btn mono" disabled={!wsOpen || flashing} onclick={flash}>Flash firmware</button>
				<button type="button" class="btn mono" disabled={!loaded || !header} onclick={exportJson}>Export JSON</button>
				<label class="btn mono" class:disabled={!editable}>
					Import JSON
					<input type="file" accept="application/json,.json" hidden disabled={!editable} onchange={importJson} />
				</label>
			</div>
			{#if errors.actions}<p class="err mono" role="alert">{errors.actions}</p>{/if}
			{#if note}<p class="note mono" role="status">{note}</p>{/if}
			{#if errors.params}<p class="err mono" role="alert">{errors.params}</p>{/if}

			<DerivePanel {schema} {value} disabled={!editable} airframe={selectedAirframe} onparam={setParam} />

			<ProfilesEditor {schema} {value} {reference} rejected={st.rejected} {pending} disabled={!editable} onparam={setParam} />

			<BlockChain
				{schema}
				{kinds}
				{kindErrors}
				{value}
				{reference}
				rejected={st.rejected}
				{pending}
				disabled={!editable}
				onkind={setKind}
				onparam={setParam}
				onreset={resetBlock}
			/>
		</section>
	{/if}

	{#if schema}
		<div hidden={view !== 'controller'}>
			<ControllerView
				profiles={schema.profiles}
				live={radioLive}
				hotplug={radioDevices}
				error={radioError}
				{wsOpen}
				visible={view === 'controller'}
				mock={mockMode}
				onsubscribe={(device) => {
					radioError = null;
					send({ type: 'radio_subscribe', device });
				}}
			/>
		</div>
	{/if}

	<div hidden={view !== 'development'}>
		<DevelopmentView
			{airframes}
			{airframesError}
			selected={simAirframe}
			status={simStatus}
			log={simLog}
			error={simError}
			pending={simPending}
			{wsOpen}
			onselect={(id) => (simAirframe = id)}
			onreload={reloadAirframes}
			onsend={sendSim}
		/>
		<ResourcesPanel {resources} error={resourcesError} pending={terminating} {wsOpen} onrefresh={() => send({ type: 'resources_request' })} onterminate={terminate} />
	</div>

	{#if flashing || flashLog.length}
		<section class="section">
			<h2>Flash</h2>
			<pre class="log mono">{flashLog.join('\n')}</pre>
		</section>
	{/if}
</article>

<style>
	.content.lab {
		max-width: 80rem;
		margin: 0 auto;
		padding: 1rem 1rem 3rem;
		display: flex;
		flex-direction: column;
		gap: 1rem;
	}
	.top {
		display: flex;
		align-items: baseline;
		justify-content: space-between;
		flex-wrap: wrap;
		gap: 0.5rem 1.5rem;
	}
	.top h1 {
		margin: 0;
	}
	.conn {
		display: flex;
		flex-wrap: wrap;
		gap: 0.4rem;
		font-size: 0.75rem;
	}
	.pill {
		border: 1px solid var(--border);
		border-radius: 999px;
		padding: 0.05rem 0.55rem;
		color: var(--muted);
	}
	.pill.ok {
		color: var(--ok);
		border-color: var(--ok);
	}
	.pill.bad {
		color: var(--bad);
		border-color: var(--bad);
	}
	.pill.busy {
		background: none;
		font: inherit;
		cursor: pointer;
	}
	.banner {
		margin: 0;
		max-width: none;
		padding: 0.5rem 0.7rem;
		border: 1px solid var(--bad);
		border-radius: 8px;
		color: var(--bad);
		font-size: 0.8rem;
	}
	.strip {
		display: flex;
		flex-wrap: wrap;
		gap: 0.3rem 1.5rem;
		padding: 0.4rem 0.6rem;
		border: 1px solid var(--border);
		border-radius: 8px;
		background: var(--surface);
		font-size: 0.8rem;
		font-variant-numeric: tabular-nums;
	}
	.strip .k {
		color: var(--muted);
		margin-right: 0.3rem;
	}
	.row {
		display: flex;
		align-items: center;
		gap: 0.6rem 1.5rem;
		flex-wrap: wrap;
	}
	.row.actions {
		gap: 0.5rem;
	}
	.row select {
		background: var(--surface-2);
		color: var(--text);
		border: 1px solid var(--border);
		border-radius: 6px;
		padding: 0.2rem 0.4rem;
		font: inherit;
		margin-left: 0.4rem;
	}
	.tabs {
		display: flex;
		gap: 0.25rem;
		border-bottom: 1px solid var(--border);
	}
	.tabs button {
		background: none;
		color: var(--muted);
		border: 1px solid transparent;
		border-bottom: none;
		border-radius: 6px 6px 0 0;
		padding: 0.35rem 0.9rem;
		font: inherit;
		font-size: 0.85rem;
		cursor: pointer;
	}
	.tabs button.on {
		color: var(--text);
		border-color: var(--border);
		background: var(--surface);
	}
	.config[hidden] {
		display: none;
	}
	.config {
		display: flex;
		flex-direction: column;
		gap: 0.75rem;
		min-width: 0;
	}
	.config-label {
		font-size: 0.8rem;
		color: var(--muted);
	}
	.config-label.custom {
		color: var(--accent);
	}
	.word {
		font-size: 0.8rem;
	}
	.word.ok {
		color: var(--ok);
	}
	.word.warn {
		color: var(--warn);
	}
	.word.bad {
		color: var(--bad);
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
	.btn:disabled,
	.btn.disabled {
		opacity: 0.5;
		cursor: default;
	}
	.note {
		margin: 0;
		font-size: 0.8rem;
		color: var(--muted);
		max-width: none;
	}
	.err {
		margin: 0;
		font-size: 0.8rem;
		color: var(--bad);
		max-width: none;
	}
	.section h2 {
		margin: 0 0 0.6rem;
	}
	.log {
		margin: 0;
		max-height: 18rem;
		overflow: auto;
		padding: 0.5rem 0.7rem;
		border: 1px solid var(--border);
		border-radius: 8px;
		background: var(--surface);
		font-size: 0.75rem;
		white-space: pre-wrap;
	}
</style>
