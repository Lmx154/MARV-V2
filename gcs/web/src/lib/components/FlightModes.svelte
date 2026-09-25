<script lang="ts">
	import {
		BANDS_MAX,
		BANDS_MIN,
		axisValue,
		bandIndex,
		bandLower,
		evenBands,
		insertBand,
		moveBand,
		removeBand,
		setBandProfile,
		setUpper,
		type Band,
		type ProfileCfg
	} from '$lib/gcs/radio';
	import type { ProfileDef } from '$lib/gcs/types';

	interface Props {
		cfg: ProfileCfg;
		profiles: ProfileDef[];
		axes: number;
		buttons: number;
		/** The live raw axes and buttons, null without frames. */
		raw: number[] | null;
		pressed: number[] | null;
		/** Axis -> the role it already has (a stick, arm). */
		axisRole: Record<number, string>;
		/** The profile the backend computes from the stored config. */
		serverProfile: string | null;
		onchange: (p: ProfileCfg) => void;
	}
	let { cfg, profiles, axes, buttons, raw, pressed, axisRole, serverProfile, onchange }: Props = $props();

	const ids = $derived(profiles.map((p) => p.id));
	const label = (id: string | null): string => (id === null ? '—' : (profiles.find((p) => p.id === id)?.label ?? id));
	const v = $derived(raw && raw[cfg.axis] !== undefined ? axisValue(raw[cfg.axis]) : NaN);
	const current = $derived(bandIndex(cfg.bands, v));
	const pct = (u: number): string => `${(u * 100).toFixed(0)}%`;
	const rawOf = (u: number): number => Math.round(u * 32767);

	function bands(b: Band[]): void {
		onchange({ ...cfg, bands: b });
	}

	function source(s: string): void {
		const src = s === 'buttons' ? 'buttons' : s === 'none' ? 'none' : 'axis';
		onchange({ ...cfg, source: src, bands: src === 'axis' && cfg.bands.length < BANDS_MIN ? evenBands(4, ids) : cfg.bands });
	}

	function count(n: number): void {
		let b = cfg.bands;
		while (b.length < n) {
			const widest = b.reduce((w, x, i) => (x.upper - bandLower(b, i) > b[w].upper - bandLower(b, w) ? i : w), 0);
			const next = insertBand(b, widest);
			if (next === b) break;
			b = next;
		}
		while (b.length > n) b = removeBand(b, b.length - 1);
		bands(b);
	}

	function button(i: number, id: string): void {
		const m = { ...cfg.buttons };
		if (id) m[String(i)] = id;
		else delete m[String(i)];
		onchange({ ...cfg, buttons: m });
	}
</script>

<div class="modes">
	<div class="line">
		<label>
			Selected by
			<select value={cfg.source} onchange={(e) => source(e.currentTarget.value)}>
				<option value="axis">a switch channel</option>
				<option value="buttons">buttons</option>
				<option value="none">nothing (no profile input)</option>
			</select>
		</label>
		{#if cfg.source === 'axis'}
			<label>
				Channel
				<select value={cfg.axis} onchange={(e) => onchange({ ...cfg, axis: Number(e.currentTarget.value) })}>
					{#each Array.from({ length: Math.max(axes, cfg.axis + 1) }, (_, i) => i) as i (i)}
						<option value={i}>CH{i + 1} (a{i}){axisRole[i] ? ` — ${axisRole[i]}` : ''}</option>
					{/each}
				</select>
			</label>
			<label>
				Positions
				<select value={cfg.bands.length} onchange={(e) => count(Number(e.currentTarget.value))}>
					{#each Array.from({ length: BANDS_MAX - BANDS_MIN + 1 }, (_, i) => i + BANDS_MIN) as n (n)}<option value={n}>{n}</option>{/each}
				</select>
			</label>
			<button type="button" class="btn sm" onclick={() => bands(evenBands(cfg.bands.length, ids))} title="Equal bands, profiles in schema order">Even split</button>
			<span class="k">now: {Number.isFinite(v) ? `${raw?.[cfg.axis]} (${pct(v)})` : 'no data'}</span>
		{/if}
		<span class="k">backend selects: <b class="cur">{label(serverProfile)}</b></span>
	</div>

	{#if cfg.source === 'axis'}
		<div class="strip" aria-hidden="true">
			{#each cfg.bands as b, i (i)}
				<div class="seg" class:on={i === current} style:width="{((b.upper - bandLower(cfg.bands, i)) / 2) * 100}%">{label(b.profile)}</div>
			{/each}
			{#if Number.isFinite(v)}<div class="ptr" style:left="{((v + 1) / 2) * 100}%"></div>{/if}
		</div>
		<table class="specs">
			<thead><tr><th>mode</th><th>range</th><th>raw</th><th>upper edge</th><th>profile</th><th></th></tr></thead>
			<tbody>
				{#each cfg.bands as b, i (i)}
					{@const lo = bandLower(cfg.bands, i)}
					<tr class:on={i === current}>
						<td><b>{i + 1}</b>{#if i === current} <span class="tag">current</span>{/if}</td>
						<td>{pct(lo)} … {pct(b.upper)}</td>
						<td class="k">{rawOf(lo)} … {rawOf(b.upper)}</td>
						<td>
							{#if i === cfg.bands.length - 1}
								<span class="k">1.00 (end)</span>
							{:else}
								<input type="number" min="-0.99" max="0.99" step="0.01" value={b.upper} onchange={(e) => bands(setUpper(cfg.bands, i, Number(e.currentTarget.value)))} />
							{/if}
						</td>
						<td>
							<select value={b.profile} onchange={(e) => bands(setBandProfile(cfg.bands, i, e.currentTarget.value))}>
								{#if !ids.includes(b.profile)}<option value={b.profile} disabled>{b.profile || '(none)'}</option>{/if}
								{#each profiles as p (p.id)}<option value={p.id}>{p.label}</option>{/each}
							</select>
						</td>
						<td class="ops">
							<button type="button" class="btn sm" disabled={i === 0} title="Swap profile with the band below" onclick={() => bands(moveBand(cfg.bands, i, -1))}>↑</button>
							<button type="button" class="btn sm" disabled={i === cfg.bands.length - 1} title="Swap profile with the band above" onclick={() => bands(moveBand(cfg.bands, i, 1))}>↓</button>
							<button type="button" class="btn sm" disabled={cfg.bands.length >= BANDS_MAX} title="Split this band in two" onclick={() => bands(insertBand(cfg.bands, i))}>split</button>
							<button type="button" class="btn sm" disabled={cfg.bands.length <= BANDS_MIN} title="Remove this band" onclick={() => bands(removeBand(cfg.bands, i))}>×</button>
						</td>
					</tr>
				{/each}
			</tbody>
		</table>
		<p class="hint">A band runs from the edge below it up to its upper edge (switch value = raw / 32767). Move the switch: the current band lights up.</p>
	{:else if cfg.source === 'buttons'}
		<table class="specs">
			<thead><tr><th>button</th><th>pressed</th><th>profile</th></tr></thead>
			<tbody>
				{#each Array.from({ length: buttons }, (_, i) => i) as i (i)}
					{@const id = cfg.buttons[String(i)] ?? ''}
					<tr class:on={Boolean(pressed?.[i])}>
						<td><b>{i}</b></td>
						<td><span class="led" class:lit={pressed?.[i]}></span></td>
						<td>
							<select value={id} onchange={(e) => button(i, e.currentTarget.value)}>
								<option value="">— none —</option>
								{#if id && !ids.includes(id)}<option value={id} disabled>{id}</option>{/if}
								{#each profiles as p (p.id)}<option value={p.id}>{p.label}</option>{/each}
							</select>
						</td>
					</tr>
				{/each}
			</tbody>
		</table>
		<p class="hint">A press selects its profile; the last one pressed stays selected.</p>
	{:else}
		<p class="hint">No input selects a profile.</p>
	{/if}
</div>

<style>
	.modes {
		display: flex;
		flex-direction: column;
		gap: 0.6rem;
	}
	.line {
		display: flex;
		flex-wrap: wrap;
		align-items: center;
		gap: 0.5rem 1.2rem;
	}
	.strip {
		position: relative;
		display: flex;
		height: 1.6rem;
		border: 1px solid var(--border);
		border-radius: 6px;
		overflow: hidden;
		font-size: 0.72rem;
	}
	.seg {
		display: flex;
		align-items: center;
		justify-content: center;
		background: var(--surface-2);
		color: var(--muted);
		white-space: nowrap;
		overflow: hidden;
	}
	.seg + .seg {
		border-left: 2px solid var(--accent);
	}
	.seg.on {
		background: var(--accent-bg);
		color: var(--accent);
		font-weight: 600;
	}
	.ptr {
		position: absolute;
		top: 0;
		bottom: 0;
		width: 3px;
		margin-left: -1px;
		background: var(--ok);
	}
	tr.on td {
		background: var(--accent-bg);
	}
	.tag {
		color: var(--accent);
		font-size: 0.7rem;
	}
	.cur {
		color: var(--accent);
	}
	.ops {
		display: flex;
		gap: 0.25rem;
	}
	input[type='number'] {
		width: 5.5rem;
	}
	.led {
		display: inline-block;
		width: 0.8rem;
		height: 0.8rem;
		border-radius: 50%;
		border: 1px solid var(--border-strong);
		background: var(--surface-2);
	}
	.led.lit {
		background: var(--ok);
		border-color: var(--ok);
	}
</style>
