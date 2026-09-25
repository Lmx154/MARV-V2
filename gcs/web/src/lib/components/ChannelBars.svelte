<script lang="ts">
	import { RAW_MAX, RAW_MIN, axisValue, type Band, type Calibration } from '$lib/gcs/radio';

	interface Props {
		/** Axes and buttons to draw. */
		axes: number;
		buttons: number;
		/** The live raw values, null without frames. */
		raw: number[] | null;
		pressed: number[] | null;
		/** Axis -> role label ('roll', 'arm', …); button -> role label. */
		axisRole: Record<number, string>;
		buttonRole: Record<number, string>;
		/** Axis -> the configured min/center/max marks. */
		marks: Record<number, { min: number; center: number; max: number }>;
		/** The profile axis and its bands, drawn as edges on its bar. */
		bandAxis: number | null;
		bands: Band[];
		/** A calibration in progress: its captured range per axis. */
		cal: Calibration | null;
	}
	let { axes, buttons, raw, pressed, axisRole, buttonRole, marks, bandAxis, bands, cal }: Props = $props();

	const pos = (v: number): number => (100 * (Math.max(RAW_MIN, Math.min(RAW_MAX, v)) - RAW_MIN)) / (RAW_MAX - RAW_MIN);
	/** A band edge (-1..1) on the raw scale. */
	const edge = (u: number): number => pos(u * 32767);
</script>

<div class="bars">
	{#each Array.from({ length: axes }, (_, i) => i) as i (i)}
		{@const v = raw?.[i]}
		{@const m = marks[i]}
		<div class="bar-row">
			<span class="name">CH{i + 1} <span class="k">a{i}</span></span>
			<span class="role" class:set={axisRole[i]}>{axisRole[i] ?? '—'}</span>
			<div class="track" title={v === undefined ? 'no data' : `raw ${v}`}>
				<div class="mid"></div>
				{#if cal && Number.isFinite(cal.min[i]) && Number.isFinite(cal.max[i])}
					<div class="cap" style:left="{pos(cal.min[i])}%" style:width="{pos(cal.max[i]) - pos(cal.min[i])}%"></div>
				{/if}
				{#if v !== undefined}
					<div class="fill" style:left="{Math.min(50, pos(v))}%" style:width="{Math.abs(pos(v) - 50)}%"></div>
					<div class="now" style:left="{pos(v)}%"></div>
				{/if}
				{#if m}
					<div class="mark" style:left="{pos(m.min)}%" title="min {m.min}"></div>
					<div class="mark c" style:left="{pos(m.center)}%" title="center {m.center}"></div>
					<div class="mark" style:left="{pos(m.max)}%" title="max {m.max}"></div>
				{/if}
				{#if bandAxis === i}
					{#each bands.slice(0, -1) as b, j (j)}<div class="edge" style:left="{edge(b.upper)}%" title="band {j + 1} | {j + 2} at {b.upper.toFixed(2)}"></div>{/each}
				{/if}
			</div>
			<span class="val">{v === undefined ? '—' : v}</span>
			<span class="pct k">{v === undefined ? '' : `${(axisValue(v) * 100).toFixed(0)}%`}</span>
		</div>
	{/each}
</div>
{#if buttons > 0}
	<div class="btns">
		{#each Array.from({ length: buttons }, (_, i) => i) as i (i)}
			<span class="led" class:lit={pressed?.[i]} title={buttonRole[i] ?? `button ${i}`}>
				<b>{i}</b>{#if buttonRole[i]}<span class="k"> {buttonRole[i]}</span>{/if}
			</span>
		{/each}
	</div>
{/if}

<style>
	.bars {
		display: flex;
		flex-direction: column;
		gap: 0.3rem;
	}
	.bar-row {
		display: grid;
		grid-template-columns: 5rem 5.5rem 1fr 4.5rem 3rem;
		align-items: center;
		gap: 0.5rem;
		font-size: 0.75rem;
		font-variant-numeric: tabular-nums;
	}
	.role {
		color: var(--muted);
	}
	.role.set {
		color: var(--accent);
	}
	.track {
		position: relative;
		height: 1.1rem;
		border: 1px solid var(--border);
		border-radius: 4px;
		background: var(--surface-2);
		overflow: hidden;
	}
	.mid {
		position: absolute;
		left: 50%;
		top: 0;
		bottom: 0;
		border-left: 1px dashed var(--border-strong);
	}
	.cap {
		position: absolute;
		top: 0;
		bottom: 0;
		background: var(--accent-bg);
		border-left: 1px solid var(--accent);
		border-right: 1px solid var(--accent);
	}
	.fill {
		position: absolute;
		top: 3px;
		bottom: 3px;
		background: var(--ok);
		opacity: 0.55;
	}
	.now {
		position: absolute;
		top: 0;
		bottom: 0;
		width: 2px;
		margin-left: -1px;
		background: var(--ok);
	}
	.mark {
		position: absolute;
		top: 0;
		height: 40%;
		width: 2px;
		margin-left: -1px;
		background: var(--text);
	}
	.mark.c {
		height: 100%;
		opacity: 0.6;
	}
	.edge {
		position: absolute;
		top: 0;
		bottom: 0;
		width: 2px;
		margin-left: -1px;
		background: var(--accent);
	}
	.val {
		text-align: right;
	}
	.pct {
		text-align: right;
	}
	.k {
		color: var(--muted);
	}
	.btns {
		display: flex;
		flex-wrap: wrap;
		gap: 0.35rem;
		margin-top: 0.5rem;
		font-size: 0.75rem;
	}
	.led {
		border: 1px solid var(--border);
		border-radius: 999px;
		padding: 0.05rem 0.55rem;
		background: var(--surface-2);
	}
	.led.lit {
		border-color: var(--ok);
		background: var(--ok);
		color: var(--bg);
	}
	.led.lit .k {
		color: var(--bg);
	}
</style>
