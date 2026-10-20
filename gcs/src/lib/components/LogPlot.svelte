<script lang="ts">
	import { formatSeconds, formatValue, plotModel } from '$lib/state/logs_state';

	/** One field of a series from the helper (min and max per time bucket), drawn as a vertical stroke per bucket. */
	let {
		field,
		min,
		max,
		t0Us,
		t1Us,
		originUs
	}: { field: string; min: Array<number | null>; max: Array<number | null>; t0Us: number; t1Us: number; originUs: number } = $props();

	// The drawing box; the SVG scales to its container.
	const WIDTH = 600;
	const HEIGHT = 140;

	const model = $derived(plotModel(min, max, WIDTH, HEIGHT));
</script>

<figure class="plot" data-plot-field={field}>
	<figcaption class="mono">{field}</figcaption>
	<svg role="img" aria-label={`${field}: minimum to maximum in each of ${min.length} time buckets`} viewBox={`0 0 ${WIDTH} ${HEIGHT}`} preserveAspectRatio="none">
		<rect class="frame" x="0" y="0" width={WIDTH} height={HEIGHT} />
		<path d={model.path} />
	</svg>
	<div class="axis mono small"><span>{formatSeconds(t0Us, originUs)} s</span><span>{formatSeconds(t1Us, originUs)} s</span></div>
	<dl class="facts mono small">
		<dt>min</dt><dd data-plot="min">{formatValue(model.dataMin)}</dd>
		<dt>max</dt><dd data-plot="max">{formatValue(model.dataMax)}</dd>
		<dt>buckets with data</dt><dd data-plot="filled">{model.filled} of {min.length}</dd>
	</dl>
</figure>

<style>
	.plot {
		margin: 0 0 0.75rem;
	}

	figcaption {
		margin: 0 0 0.2rem;
		color: var(--muted);
	}

	svg {
		display: block;
		width: 100%;
		height: 9rem;
		background: var(--surface-2);
		border: 1px solid var(--border);
	}

	.frame {
		fill: none;
	}

	path {
		fill: none;
		stroke: var(--accent);
		stroke-width: 1.5;
		stroke-linecap: round; /* a bucket with one value is a zero-length stroke: a dot, not nothing */
		vector-effect: non-scaling-stroke;
	}

	.axis {
		display: flex;
		justify-content: space-between;
		color: var(--muted);
	}

	.small {
		font-size: 0.8rem;
	}

	.facts {
		display: grid;
		grid-template-columns: max-content 1fr;
		gap: 0.15rem 1rem;
		margin: 0.25rem 0 0;
	}

	.facts dt {
		color: var(--muted);
	}

	.facts dd {
		margin: 0;
	}
</style>
