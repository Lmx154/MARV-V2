<script lang="ts">
	import type { VehicleIdentity } from '$link/vehicle';
	import Badge from './ui/Badge.svelte';

	let { identity }: { identity: VehicleIdentity | null } = $props();

	const short = (hex: string): string => hex.slice(0, 12);
	const refused = $derived(identity?.configState === 'refused');
</script>

<div class="strip mono" aria-label="Vehicle identity">
	{#if identity}
		<span class="item" title={`image SHA-256 ${identity.imageSha256}`}><span class="k">image</span> {short(identity.imageSha256)}</span>
		<span class="item" title={`config SHA-256 ${identity.configSha256}`}><span class="k">config</span> {short(identity.configSha256)}</span>
		<span class="item" title="the parameter schema hash of the image"><span class="k">schema</span> {identity.schemaHash}</span>
		<span class="item"><span class="k">source</span> {identity.configSource}</span>
		<span class="item">
			<span class="k">state</span>
			{#if refused}<Badge text={`refused: ${identity.configRefusal}`} tone="bad" />{:else}<Badge text="ok" tone="ok" />{/if}
		</span>
		<span class="item" title="the last activation reported by the vehicle"><span class="k">last activation</span> {identity.lastActivation}</span>
		<span class="item" title={identity.validationReportId ?? 'no validation report is bound to the running config'}>
			<span class="k">report</span> {identity.validationReportId ? short(identity.validationReportId) : 'none'}
		</span>
		{#if identity.stagedDiffers}<Badge text="staged differs" tone="warn" />{/if}
	{:else}
		<span class="muted">identity: not reported by the vehicle</span>
	{/if}
</div>

<style>
	.strip {
		display: flex;
		flex-wrap: wrap;
		gap: 0.25rem 1rem;
		align-items: center;
		font-size: 0.75rem;
	}

	.k {
		color: var(--muted);
		text-transform: uppercase;
		letter-spacing: 0.06em;
		font-size: 0.65rem;
		margin-right: 0.2rem;
	}
</style>
