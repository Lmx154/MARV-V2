<script lang="ts">
	import { onMount } from 'svelte';
	import L from 'leaflet';
	import 'leaflet/dist/leaflet.css';
	import { ACCEPT_WP_M } from '$lib/gcs/mission';
	import type { LatLonAlt } from '$lib/gcs/types';

	interface Props {
		home: LatLonAlt | null;
		/** The vehicle and its heading (deg, clockwise from north). */
		vehicle: { lat: number; lon: number; yaw_deg: number } | null;
		waypoints: LatLonAlt[];
		/** The executor's current target, highlighted. */
		target: LatLonAlt | null;
		/** Index of the waypoint being flown, or -1. */
		active: number;
		visible: boolean;
		onpick: (lat: number, lon: number) => void;
	}
	let { home, vehicle, waypoints, target, active, visible, onpick }: Props = $props();

	const TRAIL_POINTS = 300;
	const TRAIL_STEP_M = 0.3;

	let el: HTMLDivElement;
	let map = $state.raw<L.Map | null>(null);
	let homeMarker: L.Marker | null = null;
	let vehicleMarker: L.Marker | null = null;
	let targetMarker: L.CircleMarker | null = null;
	let wpLayer: L.LayerGroup | null = null;
	let path: L.Polyline | null = null;
	let trail: L.Polyline | null = null;
	const trailPts: L.LatLng[] = [];
	let centred = false;

	const icon = (cls: string, html: string, size: number): L.DivIcon =>
		L.divIcon({ className: `mm ${cls}`, html, iconSize: [size, size], iconAnchor: [size / 2, size / 2] });

	onMount(() => {
		const m = L.map(el).setView([20, 0], 2);
		L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
			maxZoom: 19,
			attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
		}).addTo(m);
		path = L.polyline([], { className: 'mm-path', weight: 2, dashArray: '6 6' }).addTo(m);
		trail = L.polyline([], { className: 'mm-trail', weight: 3 }).addTo(m);
		wpLayer = L.layerGroup().addTo(m);
		m.on('click', (e: L.LeafletMouseEvent) => onpick(e.latlng.lat, e.latlng.lng));
		map = m;
		return () => {
			m.remove();
			map = null;
		};
	});

	$effect(() => {
		if (!map || !home) return;
		const at = L.latLng(home.lat, home.lon);
		if (homeMarker) homeMarker.setLatLng(at);
		else homeMarker = L.marker(at, { icon: icon('mm-home', 'H', 22), title: 'home', keyboard: false }).addTo(map);
		if (!centred) {
			map.setView(at, 18);
			centred = true;
		}
	});

	$effect(() => {
		if (!map || !vehicle) return;
		const at = L.latLng(vehicle.lat, vehicle.lon);
		if (!vehicleMarker) vehicleMarker = L.marker(at, { icon: icon('mm-vehicle', '<div class="mm-arrow"></div>', 26), keyboard: false, zIndexOffset: 1000 }).addTo(map);
		else vehicleMarker.setLatLng(at);
		const arrow = vehicleMarker.getElement()?.querySelector<HTMLElement>('.mm-arrow');
		if (arrow) arrow.style.transform = `rotate(${vehicle.yaw_deg}deg)`;
		const last = trailPts.at(-1);
		if (!last || map.distance(last, at) > TRAIL_STEP_M) {
			trailPts.push(at);
			if (trailPts.length > TRAIL_POINTS) trailPts.shift();
			trail?.setLatLngs(trailPts);
		}
	});

	$effect(() => {
		if (!map || !wpLayer || !path) return;
		wpLayer.clearLayers();
		waypoints.forEach((w, i) => {
			if (!Number.isFinite(w.lat) || !Number.isFinite(w.lon)) return;
			const cls = i === active ? 'mm-wp mm-active' : 'mm-wp';
			L.circle([w.lat, w.lon], { radius: ACCEPT_WP_M, className: 'mm-accept', weight: 1, interactive: false }).addTo(wpLayer!);
			L.marker([w.lat, w.lon], { icon: icon(cls, String(i + 1), 22), title: `${i + 1}: ${w.alt_m} m`, keyboard: false }).addTo(wpLayer!);
		});
		const pts = waypoints.filter((w) => Number.isFinite(w.lat) && Number.isFinite(w.lon)).map((w) => L.latLng(w.lat, w.lon));
		path.setLatLngs(home && pts.length ? [L.latLng(home.lat, home.lon), ...pts, L.latLng(home.lat, home.lon)] : pts);
	});

	$effect(() => {
		if (!map) return;
		if (!target) {
			targetMarker?.remove();
			targetMarker = null;
			return;
		}
		const at = L.latLng(target.lat, target.lon);
		if (targetMarker) targetMarker.setLatLng(at);
		else targetMarker = L.circleMarker(at, { className: 'mm-target', radius: 16, weight: 3, fill: false, interactive: false }).addTo(map);
	});

	$effect(() => {
		if (map && visible) map.invalidateSize();
	});
</script>

<div class="map" bind:this={el} aria-label="Mission map"></div>

<style>
	.map {
		height: 30rem;
		min-height: 18rem;
		border: 1px solid var(--border);
		border-radius: 8px;
		cursor: crosshair;
	}
	:global(.mm) {
		display: flex;
		align-items: center;
		justify-content: center;
		font: 700 0.7rem var(--mono);
		border-radius: 50%;
	}
	:global(.mm-home) {
		background: var(--ok);
		color: #fff;
		border: 2px solid #fff;
	}
	:global(.mm-wp) {
		background: var(--l-software);
		color: #fff;
		border: 2px solid #fff;
	}
	:global(.mm-wp.mm-active) {
		background: var(--accent);
	}
	:global(.mm-arrow) {
		width: 0;
		height: 0;
		border-left: 8px solid transparent;
		border-right: 8px solid transparent;
		border-bottom: 22px solid var(--bad);
		filter: drop-shadow(0 0 1px #fff);
	}
	:global(.mm-path) {
		stroke: var(--l-software);
	}
	:global(.mm-trail) {
		stroke: var(--bad);
		stroke-opacity: 0.6;
	}
	:global(.mm-accept) {
		stroke: var(--l-software);
		fill: var(--l-software);
		fill-opacity: 0.1;
	}
	:global(.mm-target) {
		stroke: var(--accent);
	}
</style>
