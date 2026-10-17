// The GCS's connection state: one helper connection, one link session, one vehicle client, and the manifest on screen.
// Created on first use in the browser (the app is client-rendered only).
import { derived, get, writable, type Readable } from 'svelte/store';
import { HelperConnection, type HelperEvent } from '$link/transport';
import { LinkSession } from '$link/session';
import { VehicleClient } from '$link/vehicle';
import { findManifest, loadSelected, type ImageEntry, type ImageList, type Loaded } from '$lib/params/library';
import { HelperApi, readHelperInfo, type HelperInfo, type HelperStatus } from './helper_api';

export interface ManifestState {
	loaded: Loaded | null;
	loading: boolean;
	error: string | null;
	images: ImageList | null;
}

export interface App {
	info: HelperInfo | null;
	api: HelperApi | null;
	helper: HelperConnection | null;
	session: LinkSession | null;
	vehicle: VehicleClient | null;
	status: Readable<HelperStatus | null>;
	/** The device the helper has open, from its events and status. */
	device: Readable<string | null>;
	manifest: Readable<ManifestState>;
	error: Readable<string | null>;
	connect(): Promise<void>;
	refreshStatus(): Promise<void>;
	openSerial(path: string, baud: number): Promise<void>;
	openUdp(port: number): Promise<void>;
	closeDevice(): Promise<void>;
	silStart(): Promise<void>;
	silStop(): Promise<void>;
	loadParams(): Promise<void>;
	selectImage(entry: ImageEntry | null): Promise<void>;
	refreshImages(): Promise<void>;
}

let instance: App | null = null;

export function app(): App {
	if (!instance) instance = create();
	return instance;
}

function create(): App {
	const meta = document.querySelector('meta[name="marv-helper"]')?.getAttribute('content') ?? null;
	const info = readHelperInfo(meta, location.search, location.origin);
	const status = writable<HelperStatus | null>(null);
	const manifest = writable<ManifestState>({ loaded: null, loading: false, error: null, images: null });
	const error = writable<string | null>(info ? null : 'No helper: open the GCS from the address the helper prints (it carries the session token).');
	const api = info ? new HelperApi(info) : null;
	const helper = info ? new HelperConnection(info.wsUrl) : null;
	const session = helper ? new LinkSession(helper) : null;
	const vehicle = session ? new VehicleClient(session) : null;
	const deviceName = writable<string | null>(null);
	let selected: ImageEntry | null = null;

	const run = async (what: () => Promise<void>): Promise<void> => {
		try {
			error.set(null);
			await what();
		} catch (e) {
			error.set(e instanceof Error ? e.message : String(e));
		}
	};

	const refreshStatus = async (): Promise<void> => {
		if (!api) return;
		const s = await api.status();
		status.set(s);
		deviceName.set(s.device);
	};

	const refreshImages = async (): Promise<void> => {
		if (!api) return;
		const images = await api.images();
		manifest.update((m) => ({ ...m, images }));
	};

	const resolveManifest = async (): Promise<void> => {
		if (!api) return;
		manifest.update((m) => ({ ...m, loading: true, error: null }));
		try {
			const images = await api.images();
			const identity = vehicle ? get(vehicle.state).identity : null;
			const fetchArtifact = (hash: string, name: string): Promise<unknown> => api.artifact(hash, name);
			let loaded: Loaded | null = null;
			if (selected) loaded = await loadSelected(selected, fetchArtifact);
			else loaded = await findManifest(images, identity?.imageSha256 ?? null, identity?.schemaHash ?? null, fetchArtifact);
			manifest.set({ loaded, loading: false, error: loaded || selected ? null : 'No image in the library has this vehicle\'s parameters; they are shown read-only.', images });
		} catch (e) {
			manifest.update((m) => ({ ...m, loading: false, error: e instanceof Error ? e.message : String(e) }));
		}
	};

	if (helper) {
		helper.onEvent((event: HelperEvent) => {
			if (event.event === 'device_opened') {
				deviceName.set(typeof event.device === 'string' ? event.device : null);
				session?.reset();
				vehicle?.reset();
			} else if (event.event === 'device_closed') {
				deviceName.set(null);
				session?.reset();
				vehicle?.reset();
			}
			void refreshStatus().catch(() => undefined);
		});
	}
	// The identity arrives after the connection: load the matching manifest then.
	let lastIdentityKey = '';
	vehicle?.state.subscribe((s) => {
		const key = s.identity ? s.identity.imageSha256 + s.identity.schemaHash : '';
		if (key && key !== lastIdentityKey && !selected) {
			lastIdentityKey = key;
			void resolveManifest();
		} else if (!key) lastIdentityKey = '';
	});

	const loadParams = async (): Promise<void> => {
		if (!vehicle) return;
		await run(async () => {
			if (!get(vehicle.state).identity) vehicle.requestIdentity();
			await vehicle.requestAll();
		});
	};

	return {
		info,
		api,
		helper,
		session,
		vehicle,
		status: { subscribe: status.subscribe },
		device: derived(deviceName, (d) => d),
		manifest: { subscribe: manifest.subscribe },
		error: { subscribe: error.subscribe },
		connect: () => run(async () => {
			if (!helper) return;
			await helper.connect();
			await refreshStatus();
			await refreshImages();
		}),
		refreshStatus: () => run(refreshStatus),
		openSerial: (path, baud) => run(async () => void (await helper?.request('open', { serial: path, baud }))),
		openUdp: (port) => run(async () => void (await helper?.request('open', { udp_port: port }))),
		closeDevice: () => run(async () => void (await helper?.request('close'))),
		silStart: () => run(async () => {
			await helper?.request('sil_start');
			await refreshStatus();
		}),
		silStop: () => run(async () => {
			await helper?.request('sil_stop');
			await refreshStatus();
		}),
		loadParams,
		selectImage: async (entry) => {
			selected = entry;
			await resolveManifest();
		},
		refreshImages: () => run(refreshImages)
	};
}
