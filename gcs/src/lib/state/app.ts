// The GCS's connection state: one helper connection, one link session, one vehicle client, and the manifest on screen.
// Created on first use in the browser (the app is client-rendered only).
import { derived, get, writable, type Readable } from 'svelte/store';
import { HelperConnection, type HelperEvent } from '$link/transport';
import { LinkSession } from '$link/session';
import { VehicleClient } from '$link/vehicle';
import type { ImageEntry } from '$lib/params/library';
import { HelperApi, readHelperInfo, type HelperInfo, type HelperStatus } from './helper_api';
import { ManifestController, type ManifestState } from './manifest_state';
import { Problems, type Problem } from './problems';
import { ValidationController } from './validation_state';

export type { ManifestState } from './manifest_state';

export interface App {
	info: HelperInfo | null;
	api: HelperApi | null;
	helper: HelperConnection | null;
	session: LinkSession | null;
	vehicle: VehicleClient | null;
	/** Validation and activation of the staged config (G7); null without a helper. */
	validation: ValidationController | null;
	status: Readable<HelperStatus | null>;
	/** The device the helper has open, from its events and status. */
	device: Readable<string | null>;
	manifest: Readable<ManifestState>;
	/** What went wrong, one entry per source (a later success of another source does not erase it). */
	problems: Readable<Problem[]>;
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
	/** The library is read if it has not been (a page calls this on mount). */
	ensureImages(): Promise<void>;
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
	const problems = new Problems();
	if (!info) problems.set('helper', 'No helper: open the GCS from the address the helper prints (it carries the session token).');
	const api = info ? new HelperApi(info) : null;
	const helper = info ? new HelperConnection(info.wsUrl) : null;
	const session = helper ? new LinkSession(helper) : null;
	const vehicle = session ? new VehicleClient(session) : null;
	const deviceName = writable<string | null>(null);
	const manifest = new ManifestController({
		listImages: async () => {
			if (!api) throw new Error('no helper');
			return api.images();
		},
		artifact: (hash, name) => {
			if (!api) throw new Error('no helper');
			return api.artifact(hash, name);
		},
		vehicle: () => (vehicle ? get(vehicle.state).identity : null)
	});

	const validation =
		helper && api && vehicle
			? new ValidationController({
					request: (cmd, args) => helper.request(cmd, args),
					onEvent: (handler) => helper.onEvent(handler),
					report: (imageHash, configHash) => api.report(imageHash, configHash),
					vehicle,
					manifest: () => {
						const loaded = get(manifest.state).loaded;
						const identity = get(vehicle.state).identity;
						return loaded && identity && loaded.manifest.schemaHash === identity.schemaHash ? loaded.manifest : null;
					}
				})
			: null;

	const refreshStatus = async (): Promise<void> => {
		if (!api) return;
		const s = await api.status();
		status.set(s);
		deviceName.set(s.device);
	};

	if (helper) {
		helper.onEvent((event: HelperEvent) => {
			if (event.event === 'device_opened') {
				deviceName.set(typeof event.device === 'string' ? event.device : null);
				session?.reset();
				vehicle?.reset();
				manifest.connectionChanged();
			} else if (event.event === 'device_closed') {
				deviceName.set(null);
				session?.reset();
				vehicle?.reset();
			}
			void problems.run('status', refreshStatus);
		});
	}
	// The identity arrives after the connection: load the matching manifest then.
	let lastIdentityKey = '';
	vehicle?.state.subscribe((s) => {
		const key = s.identity ? s.identity.imageSha256 + s.identity.schemaHash : '';
		if (key && key !== lastIdentityKey) {
			lastIdentityKey = key;
			void problems.run('manifest', () => manifest.identityArrived());
		} else if (!key) lastIdentityKey = '';
	});

	const refreshImages = async (): Promise<void> => {
		await problems.run('images', () => manifest.refreshImages());
	};

	return {
		info,
		api,
		helper,
		session,
		vehicle,
		validation,
		status: { subscribe: status.subscribe },
		device: derived(deviceName, (d) => d),
		manifest: manifest.state,
		problems: problems.list,
		// The image library is plain HTTP: it is read whether or not the WebSocket opens, and a second call (a remounted layout) is harmless.
		connect: async () => {
			if (!helper) return;
			const images = refreshImages();
			const state = get(helper.state);
			if (state === 'closed') {
				const failed = await problems.run('helper', () => helper.connect());
				if (!failed) await problems.run('status', refreshStatus);
			} else if (state === 'open') {
				problems.clear('helper');
				await problems.run('status', refreshStatus);
			}
			await images;
		},
		refreshStatus: async () => void (await problems.run('status', refreshStatus)),
		openSerial: async (path, baud) => void (await problems.run('device', async () => void (await helper?.request('open', { serial: path, baud })))),
		openUdp: async (port) => void (await problems.run('device', async () => void (await helper?.request('open', { udp_port: port })))),
		closeDevice: async () => void (await problems.run('device', async () => void (await helper?.request('close')))),
		silStart: async () =>
			void (await problems.run('sil', async () => {
				await helper?.request('sil_start');
				await refreshStatus();
			})),
		silStop: async () =>
			void (await problems.run('sil', async () => {
				await helper?.request('sil_stop');
				await refreshStatus();
			})),
		loadParams: async () => {
			if (!vehicle) return;
			await problems.run('params', async () => {
				if (!get(vehicle.state).identity) vehicle.requestIdentity();
				await vehicle.requestAll();
			});
		},
		selectImage: async (entry) => void (await problems.run('manifest', () => manifest.select(entry))),
		refreshImages,
		ensureImages: async () => void (await problems.run('images', () => manifest.ensureImages()))
	};
}
