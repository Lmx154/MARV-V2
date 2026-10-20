// The GCS's connection state: one helper connection, one link session, one vehicle client, and the manifest on screen.
// Created on first use in the browser (the app is client-rendered only).
import { derived, get, writable, type Readable } from 'svelte/store';
import { HelperConnection, type HelperConnectionState } from '$link/transport';
import { LinkSession } from '$link/session';
import { ConsoleChannel, consoleUrlOf } from '$lib/console/console_channel';
import { ConsoleController } from '$lib/console/console_state';
import { VehicleClient, type VehicleState } from '$link/vehicle';
import type { ImageEntry } from '$lib/params/library';
import { mergeParams, type ParamRow } from '$lib/params/merge';
import { HelperApi, readHelperInfo, type HelperInfo, type HelperStatus } from './helper_api';
import { FlashController } from './flash_state';
import { channelMapOf, LiveController } from './live_state';
import { LogsController } from './logs_state';
import { ManifestController, type ManifestState } from './manifest_state';
import { PilotController } from './pilot_state';
import { RadioController } from './radio_state';
import { Problems, type Problem } from './problems';
import { BenchController } from './bench_state';
import { SimController } from './sim_state';
import { ShellController, type AutoconnectEvent, type Indicator } from './shell';
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
	/** Flashing an image of the library through the G7 gate; null without a helper. */
	flash: FlashController | null;
	/** The Vehicle page: the live panels of the vehicle's telemetry; null without a helper. */
	live: LiveController | null;
	/** The Radio page: the receiver channels, their mapping, the thresholds and the display checks; null without a helper. */
	radio: RadioController | null;
	/** The manifest merged with what the vehicle reports (the Parameters page's rows); empty until a manifest or the parameters are known. */
	paramRows: Readable<readonly ParamRow[]>;
	/** The Simulation page: the SIL run definition, its plan, launch and stop; null without a helper. */
	sim: SimController | null;
	/** The Bench page: the HIL preflight and launch, the verdict form and the SWD flash of the bench images; null without a helper. */
	bench: BenchController | null;
	/** The SIL pilot bridge: the RadioMaster's USB joystick, opened and closed through the helper; null without a helper. */
	pilot: PilotController | null;
	/** The Console page: the board's console text through the helper, opened and closed by the operator; null without a helper. */
	console: ConsoleController | null;
	/** The Logs page: the .mlog library of the helper, the decoder's report and the decimated series; null without a helper. */
	logs: LogsController | null;
	status: Readable<HelperStatus | null>;
	/** The device the helper has open, from its events and status. */
	device: Readable<string | null>;
	/** The shell: the status bar's indicators and what keeps them true. */
	bar: Readable<Indicator[]>;
	/** The latest autoconnect event the helper pushed. */
	autoconnect: Readable<AutoconnectEvent | null>;
	manifest: Readable<ManifestState>;
	/** What went wrong, one entry per source (a later success of another source does not erase it). */
	problems: Readable<Problem[]>;
	connect(): Promise<void>;
	refreshStatus(): Promise<void>;
	openSerial(path: string, baud: number): Promise<void>;
	openUdp(port: number): Promise<void>;
	closeDevice(): Promise<void>;
	setAutoconnect(enabled: boolean): Promise<void>;
	/** Starts the status polling and the event handling of the shell; returns what stops them. */
	startShell(): () => void;
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
	const problems = new Problems();
	if (!info) problems.set('helper', 'No helper: open the GCS from the address the helper prints (it carries the session token).');
	const api = info ? new HelperApi(info) : null;
	const helper = info ? new HelperConnection(info.wsUrl) : null;
	const session = helper ? new LinkSession(helper) : null;
	const vehicle = session ? new VehicleClient(session) : null;
	const identity = vehicle ? derived(vehicle.state, (v) => v.identity) : null;
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

	const shell = new ShellController({
		readStatus: async () => {
			if (!api) throw new Error('no helper');
			return api.status();
		},
		onEvent: (handler) => (helper ? helper.onEvent(handler) : () => undefined),
		helperState: helper ? helper.state : writable<HelperConnectionState>('closed'),
		session: session ? session.state : null,
		identity,
		guard: (source, what) => problems.run(source, what),
		deviceOpened: () => {
			session?.reset();
			vehicle?.reset();
			manifest.connectionChanged();
		},
		deviceClosed: () => {
			session?.reset();
			vehicle?.reset();
		},
		// The SIL runs again behind the same link: the vehicle's identity is read again from its next beat.
		relaunched: () => {
			session?.reset();
			vehicle?.reset();
		}
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

	const flash =
		helper && session && vehicle
			? new FlashController({
					request: (cmd, args) => helper.request(cmd, args),
					onEvent: (handler) => helper.onEvent(handler),
					vehicle,
					session,
					helperState: helper.state,
					device: shell.device
				})
			: null;

	const live = vehicle
		? new LiveController({ telemetry: vehicle.telemetry, channelMap: derived(vehicle.state, (v) => channelMapOf(v.params)) })
		: null;

	const paramRows = derived([manifest.state, vehicle ? vehicle.state : writable<VehicleState | null>(null)], ([m, v]) =>
		mergeParams(m.loaded?.manifest ?? null, m.loaded?.provenance ?? null, v && v.params.length > 0 ? v.params : null).rows
	);
	const radio = vehicle ? new RadioController({ telemetry: vehicle.telemetry, rows: paramRows }) : null;

	const sim =
		helper && api
			? new SimController({
					request: (cmd, args) => helper.request(cmd, args),
					listImages: () => api.images(),
					artifact: (hash, name) => api.artifact(hash, name),
					refresh: () => shell.refresh()
				})
			: null;

	const bench =
		helper && api
			? new BenchController({
					request: (cmd, args) => helper.request(cmd, args),
					refresh: () => shell.refresh()
				})
			: null;
	const pilot = helper ? new PilotController({ request: (cmd, args) => helper.request(cmd, args), onEvent: (handler) => helper.onEvent(handler) }) : null;
	pilot?.start();
	const consoleChannel = info && helper ? new ConsoleChannel({ url: consoleUrlOf(info.wsUrl) }) : null;
	const consoleView =
		helper && consoleChannel ? new ConsoleController({ request: (cmd, args) => helper.request(cmd, args), onEvent: (handler) => helper.onEvent(handler), channel: consoleChannel }) : null;
	consoleView?.start();
	const logs = helper ? new LogsController({ request: (cmd, args) => helper.request(cmd, args), onEvent: (handler) => helper.onEvent(handler) }) : null;
	logs?.start();

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
		flash,
		live,
		radio,
		paramRows,
		sim,
		bench,
		pilot,
		console: consoleView,
		logs,
		status: shell.status,
		device: shell.device,
		bar: shell.bar,
		autoconnect: shell.autoconnect,
		manifest: manifest.state,
		problems: problems.list,
		// The image library is plain HTTP: it is read whether or not the WebSocket opens, and a second call (a remounted layout) is harmless.
		connect: async () => {
			if (!helper) return;
			consoleChannel?.start();
			const images = refreshImages();
			const state = get(helper.state);
			if (state === 'closed') {
				const failed = await problems.run('helper', () => helper.connect());
				if (!failed) await shell.refresh();
			} else if (state === 'open') {
				problems.clear('helper');
				await shell.refresh();
			}
			await images;
		},
		refreshStatus: () => shell.refresh(),
		openSerial: async (path, baud) => void (await problems.run('device', async () => void (await helper?.request('open', { serial: path, baud })))),
		openUdp: async (port) => void (await problems.run('device', async () => void (await helper?.request('open', { udp_port: port })))),
		closeDevice: async () => void (await problems.run('device', async () => void (await helper?.request('close')))),
		setAutoconnect: async (enabled) =>
			void (await problems.run('device', async () => {
				await helper?.request('autoconnect', { enabled });
				await shell.refresh();
			})),
		startShell: () => {
			shell.start();
			return () => shell.stop();
		},
		silStart: async () =>
			void (await problems.run('sil', async () => {
				await helper?.request('sil_start');
				await shell.refresh();
			})),
		silStop: async () =>
			void (await problems.run('sil', async () => {
				await helper?.request('sil_stop');
				await shell.refresh();
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
