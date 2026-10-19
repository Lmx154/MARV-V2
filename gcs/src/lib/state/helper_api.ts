// The helper's HTTP side (tools/gs/helper.py): status, ports, the image library and the image artifacts. The token is sent as
// a Bearer header; the WebSocket URL carries it as a query (a browser WebSocket cannot set headers).
import type { ImageList } from '$lib/params/library';

export interface HelperInfo {
	/** http origin of the helper, no trailing slash. */
	base: string;
	token: string;
	wsUrl: string;
}

export interface SilStatus {
	running: boolean;
	pid: number | null;
	exit_status: number | null;
	udp_port: number | null;
	link: unknown;
}

/** The manifest of a supervised run (tools/gs/runs.py), the fields the shell reads. */
export interface RunManifest {
	run_id: string;
	kind: string;
	state: 'running' | 'stopped' | 'exited';
	exit_status: number | null;
	relaunches: number;
	/** 'sil-override' when the run has a firmware parameter override, else null. */
	label?: string | null;
	/** The overrides of a sil run, with the type the image's manifest gives each. */
	overrides?: Array<{ name: string; type: string; value: number }>;
	/** The options the run was started with (a sil run's card and gui). */
	options?: { card?: string; gui?: boolean } & Record<string, unknown>;
	argv?: string[];
	started_utc?: string;
	ended_utc?: string | null;
}

export interface RunsStatus {
	/** The run that is going (relaunch gaps included), else null. */
	active: { id: string; kind: string } | null;
	/** The latest run of this helper session, ended or not. */
	last: RunManifest | null;
}

export interface HostState {
	loadavg: number[] | null;
	cpus: number | null;
	mem_total_bytes: number | null;
	mem_available_bytes: number | null;
	heavy_lock: { held: boolean; holder: string | null; path: string };
}

export interface AutoconnectStatus {
	enabled: boolean;
	suspended: string | null;
	/** 'sil' or 'board' while the open device was opened by autoconnect. */
	owner: string | null;
}

export interface HelperStatus {
	origin: string;
	run_dir: string;
	device: string | null;
	client: boolean;
	dropped_bytes: number;
	sil: SilStatus;
	runs: RunsStatus;
	host: HostState;
	autoconnect: AutoconnectStatus;
	validation: { running: boolean; image_hash?: string; config_hash?: string; verdict?: string | null; report_id?: string | null } | null;
	gcs_built: boolean;
}

/**
 * Reads where the helper is and the session token. The page served by the helper carries <meta name="marv-helper" content='{"token":..,"ws":..}'>
 * and is on the helper's origin. A vite dev server (the helper started with --extra-origin) is given `?helper=<origin>&token=<token>`.
 */
export function readHelperInfo(metaContent: string | null, search: string, pageOrigin: string): HelperInfo | null {
	const query = new URLSearchParams(search);
	const helper = query.get('helper');
	const queryToken = query.get('token');
	if (helper && queryToken) return build(helper.replace(/\/$/, ''), queryToken, '/ws');
	if (metaContent) {
		try {
			const meta = JSON.parse(metaContent) as { token?: unknown; ws?: unknown };
			if (typeof meta.token === 'string' && typeof meta.ws === 'string') return build(pageOrigin, meta.token, meta.ws);
		} catch {
			return null;
		}
	}
	return null;
}

function build(base: string, token: string, wsPath: string): HelperInfo {
	const url = new URL(wsPath, base);
	url.protocol = url.protocol === 'https:' ? 'wss:' : 'ws:';
	url.searchParams.set('token', token);
	return { base, token, wsUrl: url.toString() };
}

export class HelperApi {
	constructor(
		private readonly info: HelperInfo,
		private readonly fetcher: typeof fetch = (...args) => fetch(...args)
	) {}

	private async get(path: string): Promise<Response> {
		const response = await this.fetcher(this.info.base + path, { headers: { Authorization: `Bearer ${this.info.token}` } });
		if (!response.ok) throw new Error(`${path}: ${response.status} ${(await response.text()).trim()}`);
		return response;
	}

	async status(): Promise<HelperStatus> {
		return (await (await this.get('/api/status')).json()) as HelperStatus;
	}

	async ports(): Promise<string[]> {
		return ((await (await this.get('/api/ports')).json()) as { serial: string[] }).serial;
	}

	async images(): Promise<ImageList> {
		return (await (await this.get('/api/images')).json()) as ImageList;
	}

	async artifact(hash: string, name: string): Promise<unknown> {
		return await (await this.get(`/api/images/${hash}/${name}`)).json();
	}

	/** The validation report of (image hash, config hash): its exact bytes, whose SHA-256 is the report id. */
	async report(imageHash: string, configHash: string): Promise<Uint8Array> {
		return new Uint8Array(await (await this.get(`/api/reports/${imageHash}/${configHash}`)).arrayBuffer());
	}
}
