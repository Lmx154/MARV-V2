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

export interface HelperStatus {
	origin: string;
	run_dir: string;
	device: string | null;
	client: boolean;
	dropped_bytes: number;
	sil: SilStatus;
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
