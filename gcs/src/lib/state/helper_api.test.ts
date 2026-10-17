import { describe, expect, it } from 'vitest';
import { HelperApi, readHelperInfo } from './helper_api';

describe('where the helper is', () => {
	it('reads the meta tag the helper injects into its pages', () => {
		const info = readHelperInfo('{"token":"abc","ws":"/ws"}', '', 'http://127.0.0.1:8765');
		expect(info).toEqual({ base: 'http://127.0.0.1:8765', token: 'abc', wsUrl: 'ws://127.0.0.1:8765/ws?token=abc' });
	});

	it('takes the helper and the token from the address of a dev server', () => {
		const info = readHelperInfo(null, '?helper=http://127.0.0.1:9000/&token=t%2Fk', 'http://localhost:5173');
		expect(info).toEqual({ base: 'http://127.0.0.1:9000', token: 't/k', wsUrl: 'ws://127.0.0.1:9000/ws?token=t%2Fk' });
	});

	it('finds no helper in a page without either', () => {
		expect(readHelperInfo(null, '', 'http://127.0.0.1:1')).toBeNull();
		expect(readHelperInfo('not json', '', 'http://127.0.0.1:1')).toBeNull();
		expect(readHelperInfo('{"token":1}', '', 'http://127.0.0.1:1')).toBeNull();
	});
});

describe('the helper\'s HTTP API', () => {
	it('sends the token as a Bearer header and returns the JSON', async () => {
		const seen: Array<{ url: string; auth: string | null }> = [];
		const fetcher = (async (url: string, init?: RequestInit) => {
			seen.push({ url, auth: new Headers(init?.headers).get('Authorization') });
			return new Response(JSON.stringify({ serial: ['/dev/ttyACM1'] }), { status: 200 });
		}) as unknown as typeof fetch;
		const api = new HelperApi({ base: 'http://127.0.0.1:1', token: 'tok', wsUrl: '' }, fetcher);
		expect(await api.ports()).toEqual(['/dev/ttyACM1']);
		expect(seen).toEqual([{ url: 'http://127.0.0.1:1/api/ports', auth: 'Bearer tok' }]);
	});

	it('fails with the status and text of a refusal', async () => {
		const fetcher = (async () => new Response('missing or wrong token\n', { status: 401 })) as unknown as typeof fetch;
		const api = new HelperApi({ base: 'http://127.0.0.1:1', token: 'x', wsUrl: '' }, fetcher);
		await expect(api.status()).rejects.toThrow('401 missing or wrong token');
	});
});
