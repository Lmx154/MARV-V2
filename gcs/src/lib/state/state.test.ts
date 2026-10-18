// The GUI's shared state without a browser: the problem slots, the read-button gating and the manifest controller (the
// image list, the picked image vs automatic matching, and the explanation of which manifest is in use).
import { describe, expect, it } from 'vitest';
import { get } from 'svelte/store';
import { entryKey, explainManifest, type ImageEntry, type ImageList } from '$lib/params/library';
import { manifestJson, provenanceJson, SCHEMA, SPECS } from '$lib/params/testing/fixture';
import { whyCannotRead } from './gating';
import { ManifestController } from './manifest_state';
import { Problems } from './problems';

describe('problems', () => {
	it('keeps the error of one source when another source succeeds', async () => {
		const problems = new Problems();
		expect(await problems.run('helper', async () => Promise.reject(new Error('could not connect to the helper')))).toBe('could not connect to the helper');
		expect(await problems.run('status', async () => undefined)).toBeNull();
		expect(get(problems.list)).toEqual([{ source: 'helper', message: 'could not connect to the helper' }]);
		await problems.run('helper', async () => undefined);
		expect(get(problems.list)).toEqual([]);
	});

	it('shows one entry per source, the latest message', async () => {
		const problems = new Problems();
		await problems.run('images', async () => Promise.reject(new Error('a')));
		await problems.run('images', async () => Promise.reject(new Error('b')));
		expect(get(problems.list)).toEqual([{ source: 'images', message: 'b' }]);
	});
});

describe('when parameters may be read', () => {
	const ok = { helper: 'open', device: '/dev/ttyACM1', heartbeat: 'alive' as const, listing: 'idle' as const };
	it('needs the helper, a device and a live heartbeat', () => {
		expect(whyCannotRead(ok)).toBeNull();
		expect(whyCannotRead({ ...ok, helper: 'closed' })).toContain('helper');
		expect(whyCannotRead({ ...ok, device: null })).toContain('device');
		expect(whyCannotRead({ ...ok, heartbeat: 'no heartbeat' })).toContain('no heartbeat');
		expect(whyCannotRead({ ...ok, heartbeat: undefined })).toContain('no heartbeat');
		expect(whyCannotRead({ ...ok, listing: 'loading' })).toContain('being read');
	});
});

const entry = (hash: string, tree: string): ImageEntry => ({
	hash, tree, name: 'marv_sil_quad_fc', kind: 'sil', params_set: 'marv_params_fc3v2', artifacts: { image: 'x', params_manifest: 'm', params_provenance: 'p' }
});
const A = entry('a'.repeat(64), 'host-gz');
const B = entry('b'.repeat(64), 'other');
const BSCHEMA = '0x2222222222222222';

function controller(identity: { imageSha256: string; schemaHash: string } | null = null, failList = { value: false }) {
	const list: ImageList = { images: [A, B], unkeyed: [] };
	let listCalls = 0;
	const c = new ManifestController({
		listImages: async () => {
			listCalls++;
			if (failList.value) throw new Error('/api/images: 401 missing or wrong token');
			return list;
		},
		artifact: async (hash, name) => {
			const schema = hash[0] === 'b' ? BSCHEMA : SCHEMA;
			return name === 'params_manifest' ? manifestJson(SPECS, schema) : provenanceJson(SPECS, schema);
		},
		vehicle: () => identity
	});
	return { c, calls: () => listCalls, setIdentity: (i: typeof identity) => (identity = i) };
}

describe('the image list', () => {
	it('is read on first need, and again after a failure, but not twice', async () => {
		const fail = { value: true };
		const { c, calls } = controller(null, fail);
		await expect(c.ensureImages()).rejects.toThrow('401');
		expect(get(c.state)).toMatchObject({ imagesState: 'error', imagesError: expect.stringContaining('401'), images: null });
		fail.value = false;
		await c.ensureImages();
		expect(get(c.state)).toMatchObject({ imagesState: 'ready', imagesError: null });
		expect(get(c.state).images?.images).toHaveLength(2);
		const n = calls();
		await c.ensureImages();
		expect(calls()).toBe(n);
	});
});

describe('picked image or automatic', () => {
	it('keeps the pick in the state, loads it without a vehicle, and returns to automatic on request', async () => {
		const { c } = controller();
		await c.refreshImages();
		await c.select(B);
		expect(get(c.state)).toMatchObject({ choice: { kind: 'picked', key: entryKey(B) }, loading: false, error: null });
		expect(get(c.state).loaded).toMatchObject({ how: 'selected' });
		expect(get(c.state).loaded?.manifest.schemaHash).toBe(BSCHEMA);
		await c.select(null);
		expect(get(c.state)).toMatchObject({ choice: { kind: 'automatic' }, loaded: null, error: null });
	});

	it('does not let the vehicle\'s identity override a pick, but a new connection does', async () => {
		const { c } = controller({ imageSha256: 'a'.repeat(64), schemaHash: SCHEMA });
		await c.refreshImages();
		await c.select(B);
		await c.identityArrived();
		expect(get(c.state).loaded?.entry.tree).toBe('other');
		c.connectionChanged();
		expect(get(c.state).choice).toEqual({ kind: 'automatic' });
		await c.identityArrived();
		expect(get(c.state).loaded).toMatchObject({ how: 'image hash' });
		expect(get(c.state).loaded?.entry.tree).toBe('host-gz');
	});

	it('says that nothing matches the vehicle', async () => {
		const { c } = controller({ imageSha256: 'f'.repeat(64), schemaHash: '0x9999999999999999' });
		await c.identityArrived();
		expect(get(c.state).loaded).toBeNull();
		expect(get(c.state).error).toContain('read-only');
	});

	it('discards a load that a later selection overtook', async () => {
		const { c } = controller();
		await c.refreshImages();
		const first = c.select(A);
		const second = c.select(B);
		await Promise.all([first, second]);
		expect(get(c.state).loaded?.entry.tree).toBe('other');
	});
});

describe('which manifest is in use and why', () => {
	const identity = { imageSha256: 'a'.repeat(64), schemaHash: SCHEMA };
	it('names the reason for each case', async () => {
		const { c, setIdentity } = controller(identity);
		await c.refreshImages();
		const use = () => {
			const s = get(c.state);
			return { loaded: s.loaded, choice: s.choice, error: s.error, loading: s.loading };
		};
		expect(explainManifest(use(), null)).toContain('no vehicle is connected');
		await c.identityArrived();
		expect(explainManifest(use(), identity)).toContain('image hash aaaaaaaaaaaa is this library entry');
		await c.select(B);
		expect(explainManifest(use(), identity)).toContain('picked by you');
		expect(explainManifest(use(), identity)).toContain('rows are read-only');
		expect(explainManifest(use(), null)).toContain('browsing the factory values');
		c.connectionChanged();
		const other = { imageSha256: 'f'.repeat(64), schemaHash: '0x9999999999999999' };
		setIdentity(other);
		await c.identityArrived();
		expect(explainManifest(use(), other)).toContain('no library entry has');
	});
});
