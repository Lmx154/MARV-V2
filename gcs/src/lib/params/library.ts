// Which manifest belongs to the vehicle: by image hash when known, else by schema hash, else the build tree the operator picked
// (the helper's image library, GET /api/images).
import { parseManifest, parseProvenance, type Manifest, type Provenance } from './manifest';

export interface ImageEntry {
	hash: string;
	tree: string;
	name: string;
	kind: string;
	params_set: string | null;
	/** What the GCS build that produced this image recorded (tools/gs/build.py): null for an image another build left in the tree. */
	build?: { preset: string; describe: string; finished_utc: string } | null;
	artifacts: Record<string, string>;
}

export interface ImageList {
	images: ImageEntry[];
	unkeyed: Array<{ tree: string; name: string; reason: string }>;
}

export interface Loaded {
	manifest: Manifest;
	provenance: Provenance | null;
	entry: ImageEntry;
	/** How it was found. */
	how: 'image hash' | 'schema hash' | 'selected';
}

export type FetchArtifact = (hash: string, artifact: 'params_manifest' | 'params_provenance') => Promise<unknown>;

export const entryLabel = (e: ImageEntry): string => `${e.tree} / ${e.name} (${e.hash.slice(0, 12)})`;
/** The identity of a library entry: the same image can sit in two build trees. */
export const entryKey = (e: ImageEntry): string => `${e.tree}/${e.hash}`;

async function loadEntry(entry: ImageEntry, fetchArtifact: FetchArtifact): Promise<{ manifest: Manifest; provenance: Provenance | null }> {
	const manifest = parseManifest(await fetchArtifact(entry.hash, 'params_manifest'));
	let provenance: Provenance | null = null;
	if (entry.artifacts.params_provenance) {
		try {
			provenance = parseProvenance(await fetchArtifact(entry.hash, 'params_provenance'));
		} catch {
			provenance = null;
		}
	}
	return { manifest, provenance };
}

/**
 * Finds the manifest of the connected image: the library entry with the vehicle's image hash; failing that, the first entry whose
 * manifest has the vehicle's schema hash. Returns null when neither exists (the parameters are then shown read-only).
 */
export async function findManifest(list: ImageList, imageHash: string | null, schemaHash: string | null, fetchArtifact: FetchArtifact): Promise<Loaded | null> {
	const withManifest = list.images.filter((e) => e.artifacts.params_manifest);
	if (imageHash) {
		const entry = withManifest.find((e) => e.hash === imageHash);
		if (entry) return { ...(await loadEntry(entry, fetchArtifact)), entry, how: 'image hash' };
	}
	if (schemaHash) {
		for (const entry of withManifest) {
			try {
				const loaded = await loadEntry(entry, fetchArtifact);
				if (loaded.manifest.schemaHash === schemaHash.toLowerCase()) return { ...loaded, entry, how: 'schema hash' };
			} catch {
				// an unreadable manifest of another image does not stop the search
			}
		}
	}
	return null;
}

export async function loadSelected(entry: ImageEntry, fetchArtifact: FetchArtifact): Promise<Loaded> {
	return { ...(await loadEntry(entry, fetchArtifact)), entry, how: 'selected' };
}

/** What the operator asked for: the image of the connected vehicle (automatic), or one library entry. */
export type Choice = { kind: 'automatic' } | { kind: 'picked'; key: string };

export interface ManifestUse {
	loaded: Loaded | null;
	choice: Choice;
	error: string | null;
	loading: boolean;
}

export interface VehicleKey {
	imageSha256: string;
	schemaHash: string;
}

/** Which manifest is on screen and why, in one sentence for the parameter page. */
export function explainManifest(use: ManifestUse, identity: VehicleKey | null): string {
	if (use.loading) return 'loading the manifest…';
	const { loaded, choice } = use;
	if (choice.kind === 'picked') {
		if (!loaded) return 'a parameter set was picked, but its manifest could not be loaded';
		const base = `picked by you: ${entryLabel(loaded.entry)}, schema ${loaded.manifest.schemaHash}`;
		if (!identity) return base + '; no vehicle is connected (browsing the factory values)';
		const same = loaded.manifest.schemaHash === identity.schemaHash;
		return base + (same ? '; its schema is the vehicle\'s' : `; the vehicle reports schema ${identity.schemaHash}: rows are read-only`);
	}
	if (loaded) {
		const via =
			loaded.how === 'image hash'
				? `the vehicle's image hash ${identity ? identity.imageSha256.slice(0, 12) : ''} is this library entry`
				: `no library entry has the vehicle's image hash${identity ? ' ' + identity.imageSha256.slice(0, 12) : ''}; this one has its schema hash ${loaded.manifest.schemaHash}`;
		return `automatic: ${entryLabel(loaded.entry)} — ${via}`;
	}
	if (identity) return `automatic: no library entry has the vehicle's image hash ${identity.imageSha256.slice(0, 12)} or schema hash ${identity.schemaHash}; the vehicle's parameters are shown read-only`;
	return 'automatic: no vehicle is connected, so no manifest is chosen; pick a parameter set to browse its factory values';
}
