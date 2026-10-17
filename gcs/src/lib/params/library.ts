// Which manifest belongs to the vehicle: by image hash when known, else by schema hash, else the build tree the operator picked
// (the helper's image library, GET /api/images).
import { parseManifest, parseProvenance, type Manifest, type Provenance } from './manifest';

export interface ImageEntry {
	hash: string;
	tree: string;
	name: string;
	kind: string;
	params_set: string | null;
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
