// The manifest on screen: the image library, the operator's choice (automatic = the connected image, or one library entry) and
// what was loaded. One source of truth for every page; the pages only read it and ask it to select. Stale loads are discarded.
import { writable, type Readable } from 'svelte/store';
import {
	entryKey,
	findManifest,
	loadSelected,
	type Choice,
	type FetchArtifact,
	type ImageEntry,
	type ImageList,
	type Loaded,
	type VehicleKey
} from '$lib/params/library';

export type ImagesState = 'idle' | 'loading' | 'ready' | 'error';

export interface ManifestState {
	loaded: Loaded | null;
	loading: boolean;
	/** Why nothing (or the wrong thing) is loaded. */
	error: string | null;
	images: ImageList | null;
	imagesState: ImagesState;
	imagesError: string | null;
	choice: Choice;
}

export interface ManifestDeps {
	listImages(): Promise<ImageList>;
	artifact: FetchArtifact;
	/** The connected vehicle's identity, or null. */
	vehicle(): VehicleKey | null;
}

export class ManifestController {
	private readonly store = writable<ManifestState>({
		loaded: null, loading: false, error: null, images: null, imagesState: 'idle', imagesError: null, choice: { kind: 'automatic' }
	});
	readonly state: Readable<ManifestState> = { subscribe: this.store.subscribe };
	private current!: ManifestState;
	private generation = 0;

	constructor(private readonly deps: ManifestDeps) {
		this.store.subscribe((s) => (this.current = s));
	}

	/** Reads the library; throws (after recording the error in the state) when the helper cannot list it. */
	async refreshImages(): Promise<void> {
		this.store.update((s) => ({ ...s, imagesState: 'loading', imagesError: null }));
		try {
			const images = await this.deps.listImages();
			this.store.update((s) => ({ ...s, images, imagesState: 'ready', imagesError: null }));
		} catch (e) {
			const message = e instanceof Error ? e.message : String(e);
			this.store.update((s) => ({ ...s, imagesState: 'error', imagesError: message }));
			throw e;
		}
	}

	/** The library is read on first need: a page calls this on mount. */
	async ensureImages(): Promise<void> {
		if (this.current.imagesState === 'idle' || this.current.imagesState === 'error') await this.refreshImages();
	}

	/** null = automatic. */
	async select(entry: ImageEntry | null): Promise<void> {
		this.store.update((s) => ({ ...s, choice: entry ? { kind: 'picked', key: entryKey(entry) } : { kind: 'automatic' } }));
		await this.resolve(entry);
	}

	/** A new connection: whatever was picked for browsing no longer decides; the new vehicle's image does. */
	connectionChanged(): void {
		if (this.current.choice.kind === 'picked') this.store.update((s) => ({ ...s, choice: { kind: 'automatic' } }));
	}

	/** The vehicle's identity arrived: load its manifest unless the operator picked one. */
	async identityArrived(): Promise<void> {
		if (this.current.choice.kind === 'picked') return;
		await this.resolve(null);
	}

	private pickedEntry(): ImageEntry | null {
		const choice = this.current.choice;
		if (choice.kind !== 'picked') return null;
		return this.current.images?.images.find((e) => entryKey(e) === choice.key) ?? null;
	}

	private async resolve(picked: ImageEntry | null): Promise<void> {
		const mine = ++this.generation;
		this.store.update((s) => ({ ...s, loading: true, error: null }));
		try {
			const images = await this.deps.listImages();
			if (mine !== this.generation) return;
			const identity = this.deps.vehicle();
			const entry = picked ?? this.pickedEntry();
			let loaded: Loaded | null;
			if (entry) loaded = await loadSelected(entry, this.deps.artifact);
			else loaded = await findManifest(images, identity?.imageSha256 ?? null, identity?.schemaHash ?? null, this.deps.artifact);
			if (mine !== this.generation) return;
			this.store.update((s) => ({
				...s,
				loaded,
				loading: false,
				images,
				imagesState: 'ready',
				imagesError: null,
				error: loaded || entry ? null : identity ? "No image in the library has this vehicle's parameters; they are shown read-only." : null
			}));
		} catch (e) {
			if (mine !== this.generation) return;
			this.store.update((s) => ({ ...s, loaded: null, loading: false, error: e instanceof Error ? e.message : String(e) }));
		}
	}
}
