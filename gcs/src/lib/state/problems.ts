// The problems the GUI shows at the top of every page: one slot per source, so that a later success of one source does not
// erase the error of another (a status poll every two seconds used to clear a failed connect).
import { derived, writable, type Readable } from 'svelte/store';

export interface Problem {
	/** The source of the problem: 'helper', 'status', 'images', 'device', 'sil', 'params', 'manifest'. */
	source: string;
	message: string;
}

export class Problems {
	private readonly slots = writable<Record<string, string>>({});
	readonly list: Readable<Problem[]> = derived(this.slots, (s) => Object.entries(s).map(([source, message]) => ({ source, message })));

	set(source: string, message: string): void {
		this.slots.update((s) => (s[source] === message ? s : { ...s, [source]: message }));
	}

	clear(source: string): void {
		this.slots.update((s) => {
			if (!(source in s)) return s;
			const { [source]: _gone, ...rest } = s;
			return rest;
		});
	}

	/** Runs `what`; its failure becomes the problem of `source` (and is returned), its success clears that source only. */
	async run(source: string, what: () => Promise<void>): Promise<string | null> {
		try {
			await what();
			this.clear(source);
			return null;
		} catch (e) {
			const message = e instanceof Error ? e.message : String(e);
			this.set(source, message);
			return message;
		}
	}
}
