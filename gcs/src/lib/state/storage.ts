// localStorage that may be blocked (privacy settings, a sandboxed frame): every access is guarded, a failure means "no stored value".
export interface StorageLike {
	getItem(key: string): string | null;
	setItem(key: string, value: string): void;
	removeItem(key: string): void;
}

const browserStorage = (): StorageLike | null => {
	try {
		return typeof localStorage === 'undefined' ? null : localStorage;
	} catch {
		return null;
	}
};

export function readSetting(key: string, storage: StorageLike | null = browserStorage()): string | null {
	try {
		return storage ? storage.getItem(key) : null;
	} catch {
		return null;
	}
}

/** null removes the key. Returns false when the storage refused. */
export function writeSetting(key: string, value: string | null, storage: StorageLike | null = browserStorage()): boolean {
	try {
		if (!storage) return false;
		if (value === null) storage.removeItem(key);
		else storage.setItem(key, value);
		return true;
	} catch {
		return false;
	}
}
