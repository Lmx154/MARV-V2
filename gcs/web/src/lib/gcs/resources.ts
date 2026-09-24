/** The Development tab's Resources panel: who holds the rig's resources (the FC's USB port, the rig lock, the sim, the
 * bridge's ground port, the GCS) and the Terminate control, and the header badge while the FC's port is held. */
import type { LinkInfo, ResourceHolder, RigResource } from './types';

type Obj = Record<string, unknown>;
const isObj = (v: unknown): v is Obj => typeof v === 'object' && v !== null && !Array.isArray(v);
const finite = (v: unknown): v is number => typeof v === 'number' && Number.isFinite(v);

/** The link's holder {name, pid}, or null. */
export function linkHolder(v: unknown): { name: string; pid: number } | null {
	if (!isObj(v)) return null;
	return { name: typeof v.name === 'string' && v.name ? v.name : 'another process', pid: finite(v.pid) ? v.pid : 0 };
}

function holder(v: unknown): ResourceHolder | null {
	if (!isObj(v) || !finite(v.pid) || v.pid <= 0) return null;
	return {
		pid: v.pid,
		name: typeof v.name === 'string' ? v.name : '',
		cmdline: typeof v.cmdline === 'string' ? v.cmdline : '',
		started: finite(v.started) ? v.started : null,
		self: v.self === true,
		child_of_launcher: v.child_of_launcher === true
	};
}

/** The resources list, loosely: entries without an id are dropped, as are holders without a pid. */
export function parseResources(v: unknown): RigResource[] {
	if (!Array.isArray(v)) return [];
	const out: RigResource[] = [];
	for (const r of v) {
		if (!isObj(r) || typeof r.id !== 'string' || !r.id) continue;
		out.push({
			id: r.id,
			label: typeof r.label === 'string' && r.label ? r.label : r.id,
			path: typeof r.path === 'string' && r.path ? r.path : null,
			holders: Array.isArray(r.holders) ? r.holders.map(holder).filter((h): h is ResourceHolder => h !== null) : []
		});
	}
	return out;
}

/** The header badge while the FC's USB port is held by another process, or null. */
export function busyBadge(link: LinkInfo | null): string | null {
	if (link?.via !== 'busy') return null;
	const h = link.holder;
	if (!h) return 'FC busy — held by another process';
	return `FC busy — held by ${h.name}${h.pid > 0 ? ` (pid ${h.pid})` : ''}`;
}

/** Why a holder's Terminate button is disabled, or null when it is enabled. */
export function terminateBlocked(h: ResourceHolder, wsOpen: boolean, pending: number | null): string | null {
	if (h.self) return 'This is the ground control backend itself: stop it from its terminal';
	if (!wsOpen) return 'The backend is not connected';
	if (pending !== null) return `Waiting for pid ${pending} to exit`;
	return null;
}

/** The confirm dialog's question, naming the process and what it holds. */
export function terminateQuestion(r: RigResource, h: ResourceHolder): string {
	const what = `${h.name || 'process'} (pid ${h.pid})`;
	const holds = `${r.label}${r.path ? ` ${r.path}` : ''}`;
	if (h.child_of_launcher) return `Stop the sim this tab launched? ${what} holds ${holds}; the whole sim stops (as Stop does).`;
	return `Terminate ${what}? It holds ${holds}.\n\n${h.cmdline}\n\nSIGTERM now, SIGKILL if it is still running 5 s later.`;
}

/** The holder's age, e.g. "3 h 12 min", from its start (unix s) to now (unix ms). */
export function age(started: number | null, nowMs: number): string {
	if (started === null) return '—';
	const s = Math.max(0, Math.floor(nowMs / 1000 - started));
	if (s < 60) return `${s} s`;
	if (s < 3600) return `${Math.floor(s / 60)} min`;
	if (s < 86400) return `${Math.floor(s / 3600)} h ${Math.floor((s % 3600) / 60)} min`;
	return `${Math.floor(s / 86400)} d ${Math.floor((s % 86400) / 3600)} h`;
}
