/**
 * Dev-only stand-in for the backend + FC (?mock): answers the client messages with the setup semantics of gcs decision
 * (b), (c). ?mock=armed starts armed (save refused); ?mock=mismatch reports another schema hash.
 */
import type { ClientMsg, Schema, SetupHeader } from './types';

interface Setup {
	kind: number[];
	values: number[];
}

const copy = (s: Setup): Setup => ({ kind: s.kind.slice(), values: s.values.slice() });

/** CRC-32 over kinds and f32 values, standing in for the FC's crc of its Setup. */
function crc(s: Setup): number {
	const buf = new DataView(new ArrayBuffer(s.kind.length + 4 * s.values.length));
	s.kind.forEach((k, i) => buf.setUint8(i, k));
	s.values.forEach((v, i) => buf.setFloat32(s.kind.length + 4 * i, v, true));
	let c = 0xffffffff;
	for (let i = 0; i < buf.byteLength; i++) {
		c ^= buf.getUint8(i);
		for (let b = 0; b < 8; b++) c = c & 1 ? (c >>> 1) ^ 0xedb88320 : c >>> 1;
	}
	return (c ^ 0xffffffff) >>> 0;
}

export class MockFc {
	onopen: (() => void) | null = null;
	onmessage: ((ev: { data: string }) => void) | null = null;
	onclose: (() => void) | null = null;

	private readonly specs: { min: number; max: number }[] = [];
	private staged: Setup;
	private running: Setup;
	private stored: Setup;
	private storedValid = true;
	private armed: boolean;
	private readonly hash: number;
	private readonly timer: ReturnType<typeof setInterval>;
	private t = 0;

	constructor(
		private readonly schema: Schema,
		flag: string
	) {
		for (const f of schema.families) for (const k of f.kinds) for (const p of k.params) this.specs[p.index] = { min: p.min, max: p.max };
		const f0 = schema.factory[0];
		this.staged = { kind: f0.kinds.slice(), values: f0.values.map((v) => Math.fround(v)) };
		this.running = copy(this.staged);
		this.stored = copy(this.staged);
		this.armed = flag === 'armed';
		this.hash = flag === 'mismatch' ? (schema.schema_hash ^ 0x5a5a5a5a) >>> 0 : schema.schema_hash;
		setTimeout(() => {
			this.onopen?.();
			this.emit({ type: 'link', mode: 'mock', connected: true, header: this.header() });
		}, 50);
		this.timer = setInterval(() => this.telemetry(), 50);
	}

	close(): void {
		clearInterval(this.timer);
		this.onclose?.();
	}

	send(text: string): void {
		const m = JSON.parse(text) as ClientMsg;
		// The link's latency, so pending edits are visible.
		setTimeout(() => this.handle(m), 30);
	}

	private handle(m: ClientMsg): void {
		switch (m.type) {
			case 'request_setup':
				return this.setup(true);
			case 'set_param': {
				const s = this.specs[m.index];
				if (s && Number.isFinite(m.value) && m.value >= s.min && m.value <= s.max) this.staged.values[m.index] = Math.fround(m.value);
				return this.emit({ type: 'param', index: m.index, value: this.staged.values[m.index] ?? 0 });
			}
			case 'set_kind':
				if (this.schema.families[m.family]?.kinds[m.kind]) this.staged.kind[m.family] = m.kind;
				return this.setup(false);
			case 'load_factory': {
				const f = this.schema.factory.find((x) => x.id === m.id);
				if (f) this.staged = { kind: f.kinds.slice(), values: f.values.map((v) => Math.fround(v)) };
				return this.setup(true);
			}
			case 'save':
				if (!this.armed) {
					this.stored = copy(this.staged);
					this.storedValid = true;
				}
				return this.setup(false);
			case 'reset':
				this.running = copy(this.staged);
				this.armed = false;
				return this.setup(false);
			case 'reboot':
				this.running = copy(this.stored);
				this.staged = copy(this.stored);
				this.armed = false;
				return this.setup(true);
			case 'flash':
				return this.flash();
		}
	}

	private flash(): void {
		const lines = ['[mock] closing serial port', '[mock] cmake --build build/fw', '[100%] Built target marv_fw', '[mock] openocd: ** Programming Finished **', '[mock] ** Verified OK **', '[mock] reopening serial port'];
		lines.forEach((line, i) => setTimeout(() => this.emit({ type: 'flash_log', line }), 200 * (i + 1)));
		setTimeout(() => {
			this.running = copy(this.stored);
			this.staged = copy(this.stored);
			this.emit({ type: 'link', mode: 'mock', connected: true, header: this.header() });
			this.setup(true);
		}, 200 * (lines.length + 1));
	}

	private header(): SetupHeader {
		return {
			schema_hash: this.hash,
			param_count: this.staged.values.length,
			kind: this.staged.kind.slice(),
			running_crc: crc(this.running),
			staged_crc: crc(this.staged),
			stored_crc: crc(this.stored),
			stored_valid: this.storedValid,
			armed: this.armed
		};
	}

	private setup(values: boolean): void {
		this.emit(values ? { type: 'setup', header: this.header(), values: this.staged.values } : { type: 'setup', header: this.header() });
	}

	private telemetry(): void {
		this.t += 0.05;
		const r = crc(this.running);
		const preset = this.schema.factory.find((f) => crc({ kind: f.kinds, values: f.values }) === r)?.id ?? 0xff;
		const yaw = 0.3 * this.t;
		const roll = 0.05 * Math.sin(this.t);
		const q = [Math.cos(yaw / 2) * Math.cos(roll / 2), Math.cos(yaw / 2) * Math.sin(roll / 2), -Math.sin(yaw / 2) * Math.sin(roll / 2), Math.sin(yaw / 2) * Math.cos(roll / 2)];
		this.emit({ type: 'telemetry', est: { p_ned: [2 * Math.cos(0.3 * this.t), 2 * Math.sin(0.3 * this.t), -1.5], q }, preset, armed: this.armed });
	}

	private emit(m: unknown): void {
		this.onmessage?.({ data: JSON.stringify(m) });
	}
}
