import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import fixture from '$lib/fixtures/schema.json';
import { MockFc } from './mock';
import { normalizeSchema, parseServer } from './protocol';
import {
	applyHeader,
	editParam,
	emptySetup,
	exportSetup,
	kindOptions,
	paramCount,
	paramEcho,
	paramKeys,
	planImport,
	refusedFamily,
	requestError,
	saveLanded,
	schemaMatches,
	shown,
	vehicleId,
	type SetupState
} from './setup';
import type { ClientMsg, Schema, ServerMsg, SetupHeader } from './types';

const schema = normalizeSchema(fixture as Schema);
const n = paramCount(schema);
const defaults = schema.factory[0].values.slice();
const header = (over: Partial<SetupHeader> = {}): SetupHeader => ({
	schema_hash: schema.schema_hash,
	param_count: n,
	kind: schema.factory[0].kinds.slice(),
	running_crc: 1,
	staged_crc: 1,
	stored_crc: 1,
	stored_valid: true,
	armed: false,
	...over
});
const loaded = (): SetupState => {
	const st = emptySetup();
	applyHeader(st, header(), defaults);
	return st;
};
const mass = paramKeys(schema).get('controller.cascaded-pid.vel_max')!.spec;

describe('fixture schema', () => {
	it('has the seven families in tick order and absolute indices 0..n-1', () => {
		expect(schema.families.map((f) => f.id)).toEqual(['vehicle', 'sensors', 'estimator', 'guidance', 'controller', 'allocation', 'actuators']);
		const idx = [...paramKeys(schema).values()].map((r) => r.spec.index).sort((a, b) => a - b);
		expect(idx).toEqual([...Array(n).keys()]);
		expect(schemaMatches(schema, header())).toBe(true);
		expect(schemaMatches(schema, header({ schema_hash: schema.schema_hash ^ 1 }))).toBe(false);
	});
});

describe('set_param echo', () => {
	it('an echo equal to the request (in f32) is accepted', () => {
		const st = loaded();
		expect(editParam(st, mass.index, 1.7)).toBe(1.7);
		paramEcho(st, mass.index, Math.fround(1.7));
		expect(st.rejected[mass.index]).toBeUndefined();
		expect(st.values[mass.index]).toBe(Math.fround(1.7));
	});

	it('an echo that differs is a rejected edit, and the held value is shown', () => {
		const st = loaded();
		editParam(st, mass.index, 99);
		paramEcho(st, mass.index, defaults[mass.index]);
		expect(st.rejected[mass.index]).toEqual({ requested: 99, held: defaults[mass.index] });
		expect(shown(st, mass.index)).toBe(defaults[mass.index]);
	});

	it('edits made while a request is outstanding are coalesced and sent on the echo', () => {
		const st = loaded();
		expect(editParam(st, mass.index, 1.6)).toBe(1.6);
		expect(editParam(st, mass.index, 1.7)).toBeNull();
		expect(editParam(st, mass.index, 1.8)).toBeNull();
		expect(shown(st, mass.index)).toBe(1.8);
		expect(paramEcho(st, mass.index, Math.fround(1.6))).toBe(1.8);
		expect(paramEcho(st, mass.index, Math.fround(1.8))).toBeNull();
		expect(st.rejected[mass.index]).toBeUndefined();
	});
});

describe('save', () => {
	it('landed iff stored_crc == staged_crc in the reply', () => {
		expect(saveLanded(header({ staged_crc: 7, stored_crc: 7 }))).toBe(true);
		expect(saveLanded(header({ staged_crc: 7, stored_crc: 3 }))).toBe(false);

		const st = loaded();
		st.save = 'pending';
		applyHeader(st, header({ staged_crc: 7, stored_crc: 3, armed: true }), null);
		expect(st.save).toBe('refused');
		st.save = 'pending';
		applyHeader(st, header({ staged_crc: 7, stored_crc: 7 }), null);
		expect(st.save).toBe('landed');
	});
});

describe('export / import', () => {
	it('round-trips by id', () => {
		const kinds = schema.factory[2].kinds.slice();
		const values = defaults.slice();
		values[mass.index] = 2.25;
		const file = JSON.parse(JSON.stringify(exportSetup(schema, kinds, values)));
		expect(file.values['controller.cascaded-pid.vel_max']).toBe(2.25);
		const plan = planImport(schema, file);
		expect(plan.unknown).toEqual([]);
		expect(plan.invalid).toEqual([]);
		const back = defaults.map(() => NaN);
		for (const p of plan.params) back[p.index] = p.value;
		expect(back).toEqual(values);
		const k = schema.families.map(() => -1);
		for (const x of plan.kinds) k[x.family] = x.kind;
		expect(k).toEqual(kinds);
	});

	it('reports unknown ids and out-of-range values without applying them', () => {
		const file = exportSetup(schema, schema.factory[0].kinds, defaults);
		file.values['vehicle.quad-x3.wingspan'] = 3;
		file.values['controller.cascaded-pid.vel_max'] = mass.max + 1;
		file.kinds.estimator = 'ukf';
		const plan = planImport(schema, file);
		expect(plan.unknown.sort()).toEqual(['estimator=ukf', 'vehicle.quad-x3.wingspan']);
		expect(plan.invalid).toEqual(['controller.cascaded-pid.vel_max']);
		expect(plan.params.some((p) => p.index === mass.index)).toBe(false);
		expect(plan.params).toHaveLength(n - 1);
		expect(plan.kinds.some((x) => schema.families[x.family].id === 'estimator')).toBe(false);
	});
});

describe('protocol', () => {
	it('parses a setup reply with and without values', () => {
		const withValues = parseServer(JSON.stringify({ type: 'setup', header: header(), values: [1, 2] }));
		expect(withValues).toMatchObject({ type: 'setup', values: [1, 2] });
		const flat = parseServer(JSON.stringify({ type: 'setup', ...header() }));
		expect(flat).toMatchObject({ type: 'setup', values: null, header: { param_count: n } });
	});

	it('parses a backend error and places it next to the action that caused it', () => {
		const m = parseServer(JSON.stringify({ type: 'error', request: 'save', error: 'no reply' }));
		expect(m).toEqual({ type: 'error', request: 'save', error: 'no reply' });
		const st = loaded();
		st.save = 'pending';
		expect(requestError(st, 'save')).toBe('actions');
		expect(st.save).toBe('idle');
		expect(requestError(st, 'load_factory')).toBe('factory');
		expect(requestError(st, 'set_param')).toBe('params');
		expect(requestError(st, 'request_setup')).toBe('link');
	});
});

describe('vehicle classes', () => {
	const fam = (id: string): number => schema.families.findIndex((f) => f.id === id);
	const ids = (family: number, kinds: number[]): string[] => kindOptions(schema, family, kinds).map((o) => o.def.id);
	const uav = schema.factory[0].kinds.slice();
	const rocket = schema.factory.find((f) => vehicleId(schema, f.kinds) === 'rocket')!.kinds.slice();

	it('the vehicle card lists every class; the others only kinds that serve the staged vehicle', () => {
		expect(vehicleId(schema, uav)).toBe('uav');
		expect(ids(fam('vehicle'), uav)).toEqual(['uav', 'rocket']);
		expect(ids(fam('vehicle'), rocket)).toEqual(['uav', 'rocket']);
		expect(ids(fam('guidance'), uav)).toEqual(['passthrough', 'trajectory']);
		expect(ids(fam('guidance'), rocket)).toEqual(['apogee-predictor']);
		expect(ids(fam('allocation'), rocket)).toEqual(['rocket-brake']);
		expect(ids(fam('estimator'), rocket)).toEqual(schema.families[fam('estimator')].kinds.map((k) => k.id));
		for (const kinds of [uav, rocket])
			schema.families.forEach((_, fi) => {
				if (fi !== fam('vehicle')) for (const o of kindOptions(schema, fi, kinds)) expect(o.def.vehicles).toContain(vehicleId(schema, kinds));
			});
	});

	describe('with the mock FC', () => {
		beforeEach(() => vi.useFakeTimers());
		afterEach(() => vi.useRealTimers());

		const run = () => {
			const fc = new MockFc(schema, '');
			const msgs: ServerMsg[] = [];
			fc.onmessage = (ev) => {
				const m = parseServer(ev.data);
				if (m && m.type !== 'telemetry') msgs.push(m);
			};
			const st = loaded();
			const send = (m: ClientMsg): ServerMsg[] => {
				msgs.length = 0;
				fc.send(JSON.stringify(m));
				vi.advanceTimersByTime(40);
				for (const r of msgs) if (r.type === 'setup') applyHeader(st, r.header, r.values);
				return msgs.slice();
			};
			return { fc, st, send };
		};

		it('after a vehicle change the kinds are the ones the header reports, not assumed', () => {
			const { fc, st, send } = run();
			const before = st.header!.kind.slice();
			fc.send(JSON.stringify({ type: 'set_kind', family: fam('vehicle'), kind: 1 }));
			expect(st.header!.kind).toEqual(before); // nothing changes until the FC answers
			const replies = send({ type: 'request_setup' });
			expect(replies.some((r) => r.type === 'error')).toBe(false);
			const kinds = st.header!.kind;
			expect(vehicleId(schema, kinds)).toBe('rocket');
			expect(schema.families[fam('guidance')].kinds[kinds[fam('guidance')]].id).toBe('apogee-predictor');
			expect(schema.families[fam('allocation')].kinds[kinds[fam('allocation')]].id).toBe('rocket-brake');
			expect(kinds[fam('estimator')]).toBe(before[fam('estimator')]);
			fc.close();
		});

		it('a refused kind comes back as an error for its card, and the held kind stays', () => {
			const { fc, st, send } = run();
			send({ type: 'set_kind', family: fam('vehicle'), kind: 1 });
			const held = st.header!.kind[fam('guidance')];
			const replies = send({ type: 'set_kind', family: fam('guidance'), kind: 0 });
			const err = replies.find((r) => r.type === 'error');
			expect(err?.type === 'error' ? refusedFamily(err) : null).toBe(fam('guidance'));
			expect(err?.type === 'error' ? err.error : '').toMatch(/refused/);
			expect(st.header!.kind[fam('guidance')]).toBe(held);
			expect(refusedFamily({ request: 'set_param' })).toBeNull();
			fc.close();
		});
	});
});
