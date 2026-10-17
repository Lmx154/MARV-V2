// A small test manifest and provenance in the form tools/gen/params_gen.py writes, and the fake vehicle that carries it.
import { f32ToBits, i32ToBits } from '$link/bits';
import type { FakeParam } from '$link/testing/fake_vehicle';

export const SCHEMA = '0x3ae00eb50938acba';
export const LONG_SOURCE = 'the long source of this parameter. '.repeat(12);

interface Spec {
	name: string;
	type: 'f32' | 'i32';
	unit: string;
	linkId: string;
	value: number;
	lock?: boolean;
	method: string;
	sigmaKind: string;
	sigma?: number;
	origin?: string;
	source: string;
	file: string;
}

export const SPECS: Spec[] = [
	{ name: 'mass', type: 'f32', unit: 'kg', linkId: 'mass', value: 0.732, method: 'measured', sigmaKind: 'Unknown', source: 'Luis weighed it', file: 'card.yaml' },
	{ name: 'inertia_xx', type: 'f32', unit: 'kg m^2', linkId: 'inertia_xx', value: 0.00427, method: 'derived', sigmaKind: 'Known', sigma: 0.00107, source: LONG_SOURCE, file: 'card.yaml' },
	{ name: 'rotor_yaw_sign_m1', type: 'i32', unit: '1', linkId: 'rotor_yaw_s_2713', value: -1, method: 'derived', sigmaKind: 'Exact', source: 'ccw gives +1, cw gives -1', file: 'mixer.yaml' },
	{ name: 'arm_throttle_min', type: 'f32', unit: '1', linkId: 'arm_throttle_min', value: 0.05, lock: true, method: 'scenario', sigmaKind: 'Choice', source: 'locked by Luis', file: 'register.yaml' },
	{ name: 'estimator_gyro_bias_x', type: 'f32', unit: 'rad/s', linkId: 'estimator_gyro_b', value: 0.001, method: 'measured', sigmaKind: 'Known', sigma: 0.0002, source: 'a link id of exactly 16 bytes has no terminator', file: 'register.yaml' },
	{ name: 'rate_kp', type: 'f32', unit: '1/s', linkId: 'rate_kp', value: 12.5, method: 'design-budget', sigmaKind: 'Choice', source: 'design budget entry', file: 'register.yaml' }
];

export function manifestJson(specs: Spec[] = SPECS, schema = SCHEMA): unknown {
	const params: Record<string, unknown> = {};
	specs.forEach((s, i) => {
		const bits = s.type === 'f32' ? f32ToBits(s.value) : i32ToBits(s.value);
		params[s.name] = {
			id: i, type: s.type, unit: s.unit, link_id: s.linkId, lock: s.lock ?? false, default_bits: '0x' + bits.toString(16).padStart(8, '0'),
			method: s.method, sigma_kind: s.sigmaKind, sigma: s.sigma ?? 0, sigma_bits: '0x00000000', origin: s.origin ?? 'DefaultFromCard',
			source: s.source, source_file: s.file
		};
	});
	return { schema_hash: schema, schema_hash_algorithm: 'test', count: specs.length, params };
}

export function provenanceJson(specs: Spec[] = SPECS, schema = SCHEMA): unknown {
	return {
		schema_hash: schema,
		entries: specs.map((s) => ({
			name: s.name, source_file: s.file, shape: null, components: [s.name], sigma_kind: [s.sigmaKind],
			lock: s.lock ? { by: 'Luis', on: '2026-10-17', via: 'manual' } : null
		}))
	};
}

export function fakeParams(specs: Spec[] = SPECS): FakeParam[] {
	const origins = ['DefaultFromCard', 'DefaultFromRegister', 'Manual', 'Identified'];
	const methods = ['published', 'measured', 'identified', 'datasheet', 'derived', 'design-budget', 'scenario'];
	const kinds = ['Known', 'Exact', 'Unknown', 'Choice'];
	return specs.map((s, i) => ({
		id: i, linkId: s.linkId, type: s.type, bits: s.type === 'f32' ? f32ToBits(s.value) : i32ToBits(s.value), locked: s.lock ?? false,
		origin: origins.indexOf(s.origin ?? 'DefaultFromCard'), method: methods.indexOf(s.method), sigmaKind: kinds.indexOf(s.sigmaKind),
		sigma: s.sigma ?? 0, source: s.source
	}));
}
