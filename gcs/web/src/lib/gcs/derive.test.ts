import { describe, expect, it } from 'vitest';
import { render } from 'svelte/server';
import fixture from '$lib/fixtures/schema.json';
import x3 from '../../../../../setups/x3.json';
import x500 from '../../../../../setups/x500.json';
import DerivePanel from '$lib/components/DerivePanel.svelte';
import { X3_LOOP_GAINS, blankInputs, derive, inputsError, prefillInputs, squareX, stagePlan, type DeriveInputs } from './derive';
import { MOCK_AIRFRAMES } from './mock';
import { normalizeSchema } from './protocol';
import { paramKeys } from './setup';
import { parseAirframes } from './sim';
import type { Schema } from './types';

const schema = normalizeSchema(fixture as Schema);
const [x3Frame, x500Frame] = parseAirframes(MOCK_AIRFRAMES);

/** PX4's x500 (sitl/gazebo/x500.sdf): base inertia, rotors at +-0.174 m, k 8.54858e-6, w_max 1000, k_m 0.016. */
const X500: DeriveInputs = {
	mass_kg: 2.0643,
	ixx: 0.02166666666666667,
	iyy: 0.02166666666666667,
	izz: 0.04000000000000001,
	rotors: squareX(0.174 * Math.SQRT2),
	motor_constant: 8.54858e-6,
	max_rot_velocity: 1000,
	moment_constant: 0.016
};

const close = (got: number, want: number): void => {
	if (want === 0) expect(got).toBe(0);
	else expect(Math.abs(got / want - 1)).toBeLessThan(5e-4);
};

describe('derive', () => {
	it('reproduces setups/x500.json', () => {
		const d = derive(X500, X3_LOOP_GAINS);
		close(d.t_max_n, 8.54858);
		close(d.tau_max[0], 2.9749);
		close(d.tau_max[1], 2.9749);
		close(d.tau_max[2], 0.27355);
		const file = x500.values as Record<string, number>;
		for (const [key, v] of Object.entries(d.values)) close(v, file[key]);
		expect(Object.keys(d.values).sort()).toEqual(
			[
				'vehicle.uav.hover_thrust',
				...['p', 'i', 'd', 'int_max'].flatMap((g) => ['x', 'y', 'z'].map((a) => `controller.cascaded-pid.rate_${g}_${a}`)),
				'controller.cascaded-pid.yaw_torque_max'
			].sort()
		);
	});

	it('reproduces setups/x3.json from the X3 (ADR-0009 Q5), each value capped at its parameter maximum', () => {
		const d = derive(prefillInputs(x3Frame), X3_LOOP_GAINS);
		const keys = paramKeys(schema);
		const file = x3.values as Record<string, number>;
		for (const [key, v] of Object.entries(d.values)) close(Math.min(v, keys.get(key)?.spec.max ?? NaN), file[key]);
		close(file['controller.cascaded-pid.rate_p_x'], 0.3781);
		close(file['controller.cascaded-pid.rate_p_y'], 1.2302);
		close(file['controller.cascaded-pid.rate_p_z'], 5.58);
		close(file['controller.cascaded-pid.yaw_torque_max'], 0.5712);
		close(file['vehicle.uav.hover_thrust'], 0.6811);
	});

	it('prefills from an airframe: its rotors, else a square X of its arm', () => {
		expect(prefillInputs(x500Frame)).toEqual({ ...X500, rotors: x500Frame.specs.rotors });
		const bare = { ...x500Frame.specs, rotors: undefined };
		const sq = prefillInputs({ ...x500Frame, specs: bare });
		sq.rotors.forEach((r, i) => r.forEach((v, j) => expect(v).toBeCloseTo(X500.rotors[i][j], 12)));
	});

	it('refuses incomplete inputs', () => {
		expect(inputsError(blankInputs())).toBe('mass must be positive');
		expect(inputsError({ ...X500, max_rot_velocity: 0 })).toBe('max rotor speed must be positive');
		expect(
			inputsError({
				...X500,
				rotors: [
					[0, 0.2],
					[0, -0.2],
					[0, 0.2],
					[0, -0.2]
				]
			})
		).toBe('rotors must be off both axes');
		expect(inputsError(X500)).toBeNull();
	});

	it('plans the changed, in-range values and flags the out-of-range ones', () => {
		const keys = paramKeys(schema);
		const current = (i: number): number => schema.factory[0].values[i];
		const rows = stagePlan(schema, derive(X500, X3_LOOP_GAINS), current);
		expect(rows.every((r) => r.index === keys.get(r.key)?.spec.index && r.inRange)).toBe(true);
		expect(rows.find((r) => r.key === 'controller.cascaded-pid.rate_d_z')?.changed).toBe(false);
		expect(rows.find((r) => r.key === 'controller.cascaded-pid.rate_p_x')?.changed).toBe(true);
		const heavy = stagePlan(schema, derive({ ...X500, mass_kg: 4 }, X3_LOOP_GAINS), current);
		expect(heavy.find((r) => r.key === 'vehicle.uav.hover_thrust')?.inRange).toBe(false);
	});

	it('prefill is user-initiated: a selected airframe does not fill the inputs or stage anything', () => {
		const sent: number[] = [];
		const html = render(DerivePanel, {
			props: { schema, value: (i: number) => schema.factory[0].values[i], disabled: false, airframe: x500Frame, onparam: (i: number) => void sent.push(i) }
		}).body;
		expect(html).toContain('Prefill from sim airframe (PX4 x500)');
		expect(html).not.toContain('2.0643');
		expect(html).not.toContain('derived values');
		expect(html).toContain('mass must be positive');
		expect(sent).toEqual([]);
	});
});
