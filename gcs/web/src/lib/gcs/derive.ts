/**
 * The setup's airframe-dependent values from the airframe's physical specs (ADR-0009 Q4, Q5): hover thrust
 * m g / (n k w_max^2), and each rate-loop gain as the physical loop gain scaled by the axis's inertia over its full
 * torque authority, K_n = K_phys I / tau_max, with the quad-x allocation's +-0.5 factors:
 * tau_max roll = 0.5 T_max sum|y|, pitch = 0.5 T_max sum|x|, yaw = 0.5 T_max k_m n, T_max = k w_max^2.
 */
import { paramKeys, sameF32 } from './setup';
import type { Airframe, Schema } from './types';

/** Standard gravity (m/s^2). */
export const G = 9.80665;

export interface DeriveInputs {
	mass_kg: number;
	ixx: number;
	iyy: number;
	izz: number;
	/** Rotor positions (x forward, y; m), one per motor of the quad-x. */
	rotors: [number, number][];
	/** Thrust per rotor = k w^2 (N s^2/rad^2). */
	motor_constant: number;
	/** rad/s */
	max_rot_velocity: number;
	/** Yaw torque per rotor = k_m thrust (m). */
	moment_constant: number;
}

/** The physical loop the rate controller should close, per axis x, y, z: bandwidth choices, not airframe specs. */
export interface LoopGains {
	/** 1/s */
	rate_p: [number, number, number];
	/** 1/s^2 */
	rate_i: [number, number, number];
	/** dimensionless */
	rate_d: [number, number, number];
	/** Angular acceleration the rate integrator may command (rad/s^2). */
	rate_int_accel: number;
	/** Yaw torque limit (N m). */
	yaw_torque_nm: number;
}

/** The X3's design choices behind factory 0 (ADR-0009 Q5). */
export const X3_LOOP_GAINS: LoopGains = { rate_p: [25, 25, 10], rate_i: [62.5, 62.5, 10], rate_d: [0.05, 0.05, 0], rate_int_accel: 5, yaw_torque_nm: 0.1 };

export const ROTORS = 4;

export const blankInputs = (): DeriveInputs => ({
	mass_kg: NaN,
	ixx: NaN,
	iyy: NaN,
	izz: NaN,
	rotors: Array.from({ length: ROTORS }, (): [number, number] => [NaN, NaN]),
	motor_constant: NaN,
	max_rot_velocity: NaN,
	moment_constant: NaN
});

/** A square X of arm length a (centre to rotor), in the quad-x motor order 0 FR, 1 RL, 2 FL, 3 RR (x forward, y right). */
export function squareX(a: number): [number, number][] {
	const h = a / Math.SQRT2;
	return [
		[h, h],
		[-h, -h],
		[h, -h],
		[-h, h]
	];
}

/** The inputs from a sim airframe's specs: its rotor positions when reported, else a square X of its arm length. */
export function prefillInputs(a: Airframe): DeriveInputs {
	const s = a.specs;
	return {
		mass_kg: s.mass_kg,
		ixx: s.ixx,
		iyy: s.iyy,
		izz: s.izz,
		rotors: s.rotors && s.rotors.length === ROTORS ? s.rotors.map((r): [number, number] => [r[0], r[1]]) : squareX(s.arm_m),
		motor_constant: s.motor_constant,
		max_rot_velocity: s.max_rot_velocity,
		moment_constant: s.moment_constant
	};
}

const pos = (v: number): boolean => typeof v === 'number' && Number.isFinite(v) && v > 0;

/** Why the inputs cannot be derived from, or null. */
export function inputsError(i: DeriveInputs): string | null {
	if (!pos(i.mass_kg)) return 'mass must be positive';
	if (!pos(i.ixx) || !pos(i.iyy) || !pos(i.izz)) return 'Ixx, Iyy, Izz must be positive';
	if (i.rotors.length !== ROTORS || !i.rotors.every((r) => r.every((v) => typeof v === 'number' && Number.isFinite(v)))) return `${ROTORS} rotor positions are needed`;
	if (!pos(i.motor_constant)) return 'motor thrust constant k must be positive';
	if (!pos(i.max_rot_velocity)) return 'max rotor speed must be positive';
	if (!pos(i.moment_constant)) return 'moment constant must be positive';
	const sx = i.rotors.reduce((a, r) => a + Math.abs(r[0]), 0);
	const sy = i.rotors.reduce((a, r) => a + Math.abs(r[1]), 0);
	if (!(sx > 0) || !(sy > 0)) return 'rotors must be off both axes';
	return null;
}

/** Why the loop gains are unusable, or null. */
export function gainsError(g: LoopGains): string | null {
	const ok = (v: number): boolean => typeof v === 'number' && Number.isFinite(v) && v >= 0;
	return [...g.rate_p, ...g.rate_i, ...g.rate_d, g.rate_int_accel, g.yaw_torque_nm].every(ok) ? null : 'loop gains must be 0 or more';
}

export interface Derived {
	/** Full thrust of one rotor (N). */
	t_max_n: number;
	/** Full torque authority roll, pitch, yaw (N m). */
	tau_max: [number, number, number];
	/** Param key (family.kind.id) -> value. */
	values: Record<string, number>;
}

const C = 'controller.cascaded-pid';

export function derive(i: DeriveInputs, g: LoopGains): Derived {
	const n = i.rotors.length;
	const tMax = i.motor_constant * i.max_rot_velocity ** 2;
	const sx = i.rotors.reduce((a, r) => a + Math.abs(r[0]), 0);
	const sy = i.rotors.reduce((a, r) => a + Math.abs(r[1]), 0);
	const tau: [number, number, number] = [0.5 * tMax * sy, 0.5 * tMax * sx, 0.5 * tMax * i.moment_constant * n];
	const inertia = [i.ixx, i.iyy, i.izz];
	const values: Record<string, number> = { 'vehicle.uav.hover_thrust': (i.mass_kg * G) / (n * tMax) };
	(['x', 'y', 'z'] as const).forEach((ax, k) => {
		const s = inertia[k] / tau[k];
		values[`${C}.rate_p_${ax}`] = g.rate_p[k] * s;
		values[`${C}.rate_i_${ax}`] = g.rate_i[k] * s;
		values[`${C}.rate_d_${ax}`] = g.rate_d[k] * s;
		values[`${C}.rate_int_max_${ax}`] = g.rate_int_accel * s;
	});
	values[`${C}.yaw_torque_max`] = g.yaw_torque_nm / tau[2];
	return { t_max_n: tMax, tau_max: tau, values };
}

export interface StageRow {
	key: string;
	label: string;
	unit: string;
	/** -1 when this schema has no such param. */
	index: number;
	value: number;
	current: number;
	/** Within the param's range: the FC would take it. */
	inRange: boolean;
	/** Differs from the staged value. */
	changed: boolean;
}

/** Each derived value against the staged one; staging sends the in-range, changed rows as set_param. */
export function stagePlan(schema: Schema, d: Derived, current: (index: number) => number): StageRow[] {
	const keys = paramKeys(schema);
	return Object.entries(d.values).map(([key, value]) => {
		const ref = keys.get(key);
		if (!ref) return { key, label: key, unit: '', index: -1, value, current: NaN, inRange: false, changed: false };
		const now = current(ref.spec.index);
		const inRange = Number.isFinite(value) && value >= ref.spec.min && value <= ref.spec.max;
		return { key, label: ref.spec.label, unit: ref.spec.unit ?? '', index: ref.spec.index, value, current: now, inRange, changed: !sameF32(value, now) };
	});
}
