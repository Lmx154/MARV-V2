"""Analytic and high-accuracy references for the L2 T4 scenarios (docs/decisions/0003 items 7, 9, 11), pure Python.

Frames NED / FRD, SI units, time in seconds, binary64. The NED origin is the site at latitude phi and height h0 above
the WGS 84 ellipsoid, so the height at down-position p_D is h0 - p_D. Gravity is tools/sim/hover.py `normal_gravity`
(WGS 84, NIMA TR8350.2 eq. (4-3)), not restated here; every function takes a `gravity(phi, h)` callable so a test can
inject a constant or a wrong-sign height gradient.

Method (free fall and hover). The motion is  p'' = g(phi, h0 - p) - T(t)/m,  p(0) = v(0) = 0, with T = 0 (free fall)
or T(t) = 4 k w(t)^2, w(t) = w_D (1 - exp(-t/tau)) (hover, item 7; w_D is the ESC map at DShot D, linear in omega:
48 -> omega_min, 2047 -> omega_max, the same expression order as sim/plant/src/plant_model.hpp). With g0 = g(phi, h0)
the motion splits exactly as p = p_c + delta, where p_c is the closed-form solution for the constant g0:
  free fall   p_c = g0 t^2 / 2                                     v_c = g0 t
  hover       v_c = g0 t - a_T [t - 2 tau (1 - E) + (tau/2)(1 - E^2)],  E = exp(-t/tau)
              p_c = g0 t^2 / 2 - a_T [t^2/2 - 2 tau (t - tau (1 - E)) + (tau/2)(t - (tau/2)(1 - E^2))]
              a_T = 4 k w_D^2 / m
and delta is the height-gradient correction,  delta'' = g(phi, h0 - p_c - delta) - g0,  delta(0) = delta'(0) = 0,
solved by classical RK4 (order 4, Hairer, Norsett, Wanner, "Solving ODEs I", sec. II.1) on n steps. With the gradient
zeroed (constant gravity) delta = 0 and the reference is the closed form.

Reference error F_ref (each of p, v, a), reported in Ref and always the sum of:
  * truncation: the Richardson estimate of the delta solver, |y_2n - y_n| / (2^4 - 1) for the returned y_2n. n starts
    at N_START and doubles until that estimate is <= u Q (u = 2^-53, tools/sim/t4_rule.py);
  * roundoff: (formula roundings + n_steps) u Q with the counted roundings below and the summation scale Q
    (Higham 2002 sec. 4.2, tools/sim/t4_rule.floor).
Counted roundings (relative size <= u each, rounded up): the gravity evaluation is bounded by 80 u, the same rule as
the L1 gravity test (tests/regression/quad/L01/unit/prim/gravity_test.cpp, tolerance kOps = 40 times epsilon = 2u);
the closed form of hover has 30 more, of free fall 2; the delta right-hand side evaluates gravity once more (80 u).

Tick-held hover (item 7, last paragraph; the reference the T4 hover tests use). marv_plant evaluates the wrench from the
END-of-step motor state and holds it over the step, one plant call per tick (item 8), so over tick j (t_j = j h,
h = the tick period) the thrust is the constant 4 k w(t_{j+1})^2, w(t) = w_D (1 - exp(-t/tau)) (the exact zero-order-hold
lag of a motor started at rest). With a_j = 4 k w(t_{j+1})^2 / m the thrust part of the motion after n ticks is
  v_T = h sum_{j<n} a_j            p_T = h^2 sum_{j<n} a_j (n - j - 1/2)
(exact integrals of the piecewise-constant acceleration) and v_c = g0 t - v_T, p_c = g0 t^2 / 2 - p_T, t = n h. The
height-gradient correction delta is the same as above but driven by the piecewise-constant thrust: the right-hand side
needs p_c at the RK4 stage times inside a tick, from prefix sums S0(k) = sum_{j<k} a_j and S1(k) = sum_{j<k} j a_j
(Neumaier compensated) as p_T(k h + f) = h^2 ((k - 1/2) S0 - S1) + v_T(k) f + a_k f^2 / 2. delta's sensitivity to an
error in p_c is |dg/dh| t^2 / 2 ~ 1e-5, so the prefix sums' roundoff is far below F_ref; the returned p and v use the
exact boundary sums at t = n h (math.fsum, exact up to the rounding of the summed terms).
F_ref of the tick-held reference is the same construction as the continuous one, with the closed form's counted
roundings (relative size <= u each): w(t_{j+1}) 5 (the product j h and the division by tau 2, expm1 2, the product with
w_D 1; the argument's relative error moves 1 - e^-x by at most that relative amount), a_j 13 (two w, three more
roundings), the weight product 1, the fsum result 1, h^2 or h 2 or 1, t = n h 1, g0 t^2 / 2 3, the subtraction 1: 21 in
all for p, 18 for v (TICK_HELD_CLOSED_ROUNDINGS = 21 covers both), plus n_steps and the two gravity evaluations as above.
The time of a tick-held state is t = n h (n an integer tick count, h a float): the test passes the float of the exact
tick period, which is bitwise the plugin's tick_period_s (one correctly rounded division of two exact doubles).

Hover acceleration and settling. a_D(t) = g(phi, h0 - p_D) - T(t)/m. For t >> tau, a_D -> g - a_T + 2 a_T e^{-t/tau},
so the transient of a_D is below the floor F_a once 2 a_T e^{-t/tau} <= F_a, i.e. t_s = tau ln(2 a_T / F_a) (item 7).
F_a is the floor of the quantity a_D: F_a = floor(n_a, a_T, HOVER_ROUNDINGS) = (n_a + 100) u a_T. The 100 counted
roundings of the plant's formula: gravity 80, the thrust chain w_D (4), exp(-t/tau) and 1 - e (3), w (1), w^2 (1),
k w^2 and /m (2), rounded up to 19, and the final g - T/m (1). n_a = 1 (one evaluation) is the default and gives the
latest t_s; the accumulation over N ticks belongs to the floor of p and v (n = N there).

Bracket (item 7): a(D_lo) > 0 > a(D_hi) and |a| <= 4 k (w(D_hi)^2 - w(D_lo)^2) / m, the change in thrust
acceleration between the two adjacent commands.

Torque-free rotation: for a diagonal inertia I from the card, |L| = |I w| and KE = (1/2) w^T I w are conserved. They
are evaluated from logged body rates; the reference is their value at the first free step (the one after the initial
rates are applied), and the floor is (N + extra) u Q with N the number of steps, Q the value, and the counted
roundings of the formula: |L| = sqrt(sum (I_i w_i)^2) has 5 (product, square, two additions, square root), KE = 1/2
sum w_i (I_i w_i) has 4 (two products, two additions; the 1/2 is exact). The reference is a formula evaluation, so its
F_ref = extra u Q. The logged rates are taken as exact inputs.

Card values are read through tools/card/schema.py.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "card"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import hover  # noqa: E402
import schema  # noqa: E402
import t4_rule  # noqa: E402

U = t4_rule.U
N_ROTORS = 4  # quad-X: four identical rotors (card rotors.layout: quad_x); thrust is 4 k w^2 at equal speeds
RK4_ORDER = 4  # classical Runge-Kutta
N_START = 16  # solver parameter: first step count of the delta solver, doubled until the Richardson estimate <= u Q
N_MAX = 1 << 20  # solver parameter: refusal limit of the doubling
GRAVITY_ROUNDINGS = 80  # see the module docstring
FREE_FALL_ROUNDINGS = GRAVITY_ROUNDINGS + 2  # plant formula: m g, then / m
HOVER_ROUNDINGS = GRAVITY_ROUNDINGS + 20  # plant formula for a_D, see the module docstring
REF_FREE_FALL_ROUNDINGS = 2 + 2 * GRAVITY_ROUNDINGS  # closed form g0 t^2/2 (2), g0, and gravity in the delta rhs
REF_HOVER_ROUNDINGS = 30 + 2 * GRAVITY_ROUNDINGS
TICK_HELD_CLOSED_ROUNDINGS = 21  # see the module docstring (tick-held hover)
REF_TICK_HELD_ROUNDINGS = TICK_HELD_CLOSED_ROUNDINGS + 2 * GRAVITY_ROUNDINGS
TICK_OMEGA_ROUNDINGS = 5  # w(t_{j+1}): argument 2, expm1 2, product with w_D 1
TICK_ACCEL_ROUNDINGS = 3  # a_j from w: k w, (k w) w, / m (4 k is exact); with w's own error: 2 * 5 + 3 = 13 (relative)
ROTATION_L_ROUNDINGS = 5
ROTATION_KE_ROUNDINGS = 4
GRADIENT_STEP_M = 1.0  # labelled: central-difference step of gravity height gradient when gravity is not hover.normal_gravity


class ReferenceError(Exception):
    pass


@dataclass(frozen=True)
class Card:
    mass_kg: float
    thrust_coeff: float
    tau_s: float
    omega_min: float
    omega_max: float
    inertia_diag: tuple


def load_card(path):
    doc = schema.load_yaml(path)

    def val(entry, name):
        if entry["value"] == schema.UNKNOWN:
            raise ReferenceError(f"{path}: {name} is {schema.UNKNOWN}")
        return entry["value"]

    speed = val(doc["rotors"]["speed_range"], "rotors.speed_range")
    return Card(
        mass_kg=float(val(doc["mass"], "mass")),
        thrust_coeff=float(val(doc["rotors"]["thrust_coeff"], "rotors.thrust_coeff")),
        tau_s=float(val(doc["rotors"]["motor_lag"]["tau"], "rotors.motor_lag.tau")),
        omega_min=float(speed[0]),
        omega_max=float(speed[1]),
        inertia_diag=tuple(float(x) for x in val(doc["inertia_diag"], "inertia_diag")),
    )


def gravity_height_gradient(phi, h, gravity=hover.normal_gravity):
    """dg/dh in 1/s^2. Analytic for hover.normal_gravity (derivative of eq. (4-3) in h); central difference of the
    callable otherwise."""
    if gravity is hover.normal_gravity:
        c = hover._C
        s2 = math.sin(phi) ** 2
        inv_a = 1.0 / c["kWgs84A"]
        f = c["kWgs84F"]
        lin = 2.0 * inv_a * (1.0 + f + c["kWgs84M"] - 2.0 * f * s2)
        quad = c["kWgs84HeightQuadCoeff"] * inv_a * inv_a
        return hover.normal_gravity_ellipsoid(phi) * (-lin + 2.0 * quad * h)
    return (gravity(phi, h + GRADIENT_STEP_M) - gravity(phi, h - GRADIENT_STEP_M)) / (2.0 * GRADIENT_STEP_M)


def esc_omega(card, dshot):
    """Rotor speed command of the plant's ESC map (sim/plant/src/plant_model.hpp omega_cmd)."""
    if dshot == 0:
        return 0.0
    c = hover._C
    span = float(c["kDshotThrottleMax"] - c["kDshotThrottleMin"])
    frac = float(dshot - c["kDshotThrottleMin"]) / span
    return card.omega_min + (card.omega_max - card.omega_min) * frac


def hover_omega(card, dshot, t):
    return esc_omega(card, dshot) * (1.0 - math.exp(-t / card.tau_s))


def thrust_accel(card, dshot, t):
    """T(t)/m = 4 k w(t)^2 / m."""
    w = hover_omega(card, dshot, t)
    return N_ROTORS * card.thrust_coeff * w * w / card.mass_kg


def tick_held_omega(card, dshot, j, tick_s, hold=1):
    """w(t_{j+hold}) with t = (j + hold) h: the rotor speed the plant holds over tick j (hold = 1, end of the step, the
    frozen L1 ABI). hold = 0 is the start-of-tick value, which exists only as the negative control."""
    return esc_omega(card, dshot) * -math.expm1(-((j + hold) * tick_s) / card.tau_s)


def tick_held_thrust_accel(card, dshot, j, tick_s, hold=1):
    """a_j = 4 k w^2 / m over tick j."""
    w = tick_held_omega(card, dshot, j, tick_s, hold)
    return N_ROTORS * card.thrust_coeff * w * w / card.mass_kg


def hover_a_T(card, dshot):
    w = esc_omega(card, dshot)
    return N_ROTORS * card.thrust_coeff * w * w / card.mass_kg


@dataclass(frozen=True)
class Ref:
    t: float
    p: float
    v: float
    a: float
    F_p: float
    F_v: float
    F_a: float
    Q_p: float
    Q_v: float
    Q_a: float
    n_steps: int
    trunc_p: float
    trunc_v: float


def _rk4(rhs, t_end, n):
    h = t_end / n
    d = w = 0.0
    for i in range(n):
        t = i * h
        k1d, k1w = w, rhs(t, d)
        k2d, k2w = w + 0.5 * h * k1w, rhs(t + 0.5 * h, d + 0.5 * h * k1d)
        k3d, k3w = w + 0.5 * h * k2w, rhs(t + 0.5 * h, d + 0.5 * h * k2d)
        k4d, k4w = w + h * k3w, rhs(t + h, d + h * k3d)
        d += h * (k1d + 2.0 * k2d + 2.0 * k3d + k4d) / 6.0
        w += h * (k1w + 2.0 * k2w + 2.0 * k3w + k4w) / 6.0
    return d, w


def _delta(rhs, t_end, q_p, q_v):
    """(delta, delta', n_steps, trunc_delta, trunc_delta') with the Richardson estimates <= u Q_p and u Q_v: one
    rounding of the summation scale (the right-hand side's own roundoff, 80 u g, keeps a stricter target unreachable)."""
    n = N_START
    d1, w1 = _rk4(rhs, t_end, n)
    scale = 2 ** RK4_ORDER - 1
    while n < N_MAX:
        n *= 2
        d2, w2 = _rk4(rhs, t_end, n)
        td, tw = abs(d2 - d1) / scale, abs(w2 - w1) / scale
        if td <= U * q_p and tw <= U * q_v:
            return d2, w2, n, td, tw
        d1, w1 = d2, w2
    raise ReferenceError("the delta solver did not reach a Richardson estimate <= u Q")


def _solve(t, phi, h0, gravity, p_c, v_c, q_p, q_v, q_a_of, closed_rounds, a_of, extra_a):
    g0 = gravity(phi, h0)
    if t == 0.0:
        a = a_of(0.0, 0.0)
        return Ref(0.0, 0.0, 0.0, a, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0.0, 0.0)

    def rhs(tt, d):
        return gravity(phi, h0 - p_c(tt) - d) - g0

    d, w, n, td, tw = _delta(rhs, t, q_p, q_v)
    p = p_c(t) + d
    v = v_c(t) + w
    a = a_of(t, p)
    F_p = td + (closed_rounds + n) * U * max(q_p, abs(d))
    F_v = tw + (closed_rounds + n) * U * max(q_v, abs(w))
    grad = abs(gravity_height_gradient(phi, h0 - p, gravity))
    q_a = q_a_of(t)
    F_a = grad * F_p + extra_a * U * q_a
    return Ref(t, p, v, a, F_p, F_v, F_a, q_p, q_v, q_a, n, td, tw)


def free_fall(t, phi, h0, gravity=hover.normal_gravity):
    """Motors off, from rest at the NED origin. p_D(t), v_D(t), a_D(t) = g(phi, h0 - p_D)."""
    g0 = gravity(phi, h0)
    p_c = g0 * t * t / 2.0
    v_c = g0 * t
    return _solve(
        t, phi, h0, gravity, lambda tt: g0 * tt * tt / 2.0, lambda tt: g0 * tt, abs(p_c), abs(v_c),
        lambda tt: g0, REF_FREE_FALL_ROUNDINGS, lambda tt, p: gravity(phi, h0 - p), GRAVITY_ROUNDINGS,
    )


def hover_state(t, card, dshot, phi, h0, gravity=hover.normal_gravity):
    """Hover at DShot `dshot`, from rest with the rotors at rest (item 7). p_D, v_D, a_D at time t. The summation
    scales are Q_p = (g0 + a_T) t^2 / 2, Q_v = (g0 + a_T) t (sum of |terms| of g and T/m) and Q_a = g0 + a_T."""
    g0 = gravity(phi, h0)
    a_T = hover_a_T(card, dshot)
    tau = card.tau_s

    def v_c(tt):
        e = math.exp(-tt / tau)
        return g0 * tt - a_T * (tt - 2.0 * tau * (1.0 - e) + (tau / 2.0) * (1.0 - e * e))

    def p_c(tt):
        e = math.exp(-tt / tau)
        return g0 * tt * tt / 2.0 - a_T * (
            tt * tt / 2.0 - 2.0 * tau * (tt - tau * (1.0 - e)) + (tau / 2.0) * (tt - (tau / 2.0) * (1.0 - e * e))
        )

    def a_of(tt, p):
        return gravity(phi, h0 - p) - thrust_accel(card, dshot, tt)

    q_scale = g0 + a_T
    return _solve(
        t, phi, h0, gravity, p_c, v_c, q_scale * t * t / 2.0, q_scale * t, lambda tt: q_scale, REF_HOVER_ROUNDINGS,
        a_of, HOVER_ROUNDINGS,
    )


def _neumaier(values):
    """Prefix sums of `values` by Neumaier's compensated summation: out[k] = sum of values[:k] (out[0] = 0.0)."""
    out = [0.0]
    s = c = 0.0
    for x in values:
        t = s + x
        c += (s - t) + x if abs(s) >= abs(x) else (x - t) + s
        s = t
        out.append(s + c)
    return out


def hover_state_tick_held(n_ticks, tick_s, card, dshot, phi, h0, gravity=hover.normal_gravity, hold=1):
    """Tick-held hover (module docstring): p_D, v_D at t = n_ticks h and a_D, the acceleration over the tick that starts
    at t (its thrust is w(t + h)). `hold` is as in tick_held_omega. Q_p, Q_v, Q_a as for hover_state."""
    n = int(n_ticks)
    if n != n_ticks or n < 0:
        raise ReferenceError("n_ticks must be a non-negative integer")
    g0 = gravity(phi, h0)
    a_T = hover_a_T(card, dshot)
    h = tick_s
    t_end = n * h
    accel = [tick_held_thrust_accel(card, dshot, j, h, hold) for j in range(n + 1)]
    q_scale = g0 + a_T
    if n == 0:
        return _solve(0.0, phi, h0, gravity, None, None, 0.0, 0.0, lambda tt: q_scale, REF_TICK_HELD_ROUNDINGS,
                      lambda tt, p: gravity(phi, h0 - p) - accel[0], HOVER_ROUNDINGS)
    s0 = _neumaier(accel[:n])
    s1 = _neumaier([j * accel[j] for j in range(n)])
    v_t_end = h * math.fsum(accel[:n])
    p_t_end = h * h * math.fsum(accel[j] * (n - j - 0.5) for j in range(n))

    def thrust_part(tt):
        if tt == t_end:
            return p_t_end, v_t_end
        k = min(int(tt / h), n - 1)
        f = tt - k * h
        v_k = h * s0[k]
        p_k = h * h * ((k - 0.5) * s0[k] - s1[k])
        return p_k + v_k * f + accel[k] * f * f / 2.0, v_k + accel[k] * f

    def p_c(tt):
        return g0 * tt * tt / 2.0 - thrust_part(tt)[0]

    def v_c(tt):
        return g0 * tt - thrust_part(tt)[1]

    return _solve(
        t_end, phi, h0, gravity, p_c, v_c, q_scale * t_end * t_end / 2.0, q_scale * t_end, lambda tt: q_scale,
        REF_TICK_HELD_ROUNDINGS, lambda tt, p: gravity(phi, h0 - p) - accel[n], HOVER_ROUNDINGS,
    )


def hover_settling(card, dshot, n_a=1):
    """(t_s, F_a, a_T) with F_a = floor(n_a, a_T, HOVER_ROUNDINGS) and t_s = tau ln(2 a_T / F_a) (item 7)."""
    a_T = hover_a_T(card, dshot)
    f_a = t4_rule.floor(n_a, a_T, HOVER_ROUNDINGS)
    return card.tau_s * math.log(2.0 * a_T / f_a), f_a, a_T


def bracket_sign_ok(a_lo, a_hi):
    """a(D_lo) > 0 > a(D_hi)."""
    return a_lo > 0.0 > a_hi


def bracket_bound(card, d_lo, d_hi):
    """4 k (w(D_hi)^2 - w(D_lo)^2) / m, with w the ESC map at the two commands (the settled rotor speeds)."""
    w_lo, w_hi = esc_omega(card, d_lo), esc_omega(card, d_hi)
    return N_ROTORS * card.thrust_coeff * (w_hi * w_hi - w_lo * w_lo) / card.mass_kg


def bracket_magnitude_ok(a, card, d_lo, d_hi):
    return abs(a) <= bracket_bound(card, d_lo, d_hi)


def rotation_invariants(rates, inertia_diag):
    """(|L|, KE) of one body-rate sample (wx, wy, wz) for a diagonal inertia, with the roundings counted above."""
    l = [i * w for i, w in zip(inertia_diag, rates)]
    l_norm = math.sqrt(l[0] * l[0] + l[1] * l[1] + l[2] * l[2])
    ke = 0.5 * (rates[0] * l[0] + rates[1] * l[1] + rates[2] * l[2])
    return l_norm, ke


@dataclass(frozen=True)
class RotationRef:
    L: float
    KE: float
    F_ref_L: float
    F_ref_KE: float


def rotation_reference(rates_first_free, inertia_diag):
    """The reference is the value at the first free step; F_ref is the formula's own roundings."""
    l_norm, ke = rotation_invariants(rates_first_free, inertia_diag)
    return RotationRef(l_norm, ke, t4_rule.floor(0, l_norm, ROTATION_L_ROUNDINGS),
                       t4_rule.floor(0, ke, ROTATION_KE_ROUNDINGS))


def rotation_floors(n_steps, ref):
    """(F_L, F_KE) = (N + extra) u Q for N logged steps."""
    return (t4_rule.floor(n_steps, ref.L, ROTATION_L_ROUNDINGS), t4_rule.floor(n_steps, ref.KE, ROTATION_KE_ROUNDINGS))
