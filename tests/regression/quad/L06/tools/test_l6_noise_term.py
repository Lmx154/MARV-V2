"""The stage (e) noise term W (tools/sim/l6_noise_term.py; decision 0019, the noise suite design memo of 2026-10-04,
section 5, commit C4), per push.

  (1) the T4e linear model (axis_run without noise) equals l6_ff_eval.linear_response with l6_ff_eval's T4e linear
      sensor bit for bit, on the acro script, every axis, at the card member and the four corners of the tau x J box;
      control: the same model with latency 0 differs;
  (2) the variance sums of the formula (member_sums: the P class runs, the seed run and the step recursion) equal the
      direct computation over executions 0 .. WINDOW: one run per injection tick (the seed tick 0 included),
      Var_white(n) = sum_i g(n, i)^2 and Var_rw(n) = sum_i H(n, i)^2 with H(n, i) = sum_(j >= i) g(n, j), summed
      with math.fsum. Every per-tick run is a class run shifted by a multiple of P ticks, bit for bit (asserted: the
      state is exactly zero before the sample), so the two sides differ by summation only, and each point passes within
      the first-order bound of direct_sums (stated there). sigma^2 is noise.white_var Var_white + noise.rw_var Var_rw.
        - decoupled: the card member, roll, w0 = 0; controls: the two dt phases swapped (classes c and c + P/2) and the
          seed path dropped each break the bound;
        - coupled, the feed-forward linearised: a box corner, the three input axes summed, w0 the sign pattern
          COUPLED_SIGNS at rate_max (rate_lead's noise rule operating point, rule step 6); controls: the formula at
          w0 = 0 and the dt phases swapped each break the bound;
  (3) controls on step_roll's window at the card member: sigma_d x 1.1 (the spec's noise-density control factor, quad
      spec 4 L6 (a)) and latency + 1 each increase W;
  (4) z reproduces the memo's values (3.506 / 3.761 at M_s = 1, 5.893 / 6.054 at M_s = 120000, tracking / safety) from
      the register's noise_term_confidence and N, to the half unit of the memo's third decimal.
"""

import dataclasses
import math
import sys
from pathlib import Path
from unittest import mock

import pytest

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "sim"))
import l6_noise_term as nt  # noqa: E402
import t4_rule  # noqa: E402

fe = nt.fe

MEMO_Z = {(nt.CLASS_TRACKING, 1): 3.506, (nt.CLASS_SAFETY, 1): 3.761, (nt.CLASS_TRACKING, 120000): 5.893,
          (nt.CLASS_SAFETY, 120000): 6.054}  # the memo's section 1 and the C4 packet
MEMO_HALF_UNIT = 0.0005  # the memo prints z to three decimals
SIGMA_D_FACTOR = 1.1  # quad spec 4 L6 pass bar (a): the noise density x 1.1 negative control
CORNERS = ((0.0, 0.0), (-1.0, -1.0), (-1.0, 1.0), (1.0, -1.0), (1.0, 1.0))  # the card member and the box corners
WINDOW = 60  # labelled test value: executions 0..60, every phase class many times and the seed path's start, per push
COUPLED_MEMBER = (-1.0, 1.0)  # a box corner (J (1 - b_J), tau (1 + b_tau))
COUPLED_SIGNS = (1, -1, 1)  # one of rate_lead.SIGN_PATTERNS, mixed signs
INPUT_AXES = 3  # the three summed per-axis contributions of the coupled formula


def _say(capsys, *lines):
    with capsys.disabled():
        print()
        for line in lines:
            print(line)


@pytest.fixture(scope="module")
def cfg(rate_lead_design):
    with mock.patch.object(fe.rl, "design", return_value=rate_lead_design):
        return fe.setup("none")


@pytest.fixture(scope="module")
def reg():
    return nt.register()


# ---- (1) the T4e linear model ---------------------------------------------------------------------------------------

def _ff_eval_response(c, a, s, t, sensor):
    """l6_ff_eval's T4e linear row member (its _row_task): linear_response on the acro script."""
    law = c["laws"]["PID"]
    jt, tt = nt.member_plant(c, s, t, a)
    maps = (c["su"].plant_map(jt, tt), fe.tick_map(c["su1"], jt, tt))
    return fe.linear_response(law.kp[a], law.ki[a], law.kd[a], law.tau_ref[a], law.d_tau[a], sensor, maps, c["sps"][a],
                              c["stamps"], c["plan"].divisor)


def _model_response(c, a, s, t, sensor):
    jt, tt = nt.member_plant(c, s, t, a)
    return nt.axis_run(nt.law_of(c, a), sensor, nt.tick_map(c, jt, tt), c["stamps"], c["plan"].divisor,
                       len(c["sps"][a]), sps=c["sps"][a])


def test_t4e_model_equals_linear_response_bit_for_bit(cfg):
    c = cfg
    t4e = c["sensors"]["T4e"].linear
    assert nt.sensor_of(c) is t4e
    assert nt.stamps_of(c, len(c["stamps"])) == c["stamps"]
    for s, t in CORNERS:
        for a in range(3):
            assert _model_response(c, a, s, t, t4e) == _ff_eval_response(c, a, s, t, t4e), (s, t, a)


def test_t4e_model_control_latency_0_differs(cfg):
    c = cfg
    t4e = c["sensors"]["T4e"].linear
    assert t4e.latency > 0
    no_latency = dataclasses.replace(t4e, latency=0)
    for a in range(3):
        assert _model_response(c, a, 0.0, 0.0, no_latency) != _ff_eval_response(c, a, 0.0, 0.0, t4e), a


# ---- (2) the formula against direct injection -------------------------------------------------------------------------

def _runner(c, s, t, op, n_exec):
    """run(at, b) -> {output axis: w per execution} of member (s, t) at op with one unit sample on axis b at tick at,
    from the module's own pieces (as member_sums sets them up)."""
    divisor = c["fixture"]["rate_loop_divisor"]
    stamps = nt.stamps_of(c, n_exec)
    tmaps = [nt.tick_map(c, *nt.member_plant(c, s, t, a)) for a in range(3)]
    sensor = nt.sensor_of(c)
    if not op.coupled:
        return lambda at, b: {b: nt.axis_run(nt.law_of(c, b), sensor, tmaps[b], stamps, divisor, n_exec, inject=at)}
    ffl = c["laws"][nt.FF_LAW]
    ff = (nt.rl.gyroscopic_jacobian(list(op.omega0), list(ffl.inertia)), ffl.motor_tau, ffl.ff_tau)
    laws = [nt.law_of(c, a, nt.FF_LAW) for a in range(3)]
    return lambda at, b: dict(enumerate(nt.coupled_run(laws, ff, sensor, tmaps, stamps, divisor, n_exec, b, at)))


def direct_sums(resp, points, divisor):
    """[(Var_white, Var_rw, bound_white, bound_rw)] per point from resp[i] = w per execution of the run with one unit
    sample at tick i. Bound of |formula - direct|, first order (Higham 2002 sec. 4.2 through t4_rule.floor, N u Q):
    the formula sums at most K = D n + 1 terms recursively, (K + 1) u S with the squaring's rounding, and its step sums
    H by recursion, K u A_i with A_i = sum_(j >= i) |g(n, j)|, which enter H^2 as 2 |H_i| K u A_i; math.fsum is
    correctly rounded, so the direct side adds u |H_i| to each H_i and 2 u S to each sum of squares."""
    u = t4_rule.U
    out = []
    for n in points:
        g = [resp[i][n] for i in range(divisor * n + 1)]
        k = len(g)
        s_w = math.fsum(x * x for x in g)
        h = [math.fsum(g[i:]) for i in range(k)]
        a_abs = [math.fsum(abs(x) for x in g[i:]) for i in range(k)]
        s_r = math.fsum(x * x for x in h)
        b_w = t4_rule.floor(k, s_w, 1) + 2 * u * s_w
        b_r = (math.fsum(2 * abs(hi) * (t4_rule.floor(k, ai) + u * abs(hi)) for hi, ai in zip(h, a_abs))
               + t4_rule.floor(k, s_r, 1) + 2 * u * s_r)
        out.append((s_w, s_r, b_w, b_r))
    return out


def _ratio(diff, bound):
    """|diff| / bound; a zero bound (both sums exactly 0) admits only diff = 0."""
    return abs(diff) / bound if bound > 0 else (0.0 if diff == 0 else math.inf)


def _within(vw, vr, direct):
    """Per point (|dVar_white| / bound, |dVar_rw| / bound); the check passes iff every ratio is <= 1."""
    return [(_ratio(x - d[0], d[2]), _ratio(y - d[1], d[3])) for x, y, d in zip(vw, vr, direct)]


def _worst(ratios):
    return max(max(p) for p in ratios)


def _class_runs_are_shifts(resp, classes, period, divisor):
    """Every per-tick run i >= 1 equals the class run of i mod P (injected at tick P + i mod P), shifted, bit for bit."""
    for i in range(1, len(resp)):
        cls = i % period
        shift = (period + cls - i) // divisor
        run, ref = resp[i], classes[cls]
        if any(run[n] != ref[n + shift] for n in range(len(run)) if 0 <= n + shift < len(ref)):
            return False
    return True


def _phase_swapped(classes, period, divisor):
    """The class runs with the other dt phase at the same tick position: class c gets the run of class c + P/2, moved
    in time so that its sample sits at tick P + c (a control: the dt phase bookkeeping alone)."""
    out = []
    for cls in range(period):
        src = (cls + period // 2) % period
        delta = (cls - src) // divisor
        run = classes[src]
        out.append([run[n - delta] if 0 <= n - delta < len(run) else 0.0 for n in range(len(run))])
    return out


def _setup_direct(c, s, t, op, axes_in):
    divisor = c["fixture"]["rate_loop_divisor"]
    period = nt.period_ticks(c)
    points = tuple(range(WINDOW + 1))
    n_run = (2 * period - 1 + divisor * WINDOW) // divisor + 1
    run = _runner(c, s, t, op, n_run)
    resp = {b: [run(i, b) for i in range(divisor * WINDOW + 1)] for b in axes_in}
    classes = {b: [run(period + k, b) for k in range(period)] for b in axes_in}
    return divisor, period, points, resp, classes


def test_formula_equals_direct_injection(cfg, capsys):
    c = cfg
    a = 0
    divisor, period, points, resp, classes = _setup_direct(c, 0.0, 0.0, nt.Op(), (a,))
    ra = [x[a] for x in resp[a]]
    ca = [x[a] for x in classes[a]]
    assert _class_runs_are_shifts(ra, ca, period, divisor)
    direct = direct_sums(ra, points, divisor)
    vw, vr = nt.member_sums(c, 0.0, 0.0, nt.Op(), points, axes=(a,))[a]
    ok = _worst(_within(vw, vr, direct))
    swapped = nt.unit_sums(ra[0], _phase_swapped(ca, period, divisor), period, period, divisor, points)
    no_seed = nt.unit_sums([0.0] * len(ra[0]), ca, period, period, divisor, points)
    ctl_swap, ctl_seed = _worst(_within(*swapped, direct)), _worst(_within(*no_seed, direct))
    _say(capsys, f"direct injection, card member, roll, executions 0..{WINDOW}: largest |formula - direct| / bound "
                 f"{ok:.3e}; controls: dt phases swapped {ctl_swap:.3e}, seed path dropped {ctl_seed:.3e}")
    assert ok <= 1
    assert ctl_swap > 1
    assert ctl_seed > 1


def test_coupled_formula_equals_direct_injection(cfg, capsys):
    c = cfg
    s, t = COUPLED_MEMBER
    p = c["fixture"]
    op = nt.Op(tuple(sg * p[f"rate_max_{ax}"] for sg, ax in zip(COUPLED_SIGNS, nt.AXES)))
    assert COUPLED_SIGNS in nt.rl.SIGN_PATTERNS
    divisor, period, points, resp, classes = _setup_direct(c, s, t, op, range(3))
    formula = nt.member_sums(c, s, t, op, points)
    lti = nt.member_sums(c, s, t, nt.Op(), points)
    worst = ctl_lti = ctl_swap = 0.0
    for a in range(3):
        parts = []
        sw_vw, sw_vr = [0.0] * len(points), [0.0] * len(points)
        for b in range(3):
            rb = [x[a] for x in resp[b]]
            cb = [x[a] for x in classes[b]]
            assert _class_runs_are_shifts(rb, cb, period, divisor), (a, b)
            parts.append(direct_sums(rb, points, divisor))
            vw, vr = nt.unit_sums(rb[0], _phase_swapped(cb, period, divisor), period, period, divisor, points)
            sw_vw, sw_vr = [x + y for x, y in zip(sw_vw, vw)], [x + y for x, y in zip(sw_vr, vr)]
        direct = []
        for k in range(len(points)):
            tw, tr = math.fsum(q[k][0] for q in parts), math.fsum(q[k][1] for q in parts)
            # the per-axis bounds, plus the formula's two additions over the input axes and fsum's rounding
            direct.append((tw, tr, math.fsum(q[k][2] for q in parts) + t4_rule.floor(INPUT_AXES, tw, 1),
                           math.fsum(q[k][3] for q in parts) + t4_rule.floor(INPUT_AXES, tr, 1)))
        worst = max(worst, _worst(_within(*formula[a], direct)))
        ctl_lti = max(ctl_lti, _worst(_within(*lti[a], direct)))
        ctl_swap = max(ctl_swap, _worst(_within(sw_vw, sw_vr, direct)))
    _say(capsys, f"direct injection, coupled (member {COUPLED_MEMBER}, {op.label()}), three input axes, executions "
                 f"0..{WINDOW}: largest |formula - direct| / bound {worst:.3e}; controls: formula at w0 = 0 {ctl_lti:.3e}, "
                 f"dt phases swapped {ctl_swap:.3e}")
    assert worst <= 1
    assert ctl_lti > 1
    assert ctl_swap > 1


# ---- (3) the controls ---------------------------------------------------------------------------------------------

def test_controls_sigma_d_and_latency_increase_w(cfg, reg, capsys):
    c = cfg
    case, _ = nt.l4_case(c, "step_roll")
    case = dataclasses.replace(case, axes=(0,))
    noise = nt.noise_inputs(c)
    z = nt.z_value(reg["noise_term_confidence"]["value"], nt.seed_count(case.klass, reg), len(case.points))
    w0 = nt.noise_term(nt.sigma(c, case, noise, grid=1), z)[0]
    louder = dataclasses.replace(noise, sigma_d=noise.sigma_d * SIGMA_D_FACTOR)
    w_sigma = nt.noise_term(nt.sigma(c, case, louder, grid=1), z)[0]
    t4e = nt.sensor_of(c)
    later = dataclasses.replace(t4e, latency=t4e.latency + 1)
    w_lat = nt.noise_term(nt.sigma(c, case, noise, grid=1, sensor=later), z)[0]
    _say(capsys, f"step_roll, card member, roll: W {w0:.9e} rad/s; sigma_d x {SIGMA_D_FACTOR}: {w_sigma:.9e} "
                 f"(x {w_sigma / w0:.6f}); latency + 1: {w_lat:.9e} (x {w_lat / w0:.9f})")
    assert w_sigma > w0
    assert w_lat > w0


# ---- (4) z --------------------------------------------------------------------------------------------------------

def test_z_reproduces_the_memo(reg):
    conf = reg["noise_term_confidence"]["value"]
    for (klass, m), want in MEMO_Z.items():
        z = nt.z_value(conf, nt.seed_count(klass, reg), m)
        assert abs(z - want) <= MEMO_HALF_UNIT, (klass, m, z)
