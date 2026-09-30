"""L5 T3 rule properties of the attitude loop on the LIVE generated parameters (decision 0006 F, "T3 margins"; owner
decision 7: rules, never pinned gain values; every metric check has a planted-violation negative control, core 7.2).

Covered here, on the product path (flatten.py with the committed measured SIM-7 U, design/measured/sim7_u/u.yaml; the
attitude design is also evaluated in process and must equal flatten's emitted values):
  1. the 17 x 17 tau x J box grid (corners included), PM_worst >= PM_min - delta_num at the live f32 gains and N*, with the
     9 x 9 -> 17 x 17 halving change recorded (core 7.5);
  2. t_cross on the 17 x 17 grid: every member's first-crossing time <= att_yaw_t_cross;
  3. the yaw-lock fallback property (F, T4 yaw-lock fallback): with the constant disturbance d = sigma_r tau_held,yaw every
     box member keeps sigma_r psi_dot > 0 up to its fallback execution (the held stick halves until none crosses, F);
  4. one added tick of delay in the lifted design model fails the margin check.
Already covered by test_attitude_design.py (not repeated): PM_worst at nominal and the four corners on all three axes, the
yaw compensation, tightness (next f32 k above the rule), the Jury/spectral radius of the live loops, w, att_yaw_alpha_min,
att_yaw_t_cross as the corner rule and a 5 x 5 grid, the SIM-7 table and att_loop_ratio, k x 1.1 at the corners.
Not in P7b (T3 envelopes, T3 step, T4): not written here.

Helpers imported from test_attitude_design.py: the independent oracle (sampled plant by matrix exponential, closed-rate-loop
lifting, complex solve, unwrapped-phase PM) and its t_c stepping. The six-state tick-delay model below is this file's own
(the oracle's are fixed at five states); its zero-delay case is asserted equal to the oracle.
"""

import cmath
import math
import struct
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_attitude_design import (  # noqa: E402  (module-level helpers of the sibling test file)
    AXES, BUDGET, CARD, EPS32, EPS64, FLATTEN, ROOT, SCENARIO, TOL, closed_step, corners, crossing_time, csolve,
    f32_up, inertia, k_yaw_effective, lifted, live, matmul, oracle_live_worst, phase_margin, r32, rate_gains, run,
    sampled_plant, t_cross_ok)

sys.path.insert(0, str(ROOT / "tools" / "card"))
sys.path.insert(0, str(ROOT / "tools" / "sim"))
import attitude  # noqa: E402
import run_l4  # noqa: E402
import schema  # noqa: E402

MEASURED_U = ROOT / "design" / "measured" / "sim7_u" / "u.yaml"
HOVER_SCENARIO = ROOT / "scenarios" / "quad" / "L04" / "chirp_yaw.yaml"  # the L4 chirp's site (hover thrust m g(phi, h0))
FINE, COARSE = 17, 9  # grid points per side; the last halving is 9 -> 17 (core 7.5)
MAX_STICK_HALVINGS = 64  # method constant: termination of F's held-stick halving only
MAX_DELAY_SCALE = 40  # method constant: termination of the planted-disturbance search of the halving control
SIGMA_R = 1.0  # the release sign; the linear model is odd in (stick, d), so sigma_r = -1 is the mirror image
GAIN_CONTROL = 1.1  # the T3 gain control of decision 0005/0006 F
HALVING_NOTE = ("grid refined by halving 9x9 -> 17x17 (core 7.5), last-halving change recorded: {change!r}")


# ---- the live parameters ----------------------------------------------------------------------------------------------


@pytest.fixture(scope="module")
def real(tmp_path_factory):
    tmp = tmp_path_factory.mktemp("attitude_t3")
    r = run(FLATTEN, "--card", CARD, "--budget", BUDGET, "--out-card", tmp / "card.yaml", "--out-register",
            tmp / "register.yaml", "--out-mixer", tmp / "mixer.yaml", "--root", ROOT, "--scenario", SCENARIO,
            "--out-scenario", tmp / "scenario.yaml", "--out-rate", tmp / "rate.yaml", "--out-attitude",
            tmp / "attitude.yaml", "--sim7-u", MEASURED_U)
    assert r.returncode == 0, r.stderr
    card, budget, scenario = (schema.load_yaml(p) for p in (CARD, BUDGET, SCENARIO))
    u = attitude.read_u(MEASURED_U)
    entries, _, res = attitude.attitude_entries(card, budget, scenario, CARD, u, "design/measured/sim7_u/u.yaml")
    emitted = schema.load_yaml(tmp / "attitude.yaml")
    for name, entry in entries:
        assert emitted[name]["value"] == entry["value"], f"{name}: the in-process design is not flatten's live value"
    return {"res": res, "flat": dict(entries), "rate_flat": schema.load_yaml(tmp / "rate.yaml"), "dir": tmp,
            "card": card, "budget": budget, "u": u["value"], "oracle": {}, "q": {},
            "scenario": schema.load_yaml(tmp / "scenario.yaml"), "mixer": schema.load_yaml(tmp / "mixer.yaml"),
            "cardyaml": schema.load_yaml(tmp / "card.yaml")}


def box_members(real, m):
    """[(a, c, jt, tau)] of the m x m grid of the tau x J box, corners included: jt the true-over-design inertia."""
    i = real["res"]["inputs"]
    return [(a, c, 1 + i["b_J"] * (2 * a / (m - 1) - 1), i["tau"] * (1 + i["b_tau"] * (2 * c / (m - 1) - 1)))
            for a in range(m) for c in range(m)]


def live_gains(real, scale=1.0):
    k32, w32 = live(real, "att_kp") * scale, live(real, "att_yaw_weight")
    return k32, k_yaw_effective(k32, w32)


# ---- 1. the 17 x 17 margin grid -------------------------------------------------------------------------------------------


@pytest.fixture(scope="module")
def pm_grid(real):
    """{scale: {(axis, a, c): PM or -inf}} on the FINE grid at the live gains (scale 1) and at k x GAIN_CONTROL."""
    n, T = live(real, "att_loop_ratio"), real["res"]["T"]
    out = {}
    for scale in (1.0, GAIN_CONTROL):
        k, k_yaw = live_gains(real, scale)
        table = {}
        for axis in AXES:
            kp, ki = rate_gains(real, axis)
            for a, c, jt, tau in box_members(real, FINE):
                p, bs = lifted(inertia(real, axis) * jt, tau, kp, ki, T, n)
                m = phase_margin(p, bs, k_yaw if axis == "yaw" else k, n * T)
                table[(axis, a, c)] = -math.inf if m is None else m[0]
        out[scale] = table
    return out


def grid_min(table, step=1):
    """(PM_min over the members whose a and c are multiples of step, the argmin key); step 2 is the COARSE sub-grid."""
    return min(((pm, key) for key, pm in table.items() if key[1] % step == 0 and key[2] % step == 0))


def grid_ok(real, table):
    """The T3 rule: every member's PM >= PM_min - delta_num."""
    i, f = real["res"]["inputs"], real["res"]["final"]
    return min(table.values()) >= i["PM_min"] - f["delta_num"]


def test_the_17x17_box_grid_keeps_pm_worst_at_or_above_pm_min_minus_delta_num(real, pm_grid):
    table = pm_grid[1.0]
    i, f = real["res"]["inputs"], real["res"]["final"]
    assert len(table) == len(AXES) * FINE * FINE
    fine, at = grid_min(table)
    coarse, _ = grid_min(table, step=(FINE - 1) // (COARSE - 1))
    change = fine - coarse
    side = lambda x, names: names[0] if x == 0 else names[2] if x == FINE - 1 else names[1]  # noqa: E731
    where = f"{at[0]}, {side(at[1], ('J-', 'J', 'J+'))}, {side(at[2], ('tau-', 'tau', 'tau+'))}"
    msg = (f"grid minimum {math.degrees(fine)!r} deg at {where} (members a={at[1]}, c={at[2]}); PM_min "
           f"{math.degrees(i['PM_min'])!r} deg, delta_num {f['delta_num']!r} rad; "
           + HALVING_NOTE.format(change=change) + f" rad (9x9 minimum {math.degrees(coarse)!r} deg)")
    print(msg)
    assert fine <= coarse, "the 9x9 members are a subset of the 17x17 ones, so the finer minimum cannot be higher"
    assert fine >= i["PM_min"] - f["delta_num"], msg
    assert grid_ok(real, table), msg
    assert abs(change) <= f["delta_num"] + TOL, "the last halving changes the minimum by more than delta_num: " + msg
    # the grid includes the corners, so its minimum is at or below the corner minimum of the generator's own result
    assert fine <= f["pm_worst"] + TOL
    _, detail = oracle_live_worst(real)
    assert fine <= min(d[2] for d in detail) + TOL


def test_control_a_planted_gain_and_a_planted_pm_entry_each_fail_the_grid_check(real, pm_grid):
    i, f = real["res"]["inputs"], real["res"]["final"]
    assert grid_ok(real, pm_grid[1.0])
    planted = pm_grid[GAIN_CONTROL]
    assert not grid_ok(real, planted), "k x 1.1 must leave the 17x17 margin rule"
    fine, _ = grid_min(planted)
    assert fine < i["PM_min"] - f["delta_num"]
    table = dict(pm_grid[1.0])
    key = next(iter(table))
    table[key] = i["PM_min"] - 2 * f["delta_num"] - TOL
    assert not grid_ok(real, table), "a PM table entry below PM_min - delta_num must be rejected"
    table[key] = -math.inf
    assert not grid_ok(real, table), "a member without a unique crossover (or unstable) must be rejected"


# ---- 2. t_cross on the 17 x 17 grid ---------------------------------------------------------------------------------------


def f32_down(x):
    return struct.unpack("<f", struct.pack("<I", struct.unpack("<I", struct.pack("<f", x))[0] - 1))[0]


@pytest.fixture(scope="module")
def tc_grid(real):
    n, j_a = live(real, "att_loop_ratio"), inertia(real, "yaw")
    return {(a, c): crossing_time(real, j_a * jt, tau, n) for a, c, jt, tau in box_members(real, FINE)}


def test_every_17x17_member_crosses_zero_no_later_than_att_yaw_t_cross(real, tc_grid):
    t = live(real, "att_yaw_t_cross")
    j_a, n = inertia(real, "yaw"), live(real, "att_loop_ratio")
    gmax = max(tc_grid.values())
    gmax_coarse = max(v for (a, c), v in tc_grid.items() if a % 2 == 0 and c % 2 == 0)
    at = max(tc_grid, key=tc_grid.get)
    msg = (f"t_cross {t!r} s, 17x17 grid maximum {gmax!r} s at members {at}; "
           + HALVING_NOTE.format(change=gmax - gmax_coarse) + f" s (9x9 maximum {gmax_coarse!r} s)")
    print(msg)
    assert len(tc_grid) == FINE * FINE and gmax >= gmax_coarse
    assert all(t >= v * (1 - 4 * EPS64) for v in tc_grid.values()), msg
    assert t_cross_ok(real, t, [(j_a * jt, tau) for _, _, jt, tau in box_members(real, FINE)]), msg
    assert t - gmax < EPS32 * 2 * gmax, "t_cross is the grid maximum rounded up by at most one f32 step: " + msg
    assert gmax - gmax_coarse <= t - gmax + EPS32 * gmax, "the halving changes the maximum by more than the f32 step: " + msg
    assert n * real["res"]["T"] <= gmax


def test_control_t_cross_one_f32_step_below_the_grid_maximum_fails_the_rule(real, tc_grid):
    t = live(real, "att_yaw_t_cross")
    j_a = inertia(real, "yaw")
    points = [(j_a * jt, tau) for _, _, jt, tau in box_members(real, FINE)]
    assert t_cross_ok(real, t, points)
    below = f32_down(t)
    assert below < max(tc_grid.values()), "one f32 step below the rounded-up value is below the grid maximum"
    assert not t_cross_ok(real, below, points)
    assert t_cross_ok(real, f32_up(t), points), "one step above still bounds the grid, so the check is not vacuous"


# ---- 3. the yaw-lock fallback property --------------------------------------------------------------------------------------


def tau_held_yaw(real):
    """tau_held,yaw of 0005's collective-held envelope rule, as the L4 chirp computes it (run_l4.chirp_design): the largest
    yaw torque the live mixer delivers at the hover thrust request with the collective held, every rotor inside
    [k idle^2, k speed_max^2] for both signs. The hover thrust is run_l4.hover_thrust (m g(phi, h0) of the L4 chirp's
    site), rounded to f32 as run_l4.plan_chirp does; the mixer, idle speed and rotor coefficients are the live f32 values."""
    doc = schema.load_yaml(HOVER_SCENARIO)
    vals = {"site_latitude_rad": doc["site_latitude_rad"]["value"], "site_height_m": doc["site_height_m"]["value"]}
    c_h = r32(run_l4.hover_thrust(CARD, vals))
    rows = [[r32(real["mixer"][f"mixer_m{i}_{c}"]["value"]) for c in run_l4.MIXER_COLUMNS] for i in range(1, 5)]
    k = r32(real["cardyaml"]["rotor_thrust_coeff"]["value"])
    lo, hi = real["cardyaml"]["rotor_speed"]["value"]
    idle = r32(real["mixer"]["idle_speed"]["value"])
    held = run_l4.tau_held(rows, run_l4.MIXER_COLUMNS.index("yaw"), c_h, k * idle ** 2, k * r32(hi) ** 2)
    assert held is not None and held > 0 and lo == real["mixer"]["idle_speed"]["value"]
    return held


def fallback_index(real, omega_r):
    """n_fb: the first attitude execution with dt >= t_cross and dt alpha_min >= omega_r (D, guard b), dt = k T_a."""
    t_a = live(real, "att_loop_ratio") * real["res"]["T"]
    t_cross, alpha = live(real, "att_yaw_t_cross"), live(real, "att_yaw_alpha_min")
    k = 0
    while not (k * t_a >= t_cross and k * t_a * alpha >= omega_r):
        k += 1
    return k


def yaw_release_rates(real, j_true, tau, stick, d_nm):
    """sigma_r psi_dot at attitude executions k = 0 .. n_fb of one box member: released from the steady rate stick x
    rate_max_yaw (state [omega, 0, 0, 0, 0]), the rate setpoint zero from the release execution on (D's braking: yaw error
    zero, omega_ff = 0) and the constant torque d_nm (N m) added to the rate loop's torque output from the release, so
    v = u + d as at the L4 chirp's injection point: stepped rate execution by rate execution on the physical closed loop."""
    kp, ki = rate_gains(real, "yaw")
    T, n = real["res"]["T"], live(real, "att_loop_ratio")
    omega_r = stick * real["scenario"]["rate_max_yaw"]["value"]
    a, b = closed_step(j_true, tau, kp, ki, T)
    _, bd = sampled_plant(j_true, tau, T)
    n_fb = fallback_index(real, omega_r)
    x = [SIGMA_R * omega_r, 0.0, 0.0, 0.0, 0.0]
    out = [x[0]]
    for i in range(1, n_fb * n + 1):
        x = [sum(a[r][c] * x[c] for c in range(5)) + (bd[r] * d_nm if r < 3 else 0.0) for r in range(5)]
        if i % n == 0:
            out.append(SIGMA_R * x[0])
    return out


def members_keep_sign(real, stick, d_nm, m=FINE):
    """(True if every member's sigma_r psi_dot > 0 at every execution up to its n_fb, smallest value, its member)."""
    j_a = inertia(real, "yaw")
    worst = (math.inf, None)
    for a, c, jt, tau in box_members(real, m):
        low = min(yaw_release_rates(real, j_a * jt, tau, stick, d_nm))
        if low < worst[0]:
            worst = (low, (a, c))
    return worst[0] > 0, worst[0], worst[1]


def held_stick(real, d_nm):
    """F's rule: the held stick starts at full (1) and halves (core 7.5) until no member crosses before its fallback."""
    stick = 1.0
    for _ in range(MAX_STICK_HALVINGS):
        ok, low, at = members_keep_sign(real, stick, d_nm)
        if ok:
            return stick, low, at
        stick /= 2
    raise AssertionError("no held stick within the halving bound keeps every member from crossing before its fallback")


def test_with_d_sigma_r_tau_held_yaw_every_17x17_member_keeps_sigma_r_psi_dot_positive_up_to_its_fallback(real):
    d = SIGMA_R * tau_held_yaw(real)
    stick, low, at = held_stick(real, d)
    omega_r = stick * real["scenario"]["rate_max_yaw"]["value"]
    coarse = members_keep_sign(real, stick, d, COARSE)[1]
    msg = (f"d = {d!r} N m; held stick {stick!r} ({'full stick suffices' if stick == 1 else 'F halved it'}); smallest "
           f"sigma_r psi_dot up to n_fb over 17x17 members: {low!r} rad/s ({low / omega_r!r} of omega_r = {omega_r!r}) at "
           f"member {at}; n_fb at omega_r: {fallback_index(real, omega_r)} attitude executions; "
           + HALVING_NOTE.format(change=low - coarse) + f" rad/s (9x9 minimum {coarse!r} rad/s)")
    print(msg)
    assert 0 < stick <= 1 and low > 0, msg
    ok, _, _ = members_keep_sign(real, stick, d)
    assert ok, msg


def test_control_a_planted_zero_disturbance_crosses_before_the_fallback_at_every_member(real):
    j_a, n = inertia(real, "yaw"), live(real, "att_loop_ratio")
    t_a = n * real["res"]["T"]
    for a, c, jt, tau in box_members(real, FINE):
        rates = yaw_release_rates(real, j_a * jt, tau, 1.0, 0.0)
        first = next(k for k, v in enumerate(rates) if v <= 0)
        assert first < len(rates) - 1, f"member {(a, c)}: the crossing {first} is not before the fallback execution"
        assert first * t_a <= live(real, "att_yaw_t_cross") * (1 + 4 * EPS64)
    assert not members_keep_sign(real, 1.0, 0.0)[0]
    with pytest.raises(AssertionError, match="no held stick"):
        held_stick(real, 0.0)


def test_control_the_held_stick_halving_rule_halves_until_no_member_crosses(real):
    """A planted smaller disturbance crosses at full stick; F's halving then lands on a stick where none crosses, and the
    stick above it (one doubling back) still crosses, so the loop stops at the first passing halving, not the last."""
    d_full = tau_held_yaw(real)
    for scale in range(1, MAX_DELAY_SCALE):
        d = d_full / 2 ** scale
        if not members_keep_sign(real, 1.0, d)[0]:
            break
    else:
        pytest.fail("no planted disturbance crosses at full stick")
    stick, low, _ = held_stick(real, d)
    assert stick < 1 and low > 0
    assert not members_keep_sign(real, 2 * stick, d)[0]
    assert members_keep_sign(real, stick, d)[0]


# ---- 4. one added tick of delay in the lifted model -------------------------------------------------------------------------


def tick_lifted(j_true, tau, kp, ki, T, divisor, n, delay):
    """(P, Bs) of the closed rate loop lifted to T_a = n T at TICK resolution (n divisor ticks per attitude period), with
    the torque the plant sees `delay` ticks (0 or 1) after the rate execution computed it. State [omega, m, theta, I,
    e_prev, u_held]; the plant is the exact ZOH sampling at the tick, the rate law that of closed_step (u = kp e + I + ki T
    e_prev at each execution, every `divisor` ticks; the attitude command r is constant over the window). One tick is
    T / divisor (the primary IMU sample period); the rate execution is `divisor` ticks."""
    tick = T / divisor
    ad, bd = sampled_plant(j_true, tau, tick)
    dim = 6

    def step_tick(x, r, execute):
        w, m, th, i_acc, e_prev, u_held = x
        if execute:
            i_acc = i_acc + ki * T * e_prev
            e_prev = r - w
            new = kp * e_prev + i_acc
        else:
            new = u_held
        applied = u_held if delay else new
        plant = [ad[q][0] * w + ad[q][1] * m + ad[q][2] * th + bd[q] * applied for q in range(3)]
        return [*plant, i_acc, e_prev, new]

    p_cols, bs = [], [0.0] * dim
    for col in range(dim):
        x = [1.0 if q == col else 0.0 for q in range(dim)]
        for t in range(n * divisor):
            x = step_tick(x, 0.0, t % divisor == 0)
        p_cols.append(x)
    x = [0.0] * dim
    for t in range(n * divisor):
        x = step_tick(x, 1.0, t % divisor == 0)
    bs = x
    return [[p_cols[c][r] for c in range(dim)] for r in range(dim)], bs


def gn_dim(p, bs, theta):
    z = cmath.exp(1j * theta)
    n = len(bs)
    return csolve([[(z if i == j else 0) - p[i][j] for j in range(n)] for i in range(n)], bs)[2]


def rho_dim(p, bs, k, squarings=16):
    n = len(bs)
    m = [row[:] for row in p]
    for i in range(n):
        m[i][2] -= k * bs[i]
    log_scale = 0.0
    for _ in range(squarings):
        m = matmul(m, m)
        s = max(sum(abs(x) for x in row) for row in m)
        m = [[x / s for x in row] for row in m]
        log_scale = 2 * log_scale + math.log(s)
    return math.exp(log_scale / 2 ** squarings)


def pm_dim(p, bs, k, t_a):
    """test_attitude_design.phase_margin for a state of any size (its body is fixed at five states): same grid, same
    bisection, same unwrapping; None without a unique crossover or when unstable."""
    if rho_dim(p, bs, k) >= 1:
        return None
    ws = [1e-2]
    while ws[-1] * 1.02 < math.pi / t_a:
        ws.append(ws[-1] * 1.02)
    mags = [abs(k * gn_dim(p, bs, w * t_a)) for w in ws]
    changes = [i for i in range(len(ws) - 1) if (mags[i] > 1) != (mags[i + 1] > 1)]
    if len(changes) != 1 or not mags[0] > 1:
        return None
    lo, hi = ws[changes[0]], ws[changes[0] + 1]
    for _ in range(200):
        mid = (lo + hi) / 2
        if mid <= lo or mid >= hi:
            break
        if abs(k * gn_dim(p, bs, mid * t_a)) > 1:
            lo = mid
        else:
            hi = mid
    wx = (lo + hi) / 2
    prev = cmath.phase(k * gn_dim(p, bs, ws[0] * t_a))
    for w in [x for x in ws[1:] if x < wx] + [wx]:
        ph = cmath.phase(k * gn_dim(p, bs, w * t_a))
        prev += (ph - cmath.phase(cmath.exp(1j * prev)) + math.pi) % (2 * math.pi) - math.pi
    return math.pi + prev


def delayed_worst(real, delay):
    """(PM_worst, [(axis, corner, PM)]) of the 15 live loops at N* with `delay` ticks added, at the live f32 gains."""
    T, n = real["res"]["T"], live(real, "att_loop_ratio")
    divisor = real["scenario"]["rate_loop_divisor"]["value"]
    k, k_yaw = live_gains(real)
    detail, worst = [], math.inf
    for axis in AXES:
        kp, ki = rate_gains(real, axis)
        for name, jt, tau in corners(real):
            p, bs = tick_lifted(inertia(real, axis) * jt, tau, kp, ki, T, divisor, n, delay)
            pm = pm_dim(p, bs, k_yaw if axis == "yaw" else k, n * T)
            detail.append((axis, name, pm))
            worst = -math.inf if pm is None or worst == -math.inf else min(worst, pm)
    return worst, detail


def test_the_tick_resolution_model_without_delay_reproduces_the_oracle_and_one_added_tick_fails_the_margin_check(real):
    i, f = real["res"]["inputs"], real["res"]["final"]
    base, base_detail = delayed_worst(real, 0)
    _, oracle_detail = oracle_live_worst(real)
    for got, want in zip(base_detail, oracle_detail):
        assert got[:2] == want[:2] and abs(got[2] - want[2]) <= 16 * TOL, (got, want)
    assert base >= i["PM_min"] - f["delta_num"], "the zero-delay tick model meets the rule"
    delayed, detail = delayed_worst(real, 1)
    tick_s = real["res"]["T"] / real["scenario"]["rate_loop_divisor"]["value"]
    msg = (f"PM_worst {math.degrees(base)!r} deg -> {math.degrees(delayed)!r} deg with one added tick "
           f"({tick_s!r} s): change {delayed - base!r} rad; PM_min - delta_num = {i['PM_min'] - f['delta_num']!r} rad")
    print(msg)
    assert delayed < i["PM_min"] - f["delta_num"], msg
    assert base - delayed > f["delta_num"] + TOL, msg
    loser = min(detail, key=lambda d: d[2])
    assert loser[1] == "J+,tau+", "the worst loop stays the (J+, tau+) corner"
