#!/usr/bin/env python3
"""L6 stage (e) bias delta: the 3-axis nonlinear model of the L4 and L5 loops and delta_c(n) = NL(corner) - NL(nominal)
(decision 0019 ruling 6, "Bias corners from the nonlinear design model: yes, as a delta only"; the noise suite design memo
of 2026-10-04, section 2 and commit C5). A library for the refdata of commit C6; no command line.

Corners (decisions 0012, 0019 items 2 and ruling 9). A corner is the six turn-on bias signs in {-1, 0, +1} (gyro x y z,
then accel x y z) and the clock corner c in {-1, 0, +1}. gyro_bias is imu_corner_config's rule (tools/card/
gen_imu_config.py): axis i gets float(s_i) x the profile's gyro turn-on bound (gen_imu_config.imu_config). The accel signs
enter nothing here: no loop of L4 or L5 reads the accelerometer (0019 item 4, test e). clock_error is the realised
outward-rounded e of the Gazebo host step (0012 question E, 0019 ruling 10): n_true = gen_world.realised_host_step_ns,
e = m tick_ns / n_true - 1 in exact rationals, rounded once; the plant's tick is then t_nom / (1 + e).

The model (run). Per tick, in the composition's order (fw/compositions/l4_rate_scripted, l5_attitude_scripted; decision
0006 E), from the initial truth state (q0, w0, the rotor speeds rotor0, at rest by default):
  1. attitude group (L5 only), every attitude.ratio rate executions, execution 0 included, on the truth state of the
     tick (the L5 attitude source is truth: decision 0019 F-f): angle mode, attitude_t3_oracle.AngleMode (decision 0006
     D, the lock rule), on that execution's sticks; then attitude.law (default attitude_rate: recovery_model.law, with
     the yaw-rate command term when it is nonzero); the rate setpoint is held to the next attitude execution;
  2. sensor, every tick: l6_ff_eval.simulate's sensor (the latency, the bias hook, the chain seeded with the first
     sample, the tracked notches on the rotor telemetry); before sample 0 the latency line holds w0 (imu_model.hpp
     step 1);
  3. rate group, every D ticks: l6_ff_eval.simulate's law, in bypass at L5 (r = the held setpoint + rate_add; the
     prefilter at L4, on `setpoints`), with the D low-pass and the feed-forward; the torque hook added before allocate;
     simulate's mixer, freeze and DShot (dshot "round", stateless rounding as simulate) or the unrounded speed of the
     clamped thrust (dshot "off");
  4. plant: simulate's (the motors, the wrench from the speeds at the tick's end, the RK4 rigid body with w x J w when
     plant.coupling) or the design plant (DesignPlant: attitude_t3_oracle.Axis per axis, the request as the torque, as
     recovery_model); the attitude q <- q (x) exp(dphi / 2) per RK4 step from the exact angle increments dphi (RK4 on
     phi' = w beside w; exact for the design plant), normalised every tick (recovery_model's rule).
Run in L4 mode (setpoints) with simulate's plant it is simulate operation for operation (asserted bit for bit by the
test). Outputs per rate execution: the truth w, q and the per-axis angle phi at the execution's tick (the design plant's
Axis.th; the sum of the RK4 angle increments otherwise). Not modelled, as in simulate: the rate group's DshotDiffuser
(decision 0017; the firmware's DShot carry), the ESC clock error and the vibration.

delta (memo section 2): delta(n) = NL(corner) - NL(nominal) per channel and execution, signed, both runs at the same
plant; NL(nominal) is the corner (0, 0, 0, 0, 0, 0), clock 0, run through the same code. l4_delta runs
l6_ff_eval.simulate (the memo's L4 model); l5_delta runs this model and reduces q, w by a channel function (default
recovery_model.channels).
"""

from __future__ import annotations

import dataclasses
import math
import sys
from fractions import Fraction
from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
import l6_ff_eval as fe  # noqa: E402  (puts tools/card and the L4 oracle on sys.path)

ROOT = fe.ROOT
L5_GZ = ROOT / "tests" / "regression" / "quad" / "L05" / "gz"
if str(L5_GZ) not in sys.path:
    sys.path.insert(0, str(L5_GZ))
import gen_imu_config as gic  # noqa: E402
import gen_world  # noqa: E402
import recovery_model as rm  # noqa: E402  (puts the L5 T3 oracle on sys.path)

aor = rm.oracle  # attitude_t3_oracle
L5_FIXTURE = ROOT / "tests" / "regression" / "quad" / "L05" / "t3" / "reference" / "attitude_t3_inputs.txt"
DSHOT_ROUND, DSHOT_OFF = "round", "off"
IDENTITY = (1.0, 0.0, 0.0, 0.0)
REST = (0.0, 0.0, 0.0)
UNBOUNDED = (math.inf, math.inf, math.inf)


# ---- corners --------------------------------------------------------------------------------------------------------

@dataclasses.dataclass(frozen=True)
class Corner:
    """Six turn-on bias signs (gyro x y z, accel x y z: imu_corner_config's order) and the clock corner."""
    signs: tuple = (0, 0, 0, 0, 0, 0)
    clock: int = 0


NOMINAL = Corner()


def check_corner(corner):
    signs = tuple(corner.signs)
    if len(signs) != gic.SIGN_COUNT or not all(isinstance(s, int) and not isinstance(s, bool) and s in gic.BIAS_SIGNS
                                               for s in signs):
        raise ValueError(f"signs must be {gic.SIGN_COUNT} integers in {list(gic.BIAS_SIGNS)}, got {signs!r}")
    if isinstance(corner.clock, bool) or corner.clock not in gen_world.CLOCK_CORNERS:
        raise ValueError(f"clock must be one of {list(gen_world.CLOCK_CORNERS)}, got {corner.clock!r}")


def gyro_bias(signs, profile=fe.PROFILE):
    """The gyro turn-on bias per axis (rad/s) at the corner signs: imu_corner_config's rule, float(s_i) x bound."""
    check_corner(Corner(tuple(signs)))
    bound = gic.imu_config(profile)[0]["gyro_turn_on_bias_bound"]
    return tuple(float(s) * bound for s in tuple(signs)[:3])


def clock_error(clock, l2_doc, m=1, profile=fe.PROFILE):
    """e of the clock corner (module docstring) for the world of the L2-schema scenario l2_doc at m ticks per step."""
    check_corner(Corner(clock=clock))
    if clock == 0:
        return 0.0
    vals = fe.run_l4.scn.values(l2_doc)
    tick_ns = Fraction(vals["tick_period_num_us"] * gen_world.NS_PER_US, vals["tick_period_den"])
    n_true = gen_world.realised_host_step_ns(l2_doc, m, clock, gic.imu_config(profile)[0]["odr_error"])
    return float(m * tick_ns / n_true - 1)


# ---- the model --------------------------------------------------------------------------------------------------------

@dataclasses.dataclass(frozen=True)
class DesignPlant:
    """The design plant: attitude_t3_oracle.Axis per axis (J w' = u_m, tau u_m' = u - u_m, exact ZOH over the step, the
    torque request as u), no mixer, DShot, rotors or w x J w (recovery_model's plant)."""
    inertia: tuple
    tau: float


@dataclasses.dataclass(frozen=True)
class RatePlan:
    """The rate executions 0 .. end_execution of a run: stamps floor(D k num / den) us, the tick, the thrust request."""
    num_us: int
    den: int
    divisor: int
    end_execution: int
    thrust_n: float
    tick_s: object

    def stamp_us(self, k):
        return (self.divisor * k * self.num_us) // self.den


@dataclasses.dataclass(frozen=True)
class Attitude:
    """The L5 attitude group (module docstring, step 1): cfg an attitude_t3_oracle.Config, the period in rate executions,
    the sticks (s_roll, s_pitch, s_yaw) per attitude execution, and the law (cfg, q, q_sp, yaw_rate_cmd) -> rate
    setpoint (None: attitude_rate)."""
    cfg: object
    ratio: int
    sticks: tuple
    law: object = None


@dataclasses.dataclass(frozen=True)
class _Gains:
    kp: float
    w: float
    rate_max: tuple


def clamp(cfg, r):
    """attitude_law.hpp clamp in binary64 (recovery_model.law's): the roll-pitch pair scaled by one factor, yaw clamped."""
    bx, by, bz = cfg.rate_max
    ax, ay = abs(r[0]), abs(r[1])
    sx, sy = (bx / ax if ax > bx else 1.0), (by / ay if ay > by else 1.0)
    sc = min(sx, sy)
    return [math.copysign(min(abs(r[0] * sc), bx), r[0]), math.copysign(min(abs(r[1] * sc), by), r[1]),
            math.copysign(min(abs(r[2]), bz), r[2])]


def attitude_rate(cfg, q, q_sp, cmd):
    """The attitude law (attitude_law.hpp execute) in binary64: recovery_model.law when the yaw-rate command is 0; else
    the same law unclamped plus rotate(conj(q^), (0, 0, cmd)), q^ = q / |q|, then clamped."""
    if cmd == 0:
        return rm.law(cfg, q, q_sp)
    r = rm.law(_Gains(cfg.kp, cfg.w, UNBOUNDED), q, q_sp)
    n = math.sqrt(sum(c * c for c in q))
    ff = aor.qrotate(aor.qconj(tuple(c / n for c in q)), (0.0, 0.0, cmd))
    return clamp(cfg, [r[i] + ff[i] for i in range(3)])


def oracle_rate(cfg, q, q_sp, cmd):
    """attitude_t3_oracle.attitude_law's values: the T3 golden's law (the same rule in its own operation order)."""
    return [x.v for x in aor.attitude_law(cfg, tuple(aor.E(c) for c in q), tuple(aor.E(c) for c in q_sp), cmd)]


def run(law, sensor, plant, plan, thrust, *, setpoints=None, attitude=None, q0=IDENTITY, w0=REST, rotor0=None,
        bias=None, clock_error=0.0, torque=None, rate_add=None, dshot=DSHOT_ROUND, substeps=fe.RK_SUBSTEPS):
    """The model (module docstring). setpoints (L4: per axis per rate execution, through the prefilter) or attitude
    (L5: an Attitude, the rate loop in bypass), exactly one. torque, rate_add: per axis per rate execution, or None.
    Returns {"w", "phi": per axis per rate execution; "q": per rate execution; "flagged": per axis; "phase": angle mode's
    phase per attitude execution (L5)}."""
    if (setpoints is None) == (attitude is None):
        raise ValueError("give exactly one of setpoints (L4) and attitude (L5)")
    if dshot not in (DSHOT_ROUND, DSHOT_OFF):
        raise ValueError(f"dshot must be {DSHOT_ROUND!r} or {DSHOT_OFF!r}")
    design = isinstance(plant, DesignPlant)
    tr = sensor.tracking
    if design and tr:
        raise ValueError("notch tracking reads the rotor speeds; the design plant has none")
    d_div = plan.divisor
    n_exec = plan.end_execution + 1
    h = float(plan.tick_s) / (1 + clock_error) if clock_error else float(plan.tick_s)
    hs = h / substeps
    dts = fe.exec_dts(plan)
    alpha_cache = {}

    def alpha(dt, tc):
        key = (dt, tc)
        if key not in alpha_cache:
            alpha_cache[key] = -math.expm1(-dt / tc)
        return alpha_cache[key]

    kp, ki, kd, tref, dtau = law.kp, law.ki, law.kd, law.tau_ref, law.d_tau
    fj = law.inertia
    ff_on = law.ff
    tau_m, ff_tau = law.motor_tau, law.ff_tau

    if design:
        axes = []
        for i in range(3):
            ax = aor.Axis(plant.inertia[i], plant.tau, hs)
            ax.w = w0[i]
            axes.append(ax)
    else:
        jx, jy, jz = plant.inertia
        cx, cy, cz = ((jz - jy), (jx - jz), (jy - jx)) if plant.coupling else (0.0, 0.0, 0.0)
        decay = math.exp(-h / plant.tau)
        k_t = plant.k
        yaw_t = [s * plant.c_q for s in plant.spin]
        px, py = plant.pos_x, plant.pos_y
        span = plant.omega_max - plant.omega_min
        dspan = float(plant.d_max - plant.d_min)
        d_lo = math.ceil(plant.d_min + (plant.idle - plant.omega_min) / span * dspan)
        f_lo, f_hi = k_t * (plant.idle * plant.idle), k_t * (plant.omega_max * plant.omega_max)
        mm = plant.m
        m_thr = [row[0] for row in mm]
        pair_r = [[f_hi * m_thr[i] - f_lo * m_thr[j] for j in range(4)] for i in range(4)]
        eps32 = 2.0 ** -23  # binary32 epsilon (rate_loop.hpp record_allocation: numeric_limits<float>::epsilon)
        n_eps = 4 * eps32
        gamma = n_eps / (1 - n_eps)

        def deriv(a, b, c, tx, ty, tz):
            return (tx - cx * b * c) / jx, (ty - cy * c * a) / jy, (tz - cz * a * b) / jz

    # state
    w = [float(x) for x in w0]
    q = tuple(q0)
    phi = [0.0, 0.0, 0.0]
    rot = [float(x) for x in rotor0] if rotor0 is not None else [0.0, 0.0, 0.0, 0.0]
    wcmd = [0.0, 0.0, 0.0, 0.0]
    delay = [list(w) for _ in range(sensor.latency)]
    notch_cache = {}
    stages = ([fe.gcd.IDENTITY] * (fe.gcd.MOTORS * len(fe.gcd.HARMONICS)) if tr else []) + list(sensor.stages)
    chain = [[[0.0, 0.0, 0.0, 0.0] for _ in stages] for _ in range(3)]
    tel_delay = None
    r = [0.0] * 3
    e_prev = [0.0] * 3
    integ = [0.0] * 3
    y_prev = [0.0] * 3
    d_f = [0.0] * 3
    g_prev = [0.0] * 3
    gdot = [0.0] * 3
    freeze = [False] * 3
    rate_sp = [0.0] * 3
    u = [0.0] * 3
    out = [[0.0] * n_exec for _ in range(3)]
    out_phi = [[0.0] * n_exec for _ in range(3)]
    out_q = [None] * n_exec
    flagged = [0, 0, 0]
    phases = []
    if attitude is not None:
        angle = aor.AngleMode(attitude.cfg)
        att_law = attitude.law or attitude_rate

    for j in range(d_div * (n_exec - 1) + 1):
        if design:
            w = [ax.w for ax in axes]
        # 1. attitude group, on the truth state of the tick
        if attitude is not None and j % (d_div * attitude.ratio) == 0:
            k = j // (d_div * attitude.ratio)
            sp_q, cmd = angle.execute(attitude.sticks[k], plan.stamp_us(j // d_div), tuple(aor.E(c) for c in q),
                                      tuple(aor.E(c) for c in w))
            rate_sp = att_law(attitude.cfg, q, tuple(aor._val(c) for c in sp_q), cmd)
            phases.append(angle.phase)
        # 2. sensor (l6_ff_eval.simulate's)
        if tr:
            now = [fe.telemetry(om, tr) for om in rot]
            if tel_delay is None:
                tel_delay = [now] * tr.latency
            tel_delay.append(now)
            reported = tel_delay.pop(0)
            if j % d_div == 0:
                stages[:len(stages) - len(sensor.stages)] = fe.notch_set(reported, tr, notch_cache)
        if delay:
            delay.append(list(w))
            sample = delay.pop(0)
        else:
            sample = w
        if bias is not None:
            sample = [x + b for x, b in zip(sample, bias)]
        y = [0.0, 0.0, 0.0]
        for a in range(3):
            v = sample[a]
            if j == 0:
                for s in chain[a]:
                    s[0] = s[1] = s[2] = s[3] = v
            for (b0, b1, b2, a1, a2), s in zip(stages, chain[a]):
                o = b0 * v + b1 * s[0] + b2 * s[1] - a1 * s[2] - a2 * s[3]
                s[1] = s[0]
                s[0] = v
                s[3] = s[2]
                s[2] = o
                v = o
            y[a] = v
        # 3. rate group (l6_ff_eval.simulate's law; bypass at L5)
        if j % d_div == 0:
            n = j // d_div
            for a in range(3):
                out[a][n] = w[a]
                out_phi[a][n] = axes[a].th if design else phi[a]
            out_q[n] = q
            u = [0.0, 0.0, 0.0]
            if n == 0:
                r = list(y)
                e_prev = [0.0] * 3
                integ = [0.0] * 3
                if ff_on:
                    jy_ = (fj[0] * y[0], fj[1] * y[1], fj[2] * y[2])
                    g_prev = [y[1] * jy_[2] - y[2] * jy_[1], y[2] * jy_[0] - y[0] * jy_[2], y[0] * jy_[1] - y[1] * jy_[0]]
            else:
                dt = dts[n - 1]
                for a in range(3):
                    if setpoints is not None:
                        r[a] = r[a] + alpha(dt, tref[a]) * (setpoints[a][n] - r[a])
                    else:
                        r[a] = rate_sp[a] + rate_add[a][n] if rate_add is not None else rate_sp[a]
                    e = r[a] - y[a]
                    if not freeze[a]:
                        integ[a] = integ[a] + ki[a] * e_prev[a] * dt
                    d = -kd[a] * (y[a] - y_prev[a]) / dt
                    if dtau[a] > 0:
                        d_f[a] = d_f[a] + alpha(dt, dtau[a]) * (d - d_f[a])
                        d = d_f[a]
                    u[a] = kp[a] * e + integ[a] + d
                    e_prev[a] = e
                if ff_on:
                    jy_ = (fj[0] * y[0], fj[1] * y[1], fj[2] * y[2])
                    g = (y[1] * jy_[2] - y[2] * jy_[1], y[2] * jy_[0] - y[0] * jy_[2], y[0] * jy_[1] - y[1] * jy_[0])
                    for a in range(3):
                        raw = (g[a] - g_prev[a]) / dt
                        if ff_tau > 0:
                            gdot[a] = gdot[a] + alpha(dt, ff_tau) * (raw - gdot[a])
                        else:
                            gdot[a] = raw
                        u[a] = u[a] + (g[a] + tau_m * gdot[a])
                    g_prev = list(g)
            y_prev = list(y)
            if torque is not None:
                u = [u[a] + torque[a][n] for a in range(3)]
            if not design:
                _mix(u, thrust, mm, m_thr, pair_r, f_lo, f_hi, gamma, eps32, plant, e_prev, freeze, flagged, k_t, span,
                     dspan, d_lo, wcmd, dshot)
            if n == n_exec - 1:
                break
        # 4. plant
        if design:
            for _ in range(substeps):
                dphi = [ax.step(u[i]) for i, ax in enumerate(axes)]
                q = aor.qmul(q, aor.quat_exp(dphi))
        else:
            for i in range(4):
                rot[i] = wcmd[i] + (rot[i] - wcmd[i]) * decay
            thr = [k_t * om * om for om in rot]
            tx = -sum(py[i] * thr[i] for i in range(4))
            ty = sum(px[i] * thr[i] for i in range(4))
            tz = sum(yaw_t[i] * thr[i] for i in range(4))
            a0, b0_, c0 = w
            for _ in range(substeps):
                k1 = deriv(a0, b0_, c0, tx, ty, tz)
                m1 = (a0 + hs / 2 * k1[0], b0_ + hs / 2 * k1[1], c0 + hs / 2 * k1[2])
                k2 = deriv(*m1, tx, ty, tz)
                m2 = (a0 + hs / 2 * k2[0], b0_ + hs / 2 * k2[1], c0 + hs / 2 * k2[2])
                k3 = deriv(*m2, tx, ty, tz)
                m3 = (a0 + hs * k3[0], b0_ + hs * k3[1], c0 + hs * k3[2])
                k4 = deriv(*m3, tx, ty, tz)
                dphi = [hs / 6 * (s0 + 2 * s1 + 2 * s2 + s3) for s0, s1, s2, s3 in zip((a0, b0_, c0), m1, m2, m3)]
                a0 = a0 + hs / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0])
                b0_ = b0_ + hs / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1])
                c0 = c0 + hs / 6 * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2])
                for i in range(3):
                    phi[i] += dphi[i]
                q = aor.qmul(q, aor.quat_exp(dphi))
            w = [a0, b0_, c0]
        nq = math.sqrt(sum(c * c for c in q))
        q = tuple(c / nq for c in q)
    return {"w": out, "phi": out_phi, "q": out_q, "flagged": flagged, "phase": phases}


def _mix(u, thrust, mm, m_thr, pair_r, f_lo, f_hi, gamma, eps32, plant, e_prev, freeze, flagged, k_t, span, dspan, d_lo,
         wcmd, dshot):
    """l6_ff_eval.simulate's mixer allocate, record_allocation's freeze and thrust_to_dshot, then the ESC map, into wcmd;
    with dshot "off" the commanded speed is the unrounded speed of the clamped thrust."""
    tx, ty, tz = u
    av = [row[1] * tx + row[2] * ty for row in mm]
    yv = [row[3] * tz for row in mm]
    s_ = 1.0
    for i in range(4):
        for q in range(4):
            if i != q:
                pa = av[q] * m_thr[i] - av[i] * m_thr[q]
                if pa > 0:
                    rr = pair_r[i][q] / pa
                    if rr < s_:
                        s_ = rr
    t_ = 0.0
    if s_ >= 1:
        t_ = 1.0
        for i in range(4):
            for q in range(4):
                if i != q:
                    py_ = yv[q] * m_thr[i] - yv[i] * m_thr[q]
                    if py_ > 0:
                        room = pair_r[i][q] - (av[q] * m_thr[i] - av[i] * m_thr[q])
                        if room < 0:
                            room = 0.0
                        rr = room / py_
                        if rr < t_:
                            t_ = rr
    p = [s_ * av[i] + t_ * yv[i] for i in range(4)]
    c_lo = max((f_lo - p[i]) / m_thr[i] for i in range(4))
    c_hi = min((f_hi - p[i]) / m_thr[i] for i in range(4))
    c = thrust
    if c > c_hi:
        c = c_hi
    if c < c_lo:
        c = c_lo
    ach = (s_ * tx, s_ * ty, t_ * tz)
    flags = (s_ < 1 and tx != 0, s_ < 1 and ty != 0, t_ < 1 and tz != 0)
    vv = (abs(c), abs(ach[0]), abs(ach[1]), abs(ach[2]))
    for a in range(3):
        bound = (gamma + eps32) * sum(plant.abs_bm[a][q] * vv[q] for q in range(4))
        diff = u[a] - ach[a]
        same = (e_prev[a] > 0 and diff > 0) or (e_prev[a] < 0 and diff < 0)
        freeze[a] = flags[a] and abs(diff) > bound and same
        if flags[a]:
            flagged[a] += 1
    for i in range(4):
        f = p[i] + c * m_thr[i]
        if not f >= f_lo:
            f = f_lo
        elif f > f_hi:
            f = f_hi
        om = math.sqrt(f / k_t)
        if dshot == DSHOT_OFF:
            wcmd[i] = om
            continue
        dq = fe.round_half_away(plant.d_min + (om - plant.omega_min) / span * dspan)
        if not dq >= d_lo:
            dq = d_lo
        elif dq > plant.d_max:
            dq = plant.d_max
        wcmd[i] = plant.omega_min + (plant.omega_max - plant.omega_min) * ((dq - plant.d_min) / dspan)


# ---- the L5 inputs ------------------------------------------------------------------------------------------------------

def l5_setup(fixture_path=L5_FIXTURE):
    """(the L5 T3 fixture's values, its attitude_t3_oracle.Setup)."""
    p = aor.read_inputs(fixture_path)
    return p, aor.Setup(p)


def l5_law(p, ff=None):
    """The L5 rate law of the fixture p (bypass: tau_ref unused), as an l6_ff_eval.Law; ff = (J, tau_m, T_ff) or None."""
    get = (lambda q: tuple(p[f"rate_{q}_{a}"] for a in aor.AXES))
    law = fe.Law("L5 fixture", kp=get("kp"), ki=get("ki"), kd=get("kd"), tau_ref=get("tau_ref"), d_tau=get("d_filter_tau"))
    if ff is None:
        return law
    j, tau_m, t_ff = ff
    return dataclasses.replace(law, name="L5 fixture +FF+lag", inertia=tuple(j), motor_tau=tau_m, ff_tau=t_ff)


def l5_plan(su, n_exec, thrust):
    """The RatePlan of n_exec rate executions on the L5 oracle's tick (su.tick_s) and stamps."""
    return RatePlan(num_us=su.num, den=su.den, divisor=su.divisor, end_execution=n_exec - 1, thrust_n=thrust,
                    tick_s=su.tick_s)


# ---- the bias delta -----------------------------------------------------------------------------------------------------

def corner_inputs(corner, l2_doc, m=1, profile=fe.PROFILE):
    """(gyro bias, clock error) of a corner (module docstring)."""
    check_corner(corner)
    return gyro_bias(corner.signs, profile), clock_error(corner.clock, l2_doc, m, profile)


def l4_run(law, sensor, plant, plan, sps, corner, l2_doc, torque=None):
    """w per axis per execution of l6_ff_eval.simulate at the corner (the bias and clock hooks)."""
    bias, e = corner_inputs(corner, l2_doc)
    w, _ = fe.simulate(law, sensor, plant, plan, sps, plan.thrust_n, bias=bias, clock_error=e, torque=torque)
    return w


def l4_delta(law, sensor, plant, plan, sps, corner, l2_doc, torque=None):
    """delta per axis per execution (module docstring): l4_run(corner) - l4_run(NOMINAL)."""
    a = l4_run(law, sensor, plant, plan, sps, corner, l2_doc, torque)
    b = l4_run(law, sensor, plant, plan, sps, NOMINAL, l2_doc, torque)
    return [[x - y for x, y in zip(ra, rb)] for ra, rb in zip(a, b)]


def l5_run(law, sensor, plant, plan, attitude, corner, l2_doc, channels=rm.channels, **kw):
    """Channels per attitude execution of run() in L5 mode at the corner; kw: run()'s initial state, torque, rate_add,
    dshot."""
    bias, e = corner_inputs(corner, l2_doc)
    res = run(law, sensor, plant, plan, plan.thrust_n, attitude=attitude, bias=bias, clock_error=e, **kw)
    w, q = res["w"], res["q"]
    return [channels(q[n], [w[a][n] for a in range(3)]) for n in range(0, len(q), attitude.ratio)]


def l5_delta(law, sensor, plant, plan, attitude, corner, l2_doc, channels=rm.channels, **kw):
    """delta per channel (outer) per attitude execution: l5_run(corner) - l5_run(NOMINAL)."""
    a = l5_run(law, sensor, plant, plan, attitude, corner, l2_doc, channels, **kw)
    b = l5_run(law, sensor, plant, plan, attitude, NOMINAL, l2_doc, channels, **kw)
    return [[x[c] - y[c] for x, y in zip(a, b)] for c in range(len(a[0]))]
