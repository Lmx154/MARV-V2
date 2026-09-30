"""The 3-axis design-model envelope of the L5 T4 large-angle recovery scenarios (decision 0006 F "T4 large-angle recovery",
"T3 envelopes", "Tolerance terms at T4"; owner decisions 15 and 19).

Helper of test_t4_recovery.py (tests/regression/quad/L05/gz). The recorded T3 envelope file has no recovery script and t3/ is
not this packet's to extend, so the envelope is computed here from the T3 oracle's own pieces (attitude_t3_oracle.py: the
fixture reader, Setup, Axis, the quaternion helpers, grid_members, Envelope, Quantiser), by the oracle's rules:

  Design model. Per axis the exact ZOH plant J w' = u_m, tau u_m' = u - u_m (oracle Axis, stepped per tick with u held over
  the rate period), the rate loop in bypass with the f32 nominal gains (the oracle's simulate loop: seed execution u = 0,
  integral step = the stamp dt), the attitude law of decision 0006 C in closed form on the full quaternion, the quaternion
  integrated per tick from the exact per-axis angle increments (q <- q (x) exp(dphi / 2)). No w x Jw (as at L4): a coupling
  limit shows up as an envelope exit. The state is seeded once, at execution 0, with the scenario's exact initial attitude
  and rates (motor torque state, integrator and previous error 0) and never re-seeded. The setpoint is level, locked at the
  heading of angle mode's first execution (2 atan2(z, w) of the initial attitude, 0 at w = z = 0).
  Envelope. Pointwise min / max over the 17 x 17 tau x J grid of the band box (J (1 +- inertia_robustness_band), tau
  (1 +- tau_robustness_band), all three axes at once), its last-halving change from the 9 x 9 grid.
  Channels (per attitude execution, before that execution's torque acts): the body-frame rotation vector of the attitude
  error q_e = canonical(conj(q) (x) q_sp), components (x, y) the tilt error and z the heading error, and the body rates.
  Kinematics term. The largest change of a channel between the run and the run with every tick split in two sub-steps, at
  the nominal and the four corner members.
  Q (owner decision 19). The largest |quantised - unquantised| of a channel over time at the corners and over the 9 x 9
  grid (oracle q_script's rule): the torque request passes through the oracle's Quantiser (mixer allocation at the
  scenario collective, thrust_to_dshot rounding, marv_plant's ESC map and rotor geometry), here for the full 3-vector.
"""

import math
import multiprocessing
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REFERENCE = os.path.abspath(os.path.join(HERE, os.pardir, "t3", "reference"))
sys.path.insert(0, REFERENCE)
import attitude_t3_oracle as oracle  # noqa: E402

CHANNELS = ("err_x", "err_y", "err_z", "w_x", "w_y", "w_z")
HORIZON_TAUS = oracle.HORIZON_TAUS
MAX_DOUBLINGS = oracle.MAX_DOUBLINGS


def rotation_vector(q):
    """2 atan2(|q_v|, w) q_v / |q_v| of the canonical (w >= 0) quaternion q = [w, x, y, z]."""
    w, x, y, z = q
    if w < 0:
        w, x, y, z = -w, -x, -y, -z
    n = math.sqrt(x * x + y * y + z * z)
    if n == 0.0:
        return (0.0, 0.0, 0.0)
    s = 2.0 * math.atan2(n, w) / n
    return (x * s, y * s, z * s)


def canonical(q):
    for c in q:
        if c != 0:
            return tuple(-x for x in q) if c < 0 else tuple(q)
    return tuple(q)


def level_error(q):
    """The error quaternion canonical(conj(q^) (x) [1, 0, 0, 0]) of a level setpoint, q^ = q / |q|."""
    n = math.sqrt(sum(c * c for c in q))
    return canonical((q[0] / n, -q[1] / n, -q[2] / n, -q[3] / n))


def channels(q, w):
    """(err_x, err_y, err_z, w_x, w_y, w_z) of attitude q [w, x, y, z] and body rates w."""
    return (*rotation_vector(level_error(q)), *w)


def law(cfg, q, q_sp):
    """decision 0006 C on binary64: the rate setpoint of the attitude law for yaw_rate_cmd = 0, clamped as the firmware does."""
    n = math.sqrt(sum(c * c for c in q))
    qh = (q[0] / n, q[1] / n, q[2] / n, q[3] / n)
    ns = math.sqrt(sum(c * c for c in q_sp))
    qs = tuple(c / ns for c in q_sp)
    qe = canonical(oracle_qmul_f(oracle.qconj(qh), qs))
    rho = math.hypot(qe[0], qe[3])
    if rho > 0:
        a = (qe[0] * qe[1] - qe[2] * qe[3]) / rho
        b = (qe[0] * qe[2] + qe[1] * qe[3]) / rho
        qz = canonical((qe[0] / rho, 0.0, 0.0, qe[3] / rho))
        half = cfg.w * 2.0 * math.atan2(qz[3], qz[0]) * 0.5
        c, s = math.cos(half), math.sin(half)
        qe = (rho * c, c * a + s * b, c * b - s * a, rho * s)
    qc = canonical(qe)
    r = [cfg.kp * (2 * qc[1]), cfg.kp * (2 * qc[2]), (cfg.kp / cfg.w) * (2 * qc[3])]
    bx, by, bz = cfg.rate_max
    ax, ay = abs(r[0]), abs(r[1])
    sx, sy = (bx / ax if ax > bx else 1.0), (by / ay if ay > by else 1.0)
    sc = min(sx, sy)
    r[0], r[1] = math.copysign(min(abs(r[0] * sc), bx), r[0]), math.copysign(min(abs(r[1] * sc), by), r[1])
    r[2] = math.copysign(min(abs(r[2]), bz), r[2])
    return r


def oracle_qmul_f(a, b):
    return oracle.qmul(a, b)


class Quant3:
    """The oracle Quantiser for a 3-vector torque request: the allocation and the DShot rounding of oracle.Quantiser, and
    the plant's torque of all three axes (oracle Quantiser.torque projects one)."""

    def __init__(self, q):
        self.qz = oracle.Quantiser(q, 0)
        self.saturated = 0

    def __call__(self, u):
        qz = self.qz
        f, scaled = qz.allocate(*u)
        if scaled:
            self.saturated += 1
        d = qz.dshot(f)
        out = [0.0, 0.0, 0.0]
        span = float(oracle.DSHOT_THROTTLE_MAX - oracle.DSHOT_THROTTLE_MIN)
        for di, g in zip(d, qz.geo):
            omega = 0.0 if di == 0 else qz.p_min + (qz.p_max - qz.p_min) * (float(di - oracle.DSHOT_THROTTLE_MIN) / span)
            t = qz.kp * omega * omega
            for ax in range(3):
                out[ax] += g[ax] * t
        return out


def member_run(su, s, t, q0, w0, n_exec, quant=None, m_sub=1, fn=None):
    """Channels per attitude execution 0 .. n_exec - 1 of one band member (module docstring), and the quantiser's saturated
    count."""
    p, cfg = su.p, su.cfg
    n_ticks = su.divisor * su.ratio
    heading = 0.0 if (q0[0] == 0 and q0[3] == 0) else 2.0 * math.atan2(q0[3], q0[0])
    q_sp = (math.cos(heading / 2), 0.0, 0.0, math.sin(heading / 2))
    plants = []
    for i, name in enumerate(oracle.AXES):
        pl = oracle.Axis(p[oracle.INERTIA[name]] * (1 + s * su.j_band), p["motor_tau"] * (1 + t * su.tau_band),
                         su.tick_s / m_sub)
        pl.w = w0[i]
        plants.append(pl)
    kp = [p[f"rate_kp_{a}"] for a in oracle.AXES]
    ki = [p[f"rate_ki_{a}"] for a in oracle.AXES]
    integral, e_prev, u = [0.0] * 3, [0.0] * 3, [0.0] * 3
    q = tuple(q0)
    out = []
    r_hold = [0.0] * 3
    for j in range(n_exec * n_ticks):
        if j % n_ticks == 0:
            a = j // n_ticks
            out.append((fn or channels)(q, [pl.w for pl in plants]))
            r_hold = law(cfg, q, q_sp)
            if a == 0:
                u = [0.0] * 3
            else:
                dt = su.dt_exec(a)
                y = [pl.w for pl in plants]
                req = [0.0] * 3
                for i in range(3):
                    integral[i] += ki[i] * e_prev[i] * dt
                    e = r_hold[i] - y[i]
                    req[i] = kp[i] * e + integral[i]
                    e_prev[i] = e
                u = quant(req) if quant is not None else req
            if a == 0 and quant is not None:
                u = quant([0.0, 0.0, 0.0])
        for _ in range(m_sub):
            d = [pl.step(u[i]) for i, pl in enumerate(plants)]
            q = oracle.qmul(q, oracle.quat_exp(d))
        nq = math.sqrt(sum(c * c for c in q))
        q = tuple(c / nq for c in q)
    return out, (quant.saturated if quant is not None else 0)


_CTX = {}


def _job(args):
    index, s, t, coarse, with_q = args
    su, q0, w0, n, qfix, fn = (_CTX[k] for k in ("su", "q0", "w0", "n", "qfix", "fn"))
    base, _ = member_run(su, s, t, q0, w0, n, fn=fn)
    quantised = saturated = None
    if with_q:
        qr, saturated = member_run(su, s, t, q0, w0, n, Quant3(qfix), fn=fn)
        quantised = [max(abs(a - b) for a, b in zip(x, y)) for x, y in zip(zip(*base), zip(*qr))]
        quantised = [max(abs(a - b) for a, b in zip(x, y)) for x, y in zip(zip(*base), zip(*qr))]
    kin = None
    if (abs(s) == 1.0 and abs(t) == 1.0) or (s == 0.0 and t == 0.0):
        fine, _ = member_run(su, s, t, q0, w0, n, m_sub=2, fn=fn)
        kin = [max(abs(a[c] - b[c]) for a, b in zip(base, fine)) for c in range(len(base[0]))]
    return index, s, t, coarse, base, quantised, saturated, kin


def envelope(su, qfix, q0, w0, n_exec, procs=None, fn=None):
    """{"lo", "hi": per channel lists per execution; "halving": per channel; "kin": per channel; "q": per channel dict; "nominal":
    the nominal member's channels per execution; "saturated": count} for the scenario (initial attitude q0, rates w0) over
    n_exec attitude executions."""
    _CTX.update(su=su, q0=tuple(q0), w0=tuple(w0), n=n_exec, qfix=qfix, fn=fn)
    members = oracle.grid_members()
    jobs = []
    for i, s, t, coarse in members:
        corner = (abs(s) == 1.0 and abs(t) == 1.0) or (s == 0.0 and t == 0.0)
        jobs.append((i, s, t, coarse, coarse or corner))
    procs = procs or os.cpu_count() or 1
    with multiprocessing.get_context("fork").Pool(procs) as pool:
        results = pool.map(_job, jobs, chunksize=1)
    nch = len((fn or channels)(q0, w0))
    envs = [oracle.Envelope(n_exec) for _ in range(nch)]
    q_best = [0.0] * nch
    q_corner = [0.0] * nch
    kin = [0.0] * nch
    saturated = 0
    nominal = None
    for _, s, t, coarse, base, quantised, sat, k in results:
        cols = list(zip(*base))
        for c in range(nch):
            envs[c].add(cols[c], coarse)
        if quantised is not None:
            for c in range(nch):
                q_best[c] = max(q_best[c], quantised[c])
                if (abs(s) == 1.0 and abs(t) == 1.0) or (s == 0.0 and t == 0.0):
                    q_corner[c] = max(q_corner[c], quantised[c])
            saturated += sat
        if k is not None:
            kin = [max(a, b) for a, b in zip(kin, k)]
        if s == 0.0 and t == 0.0:
            nominal = base
    return {"lo": [e.lo for e in envs], "hi": [e.hi for e in envs], "halving": [e.halving_change() for e in envs],
            "kin": kin, "q": q_best, "q_corners": q_corner, "nominal": nominal, "saturated": saturated}


def alpha_channel(q, w):
    """(alpha,): the tilt angle, the angle between body z and NED down, of the level error (axis-invariant)."""
    v = rotation_vector(level_error(q))
    th = math.sqrt(sum(c * c for c in v))
    if th == 0:
        return (0.0,)
    k = math.sin(th / 2) / th
    return (2.0 * math.asin(min(1.0, math.hypot(v[0] * k, v[1] * k))),)


def settle_ok(env, f_term):
    """The T3 property at the last execution: every channel's envelope end value is below its F."""
    return [max(abs(env["lo"][c][-1]), abs(env["hi"][c][-1])) < f_term[c] for c in range(len(env["lo"]))]
