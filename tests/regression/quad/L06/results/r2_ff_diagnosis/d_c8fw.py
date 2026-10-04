"""(4) c8's run C (design model controller + plant C: rigid body with w x Jw, first-order torque lag from tau0) with the
feed-forward in different forms, against envelope A (E = 0). Binary64 throughout.
  ideal   c8 exactly: continuous f = g + tau dg/dt on the true state, inside the plant (r2_envelope_a.Body ff=True)
  zoh     the same f sampled at each rate execution on the true state and held (discrete, unfiltered, true w)
  fw      the firmware's form (fw/rate rate_loop.hpp): g on the chain output y, gdot = T_ff-filtered backward difference,
          seed execution outputs 0 and seeds g_prev = g(y); J, tau_m nominal
  fw0     fw with T_ff = 0 (raw backward difference)
  +zr     the harness's execution-0 zero gyro read (chain and rate loop seed on w = 0)
"""
import math
import os
import pickle
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[5]
S = Path(os.environ["MARV_R2FF_WORK"]).resolve()
sys.path.insert(0, str(ROOT / "tests" / "regression" / "quad" / "L06" / "results" / "r2_envelope_a"))
sys.path.insert(0, str(HERE))
import r2_envelope_a as ea  # noqa: E402
from common import against, fmt_res  # noqa: E402

rm, oracle, run_l5 = ea.rm, ea.oracle, ea.run_l5
ctx = ea.setup()
su, q0, w0, N = ctx["su"], ctx["q0"], ctx["w0"], ctx["n"]
design = pickle.loads((S / "design_r2.pkl").read_bytes())
J, TAU, TAU0 = ctx["J"], ctx["tau"], ctx["tau0"]
p = su.p
T_FF = 0.012311602011322975  # the build's rate_ff_filter_tau (gz_r2.txt params line)


def run(ff, zero_read=False, t_ff=T_FF, substeps=4, n_exec=N, j_ff=None, tau_ff=None):
    j_ff = j_ff or J
    tau_ff = TAU if tau_ff is None else tau_ff
    body = ea.Body(J, TAU, su.tick_s, TAU0, substeps, coupled=True, ff=(ff == "ideal"))
    body.w = list(w0)
    cfg = su.cfg
    n_ticks = su.divisor * su.ratio
    heading = 0.0 if (q0[0] == 0 and q0[3] == 0) else 2.0 * math.atan2(q0[3], q0[0])
    q_sp = (math.cos(heading / 2), 0.0, 0.0, math.sin(heading / 2))
    kp = [p[f"rate_kp_{a}"] for a in oracle.AXES]
    ki = [p[f"rate_ki_{a}"] for a in oracle.AXES]
    kd = [p[f"rate_kd_{a}"] for a in oracle.AXES]
    b0, b1, b2, a1, a2 = su.lpf
    chain, y_tick = [None] * 3, [0.0] * 3
    integral, e_prev, u = [0.0] * 3, [0.0] * 3, [0.0] * 3
    y_prev, d_f = [0.0] * 3, [0.0] * 3
    g_prev, gdot = [0.0] * 3, [0.0] * 3
    q = tuple(q0)
    out, reqs = [], []
    for j in range(n_exec * n_ticks):
        for i in range(3):
            x = 0.0 if (zero_read and j == 0) else body.w[i]
            x1, x2, y1, y2 = chain[i] if j > 0 else (x, x, x, x)
            y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2
            chain[i] = (x, x1, y, y1)
            y_tick[i] = y
        if j % n_ticks == 0:
            a = j // n_ticks
            out.append(rm.channels(q, list(body.w)))
            r_hold = rm.law(cfg, q, q_sp)
            yv = list(y_tick)
            if a == 0:
                u = [0.0] * 3
                y_prev = list(yv)
                g_prev = list(run_l5.euler_coupling(j_ff, yv))
                gdot = [0.0] * 3
            else:
                dt = su.dt_exec(a)
                for i in range(3):
                    alpha = su.alpha(dt, i)[0]
                    integral[i] += ki[i] * e_prev[i] * dt
                    e = r_hold[i] - yv[i]
                    d_raw = -kd[i] * (yv[i] - y_prev[i]) / dt
                    d_f[i] = d_f[i] + alpha * (d_raw - d_f[i]) if alpha != 1.0 else d_raw
                    u[i] = kp[i] * e + integral[i] + d_f[i]
                    e_prev[i] = e
                    y_prev[i] = yv[i]
                if ff in ("fw", "fw0"):
                    gg = run_l5.euler_coupling(j_ff, yv)
                    tf = 0.0 if ff == "fw0" else t_ff
                    for i in range(3):
                        raw = (gg[i] - g_prev[i]) / dt
                        gdot[i] = gdot[i] + (1 - math.exp(-dt / tf)) * (raw - gdot[i]) if tf > 0 else raw
                        u[i] += gg[i] + tau_ff * gdot[i]
                    g_prev = list(gg)
            if ff == "zoh":
                w = body.w
                c = run_l5.euler_coupling(J, w)
                wd = [(body.m[k] - c[k]) / J[k] for k in range(3)]
                jw = [J[k] * w[k] for k in range(3)]
                jwd = [J[k] * wd[k] for k in range(3)]
                cd = [x + y for x, y in zip(ea.cross(wd, jw), ea.cross(w, jwd))]
                u = [u[k] + c[k] + TAU * cd[k] for k in range(3)]
            reqs.append(list(u))
        body.u = list(u)
        d = body.advance()
        q = oracle.qmul(q, ea.REAL_QUAT_EXP(d))
        nq = math.sqrt(sum(c * c for c in q))
        q = tuple(c / nq for c in q)
    return out, reqs


if __name__ == "__main__":
    L = []
    which = sys.argv[1:] or ["ideal", "zoh", "fw", "fw0", "ideal+zr", "fw+zr", "none"]
    for v in which:
        ff, zr = v.split("+")[0], v.endswith("+zr")
        y, reqs = run(ff, zero_read=zr)
        res = against(design, y)
        tot = sum(r["viol"] for r in res.values())
        L.append(f"{v:<9} viol {tot:5d} | {fmt_res(res)} | max|u| first 300 {[round(max(abs(r[k]) for r in reqs[:300]), 3) for k in range(3)]}")
        print(L[-1], flush=True)
    (S / f"d_c8fw_{'_'.join(which)}.txt").write_text("\n".join(L) + "\n")
