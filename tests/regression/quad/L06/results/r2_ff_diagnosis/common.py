"""Scratch helpers (diagnosis only): load W4's FF-on R2 gz logs, run recovery_cause_tool replay/sim."""
import math
import os
import subprocess
import sys
import types
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[5]
S = Path(os.environ["MARV_R2FF_WORK"]).resolve()  # the regenerate work dir: tools, sim and replay files, the pickle
W4 = Path(os.environ.get("MARV_R2FF_GZ", S / "gz")).resolve()  # the gz logs: recover_tumble_ff_{off,on}_*/
for p in (ROOT / "tools" / "sim", ROOT / "tools" / "card", ROOT / "tests" / "regression" / "quad" / "L05" / "gz",
          ROOT / "tests" / "regression" / "quad" / "L05" / "results" / "step_cause"):
    sys.path.insert(0, str(p))
import l5_scenario as l5s  # noqa: E402
import lockstep_log  # noqa: E402
import recovery_model as rm  # noqa: E402
import run_l4  # noqa: E402
import run_l5  # noqa: E402
import step_cause as sc  # noqa: E402

TOOL = S / "recovery_cause_tool"
CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
SCEN = ROOT / "scenarios" / "quad" / "L05" / "recover_tumble.yaml"
FF_ON = {"rate_ff_enable": ("i32", "1")}


def load_run(variant, m):
    d = next((W4).glob(f"recover_tumble_ff_{variant}_*"))
    log = lockstep_log.read(next(d.glob(f"*_m{m}_seed1.bin")))
    sdf = next(d.glob(f"*_m{m}_seed1.sdf"))
    _, defaults = run_l5.build_parameters(run_l5.DEFAULT_PLUGIN_DIR)
    params = run_l4.read_param_defaults(defaults)
    p = run_l5.plan(l5s.load(SCEN), params, CARD)
    ov = p.overrides()
    if variant == "on":
        ov.update(FF_ON)
    ex = run_l5.executions(log, p, m)
    s = types.SimpleNamespace(run=types.SimpleNamespace(log=log, world_path=str(sdf), m=m), plan=p, overrides=ov,
                              executions=ex)
    return s, params


def run_tool(args):
    r = subprocess.run([str(TOOL), *map(str, args)], capture_output=True, text=True, check=False)
    if r.returncode:
        sys.exit(f"tool {args}: {r.stderr}")


def sim(s, card, tag, mode, rotors, q0, w0, first_zero, extra_overrides=None, sub=1):
    ov = dict(s.overrides)
    ov.update(extra_overrides or {})
    lines = [f"override {k} {t} {v}" for k, (t, v) in ov.items()] + sc.plant_lines(card)
    lines += [f"ticks {len(s.run.log['ticks'])}", "init " + " ".join(repr(float(x)) for x in (*q0, *w0)),
              f"rotors {rotors}", f"first_read_zero {first_zero}"]
    inp, out = S / f"sim_in_{tag}.txt", S / f"sim_{tag}.txt"
    inp.write_text("\n".join(lines) + "\n")
    run_tool(["sim", inp, mode, sub, out])
    att, rate = [], []
    for ln in out.read_text().splitlines():
        v = ln.split()
        if v[0] == "A":
            att.append(([float(x) for x in v[4:8]], [float(x) for x in v[8:11]]))
        else:
            rate.append({"k": int(v[1]), "tick": int(v[2]), "d": [int(x) for x in v[3:7]],
                         "req": [float(x) for x in v[7:10]], "ach": [float(x) for x in v[10:13]],
                         "flag": [int(x) for x in v[13:16]]})
    return att, rate


def world_rotors(s):
    import xml.etree.ElementTree as ET
    e = ET.parse(s.run.world_path).getroot().find(".//plugin[@name='marv::gz::Lockstep']/initial_rotor_speed_rad_s")
    return e.text.strip() if e is not None else "rest"


def against(design, y, chs=rm.CHANNELS, E=None):
    """Excess over envelope (out - (E + F + Q)) per channel; E per execution list or None (0)."""
    res = {}
    for ci, c in enumerate(chs):
        worst, viol, first = (-math.inf, None, None, None), 0, None
        for n in range(min(design.count, len(y))):
            if n == 0 and c.startswith("w_"):
                continue
            v = y[n][ci]
            out = max(design.lo[ci][n] - v, v - design.hi[ci][n])
            e = E[n][ci] if E is not None else 0.0
            sl = out - (e + design.F[ci] + design.Q[ci])
            if sl > 0:
                viol += 1
                if first is None:
                    first = n
            if sl > worst[0]:
                worst = (sl, n, v, (design.lo[ci][n], design.hi[ci][n]))
        res[c] = {"excess": worst[0], "n": worst[1], "y": worst[2], "env": worst[3], "viol": viol, "first": first}
    return res


def fmt_res(res):
    return "; ".join(f"{c} {r['excess']:+.4f}@{r['n']} (viol {r['viol']}, first {r['first']})" for c, r in res.items())
