"""(2) Counterfactual ladder around the firmware composition path (tool2 = recovery_cause_tool + scratch modes), FF on,
against R2's envelope A (test_t4_recovery.make_design, E = 0)."""
import math
import pickle
import sys
import common
from common import S, load_run, sc, rm, l5s, SCEN, against, fmt_res, world_rotors

common.TOOL = S / "tool2"
import os
os.cpu_count = lambda: 6
import test_t4_recovery as t4r  # noqa: E402

s1, params = load_run("on", 1)
s2, _ = load_run("on", 2)
card = sc.card_of(s1.run.world_path)
dp = S / "design_r2.pkl"
if dp.exists():
    design = pickle.loads(dp.read_bytes())
else:
    live = {k: params[k] for k in t4r.recorded_inputs() if k in params}
    design = t4r.make_design(t4r.R2, live)
    dp.write_bytes(pickle.dumps(design))
st = l5s.values(l5s.load(SCEN))["initial_state"]
q0, w0 = st["attitude_q_wxyz"], st["body_rates_frd_rad_s"]
rot = world_rotors(s1)
y_gz1 = t4r.series(s1, design.count)
y_gz2 = t4r.series(s2, design.count)
E = [[abs(a - b) for a, b in zip(x, y)] for x, y in zip(y_gz1, y_gz2)]
L = [f"rotors {rot}; N {design.count}; F+Q {[round(f + q, 5) for f, q in zip(design.F, design.Q)]}",
     "gz m=1 FF on (test predicate with E): " + fmt_res(against(design, y_gz1, E=E)),
     "gz m=1 FF on (E = 0):                 " + fmt_res(against(design, y_gz1))]
OFF = {"rate_ff_enable": ("i32", "0")}
cases = [
    # tag, mode, first_zero, extra lines flag, overrides
    ("A_qg_z1_ff", "quant_gyro", 1, [], None),
    ("B_qg_z0_ff", "quant_gyro", 0, [], None),
    ("C_qg_z0_ff_seed", "quant_gyro", 0, ["seed_ff"], None),
    ("D_cg_z0_ff_seed", "cont_gyro", 0, ["seed_ff"], None),
    ("E_lag_z0_ff_seed_ach", "lag_gyro", 0, ["seed_ff"], None),
    ("F_lag_z0_ff_seed_req", "lag_gyro", 0, ["seed_ff", "lag_on_request"], None),
    ("G_lag_z0_idealff_req", "lag_gyro", 0, ["ideal_ff 0.033", "lag_on_request"], OFF),
    ("H_qg_z0_idealff", "quant_gyro", 0, ["ideal_ff 0.033"], OFF),
    ("I_qg_z0_off", "quant_gyro", 0, [], OFF),
    ("J_qg_z1_off", "quant_gyro", 1, [], OFF),
    ("K_lag_z1_ff_req", "lag_gyro", 1, ["lag_on_request"], None),
    ("M_lag_z0_ff_req_noseed", "lag_gyro", 0, ["lag_on_request"], None),
]
only = sys.argv[1:]
for tag, mode, fz, extra, ov in cases:
    if only and tag.split("_")[0] not in only:
        continue
    orig = common.sim.__defaults__
    # write extra input records by wrapping plant_lines
    pl = sc.plant_lines
    sc.plant_lines = lambda c, _e=extra, _pl=pl: _pl(c) + list(_e)
    att, rate = common.sim(s1, card, tag, mode, rot, q0, w0, fz, extra_overrides=ov)
    sc.plant_lines = pl
    y = [rm.channels(q, w) for q, w in att[:design.count]]
    dgz = max(max(abs(a - b) for a, b in zip(u, v)) for u, v in zip(y[1:], y_gz1[1:]))
    fl = [sum(r["flag"][a] for r in rate) for a in range(3)]
    d4 = sum(1 for r in rate if 2047 in r["d"])
    L.append(f"{tag:<24} {fmt_res(against(design, y))} | max|sim-gz| {dgz:.4f}; flags {fl}; execs with a 2047 {d4}; "
             f"max|req| first300 {[round(max(abs(r['req'][a]) for r in rate[:300]), 3) for a in range(3)]}")
    print(L[-1], flush=True)
print("\n".join(L))
out = S / ("b_ladder.txt" if not only else f"b_ladder_{'_'.join(only)}.txt")
out.write_text("\n".join(L) + "\n")
