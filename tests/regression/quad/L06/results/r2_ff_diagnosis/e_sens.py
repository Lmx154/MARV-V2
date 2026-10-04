"""(5) Sensitivity of R2's coupled trajectory: python fw (design world) vs tool F (firmware f32 path, same lag plant),
and python fw under tiny perturbations."""
import math, pickle, sys
sys.argv = [sys.argv[0]]
from d_c8fw import run, against, fmt_res, design, J, S
import d_c8fw as d
import common
from common import rm
base, _ = run("fw")
L = []
def cmp(tag, y):
    dif = [max(abs(y[n][c] - base[n][c]) for n in range(1, len(y))) for c in range(6)]
    gr = [(n, round(max(abs(y[n][c] - base[n][c]) for c in range(3, 6)), 6)) for n in (1, 2, 5, 20, 100, 300, 600, 1000, 1500, 2000, 2400)]
    L.append(f"{tag:<26} max|y - fw| {[f'{x:.2e}' for x in dif]}; rate diff by n {gr}")
    L.append(f"{'':<26} vs A: {fmt_res(against(design, y))}")
    print(L[-2]); print(L[-1], flush=True)
# tool F output (firmware path, lag plant, request-driven, seed_ff): read from the sim file of b_ladder
att = []
for ln in (S / "sim_F_lag_z0_ff_seed_req.txt").read_text().splitlines():
    v = ln.split()
    if v[0] == "A":
        att.append(([float(x) for x in v[4:8]], [float(x) for x in v[8:11]]))
cmp("tool F (fw f32 path)", [rm.channels(q, w) for q, w in att[:design.count]])
att = []
for ln in (S / "sim_M_lag_z0_ff_req_noseed.txt").read_text().splitlines():
    v = ln.split()
    if v[0] == "A":
        att.append(([float(x) for x in v[4:8]], [float(x) for x in v[8:11]]))
cmp("tool M (fw, no seed fix)", [rm.channels(q, w) for q, w in att[:design.count]])
y, _ = run("fw", j_ff=[j * (1 + 1e-6) for j in J]); cmp("fw, J_ff x (1+1e-6)", y)
d.w0 = tuple(w * (1 + 1e-7) for w in d.w0); y, _ = run("fw"); cmp("fw, w0 x (1+1e-7)", y)
(S / "e_sens.txt").write_text("\n".join(L) + "\n")
