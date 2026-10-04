"""(6) More counterfactuals at the gz-like plant (tool2), FF on unless stated, no harness zero read unless stated."""
import math, os, pickle, sys
import common
from common import S, load_run, sc, rm, l5s, SCEN, against, fmt_res, world_rotors, CARD
common.TOOL = S / "tool2"
sys.path.insert(0, str(common.ROOT / "tests/regression/quad/L05/gz"))
import test_t4_recovery as t4r  # noqa
import run_l5, mixer as cmix, gen_plant_config as gpc  # noqa
s1, params = load_run("on", 1)
card = sc.card_of(s1.run.world_path)
design = pickle.loads((S / "design_r2.pkl").read_bytes())
vals = l5s.values(l5s.load(SCEN)); st = vals["initial_state"]
q0, w0 = st["attitude_q_wxyz"], st["body_rates_frd_rad_s"]
# centred collective: maximise the smallest thrust headroom to [f_lo, f_hi]
tr = run_l5.steady_tumble(CARD, vals)
k = card["k"]; f_lo, f_hi = tr["thrust_range"]
card_doc, profile = gpc.load_linted(CARD, run_l5.ROOT)
M, _, _ = cmix.mixer_matrix(card_doc, CARD)
dvec = [tr["thrusts"][i] - M[i][0] * tr["collective"] for i in range(4)]
c_lo = max((f_lo - dvec[i]) / M[i][0] for i in range(4)); c_hi = min((f_hi - dvec[i]) / M[i][0] for i in range(4))
c_mid = 0.5 * (c_lo + c_hi)
T_mid = [M[i][0] * c_mid + dvec[i] for i in range(4)]
print(f"c* {tr['collective']:.4f} N thrusts {[round(x, 4) for x in tr['thrusts']]}; c range [{c_lo:.4f}, {c_hi:.4f}]; centred c {c_mid:.4f} "
      f"thrusts {[round(x, 4) for x in T_mid]} headroom {min(min(t - f_lo, f_hi - t) for t in T_mid):.4f} N")
rot_mid = " ".join(repr(math.sqrt(t / k)) for t in T_mid)
BIG = {"rotor_speed_max": ("f32", "6000.0")}
OFF = {"rate_ff_enable": ("i32", "0")}
cases = [("H2_cont_ideal_bigfmax", "cont_gyro", 0, ["ideal_ff 0.033"], dict(BIG, **OFF), None),
         ("D2_cont_bigfmax_Tff0", "cont_gyro", 0, ["seed_ff"], dict(BIG, rate_ff_filter_tau=("f32", "0.0")), None),
         ("H_qg_ideal_sat", "quant_gyro", 0, ["ideal_ff 0.033"], OFF, None)]
L = []
pl = sc.plant_lines
for tag, mode, fz, extra, ov, rot in cases:
    sc.plant_lines = lambda c, _e=extra: pl(c) + list(_e)
    att, rate = common.sim(s1, card, tag, mode, rot or world_rotors(s1), q0, w0, fz, extra_overrides=ov)
    sc.plant_lines = pl
    y = [rm.channels(q, w) for q, w in att[:design.count]]
    res = against(design, y)
    fl = [sum(r["flag"][a] for r in rate) for a in range(3)]
    L.append(f"{tag:<20} viol {sum(r['viol'] for r in res.values()):5d} | {fmt_res(res)} | flags {fl}; DShot max {max(max(r['d']) for r in rate)}")
    print(L[-1], flush=True)
(S / "h_more2.txt").write_text("\n".join(L) + "\n")
