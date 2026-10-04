"""(3) Where does the residual w_z excess come from? Firmware path in the tool's T3 design plant (no coupling) and the
lag+ideal-FF plant, against the design model's nominal member (recovery_model.member_run, s = t = 0)."""
import os
import pickle
import common
from common import S, load_run, sc, rm, l5s, SCEN, against, fmt_res, world_rotors

common.TOOL = S / "tool2"
os.cpu_count = lambda: 6
import test_t4_recovery as t4r  # noqa: E402

s1, params = load_run("on", 1)
card = sc.card_of(s1.run.world_path)
design = pickle.loads((S / "design_r2.pkl").read_bytes())
st = l5s.values(l5s.load(SCEN))["initial_state"]
q0, w0 = st["attitude_q_wxyz"], st["body_rates_frd_rad_s"]
rec = t4r.recorded_inputs()
su = rm.oracle.Setup(rec)
nom, _ = rm.member_run(su, 0.0, 0.0, tuple(q0), tuple(w0), design.count)
OFF = {"rate_ff_enable": ("i32", "0")}
L = []
pl = sc.plant_lines
for tag, mode, extra, ov in (("design_off", "design", [], OFF), ("G_lag_ideal", "lag_gyro", ["ideal_ff 0.033", "lag_on_request"], OFF),
                             ("F_lag_fwff", "lag_gyro", ["seed_ff", "lag_on_request"], None)):
    sc.plant_lines = lambda c, _e=extra: pl(c) + list(_e)
    att, rate = common.sim(s1, card, "c_" + tag, mode, "rest" if mode == "design" else world_rotors(s1), q0, w0, 0,
                           extra_overrides=ov)
    sc.plant_lines = pl
    y = [rm.channels(q, w) for q, w in att[:design.count]]
    d = [max(abs(y[n][c] - nom[n][c]) for n in range(1, design.count)) for c in range(6)]
    at = [max(range(1, design.count), key=lambda n: abs(y[n][c] - nom[n][c])) for c in range(6)]
    L.append(f"{tag:<12} max|sim - nominal| per channel {[f'{x:.2e}' for x in d]} at {at}")
    L.append(f"{'':<12} vs envelope A: {fmt_res(against(design, y))}")
    for n in (500, 1000, 1500, 2000, 2300, 3000, 5000):
        L.append(f"   n {n}: sim w {[round(v, 4) for v in y[n][3:]]} nominal {[round(v, 4) for v in nom[n][3:]]} "
                 f"A_z [{design.lo[5][n]:.4f}, {design.hi[5][n]:.4f}] err sim {[round(v, 4) for v in y[n][:3]]} nom {[round(v, 4) for v in nom[n][:3]]}")
print("\n".join(L))
(S / "c_wz.txt").write_text("\n".join(L) + "\n")
