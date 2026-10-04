"""(1) W4's FF-on R2 gz run (m = 1) replayed through the composition path: saturation, headroom, start-up."""
import math
from common import *  # noqa: F401,F403
from common import S, load_run, sc, run_l4, run_l5, CARD, l5s, SCEN

out = []
for variant in ("on", "off"):
    s, params = load_run(variant, 1)
    card = sc.card_of(s.run.world_path)
    rows, bad = sc.replay(S / "recovery_cause_tool", s, card, S)
    (S / f"replay_rows_{variant}.txt").write_text((S / "replay_out.txt").read_text())
    out.append(f"== FF {variant}: replay DShot mismatches {len(bad)} of {len(rows)} rate executions")
    log = s.run.log
    k = card["k"]
    fmin, fmax = k * params["idle_speed"] ** 2, k * params["rotor_speed_max"] ** 2
    out.append(f"f_min {fmin:.6f} N (idle {params['idle_speed']}), f_max {fmax:.4f} N; card wmin {card['wmin']} wmax {card['wmax']}")
    for t in log["ticks"][:4]:
        g = run_l4.IMU.unpack(t["imu"])[:3]
        out.append(f"  tick {t['tick']}: gyro {[round(x, 4) for x in g]}, dshot {t['dshot']}, rotor {[round(x, 2) for x in t['rotor_speed']]}")
    # rows: k tick t_us sp req ach flags d qt
    for lim in (300, 1000, len(rows)):
        rr = rows[:lim]
        fl = [sum(int(r[f"flag_{a}"]) for r in rr) for a in "xyz"]
        sratio = []
        for r in rr:
            for a in "xyz":
                if abs(r[f"req_{a}"]) > 1e-9:
                    sratio.append(r[f"ach_{a}"] / r[f"req_{a}"])
        dmin = [min(int(r[f"d{i}"]) for r in rr) for i in range(1, 5)]
        dmax = [max(int(r[f"d{i}"]) for r in rr) for i in range(1, 5)]
        at48 = [sum(int(r[f"d{i}"]) == 48 for r in rr) for i in range(1, 5)]
        at2047 = [sum(int(r[f"d{i}"]) == 2047 for r in rr) for i in range(1, 5)]
        out.append(f" executions 0..{lim - 1}: flags x/y/z {fl}; min ach/req {min(sratio):.6f}; DShot min {dmin} max {dmax}; "
                   f"#at 48 {at48}; #at 2047 {at2047}")
    first2047 = next((int(r["k"]) for r in rows if max(int(r[f"d{i}"]) for i in range(1, 5)) == 2047), None)
    out.append(f" first execution with a motor at 2047: {first2047}")
    if first2047 is not None:
        r = rows[first2047]
        out.append(f"  there: req {[round(r['req_' + a], 4) for a in 'xyz']}, d {[int(r[f'd{i}']) for i in range(1, 5)]}")
    for kk in (0, 1, 2, 3, 4, 6, 10, 20, 40, 80, 160, 263, 400):
        r = rows[kk]
        tr = s.executions[kk]["truth"]["omega_frd"]
        out.append(f"  k {kk}: sp {[round(r['sp_' + a], 3) for a in 'xyz']} req {[round(r['req_' + a], 4) for a in 'xyz']} "
                   f"d {[int(r[f'd{i}']) for i in range(1, 5)]} static torque {[round(r['qt_' + a], 4) for a in 'xyz']} "
                   f"truth w {[round(x, 3) for x in tr]}")
    # peak |request| over first 300
    out.append(f" max |req| first 300: {[round(max(abs(r['req_' + a]) for r in rows[:300]), 4) for a in 'xyz']}")
print("\n".join(out))
(S / "a_replay.txt").write_text("\n".join(out) + "\n")
