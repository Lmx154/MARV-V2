#!/usr/bin/env python3
"""Decision 0058 evidence: the acro roll step and the roll flick of the quad_fc flight composition in gz SIL, before and
after the reference feed-forward. This is evidence, not a test. Nothing here is tuned, and the card values it flies are
INFERRED / PLACEHOLDER (vehicles/marv_7in_fc3v2.yaml).

  uv run python tests/regression/quad/L10/results/rate_ff/rate_ff_sil.py --out-dir <dir> [--plugin-dir <dir>]
                                                                          [--runs before|after|both]

It needs `cmake --preset host-gz-quadfc && cmake --build --preset host-gz-quadfc` and gz-sim 8. The world is
tools/sim/run_quad_fc.py's: the real card, the FC3v2 tick, the L2 site, m = 1, the model gyro, <launch_pad/>, the N-tick
command delay and the same frame period. Only the stick script and the analysis differ.
  after   the set as built: marv_params_fc3v2 with rate_ref_ff_enable 1 and rule step 14's tau_ref.
  before  the same image with <sil_override> rate_ref_ff_enable = 0 and rate_tau_ref_{roll,pitch} = decision 0005's
          tau_ref for this card (TAU_REF_0005, rate_lead.tau_ref_rule(design, False)). With the feed-forward off the law is
          bit-identical (decision 0058 I2), so this is the pre-change loop.

The script runs in SIL time. Every duration and stick is a labelled scenario value. The angle switch is HIGH for angle
mode and LOW for acro.
  ready, arm_idle             as run_quad_fc (angle)
  hover      HOVER_S          angle, the derived hover stick (lift-off from the pad)
  acro_0     ACRO_SETTLE_S    acro, sticks centred
  step       STEP_S           acro, roll stick for STEP_RATE (the architect's 100 deg/s step)
  step_back  STEP_S           acro, the opposite stick (returns the attitude near level; not analysed)
  acro_1     ACRO_SETTLE_S    acro, centred
  recover_0  RECOVER_S        angle (re-levels)
  acro_2     ACRO_SETTLE_S    acro, centred
  flick      FLICK_S          acro, roll stick for FLICK_RATE (the architect's 200 deg/s, 175 ms flick)
  coast      COAST_S          acro, centred (the flick's stop)
  recover_1  RECOVER_S        angle
  disarmed   TAIL_US          arm off
The commanded rate is the rates curve (fw/stick_rates: r = c (a - a^2) + m a^2 at expo 0) of the stick the firmware
received (the log's MANUAL record), with c = rates_centre_roll from the quad_fc register and m = rate_max_roll from the
FC3v2 scenario register. The response is the truth body roll rate (STEP body_omega_frd). Metrics, with the architect's
definitions (scratchpad arch-rate/compare.py):
  step   t63: from the tick the step stick arrives until the rate first reaches 0.632 of the command. overshoot: the
         largest rate in the step window over the command, minus 1.
  flick  reach: the largest rate after the flick stick arrives, over the command. Peak delay: from release (the tick the
         centred stick arrives) to that peak. To 10 % (release): from release until the rate first falls to 10 % of the
         peak. To 10 % (peak): from the peak to the same point (compare.py's "to 10% of pk").
"""

from __future__ import annotations

import argparse
import math
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[5]
sys.path.insert(0, str(ROOT / "tools" / "sim"))
sys.path.insert(0, str(ROOT / "tools" / "card"))
import gen_world  # noqa: E402
import run_quad_fc as rq  # noqa: E402
import run_scenario  # noqa: E402

HOVER_S = 3.0          # labelled scenario value: angle-mode hover after the raise (lift-off and settle)
ACRO_SETTLE_S = 0.5    # labelled scenario value: acro with centred sticks before an event
STEP_S = 0.6           # labelled scenario value: the step's length (several t63 of either loop)
RECOVER_S = 3.0        # labelled scenario value: angle mode re-levels between the events
FLICK_S = 0.175        # the architect's flick length (memory quad-m4/architect-rate-response-2026-10-18)
COAST_S = 1.0          # labelled scenario value: acro after the flick (longer than the architect's 610 ms stop)
STEP_RATE = math.radians(100.0)   # the architect's step command, rad/s
FLICK_RATE = math.radians(200.0)  # the architect's flick command, rad/s
FRAC_63 = 1.0 - math.exp(-1.0)    # t63: the first-order time constant's fraction
FRAC_STOP = 0.1                   # the architect's "to 10 %"
TAU_REF_0005 = 0.21945825219154358  # s, rate_lead.tau_ref_rule(design, False) for roll and pitch on this card (rate_ff/README.md)
TRACE_DECIMATE = 8     # labelled analysis value: one trace row per 8 ticks (about 940 Hz)
TRACE_PRE_S, TRACE_POST_S = 0.1, 1.0  # labelled analysis value: the trace window around each event


def stick_for(rate):
    """The roll stick whose rates-curve output is `rate` (expo 0): c a + (m - c) a^2 = rate, a in [0, 1]."""
    c, m = rq.register_value("rates_centre_roll"), rq.scenario_value("rate_max_roll")
    assert rq.register_value("rates_expo_roll") == 0
    return (-c + math.sqrt(c * c + 4 * (m - c) * rate)) / (2 * (m - c))


def curve(s):
    c, m = rq.register_value("rates_centre_roll"), rq.scenario_value("rate_max_roll")
    a = abs(s)
    return math.copysign(c * (a - a * a) + m * a * a, s)


def phases(hover):
    cal, recovery = rq.register_value("gyro_cal_duration_us"), rq.register_value("failsafe_recovery_us")
    hv, lo, hi = hover.stick_f32, rq.SWITCH_LOW, rq.SWITCH_HIGH
    s_step, s_flick = stick_for(STEP_RATE), stick_for(FLICK_RATE)
    spec = [("ready", cal + recovery + rq.MARGIN_US, lo, hi, 0.0, 0.0), ("arm_idle", rq.ARM_IDLE_US, hi, hi, 0.0, 0.0),
            ("hover", rq.us(HOVER_S), hi, hi, 0.0, hv), ("acro_0", rq.us(ACRO_SETTLE_S), hi, lo, 0.0, hv),
            ("step", rq.us(STEP_S), hi, lo, s_step, hv), ("step_back", rq.us(STEP_S), hi, lo, -s_step, hv),
            ("acro_1", rq.us(ACRO_SETTLE_S), hi, lo, 0.0, hv), ("recover_0", rq.us(RECOVER_S), hi, hi, 0.0, hv),
            ("acro_2", rq.us(ACRO_SETTLE_S), hi, lo, 0.0, hv), ("flick", rq.us(FLICK_S), hi, lo, s_flick, hv),
            ("coast", rq.us(COAST_S), hi, lo, 0.0, hv), ("recover_1", rq.us(RECOVER_S), hi, hi, 0.0, hv),
            ("disarmed", rq.TAIL_US, lo, hi, 0.0, hv)]
    out, t = [], 0
    for name, dur, arm, angle, roll, thr in spec:
        out.append({"name": name, "start_us": t, "duration_us": dur, "arm": arm, "angle": angle, "roll": roll, "throttle": thr})
        t += dur
    return out


def script_xml(ph):
    segs = "".join(
        f"<segment><start_us>{p['start_us']}</start_us><duration_us>{p['duration_us']}</duration_us><link>on</link>"
        f"<roll>{p['roll']!r}</roll><pitch>0.0</pitch><yaw>0.0</yaw><throttle>{p['throttle']!r}</throttle>"
        f"<arm>{p['arm']!r}</arm><angle>{p['angle']!r}</angle></segment>" for p in ph)
    return (f"<manual_control_script><frame_period_us>{rq.FRAME_PERIOD_US}</frame_period_us><arm_channel>{rq.ARM_CHANNEL}"
            f"</arm_channel><angle_channel>{rq.ANGLE_CHANNEL}</angle_channel>{segs}</manual_control_script>")


def sdf_edit(ph, overrides):
    def edit(text):
        text = re.sub(r'\s*<sil_override param="ol_dshot_m\d" type="i32">\d+</sil_override>', "", text)
        add = "<launch_pad/>\n" + script_xml(ph) + "\n"
        for name, typ, value in overrides:
            add += f'<sil_override param="{name}" type="{typ}">{value}</sil_override>\n'
        assert text.count("<gyro_source>") == 1
        return text.replace("<gyro_source>", add + "<gyro_source>", 1)
    return edit


OVERRIDES = {"after": (), "before": (("rate_ref_ff_enable", "i32", "0"), ("rate_tau_ref_roll", "f32", repr(TAU_REF_0005)),
                                    ("rate_tau_ref_pitch", "f32", repr(TAU_REF_0005)))}


def series(log):
    steps, ticks, manuals = log["steps"], log["ticks"], log["manuals"]
    assert len(steps) == len(ticks)
    t, w, stick, mi = [], [], [], -1
    for j, (st, tk) in enumerate(zip(steps, ticks)):
        while mi + 1 < len(manuals) and manuals[mi + 1]["tick"] <= j:
            mi += 1
        t.append(tk["sil_t_us"])
        w.append(st["body_omega_frd"][0])
        stick.append(manuals[mi]["roll"] if mi >= 0 else 0.0)
    return t, w, stick


def phase_of(ph, name):
    return next(p for p in ph if p["name"] == name)


def first(idx, pred):
    return next((i for i in idx if pred(i)), None)


def analyse(label, t, w, stick, ph):
    out = {}
    n = len(t)
    step, back = phase_of(ph, "step"), phase_of(ph, "step_back")
    i0 = first(range(n), lambda i: t[i] >= step["start_us"] and stick[i] != 0.0)
    cmd = curve(stick[i0])
    i_end = first(range(i0, n), lambda i: t[i] >= back["start_us"] and stick[i] != stick[i0])
    i63 = first(range(i0, i_end), lambda i: w[i] >= FRAC_63 * cmd)
    peak = max(w[i0:i_end])
    out["step"] = {"cmd": cmd, "stick": stick[i0], "t0_us": t[i0], "t63_s": None if i63 is None else (t[i63] - t[i0]) * 1e-6,
                   "overshoot": peak / cmd - 1.0, "end_rate": w[i_end - 1] / cmd}
    flick, coast = phase_of(ph, "flick"), phase_of(ph, "coast")
    f0 = first(range(n), lambda i: t[i] >= flick["start_us"] and stick[i] != 0.0)
    fcmd = curve(stick[f0])
    rel = first(range(f0, n), lambda i: stick[i] == 0.0)
    f_end = first(range(rel, n), lambda i: t[i] >= coast["start_us"] + coast["duration_us"])
    ipk = max(range(f0, f_end), key=lambda i: w[i])
    i10 = first(range(ipk, f_end), lambda i: w[i] <= FRAC_STOP * w[ipk])
    out["flick"] = {"cmd": fcmd, "stick": stick[f0], "t0_us": t[f0], "hold_s": (t[rel] - t[f0]) * 1e-6,
                    "reach": w[ipk] / fcmd, "peak_after_release_s": (t[ipk] - t[rel]) * 1e-6,
                    "to10_release_s": None if i10 is None else (t[i10] - t[rel]) * 1e-6,
                    "to10_peak_s": None if i10 is None else (t[i10] - t[ipk]) * 1e-6,
                    "undershoot": min(w[ipk:f_end]) / fcmd}
    out["windows"] = [(i0, i_end), (f0, f_end)]
    return out


def fmt(x, scale=1.0, unit=""):
    return "n/a" if x is None else f"{x * scale:.1f}{unit}"


def report(label, res, log_sha, overrides):
    s, f = res["step"], res["flick"]
    lines = [f"run {label}: overrides {list(overrides) or 'none'}; log sha256 {log_sha}",
             f"  step  stick {s['stick']!r} -> command {math.degrees(s['cmd']):.2f} deg/s at {s['t0_us']} us: t63 "
             f"{fmt(s['t63_s'], 1e3, ' ms')}, overshoot {s['overshoot'] * 100:.1f} %, rate/command at the window's end "
             f"{s['end_rate']:.3f}",
             f"  flick stick {f['stick']!r} -> command {math.degrees(f['cmd']):.2f} deg/s held {f['hold_s'] * 1e3:.1f} ms at "
             f"{f['t0_us']} us: reach {f['reach'] * 100:.1f} % of command, peak {fmt(f['peak_after_release_s'], 1e3, ' ms')} "
             f"after release, to 10 % of peak {fmt(f['to10_release_s'], 1e3, ' ms')} after release "
             f"({fmt(f['to10_peak_s'], 1e3, ' ms')} after the peak), undershoot {f['undershoot'] * 100:.1f} % of command"]
    return "\n".join(lines)


def write_trace(path, t, w, stick, windows):
    rows = ["t_us,stick_roll,command_rad_s,omega_x_rad_s"]
    for i0, i1 in windows:
        a = max(0, i0 - int(TRACE_PRE_S / (t[1] - t[0]) * 1e6))
        for i in range(a, i1, TRACE_DECIMATE):
            rows.append(f"{t[i]},{stick[i]!r},{curve(stick[i])!r},{w[i]!r}")
    Path(path).write_text("\n".join(rows) + "\n", encoding="utf-8")


def run(label, out_dir, plugin_dir):
    out_dir = Path(out_dir) / label
    out_dir.mkdir(parents=True, exist_ok=True)
    hover = rq.derive_hover()
    ph = phases(hover)
    scenario, num, den = rq.fc3_scenario(out_dir)
    iterations =-(-(ph[-1]["start_us"] + ph[-1]["duration_us"]) * den // num)
    sensors = gen_world.SensorSet(gyro="model", bias_signs=(0,) * 6)
    r = run_scenario.run(rq.CARD, scenario, rq.SEED, rq.M, "test", "lo", out_dir, plugin_dir, sdf_edit(ph, OVERRIDES[label]),
                         iterations=iterations, timeout_s=rq.TIMEOUT_S, sensors=sensors,
                         command_delay_ticks=rq.COMMAND_DELAY_TICKS)
    t, w, stick = series(r.log)
    res = analyse(label, t, w, stick, ph)
    stem = Path(r.log_path).with_suffix("")
    text = report(label, res, r.log_sha256, OVERRIDES[label])
    Path(f"{stem}.analysis.txt").write_text(text + "\n", encoding="utf-8")
    write_trace(f"{stem}.traces.csv", t, w, stick, res["windows"])
    print(text, flush=True)
    return res


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--plugin-dir", default=str(rq.PLUGIN_DIR))
    ap.add_argument("--runs", choices=("before", "after", "both"), default="both")
    args = ap.parse_args(argv)
    for label in (("before", "after") if args.runs == "both" else (args.runs,)):
        run(label, args.out_dir, args.plugin_dir)
    return 0


if __name__ == "__main__":
    sys.exit(main())
