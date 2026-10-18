#!/usr/bin/env python3
"""Check 4 of the configuration validator (ground-segment unit 9, decision 0053 G7): a quad_fc Gazebo SIL run on the config's
card with the existing behaviour predicates, the existing-predicates form (lead, 2026-10-18, recorded in decision 0053; Luis may
choose the stricter S/B form with a card-derived deadline later).

How the candidate is loaded (never through the store's binding path, so no report id is needed): one <sil_override> per record
of the candidate table, f32 as the repr of the exact double (the plugin parses it with std::from_chars, so the value is the
record's binary32 bit for bit), i32 as its integer; the world has no <config_dir> and no <link_udp_port>, so the SIL's store
boot returns the compiled table with status Ok and marv_sil_init applies the overrides on top of it before params_init and
composition::init (fw/sil/src/marv_sil.cpp). A config the composition's validator rejects makes init panic: the run is refused
and the check fails. The overrides carry the values only (provenance there is "sil-override"); checks 1 and 2 cover the
records, so a locked parameter cannot differ here.

The run: tools/sim/run_quad_fc.py's S2e world and stick script (ready, arm, hover hold, roll step and centre, pitch step and
centre, dip, hold, disarm; step stick STEP_STICK, the derived hover stick of the card), at the candidate's tick period, its arm
and angle channels and the gyro calibration and link recovery times of the table, the launch pad, the model gyro at the
nominal corner and N = COMMAND_DELAY_TICKS. Two gz processes, one after the other: m = 1 (the predicates) and m = 2 (E, the
|m1 - m2| of tools/sim/t4_rule.py). The quantities are read at every second tick (the rate and attitude executions of the
FC3v2 set, rate_loop_divisor 2, and a host step of the m = 2 run), from the STEP records' true body attitude.

The predicates (no new threshold):
  arm       the plumbing test's (tests/regression/quad/L10/gz/test_quad_fc_plumbing.py test_switch_raised_after_calibration_
            and_recovery_arms): DShot stop on every tick before the raised frame, a throttle command (>= 48) on every motor from
            a tick within the arm phase after it, on every tick until the disarm frame, and no tick neither stop nor throttle.
  hover     B (t4_rule.scenario_checks, decisions 0029 and 0031) on the true tilt over the hold phase, role HELD: |tilt(n)| <=
            |tilt(n0)| + s Delta + E_max at every execution, s = t4_settle_fraction, Delta the largest |error| of the moving
            segments (the roll and pitch steps and centres), E_max the largest |tilt(m = 1) - tilt(m = 2)| of the hold.
  roll, pitch  T (t4_rule.shrinks) on the stepped axis's error (setpoint - true tilt component in the heading frame,
            setpoint = stick x the table's angle_tilt_max) from the first execution with the step's frame to the end of the
            step, and T on the true tilt over the centre phase from the first execution with the centred frame.
  disarm    DShot stop on every motor on every tick from the disarm frame + SLACK_US on (SLACK_US: the plumbing test's
            allowance on a failsafe edge, test_quad_fc_plumbing.py:67), and the run has such ticks.
  finite    every logged body state (position, velocity, attitude, body rate) of both runs is finite; both gz runs end
            cleanly with every step logged (run_scenario.check_exit).
Thresholds come from the budget and must equal the image's compiled values (t4_settle_fraction); the candidate never sets
its own bound.
"""

from __future__ import annotations

import math
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "tools" / "sim"))
sys.path.insert(0, str(ROOT / "tools" / "card"))
sys.path.insert(0, str(ROOT / "tools" / "config"))
import gen_world  # noqa: E402
import mcfg  # noqa: E402
import run_quad_fc as rq  # noqa: E402
import run_scenario  # noqa: E402
import schema  # noqa: E402
import t4_rule  # noqa: E402

SLACK_US = 10_000  # tests/regression/quad/L10/gz/test_quad_fc_plumbing.py:67 SLACK_US: the allowance on a failsafe edge
STEP_STICK = rq.STEP_STICK
DSHOT_THROTTLE_MIN = rq.DSHOT_MIN
MEMBERS = (1, 2)
UNIT = "rad"
ROLES = {"roll_step": t4_rule.STEPPED, "roll_centre": t4_rule.STEPPED, "pitch_step": t4_rule.STEPPED,
         "pitch_centre": t4_rule.STEPPED, "hold": t4_rule.HELD}


class CheckError(Exception):
    pass


def override_xml(manifest, records):
    """One <sil_override> per record, bit-exact (module docstring)."""
    out = []
    for (name, e), r in zip(manifest["params"].items(), records, strict=True):
        if r.type == "f32":
            v = mcfg.f32_of(r.value_bits)
            if not math.isfinite(v):
                raise CheckError(f"{name}: value is not finite")
            text = repr(v)
        else:
            text = str(mcfg.i32_of(r.value_bits))
        out.append(f'<sil_override param="{name}" type="{r.type}">{text}</sil_override>')
    return "\n".join(out) + "\n"


def phases_for(values, hover):
    ready = int(values["gyro_cal_duration_us"]) + int(values["failsafe_recovery_us"]) + rq.MARGIN_US
    hv = hover.stick_f32
    spec = [("ready", ready, rq.SWITCH_LOW, 0.0, 0.0, 0.0),
            ("arm_idle", rq.ARM_IDLE_US, rq.SWITCH_HIGH, 0.0, 0.0, 0.0),
            ("hold", rq.us(rq.HOLD_S), rq.SWITCH_HIGH, 0.0, 0.0, hv),
            ("roll_step", rq.us(rq.STEP_S), rq.SWITCH_HIGH, STEP_STICK, 0.0, hv),
            ("roll_centre", rq.us(rq.CENTRE_S), rq.SWITCH_HIGH, 0.0, 0.0, hv),
            ("pitch_step", rq.us(rq.STEP_S), rq.SWITCH_HIGH, 0.0, STEP_STICK, hv),
            ("pitch_centre", rq.us(rq.CENTRE_S), rq.SWITCH_HIGH, 0.0, 0.0, hv),
            ("dip", rq.us(rq.DIP_S), rq.SWITCH_HIGH, 0.0, 0.0, 0.0),
            ("hold2", rq.us(rq.HOLD2_S), rq.SWITCH_HIGH, 0.0, 0.0, hv),
            ("disarmed", rq.TAIL_US, rq.SWITCH_LOW, 0.0, 0.0, hv)]
    phases, t = [], 0
    for name, dur, arm, roll, pitch, thr in spec:
        phases.append(rq.Phase(name, t, dur, arm, roll, pitch, thr))
        t += dur
    return phases


def script_xml(phases, arm_channel, angle_channel):
    segs = "".join(
        f"<segment><start_us>{p.start_us}</start_us><duration_us>{p.duration_us}</duration_us><link>on</link>"
        f"<roll>{p.roll!r}</roll><pitch>{p.pitch!r}</pitch><yaw>0.0</yaw><throttle>{p.throttle!r}</throttle>"
        f"<arm>{p.arm!r}</arm><angle>{rq.SWITCH_HIGH!r}</angle></segment>" for p in phases)
    return (f"<manual_control_script><frame_period_us>{rq.FRAME_PERIOD_US}</frame_period_us><arm_channel>{arm_channel}"
            f"</arm_channel><angle_channel>{angle_channel}</angle_channel>{segs}</manual_control_script>")


def sdf_edit(script, overrides):
    def edit(text):
        text = re.sub(r'\s*<sil_override param="[^"]*" type="(i32|f32)">[^<]*</sil_override>', "", text)
        if "<config_dir>" in text or "<link_udp_port>" in text:
            raise CheckError("the generated world has a config dir or a link: the candidate must be loaded by overrides only")
        assert text.count("<gyro_source>") == 1
        return text.replace("<gyro_source>", "<launch_pad/>\n" + script + "\n" + overrides + "<gyro_source>", 1)
    return edit


def scenario_for(out_dir, num, den):
    text = rq.BASE_SCENARIO.read_text(encoding="utf-8")
    text, k1 = re.subn(r"(tick_period_num_us:\n  value: )\d+", rf"\g<1>{num}", text)
    text, k2 = re.subn(r"(tick_period_den:\n  value: )\d+\n", rf"\g<1>{den}\n", text)
    if (k1, k2) != (1, 1):
        raise CheckError(f"{rq.BASE_SCENARIO}: the tick entries were not found")
    path = Path(out_dir) / "scenario" / "hover.yaml"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    return path


def stopped(t):
    return all(v == 0 for v in t["dshot"])


def throttle(t):
    return all(v >= DSHOT_THROTTLE_MIN for v in t["dshot"])


def finite_steps(log):
    keys = ("body_pos_ned", "body_vel_ned", "body_q_wxyz", "body_omega_frd")
    return all(all(math.isfinite(x) for k in keys for x in st[k]) for st in log["steps"])


def samples(log, m, stride, tilt_max):
    """[(tick, t_us, tilt, heading-frame tilt vector, setpoint vector)] at every `stride`-th tick."""
    ticks, steps, manuals = log["ticks"], log["steps"], log["manuals"]
    out, mi = [], -1
    for j in range(0, len(ticks), stride):
        if j // m >= len(steps):
            break
        while mi + 1 < len(manuals) and manuals[mi + 1]["tick"] <= j:
            mi += 1
        mc = manuals[mi] if mi >= 0 else None
        tilt, v, psi = rq.tilt_split(steps[j // m]["body_q_wxyz"])
        sp = (tilt_max * mc["roll"], tilt_max * mc["pitch"]) if mc else (0.0, 0.0)
        out.append((j, ticks[j]["sil_t_us"], tilt, rq.heading_frame(v, psi), sp))
    return out


def segment_windows(s1, phases, tilt_max):
    """{segment: (index lo, index hi)} into the samples: the hold phase; each step from its first execution with the step's
    frame; each centre from its first execution with the centred frame."""
    P = {p.name: p for p in phases}
    out = {}

    def span(p, pred=None):
        idx = [i for i, x in enumerate(s1) if p.start_us <= x[1] < p.end_us and (pred is None or pred(x))]
        if not idx:
            raise CheckError(f"no execution of phase {p.name} (with its frame) in the run")
        return idx[0], idx[-1] + 1

    out["hold"] = span(P["hold"])
    target = tilt_max * STEP_STICK
    out["roll_step"] = span(P["roll_step"], lambda x: x[4] == (target, 0.0))
    out["roll_centre"] = span(P["roll_centre"], lambda x: x[4] == (0.0, 0.0))
    out["pitch_step"] = span(P["pitch_step"], lambda x: x[4] == (0.0, target))
    out["pitch_centre"] = span(P["pitch_centre"], lambda x: x[4] == (0.0, 0.0))
    return out


def quantity(seg, x):
    if seg == "roll_step":
        return x[4][0] - x[3][0]
    if seg == "pitch_step":
        return x[4][1] - x[3][1]
    return x[2]


def evaluate(runs, phases, values, settle_fraction):
    """The predicates on {m: log} (module docstring): dict(predicates [dict(name, ok, value, bound, detail)], pass)."""
    tilt_max = float(values["angle_tilt_max"])
    arm_ch = int(values["rc_channel_arm"])
    log1 = runs[1]
    ticks = log1["ticks"]
    preds = []

    def add(name, ok, detail, value=None, bound=None):
        preds.append({"name": name, "ok": bool(ok), "value": value, "bound": bound, "detail": detail})

    finite = all(finite_steps(runs[m]) for m in runs)
    add("finite", finite, "every logged body state of the m = 1 and m = 2 runs is finite" if finite else
        "a logged body state is not finite")

    manuals = log1["manuals"]
    raised = next((mm for mm in manuals if mm["channels"][arm_ch - 1] == rq.SWITCH_HIGH), None)
    lowered = None if raised is None else next(
        (mm for mm in manuals if mm["tick"] > raised["tick"] and mm["channels"][arm_ch - 1] == rq.SWITCH_LOW), None)
    if raised is None or lowered is None:
        add("arm", False, "the script's arm raise or disarm frame is not in the log")
        add("disarm", False, "the script's disarm frame is not in the log")
    else:
        before = all(stopped(t) for t in ticks[:raised["tick"]])
        first = next((j for j in range(raised["tick"], len(ticks)) if throttle(ticks[j])), None)
        delay = None if first is None else ticks[first]["sil_t_us"] - raised["t_us"]
        held = first is not None and all(throttle(t) for t in ticks[first:lowered["tick"]])
        clean = all(stopped(t) or throttle(t) for t in ticks)
        ok = before and first is not None and delay < rq.ARM_IDLE_US and held and clean
        add("arm", ok, f"stop before the raise at {raised['t_us']} us: {before}; first throttle on every motor "
                       f"{'never' if first is None else str(delay) + ' us after the raise'} (within the arm phase, "
                       f"{rq.ARM_IDLE_US} us); throttle on every tick to the disarm frame: {held}; no tick neither stop nor "
                       f"throttle: {clean}", delay, rq.ARM_IDLE_US)
        edge = lowered["t_us"] + SLACK_US
        after = [t for t in ticks if t["sil_t_us"] >= edge]
        last_running = max((t["sil_t_us"] for t in ticks[lowered["tick"]:] if not stopped(t)), default=None)
        ok = bool(after) and all(stopped(t) for t in after)
        add("disarm", ok, f"disarm frame at {lowered['t_us']} us; DShot stop on every motor from {edge} us on "
                          f"({len(after)} ticks): {ok}; last non-stop tick "
                          f"{'none' if last_running is None else str(last_running - lowered['t_us']) + ' us after the frame'}",
            None if last_running is None else last_running - lowered["t_us"], SLACK_US)

    stride = 2
    s = {m: samples(runs[m], m, stride, tilt_max) for m in runs}
    try:
        win = segment_windows(s[1], phases, tilt_max)
    except CheckError as e:
        add("hover", False, str(e))
        add("roll", False, str(e))
        add("pitch", False, str(e))
        return {"predicates": preds, "pass": False}
    by_tick2 = {x[0]: x for x in s[2]}
    segments, e_end, e_max = {}, {}, {}
    for seg, (lo, hi) in win.items():
        q1 = [quantity(seg, x) for x in s[1][lo:hi]]
        q2 = [quantity(seg, by_tick2[x[0]]) for x in s[1][lo:hi] if x[0] in by_tick2]
        if len(q2) != len(q1):
            raise CheckError(f"segment {seg}: the m = 2 run lacks executions of the m = 1 run")
        d = [abs(a - b) for a, b in zip(q1, q2, strict=True)]
        segments[seg] = {"q": (UNIT, ROLES[seg], q1)}
        e_end[seg], e_max[seg] = {"q": d[-1]}, {"q": max(d)}
    checks = t4_rule.scenario_checks(segments, settle_fraction, e_end, e_max)
    b = checks["hold"]["B"][0]
    deg = 180.0 / math.pi
    add("hover", b.ok, f"B on the true tilt over the hold phase (HELD): largest |tilt(n)| - |tilt(n0)| {b.value * deg:.4f} deg "
                       f"<= s Delta + E_max {b.bound * deg:.4f} deg (s {settle_fraction!r}, E_max "
                       f"{e_max['hold']['q'] * deg:.3e} deg)", b.value, b.bound)
    for axis in ("roll", "pitch"):
        step, centre = segments[f"{axis}_step"]["q"][2], segments[f"{axis}_centre"]["q"][2]
        t_step, t_centre = t4_rule.shrinks(step), t4_rule.shrinks(centre)
        add(axis, t_step and t_centre,
            f"T on the {axis} error over the step: |{step[0] * deg:.3f}| -> |{step[-1] * deg:.3f}| deg: "
            f"{'shrinks' if t_step else 'does NOT shrink'}; T on the tilt over the centre: {centre[0] * deg:.3f} -> "
            f"{centre[-1] * deg:.3f} deg: {'shrinks' if t_centre else 'does NOT shrink'}",
            abs(centre[-1]), abs(centre[0]))
    return {"predicates": preds, "pass": all(p["ok"] for p in preds)}


def run_check(manifest, cfg, card, plugin_dir, out_dir, budget, timeout_s=rq.TIMEOUT_S):
    """Check 4 on the decoded candidate `cfg` (mcfg.Config): dict(result PASS/FAIL, predicates, runs, form, error)."""
    schema_hash = mcfg.manifest_schema_hash(manifest)
    section = next((x for x in cfg.sections if x.schema_hash == schema_hash), None)
    if section is None:
        raise CheckError("the config has no section of the image's schema")
    records = mcfg.section_records(section)
    names = list(manifest["params"])
    values = {n: (mcfg.f32_of(r.value_bits) if r.type == "f32" else mcfg.i32_of(r.value_bits))
              for n, r in zip(names, records, strict=True)}
    settle = float(schema.load_yaml(budget)["t4_settle_fraction"]["value"])
    compiled = mcfg.f32_of(int(manifest["params"]["t4_settle_fraction"]["default_bits"], 16))
    if rq.f32(settle) != compiled:
        raise CheckError(f"budget t4_settle_fraction {settle!r} is not the image's compiled {compiled!r}")
    rq.CARD = Path(card)
    hover = rq.derive_hover()
    phases = phases_for(values, hover)
    num, den = int(values["tick_period_num_us"]), int(values["tick_period_den"])
    out_dir = Path(out_dir)
    scenario = scenario_for(out_dir, num, den)
    edit = sdf_edit(script_xml(phases, int(values["rc_channel_arm"]), int(values["rc_channel_angle"])),
                    override_xml(manifest, records))
    total_us = phases[-1].end_us
    ticks = -(-total_us * den // num)
    sensors = gen_world.SensorSet(gyro="model", bias_signs=(0,) * 6)
    runs, info = {}, []
    for m in MEMBERS:
        try:
            r = run_scenario.run(Path(card), scenario, rq.SEED, m, "test", "lo", out_dir / f"m{m}", plugin_dir, edit,
                                 iterations=-(-ticks // m), timeout_s=timeout_s, sensors=sensors,
                                 command_delay_ticks=rq.COMMAND_DELAY_TICKS)
        except run_scenario.RunError as e:
            return {"result": "FAIL", "predicates": [], "runs": info,
                    "error": f"the m = {m} gz run did not end cleanly (a refused config is refused here): {e}"}
        runs[m] = r.log
        info.append({"m": m, "log_sha256": r.log_sha256, "wall_s": round(r.wall_s, 3),
                     "steps": len(r.log["steps"]), "ticks": len(r.log["ticks"]),
                     "release_tick": (r.log["pad"] or {}).get("release_tick"),
                     "gz": r.log["header"].get("gz_sim_version", "UNKNOWN")})
        Path(r.log_path).unlink(missing_ok=True)  # ~80 MB; its sha256 is in the report
    ev = evaluate(runs, phases, values, settle)
    return {"result": "PASS" if ev["pass"] else "FAIL", "predicates": ev["predicates"], "runs": info,
            "hover_stick": hover.stick_f32, "step_stick": STEP_STICK, "settle_fraction": settle}
