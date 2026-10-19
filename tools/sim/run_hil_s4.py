#!/usr/bin/env python3
"""HIL S4 (quad spec 4.0 M4, decision 0046 stage S4): the real-card Gazebo world flown closed-loop by the FC3v2 board. Bench
only; never run by CI. Evidence, not a test: nothing is tuned and the card values are INFERRED / UNVERIFIED / PLACEHOLDER.

  MARV_PROBE_SERIAL=<probe> uv run python tools/sim/run_hil_s4.py --port <board port> --out-dir <dir> [--seconds 60]
      [--delay-ticks N] [--samples-per-record F] [--plugin-dir <host-gz-quadfc plugin dir>] [--no-reset] [--expect s4a|late]

Needs `cmake --preset host-gz-quadfc && cmake --build --preset host-gz-quadfc`, gz-sim 8, and the board running the HIL S4
image (`cmake --preset m33 -B build/m33-hil -DMARV_FC3V2_HIL=ON -DMARV_FC3V2_HIL_RECEIVER=ON`, flashed with
tools/target/flash.sh; ESC battery unplugged: the image drives no motor pad). The board's arming and sticks come from its own
CRSF receiver (decision 0046 item 4); the host sends IMU samples only.

What it does. Reboots the board through the debug probe (tools/target/flash.sh --probe-only; --no-reset skips it), so the
composition starts fresh (the plugin refuses a board that has already run ticks). Generates the S2e world of
tools/sim/run_quad_fc.py (the real card vehicles/marv_5in_fc3v2.yaml with drag, the fc3v2 tick, m = 1, the model IMU at the
nominal corner, <launch_pad/>) with the <hil> element in place of the stick script and of <command_delay_ticks>: the lockstep
plugin's HIL source (sim/gz/plugin/src/hil_source.hpp) paces the ticks on the wall clock, sends the IMU samples to the board F
per record when their stamps are due, and steps the plant at tick j with the board's command of tick j - N; a command not
there when needed is the stated late-command fault, which stops the run. Runs gz for --seconds of sim time (= wall time) with
the plugin's status line live on the terminal (board arming state, receiver frames, DShot, height, tilt, command margin), then
writes <stem>.hil_report.txt and prints it. Files in <dir>: the world (.sdf), the gz log (.bin), the board's uplink bytes as
received (.uplink.bin, tools/target/hil_fc.py parse_uplink), the frame and report timing (.frames.csv, .reports.csv), gz's
stderr (.stderr.txt) and the report.

--expect s4a (default): the S4a verdict with the radio off: the run completes; no link fault either way; no IMU deadline miss
while streaming and exactly one after (D_HIL); every rate tick's command arrived before it was needed; the board never armed
(states boot / calibrating / no_link) and every DShot value is 0; no receiver frame; the pad held the whole run.
--expect late: the negative control: the run must stop with the plugin's "HIL late command" fault.
--expect none: no verdict (S4b, Luis flying).

Choosing N (decision 0046 item 2): N >= F + the measured turnaround (the report's "turnaround" line gives the measured
maximum in ticks) and N <= N_max = 249 (tests/regression/quad/L09/results/hil_s2/hil_delay_margin.txt, whose nominal rows the
report quotes for the N run).
"""

from __future__ import annotations

import argparse
import math
import os
import re
import signal
import statistics
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "card"))
sys.path.insert(0, str(HERE.parent / "target"))
import gen_world  # noqa: E402
import hil_compare  # noqa: E402
import hil_fc  # noqa: E402
import lockstep_log  # noqa: E402
import run_quad_fc  # noqa: E402
import run_scenario  # noqa: E402

PLUGIN_DIR = ROOT / "build" / "host-gz-quadfc" / "sim" / "gz" / "plugin"
PARAM_DEFAULTS = ROOT / "build" / "host-gz-quadfc" / hil_compare.PARAM_DEFAULTS_REL
MARGIN_TABLE = ROOT / "tests" / "regression" / "quad" / "L09" / "results" / "hil_s2" / "hil_delay_margin.txt"
FLASH = ROOT / "tools" / "target" / "flash.sh"
M = 1
SAMPLES_PER_RECORD = 4  # F: the S1/S3 configuration (round trip measured at 4 samples per record, L09/results/hil_s1)
DELAY_TICKS = 25  # N: chosen from the S4a calibration run (tests/regression/quad/L10/results/s4/README.md), <= N_max 249
SECONDS = 60.0  # S4a: at least 60 s of sim time
# Labelled harness values (not physics): how long the plugin waits for the board's hello, receiver-boot and first status
# records (the status comes once per second; the plugin repeats the DTR edge while another process holds the port); how long
# it waits on a late command to state how late it was before stopping the run (about 7 x D_HIL = 14.5 ms); how long it keeps
# reading after the last record (more than two status periods, so the status that counts the end-of-stream deadline miss
# arrives, as hil_fc.py DRAIN_S); the settle after the reboot: on this host ModemManager probes the re-enumerated port (udev
# ID_MM_CANDIDATE=1: 17 bytes written, the port held for 6 to 10 s after the reboot, measured 2026-10-17), and a second
# reader would take part of the uplink, so the run starts after it; the port wait; gz's timeout.
HELLO_WAIT_US = 20_000_000
LATE_WAIT_US = 100_000
DRAIN_US = 2_500_000
RESET_SETTLE_S = 12.0
PORT_WAIT_S = 10.0
TIMEOUT_EXTRA_S = 120.0
ARMED = {"armed", "armed_hold", "armed_failsafe_centred"}
S4A_STATES = {"boot", "calibrating", "no_link"}
NOISE = ("libprotobuf ERROR", "DynamicFactory()")


def reset_board(port):
    serial = os.environ.get("MARV_PROBE_SERIAL")
    if not serial:
        raise SystemExit("run_hil_s4: set MARV_PROBE_SERIAL (the debug probe that reboots the board) or pass --no-reset")
    print(f"run_hil_s4: rebooting the board through probe {serial} (flash.sh --probe-only)", flush=True)
    r = subprocess.run([str(FLASH), "--probe-only"], capture_output=True, text=True, timeout=120)
    if r.returncode != 0:
        raise SystemExit(f"run_hil_s4: flash.sh --probe-only failed:\n{r.stdout[-2000:]}\n{r.stderr[-2000:]}")
    time.sleep(RESET_SETTLE_S)
    deadline = time.monotonic() + PORT_WAIT_S
    while not Path(port).exists():
        if time.monotonic() > deadline:
            raise SystemExit(f"run_hil_s4: {port} did not come back within {PORT_WAIT_S} s of the reboot")
        time.sleep(0.1)


def hil_xml(port, stem, f, n, values_crc):
    return (f"<hil><device>{port}</device><samples_per_record>{f}</samples_per_record><delay_ticks>{n}</delay_ticks>"
            f"<values_crc>{values_crc}</values_crc><hello_wait_us>{HELLO_WAIT_US}</hello_wait_us>"
            f"<late_wait_us>{LATE_WAIT_US}</late_wait_us><drain_us>{DRAIN_US}</drain_us>"
            f"<uplink_path>{stem}.uplink.bin</uplink_path><frames_path>{stem}.frames.csv</frames_path>"
            f"<reports_path>{stem}.reports.csv</reports_path></hil>")


def world(out_dir, port, f, n, values_crc):
    scenario, num, den = run_quad_fc.fc3_scenario(out_dir)
    name, _ = gen_world.generate(run_quad_fc.CARD, scenario, "test", M, "lo", None, ROOT)
    stem = (Path(out_dir) / f"{Path(name).stem}_hil_n{n}").resolve()
    sensors = gen_world.SensorSet(gyro="model", bias_signs=(0,) * 6)
    _, text = gen_world.generate(run_quad_fc.CARD, scenario, "test", M, "lo", f"{stem}.bin", ROOT, sensors=sensors)
    text = re.sub(r'\s*<sil_override param="ol_dshot_m\d" type="i32">\d+</sil_override>', "", text)
    assert text.count("<gyro_source>") == 1
    text = text.replace("<gyro_source>", "<launch_pad/>\n" + hil_xml(port, stem, f, n, values_crc) + "\n<gyro_source>", 1)
    seed = int(re.search(r"<seed>(\d+)</seed>", text).group(1))
    Path(f"{stem}.sdf").write_text(text, encoding="utf-8")
    return stem, num, den, seed


# --gui (S4b, Luis flying, 2026-10-18: "it's tough for me to do anything when i can't see the gazebo 3d env"): display-only
# additions to the world and a gz GUI client on the server's partition. Nothing here has mass, inertia or collision, so the
# plant is unchanged; every size below is a labelled display value, not vehicle data. The rotor discs sit at the card's rotor
# positions (FRD in the plugin block, drawn in the link's FLU frame: y and z negated).
DISPLAY_PROP_RADIUS_M = 0.0889  # display only: a 7 inch propeller disc (Luis, 2026-10-18: "it's a 7 inch frame")
DISPLAY_BODY_M = (0.10, 0.05, 0.03)  # display only: the body box
DISPLAY_GROUND_Z_M = -0.04  # display only: the ground visual just below the body (the world has no ground and no gravity)
DISPLAY_MARKER_SPACING_M = 2.0  # display only: reference posts every 2 m so motion is visible against the ground
DISPLAY_MARKER_COUNT = 5  # display only: posts per side of the grid
DISPLAY_GUI_START_S = 15.0  # harness value: the GUI loads its rendering engine before the real-time run starts
DISPLAY_SUN = "<light type='directional' name='sun'><cast_shadows>true</cast_shadows><pose>0 0 10 0 0 0</pose>" \
    "<diffuse>0.9 0.9 0.9 1</diffuse><direction>-0.5 0.1 -0.9</direction></light>"


def display_world(text):
    """The world with display-only visuals (quad body, arms, rotor discs, a red nose, ground plane and reference posts),
    the scene broadcaster a GUI client needs, and a light."""
    rotors = re.findall(r'<rotor motor="(\d)">\s*<position_frd_m unit="m">([-\d.e]+) ([-\d.e]+) ([-\d.e]+)</position_frd_m>', text)
    assert len(rotors) == 4, rotors
    bx, by, bz = DISPLAY_BODY_M
    vis = [f"<visual name='body'><geometry><box><size>{bx} {by} {bz}</size></box></geometry>"
           "<material><ambient>0.15 0.15 0.15 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material></visual>",
           f"<visual name='nose'><pose>{bx / 2} 0 0 0 0 0</pose><geometry><box><size>{bz} {bz} {bz}</size></box></geometry>"
           "<material><ambient>0.8 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material></visual>"]
    for motor, x, y, z in rotors:
        fx, fy, fz = float(x), -float(y), -float(z)
        arm = math.hypot(fx, fy)
        yaw = math.atan2(fy, fx)
        colour = "0.1 0.4 1 0.6" if float(x) > 0 else "0.9 0.9 0.9 0.6"  # front rotors blue, rear white
        vis.append(f"<visual name='arm{motor}'><pose>{fx / 2} {fy / 2} 0 0 0 {yaw}</pose><geometry><box><size>{arm} 0.012 0.008"
                   "</size></box></geometry><material><diffuse>0.3 0.3 0.3 1</diffuse></material></visual>")
        vis.append(f"<visual name='rotor{motor}'><pose>{fx} {fy} {fz} 0 0 0</pose><geometry><cylinder><radius>"
                   f"{DISPLAY_PROP_RADIUS_M}</radius><length>0.004</length></cylinder></geometry><material><ambient>{colour}"
                   f"</ambient><diffuse>{colour}</diffuse></material><transparency>0.4</transparency></visual>")
    text = text.replace("</inertial>", "</inertial>\n" + "\n".join(vis), 1)
    posts = []
    half = DISPLAY_MARKER_COUNT // 2
    for i in range(-half, half + 1):
        for j in range(-half, half + 1):
            if i == 0 and j == 0:
                continue
            px, py = i * DISPLAY_MARKER_SPACING_M, j * DISPLAY_MARKER_SPACING_M
            posts.append(f"<visual name='post_{i}_{j}'><pose>{px} {py} 0.5 0 0 0</pose><geometry><cylinder><radius>0.03</radius>"
                         "<length>1.0</length></cylinder></geometry><material><diffuse>1 0.6 0 1</diffuse></material></visual>")
    ground = ("<model name='display_ground'><static>true</static><pose>0 0 " f"{DISPLAY_GROUND_Z_M}" " 0 0 0</pose>"
              "<link name='ground_link'><visual name='ground'><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane>"
              "</geometry><material><ambient>0.3 0.45 0.3 1</ambient><diffuse>0.35 0.55 0.35 1</diffuse></material></visual>"
              + "".join(posts) + "</link></model>")
    extra = ("<plugin filename='gz-sim-scene-broadcaster-system' name='gz::sim::systems::SceneBroadcaster'/>\n"
             + DISPLAY_SUN + "\n" + ground + "\n")
    i = text.index("<model name=")
    return text[:i] + extra + text[i:]


def unpause(world_name, env):
    """Start the paused world (the --gui run): gz-sim's world control service, pause false."""
    req = ["gz", "service", "--force-version", "13", "-s", f"/world/{world_name}/control", "--reqtype", "gz.msgs.WorldControl",
           "--reptype", "gz.msgs.Boolean", "--timeout", "5000", "--req", "pause: false"]
    r = subprocess.run(req, env=env, capture_output=True, text=True)
    print(f"run_hil_s4: world unpaused ({r.stdout.strip() or r.stderr.strip()})", flush=True)


def run_gz(stem, iterations, seed, plugin_dir, seconds, gui=False):
    # --gui: the server starts PAUSED (no -r) and the GUI client joins it; the world runs only after DISPLAY_GUI_START_S,
    # once the GUI holds the scene. Tries of 2026-10-18: a GUI joining a running world (either started together, or the GUI
    # first) stalled the server ~106 ms at t = 1-2 s (the scene broadcaster's first full scene), the board missed its IMU
    # deadline and latched imu_stale. The plugin opens the board and reads its hello at load (Configure) and starts the
    # wall-clock pacing at the first unpaused step, so nothing is paced while paused.
    cmd = [*run_scenario.GZ_ARGS, "-s", *([] if gui else ["-r"]), "--iterations", str(iterations), "--seed", str(seed),
           f"{stem}.sdf"]
    Path(f"{stem}.bin").unlink(missing_ok=True)
    env = run_scenario.gz_env(plugin_dir)
    t0 = time.monotonic()
    gui_p = None
    timer = None
    if gui:
        world_name = re.search(r'<world name="([^"]+)"', Path(f"{stem}.sdf").read_text(encoding="utf-8")).group(1)
        gui_log = open(f"{stem}.gui.txt", "w", encoding="utf-8")
        gui_p = subprocess.Popen(["nice", "-n", "19", *run_scenario.GZ_ARGS, "-g"], env=env, stdout=gui_log,
                                 stderr=subprocess.STDOUT)
        print(f"run_hil_s4: gz GUI starting; the world is paused and starts in {DISPLAY_GUI_START_S:.0f} s", flush=True)
        timer = threading.Timer(DISPLAY_GUI_START_S, unpause, args=(world_name, env))
        timer.start()
    with open(f"{stem}.stderr.txt", "w", encoding="utf-8") as err, open(f"{stem}.stdout.txt", "w", encoding="utf-8") as out:
        p = subprocess.Popen(cmd, env=env, stdout=out, stderr=subprocess.PIPE, text=True, bufsize=1)
        try:
            for line in p.stderr:
                err.write(line)
                if not any(x in line for x in NOISE):
                    print(line, end="", flush=True)
            rc = p.wait(timeout=seconds + TIMEOUT_EXTRA_S)
        except KeyboardInterrupt:
            print("run_hil_s4: interrupted; stopping gz (the board sees no more samples: IMU deadline miss, disarm)", flush=True)
            p.send_signal(signal.SIGINT)
            rc = p.wait(timeout=TIMEOUT_EXTRA_S)
        finally:
            if timer is not None:
                timer.cancel()
            if gui_p is not None:
                gui_p.terminate()
                try:
                    gui_p.wait(timeout=PORT_WAIT_S)
                except subprocess.TimeoutExpired:
                    gui_p.kill()
    return rc, time.monotonic() - t0


def read_csv(path):
    lines = Path(path).read_text(encoding="utf-8").splitlines()[1:]
    return [tuple(int(x) for x in ln.split(",")) for ln in lines if ln]


def pct(v, p):
    s = sorted(v)
    return s[min(len(s) - 1, max(0, math.ceil(p / 100 * len(s)) - 1))] if s else float("nan")


def dist(v, scale=1e-3):
    if not v:
        return "none"
    return (f"min {min(v) * scale:.1f}  median {statistics.median(v) * scale:.1f}  p99 {pct(v, 99) * scale:.1f}  "
            f"max {max(v) * scale:.1f}  (n {len(v)})")


def margin_row(n):
    for ln in MARGIN_TABLE.read_text(encoding="utf-8").splitlines():
        m = re.match(r"\s*(\d+)\s+(\d+) \|\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)\s+\d+\s+\d+\s+(yes|NO)", ln)
        if m and int(m.group(1)) == n:
            return float(m.group(3)), float(m.group(5)), m.group(6)
    return None


def analyse(stem, n, f, seconds, iterations, rc, wall_s, expect):
    L, reasons = [], []
    stderr = Path(f"{stem}.stderr.txt").read_text(encoding="utf-8", errors="replace")
    refused = [ln for ln in stderr.splitlines() if "REFUSED" in ln]
    hil_lines = [ln for ln in stderr.splitlines() if "marv_gz_lockstep: hil " in ln and "hil t=" not in ln]
    L.append(f"HIL S4 run {Path(stem).name}: N = {n} ticks, F = {f} samples per record, {seconds} s requested "
             f"({iterations} ticks at m = {M}); gz exit {rc}, wall {wall_s:.1f} s")
    L.extend(hil_lines)
    if refused:
        L.append("gz refused / stopped: " + refused[-1].split("REFUSED: ", 1)[-1])
    up = hil_fc.parse_uplink(Path(f"{stem}.uplink.bin").read_bytes()) if Path(f"{stem}.uplink.bin").exists() else None
    h = up.hello if up else None
    if h is None:
        L.append("no hello in the uplink")
        return L, ["no hello"] if expect == "s4a" else []
    num, den, d = h["tick_period_num_us"], h["tick_period_den"], h["rate_loop_divisor"]
    tick_ns = num * 1000 / den
    L.append(f"board: version {up.version!r}, schema hash 0x{(h['schema_hash_high'] << 32) | h['schema_hash_low']:016x}, "
             f"values CRC 0x{h['param_values_crc']:08x}, tick {num}/{den} us ({tick_ns / 1000:.4f} us), rate divisor {d}, "
             f"D_HIL {h['deadline_us']} us; receiver boot {up.receiver_boot}")
    frames = read_csv(f"{stem}.frames.csv")
    reports = read_csv(f"{stem}.reports.csv")
    sent_at = {ft: sent for ft, _, _, _, sent in frames}
    turn = [arr - sent_at[t - t % f] for t, arr, _ in reports if t - t % f in sent_at]
    margin = [need - arr for _, arr, need in reports]
    late = [sent - (due + lag) for _, _, due, lag, sent in frames]
    sent_samples = sum(c for _, c, _, _, _ in frames)
    lag_ns = frames[-1][3] if frames else 0
    sim_ns = frames[-1][2] if frames else 0
    L.append(f"downlink: {len(frames)} sample records, {sent_samples} samples; send time after the record's (lag-shifted) "
             f"instant (us): {dist(late)}")
    L.append(f"wall-clock pacing: schedule lag absorbed {lag_ns / 1e6:.3f} ms over {sim_ns / 1e9:.3f} s of stamps "
             f"(sim / wall rate {sim_ns / (sim_ns + lag_ns) if sim_ns else float('nan'):.6f})")
    L.append(f"turnaround, frame sent -> its rate tick's DShot report read (us): {dist(turn)}; max = "
             f"{max(turn) / tick_ns:.2f} ticks" if turn else "turnaround: no report")
    if turn:
        need_n = f + math.ceil(max(turn) / tick_ns)
        L.append(f"N rule (decision 0046 item 2): N >= F + ceil(max turnaround / tick) = {f} + {need_n - f} = {need_n}; "
                 f"N = {n}: {'met' if n >= need_n else 'NOT met'}, {n - need_n} ticks to spare")
    L.append(f"command margin, needed - arrived, per rate tick (us): {dist(margin)}")
    row = margin_row(n)
    L.append(f"QF-3 at command delay N = {n} (nominal rows, {MARGIN_TABLE.relative_to(ROOT)}): " +
             (f"PM {row[0]:.3f} deg (PM_min 45), Ms {row[1]:.4f} (Ms_max 2.0), pass {row[2]}; N_max 249"
              if row else "row not listed: run tools/card/hil_delay_margin.py"))
    L.append("worst-corner rows: not a margin result (decision 0046 item 2): the gain rule puts the worst corner at PM_min "
             "at N = 0, so any N > 0 is below it there (hil_delay_margin.txt: -0.13 deg at N = 1)")
    st = up.status
    clk = h["clk_sys_hz"]
    L.append(f"board status records: {len(st)}; last: " + (hil_fc.status_line(st[-1], clk) if st else "none"))
    base = next((s for s in reversed(st) if s["sample_records"] == 0), None)
    base_faults = {k: base[k] for k in (*hil_fc.LINK_FAULTS, "rx_bytes") if base and base[k]}
    L.append("board counters before the stream (the plugin's first status; another writer such as a ModemManager probe "
             f"shows here): {base_faults or 'all zero'}")
    states = []
    for s in st:
        name = hil_fc.ARM_STATES[s["arming_state"]] if s["arming_state"] < len(hil_fc.ARM_STATES) else str(s["arming_state"])
        if not states or states[-1][0] != name:
            states.append((name, s["board_time_us"]))
    L.append("arming states in order (board us): " + ", ".join(f"{a} @{b}" for a, b in states))
    nonzero = [(t, v) for t, v in up.reports if any(v)]
    L.append(f"DShot reports {len(up.reports)}, non-zero {len(nonzero)}" + (f", first {nonzero[0]}" if nonzero else ""))
    rs = up.receiver_status[-1] if up.receiver_status else None
    L.append(f"receiver: frames received {len(up.receiver_frames)}; last receiver status {rs}")
    log, log_err = None, None
    try:
        log = lockstep_log.read(f"{stem}.bin")
    except (OSError, lockstep_log.LogError) as e:
        log_err = str(e)
    if log:
        pad = log["pad"]
        ticks = log["ticks"]
        L.append(f"gz log: {len(log['steps'])} steps, {len(ticks)} ticks ({len(ticks) * tick_ns / 1e9:.3f} s of sim time), "
                 f"trailer {log['trailer']}; launch pad {'present' if pad else 'ABSENT'}, release "
                 f"{'none: held the whole run' if pad and pad['release_tick'] is None else pad and pad['release_tick']}")
        if ticks:
            z = [s["body_pos_ned"][2] for s in log["steps"]]
            L.append(f"body z NED over the run: {min(z):+.6f} .. {max(z):+.6f} m; applied DShot non-zero ticks "
                     f"{sum(1 for t in ticks if any(t['dshot']))}")
    else:
        L.append(f"gz log unreadable: {log_err}")
    up_faults = {k: getattr(up, k) for k in ("crc_failures", "bad_headers", "seq_gaps", "unframed_bytes") if getattr(up, k)}
    L.append(f"uplink host-side faults: {up_faults or 'none'}")

    if expect == "late":
        fault = next((re.search(r"HIL late command: tick (\d+)", ln) for ln in refused if "HIL late command" in ln), None)
        if fault is None:
            reasons.append("the run did not stop with the HIL late-command fault")
        else:
            j = int(fault.group(1))
            stepped = [t["tick"] for t in log["ticks"]] if log else []
            L.append(f"late-command fault at tick {j}; the gz log's last stepped tick {stepped[-1] if stepped else 'none'} "
                     f"({len(stepped)} ticks): tick {j} was never stepped, no command reused")
            if stepped and stepped[-1] >= j:
                reasons.append(f"tick {stepped[-1]} was stepped at or after the late tick {j}")
    elif expect == "s4a":
        if rc != 0 or refused:
            reasons.append(f"gz exit {rc}" + (f", refused: {refused[-1]}" if refused else ""))
        if not log or log["trailer"] != {"steps": iterations, "ticks": iterations * M, "applied": iterations}:
            reasons.append("the gz log is not complete")
        if iterations * tick_ns / 1e9 < seconds:
            reasons.append("less sim time than requested")
        if up_faults:
            reasons.append(f"uplink faults {up_faults}")
        if not st:
            reasons.append("no status record")
        else:
            last = st[-1]
            for k in hil_fc.LINK_FAULTS:
                if last[k] - (base[k] if base else 0):
                    reasons.append(f"board {k} +{last[k] - (base[k] if base else 0)} during the run")
            if last["samples"] != sent_samples or last["ticks_run"] != sent_samples:
                reasons.append(f"board samples {last['samples']} / ticks {last['ticks_run']}, host sent {sent_samples}")
            if any(s["deadline_misses"] for s in st if s["ticks_run"] < sent_samples):
                reasons.append("an IMU deadline miss while streaming")
            if last["deadline_misses"] != 1:
                reasons.append(f"{last['deadline_misses']} deadline misses after the stream, expected exactly 1 (D_HIL)")
            if last["rate_timing_faults"]:
                reasons.append(f"rate timing faults {last['rate_timing_faults']}")
            bad = {a for a, _ in states} - S4A_STATES
            if bad:
                reasons.append(f"arming states outside {sorted(S4A_STATES)}: {sorted(bad)}")
        expected_reports = len(range(0, sent_samples, d))
        if len(up.reports) != expected_reports:
            reasons.append(f"{len(up.reports)} DShot reports, expected {expected_reports}")
        used_expected = len(range(0, iterations - n, d)) if iterations > n else 0
        if len(reports) != used_expected:
            reasons.append(f"{len(reports)} reports used by the plant, expected {used_expected}")
        if not margin or min(margin) <= 0:
            reasons.append("a command arrived after it was needed")
        if nonzero:
            reasons.append(f"{len(nonzero)} non-zero DShot reports")
        if up.receiver_frames or (rs and rs["rc_frames"]):
            reasons.append("receiver frames arrived (radio on?)")
        if up.receiver_boot is None:
            reasons.append("no receiver-boot record (not the HIL S4 image)")
        if not log or not log["pad"] or log["pad"]["release_tick"] is not None:
            reasons.append("the launch pad did not hold the whole run")
    if expect != "none":
        L.append(("VERDICT OK" if not reasons else "VERDICT NOT OK: " + "; ".join(reasons)) + f" ({expect})")
    return L, reasons


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", required=True, help="the board's USB CDC port (/dev/serial/by-id/...)")
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--seconds", type=float, default=SECONDS)
    ap.add_argument("--delay-ticks", type=int, default=DELAY_TICKS, help="N (default %(default)s)")
    ap.add_argument("--samples-per-record", type=int, default=SAMPLES_PER_RECORD, help="F (default %(default)s)")
    ap.add_argument("--plugin-dir", default=str(PLUGIN_DIR))
    ap.add_argument("--no-reset", action="store_true", help="do not reboot the board first")
    ap.add_argument("--expect", choices=("s4a", "late", "none"), default="s4a")
    ap.add_argument("--gui", action="store_true",
                    help="display-only visuals in the world and a gz GUI client (S4b: see the quad while flying)")
    a = ap.parse_args(argv)
    out_dir = Path(a.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    if not run_scenario.plugin_present(a.plugin_dir):
        raise SystemExit(f"run_hil_s4: no plugin in {a.plugin_dir} (cmake --build --preset host-gz-quadfc)")
    values_crc = hil_compare.values_crc(hil_compare.load_table(PARAM_DEFAULTS))
    if not a.no_reset:
        reset_board(a.port)
    stem, num, den, seed = world(out_dir, a.port, a.samples_per_record, a.delay_ticks, values_crc)
    if a.gui:
        sdf = Path(f"{stem}.sdf")
        sdf.write_text(display_world(sdf.read_text(encoding="utf-8")), encoding="utf-8")
    iterations = math.ceil(a.seconds * 1e6 * den / num) // M
    print(f"run_hil_s4: world {stem}.sdf; {iterations} ticks ({a.seconds} s), N = {a.delay_ticks}, F = {a.samples_per_record}",
          flush=True)
    rc, wall = run_gz(stem, iterations, seed, a.plugin_dir, a.seconds, gui=a.gui)
    lines, reasons = analyse(stem, a.delay_ticks, a.samples_per_record, a.seconds, iterations, rc, wall, a.expect)
    text = "\n".join(lines) + "\n"
    Path(f"{stem}.hil_report.txt").write_text(text, encoding="utf-8")
    print(text)
    return 0 if (a.expect == "none" or not reasons) else 1


if __name__ == "__main__":
    sys.exit(main())
