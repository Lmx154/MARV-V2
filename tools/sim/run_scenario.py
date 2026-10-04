#!/usr/bin/env python3
"""Scenario runner of quad L2 (quad spec section 4 L2, "(card, scenario, seed) -> logged run"; docs/decisions/0003).

  run_scenario.py --card <card> --scenario <scenario> [--m <int> | --sequence] [--seed <int>] [--mode test|pilot]
                  [--hover lo|hi] --out-dir <dir> [--plugin-dir <dir>] [--measure-rtf]

As a module: run(), run_sequence(), measure_rtf(), write_report(), render_report(). One gz process per (world, m): one SIL
per process. Every gz process is `gz sim --force-version 8 -s -r --iterations N --seed S <world>` with its own GZ_PARTITION
and GZ_IP=127.0.0.1, and GZ_SIM_SYSTEM_PLUGIN_PATH set to the plugin build directory.

Files written to out_dir, for base = <world stem>_seed<seed>:  <base>.sdf (the world as run, after sdf_edit), <base>.bin
(the plugin log; the world's <log_path> points here), <base>.report.txt (the run report). A sequence writes
<vehicle>_<scenario>_<mode>_seed<seed>_sequence.report.txt beside its runs' files.

Exit code. A non-zero exit is a failure (RunError), except that a known gz-transport discovery segfault at exit was seen once:
exit by SIGSEGV (-11, or 139 through a shell) with a complete log (trailer counts equal the run's iterations) is accepted, is
returned in RunResult.warnings, is reported in the run report and raises a RuntimeWarning. Nothing else is accepted.

Seed. `--seed S` goes to gz and, when S differs from the scenario's seed, into the world's plugin <seed> element (the log
header carries it). marv_plant v0 and the composition draw no random numbers, so the seed is RESERVED at v0 (no effect).
With sensors (a gen_world.SensorSet, decision 0019; run(), run_sequence()) the world carries the plugin's sensor elements:
with the model gyro the plant's IMU draws from the seed's noise stream 0, and the report says so; None is the world and
the report as without the argument.

Stale steps (0003 item 11). Step i is stale when its raw gz read (position, attitude, linear and angular velocity: 13
doubles) is bitwise equal to step i-1's while the wrench applied between the two reads (APPLIED record i-1) was nonzero.
A step is fresh when its read differs from the previous step's (step 0 is fresh). The report gives the count of stale
steps and the last fresh step's index and time.

SIM-3 (0003 item 2). measure_rtf runs the test world for the full duration and a 10-iteration run of the same world in
separate processes. Per-core RTF assumes one process = one core (checked, not assumed: the CPU time of the child,
resource.getrusage(RUSAGE_CHILDREN), must not exceed its wall time). Startup overhead = wall time of the 10-iteration run;
the marginal RTF is the extra simulated time over the extra wall time of the full run over the short one. The required
value is design/budget.yaml batch_plan, which is UNKNOWN; it is never invented and the comparison stays "UNKNOWN, SIM-3 open".
"""

from __future__ import annotations

import argparse
import dataclasses
import glob
import hashlib
import math
import os
import re
import resource
import shutil
import signal
import struct
import subprocess
import sys
import time
import uuid
import warnings as pywarnings
from fractions import Fraction
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "card"))
import gen_world  # noqa: E402
import gen_plant_config as gpc  # noqa: E402
import hover as hover_mod  # noqa: E402
import lockstep_log  # noqa: E402
import report as card_report  # noqa: E402
import scenario as scn  # noqa: E402
import schema  # noqa: E402

UNKNOWN = "UNKNOWN"
DEFAULT_PLUGIN_DIR = ROOT / "build" / "host-gz" / "sim" / "gz" / "plugin"
DEFAULT_BUDGET = ROOT / "design" / "budget.yaml"
PLUGIN_FILE = "libmarv_gz_lockstep.so"
GZ_ARGS = ["gz", "sim", "--force-version", "8"]
# Labelled scenario value (harness setting, not physics): the wall-clock limit on one gz process. It only bounds a hung
# run; the longest committed run takes a few seconds (tests/regression/quad/L02/results/sim3).
TIMEOUT_S = 600
# Labelled scenario value (harness setting, not physics): the host steps of the short run whose wall time measure_rtf
# subtracts as startup overhead (SIM-3). Any count far below a scenario's own step count serves.
STARTUP_ITERATIONS = 10
SIGSEGV_CODES = (-signal.SIGSEGV, 128 + signal.SIGSEGV)
DPKG = {"gz-physics": "libgz-physics7", "sdformat": "libsdformat14"}
DARTSIM_PLUGIN_GLOB = "/usr/lib/*/gz-physics-7/engine-plugins/libgz-physics7-dartsim-plugin.so"
PERFECT_MODEL = card_report.MODEL_LINE
SEGV_WARNING = ("gz exited by SIGSEGV after a complete log (the known gz-transport discovery segfault at shutdown, seen "
                "once); the run is accepted because the trailer counts equal the run's iterations")
STEP_GZ_FIELDS = (("gz_pos_enu", 3), ("gz_q_wxyz", 4), ("gz_lin_vel_enu", 3), ("gz_ang_vel_enu", 3),
                  ("body_pos_ned", 3), ("body_vel_ned", 3), ("body_q_wxyz", 4), ("body_omega_frd", 3))
DOUBLE = 8


class RunError(Exception):
    pass


@dataclasses.dataclass
class GzProcess:
    returncode: int
    stdout: str
    stderr: str
    wall_s: float
    cpu_s: float


@dataclasses.dataclass
class RunResult:
    card: str
    scenario: str
    scenario_name: str
    seed: int
    m: int
    mode: str
    hover: str | None
    iterations: int
    root: str
    budget: str
    plugin_dir: str
    world_path: str
    log_path: str
    report_path: str | None
    returncode: int
    warnings: list
    log: dict
    log_sha256: str
    world_sha256: str
    sdf_edited: bool
    scenario_sha256: str
    wall_s: float
    cpu_s: float
    sim_s: float
    tick_period_s: Fraction
    stale: dict
    stderr_tail: str
    sensors: object = None

    @property
    def rtf(self):
        return self.sim_s / self.wall_s if self.wall_s > 0 else float("nan")


@dataclasses.dataclass
class SequenceResult:
    runs: list
    report_path: str | None


@dataclasses.dataclass
class Sim3Result:
    card: str
    scenario: str
    m: int
    seed: int
    iterations: int
    startup_iterations: int
    sim_s: float
    wall_s: float
    cpu_s: float
    startup_sim_s: float
    startup_wall_s: float
    startup_cpu_s: float
    rtf_wall: float
    rtf_marginal: float | None
    cpu_over_wall: float
    marginal_cpu_over_wall: float | None
    cores: int
    plugin_build_type: str
    required: str
    comparison: str


def sha256_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def sha256_text(text):
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


# ---- the gz process -------------------------------------------------------------------------------------------------

def gz8_available():
    if shutil.which("gz") is None:
        return False
    try:
        out = subprocess.run([*GZ_ARGS, "--versions"], capture_output=True, text=True, timeout=60)
    except (OSError, subprocess.TimeoutExpired):
        return False
    return re.search(r"(^|[^0-9.])8\.[0-9]+", out.stdout) is not None


def plugin_present(plugin_dir=DEFAULT_PLUGIN_DIR):
    return (Path(plugin_dir) / PLUGIN_FILE).exists()


def gz_env(plugin_dir, extra_env=None):
    return dict(os.environ, GZ_PARTITION=f"marv_{uuid.uuid4().hex}", GZ_IP="127.0.0.1",
                GZ_SIM_SYSTEM_PLUGIN_PATH=str(plugin_dir), **(extra_env or {}))


def _cpu_children():
    r = resource.getrusage(resource.RUSAGE_CHILDREN)
    return r.ru_utime + r.ru_stime


def run_gz_process(sdf_path, iterations, seed, plugin_dir=DEFAULT_PLUGIN_DIR, extra_env=None, timeout_s=TIMEOUT_S):
    """One `gz sim -s -r --iterations N --seed S` process on a world file. The CPU time is that of all children reaped
    during the call, so callers must not run other child processes concurrently."""
    cmd = [*GZ_ARGS, "-s", "-r", "--iterations", str(iterations), "--seed", str(seed), str(sdf_path)]
    cpu0 = _cpu_children()
    t0 = time.perf_counter()
    try:
        r = subprocess.run(cmd, env=gz_env(plugin_dir, extra_env), capture_output=True, text=True, timeout=timeout_s)
    except subprocess.TimeoutExpired as e:
        raise RunError(f"gz did not finish within {timeout_s} s: {' '.join(cmd)}") from e
    wall = time.perf_counter() - t0
    return GzProcess(r.returncode, r.stdout, r.stderr, wall, _cpu_children() - cpu0)


def stderr_tail(stderr, lines=8):
    """The last lines of gz's stderr without the libprotobuf descriptor-pool noise gz-msgs prints at every start."""
    keep = [ln for ln in stderr.splitlines()
            if ln.strip() and "libprotobuf ERROR" not in ln and not ln.startswith("DynamicFactory()")]
    return "\n".join(keep[-lines:])


def complete_trailer(log, iterations, m):
    return (log is not None and log["trailer"] == {"steps": iterations, "ticks": iterations * m, "applied": iterations}
            and len(log["steps"]) == iterations and len(log["ticks"]) == iterations * m
            and len(log["applied"]) == iterations)


def check_exit(returncode, stderr, log, log_error, iterations, m):
    """The warnings of an accepted run; raises RunError for a failed one (see the module docstring)."""
    tail = stderr_tail(stderr)
    if "REFUSED" in stderr:
        raise RunError(f"the plugin refused the world:\n{tail}")
    complete = complete_trailer(log, iterations, m)
    if returncode == 0:
        if not complete:
            raise RunError(f"gz exited 0 but the log is not complete ({log_error or 'trailer counts differ from the run'})"
                           f":\n{tail}")
        return []
    if returncode in SIGSEGV_CODES and complete:
        return [SEGV_WARNING + f" (exit {returncode})"]
    raise RunError(f"gz exited {returncode} (log complete: {complete}{'; ' + log_error if log_error else ''}):\n{tail}")


def ixx_one_ulp(text):
    """The determinism control of 0003 item 10: the generated SDF's ixx one ulp higher (an sdf_edit hook)."""
    out, n = re.subn(r"(<ixx>)([^<]*)", lambda mo: mo.group(1) + repr(math.nextafter(float(mo.group(2)), math.inf)), text,
                     count=1)
    if n != 1:
        raise RunError("the world has no <ixx> element")
    return out


# ---- log analysis ---------------------------------------------------------------------------------------------------

def _raw_read(step):
    return struct.pack("<13d", *step["gz_pos_enu"], *step["gz_q_wxyz"], *step["gz_lin_vel_enu"], *step["gz_ang_vel_enu"])


def _wrench_nonzero(applied):
    return any(c != 0.0 for c in (*applied["force_enu"], *applied["torque_enu"]))


def fresh_steps(log):
    """Indices of the fresh steps: step 0 and every step whose raw gz read differs bitwise from the previous step's."""
    fresh, prev = [], None
    for i, s in enumerate(log["steps"]):
        raw = _raw_read(s)
        if prev is None or raw != prev:
            fresh.append(i)
        prev = raw
    return fresh


def stale_report(log):
    """{"stale": count, "steps": n, "unchanged_zero_wrench": count, "last_fresh_index", "last_fresh_time_ns"}."""
    steps, applied = log["steps"], log["applied"]
    fresh = set(fresh_steps(log))
    stale = sum(1 for i in range(1, len(steps)) if i not in fresh and _wrench_nonzero(applied[i - 1]))
    zero = sum(1 for i in range(1, len(steps)) if i not in fresh and not _wrench_nonzero(applied[i - 1]))
    last = max(fresh) if fresh else None
    return {"stale": stale, "steps": len(steps), "unchanged_zero_wrench": zero, "last_fresh_index": last,
            "last_fresh_time_ns": steps[last]["sim_time_ns"] if last is not None else None}


def locate_offset(data, offset):
    """Where a byte offset of a log file lies: {"region": "header"|"STEP"|"TICK"|"APPLIED"|"TRAILER"|"beyond", "record":
    index of the record among all records, "step": index among the STEP records, "field", "field_byte"}. For a STEP record
    the field is one of the 13-double gz read or plant body groups, "iteration" or "sim_time_ns"."""
    header = lockstep_log._header(data)
    if offset < header["header_size"]:
        return {"region": "header", "record": None, "step": None, "field": None, "field_byte": offset}
    pos, record, steps = header["header_size"], 0, 0
    names = {lockstep_log.STEP: "STEP", lockstep_log.TICK: "TICK", lockstep_log.APPLIED: "APPLIED",
             lockstep_log.TRAILER: "TRAILER"}
    while pos < len(data):
        kind = data[pos]
        size = 1 + lockstep_log._SIZES[kind]
        if offset < pos + size:
            rel = offset - pos
            field, fbyte = "type", 0
            if rel > 0 and kind == lockstep_log.STEP:
                rel -= 1
                if rel < DOUBLE:
                    field, fbyte = "iteration", rel
                elif rel < 2 * DOUBLE:
                    field, fbyte = "sim_time_ns", rel - DOUBLE
                else:
                    rel -= 2 * DOUBLE
                    for name, n in STEP_GZ_FIELDS:
                        if rel < n * DOUBLE:
                            field, fbyte = f"{name}[{rel // DOUBLE}]", rel % DOUBLE
                            break
                        rel -= n * DOUBLE
            elif rel > 0:
                field, fbyte = "payload", rel - 1
            return {"region": names[kind], "record": record, "step": steps if kind == lockstep_log.STEP else None,
                    "field": field, "field_byte": fbyte}
        pos += size
        record += 1
        steps += kind == lockstep_log.STEP
    return {"region": "beyond", "record": None, "step": None, "field": None, "field_byte": offset}


def first_difference(a, b):
    """Offset of the first differing byte of two byte strings (the shorter's length if one is a prefix), None if equal."""
    if a == b:
        return None
    n = min(len(a), len(b))
    lo, hi = 0, n  # invariant: a[:lo] == b[:lo], and the first difference is in [lo, hi] or absent
    while lo < hi:
        mid = (lo + hi) // 2
        if a[lo:mid + 1] == b[lo:mid + 1]:
            lo = mid + 1
        else:
            hi = mid
    return lo


# ---- versions and provenance ----------------------------------------------------------------------------------------

def _dpkg_query(*args):
    if shutil.which("dpkg-query") is None:
        return None
    try:
        r = subprocess.run(["dpkg-query", *args], capture_output=True, text=True, timeout=60)
    except (OSError, subprocess.TimeoutExpired):
        return None
    return r.stdout.strip() if r.returncode == 0 and r.stdout.strip() else None


def _dpkg_version(package):
    out = _dpkg_query("-W", "-f=${Version}", package)
    return f"{out} (dpkg {package})" if out else UNKNOWN


def dart_version():
    """The DART the dartsim plugin links: ldd on the engine plugin, dpkg -S on the libdart it resolves, dpkg -W."""
    plugins = sorted(glob.glob(DARTSIM_PLUGIN_GLOB))
    if not plugins or shutil.which("ldd") is None:
        return UNKNOWN
    try:
        ldd = subprocess.run(["ldd", plugins[0]], capture_output=True, text=True, timeout=60).stdout
    except (OSError, subprocess.TimeoutExpired):
        return UNKNOWN
    m = re.search(r"^\s*libdart\.so\.\S+ => (\S+)", ldd, flags=re.M)
    if not m:
        return UNKNOWN
    real = os.path.realpath(m.group(1))
    owner = _dpkg_query("-S", real)
    if not owner:
        return UNKNOWN
    package = owner.split(":")[0]
    version = _dpkg_query("-W", "-f=${Version}", package)
    return f"{version} (dpkg {package}, {Path(real).name})" if version else UNKNOWN


def plugin_build_type(plugin_dir):
    for d in [Path(plugin_dir).resolve(), *Path(plugin_dir).resolve().parents]:
        cache = d / "CMakeCache.txt"
        if cache.exists():
            m = re.search(r"^CMAKE_BUILD_TYPE:[A-Z]+=(.*)$", cache.read_text(encoding="utf-8", errors="replace"), flags=re.M)
            return (m.group(1) or "(empty)") if m else UNKNOWN
    return UNKNOWN


def versions(gz_sim_version, plugin_dir):
    plugin = Path(plugin_dir) / PLUGIN_FILE
    return {
        "gz-sim": f"{gz_sim_version} (log header)", "gz-physics": _dpkg_version(DPKG["gz-physics"]),
        "DART": dart_version(), "sdformat": _dpkg_version(DPKG["sdformat"]),
        "physics engine": f"{gen_world.PHYSICS_TYPE}sim ({gen_world.PHYSICS_ENGINE_FILENAME}), named in the world",
        "plugin build type": plugin_build_type(plugin_dir),
        "plugin sha256": sha256_file(plugin) if plugin.exists() else UNKNOWN,
    }


# ---- running --------------------------------------------------------------------------------------------------------

def _resolve(path):
    return str(Path(path).resolve())


def _with_seed(text, seed, scenario_seed):
    if seed == scenario_seed:
        return text
    out, n = re.subn(r"(<seed>)[^<]*(</seed>)", rf"\g<1>{seed}\g<2>", text)
    if n != 1:
        raise RunError(f"the world holds {n} <seed> elements, expected exactly 1")
    return out


def _run(card, scenario, seed, m, mode, hover, out_dir, plugin_dir, sdf_edit, iterations, root, budget, timeout_s,
         sensors=None):
    card, scenario, out_dir = Path(card), Path(scenario), Path(out_dir)
    doc = scn.load(scenario, card)
    vals = scn.values(doc)
    seed = vals["seed"] if seed is None else seed
    tick = scn.tick_period_s(doc)
    n = scn.iterations(doc, m) if iterations is None else iterations
    name, _ = gen_world.generate(card, scenario, mode, m, hover, None, root)
    base = f"{Path(name).stem}_seed{seed}"
    out_dir.mkdir(parents=True, exist_ok=True)
    log_path, world_path = out_dir / f"{base}.bin", out_dir / f"{base}.sdf"
    _, generated = gen_world.generate(card, scenario, mode, m, hover, _resolve(log_path), root, sensors=sensors)
    text = _with_seed(generated, seed, vals["seed"])
    if sdf_edit is not None:
        text = sdf_edit(text)
    world_path.write_text(text, encoding="utf-8")
    log_path.unlink(missing_ok=True)
    proc = run_gz_process(world_path, n, seed, plugin_dir, None, timeout_s)
    log, log_error = None, None
    try:
        log = lockstep_log.read(log_path)
    except (OSError, lockstep_log.LogError) as e:
        log_error = str(e)
    warns = check_exit(proc.returncode, proc.stderr, log, log_error, n, m)
    for w in warns:
        pywarnings.warn(w, RuntimeWarning, stacklevel=3)
    return RunResult(
        card=str(card), scenario=str(scenario), scenario_name=name, seed=seed, m=m, mode=mode, hover=hover, iterations=n,
        root=str(root), budget=str(budget), plugin_dir=str(plugin_dir), world_path=str(world_path),
        log_path=str(log_path), report_path=None, returncode=proc.returncode, warnings=warns, log=log,
        log_sha256=sha256_file(log_path), world_sha256=sha256_text(text), sdf_edited=text != generated,
        scenario_sha256=sha256_file(scenario), wall_s=proc.wall_s, cpu_s=proc.cpu_s, sim_s=float(n * m * tick),
        tick_period_s=tick, stale=stale_report(log), stderr_tail=stderr_tail(proc.stderr), sensors=sensors)


def run(card, scenario, seed, m, mode="test", hover=None, out_dir=None, plugin_dir=DEFAULT_PLUGIN_DIR, sdf_edit=None,
        *, iterations=None, halving=None, root=ROOT, budget=None, timeout_s=TIMEOUT_S, sensors=None):
    """Generate the world, run one gz process, check it, parse the log; write the run report beside the log. `seed` None
    means the scenario's seed. `sdf_edit` is a callable on the SDF text (after the seed edit, before the world is written).
    `iterations` overrides the scenario's duration (measure_rtf uses it; the log's trailer is then checked against it).
    `halving` is the T4 rule's output, a dict printed in the report's halving section. `sensors` is a gen_world.SensorSet
    or None (module docstring)."""
    if out_dir is None:
        raise ValueError("run() needs an out_dir")
    r = _run(card, scenario, seed, m, mode, hover, out_dir, plugin_dir, sdf_edit, iterations, root,
             budget or Path(root) / "design" / "budget.yaml", timeout_s, sensors)
    r.report_path = str(Path(r.log_path).with_suffix(".report.txt"))
    write_report([r], halving, r.report_path)
    return r


def run_sequence(card, scenario, seed, mode="test", hover=None, out_dir=None, plugin_dir=DEFAULT_PLUGIN_DIR,
                 sdf_edit=None, *, halving=None, root=ROOT, budget=None, timeout_s=TIMEOUT_S, sensors=None):
    """run() for every m of the scenario's m_sequence, one gz process (one SIL) per m; one sequence report."""
    if out_dir is None:
        raise ValueError("run_sequence() needs an out_dir")
    doc = scn.load(scenario, card)
    budget = budget or Path(root) / "design" / "budget.yaml"
    runs = [_run(card, scenario, seed, m, mode, hover, out_dir, plugin_dir, sdf_edit, None, root, budget, timeout_s,
                 sensors) for m in scn.values(doc)["m_sequence"]]
    first = runs[0]
    stem = re.sub(r"_m\d+$", "", Path(first.world_path).stem)
    path = str(Path(out_dir) / f"{stem}_sequence.report.txt")
    for r in runs:
        r.report_path = path
    write_report(runs, halving, path)
    return SequenceResult(runs, path)


# ---- the run report -------------------------------------------------------------------------------------------------

def _fmt_value(v):
    if isinstance(v, float):
        return repr(v)
    if isinstance(v, dict):
        return "{" + ", ".join(f"{k}: {_fmt_value(x)}" for k, x in v.items()) + "}"
    if isinstance(v, (list, tuple)):
        return "[" + ", ".join(_fmt_value(x) for x in v) + "]"
    return str(v)


def _mapping_lines(d, indent):
    pad = " " * indent
    lines = []
    for k, v in d.items():
        if isinstance(v, dict) and v:
            lines.append(f"{pad}{k}:")
            lines.extend(_mapping_lines(v, indent + 2))
        else:
            lines.append(f"{pad}{k}: {_fmt_value(v)}")
    return lines


def _tick_text(tick):
    return f"{tick.numerator}/{tick.denominator} s ({repr(float(tick * 10**6))} us)"


def _run_block(r, ver):
    h = r.log["header"]
    trailer = r.log["trailer"]
    st = r.stale
    last = st["last_fresh_index"]
    stale_time = (f"{st['last_fresh_time_ns']} ns = {repr(st['last_fresh_time_ns'] / 1e9)} s"
                  if last is not None else "none")
    sensors = r.sensors
    model = sensors is not None and sensors.gyro == gen_world.GYRO_SOURCE_MODEL
    seed_line = (f"  seed: {r.seed} (the plant IMU model draws from its noise stream 0, decision 0019); log header seed "
                 f"{h['seed']}" if model else
                 f"  seed: {r.seed} (RESERVED at v0: marv_plant v0 and the composition draw no random numbers); log header "
                 f"seed {h['seed']}")
    sensor_lines = [] if sensors is None else [
        f"  sensors (decision 0019): gyro {sensors.gyro or 'not set by the sensors'}"
        + (f", turn-on bias signs {list(sensors.bias_signs)}" if model else "")
        + f", rotor speed {'on' if sensors.rotor else 'off'}, clock corner "
        + ("none" if sensors.clock_corner is None else f"{sensors.clock_corner:+d}")
        + (f"; record 6: host step {r.log['sensors']['host_step_ns']} ns, plant tick {r.log['sensors']['tick_s']!r} s"
           if r.log.get("sensors") else "")]
    lines = [
        f"run: m = {r.m}" + (f", hover {r.hover}" if r.hover else "") + f", mode {r.mode}",
        f"  world: {r.world_path}",
        f"  world sha256: {r.world_sha256}" + ("  (sdf_edit changed the generated world)" if r.sdf_edited else ""),
        seed_line,
        *sensor_lines,
        f"  m (ticks per host step): {r.m}",
        f"  tick period: {h['tick_period_num_us']}/{h['tick_period_den']} us = {_tick_text(r.tick_period_s)}",
        f"  host step H = m * tick = {_tick_text(r.tick_period_s * r.m)}",
        f"  host steps: {r.iterations}; ticks: {r.iterations * r.m}",
        "  versions:",
        *[f"    {k}: {v}" for k, v in ver.items()],
        f"  gz exit code: {r.returncode}",
        f"  warnings ({len(r.warnings)}):",
        *[f"    {w}" for w in r.warnings or ["(none)"]],
        f"  log: {r.log_path}",
        f"  log sha256: {r.log_sha256}",
        f"  log records: steps {len(r.log['steps'])}, ticks {len(r.log['ticks'])}, applied {len(r.log['applied'])}; "
        f"trailer {_fmt_value(trailer)}",
        f"  stale steps: {st['stale']} of {st['steps']} (raw gz read bitwise equal to the previous step's while the "
        f"applied wrench was nonzero; 0003 item 11); unchanged with a zero wrench: {st['unchanged_zero_wrench']}",
        f"  last fresh step: index {last if last is not None else 'none'}, time {stale_time}",
        f"  simulated time: {repr(r.sim_s)} s",
        f"  wall time: {r.wall_s:.3f} s (gz process, including its startup)",
        f"  RTF: {r.rtf:.3f} (simulated / wall); child CPU time {r.cpu_s:.3f} s",
    ]
    return lines


def render_report(runs, halving=None):
    """The run report text of one run or of a sequence (runs sharing card, scenario and seed)."""
    r0 = runs[0]
    root, card, budget = Path(r0.root), Path(r0.card), Path(r0.budget)
    card_doc = schema.load_yaml(card)
    files = [card, *card_report.profile_files(card_doc, root)]
    ver = versions(r0.log["header"]["gz_sim_version"], r0.plugin_dir)
    out = [
        "MARV L2 run report",
        "",
        PERFECT_MODEL,
        f"git commit: {card_report.generator_commit(root)}",
        f"card: {card_report._rel(card, root)}",
        f"card hash: {card_report.framed_hash(files, root)}",
        f"scenario: {card_report._rel(r0.scenario, root)}",
        f"scenario sha256: {r0.scenario_sha256}",
        f"seed: {r0.seed}",
        f"m sequence run: {[r.m for r in runs]}",
        "",
    ]
    for r in runs:
        out.extend(_run_block(r, ver))
        out.append("")
    out.append("halving")
    out.extend(_mapping_lines(halving, 2) if halving else ["  (not provided: the T4 rule's output goes here)"])
    out.append("")
    out.append("card and budget report (tools/card/report.py build_report)")
    out.append("")
    out.append(card_report.build_report(card, budget, root).rstrip("\n"))
    return "\n".join(out) + "\n"


def write_report(runs, halving=None, path=None):
    """Render and write the run report; `path` defaults to the first run's report_path. Returns the text."""
    text = render_report(runs, halving)
    path = path or runs[0].report_path
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(text, encoding="utf-8", newline="\n")
    return text


# ---- SIM-3 ----------------------------------------------------------------------------------------------------------

def batch_plan_text(budget):
    entry = schema.load_yaml(budget).get("batch_plan", {})
    value = entry.get("value", UNKNOWN)
    if value == UNKNOWN:
        return "required: UNKNOWN (design/budget.yaml batch_plan)"
    return (f"required: batch_plan value {_fmt_value(value)} is set, but the rule of quad spec 3.3 SIM-3 is not implemented "
            "in the runner: UNKNOWN (design/budget.yaml batch_plan)")


def measure_rtf(card, scenario, m, seed=None, out_dir=None, plugin_dir=DEFAULT_PLUGIN_DIR, *, hover=None,
                iterations=None, root=ROOT, budget=None, timeout_s=TIMEOUT_S):
    """SIM-3: one gz process per measurement, a full run and a STARTUP_ITERATIONS run of the test world."""
    if out_dir is None:
        raise ValueError("measure_rtf() needs an out_dir")
    budget = budget or Path(root) / "design" / "budget.yaml"
    full = run(card, scenario, seed, m, "test", hover, Path(out_dir) / "full", plugin_dir, iterations=iterations,
               root=root, budget=budget, timeout_s=timeout_s)
    short = run(card, scenario, seed, m, "test", hover, Path(out_dir) / "startup", plugin_dir,
                iterations=STARTUP_ITERATIONS, root=root, budget=budget, timeout_s=timeout_s)
    d_sim, d_wall = full.sim_s - short.sim_s, full.wall_s - short.wall_s
    marginal = d_sim / d_wall if d_wall > 0 else None
    marginal_cpu = (full.cpu_s - short.cpu_s) / d_wall if d_wall > 0 else None
    return Sim3Result(
        card=str(card), scenario=str(scenario), m=m, seed=full.seed, iterations=full.iterations,
        startup_iterations=STARTUP_ITERATIONS, sim_s=full.sim_s, wall_s=full.wall_s, cpu_s=full.cpu_s,
        startup_sim_s=short.sim_s, startup_wall_s=short.wall_s, startup_cpu_s=short.cpu_s, rtf_wall=full.rtf,
        rtf_marginal=marginal, cpu_over_wall=full.cpu_s / full.wall_s,
        marginal_cpu_over_wall=marginal_cpu, cores=len(os.sched_getaffinity(0)),
        plugin_build_type=plugin_build_type(plugin_dir), required=batch_plan_text(budget),
        comparison="UNKNOWN, SIM-3 open")


def format_sim3(s):
    marginal = f"{s.rtf_marginal:.3f}" if s.rtf_marginal is not None else "n/a (the full run was not slower than the startup run)"
    net_wall = s.wall_s - s.startup_wall_s
    lines = [
        "SIM-3 throughput (0003 item 2)",
        f"  scenario {Path(s.scenario).stem}, m = {s.m}, seed {s.seed}, test world, one gz process, plugin build type "
        f"{s.plugin_build_type}",
        "  assumption: one process = one core; backed by the child CPU time, resource.getrusage(RUSAGE_CHILDREN)",
        f"  full run: {s.iterations} host steps, simulated {s.sim_s:.6f} s, wall {s.wall_s:.3f} s, CPU {s.cpu_s:.3f} s",
        f"  startup run: {s.startup_iterations} host steps, simulated {s.startup_sim_s:.6f} s, wall {s.startup_wall_s:.3f} s, "
        f"CPU {s.startup_cpu_s:.3f} s",
        f"  startup overhead (wall of the {s.startup_iterations}-iteration run): {s.startup_wall_s:.3f} s",
        f"  RTF, simulated / wall, startup included: {s.rtf_wall:.3f}",
        f"  RTF per core, marginal (extra simulated / extra wall of the full run over the startup run, "
        f"{net_wall:.3f} s): {marginal}",
        f"  CPU time / wall time of the full run: {s.cpu_over_wall:.3f} "
        + ("(<= 1: consistent with one core)" if s.cpu_over_wall <= 1.0 else
           "(> 1: the process used more than one core; the one-process-one-core assumption FAILS)"),
        "  CPU / wall of the extra work (full run minus startup run): "
        + (f"{s.marginal_cpu_over_wall:.3f}" if s.marginal_cpu_over_wall is not None else "n/a"),
        f"  cores available to this process: {s.cores}",
        f"  {s.required}",
        f"  comparison: {s.comparison}",
    ]
    return "\n".join(lines) + "\n"


# ---- CLI ------------------------------------------------------------------------------------------------------------

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--card", required=True, metavar="FILE")
    ap.add_argument("--scenario", required=True, metavar="FILE")
    ap.add_argument("--seed", type=int, help="default: the scenario's seed")
    ap.add_argument("--m", type=int, help="ticks per host step; needed unless --sequence")
    ap.add_argument("--sequence", action="store_true", help="run the scenario's m_sequence, one process per m")
    ap.add_argument("--mode", choices=gen_world.MODES, default="test")
    ap.add_argument("--hover", choices=scn.HOVER_MEMBERS)
    ap.add_argument("--out-dir", required=True, metavar="DIR")
    ap.add_argument("--plugin-dir", default=str(DEFAULT_PLUGIN_DIR), metavar="DIR")
    ap.add_argument("--measure-rtf", action="store_true", help="SIM-3 measurement at --m (needs --m)")
    ap.add_argument("--iterations", type=int, help="with --measure-rtf: host steps of the full run (default: the scenario's)")
    args = ap.parse_args(argv)
    if not (args.sequence or args.m):
        ap.error("--m or --sequence is required")
    if args.sequence and (args.m or args.measure_rtf):
        ap.error("--sequence excludes --m and --measure-rtf")
    if args.iterations and not args.measure_rtf:
        ap.error("--iterations goes with --measure-rtf")
    if args.measure_rtf and args.mode != "test":
        ap.error("--measure-rtf runs the test world")
    try:
        if args.measure_rtf:
            sys.stdout.write(format_sim3(measure_rtf(args.card, args.scenario, args.m, args.seed, args.out_dir,
                                                     args.plugin_dir, hover=args.hover,
                                                     iterations=args.iterations)))
        elif args.sequence:
            res = run_sequence(args.card, args.scenario, args.seed, args.mode, args.hover, args.out_dir,
                               args.plugin_dir)
            print(f"report: {res.report_path}")
            for r in res.runs:
                print(f"m = {r.m}: log {r.log_path} sha256 {r.log_sha256}")
        else:
            r = run(args.card, args.scenario, args.seed, args.m, args.mode, args.hover, args.out_dir, args.plugin_dir)
            print(f"report: {r.report_path}\nlog: {r.log_path}\nlog sha256: {r.log_sha256}")
    except (gpc.GenError, scn.ScenarioError) as e:
        for line in e.lines:
            print(line, file=sys.stderr)
        return 1
    except (hover_mod.HoverError, RunError) as e:
        print(f"run_scenario: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
