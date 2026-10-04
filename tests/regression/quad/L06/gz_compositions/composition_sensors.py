"""Shared helpers of the L4 and L5 sensor tests (decision 0019, W1 C3): test_l4_sensors.py and test_l5_sensors.py.

  capture_world   the world a driver call (run_l4 / run_l5) writes, taken before its gz process: run_gz_process is
                  replaced by a stub that reads the world file and stops the call, so no gz process starts.
  world_problems  what differs between a sensors world of a driver and its default world besides what the SensorSet
                  inserts (test (a)(ii) of C1, tests/regression/quad/L06/tools/test_world_sensors.py, on the drivers'
                  worlds; the rule is in its docstring).
  masked_log      a log with the bytes that two turn-on corners differing only in the accelerometer signs may differ
                  in zeroed (test e; the masked fields are in its docstring).
  accel_readers   the files of a SIL library's compile closure that name an accelerometer field or flag (the static
                  guard of test e).
"""

import json
import math
import re
import shlex
import struct
import subprocess
import sys
import xml.etree.ElementTree as ET
from difflib import SequenceMatcher
from fractions import Fraction
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
sys.path.insert(0, str(ROOT / "tools" / "sim"))
sys.path.insert(0, str(ROOT / "tests" / "regression" / "quad" / "L06" / "tools"))
import gen_world  # noqa: E402
import lockstep_log  # noqa: E402
import run_l4  # noqa: E402
import run_scenario  # noqa: E402
import scenario as scn  # noqa: E402
import test_world_sensors as tws  # noqa: E402

CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
SENSOR_SETS = tws.SENSOR_SETS  # the SensorSets of test (a)(ii)
NEW_TAGS = tws.NEW_TAGS
TRUTH_GYRO_LINE = run_l4.GYRO_ELEMENT
PLUGIN_XPATH = ".//plugin[@filename='marv_gz_lockstep']"


def set_id(s):
    return f"{s.gyro}-{s.rotor}-{s.clock_corner}"


# ---- the drivers' worlds ---------------------------------------------------------------------------------------------

class _Captured(Exception):
    def __init__(self, text):
        super().__init__("world captured")
        self.text = text


def capture_world(call):
    """The text of the world file that `call()` (a run_l4 / run_l5 entry point) writes before it starts gz."""
    def stub(sdf_path, *args, **kwargs):
        raise _Captured(Path(sdf_path).read_text(encoding="utf-8"))
    with pytest.MonkeyPatch.context() as mp:
        mp.setattr(run_scenario, "run_gz_process", stub)
        try:
            call()
        except _Captured as c:
            return c.text
    raise AssertionError("the driver call wrote no world (no gz process was started)")


def l2_doc(out_dir):
    """The L2-schema scenario the driver wrote into out_dir (the input of its world)."""
    paths = sorted(Path(out_dir).glob("*.yaml"))
    assert len(paths) == 1, paths
    return scn.load(paths[0], CARD)


def world_problems(default, text, s, doc, m):
    """What differs between the driver's sensors world `text` (SensorSet s) and its default world besides: the inserted
    elements tws.expected_tags(s), children of the lockstep plugin element; the truth-gyro element removed iff s.gyro is
    the model (gen_world writes the model's element instead); max_step_size iff s.clock_corner is nonzero, then
    fl(n_true / 1e9). [] if nothing else differs."""
    a, b = default.splitlines(), text.splitlines()
    deleted, inserted, problems = [], [], []
    for op, i1, i2, j1, j2 in SequenceMatcher(None, a, b, autojunk=False).get_opcodes():
        if op != "equal":
            deleted.extend(a[i1:i2])
            inserted.extend(b[j1:j2])
    step_old = [x for x in deleted if "<max_step_size>" in x]
    step_new = [x for x in inserted if "<max_step_size>" in x]
    if s.clock_corner:
        n = gen_world.realised_host_step_ns(doc, m, s.clock_corner, tws.odr_error())
        want = f"<max_step_size>{float(Fraction(n, 10 ** 9))!r}</max_step_size>"
        if len(step_old) != 1 or [x.strip() for x in step_new] != [want]:
            problems.append(f"max_step_size {step_old} -> {step_new}, expected one line -> {want}")
    elif step_old or step_new:
        problems.append(f"max_step_size changed without a nonzero clock corner: {step_old} -> {step_new}")
    deleted = [x.strip() for x in deleted if "<max_step_size>" not in x]
    inserted = [x for x in inserted if "<max_step_size>" not in x]
    want_deleted = [TRUTH_GYRO_LINE] if s.gyro == gen_world.GYRO_SOURCE_MODEL else []
    if deleted != want_deleted:
        problems.append(f"deleted lines {deleted}, expected {want_deleted}")
    try:
        got = [e.tag for e in ET.fromstring("<x>" + "\n".join(inserted) + "</x>")] if inserted else []
    except ET.ParseError as e:
        got = [f"(the inserted lines are not whole elements: {e})"]
    if got != tws.expected_tags(s):
        problems.append(f"inserted elements {got}, expected {tws.expected_tags(s)}")
    plugin = ET.fromstring(text).find(PLUGIN_XPATH)
    if [c.tag for c in plugin if c.tag in NEW_TAGS and c.text != "truth"] != tws.expected_tags(s):
        problems.append("the inserted elements are not the lockstep plugin's children")
    return problems


def default_world_problems(text):
    """The default world of a driver holds exactly one truth-gyro element and none of the sensor elements."""
    problems = []
    if text.count(TRUTH_GYRO_LINE) != 1:
        problems.append(f"{text.count(TRUTH_GYRO_LINE)} truth-gyro elements, expected 1")
    plugin = ET.fromstring(text).find(PLUGIN_XPATH)
    extra = [c.tag for c in plugin if c.tag in NEW_TAGS and c.tag != "gyro_source"]
    if extra:
        problems.append(f"sensor elements {extra} in the default world")
    return problems


# ---- the log comparison of test e --------------------------------------------------------------------------------------

# Byte ranges inside a record's payload (after its type byte), from lockstep_log's layouts.
_SENSORS_PREFIX = "<BBBbdQdQIQI"  # four u8, odr_error, host_step_ns, tick_s, seed, stream id, counter base, latency
SENSORS_ACCEL_BIAS = (struct.calcsize(_SENSORS_PREFIX + "7d4d"), struct.calcsize(_SENSORS_PREFIX + "7d7d"))
SENSORS_ACCEL_SIGNS = (struct.calcsize(_SENSORS_PREFIX + "7d7d2d3b"), struct.calcsize(_SENSORS_PREFIX + "7d7d2d6b"))
TICK_IMU = struct.calcsize("<QQ")  # the 32 bytes of marv_imu_meas after tick and sil_t_us
# marv_imu_meas (fw/sil/include/marv_sil.h): gyro_rad_s x y z, accel_m_s2 x y z, temp_k (f32 each), flags (u32).
F32 = struct.calcsize("<f")
IMU_ACCEL = (struct.calcsize("<3f"), struct.calcsize("<6f"))
IMU_FLAGS = struct.calcsize("<7f")
AXES = (IMU_ACCEL[1] - IMU_ACCEL[0]) // F32
TICK_DSHOT = (struct.calcsize("<QQ32s"), struct.calcsize("<QQ32s4H"))
ACCEL_VALID = 1 << 7  # marv_sil.h MARV_IMU_ACCEL_VALID: the enum position of AccelValid (fw/types/imu_sample.hpp)
NAMES = {lockstep_log.STEP: "STEP", lockstep_log.TICK: "TICK", lockstep_log.APPLIED: "APPLIED",
         lockstep_log.TRAILER: "TRAILER", lockstep_log.TRUTH: "TRUTH", lockstep_log.SENSORS: "SENSORS",
         lockstep_log.ROTOR: "ROTOR"}


def records(data):
    """[(type, payload offset)] of every record of a log, in order."""
    pos = lockstep_log._header(data)["header_size"]
    out = []
    while pos < len(data):
        kind = data[pos]
        if kind not in lockstep_log._SIZES:
            raise lockstep_log.LogError(f"unknown record type {kind} at byte {pos}")
        out.append((kind, pos + 1))
        pos += 1 + lockstep_log._SIZES[kind]
    if pos != len(data):
        raise lockstep_log.LogError("the last record is cut")
    return out


def masked_log(data):
    """The log with the accelerometer bytes zeroed: record 6's accel turn-on biases and accel signs, and every TICK
    record's accel_m_s2 (12 bytes of its IMU sample). Every other byte is kept."""
    b = bytearray(data)
    for kind, at in records(data):
        if kind == lockstep_log.SENSORS:
            for lo, hi in (SENSORS_ACCEL_BIAS, SENSORS_ACCEL_SIGNS):
                b[at + lo:at + hi] = bytes(hi - lo)
        elif kind == lockstep_log.TICK:
            lo, hi = at + TICK_IMU + IMU_ACCEL[0], at + TICK_IMU + IMU_ACCEL[1]
            b[lo:hi] = bytes(hi - lo)
    return bytes(b)


def where(data, offset):
    """(record index, record name, byte inside its payload) of a byte offset of a log; 'header' before the records."""
    if offset < lockstep_log._header(data)["header_size"]:
        return ("header", None, offset)
    for i, (kind, at) in enumerate(records(data)):
        if offset < at + lockstep_log._SIZES[kind]:
            return (i, NAMES[kind], offset - at)
    return ("beyond", None, offset)


def ticks(data):
    """The payload offsets of the TICK records of a log."""
    return [at for kind, at in records(data) if kind == lockstep_log.TICK]


def accel_differs_every_tick(a, b):
    """(ticks, ticks at which some accel axis is bitwise equal in a and b): a and b must have the same TICK layout."""
    ta, tb = ticks(a), ticks(b)
    assert ta == tb
    same = 0
    for at in ta:
        for axis in range(AXES):
            lo = at + TICK_IMU + IMU_ACCEL[0] + axis * F32
            if a[lo:lo + F32] == b[lo:lo + F32]:
                same += 1
                break
    return len(ta), same


def accel_valid_and_finite(data):
    """The ticks whose IMU sample has AccelValid clear or a non-finite accel axis (the SIL's boundary check of the
    accelerometer, fw/sil/src/marv_sil.cpp sample_valid, passes on every other tick)."""
    bad = 0
    for at in ticks(data):
        imu = at + TICK_IMU
        flags = struct.unpack_from("<I", data, imu + IMU_FLAGS)[0]
        accel = struct.unpack_from("<3f", data, imu + IMU_ACCEL[0])
        bad += not (flags & ACCEL_VALID) or not all(math.isfinite(x) for x in accel)
    return bad


def dshot_differs(a, b):
    """The number of ticks whose DShot differs between two logs with the same TICK layout, and the first such tick."""
    n, first = 0, None
    for i, (x, y) in enumerate(zip(ticks(a), ticks(b))):
        if a[x + TICK_DSHOT[0]:x + TICK_DSHOT[1]] != b[y + TICK_DSHOT[0]:y + TICK_DSHOT[1]]:
            n += 1
            first = i if first is None else first
    return n, first


# ---- the static guard: the SIL library's readers of the accelerometer ------------------------------------------------

ACCEL_NAMES = re.compile(r"\baccel_m_s2\b|\bAccel(?:Sat[XYZ]|Valid)\b|\bMARV_IMU_ACCEL_\w+")
# The SIL boundary (fw/sil): the C struct, the sample type, and marv_sil.cpp's validity check of each sample
# (sample_valid, all_finite) and its copy of the sample into the composition's ImuSample (to_sample).
BOUNDARY = {"fw/sil/include/marv_sil.h", "fw/types/include/marv/types/imu_sample.hpp", "fw/sil/src/marv_sil.cpp"}


def cache_value(build, key):
    m = re.search(rf"^{re.escape(key)}:[A-Z]+=(.*)$", (Path(build) / "CMakeCache.txt").read_text(encoding="utf-8"),
                  flags=re.M)
    return m.group(1) if m else None


def sil_library(build, name):
    """The linked file of the SIL library `name` of a build tree (its G3 manifest, cmake/flight_targets.cmake)."""
    manifest = json.loads((Path(build) / "g3_manifest.json").read_text(encoding="utf-8"))
    files = [s["file"] for s in manifest["sil_libraries"] if s["name"] == name]
    assert len(files) == 1, (name, files)
    return files[0]


def _ninja(build, *args):
    r = subprocess.run(["ninja", "-C", str(build), "-t", *args], capture_output=True, text=True, check=False)
    assert r.returncode == 0, r.stderr
    return r.stdout


def compile_closure(build, target):
    """Every repository file compiled into `target` (a file of the build tree): the sources of the compile commands
    ninja runs for it and every header their recorded dependencies name (ninja -t commands, ninja -t deps). A missing
    or stale dependency record fails."""
    build = Path(build).resolve()
    target = Path(target)
    objects = []
    for line in _ninja(build, "commands", str(target.relative_to(build) if target.is_absolute() else target)).splitlines():
        args = shlex.split(line)
        if "-c" in args and "-o" in args:
            objects.append(args[args.index("-o") + 1])
    assert objects, f"no compile command for {target}"
    files = set()
    for obj in objects:
        lines = _ninja(build, "deps", obj).splitlines()
        assert lines and lines[0].endswith("(VALID)"), (obj, lines[:1])
        for dep in (x.strip() for x in lines[1:]):
            path = (build / dep).resolve()  # an absolute dep stays itself
            if dep and path.is_relative_to(ROOT):
                files.add(path.relative_to(ROOT).as_posix())
    return files


def accel_readers(build, sil_name):
    """(closure, {file: [line numbers]}): the files of the SIL library's compile closure that name an accelerometer
    field or flag (ACCEL_NAMES)."""
    closure = compile_closure(build, sil_library(build, sil_name))
    readers = {}
    for rel in sorted(closure):
        lines = [i for i, line in enumerate((ROOT / rel).read_text(encoding="utf-8", errors="replace").splitlines(), 1)
                 if ACCEL_NAMES.search(line)]
        if lines:
            readers[rel] = lines
    return closure, readers
