"""Scenario runner (tools/sim/run_scenario.py) without gz. L02 tests, host-only.

Logs are synthetic, written here with the structs of tools/sim/lockstep_log.py (the format's reader; the plugin's
lockstep_log.hpp is its home). A fake `run_gz_process` stands in for the gz process: it reads the world's <log_path>, writes a
synthetic log there and returns a chosen exit code, so run() and run_sequence() are exercised end to end (world
generation, exit-code rule, parsing, hashing, the report file) with no gz.

What is checked, each with a negative control: the stale-step and fresh-step rule of 0003 item 11 (bitwise, and only
with a nonzero applied wrench); the exit-code rule (a nonzero exit fails unless SIGSEGV after a complete log, and then
it is a warning in the result, in a RuntimeWarning and in the report); the offset mapper the determinism control uses;
the report's contents (identity, seed, m, tick rational and H, versions with UNKNOWN when dpkg is missing, stale
steps, halving section, the build_report text); and the SIM-3 text (required and comparison stay UNKNOWN).
"""

import hashlib
import math
import re
import signal
import struct
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
sys.path.insert(0, str(ROOT / "tools" / "sim"))
import lockstep_log as ll  # noqa: E402
import report as card_report  # noqa: E402
import run_scenario as rs  # noqa: E402

CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
SCEN = ROOT / "scenarios" / "quad" / "L02" / "determinism.yaml"
SEQ_SCEN = ROOT / "scenarios" / "quad" / "L02" / "free_fall.yaml"
BUDGET = ROOT / "design" / "budget.yaml"
GZ_VERSION = "8.99.0"
ZERO = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
PUSH = (0.0, 0.0, 1.0, 0.0, 0.0, 0.0)


def header_bytes(m=1, seed=1):
    version = GZ_VERSION.encode("ascii")
    fixed = struct.Struct("<8sIIIIIQI")
    return fixed.pack(ll.MAGIC, ll.VERSION, fixed.size + len(version), m, 625, 4, seed, len(version)) + version


def log_bytes(reads, wrenches, m=1, seed=1, trailer=True):
    """A log of len(reads) host steps: reads[i] is the 13-double gz read of step i, wrenches[i] the applied wrench (6
    doubles) after it."""
    out = bytearray(header_bytes(m, seed))
    tick = 0
    for i, (read, w) in enumerate(zip(reads, wrenches)):
        out += bytes([ll.STEP]) + ll._STEP.pack(i, i * 156250, *read, *([0.0] * 13))
        for _ in range(m):
            out += bytes([ll.TICK]) + ll._TICK.pack(tick, tick * 156, bytes(32), 48, 48, 48, 48, 1, 0, *([0.0] * 20))
            tick += 1
        out += bytes([ll.APPLIED]) + ll._APPLIED.pack(*w)
    if trailer:
        n = len(reads)
        out += bytes([ll.TRAILER]) + ll._TRAILER.pack(n, n * m, n)
    return bytes(out)


def read_of(x):
    return (x, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def canned(tmp_path, reads, wrenches, **kw):
    p = tmp_path / "canned.bin"
    p.write_bytes(log_bytes(reads, wrenches, **kw))
    return p, ll.read(p)


# ---- stale and fresh steps -------------------------------------------------------------------------------------------

def test_stale_rule(tmp_path):
    reads = [read_of(x) for x in (0.0, 1.0, 1.0, 1.0, 2.0, 2.0)]
    wrenches = [PUSH, PUSH, PUSH, ZERO, PUSH, PUSH]
    _, log = canned(tmp_path, reads, wrenches)
    assert rs.fresh_steps(log) == [0, 1, 4]
    st = rs.stale_report(log)
    # step 2: equal to 1, wrench 1 nonzero -> stale; step 3: equal, wrench 2 nonzero -> stale; step 5: equal, wrench 4 nonzero
    assert st["stale"] == 3 and st["unchanged_zero_wrench"] == 0
    assert st["last_fresh_index"] == 4 and st["last_fresh_time_ns"] == 4 * 156250


def test_stale_needs_a_nonzero_wrench(tmp_path):
    """Negative control: an unchanged read under a zero wrench is not stale."""
    reads = [read_of(x) for x in (0.0, 1.0, 1.0, 1.0)]
    _, log = canned(tmp_path, reads, [PUSH, ZERO, ZERO, ZERO])
    st = rs.stale_report(log)
    assert st["stale"] == 0 and st["unchanged_zero_wrench"] == 2


def test_stale_negative_control_all_fresh_and_all_stale(tmp_path):
    fresh = [read_of(float(i)) for i in range(5)]
    _, log = canned(tmp_path, fresh, [PUSH] * 5)
    assert rs.stale_report(log)["stale"] == 0 and rs.stale_report(log)["last_fresh_index"] == 4
    frozen = [read_of(1.0)] * 5
    _, log = canned(tmp_path, frozen, [PUSH] * 5)
    assert rs.stale_report(log)["stale"] == 4 and rs.stale_report(log)["last_fresh_index"] == 0


def test_freshness_is_bitwise(tmp_path):
    """+0.0 and -0.0 are equal as floats and different as bytes, so a step that flips the sign of zero is fresh."""
    a = read_of(0.0)
    b = (-0.0,) + a[1:]
    _, log = canned(tmp_path, [a, b, b], [PUSH] * 3)
    assert rs.fresh_steps(log) == [0, 1]
    assert rs.stale_report(log)["stale"] == 1


# ---- the exit-code rule ----------------------------------------------------------------------------------------------

def full_log(tmp_path, n=4, m=1):
    return canned(tmp_path, [read_of(float(i)) for i in range(n)], [PUSH] * n, m=m)[1]


def test_exit_zero_with_complete_log_is_accepted(tmp_path):
    assert rs.check_exit(0, "", full_log(tmp_path), None, 4, 1) == []


@pytest.mark.parametrize("code", [1, 2, -signal.SIGABRT, -signal.SIGKILL, 134])
def test_nonzero_exit_fails_even_with_a_complete_log(tmp_path, code):
    with pytest.raises(rs.RunError, match=f"exited {code}"):
        rs.check_exit(code, "", full_log(tmp_path), None, 4, 1)


@pytest.mark.parametrize("code", [-signal.SIGSEGV, 128 + signal.SIGSEGV])
def test_sigsegv_after_complete_log_is_a_recorded_warning(tmp_path, code):
    w = rs.check_exit(code, "", full_log(tmp_path), None, 4, 1)
    assert len(w) == 1 and "SIGSEGV" in w[0] and f"exit {code}" in w[0]


def test_sigsegv_without_complete_log_fails(tmp_path):
    p, _ = canned(tmp_path, [read_of(float(i)) for i in range(4)], [PUSH] * 4, trailer=False)
    log = ll.read(p)  # a log without a trailer parses; the failure below is the count check
    with pytest.raises(rs.RunError, match="log complete: False"):
        rs.check_exit(-signal.SIGSEGV, "", log, None, 4, 1)
    with pytest.raises(rs.RunError, match="log complete: False"):
        rs.check_exit(-signal.SIGSEGV, "", None, "file shorter than the header", 4, 1)


def test_sigsegv_with_a_log_of_the_wrong_length_fails(tmp_path):
    with pytest.raises(rs.RunError, match="log complete: False"):
        rs.check_exit(-signal.SIGSEGV, "", full_log(tmp_path, n=3), None, 4, 1)


def test_exit_zero_with_incomplete_log_fails(tmp_path):
    with pytest.raises(rs.RunError, match="not complete"):
        rs.check_exit(0, "", full_log(tmp_path, n=3), None, 4, 1)
    with pytest.raises(rs.RunError, match="not complete"):
        rs.check_exit(0, "", None, "record type 1 cut at byte 46", 4, 1)


def test_refusal_fails_whatever_the_exit_code(tmp_path):
    with pytest.raises(rs.RunError, match="refused"):
        rs.check_exit(0, "marv_gz_lockstep: REFUSED: world gravity is not zero", full_log(tmp_path), None, 4, 1)


def test_stderr_tail_drops_protobuf_noise():
    noise = "[libprotobuf ERROR google/protobuf/descriptor_database.cc:121] File already exists in database: a.proto\n"
    text = noise * 5 + "DynamicFactory(). Unable to place descriptors\nreal line\n"
    assert rs.stderr_tail(text) == "real line"


# ---- offsets ---------------------------------------------------------------------------------------------------------

def test_first_difference_and_locate(tmp_path):
    p, log = canned(tmp_path, [read_of(float(i)) for i in range(3)], [PUSH] * 3)
    data = p.read_bytes()
    hs = log["header"]["header_size"]
    step_size = 1 + ll._SIZES[ll.STEP]
    tick_size = 1 + ll._SIZES[ll.TICK]
    applied_size = 1 + ll._SIZES[ll.APPLIED]
    rec = step_size + tick_size + applied_size

    def flip(off):
        b = bytearray(data)
        b[off] ^= 1
        return bytes(b)

    assert rs.first_difference(data, data) is None
    assert rs.first_difference(data, data[:-3]) == len(data) - 3
    assert rs.locate_offset(data, rs.first_difference(data, flip(5)))["region"] == "header"
    # step 1's gz_q_wxyz[0] is the fifth double after the 1 + 2 * 8 byte record head... (pos 3, then q): offset 17 + 3 * 8
    off = hs + rec + 17 + 3 * 8 + 2
    w = rs.locate_offset(data, rs.first_difference(data, flip(off)))
    assert (w["region"], w["step"], w["field"], w["field_byte"]) == ("STEP", 1, "gz_q_wxyz[0]", 2)
    off = hs + rec + step_size + 20
    assert rs.locate_offset(data, rs.first_difference(data, flip(off)))["region"] == "TICK"
    off = hs + 3 * rec + 1
    assert rs.locate_offset(data, off)["region"] == "TRAILER"
    assert rs.locate_offset(data, hs + 1 + 16 + 13 * 8 * 2 - 1)["field"] == "body_omega_frd[2]"


def test_ixx_one_ulp_is_one_ulp():
    out = rs.ixx_one_ulp("<inertia><ixx>0.0025</ixx><ixy>0</ixy><ixx>7</ixx></inertia>")
    assert out == f"<inertia><ixx>{math.nextafter(0.0025, math.inf)!r}</ixx><ixy>0</ixy><ixx>7</ixx></inertia>"
    with pytest.raises(rs.RunError):
        rs.ixx_one_ulp("<inertia></inertia>")


def test_seed_edit():
    text = "<plugin><seed>1</seed></plugin>"
    assert rs._with_seed(text, 1, 1) == text
    assert rs._with_seed(text, 7, 1) == "<plugin><seed>7</seed></plugin>"
    with pytest.raises(rs.RunError):
        rs._with_seed("<plugin></plugin>", 7, 1)


# ---- run() and the report, with a fake gz ----------------------------------------------------------------------------

@pytest.fixture
def no_dpkg(monkeypatch):
    monkeypatch.setattr(rs, "_dpkg_query", lambda *a: None)


def fake_gz(monkeypatch, returncode=0, stderr="", n_written=None, trailer=True, wall=2.0, cpu=1.0):
    calls = []

    def fake(sdf_path, iterations, seed, plugin_dir=None, extra_env=None, timeout_s=None):
        text = Path(sdf_path).read_text(encoding="utf-8")
        log_path = re.search(r"<log_path>([^<]*)</log_path>", text).group(1)
        m = int(re.search(r"<ticks_per_step>(\d+)</ticks_per_step>", text).group(1))
        n = iterations if n_written is None else n_written
        reads = [read_of(float(i)) for i in range(n)]
        Path(log_path).write_bytes(log_bytes(reads, [PUSH] * n, m=m, seed=seed, trailer=trailer))
        calls.append({"iterations": iterations, "seed": seed, "sdf": text, "m": m})
        return rs.GzProcess(returncode, "", stderr, wall, cpu)

    monkeypatch.setattr(rs, "run_gz_process", fake)
    return calls


def test_run_writes_log_world_and_report(tmp_path, monkeypatch, no_dpkg):
    calls = fake_gz(monkeypatch)
    r = rs.run(CARD, SCEN, 1, 1, "test", None, tmp_path, tmp_path / "noplugin", iterations=8)
    assert calls[0]["iterations"] == 8 and calls[0]["seed"] == 1
    assert Path(r.log_path).exists() and Path(r.world_path).exists() and Path(r.report_path).exists()
    assert r.log_sha256 == hashlib.sha256(Path(r.log_path).read_bytes()).hexdigest()
    assert r.warnings == [] and r.returncode == 0 and r.iterations == 8
    assert r.sim_s == 8 / 6400 and r.wall_s == 2.0 and r.rtf == r.sim_s / 2.0
    assert r.stale["stale"] == 0 and r.stale["last_fresh_index"] == 7
    assert r.scenario_sha256 == hashlib.sha256(SCEN.read_bytes()).hexdigest()
    assert r.sdf_edited is False


def test_run_applies_sdf_edit_and_seed(tmp_path, monkeypatch, no_dpkg):
    calls = fake_gz(monkeypatch)
    r = rs.run(CARD, SCEN, 5, 1, "test", None, tmp_path, tmp_path, lambda t: t.replace("<seed>5</seed>", "<seed>5 </seed>"),
               iterations=4)
    assert calls[0]["seed"] == 5 and "<seed>5 </seed>" in calls[0]["sdf"]
    assert r.sdf_edited and r.log["header"]["seed"] == 5 and r.seed == 5


def test_run_fails_on_nonzero_exit(tmp_path, monkeypatch, no_dpkg):
    fake_gz(monkeypatch, returncode=1, stderr="boom\n")
    with pytest.raises(rs.RunError, match="exited 1"):
        rs.run(CARD, SCEN, 1, 1, "test", None, tmp_path, tmp_path, iterations=4)


def test_run_sigsegv_after_complete_log_warns_and_reports(tmp_path, monkeypatch, no_dpkg):
    fake_gz(monkeypatch, returncode=-signal.SIGSEGV)
    with pytest.warns(RuntimeWarning, match="SIGSEGV"):
        r = rs.run(CARD, SCEN, 1, 1, "test", None, tmp_path, tmp_path, iterations=4)
    assert len(r.warnings) == 1
    text = Path(r.report_path).read_text(encoding="utf-8")
    assert "warnings (1):" in text and "SIGSEGV" in text and "gz exit code: -11" in text


def test_run_sigsegv_with_a_cut_log_fails(tmp_path, monkeypatch, no_dpkg):
    fake_gz(monkeypatch, returncode=-signal.SIGSEGV, n_written=3, trailer=False)
    with pytest.raises(rs.RunError, match="log complete: False"):
        rs.run(CARD, SCEN, 1, 1, "test", None, tmp_path, tmp_path, iterations=4)


def test_report_contents(tmp_path, monkeypatch, no_dpkg):
    fake_gz(monkeypatch)
    r = rs.run(CARD, SCEN, 1, 1, "test", None, tmp_path, tmp_path / "noplugin", iterations=4)
    text = Path(r.report_path).read_text(encoding="utf-8")
    profile = card_report.profile_files(card_report.schema.load_yaml(CARD), ROOT)
    for line in [
        card_report.MODEL_LINE,
        f"card hash: {card_report.framed_hash([CARD, *profile], ROOT)}",
        f"scenario sha256: {hashlib.sha256(SCEN.read_bytes()).hexdigest()}",
        "seed: 1 (RESERVED at v0", "m (ticks per host step): 1", "tick period: 625/4 us = 1/6400 s (156.25 us)",
        "host step H = m * tick = 1/6400 s (156.25 us)", f"gz-sim: {GZ_VERSION} (log header)", "gz-physics: UNKNOWN",
        "DART: UNKNOWN", "sdformat: UNKNOWN", "plugin build type: UNKNOWN", "plugin sha256: UNKNOWN",
        "stale steps: 0 of 4", "last fresh step: index 3, time 468750 ns", "wall time: 2.000 s", "RTF: 0.000",
        f"log sha256: {r.log_sha256}", "halving", "  (not provided", "MARV L1 run report",
    ]:
        assert line in text, line
    assert re.search(r"^git commit: (UNKNOWN|[0-9a-f]{40})( dirty)?$", text, flags=re.M)
    assert card_report.build_report(CARD, BUDGET, ROOT).rstrip("\n") in text
    assert "UNVERIFIED (" in text


def test_report_reads_the_plugin_build_type(tmp_path, monkeypatch, no_dpkg):
    build = tmp_path / "build"
    plugin_dir = build / "sim" / "plugin"
    plugin_dir.mkdir(parents=True)
    (build / "CMakeCache.txt").write_text("CMAKE_BUILD_TYPE:STRING=RelWithDebInfo\n")
    (plugin_dir / rs.PLUGIN_FILE).write_bytes(b"plugin")
    fake_gz(monkeypatch)
    r = rs.run(CARD, SCEN, 1, 1, "test", None, tmp_path / "out", plugin_dir, iterations=4)
    text = Path(r.report_path).read_text(encoding="utf-8")
    assert "plugin build type: RelWithDebInfo" in text
    assert f"plugin sha256: {hashlib.sha256(b'plugin').hexdigest()}" in text


def test_report_versions_from_dpkg(tmp_path, monkeypatch):
    seen = []

    def fake_dpkg(*args):
        seen.append(args)
        return "9.9.9" if args[0] == "-W" else None

    monkeypatch.setattr(rs, "_dpkg_query", fake_dpkg)
    v = rs.versions("8.15.0", tmp_path)
    assert v["gz-physics"] == "9.9.9 (dpkg libgz-physics7)" and v["sdformat"] == "9.9.9 (dpkg libsdformat14)"
    assert v["gz-sim"] == "8.15.0 (log header)"


def test_halving_section_is_printed(tmp_path, monkeypatch, no_dpkg):
    fake_gz(monkeypatch)
    halving = {"free_fall": {"d_k": [0.5, 0.25], "rho": 0.5, "E": 0.25, "verdict": "pass"}, "note": "x"}
    r = rs.run(CARD, SCEN, 1, 1, "test", None, tmp_path, tmp_path, halving=halving, iterations=4)
    text = Path(r.report_path).read_text(encoding="utf-8")
    assert "halving\n  free_fall:\n    d_k: [0.5, 0.25]\n    rho: 0.5\n    E: 0.25\n    verdict: pass\n  note: x\n" in text
    assert "(not provided" not in text
    rs.write_report([r], None, r.report_path)
    assert "(not provided" in Path(r.report_path).read_text(encoding="utf-8")


def test_sequence_runs_one_process_per_m_and_writes_one_report(tmp_path, monkeypatch, no_dpkg):
    calls = fake_gz(monkeypatch)
    res = rs.run_sequence(CARD, SEQ_SCEN, None, "test", None, tmp_path, tmp_path, halving={"k": 1})
    assert [c["m"] for c in calls] == [4, 2, 1] and [r.m for r in res.runs] == [4, 2, 1]
    assert [c["iterations"] for c in calls] == [1600, 3200, 6400]
    text = Path(res.report_path).read_text(encoding="utf-8")
    assert "m sequence run: [4, 2, 1]" in text
    for m, h in ((4, "1/1600"), (2, "1/3200"), (1, "1/6400")):
        assert f"run: m = {m}, mode test" in text and f"host step H = m * tick = {h} s" in text
    assert text.count("MARV L1 run report") == 1 and "  k: 1\n" in text
    assert len({r.log_path for r in res.runs}) == 3


def test_m_outside_the_sequence_is_refused(tmp_path, monkeypatch, no_dpkg):
    fake_gz(monkeypatch)
    with pytest.raises(rs.gpc.GenError):
        rs.run(CARD, SCEN, 1, 2, "test", None, tmp_path, tmp_path)


# ---- SIM-3 -----------------------------------------------------------------------------------------------------------

def test_batch_plan_is_unknown_in_the_budget():
    assert rs.batch_plan_text(BUDGET) == "required: UNKNOWN (design/budget.yaml batch_plan)"


def test_batch_plan_with_a_value_still_invents_nothing(tmp_path):
    """Negative control: a budget with a value set changes the text, and the runner still states no required RTF."""
    b = tmp_path / "budget.yaml"
    b.write_text("batch_plan:\n  value: 12\n  unit: '1'\n", encoding="utf-8")
    text = rs.batch_plan_text(b)
    assert text != rs.batch_plan_text(BUDGET) and "not implemented" in text and "UNKNOWN" in text


def test_measure_rtf_with_a_fake_gz(tmp_path, monkeypatch, no_dpkg):
    walls = iter([4.0, 2.0])  # the full run, then the 10-iteration run
    fake_gz(monkeypatch)
    real = rs.run_gz_process

    def timed(*a, **k):
        p = real(*a, **k)
        p.wall_s = next(walls)
        p.cpu_s = p.wall_s / 2
        return p

    monkeypatch.setattr(rs, "run_gz_process", timed)
    s = rs.measure_rtf(CARD, SCEN, 1, None, tmp_path, tmp_path, iterations=6410)
    assert s.sim_s == 6410 / 6400 and s.startup_sim_s == 10 / 6400
    assert s.startup_wall_s == 2.0 and s.rtf_wall == (6410 / 6400) / 4.0
    assert s.rtf_marginal == pytest.approx(0.5, rel=1e-12) and s.cpu_over_wall == 0.5 and s.marginal_cpu_over_wall == 0.5
    assert s.required == "required: UNKNOWN (design/budget.yaml batch_plan)" and s.comparison == "UNKNOWN, SIM-3 open"
    text = rs.format_sim3(s)
    assert "required: UNKNOWN (design/budget.yaml batch_plan)" in text and "comparison: UNKNOWN, SIM-3 open" in text
    assert "one process = one core" in text and "RUSAGE_CHILDREN" in text and "startup overhead" in text
    assert "consistent with one core" in text


def test_sim3_flags_a_multi_core_process():
    """Negative control: CPU time above wall time says the one-process-one-core assumption fails."""
    s = rs.Sim3Result("c", "s.yaml", 1, 1, 100, 10, 1.0, 1.0, 2.0, 0.1, 0.5, 0.5, 1.0, 5.0, 2.0, 2.0, 16, "Debug",
                      "required: UNKNOWN (design/budget.yaml batch_plan)", "UNKNOWN, SIM-3 open")
    assert "assumption FAILS" in rs.format_sim3(s)
