"""Builds from the GCS (ground-segment unit GX-14, tools/gs/build.py): the allow-list and the refusal of everything else, the heavy lock
(refused while held, held by the build, the same file and flock as ci/heavy.sh), the free-memory budget, one build at a time, cancel
(H-2: the process group and every descendant are gone), a launch never builds (H-3), the describe string in the library entry and the
`ninja -n` status parse. Each refusal has a control (the same request with the condition satisfied builds). Fakes only: the cmake and ninja
of tools/gs/e2e_fake_build.py are put first on the PATH; nothing is built and no real tree is touched.
"""

import asyncio
import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "gs"))

import artifacts  # noqa: E402
import build as build_mod  # noqa: E402
import hoststate  # noqa: E402
from test_gs_helper import (  # noqa: E402  (module-level helpers of the sibling test file)
    HEAVY, Session, hold_heavy, http_get, release_heavy, repo_state, run_with_helper, wait_for, ws_connect)

FAKE = ROOT / "tools" / "gs" / "e2e_fake_build.py"
GZ = "host-gz-quadfc"
LIB = Path("sim") / "quad_fc" / "libmarv_sil_quad_fc.so"
KIB = 1024  # /proc/meminfo counts kB
FREE_ENOUGH_KB = 4 * (build_mod.BUILD_MIN_FREE_BYTES // KIB)  # a fake host with four times the budget available
STOP_S = 0.5  # labelled harness value: the cancel's SIGTERM-to-SIGKILL wait in these tests


class Fake:
    """The fake build tools and the fake memory of a test."""

    def __init__(self, tmp_path, monkeypatch):
        self.dir = tmp_path / "fake"
        subprocess.run([sys.executable, str(FAKE), "--install", str(self.dir / "bin")], check=True)
        monkeypatch.setenv("PATH", f"{self.dir / 'bin'}{os.pathsep}{os.environ['PATH']}")
        monkeypatch.setenv("MARV_FAKE_BUILD_DIR", str(self.dir))
        for name in ("STEPS", "STEP_S", "STATUS", "HOLD", "STUBBORN"):
            monkeypatch.delenv(f"MARV_FAKE_BUILD_{name}", raising=False)
        monkeypatch.delenv("MARV_HEAVY_HELD", raising=False)
        self.monkeypatch = monkeypatch
        self.meminfo = tmp_path / "meminfo"
        self.available(FREE_ENOUGH_KB)
        monkeypatch.setattr(hoststate, "MEMINFO", self.meminfo)

    def available(self, kb):
        self.meminfo.write_text(f"MemTotal:       {kb * 2} kB\nMemFree:        1 kB\nMemAvailable:   {kb} kB\n")

    def env(self, **kw):
        for k, v in kw.items():
            self.monkeypatch.setenv(f"MARV_FAKE_BUILD_{k}", str(v))

    def calls(self):
        path = self.dir / "argv.jsonl"
        return [json.loads(line) for line in path.read_text().splitlines()] if path.exists() else []

    def pids(self):
        return [int(p) for p in (self.dir / "pids").read_text().split()]


@pytest.fixture
def fake(tmp_path, monkeypatch):
    return Fake(tmp_path, monkeypatch)


def gone(pid):
    """No such process, or only a zombie (a dead process nobody has reaped yet)."""
    try:
        state = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()[0]
    except OSError:
        return True
    return state == "Z"


def lock_status(tmp_path):
    done = subprocess.run(["bash", str(HEAVY), "--status"], env={**os.environ, "MARV_HEAVY_LOCK": str(tmp_path / "heavy.lock")}, capture_output=True,
                          text=True)
    return done.returncode, done.stdout.strip()


def describe():
    return subprocess.run(["git", "-C", str(ROOT), "describe", "--dirty", "--always"], capture_output=True, text=True, check=True).stdout.strip()


def run(tmp_path, body, **kwargs):
    return run_with_helper(tmp_path, body, build_root=tmp_path / "build", **kwargs)


async def built(s, preset=GZ):
    """Starts a build and waits for its build_done."""
    started = await s.cmd(cmd="build", preset=preset)
    assert started["ok"], started
    return await s.wait_event("build_done", preset=preset, timeout=30)


# ---- the status parse --------------------------------------------------------------------------------------------------------------

def test_the_ninja_dry_run_parse_gives_up_to_date_or_the_number_of_steps():
    assert build_mod.parse_dry_run(0, "ninja: no work to do.\n") == {"kind": "up_to_date", "steps": None, "text": "up to date"}
    steps = build_mod.parse_dry_run(0, "[1/312] Building CXX object a.o\n[2/312] Linking CXX executable b\n[312/312] Linking x\n")
    assert steps == {"kind": "steps", "steps": 312, "text": "312 steps", "glob_recheck": False}
    assert build_mod.parse_dry_run(0, "[1/1] Linking CXX executable b\n")["text"] == "1 step"
    for rc, text in ((1, "ninja: error: loading 'build.ninja': No such file or directory\n"), (0, ""), (0, "something else\n")):
        assert build_mod.parse_dry_run(rc, text)["kind"] == "error", (rc, text)


RECHECK = "[0/2] Re-checking globbed directories...\n[1/2] Re-running CMake...\n"


def test_cmakes_always_dirty_glob_steps_are_not_counted_and_say_so():
    """The dry run recorded after the real build (tests/regression/quad/L10/results/gcs/gx14/ninja_dry_run_after.txt)."""
    recorded = (ROOT / "tests" / "regression" / "quad" / "L10" / "results" / "gcs" / "gx14" / "ninja_dry_run_after.txt").read_text()
    assert build_mod.parse_dry_run(0, recorded) == {"kind": "up_to_date", "steps": None, "text": "up to date", "glob_recheck": True}
    assert build_mod.parse_dry_run(0, RECHECK)["glob_recheck"] is True
    # the same two steps plus a compile step: that one counts, and only that one
    one = build_mod.parse_dry_run(0, "[0/3] Re-checking globbed directories...\n[1/3] Re-running CMake...\n[2/3] Building CXX object a.o\n")
    assert one == {"kind": "steps", "steps": 1, "text": "1 step", "glob_recheck": True}
    # control: a step whose description is not exactly one of the two is counted, whatever its position
    for other in ("Re-checking globbed directories", "Re-running CMake", "Re-checking globbed directories... x", "Re-running CMake (again)...", "Building CXX object"):
        got = build_mod.parse_dry_run(0, f"[0/2] Re-checking globbed directories...\n[1/2] {other}\n")
        assert got == {"kind": "steps", "steps": 1, "text": "1 step", "glob_recheck": True}, other
        got = build_mod.parse_dry_run(0, f"[0/2] {other}\n[1/2] {other}\n")
        assert got["kind"] == "steps" and got["steps"] == 2 and got["glob_recheck"] is False, other


def test_the_status_runs_ninja_n_on_the_tree_and_names_not_configured_up_to_date_and_n_steps(tmp_path, fake):
    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            status = (await s.cmd(cmd="build_status", preset=GZ))["build"]
            assert status["status"] == {"kind": "not_configured", "steps": None, "text": "not configured"} and fake.calls() == []
            assert [a["preset"] for a in status["allowed"]] == ["host-gz-quadfc", "m33", "m33-hil"]
            tree = tmp_path / "build" / GZ
            tree.mkdir(parents=True)
            (tree / "build.ninja").write_text("# fake\n")
            fake.env(STEPS=7)
            status = (await s.cmd(cmd="build_status", preset=GZ))["build"]
            assert status["status"] == {"kind": "steps", "steps": 7, "text": "7 steps", "glob_recheck": False}
            assert fake.calls() == [["ninja", "-n", "-C", str(tree)]]
            assert status["refusals"] == [] and status["running"] is False and status["current"] is None
            done = await built(s)
            assert done["ok"] is True
            status = (await s.cmd(cmd="build_status", preset=GZ))["build"]
            assert status["status"] == {"kind": "up_to_date", "steps": None, "text": "up to date"}
            assert status["current"]["ok"] is True and status["current"]["running"] is False
            # without a preset: the allow-list and the latest build only, no dry run
            before = len(fake.calls())
            assert (await s.cmd(cmd="build_status"))["build"]["status"] is None and len(fake.calls()) == before
            s.close()
    run(tmp_path, body)


# ---- the allow-list ----------------------------------------------------------------------------------------------------------------

def test_the_allow_list_is_the_gcs_presets_and_each_names_a_configure_preset_of_cmakepresets():
    presets = {p["name"] for p in json.loads((ROOT / "CMakePresets.json").read_text())["configurePresets"]}
    assert set(build_mod.PRESETS) == {"host-gz-quadfc", "m33", "m33-hil"}
    for name, p in build_mod.PRESETS.items():
        assert p.configure in presets, name
    assert build_mod.PRESETS["m33-hil"].configure == "m33" and build_mod.PRESETS["m33-hil"].tree == "m33-hil"


@pytest.mark.parametrize("name", ["host-debug", "host-gz-l5", "host-release", "M33", " m33", "m33 ", "m33\n", "", "m33; touch pwned",
                                  "m33 && touch pwned", "m33 -DMARV_X=1", "m33 --target all", "../m33", "m33/../m33", "m33-hil/..", "$(touch pwned)",
                                  "`touch pwned`", "m33|touch pwned", "--preset=m33", "-B/tmp/x", "host-gz-quadfc\x00", None, 5, True, ["m33"], {"m33": 1}])
def test_a_preset_not_on_the_allow_list_and_a_preset_name_injection_are_refused_and_nothing_runs(tmp_path, fake, name):
    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            refused = await s.cmd(cmd="build", preset=name)
            assert refused["ok"] is False and "allow-list" in refused["error"] and refused["refusals"] == [refused["error"]], refused
            assert name is None or not (await s.cmd(cmd="build_status", preset=name))["ok"]  # no preset: the status of no preset
            assert fake.calls() == [] and not (tmp_path / "build").exists() and not (tmp_path / "pwned").exists()
            assert h.builds.state is None and not h.builds.busy()
            assert not (await s.cmd(cmd="build"))["ok"]  # no preset at all
            s.close()
    run(tmp_path, body)


def test_the_argv_of_each_allowed_preset_is_a_list_with_the_tree_and_the_cache_entries_and_the_browser_adds_nothing(tmp_path, fake):
    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            for name in build_mod.PRESETS:
                reply = await s.cmd(cmd="build", preset=name, extra="-DEVIL=1", target="evil", **{"-B": "/tmp"})
                assert reply["ok"], reply
                assert (await s.wait_event("build_done", preset=name, timeout=30))["ok"] is True
            s.close()
    run(tmp_path, body)
    tree = lambda n: str(tmp_path / "build" / n)  # noqa: E731
    assert fake.calls() == [
        ["cmake", "--preset", "host-gz-quadfc", "-B", tree("host-gz-quadfc")], ["cmake", "--build", tree("host-gz-quadfc")],
        ["cmake", "--preset", "m33", "-B", tree("m33")], ["cmake", "--build", tree("m33")],
        ["cmake", "--preset", "m33", "-B", tree("m33-hil"), "-DMARV_FC3V2_HIL=ON", "-DMARV_FC3V2_HIL_RECEIVER=ON"],
        ["cmake", "--build", tree("m33-hil"), "--target", "marv_fc3v2"]]


# ---- the heavy lock ----------------------------------------------------------------------------------------------------------------

def test_a_build_is_refused_while_the_heavy_lock_is_held_and_the_same_build_runs_when_it_is_free(tmp_path, fake):
    holder = hold_heavy(tmp_path)

    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            status = (await s.cmd(cmd="build_status", preset=GZ))["build"]
            assert any("the heavy lock is held" in r for r in status["refusals"]), status["refusals"]
            refused = await s.cmd(cmd="build", preset=GZ)
            assert refused["ok"] is False and "the heavy lock is held" in refused["error"] and "sleep" in refused["error"], refused
            assert fake.calls() == [] and not (tmp_path / "build").exists() and not h.builds.busy()
            release_heavy(holder)
            assert (await s.cmd(cmd="build_status", preset=GZ))["build"]["refusals"] == []
            assert (await built(s))["ok"] is True  # the control
            s.close()
    try:
        run(tmp_path, body)
    finally:
        release_heavy(holder)


def test_the_lock_itself_decides_a_holder_that_appears_after_the_check_refuses_the_build(tmp_path, fake, monkeypatch):
    holder = None

    async def body(h):
        nonlocal holder
        real = h.builds.refusals

        def late():  # the pre-check sees a free lock; the holder takes it right after (the race the flock closes)
            nonlocal holder
            reasons = real()
            holder = hold_heavy(tmp_path)
            return reasons
        monkeypatch.setattr(h.builds, "refusals", late)
        async with ws_connect(h) as ws:
            s = Session(ws)
            refused = await s.cmd(cmd="build", preset=GZ)
            assert refused["ok"] is False and "the heavy lock is held" in refused["error"], refused
            assert fake.calls() == [] and not h.builds.busy() and h.builds._lock_fd is None
            s.close()
    try:
        run(tmp_path, body)
    finally:
        if holder is not None:
            release_heavy(holder)


def test_a_build_holds_the_heavy_lock_on_the_same_file_as_ci_heavy_sh_and_releases_it_when_done(tmp_path, fake):
    fake.env(HOLD=1)

    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            assert lock_status(tmp_path) == (0, "free")
            assert (await s.cmd(cmd="build", preset=GZ))["ok"]
            assert await asyncio.to_thread(wait_for, lambda: (fake.dir / "pids").exists())
            code, text = lock_status(tmp_path)
            assert code == 75 and text.startswith("held: pid=") and f"pid={os.getpid()}" in text and f"cmd=GCS build {GZ}" in text, (code, text)
            state = hoststate.heavy_lock_state(tmp_path / "heavy.lock")
            assert state["held"] and f"cmd=GCS build {GZ}" in state["holder"]
            # ci/heavy.sh and the helper's own run refuse while the build holds it, in both directions
            other = subprocess.run(["bash", str(HEAVY), "--wait", "0", "--", "true"], env={**os.environ, "MARV_HEAVY_LOCK": str(tmp_path / "heavy.lock")},
                                   capture_output=True, text=True)
            assert other.returncode == 75 and "is held" in other.stderr, other
            assert "heavy lock is held" in (await s.cmd(cmd="run_start", kind="sil"))["error"]
            # no process of the build keeps the lock file open: the lock is the helper's alone (as heavy.sh's shell holds it)
            leader = fake.pids()[0]
            lock = (tmp_path / "heavy.lock").resolve()
            assert all(Path(os.readlink(f"/proc/{leader}/fd/{fd}")) != lock for fd in os.listdir(f"/proc/{leader}/fd"))
            assert (await s.cmd(cmd="build_cancel"))["cancelled"] is True
            assert lock_status(tmp_path) == (0, "free")
            s.close()
    run(tmp_path, body)


def test_the_build_exports_marv_heavy_held_so_that_a_nested_heavy_sh_does_not_wait_on_it(tmp_path, fake, monkeypatch):
    seen = tmp_path / "held"
    wrapper = tmp_path / "bin2"
    wrapper.mkdir()
    cmake = wrapper / "cmake"
    cmake.write_text(f'#!/bin/sh\nprintf "%s" "$MARV_HEAVY_HELD" > "{seen}"\nexec "{fake.dir / "bin" / "cmake"}" "$@"\n')
    cmake.chmod(0o755)
    monkeypatch.setenv("PATH", f"{wrapper}{os.pathsep}{os.environ['PATH']}")

    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            assert (await built(s))["ok"]
            s.close()
    run(tmp_path, body)
    assert seen.read_text() == str(tmp_path / "heavy.lock")


# ---- free memory -------------------------------------------------------------------------------------------------------------------

def test_a_build_is_refused_below_the_free_memory_budget_and_the_same_build_runs_at_the_budget(tmp_path, fake):
    budget_kb = build_mod.BUILD_MIN_FREE_BYTES // KIB

    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            fake.available(budget_kb - 1)
            assert any("free memory" in r and "below the build budget" in r for r in (await s.cmd(cmd="build_status", preset=GZ))["build"]["refusals"])
            refused = await s.cmd(cmd="build", preset=GZ)
            assert refused["ok"] is False and "below the build budget" in refused["error"], refused
            assert fake.calls() == [] and not h.builds.busy() and lock_status(tmp_path) == (0, "free")
            fake.available(budget_kb)  # the control: exactly the budget is enough
            assert (await s.cmd(cmd="build_status", preset=GZ))["build"]["refusals"] == []
            assert (await built(s))["ok"] is True
            s.close()
    run(tmp_path, body)


def test_unreadable_free_memory_refuses_the_build(tmp_path, fake):
    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            fake.meminfo.write_text("MemTotal: 1 kB\n")
            refused = await s.cmd(cmd="build", preset=GZ)
            assert refused["ok"] is False and "free memory cannot be read" in refused["error"] and fake.calls() == []
            fake.available(FREE_ENOUGH_KB)
            assert (await built(s))["ok"]
            s.close()
    run(tmp_path, body)


# ---- one at a time, progress, failure ----------------------------------------------------------------------------------------------

def test_one_build_at_a_time_a_second_is_refused_naming_the_running_one_and_runs_after_the_first_ends(tmp_path, fake):
    fake.env(HOLD=1)

    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            assert (await s.cmd(cmd="build", preset=GZ))["ok"]
            for other in (GZ, "m33"):
                second = await s.cmd(cmd="build", preset=other)
                assert second["ok"] is False and any(f"a build is running ({GZ}): one build at a time" in r for r in second["refusals"]), second
            assert await asyncio.to_thread(wait_for, lambda: (fake.dir / "pids").exists())
            assert [c for c in fake.calls() if c[:2] == ["cmake", "--build"]] == [["cmake", "--build", str(tmp_path / "build" / GZ)]]
            assert (await s.cmd(cmd="build_cancel"))["cancelled"] is True
            fake.monkeypatch.delenv("MARV_FAKE_BUILD_HOLD")
            assert (await built(s, "m33"))["ok"] is True  # the control
            s.close()
    run(tmp_path, body)


def test_progress_events_carry_the_step_and_the_output_is_kept_in_the_status_and_in_a_log_file(tmp_path, fake):
    fake.env(STEPS=5)

    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            started = await s.cmd(cmd="build", preset=GZ)
            assert started["started"] is True and started["build"]["running"] is True and started["build"]["describe"] == describe()
            assert (await s.wait_event("build_started", preset=GZ))["describe"] == describe()
            done = await s.wait_event("build_done", preset=GZ, timeout=30)
            assert done["ok"] is True and done["exit_status"] == 0 and done["error"] is None and done["cancelled"] is False
            progress = s.seen("build_progress", preset=GZ)
            assert progress and progress[-1]["line"] == "[5/5] Building CXX object fake_5.o" and progress[-1]["step"] == 5 and progress[-1]["steps"] == 5
            current = (await s.cmd(cmd="status"))["status"]["build"]
            assert current["tail"][-1] == "[5/5] Building CXX object fake_5.o" and current["step"] == 5 and current["running"] is False
            log = Path(current["log"])
            assert log.parent == h.run_dir / "builds" and log.name.endswith(f"-{GZ}.log")
            text = log.read_text().splitlines()
            assert text[0] == f"+ cmake --preset {GZ} -B {tmp_path / 'build' / GZ}" and "[3/5] Building CXX object fake_3.o" in text
            s.close()
    run(tmp_path, body)


def test_a_failing_build_reports_the_status_records_nothing_and_releases_the_lock(tmp_path, fake):
    fake.env(STATUS=3)

    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            done = await built(s)
            assert done["ok"] is False and done["exit_status"] == 3 and done["images"] == [] and "status 3" in done["error"]
            assert not (tmp_path / "build" / GZ / build_mod.BUILD_RECORD).exists()
            assert lock_status(tmp_path) == (0, "free") and not h.builds.busy()
            fake.monkeypatch.delenv("MARV_FAKE_BUILD_STATUS")
            assert (await built(s))["ok"] is True  # the control
            s.close()
    run(tmp_path, body)


# ---- cancel and stop: no process is left (H-2) -------------------------------------------------------------------------------------

@pytest.mark.parametrize("stubborn", [False, True])
def test_cancel_stops_the_process_group_and_every_descendant_and_leaves_no_process(tmp_path, fake, stubborn):
    fake.env(HOLD=1)
    if stubborn:
        fake.env(STUBBORN=1)

    async def body(h):
        h.builds.stop_s = STOP_S
        async with ws_connect(h) as ws:
            s = Session(ws)
            assert (await s.cmd(cmd="build", preset=GZ))["ok"]
            assert await asyncio.to_thread(wait_for, lambda: (fake.dir / "pids").exists())
            leader, own_session, same_group = fake.pids()
            assert os.getpgid(leader) == leader != os.getpgrp()  # the step is its own process group
            assert os.getpgid(own_session) == own_session != leader  # as ninja's jobs are: killing the group alone would miss it
            assert os.getpgid(same_group) == leader
            assert not any(gone(p) for p in (leader, own_session, same_group))
            reply = await s.cmd(cmd="build_cancel")
            assert reply["ok"] and reply["cancelled"] is True and reply["build"]["cancelled"] is True and reply["build"]["running"] is False
            assert all(gone(p) for p in (leader, own_session, same_group)), [p for p in (leader, own_session, same_group) if not gone(p)]
            done = await s.wait_event("build_done", preset=GZ)
            assert done["ok"] is False and done["cancelled"] is True and done["error"] == "cancelled" and done["images"] == []
            assert lock_status(tmp_path) == (0, "free") and not h.builds.busy()
            assert not (tmp_path / "build" / GZ / build_mod.BUILD_RECORD).exists()
            # the control: nothing is cancelled when nothing runs, and a new build starts
            again = await s.cmd(cmd="build_cancel")
            assert again["ok"] is False and "no build is running" in again["error"]
            fake.monkeypatch.delenv("MARV_FAKE_BUILD_HOLD")
            fake.monkeypatch.delenv("MARV_FAKE_BUILD_STUBBORN", raising=False)
            assert (await built(s))["ok"] is True
            s.close()
    run(tmp_path, body)


def test_a_helper_that_stops_during_a_build_leaves_no_process_and_releases_the_lock(tmp_path, fake):
    fake.env(HOLD=1)
    seen = {}

    async def body(h):
        h.builds.stop_s = STOP_S
        async with ws_connect(h) as ws:
            s = Session(ws)
            assert (await s.cmd(cmd="build", preset=GZ))["ok"]
            assert await asyncio.to_thread(wait_for, lambda: (fake.dir / "pids").exists())
            seen["pids"] = fake.pids()
            s.close()
    run(tmp_path, body)
    assert all(gone(p) for p in seen["pids"]), [p for p in seen["pids"] if not gone(p)]
    assert lock_status(tmp_path) == (0, "free")


# ---- a launch never builds (H-3), the library entry, W-1 ---------------------------------------------------------------------------

def test_a_launch_never_builds_and_leaves_the_image_hash_unchanged(tmp_path, fake):
    lib = tmp_path / "build" / GZ / LIB
    lib.parent.mkdir(parents=True)
    lib.write_text("an existing SIL library\n")
    sha = hashlib.sha256(lib.read_bytes()).hexdigest()

    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            before = (await asyncio.to_thread(h.images.scan))["images"]
            assert [e["hash"] for e in before] == [sha] and before[0]["build"] is None
            assert (await s.cmd(cmd="run_plan", kind="sil"))["ok"]
            started = await s.cmd(cmd="sil_start")
            assert started["ok"], started
            assert (await s.cmd(cmd="run_stop"))["ok"]
            assert (await s.cmd(cmd="run_start", kind="sil"))["ok"]
            assert (await s.cmd(cmd="run_stop"))["ok"]
            for c in ("status", "host_state", "cards", "run_list", "console_status", "log_list"):
                assert (await s.cmd(cmd=c))["ok"], c
            assert fake.calls() == [] and h.builds.state is None and not h.builds.busy()
            assert (await asyncio.to_thread(h.images.scan))["images"] == before
            assert hashlib.sha256(lib.read_bytes()).hexdigest() == sha and not (tmp_path / "build" / GZ / "build.ninja").exists()
            s.close()
    run(tmp_path, body, sil_command=["sleep", "600"])


def test_the_library_entry_of_a_gcs_build_records_the_describe_string_and_another_trees_image_has_none(tmp_path, fake):
    other = tmp_path / "build" / "host-gz-l5" / LIB.parent / "libmarv_sil_l5_attitude_scripted.so"  # an image a build of another kind left
    other.parent.mkdir(parents=True)
    other.write_text("not built by the GCS\n")

    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            done = await built(s)
            lib = tmp_path / "build" / GZ / LIB
            sha = hashlib.sha256(lib.read_bytes()).hexdigest()
            assert done["ok"] and done["describe"] == describe() and done["images"] == [{"hash": sha, "name": "marv_sil_quad_fc", "kind": "sil"}]
            status, _, body_ = await http_get(h, "/api/images")
            assert status == 200
            entries = {e["tree"]: e for e in json.loads(body_)["images"]}
            assert entries[GZ]["hash"] == sha and entries[GZ]["build"] == {"preset": GZ, "describe": describe(), "finished_utc": done["finished_utc"]}
            assert entries["host-gz-l5"]["build"] is None
            record = json.loads((tmp_path / "build" / GZ / build_mod.BUILD_RECORD).read_text())
            assert record["describe"] == describe() and record["images"] == {sha: "marv_sil_quad_fc"} and record["preset"] == GZ
            # a second build produces another image: the entry carries the new hash, the record the new build
            second = await built(s)
            sha2 = hashlib.sha256(lib.read_bytes()).hexdigest()
            assert sha2 != sha and second["images"][0]["hash"] == sha2
            assert [e["hash"] for e in h.images.scan()["images"] if e["tree"] == GZ] == [sha2]
            # an image the GCS did not build (another build replaced the file) is not credited with the describe string
            lib.write_text("rebuilt by an agent\n")
            entry = next(e for e in h.images.scan()["images"] if e["tree"] == GZ)
            assert entry["hash"] != sha2 and entry["build"] is None
            s.close()
    run(tmp_path, body)


def test_the_library_ignores_a_damaged_build_record(tmp_path):
    lib = tmp_path / "build" / GZ / LIB
    lib.parent.mkdir(parents=True)
    lib.write_text("x\n")
    for text in ("not json", "[1]", '{"images": 3}'):
        (tmp_path / "build" / GZ / build_mod.BUILD_RECORD).write_text(text)
        assert artifacts.ImageLibrary(tmp_path / "build").scan()["images"][0]["build"] is None, text


def test_no_build_command_writes_into_the_repository(tmp_path, fake):
    """W-1: git status and the diff against HEAD are the same after every build command as before it."""
    before = repo_state()
    fake.env(HOLD=1)
    seen = []

    async def body(h):
        h.builds.stop_s = STOP_S
        async with ws_connect(h) as ws:
            s = Session(ws)
            for msg in ({"cmd": "build_status", "preset": GZ}, {"cmd": "build", "preset": "nope"}, {"cmd": "build", "preset": GZ}, {"cmd": "build_cancel"}):
                await s.cmd(**msg)
                seen.append(repo_state())
            fake.monkeypatch.delenv("MARV_FAKE_BUILD_HOLD")
            await built(s)
            seen.append(repo_state())
            s.close()
    run(tmp_path, body)
    assert seen and all(state == before for state in seen)


def test_the_helper_takes_the_build_memory_budget_from_its_option(tmp_path, fake):
    import helper as helper_mod

    assert helper_mod.parse_args([]).build_min_free_bytes == build_mod.BUILD_MIN_FREE_BYTES
    assert helper_mod.parse_args(["--build-min-free-bytes", "1"]).build_min_free_bytes == 1

    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            assert h.builds.min_free_bytes == 1 << 62
            refused = await s.cmd(cmd="build", preset=GZ)
            assert refused["ok"] is False and "below the build budget" in refused["error"] and fake.calls() == []
            h.builds.min_free_bytes = 1  # the control
            assert (await built(s))["ok"] is True
            s.close()
    run(tmp_path, body, build_min_free_bytes=1 << 62)
