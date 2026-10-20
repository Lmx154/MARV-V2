"""DERIVE from the GCS (ground-segment unit GX-13, tools/gs/derive.py): the request checks (card names, edit keys, values, locks), the lint
refusal of a copy without provenance, the work directory (W-1: the repository is not written), the heavy lock and the other refusals, cancel,
the config library that `validate` reads, and the reports. The card copy is linted by the repository's real tools/card/lint.py; the three heavy
steps are replaced by cheap stand-ins (the real params_gen is not needed to see the helper's behaviour). That the real steps give the bytes a CMake
build gives is tests/tools/test_gs_derive_cmake.py (the full run, a merge-gate file: ci/run_ci.sh nightly_tools_tests).
"""

import asyncio
import hashlib
import json
import os
import re
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "gs"))

import derive as derive_mod  # noqa: E402
import hoststate  # noqa: E402
from test_gs_helper import (  # noqa: E402  (module-level helpers of the sibling test file)
    Session, hold_heavy, make_flash_root, flash_manifest, release_heavy, repo_state, run_with_helper, wait_for, ws_connect)

CARD = "vehicles/marv_7in_fc3v2.yaml"
UZH = "vehicles/uzh_neurobem_5in.yaml"
SOURCE = "Luis, 2026-10-21, scale reading 0.800 kg with the pack and props on (test text)"
KIB = 1024
FREE_ENOUGH_KB = 4 * (derive_mod.DERIVE_MIN_FREE_BYTES // KIB)
STOP_S = 0.5  # labelled harness value: the cancel's SIGTERM-to-SIGKILL wait in these tests

WRITE_MANIFEST = ("import json, pathlib, sys; d = pathlib.Path(sys.argv[1]); d.mkdir(parents=True, exist_ok=True); "
                  "(d / 'params_manifest.json').write_text(sys.argv[2])")
WRITE_REPORT = "import pathlib, sys; pathlib.Path(sys.argv[1], 'x_report.txt').write_text('a derivation report\\n')"


def standin_manifest():
    """The manifest of tests/tools/test_gs_helper.py's board image, with the first value changed: the derived config differs from its factory config."""
    manifest = flash_manifest()
    manifest["params"]["a_gain"]["default_bits"] = "0x40000000"
    for name, entry in manifest["params"].items():
        entry.update(unit="1", link_id=name, sigma=0.0, source_file="stand-in")
    return manifest


@pytest.fixture
def standin_steps(tmp_path, monkeypatch):
    """The three steps replaced: a stand-in flatten (a report), a stand-in params_gen (the manifest above) and the real mcfg.py encode. Every step
    appends its name to tmp/steps.log. The hold file makes the first step wait (for the cancel and the lock tests)."""
    log = tmp_path / "steps.log"
    hold = tmp_path / "hold"

    def plan_steps(self, paths, work):
        gen = work / "gen" / "marv" / "params"
        wait = f"import os, time; [time.sleep(0.05) for _ in iter(lambda: not os.path.exists({str(hold)!r}), False)]; "
        note = lambda name: f"open({str(log)!r}, 'a').write({name!r} + chr(10)); "  # noqa: E731
        return [
            {"name": "flatten", "argv": [sys.executable, "-c", wait + note("flatten") + WRITE_REPORT, str(work / "flatten")]},
            {"name": "params_gen", "argv": [sys.executable, "-c", note("params_gen") + WRITE_MANIFEST, str(gen), json.dumps(standin_manifest())]},
            {"name": "mcfg", "argv": [sys.executable, str(ROOT / "tools" / "config" / "mcfg.py"), "encode", "--manifest", str(gen / "params_manifest.json"),
                                      "--section", "quad", "--max-bytes", str(self.slot_half()), "--out", str(work / f"{derive_mod.PARAMS_SET}.mcfg")]},
        ]
    monkeypatch.setattr(derive_mod.DeriveManager, "plan_steps", plan_steps)
    monkeypatch.setattr(hoststate, "MEMINFO", _meminfo(tmp_path, FREE_ENOUGH_KB))
    monkeypatch.delenv("MARV_HEAVY_HELD", raising=False)

    class Steps:
        def ran(self):
            return log.read_text().split() if log.exists() else []

        def hold(self):
            hold.unlink(missing_ok=True)

        def release(self):
            hold.write_text("x")
    steps = Steps()
    steps.release()
    return steps


def _meminfo(tmp_path, kb):
    path = tmp_path / "meminfo"
    path.write_text(f"MemTotal:       {kb * 2} kB\nMemFree:        1 kB\nMemAvailable:   {kb} kB\n")
    return path


def run(tmp_path, body, **kwargs):
    return run_with_helper(tmp_path, body, build_root=tmp_path / "build", **kwargs)


async def derived(s, card=CARD, edits=(), timeout=30):
    started = await s.cmd(cmd="derive", card=card, edits=list(edits))
    assert started["ok"], started
    return started, await s.wait_event("derive_done", id=started["derive"]["id"], timeout=timeout)


def mass(value=0.8, source=SOURCE):
    return {"key": "mass", "value": value, "source": source}


# ---- the recipe is the CMake call ------------------------------------------------------------------------------------------------

def test_the_recipe_table_is_the_cmake_call_of_the_flight_set():
    """tools/gs/derive.py repeats the arguments of `marv_add_card_param_set(marv_params_fc3v2 ...)`; this is the drift guard of that copy (the byte
    comparison with a real build is test_gs_derive_cmake.py)."""
    text = (ROOT / "fw" / "params" / "CMakeLists.txt").read_text()
    call = re.search(r"marv_add_card_param_set\(marv_params_fc3v2 .*?\)\n\n", text, re.S).group(0)
    assert "BUDGET \"${MARV_DESIGN_BUDGET}\"" in call and "RATE_GUARD mean_period" in call and derive_mod.RATE_GUARD == "mean_period"
    assert re.search(r'set\(MARV_DESIGN_BUDGET "([^"]+)"', text).group(1) == derive_mod.BUDGET
    prefix = "${PROJECT_SOURCE_DIR}/"
    scenario = re.search(r'SCENARIO "([^"]+)"', call).group(1)
    assert scenario.startswith(prefix) and scenario[len(prefix):] == derive_mod.SCENARIO
    registers = re.search(r"EXTRA_REGISTER (.*?)\)", call, re.S).group(1)
    assert [r[len(prefix):] for r in re.findall(r'"([^"]+)"', registers)] == list(derive_mod.EXTRA_REGISTERS)
    assert re.search(r'CARD "(vehicles/[^"]+)"', call).group(1) == CARD
    function = text[text.index("function(marv_add_card_param_set"):text.index("function(marv_add_card_report")]
    cmake_flags = set(re.findall(r"--[a-z][a-z-]*", function))  # the flags of the flatten_cached.py call, with and without SCENARIO
    argv = derive_mod.DeriveManager(root=ROOT, run_dir=ROOT, build_root=ROOT, library=None, log=print, push=print, lock_path=lambda: ROOT / "x").plan_steps(
        {"card": Path("c"), "budget": Path("b"), "scenario": Path("s"), "root": Path("r")}, Path("/w"))
    assert {a for a in argv[0]["argv"] if a.startswith("--")} == cmake_flags, cmake_flags ^ {a for a in argv[0]["argv"] if a.startswith("--")}
    order = re.search(r"marv_add_param_set\(\$\{name\} CARD (.*?) REGISTER", function, re.S).group(1)
    assert re.findall(r"\$\{(\w+)\}", order) == ["scenario_outputs" if p == "scenario" else f"flat_{p}" for p in derive_mod.FLATTEN_PARTS]
    gen = argv[1]["argv"]
    assert [Path(gen[i + 1]).name for i, a in enumerate(gen) if a == "--card"] == [f"{derive_mod.PARAMS_SET}_{p}.yaml" for p in derive_mod.FLATTEN_PARTS]
    encode = argv[2]["argv"]
    assert encode[2:4] == ["encode", "--manifest"] and encode[encode.index("--section") + 1] == "quad"
    assert "--section quad --max-bytes ${marv_mcfg_max_bytes}" in re.sub(r"\s+", " ", text)


def test_the_slot_limit_is_half_the_budgets_config_slot():
    m = derive_mod.DeriveManager(root=ROOT, run_dir=ROOT, build_root=ROOT, library=None, log=print, push=print, lock_path=lambda: ROOT / "x")
    value = int(re.search(r"\nconfig_slot_bytes:[ \t]*\n[ \t]+value:[ \t]*([0-9]+)", (ROOT / "design" / "budget.yaml").read_text()).group(1))
    assert m.slot_half() == value // 2 > 0


# ---- the card and its edits ------------------------------------------------------------------------------------------------------

def test_the_editable_entries_are_the_cards_entries_with_a_value_and_the_locked_ones_say_so():
    entries = {e["key"]: e for e in derive_mod.card_entries(CARD)["entries"]}
    assert set(entries) >= {"mass", "inertia_diag", "drag", "rotors.thrust_coeff", "rotors.torque_ratio", "rotors.speed_range", "rotors.motor_lag.tau",
                            "rotors.m1.position", "rotors.m4.spin"}
    assert "rotors.esc_map" not in entries and "rotors.motor_lag.model" not in entries and "sensor_profile" not in entries  # no value to edit
    assert entries["rotors.speed_range"]["locked"] is True and entries["rotors.speed_range"]["lock"]["by"] == "Luis"
    assert entries["mass"]["locked"] is False and entries["mass"]["value"] == 0.732 and entries["mass"]["unit"] == "kg"
    assert [e["key"] for e in derive_mod.card_entries(UZH)["entries"]][:2] == ["mass", "inertia_diag"]


@pytest.mark.parametrize("card", ["", None, 5, ["vehicles/marv_7in_fc3v2.yaml"], "vehicles/", "vehicles/missing.yaml", "missing.yaml", "../vehicles/marv_7in_fc3v2.yaml",
                                  "vehicles/../vehicles/marv_7in_fc3v2.yaml", "vehicles/../design/budget.yaml", "design/budget.yaml", "/etc/passwd",
                                  str(ROOT / "vehicles" / "marv_7in_fc3v2.yaml"), "vehicles/marv_7in_fc3v2.yaml\0", "vehicles/marv_7in_fc3v2.txt",
                                  "vehicles/sub/x.yaml", "vehicles/.hidden.yaml", "vehicles/marv_7in_fc3v2_inertia_model.py", "sensors/profiles/madflight_fc3v2_revc.yaml",
                                  "~/x.yaml", "vehicles/marv_7in_fc3v2.yaml; touch pwned"])
def test_a_card_that_is_not_a_file_of_the_vehicles_directory_is_refused_before_anything_is_written(tmp_path, standin_steps, card):
    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            for cmd in ("derive", "derive_card"):
                refused = await s.cmd(cmd=cmd, card=card, edits=[])
                assert refused["ok"] is False and ("card" in refused["error"]), (cmd, refused)
            assert not (h.run_dir / "derive").exists() and standin_steps.ran() == [] and not h.derive.busy()
            assert (await s.cmd(cmd="derive_card", card=UZH))["ok"]  # the control: a card of the directory
            s.close()
    run(tmp_path, body)


def test_edit_keys_must_exist_in_the_card_schema_and_each_edit_is_exactly_key_value_source(tmp_path, standin_steps):
    entries = {e["key"]: e for e in derive_mod.card_entries(CARD)["entries"]}
    assert "rotors.speed_range" in entries and entries["rotors.speed_range"]["locked"]
    bad = [
        ({"key": "mess", "value": 1, "source": SOURCE}, "not an editable entry"),
        ({"key": "rotors.m5.position", "value": [1, 1, 1], "source": SOURCE}, "not an editable entry"),
        ({"key": "rotors.esc_map", "value": 1, "source": SOURCE}, "not an editable entry"),
        ({"key": "rotors", "value": 1, "source": SOURCE}, "not an editable entry"),
        ({"key": "mass.value", "value": 1, "source": SOURCE}, "not an editable entry"),
        ({"key": "mass.__class__", "value": 1, "source": SOURCE}, "not an editable entry"),
        ({"key": "../mass", "value": 1, "source": SOURCE}, "not an editable entry"),
        ({"key": "vehicle", "value": "x", "source": SOURCE}, "not an editable entry"),
        ({"key": "sensor_profile", "value": "../../etc/passwd", "source": SOURCE}, "not an editable entry"),
        ({"key": "mass\0", "value": 1, "source": SOURCE}, "not an editable entry"),
        ({"key": 5, "value": 1, "source": SOURCE}, "not an editable entry"),
        ({"key": "mass", "value": 1}, "exactly {key, value, source}"),
        ({"key": "mass", "value": 1, "source": SOURCE, "unit": "g"}, "exactly {key, value, source}"),
        ({"key": "mass", "value": "heavy", "source": SOURCE}, "finite number"),
        ({"key": "mass", "value": True, "source": SOURCE}, "finite number"),
        ({"key": "mass", "value": float("nan"), "source": SOURCE}, "finite number"),
        ({"key": "mass", "value": [], "source": SOURCE}, "finite number"),
        ({"key": "mass", "value": [1, "x"], "source": SOURCE}, "finite number"),
        ({"key": "mass", "value": 1, "source": 5}, "source"),
        ({"key": "mass", "value": 1, "source": "x" * (derive_mod.MAX_SOURCE_CHARS + 1)}, "source"),
        ({"key": "rotors.m1.spin", "value": 1, "source": SOURCE}, "must be text"),
        ({"key": "rotors.speed_range", "value": [0, 2000], "source": SOURCE}, "is locked by Luis"),
    ]
    assert len(bad) == len({json.dumps(b[0], default=str) for b in bad})

    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            for edit, why in bad:
                refused = await s.cmd(cmd="derive", card=CARD, edits=[edit])
                assert refused["ok"] is False and why in refused["error"], (edit, refused)
            for edits, why in (("mass", "list"), ({"key": "mass"}, "list"), ([mass(), mass(0.9)], "twice"), ([mass()] * (derive_mod.MAX_EDITS + 1), "at most")):
                refused = await s.cmd(cmd="derive", card=CARD, edits=edits)
                assert refused["ok"] is False and why in refused["error"], (edits, refused)
            refused = await s.cmd(cmd="derive", card=CARD, edits=[], preset="evil")
            assert refused["ok"] is False and "unknown key" in refused["error"]
            assert standin_steps.ran() == [] and not h.derive.busy() and h.derive._lock_fd is None
            assert not list((h.run_dir / "derive").glob("*/derive.json")) or all(
                json.loads(p.read_text())["state"] == "refused" for p in (h.run_dir / "derive").glob("*/derive.json"))
            ok, done = await derived(s, edits=[mass()])  # the control: a valid edit of an unlocked entry
            assert done["ok"] is True, (done, json.loads(next((h.run_dir / "derive").glob("*/derive.json")).read_text())["error"])
            s.close()
    run(tmp_path, body)


# ---- the provenance refusal ------------------------------------------------------------------------------------------------------

def test_a_card_copy_without_provenance_for_an_edit_is_refused_with_the_lint_finding_and_nothing_heavy_runs(tmp_path, standin_steps):
    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            for blank in ("", "   ", "\n\t"):
                refused = await s.cmd(cmd="derive", card=CARD, edits=[mass(source=blank)])
                assert refused["ok"] is False and "does not lint" in refused["error"], refused
                assert refused["findings"] == [{"file": CARD, "entry": "mass", "reason": "source: must be non-empty text"}], refused
            old = next(e for e in derive_mod.card_entries(CARD)["entries"] if e["key"] == "mass")["source"]
            same = await s.cmd(cmd="derive", card=CARD, edits=[mass(source=old)])  # the old text says nothing about the new value
            assert same["ok"] is False and same["findings"][0]["entry"] == "mass" and "own provenance text" in same["findings"][0]["reason"], same
            both = await s.cmd(cmd="derive", card=CARD, edits=[mass(source=""), {"key": "rotors.thrust_coeff", "value": 3.0e-6, "source": ""}])
            assert [f["entry"] for f in both["findings"]] == ["mass", "rotors.thrust_coeff"], both
            assert standin_steps.ran() == [] and h.derive._lock_fd is None and not h.derive.busy()
            assert not [e for e in s.events if e["event"].startswith("derive_")]
            assert hoststate.heavy_lock_state(tmp_path / "heavy.lock")["held"] is False
            records = [json.loads(p.read_text()) for p in sorted((h.run_dir / "derive").glob("*/derive.json"))]
            assert records and all(r["state"] == "refused" and r["ok"] is False and r["findings"] for r in records)
            # the control: the same edit with a provenance text lints clean and is derived
            ok, done = await derived(s, edits=[mass()])
            assert done["ok"] is True and done["findings"] == [], done
            assert standin_steps.ran() == ["flatten", "params_gen"]
            s.close()
    run(tmp_path, body)


def test_a_value_the_lint_judges_is_refused_with_the_lints_own_finding(tmp_path, standin_steps):
    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            refused = await s.cmd(cmd="derive", card=CARD, edits=[{"key": "rotors.m1.position", "value": [-0.1, 0.1, -0.02], "source": SOURCE}])
            assert refused["ok"] is False and "wrong quadrant" in refused["findings"][0]["reason"], refused
            assert refused["findings"][0]["entry"] == "rotors.m1.position" and standin_steps.ran() == []
            refused = await s.cmd(cmd="derive", card=CARD, edits=[{"key": "rotors.thrust_coeff", "value": [1, 2], "source": SOURCE}])
            assert refused["ok"] is False and refused["findings"], refused
            s.close()
    run(tmp_path, body)


# ---- the work directory ----------------------------------------------------------------------------------------------------------

def test_a_card_without_edits_is_copied_byte_for_byte_and_an_edit_changes_the_value_and_its_source_only(tmp_path, standin_steps):
    import yaml
    sys.path.insert(0, str(ROOT / "tools" / "card"))
    import schema

    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            _, done = await derived(s)
            work = h.run_dir / "derive" / done["id"]
            assert (work / "root" / CARD).read_bytes() == (ROOT / CARD).read_bytes()
            for rel in (derive_mod.BUDGET, derive_mod.SCENARIO):
                assert (work / "root" / rel).read_bytes() == (ROOT / rel).read_bytes()
            assert (work / "root" / "sensors" / "profiles" / "madflight_fc3v2_revc.yaml").read_bytes() == (ROOT / "sensors" / "profiles" / "madflight_fc3v2_revc.yaml").read_bytes()
            rec = json.loads((work / "derive.json").read_text())
            assert rec["edits"] == [] and rec["inputs"][CARD] == hashlib.sha256((ROOT / CARD).read_bytes()).hexdigest()
            assert set(rec["inputs"]) >= {CARD, derive_mod.BUDGET, derive_mod.SCENARIO, *derive_mod.EXTRA_REGISTERS}
            _, done = await derived(s, edits=[mass(), {"key": "rotors.m2.position", "value": [-0.11, -0.13, -0.0233], "source": SOURCE + " (2)"}])
            work = h.run_dir / "derive" / done["id"]
            old, new = schema.load_yaml(ROOT / CARD), schema.load_yaml(work / "root" / CARD)
            assert new["mass"]["value"] == 0.8 and new["mass"]["source"] == SOURCE and new["rotors"]["m2"]["position"]["value"] == [-0.11, -0.13, -0.0233]
            new["mass"].update(value=old["mass"]["value"], source=old["mass"]["source"])
            new["rotors"]["m2"]["position"].update(value=old["rotors"]["m2"]["position"]["value"], source=old["rotors"]["m2"]["position"]["source"])
            assert new == old  # nothing else of the card changed (anchors, dates, lock, status ...)
            rec = json.loads((work / "derive.json").read_text())
            assert [(e["key"], e["old_value"], e["value"]) for e in rec["edits"]] == [("mass", 0.732, 0.8), ("rotors.m2.position", [-0.112, -0.1275, -0.0233], [-0.11, -0.13, -0.0233])]
            assert rec["edits"][0]["old_source"].startswith("Luis, 2026-10-16/17") and rec["describe"]
            s.close()
    run(tmp_path, body)


def test_the_repository_is_not_written_by_a_derivation_w1(tmp_path, standin_steps):
    before = repo_state()

    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            assert (await derived(s, edits=[mass()]))[1]["ok"]
            assert (await derived(s))[1]["ok"]
            refused = await s.cmd(cmd="derive", card=CARD, edits=[mass(source="")])
            assert refused["ok"] is False
            assert h.run_dir.resolve().is_relative_to(tmp_path.resolve()) and len(list((h.run_dir / "derive").iterdir())) == 3
            s.close()
    run(tmp_path, body)
    assert repo_state() == before


def test_the_steps_events_and_the_result_of_a_derivation(tmp_path, standin_steps):
    image = make_flash_root(tmp_path / "build")

    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            started, done = await derived(s, edits=[mass()])
            assert started["started"] is True and started["derive"]["state"] == "running" and started["derive"]["edits"][0]["key"] == "mass"
            assert [e["step"] for e in s.seen("derive_step")] == ["flatten", "params_gen", "mcfg"] and s.seen("derive_started")
            assert done["ok"] is True, (done, json.loads(next((h.run_dir / "derive").glob("*/derive.json")).read_text())["error"])
            assert done["state"] == "done" and done["reports"] == ["x_report.txt"] and done["findings"] == []
            work = h.run_dir / "derive" / done["id"]
            data = (work / f"{derive_mod.PARAMS_SET}.mcfg").read_bytes()
            rec = json.loads((work / "derive.json").read_text())
            assert rec["mcfg"] == {"name": "marv_params_fc3v2.mcfg", "size": len(data), "sha256": hashlib.sha256(data).hexdigest()}
            assert done["config_hash"] == rec["config_hash"] and rec["schema_hash"] == done["schema_hash"] == f"{0x1234ABCD5678EF01:#018x}"
            assert {i["name"] for i in done["images"]} == {"marv_fc3v2", "marv_sil_quad_fc"} and image in {i["hash"] for i in done["images"]}
            assert rec["baseline"]["hash"] in {i["hash"] for i in done["images"]}
            assert [(c["name"], c["from_bits"], c["to_bits"]) for c in rec["changes"]] == [("a_gain", "0x3f800000", "0x40000000")]
            assert [s_["status"] for s_ in rec["steps"]] == [0, 0, 0] and rec["ok"] is True and rec["state"] == "done"
            status = (await s.cmd(cmd="derive_status"))["derive"]
            assert status["params_set"] == "marv_params_fc3v2" and status["running"] is False and status["current"]["ok"] is True
            assert [d["id"] for d in status["derivations"]] == [done["id"]] and status["refusals"] == []
            assert (await s.cmd(cmd="status"))["status"]["derive"]["id"] == done["id"]
            report = await s.cmd(cmd="derive_report", derive_id=done["id"], name="x_report.txt")
            assert report["ok"] and report["text"] == "a derivation report\n"
            values = await s.cmd(cmd="derive_values", derive_id=done["id"])
            assert values["ok"] and values["config_hash"] == done["config_hash"]
            assert [(p["name"], p["bits"], p["type"], p["locked"]) for p in values["params"]] == [("a_gain", 0x40000000, "f32", False), ("b_count", 7, "i32", True)]
            s.close()
    run(tmp_path, body)


def test_the_report_and_values_commands_take_only_names_of_this_runs_derivations(tmp_path, standin_steps):
    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            _, done = await derived(s)
            did = done["id"]
            for bad_id in ("", "..", "../x", f"{did}/..", did + "\0", None, 5, "20260101T000000Z-0"):
                for cmd, extra in (("derive_report", {"name": "x_report.txt"}), ("derive_values", {})):
                    r = await s.cmd(cmd=cmd, derive_id=bad_id, **extra)
                    assert r["ok"] is False and "not a derivation" in r["error"], (cmd, bad_id, r)
            for name in ("", "../derive.json", "derive.json", "x_report.txt/", "../../x_report.txt", "x_report.txt\0", "other_report.txt", None, 5,
                         "x_report.txt; cat", "flatten/x_report.txt"):
                r = await s.cmd(cmd="derive_report", derive_id=did, name=name)
                assert r["ok"] is False, (name, r)
            assert (await s.cmd(cmd="derive_report", derive_id=did, name="x_report.txt"))["ok"]  # the control
            s.close()
    run(tmp_path, body)


# ---- the refusals of the heavy part ----------------------------------------------------------------------------------------------

def test_a_derivation_is_refused_while_the_heavy_lock_is_held_and_runs_when_it_is_free(tmp_path, standin_steps):
    holder = hold_heavy(tmp_path)

    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            status = (await s.cmd(cmd="derive_status"))["derive"]
            assert any("the heavy lock is held" in r for r in status["refusals"])
            refused = await s.cmd(cmd="derive", card=CARD, edits=[mass()])
            assert refused["ok"] is False and "the heavy lock is held" in refused["error"] and "sleep" in refused["error"] and refused["refusals"], refused
            assert standin_steps.ran() == [] and not h.derive.busy() and h.derive._lock_fd is None
            release_heavy(holder)
            assert (await derived(s, edits=[mass()]))[1]["ok"] is True  # the control
            s.close()
    try:
        run(tmp_path, body)
    finally:
        release_heavy(holder)


def test_the_lock_itself_decides_a_holder_that_appears_after_the_check_refuses(tmp_path, standin_steps, monkeypatch):
    holder = None

    async def body(h):
        nonlocal holder
        real = h.derive.refusals

        def late(**kw):
            nonlocal holder
            reasons = real(**kw)
            holder = hold_heavy(tmp_path)
            return reasons
        monkeypatch.setattr(h.derive, "refusals", late)
        async with ws_connect(h) as ws:
            s = Session(ws)
            refused = await s.cmd(cmd="derive", card=CARD, edits=[mass()])
            assert refused["ok"] is False and "the heavy lock is held" in refused["error"], refused
            assert standin_steps.ran() == [] and not h.derive.busy() and h.derive._lock_fd is None
            s.close()
    try:
        run(tmp_path, body)
    finally:
        if holder is not None:
            release_heavy(holder)


def test_a_derivation_holds_the_heavy_lock_while_it_runs_and_releases_it_when_done(tmp_path, standin_steps):
    standin_steps.hold()

    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            assert (await s.cmd(cmd="derive", card=CARD, edits=[mass()]))["ok"]
            assert await asyncio.to_thread(wait_for, lambda: hoststate.heavy_lock_state(tmp_path / "heavy.lock")["held"])
            state = hoststate.heavy_lock_state(tmp_path / "heavy.lock")
            assert "cmd=GCS derive " in state["holder"] and f"pid={os.getpid()} " in state["holder"]
            second = await s.cmd(cmd="derive", card=CARD, edits=[])
            assert second["ok"] is False and "one at a time" in second["error"], second
            standin_steps.release()
            await s.wait_event("derive_done")
            assert hoststate.heavy_lock_state(tmp_path / "heavy.lock")["held"] is False
            s.close()
    run(tmp_path, body)


def test_free_memory_below_the_budget_refuses_and_the_same_request_runs_with_enough(tmp_path, standin_steps, monkeypatch):
    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            monkeypatch.setattr(hoststate, "MEMINFO", _meminfo(tmp_path, 1))
            refused = await s.cmd(cmd="derive", card=CARD, edits=[mass()])
            assert refused["ok"] is False and "below the derivation budget" in refused["error"] and "PLACEHOLDER" in refused["error"], refused
            monkeypatch.setattr(hoststate, "MEMINFO", tmp_path / "no-such-meminfo")
            refused = await s.cmd(cmd="derive", card=CARD, edits=[mass()])
            assert refused["ok"] is False and "cannot be read" in refused["error"], refused
            assert standin_steps.ran() == []
            monkeypatch.setattr(hoststate, "MEMINFO", _meminfo(tmp_path, FREE_ENOUGH_KB))
            assert (await derived(s, edits=[mass()]))[1]["ok"] is True
            s.close()
    run(tmp_path, body)


def test_cancel_stops_the_steps_process_group_and_releases_the_lock(tmp_path, standin_steps):
    standin_steps.hold()

    async def body(h):
        h.derive.stop_s = STOP_S
        async with ws_connect(h) as ws:
            s = Session(ws)
            started = await s.cmd(cmd="derive", card=CARD, edits=[mass()])
            assert started["ok"]
            assert await asyncio.to_thread(wait_for, lambda: h.derive._proc is not None)
            pgid = h.derive._proc.pid
            cancelled = await s.cmd(cmd="derive_cancel")
            assert cancelled["ok"] and cancelled["cancelled"] is True and cancelled["derive"]["state"] == "cancelled", cancelled
            done = await s.wait_event("derive_done")
            assert done["ok"] is False and done["error"] == "cancelled" and done["state"] == "cancelled"
            with pytest.raises(ProcessLookupError):
                os.killpg(pgid, 0)
            assert hoststate.heavy_lock_state(tmp_path / "heavy.lock")["held"] is False and standin_steps.ran() == []
            idle = await s.cmd(cmd="derive_cancel")
            assert idle["ok"] is False and "no derivation is running" in idle["error"]
            assert h.derive.find_config(done["config_hash"] or "0" * 64) is None
            standin_steps.release()
            assert (await derived(s, edits=[mass()]))[1]["ok"] is True  # the control: the next derivation runs
            s.close()
    run(tmp_path, body)


def test_a_failing_step_is_reported_with_its_status_and_leaves_no_config(tmp_path, standin_steps, monkeypatch):
    real = derive_mod.DeriveManager.plan_steps

    def failing(self, paths, work):
        steps = real(self, paths, work)
        steps[1] = {"name": "params_gen", "argv": [sys.executable, "-c", "import sys; print('params_gen: entry x is refused'); sys.exit(3)"]}
        return steps
    monkeypatch.setattr(derive_mod.DeriveManager, "plan_steps", failing)

    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            _, done = await derived(s, edits=[mass()])
            assert done["ok"] is False and done["state"] == "failed" and "params_gen step exited with status 3" in done["error"] and done["config_hash"] is None
            cur = (await s.cmd(cmd="derive_status"))["derive"]["current"]
            assert cur["running"] is False and cur["tail"][-1] == "params_gen: entry x is refused" and "mcfg" not in {x["name"] for x in cur["steps"] if "status" in x}
            assert h.derive.find_config("0" * 64) is None and hoststate.heavy_lock_state(tmp_path / "heavy.lock")["held"] is False
            s.close()
    run(tmp_path, body)


# ---- the config library: the G7 path ---------------------------------------------------------------------------------------------

def test_a_derived_config_is_validated_by_its_hash_and_the_report_store_then_holds_the_pair(tmp_path, standin_steps):
    image = make_flash_root(tmp_path / "build")
    probe = ("import hashlib, json, sys; a = sys.argv; arg = lambda k: a[a.index(k) + 1]; "
             "img = open(arg('--image'), 'rb').read(); cfg = open(arg('--config'), 'rb').read(); "
             "sys.path.insert(0, %r); import mcfg; c = mcfg.decode(cfg); "
             "body = (json.dumps({'verdict': 'PASS', 'image': {'sha256': hashlib.sha256(img).hexdigest()}, "
             "'config': {'config_hash': mcfg.config_hash(c).hex()}}) + chr(10)).encode(); "
             "open(arg('--out'), 'wb').write(body); print('report_id: ' + hashlib.sha256(body).hexdigest()); print('verdict: PASS')"
             % str(ROOT / "tools" / "config"))
    command = [sys.executable, "-c", probe, "--config", "{config}", "--image", "{image}", "--build", "{build}", "--work-dir", "{work}", "--out", "{out}"]

    async def body(h):
        async with ws_connect(h) as ws:
            s = Session(ws)
            unknown = await s.cmd(cmd="validate", image_hash=image, config="5" * 64)
            assert unknown["ok"] is False and "holds no copy of config" in unknown["error"]  # a hash nobody derived
            _, done = await derived(s, edits=[mass()])
            config = done["config_hash"]
            assert h.reports.find_config(config) is None and h.derive.find_config(config) is not None
            started = await s.cmd(cmd="validate", image_hash=image, config=config)
            assert started == {"ok": True, "config_hash": config, "started": True}, started
            end = await s.wait_event("validate_done", image_hash=image)
            assert end["verdict"] == "PASS" and end["config_hash"] == config
            stored = h.reports.directory(image, config)
            assert (stored / "config.mcfg").read_bytes() == (h.run_dir / "derive" / done["id"] / "marv_params_fc3v2.mcfg").read_bytes()
            assert h.reports.report(image, config) is not None and h.reports.find_config(config) is not None
            tampered = h.run_dir / "derive" / done["id"] / "marv_params_fc3v2.mcfg"
            raw = bytearray(tampered.read_bytes())
            raw[len(raw) // 2] ^= 0xFF
            tampered.write_bytes(bytes(raw))  # a file that no longer holds what its record says is not offered
            assert h.derive.find_config(config) is None
            s.close()
    run(tmp_path, body, validate_command=command)
