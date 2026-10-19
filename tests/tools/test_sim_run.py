"""The interactive SIL's run definition (ground-segment unit GX-3): tools/sim/run_quad_fc.py --interactive with --card, --override
and --gui, the shared display-only visuals (tools/sim/display_world.py), the checks both the run and the GCS helper make
(tools/sim/sil_run.py), and the validator's refusal of a record made by a SIL override (invariant O-1).

No gz and no build: the worlds are text, the parameter set is a small synthetic manifest, the validator's records check runs on
a synthetic config. The helper's side (run_plan / run_start with these options) is in test_gs_helper.py."""

import dataclasses
import difflib
import json
import re
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
for sub in ("sim", "card", "config", "validate"):
    sys.path.insert(0, str(ROOT / "tools" / sub))

import display_world  # noqa: E402
import gen_world  # noqa: E402
import mcfg  # noqa: E402
import run_quad_fc  # noqa: E402
import sil_run  # noqa: E402
import validate_config  # noqa: E402

OTHER_CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
TYPES = {"att_kp": "f32", "rate_ki_roll": "f32", "rc_channel_roll": "i32", "rc_channel_arm": "i32"}  # a stand-in parameter set
LINK = run_quad_fc.link_xml(14550, "/tmp/some/store")


def reference_world(tmp_path, card, link):
    """The interactive world as run_interactive wrote it before the card, override and gui options (the code of b457325, inline)."""
    scenario, _, _ = run_quad_fc.fc3_scenario(tmp_path)
    total_us = run_quad_fc.us(run_quad_fc.INTERACTIVE_S)
    phases = [run_quad_fc.Phase("idle", 0, total_us, run_quad_fc.SWITCH_LOW)]
    sensors = gen_world.SensorSet(gyro="model", bias_signs=(0,) * 6)
    name, text = gen_world.generate(card, scenario, "pilot", run_quad_fc.M, "lo", None, ROOT, sensors=sensors,
                                    command_delay_ticks=run_quad_fc.COMMAND_DELAY_TICKS)
    return name, run_quad_fc.sdf_edit_for(phases, (), 0.0, link, ground=True)(text)


def changes(old, new):
    """The (removed lines, added lines) from `old` to `new`."""
    diff = [d for d in difflib.unified_diff(old.splitlines(), new.splitlines(), lineterm="", n=0)
            if not d.startswith(("---", "+++", "@@"))]
    return [d[1:] for d in diff if d[0] == "-"], [d[1:] for d in diff if d[0] == "+"]


# ---- the world: byte-identical without the options, different exactly by them -------------------------------------------------

@pytest.mark.parametrize("link", ["", LINK])
def test_world_without_the_options_is_byte_identical_to_the_one_before_them(tmp_path, link):
    name, text = run_quad_fc.interactive_world(tmp_path, link)
    ref_name, ref = reference_world(tmp_path, run_quad_fc.CARD, link)
    assert (name, text) == (ref_name, ref)
    assert name == run_quad_fc.interactive_world(tmp_path, link, run_quad_fc.CARD, [], False)[0]
    assert run_quad_fc.interactive_world(tmp_path, link, run_quad_fc.CARD, [], False)[1] == ref  # explicit defaults too
    assert 'param="att_kp"' not in ref and "scene-broadcaster" not in ref and "display_ground" not in ref


def test_overrides_add_exactly_one_sil_override_each_with_its_type_and_nothing_else(tmp_path):
    overrides = sil_run.check_overrides([("att_kp", 3.5), ("rc_channel_roll", 7)], TYPES)
    _, base = run_quad_fc.interactive_world(tmp_path, LINK)
    _, edited = run_quad_fc.interactive_world(tmp_path, LINK, run_quad_fc.CARD, overrides, False)
    removed, added = changes(base, edited)
    assert removed == []
    assert added == ['<sil_override param="att_kp" type="f32">3.5</sil_override>',
                     '<sil_override param="rc_channel_roll" type="i32">7</sil_override>']
    assert edited.count("<sil_override") == base.count("<sil_override") + 2
    # control: each override alone adds only its own line, and the other order gives the other order
    _, one = run_quad_fc.interactive_world(tmp_path, LINK, run_quad_fc.CARD, overrides[:1], False)
    assert changes(base, one) == ([], [added[0]])
    _, swapped = run_quad_fc.interactive_world(tmp_path, LINK, run_quad_fc.CARD, overrides[::-1], False)
    assert changes(base, swapped) == ([], added[::-1])


def test_gui_adds_only_the_display_block_and_leaves_the_plant_elements_alone(tmp_path):
    _, base = run_quad_fc.interactive_world(tmp_path, LINK)
    _, shown = run_quad_fc.interactive_world(tmp_path, LINK, run_quad_fc.CARD, [], True)
    assert shown == display_world.display_world(base) != base
    removed, added = changes(base, shown)
    assert {r.strip() for r in removed} <= {a.strip() for a in added}  # only lines it moves (the display block is inserted mid-line)
    assert {line.strip() for line in base.splitlines()} <= {line.strip() for line in shown.splitlines()}  # nothing of the world is lost
    plugin = re.compile(r'<plugin filename="marv_gz_lockstep".*?</plugin>', re.S)
    assert plugin.search(shown).group(0) == plugin.search(base).group(0)
    assert shown.count("<visual") > base.count("<visual") and "SceneBroadcaster" in shown and "SceneBroadcaster" not in base
    # the display block has no mass, inertia or collision of its own
    assert not re.search(r"<collision|<mass>|<inertia>", "\n".join(added))
    # all three together: the same two edits, independent of each other
    overrides = sil_run.check_overrides([("att_kp", 2.0)], TYPES)
    _, both = run_quad_fc.interactive_world(tmp_path, LINK, run_quad_fc.CARD, overrides, True)
    _, only_override = run_quad_fc.interactive_world(tmp_path, LINK, run_quad_fc.CARD, overrides, False)
    assert both == display_world.display_world(only_override)


def test_card_changes_the_plant_of_the_world_and_is_otherwise_the_same_recipe(tmp_path):
    name, other = run_quad_fc.interactive_world(tmp_path, LINK, OTHER_CARD)
    assert (name, other) == reference_world(tmp_path, OTHER_CARD, LINK)
    base_name, base = run_quad_fc.interactive_world(tmp_path, LINK)
    assert name != base_name and other != base
    for element in ("<launch_pad/>", "<ground/>", "<manual_control_script>", "<link_udp_port>14550</link_udp_port>", "<gyro_source>"):
        assert element in other and element in base


# ---- the run: the definition is written, the GUI is started paused and stopped, the options are checked --------------------------

def fake_gz(monkeypatch, status=0, fail=False):
    calls = {}

    def run(cmd, **kwargs):
        calls["cmd"] = cmd
        if fail:
            raise OSError("gz did not start")
        return subprocess.CompletedProcess(cmd, status)
    monkeypatch.setattr(run_quad_fc.subprocess, "run", run)
    monkeypatch.setattr(display_world, "start_gui", lambda *a, **k: calls.setdefault("gui", a) and ("gui-process", "timer"))
    monkeypatch.setattr(display_world, "stop_gui", lambda gui_p, timer: calls.setdefault("stopped", (gui_p, timer)))
    return calls


def test_the_run_definition_records_every_override_and_labels_the_run(tmp_path, monkeypatch):
    calls = fake_gz(monkeypatch)
    overrides = sil_run.check_overrides([("att_kp", 3.5), ("rc_channel_arm", 6)], TYPES)
    assert run_quad_fc.run_interactive(tmp_path, tmp_path, "", OTHER_CARD, overrides, False) == 0
    d = json.loads((tmp_path / "interactive" / "run_definition.json").read_text())
    assert d == {"schema": 1, "card": "vehicles/uzh_neurobem_5in.yaml", "gui": False, "label": "sil-override",
                 "overrides": [{"name": "att_kp", "type": "f32", "value": 3.5}, {"name": "rc_channel_arm", "type": "i32", "value": 6}]}
    assert "-r" in calls["cmd"] and "gui" not in calls  # not paused, no GUI client
    # control: a run without overrides is not labelled
    run_quad_fc.run_interactive(tmp_path, tmp_path, "", run_quad_fc.CARD, [], False)
    d = json.loads((tmp_path / "interactive" / "run_definition.json").read_text())
    assert d["label"] is None and d["overrides"] == [] and d["card"] == "vehicles/marv_7in_fc3v2.yaml"
    world = next((tmp_path / "interactive").glob("*.sdf")).read_text()
    assert "att_kp" not in world


def test_gui_run_starts_the_server_paused_and_always_stops_the_client(tmp_path, monkeypatch):
    calls = fake_gz(monkeypatch)
    run_quad_fc.run_interactive(tmp_path, tmp_path, "", run_quad_fc.CARD, [], True)
    assert "-r" not in calls["cmd"] and calls["gui"][3] == "run_quad_fc" and calls["stopped"] == ("gui-process", "timer")
    calls = fake_gz(monkeypatch, fail=True)
    with pytest.raises(OSError):  # the client is stopped when the server does not even start
        run_quad_fc.run_interactive(tmp_path, tmp_path, "", run_quad_fc.CARD, [], True)
    assert calls["stopped"] == ("gui-process", "timer")


@pytest.fixture
def plugin_tree(tmp_path):
    """A build tree with a parameter manifest where run_quad_fc looks for it: <build>/sim/gz/plugin and <build>/generated/..."""
    plugin = tmp_path / "build" / "sim" / "gz" / "plugin"
    plugin.mkdir(parents=True)
    manifest = tmp_path / "build" / sil_run.MANIFEST_REL
    manifest.parent.mkdir(parents=True)
    manifest.write_text(json.dumps({"params": {n: {"type": t} for n, t in TYPES.items()}}))
    return plugin


def parse(monkeypatch, plugin, *args):
    got = {}
    monkeypatch.setattr(run_quad_fc, "run_interactive", lambda *a: got.setdefault("args", a) and 0)
    code = run_quad_fc.main(["--out-dir", "o", "--plugin-dir", str(plugin), *args])
    return code, got.get("args")


def test_command_line_passes_the_card_overrides_and_gui_on(plugin_tree, monkeypatch):
    code, args = parse(monkeypatch, plugin_tree, "--interactive", "--card", "vehicles/uzh_neurobem_5in.yaml",
                       "--override", "att_kp=3.5", "--override", "rc_channel_roll=7", "--gui")
    assert code == 0
    _, plugin_dir, link, card, overrides, gui = args
    assert card == OTHER_CARD and gui is True and link == ""
    assert overrides == [{"name": "att_kp", "type": "f32", "value": 3.5}, {"name": "rc_channel_roll", "type": "i32", "value": 7}]
    code, args = parse(monkeypatch, plugin_tree, "--interactive")  # control: no options, today's run
    assert code == 0 and args[3:] == (run_quad_fc.CARD, [], False)


@pytest.mark.parametrize("bad", [
    ["--card", "vehicles/no_such_card.yaml"],  # unknown card
    ["--card", "vehicles"],  # a directory
    ["--override", "no_such_parameter=1"],  # unknown parameter
    ["--override", "att_kp=nan"], ["--override", "att_kp=inf"], ["--override", "att_kp=1e39"], ["--override", "att_kp=abc"],  # outside f32
    ["--override", "rc_channel_roll=1.5"], ["--override", "rc_channel_roll=2147483648"],  # outside i32
    ["--override", "att_kp"], ["--override", "att_kp="], ["--override", "=3"],  # not name=value
    ["--override", "att_kp=1", "--override", "att_kp=2"],  # a name twice
])
def test_command_line_refuses_an_unknown_card_parameter_or_value_outside_its_domain(plugin_tree, monkeypatch, capsys, bad):
    with pytest.raises(SystemExit) as e:
        parse(monkeypatch, plugin_tree, "--interactive", *bad)
    assert e.value.code == 2 and capsys.readouterr().err.strip()


def test_command_line_refuses_overrides_without_a_parameter_manifest_and_the_options_without_interactive(tmp_path, monkeypatch, capsys):
    (tmp_path / "plugin" / "a" / "b").mkdir(parents=True)  # no build tree around it: no manifest
    for args in (["--interactive", "--override", "att_kp=1"], ["--override", "att_kp=1"], ["--card", "vehicles/uzh_neurobem_5in.yaml"],
                 ["--gui"]):
        with pytest.raises(SystemExit) as e:
            parse(monkeypatch, tmp_path / "plugin" / "a" / "b", *args)
        assert e.value.code == 2, args
    assert "build host-gz-quadfc first" in capsys.readouterr().err


def test_check_overrides_accepts_numbers_of_the_types_domain_and_orders_nothing():
    got = sil_run.check_overrides([("rc_channel_roll", 3.0), ("att_kp", 1), ("rate_ki_roll", "0.25")], TYPES)
    assert got == [{"name": "rc_channel_roll", "type": "i32", "value": 3}, {"name": "att_kp", "type": "f32", "value": 1.0},
                   {"name": "rate_ki_roll", "type": "f32", "value": 0.25}]
    assert [sil_run.cli_arg(o) for o in got] == ["rc_channel_roll=3", "att_kp=1.0", "rate_ki_roll=0.25"]
    assert sil_run.check_overrides([("rc_channel_roll", sil_run.INT32_MAX), ("rc_channel_arm", sil_run.INT32_MIN)], TYPES)
    for bad in (True, None, [1], float("nan"), float("inf"), 1e39, "x"):
        with pytest.raises(ValueError):
            sil_run.check_overrides([("att_kp", bad)], TYPES)
    for bad in (1.5, sil_run.INT32_MAX + 1, sil_run.INT32_MIN - 1, float("inf"), "1.5"):
        with pytest.raises(ValueError):
            sil_run.check_overrides([("rc_channel_roll", bad)], TYPES)


# ---- O-1: the validator refuses a record made by a SIL override --------------------------------------------------------------

def synthetic_manifest():
    def entry(i, ptype, bits, locked=False):
        return {"id": i, "type": ptype, "unit": "1", "link_id": f"p{i}", "lock": locked, "default_bits": bits, "method": "scenario",
                "sigma_kind": "Choice", "sigma": 0.0, "sigma_bits": "0x00000000", "origin": "DefaultFromRegister",
                "source": f"register entry {i}", "source_file": "register.yaml"}
    params = {"att_kp": entry(0, "f32", "0x40a00000"), "rc_channel_arm": entry(1, "i32", "0x00000005", locked=True)}
    return {"schema_hash": "0x00000000000000ab", "schema_hash_algorithm": "test", "count": len(params), "params": params}


CHECK = {"id": "records", "title": "record, provenance and lock checks"}
TABLE = {"fault": "none", "validator": "accepted", "verdict": "accepted"}


def records_result(source_of_first, source_of_locked=None):
    manifest = synthetic_manifest()
    cfg = mcfg.factory_config(manifest)
    records = mcfg.section_records(cfg.sections[0])
    records[0] = dataclasses.replace(records[0], source=source_of_first)
    if source_of_locked is not None:
        records[1] = dataclasses.replace(records[1], source=source_of_locked)
    section = mcfg.build_section("quad", mcfg.manifest_schema_hash(manifest), records)
    cfg = mcfg.decode(mcfg.encode(dataclasses.replace(cfg, sections=(section,))))
    return validate_config.check_records(CHECK, cfg, None, manifest, TABLE)


def test_validator_refuses_a_record_whose_source_is_sil_override_and_accepts_the_same_config_without_it():
    control = records_result("Luis 2026-10-19: a gain from the bench")
    assert control["result"] == "PASS" and control["failures"] == []
    refused = records_result(sil_run.LABEL)
    assert refused["result"] == "FAIL"
    assert len(refused["failures"]) == 1 and "att_kp" in refused["failures"][0] and "O-1" in refused["failures"][0]
    # the label as the plugin writes it (padding and a trailing note do not hide it), and on a locked parameter too
    assert records_result("  sil-override: from the Simulation page ")["result"] == "FAIL"
    # a hand-edited case change does not hide it either
    assert records_result("SIL-Override (edited)")["result"] == "FAIL"
    both = records_result("fine", sil_run.LABEL)
    assert both["result"] == "FAIL" and "rc_channel_arm" in " ".join(both["failures"])
    # control: the word inside a longer source text, not at its start, is a source like any other
    assert records_result("a note about the sil-override label")["result"] == "PASS"


def test_the_label_the_validator_refuses_is_the_one_the_firmware_writes_and_the_run_uses():
    source = (ROOT / "fw" / "sil" / "src" / "param_override.cpp").read_text(encoding="utf-8")
    assert f'r.source = "{validate_config.SIL_OVERRIDE_SOURCE}";' in source
    assert sil_run.LABEL == validate_config.SIL_OVERRIDE_SOURCE
