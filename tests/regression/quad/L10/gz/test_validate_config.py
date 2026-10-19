"""The configuration validator (core G7; ground-segment unit 9, decision 0053): tools/validate/validate_config.py on the FC3v2
parameter set of the host-gz-quadfc build, run as the helper runs it (a .mcfg file in, a report out).

Skipped unless `gz sim --force-version 8` reports 8.x and the host-gz-quadfc build holds libmarv_gz_lockstep.so
(`cmake --preset host-gz-quadfc && cmake --build --preset host-gz-quadfc`, the full build: the validator runs the build's
config-validation ctests and marv_validate_table); the gz CI step fails on a skip.

What is checked (requirements, not outputs: the required-check list is tools/validate/required_checks.yaml, signed by Luis):
  - The factory config passes every required check (verdict PASS, exit 0), and the report's id is the SHA-256 of its bytes.
  - A config whose rate gain is past its margin (rate_kp_roll x RATE_GAIN_FACTOR, a labelled scenario value in the spirit of the
    QF-3 negative control's gains x 1.1) fails: verdict FAIL, exit 1, the ctest check FAIL naming qf3_table_fc3v2, and the
    report shows the roll loop's margin as a negative number against the image's limit (the margin is the report's evidence of
    why, not a bool).
  - Control for check 4: a config with the loops disabled (both rate integral gains and the attitude gain zero, parameters of the
    config like any other) runs through the same check-4 function as the factory config and FAILS on the behaviour predicates
    (not on a refused run), so a passing hover is not the check's default.
"""

import hashlib
import json
import re
import struct
import subprocess
import sys
from dataclasses import replace
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "sim"))
sys.path.insert(0, str(ROOT / "tools" / "config"))
sys.path.insert(0, str(ROOT / "tools" / "validate"))
import mcfg  # noqa: E402
import run_scenario  # noqa: E402
import sil_check  # noqa: E402

VALIDATOR = ROOT / "tools" / "validate" / "validate_config.py"
CARD = ROOT / "vehicles" / "marv_7in_fc3v2.yaml"
BUDGET = ROOT / "design" / "budget.yaml"
BUILD = ROOT / "build" / "host-gz-quadfc"
PLUGIN_DIR = BUILD / "sim" / "gz" / "plugin"
PLUGIN = PLUGIN_DIR / "libmarv_gz_lockstep.so"
PARAMS_DIR = BUILD / "generated" / "marv_params_fc3v2"
FACTORY_MCFG = PARAMS_DIR / "marv_params_fc3v2.mcfg"
MANIFEST = PARAMS_DIR / "marv" / "params" / "params_manifest.json"

VALIDATE_TIMEOUT_S = 3600  # labelled harness value: bounds a hung validator run only (two gz runs and the ctests)
RATE_GAIN_FACTOR = 1.2  # labelled scenario value: rate_kp_roll x this is past the QF-3 rate margin (the margin is read from the report)
GAIN_PARAM = "rate_kp_roll"
LOOP_OFF_PARAMS = ("rate_ki_roll", "rate_ki_pitch", "att_kp")  # zero: the rate integral and the attitude loops do nothing
REQUIRED = ("records", "composition", "ctest", "sil")  # the ids of tools/validate/required_checks.yaml, in its order

pytestmark = pytest.mark.skipif(not (PLUGIN.exists() and run_scenario.gz8_available()),
                                reason="gz-sim 8 or the host-gz-quadfc build of libmarv_gz_lockstep.so is absent")


def config_with(values, path):
    """The factory config with `values` {name: f32} applied as Manual records; written to path, returned decoded."""
    manifest = mcfg.load_manifest(MANIFEST)
    factory = mcfg.decode(FACTORY_MCFG.read_bytes())
    records = mcfg.section_records(factory.section("quad"))
    names = list(manifest["params"])
    for name, value in values.items():
        i = names.index(name)
        assert records[i].type == "f32" and not records[i].locked, name
        bits = struct.unpack("<I", struct.pack("<f", value))[0]
        records[i] = replace(records[i], value_bits=bits, origin="Manual", method="scenario", sigma_kind="Choice", sigma_bits=0,
                             source="test_validate_config.py: a scenario value of the validator's frozen test, not a tuning")
    cfg = mcfg.Config(0, bytes(mcfg.DIGEST), bytes(mcfg.DIGEST), (mcfg.build_section("quad", mcfg.manifest_schema_hash(manifest), records),))
    path.write_bytes(mcfg.encode(cfg))
    return manifest, mcfg.decode(path.read_bytes())


def factory_value(name):
    manifest = mcfg.load_manifest(MANIFEST)
    record = mcfg.section_records(mcfg.decode(FACTORY_MCFG.read_bytes()).section("quad"))[list(manifest["params"]).index(name)]
    return mcfg.f32_of(record.value_bits)


def validate(config, work):
    """Runs the validator as the helper does; (exit status, stdout lines as a dict, the report's bytes, the report)."""
    work.mkdir(exist_ok=True)
    out = work / "report.json"
    proc = subprocess.run([sys.executable, str(VALIDATOR), "--config", str(config), "--build", str(BUILD), "--work-dir", str(work / "work"),
                           "--out", str(out)], capture_output=True, text=True, timeout=VALIDATE_TIMEOUT_S, check=False)
    fields = dict(line.split(": ", 1) for line in proc.stdout.splitlines() if re.match(r"^(report|report_id|verdict): ", line))
    assert out.exists(), f"no report (exit {proc.returncode}):\n{proc.stderr[-3000:]}"
    data = out.read_bytes()
    return proc.returncode, fields, data, json.loads(data)


def check_of(report, check_id):
    return next(c for c in report["checks"] if c["id"] == check_id)


@pytest.fixture(scope="module")
def factory_run(tmp_path_factory):
    return validate(FACTORY_MCFG, tmp_path_factory.mktemp("validate_factory"))


@pytest.fixture(scope="module")
def rate_gain_run(tmp_path_factory):
    tmp = tmp_path_factory.mktemp("validate_rate_gain")
    config = tmp / "rate_gain.mcfg"
    config_with({GAIN_PARAM: factory_value(GAIN_PARAM) * RATE_GAIN_FACTOR}, config)
    return validate(config, tmp / "run")


def test_factory_config_passes_every_required_check(factory_run):
    status, fields, data, report = factory_run
    assert [c["id"] for c in report["checks"]] == list(REQUIRED)
    assert {c["id"]: c["result"] for c in report["checks"]} == {i: "PASS" for i in REQUIRED}, [
        (c["id"], c["failures"]) for c in report["checks"] if c["failures"]]
    assert report["verdict"] == "PASS" and status == 0 and fields["verdict"] == "PASS"
    assert fields["report_id"] == hashlib.sha256(data).hexdigest(), "the report id is not the SHA-256 of the report's bytes"
    factory = mcfg.decode(FACTORY_MCFG.read_bytes())
    assert report["config"]["config_hash"] == mcfg.config_hash(factory).hex()
    assert report["image"]["sha256"] == hashlib.sha256((BUILD / "sim" / "quad_fc" / "libmarv_sil_quad_fc.so").read_bytes()).hexdigest()


def test_a_rate_gain_past_its_margin_fails_with_the_margin_shown(factory_run, rate_gain_run):
    status, fields, data, report = rate_gain_run
    assert report["verdict"] == "FAIL" and status == 1 and fields["verdict"] == "FAIL"
    assert fields["report_id"] == hashlib.sha256(data).hexdigest()
    assert report["config"]["config_hash"] != factory_run[3]["config"]["config_hash"]
    ctest = check_of(report, "ctest")
    assert ctest["result"] == "FAIL" and any("qf3_table_fc3v2" in f for f in ctest["failures"]), ctest["failures"]
    roll = [v for v in ctest["values"] if v.startswith("  qf3_table: rate roll ")]
    assert len(roll) == 1 and roll[0].endswith("FAIL"), roll
    margins = re.findall(r"margin ([+-][0-9.e+-]+)", roll[0])
    assert len(margins) == 2 and any(float(m) < 0 for m in margins), f"no negative margin in: {roll[0]}"
    assert "limit" in roll[0]
    for passing in ("records", "composition"):
        assert check_of(report, passing)["result"] == "PASS"


def test_control_the_loops_disabled_fail_check_4_on_the_behaviour_predicates(tmp_path):
    manifest, cfg = config_with({name: 0.0 for name in LOOP_OFF_PARAMS}, tmp_path / "loops_off.mcfg")
    result = sil_check.run_check(manifest, cfg, CARD, PLUGIN_DIR, tmp_path / "check4", BUDGET)
    assert result["result"] == "FAIL"
    assert "error" not in result, f"the run was refused rather than judged: {result.get('error')}"
    failed = [p["name"] for p in result["predicates"] if not p["ok"]]
    assert failed, "no behaviour predicate failed with the loops disabled"
