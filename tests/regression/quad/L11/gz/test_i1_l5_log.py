"""I1 guard (decision 0020 section 2): the L5 plugin path without <vertical_source> writes the log it wrote before the
element existed.

Skipped only as conftest.py says; fails if the host-gz-l5 build is absent. One run of the L5 scenario step_roll at m = 2
and its committed seed on the host-gz-l5 build (tools/sim/run_l5.py run_step). Claims: the log has no record 8 (VERTICAL),
and the SHA-256 and length of its records (everything after the header) equal the values recorded in
tests/regression/quad/L11/i1/l5_step_roll_m2_records.txt from the plugin source of commit e08179e (that file names the
command).

Negative control: the same log bytes with one VERTICAL record planted after the first TRUTH record (where the plugin with
the element would write it) fail both checks.
"""

import hashlib
import struct
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import l11_gz  # noqa: E402

import lockstep_log  # noqa: E402
import run_l5  # noqa: E402
import run_scenario  # noqa: E402

RECORD = l11_gz.ROOT / "tests" / "regression" / "quad" / "L11" / "i1" / "l5_step_roll_m2_records.txt"
VERTICAL = struct.Struct("<BQffI")


def recorded():
    out = {}
    for line in RECORD.read_text(encoding="utf-8").splitlines():
        if line.strip() and not line.startswith("#"):
            key, value = line.split(":", 1)
            out[key.strip()] = value.strip()
    return out


def violations(log, want):
    raw = log["raw_records"]
    out = []
    if any("vertical" in t for t in log["ticks"]):
        out.append("a record 8 (VERTICAL)")
    if hashlib.sha256(raw).hexdigest() != want["records_sha256"] or len(raw) != int(want["records_bytes"]):
        out.append(f"records sha256 {hashlib.sha256(raw).hexdigest()} ({len(raw)} bytes) is not the recorded "
                   f"{want['records_sha256']} ({want['records_bytes']} bytes)")
    return out


@pytest.fixture(scope="module")
def flown(tmp_path_factory):
    assert run_scenario.plugin_present(l11_gz.PLUGIN_DIR_L5), f"{l11_gz.PLUGIN_DIR_L5}: the plugin is not built"
    s = run_l5.run_step(l11_gz.CARD, l11_gz.SOURCE, l11_gz.M, tmp_path_factory.mktemp("i1"), l11_gz.PLUGIN_DIR_L5)
    return s.run


def test_the_l5_log_has_no_vertical_record_and_its_records_are_the_recorded_ones(flown):
    assert run_scenario.complete_trailer(flown.log, flown.iterations, l11_gz.M)
    assert len(flown.log["truths"]) == len(flown.log["ticks"])
    assert violations(flown.log, recorded()) == []


def test_control_a_planted_vertical_record_fails_both_checks(flown, tmp_path):
    data = Path(flown.log_path).read_bytes()
    first_truth = flown.log["header"]["header_size"] + sum(1 + lockstep_log._SIZES[k]
                                                           for k in (lockstep_log.STEP, lockstep_log.TICK,
                                                                     lockstep_log.TRUTH))
    assert data[first_truth - 1 - lockstep_log._SIZES[lockstep_log.TRUTH]] == lockstep_log.TRUTH
    planted = tmp_path / "planted.bin"
    planted.write_bytes(data[:first_truth] + VERTICAL.pack(lockstep_log.VERTICAL, 0, 0.0, 0.0, l11_gz.VALID)
                        + data[first_truth:])
    v = violations(lockstep_log.read(planted), recorded())
    assert len(v) == 2 and v[0] == "a record 8 (VERTICAL)", v
