"""The truth-attitude channel on the host side (decision 0006 section B): record type 5 (TRUTH) of the lockstep log
(tools/sim/lockstep_log.py, home sim/gz/plugin/src/lockstep_log.hpp) and the optional <attitude_source> element of the
world generator (tools/card/gen_world.py).

The test builds logs from the layout written out here (u64 tick, then the 48 bytes of marv_truth_state: u32 struct_size,
u32 flags, u64 tick, 4 x f32 q, 3 x f32 omega, 4 bytes of padding), independently of the reader's own struct. Negative
controls: a perturbed field is read back perturbed, a record out of place or cut is refused, and a world generated
without the option differs from the one with it by exactly that element.
"""

import re
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
sys.path.insert(0, str(ROOT / "tools" / "sim"))
import gen_world  # noqa: E402
import lockstep_log as ll  # noqa: E402

CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
SCEN = ROOT / "scenarios" / "quad" / "L02"
TRUTH_TYPE = 5
TRUTH_STATE_SIZE = 48
VALID_FLAG = 1
# Scenario values: 2 ticks per host step, 3 host steps, the tick period 625/4 us.
M = 2
STEPS = 3
PERIOD_NUM_US, PERIOD_DEN = 625, 4


def header_bytes(m=M):
    version = b"test"
    fixed = struct.Struct("<8sIIIIIQI")
    return fixed.pack(b"MARVLOCK", 1, fixed.size + len(version), m, PERIOD_NUM_US, PERIOD_DEN, 1, len(version)) + version


def step_record(i):
    return bytes([1]) + struct.pack("<QQ26d", i, i, *([0.0] * 26))


def tick_record(tick):
    return bytes([2]) + struct.pack("<QQ32s4HII20d", tick, tick, bytes(32), 48, 48, 48, 48, 1, 0, *([0.0] * 20))


def truth_state(tick, q, omega, flags=VALID_FLAG):
    return struct.pack("<IIQ4f3f4x", TRUTH_STATE_SIZE, flags, tick, *q, *omega)


def truth_record(tick, state=None, q=(1.0, 0.0, 0.0, 0.0), omega=(0.0, 0.0, 0.0)):
    body = truth_state(tick, q, omega) if state is None else state
    return bytes([TRUTH_TYPE]) + struct.pack("<Q", tick) + body


def applied_record():
    return bytes([3]) + struct.pack("<6d", *([0.0] * 6))


def trailer_record(n_ticks):
    return bytes([4]) + struct.pack("<QQQ", STEPS, n_ticks, STEPS)


def f32(x):
    return struct.unpack("<f", struct.pack("<f", x))[0]


def expected_q(step, tick):
    return (f32(0.5 + 0.125 * step), f32(-0.25 * tick), f32(0.0625), f32(1.0 / 3.0))


def expected_omega(step):
    return (f32(0.1 * step), f32(-2.0 / 3.0), f32(1.0e-3 * (step + 1)))


def log_with_truth(with_truth=True):
    out = bytearray(header_bytes())
    tick = 0
    for i in range(STEPS):
        out += step_record(i)
        for _ in range(M):
            out += tick_record(tick)
            if with_truth:
                out += truth_record(tick, q=expected_q(i, tick), omega=expected_omega(i))
            tick += 1
        out += applied_record()
    out += trailer_record(tick)
    return bytes(out)


def read(tmp_path, data):
    p = tmp_path / "log.bin"
    p.write_bytes(data)
    return ll.read(p)


def test_type_5_is_parsed_and_follows_each_tick(tmp_path):
    log = read(tmp_path, log_with_truth())
    assert len(log["ticks"]) == STEPS * M and len(log["truths"]) == STEPS * M
    assert log["trailer"] == {"steps": STEPS, "ticks": STEPS * M, "applied": STEPS}
    for tick, t in enumerate(log["truths"]):
        step = tick // M
        assert t["tick"] == t["state_tick"] == log["ticks"][tick]["tick"] == tick
        assert t["struct_size"] == TRUTH_STATE_SIZE and t["flags"] == VALID_FLAG
        assert t["q_wxyz"] == expected_q(step, tick)
        assert t["omega_frd"] == expected_omega(step)
        assert t["raw"] == truth_state(tick, expected_q(step, tick), expected_omega(step)) and len(t["raw"]) == TRUTH_STATE_SIZE


def test_a_log_without_type_5_has_no_truths_and_its_records_are_untouched(tmp_path):
    data = log_with_truth(with_truth=False)
    log = read(tmp_path, data)
    assert log["truths"] == []
    assert log["raw_records"] == data[len(header_bytes()):]
    assert len(log["ticks"]) == STEPS * M and len(log["steps"]) == STEPS and len(log["applied"]) == STEPS


def test_negative_control_a_perturbed_field_is_read_back_perturbed(tmp_path):
    """The parse is live: one float ulp in q or omega, or one flag bit, changes what is read."""
    good = read(tmp_path, log_with_truth())["truths"][0]
    tick = 0
    for i in range(4):
        q = list(expected_q(0, tick))
        q[i] = struct.unpack("<f", struct.pack("<I", struct.unpack("<I", struct.pack("<f", q[i]))[0] + 1))[0]
        data = header_bytes() + step_record(0) + tick_record(0) + truth_record(0, q=q, omega=expected_omega(0))
        bad = ll.read(_write(tmp_path, data))["truths"][0]
        assert bad["q_wxyz"] != good["q_wxyz"] and bad["omega_frd"] == good["omega_frd"]
    data = (header_bytes() + step_record(0) + tick_record(0) +
            bytes([TRUTH_TYPE]) + struct.pack("<Q", 0) + truth_state(0, expected_q(0, 0), expected_omega(0), flags=0))
    assert ll.read(_write(tmp_path, data))["truths"][0]["flags"] != good["flags"]


def _write(tmp_path, data):
    p = tmp_path / "bad.bin"
    p.write_bytes(data)
    return p


def test_a_truth_record_that_does_not_follow_its_tick_is_refused(tmp_path):
    head = header_bytes() + step_record(0)
    with pytest.raises(ll.LogError, match="TRUTH"):
        ll.read(_write(tmp_path, head + truth_record(0)))  # after a STEP, no TICK yet
    with pytest.raises(ll.LogError, match="TRUTH"):
        ll.read(_write(tmp_path, head + tick_record(0) + truth_record(1)))  # the tick of another tick
    with pytest.raises(ll.LogError, match="TRUTH"):
        ll.read(_write(tmp_path, head + tick_record(0) + applied_record() + truth_record(0)))  # after an APPLIED
    with pytest.raises(ll.LogError, match="TRUTH"):
        ll.read(_write(tmp_path, head + tick_record(0) + truth_record(0) + truth_record(0)))  # twice


def test_a_cut_truth_record_is_refused(tmp_path):
    data = header_bytes() + step_record(0) + tick_record(0) + truth_record(0)
    for cut in (1, 9, TRUTH_STATE_SIZE):
        with pytest.raises(ll.LogError, match="cut"):
            ll.read(_write(tmp_path, data[:-cut]))


def test_the_reader_knows_the_record_size_the_plugin_writes():
    assert ll._TRUTH.size == struct.calcsize("<Q") + TRUTH_STATE_SIZE
    assert ll.TRUTH == TRUTH_TYPE


# ---- gen_world ------------------------------------------------------------------------------------------------------

ELEMENT = "<attitude_source>truth</attitude_source>"


def world(**kw):
    return gen_world.generate(CARD, SCEN / "free_fall.yaml", "test", 1, None, "/tmp/log.bin", **kw)


def plugin_children(text):
    plugin = ET.fromstring(text).find(".//plugin[@filename='marv_gz_lockstep']")
    return [c.tag for c in plugin]


def test_the_world_has_no_attitude_source_by_default_and_with_it_differs_by_that_element_only():
    name, plain = world()
    name_on, text = world(attitude_source=True)
    assert name_on == name
    assert "attitude_source" not in plain and plain == gen_world.generate(
        CARD, SCEN / "free_fall.yaml", "test", 1, None, "/tmp/log.bin")[1]
    assert text.count(ELEMENT) == 1
    assert plugin_children(text).count("attitude_source") == 1
    assert "gyro_source" not in text  # the generator does not write it; the caller does (run_l4.py)
    without = re.sub(r"\n[ \t]*" + re.escape(ELEMENT), "", text)
    assert without == plain
    # Negative control: the equality is not vacuous.
    assert text != plain and re.sub(r"\n[ \t]*" + re.escape(ELEMENT), "", plain) == plain


def test_the_element_is_inside_the_plugin_and_before_log_path():
    _, text = world(attitude_source=True)
    kids = plugin_children(text)
    assert kids.index("attitude_source") < kids.index("log_path")
    assert ET.fromstring(text).find(".//plugin/attitude_source").text == "truth"


def test_the_command_line_flag_writes_the_element(tmp_path):
    base = ["--card", str(CARD), "--scenario", str(SCEN / "free_fall.yaml"), "--mode", "test", "--m", "1"]
    assert gen_world.main(base + ["--out-dir", str(tmp_path / "off")]) == 0
    assert gen_world.main(base + ["--attitude-source", "--out-dir", str(tmp_path / "on")]) == 0
    off = next((tmp_path / "off").glob("*.sdf")).read_text()
    on = next((tmp_path / "on").glob("*.sdf")).read_text()
    assert ELEMENT not in off and ELEMENT in on
