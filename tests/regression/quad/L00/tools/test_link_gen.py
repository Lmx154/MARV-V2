"""Drift guard of the link generators (ground segment unit 5; GS-3 row of the ground spec).

tools/link/mavxml.py derives CRC_EXTRA and the payload lengths from the dialect XML by the published rules; the C++ codec
uses pymavlink's mavgen output, the TypeScript codec uses mavxml. The two must agree for every message of the dialect, and
the committed gcs/src/lib/link/generated/messages.ts must be what tools/link/gen_ts.py writes now. The standard definitions
come from tools/link/fetch_defs.py (the pinned commit, hash-checked); pymavlink is a build and test tool only.
"""

import re
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
LINK_TOOLS = ROOT / "tools" / "link"
sys.path.insert(0, str(LINK_TOOLS))
import mavxml  # noqa: E402

CRCS_RE = re.compile(r"#define MAVLINK_MESSAGE_CRCS \{(.*)\}")
ROW_RE = re.compile(r"\{(\d+), (\d+), (\d+), (\d+), \d+, \d+, \d+\}")


@pytest.fixture(scope="module")
def defs(tmp_path_factory):
    out = tmp_path_factory.mktemp("link_defs")
    subprocess.run([sys.executable, str(LINK_TOOLS / "fetch_defs.py"), "--out-dir", str(out)], check=True, capture_output=True)
    return out


@pytest.fixture(scope="module")
def dialect(defs):
    return mavxml.parse_dialect(defs / "marv.xml", [defs])


def test_crc_extra_and_lengths_equal_mavgen_for_every_message(defs, dialect, tmp_path):
    c_dir = tmp_path / "c"
    subprocess.run(
        [sys.executable, "-m", "pymavlink.tools.mavgen", "--lang", "C", "--wire-protocol", "2.0", "--output", str(c_dir),
         str(defs / "marv.xml")],
        check=True, capture_output=True)
    header = (c_dir / "marv" / "marv.h").read_text()
    match = CRCS_RE.search(header)
    assert match, "MAVLINK_MESSAGE_CRCS not found in the mavgen header"
    table = {int(i): (int(crc), int(lo), int(hi)) for i, crc, lo, hi in ROW_RE.findall(match.group(1))}
    assert len(table) == len(dialect.messages) > 0
    mismatches = []
    for msg in dialect.messages.values():
        got = (msg.crc_extra, msg.min_len, msg.max_len)
        if table.get(msg.id) != got:
            mismatches.append((msg.name, msg.id, got, table.get(msg.id)))
    assert not mismatches, f"(name, id, mavxml, mavgen) differ: {mismatches}"


def test_committed_messages_ts_equals_gen_ts_output(defs, tmp_path):
    out = tmp_path / "messages.ts"
    subprocess.run(
        [sys.executable, str(LINK_TOOLS / "gen_ts.py"), "--dialect", str(ROOT / "link" / "marv.xml"), "--include-dir", str(defs),
         "--scope", str(ROOT / "link" / "scope.txt"), "--out", str(out)],
        check=True, capture_output=True)
    committed = ROOT / "gcs" / "src" / "lib" / "link" / "generated" / "messages.ts"
    assert out.read_text() == committed.read_text(), "regenerate: tools/link/gen_ts.py --dialect link/marv.xml --include-dir <fetch_defs dir> --scope link/scope.txt --out <this file>"



# PB-1 (ground-segment unit GX-9): manual control never goes on the MAVLink link. The SIL pilot bridge sends RC frames to the plugin's
# own port (tools/gs/pilot.py); IF-8 to a real vehicle stays PARKED.
MANUAL_CONTROL_MESSAGES = ("MANUAL_CONTROL", "RC_CHANNELS_OVERRIDE")


def scope_names(text):
    return {line.strip() for line in text.splitlines() if line.strip() and not line.lstrip().startswith("#")}


def test_pb1_no_manual_control_message_is_in_the_link_scope_or_its_generated_codec():
    scope = scope_names((ROOT / "link" / "scope.txt").read_text())
    assert scope, "the scope file lists messages"
    assert not scope & set(MANUAL_CONTROL_MESSAGES)
    generated = (ROOT / "gcs" / "src" / "lib" / "link" / "generated" / "messages.ts").read_text()
    assert not any(name in generated for name in MANUAL_CONTROL_MESSAGES)


def test_pb1_the_check_sees_a_manual_control_message_planted_in_a_scope():
    planted = (ROOT / "link" / "scope.txt").read_text() + "MANUAL_CONTROL\n"
    assert scope_names(planted) & set(MANUAL_CONTROL_MESSAGES) == {"MANUAL_CONTROL"}
