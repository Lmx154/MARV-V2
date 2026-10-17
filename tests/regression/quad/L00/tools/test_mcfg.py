"""tools/config/mcfg.py: the binary .mcfg (decision 0053 item 3). The format is the module's docstring."""

import hashlib
import importlib.util
import json
import struct
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
GEN = ROOT / "tools" / "gen" / "params_gen.py"
MCFG = ROOT / "tools" / "config" / "mcfg.py"
FIXTURES = ROOT / "tests" / "regression" / "quad" / "L00" / "fixtures"
_spec = importlib.util.spec_from_file_location("mcfg", MCFG)
mcfg = importlib.util.module_from_spec(_spec)
sys.modules["mcfg"] = mcfg
_spec.loader.exec_module(mcfg)

LOCK = "lock: {by: t, on: 2026-10-17, via: manual}"


def entry(name, value, ptype="f32", sigma="choice", method="scenario", extra="", unit="1"):
    return (f'{name}: {{type: {ptype}, value: {value}, unit: "{unit}", method: {method}, source: "src of {name}", '
            f"sigma: {sigma}{extra}}}\n")


def generate(tmp_path, text, name, table=False):
    src = tmp_path / f"{name}.yaml"
    src.write_text(text)
    out = tmp_path / name
    args = [sys.executable, str(GEN), "--register", str(src), "--out-dir", str(out)]
    if table:
        args += ["--out-table", str(tmp_path / f"{name}_table.txt"), "--table-input", f"{name}.yaml"]
    r = subprocess.run(args, capture_output=True, text=True, check=False)
    assert r.returncode == 0, r.stderr
    return out / "params_manifest.json"


def manifest(path):
    return json.loads(Path(path).read_text())


def cli(*args):
    return subprocess.run([sys.executable, str(MCFG), *map(str, args)], capture_output=True, text=True, check=False)


@pytest.fixture(scope="module")
def fixture_manifest(tmp_path_factory):
    tmp = tmp_path_factory.mktemp("fx")
    out = tmp / "gen"
    table = tmp / "table.txt"
    args = [sys.executable, str(GEN), "--card", str(FIXTURES / "l0_card.yaml"), "--register",
            str(FIXTURES / "l0_register.yaml"), "--out-dir", str(out), "--out-table", str(table), "--table-input",
            "fixtures"]
    assert subprocess.run(args, capture_output=True, text=True, check=False).returncode == 0
    return out / "params_manifest.json", table


EXTREMES = (
    entry("neg_zero", "-0.0", sigma="0", method="design-budget", extra="")
    + entry("tiny", "1.401298464324817e-45", sigma="choice")
    + entry("big", "3.4028234663852886e38", sigma="choice")
    + entry("int_min", -(2**31), "i32", sigma="choice")
    + entry("int_max", 2**31 - 1, "i32", sigma="exact", method="datasheet", extra=", " + LOCK)
    + entry("measured", "0.1", sigma="0.003", method="measured", unit="kg")
    + entry("unknown_sigma", "2.5", sigma="UNKNOWN", method="published")
    + entry("derived", "7.0", sigma="UNKNOWN", method='"derived(2 * 3.5)"')
)


@pytest.fixture(scope="module")
def extremes(tmp_path_factory):
    tmp = tmp_path_factory.mktemp("ex")
    return generate(tmp, EXTREMES, "ex", table=True), tmp / "ex_table.txt"


def test_round_trip_is_bit_exact(extremes):
    m = manifest(extremes[0])
    cfg = mcfg.factory_config(m)
    data = mcfg.encode(cfg)
    back = mcfg.decode(data)
    assert back == cfg
    assert mcfg.encode(back) == data
    records = mcfg.section_records(back.section("quad"))
    assert records == mcfg.manifest_records(m)
    by_name = dict(zip(m["params"], records, strict=True))
    assert by_name["neg_zero"].value_bits == 0x80000000
    assert by_name["tiny"].value_bits == 1
    assert by_name["int_min"].value_bits == 0x80000000 and by_name["int_max"].value_bits == 0x7FFFFFFF
    assert by_name["int_max"].locked and not by_name["tiny"].locked
    assert by_name["derived"].method == "derived" and by_name["measured"].sigma_kind == "Known"
    assert back.section("quad").schema_hash == int(m["schema_hash"], 16)


def test_fixture_set_round_trips_and_matches_its_table_bit_for_bit(fixture_manifest, tmp_path):
    man, table = fixture_manifest
    out = tmp_path / "f.mcfg"
    assert cli("encode", "--manifest", man, "--out", out).returncode == 0
    cfg = mcfg.decode(out.read_bytes())
    assert len(mcfg.section_records(cfg.section("quad"))) == manifest(man)["count"]
    r = cli("dump", out, "--compare-table", table)
    assert r.returncode == 0, r.stderr
    assert "match" in r.stdout


def test_compare_table_reports_a_changed_value(extremes, tmp_path):
    man, table = extremes
    out = tmp_path / "e.mcfg"
    assert cli("encode", "--manifest", man, "--out", out).returncode == 0
    assert cli("dump", out, "--compare-table", table).returncode == 0
    bad = tmp_path / "bad.txt"
    bad.write_text(table.read_text().replace("0.100000001", "0.100000009"))
    assert bad.read_text() != table.read_text()
    r = cli("dump", out, "--compare-table", bad)
    assert r.returncode == 1 and "measured" in r.stderr


def test_layout_is_the_documented_bytes():
    rec = mcfg.Record("f32", 0x3F800000, 0, "Choice", "scenario", "DefaultFromRegister", True, "why")
    rec2 = mcfg.Record("i32", 0xFFFFFFFF, 0x3C23D70A, "Known", "measured", "Manual", False, "why")
    section = mcfg.build_section("quad", 0x1122334455667788, [rec, rec2])
    expected_body = (struct.pack("<IIIBBBBB3x", 0x3F800000, 0, 0, 0, 3, 6, 1, 1)
                     + struct.pack("<IIIBBBBB3x", 0xFFFFFFFF, 0x3C23D70A, 0, 1, 0, 1, 2, 0)
                     + b"why\0" )
    assert section.body == expected_body
    cfg = mcfg.Config(7, b"\x01" * 32, b"\x02" * 32, (section,))
    payload = struct.pack("<B3xQIII", 1, 0x1122334455667788, 2, 24, len(expected_body)) + expected_body
    header = (b"MARVMCFG" + struct.pack("<HHIHHI", 1, 128, 7, 1, 0, len(payload)) + b"\x01" * 32 + b"\x02" * 32
              + hashlib.sha256(payload).digest() + bytes(8))
    assert len(header) == 128
    assert mcfg.encode(cfg) == header + payload
    # the config hash: SHA-256 of (kind, schema hash, record count, length) and the body
    want = hashlib.sha256(struct.pack("<B3xQII", 1, 0x1122334455667788, 2, len(expected_body)) + expected_body)
    assert mcfg.config_hash(cfg) == want.digest()


def test_identical_sources_share_one_string(extremes):
    cfg = mcfg.factory_config(manifest(extremes[0]))
    recs = mcfg.section_records(cfg.section("quad"))
    doubled = [mcfg.Record(r.type, r.value_bits, r.sigma_bits, r.sigma_kind, r.method, r.origin, r.locked, "same")
               for r in recs]
    assert mcfg.build_section("quad", 1, doubled).body.endswith(b"same\0\0\0\0")


def test_hash_is_stable_and_covers_exactly_the_section_content(extremes):
    m = manifest(extremes[0])
    a = mcfg.factory_config(m, sequence=1)
    b = mcfg.factory_config(m, sequence=99, image_sha256=b"\xaa" * 32, report_id=b"\xbb" * 32)
    assert mcfg.config_hash(a) == mcfg.config_hash(b), "the header (sequence, binding) is not in the config hash"
    assert mcfg.config_hash(a) == mcfg.config_hash(mcfg.decode(mcfg.encode(a)))
    records = mcfg.manifest_records(m)
    flipped = [records[0].__class__(**{**records[0].__dict__, "value_bits": records[0].value_bits ^ 1}), *records[1:]]
    c = mcfg.Config(1, bytes(32), bytes(32), (mcfg.build_section("quad", int(m["schema_hash"], 16), flipped),))
    assert mcfg.config_hash(c) != mcfg.config_hash(a)


def test_hash_command_prints_the_hash(extremes, tmp_path):
    out = tmp_path / "e.mcfg"
    assert cli("encode", "--manifest", extremes[0], "--out", out).returncode == 0
    cfg = mcfg.decode(out.read_bytes())
    r = cli("hash", out)
    assert r.stdout.strip() == mcfg.config_hash(cfg).hex()
    assert cli("hash", out, "--sections", "quad").stdout.strip() == r.stdout.strip()
    assert cli("hash", out, "--sections", "bogus").returncode == 1


def test_unread_section_is_carried_byte_for_byte(extremes, tmp_path):
    m = manifest(extremes[0])
    rocket = mcfg.build_section("rocket", 0xFEEDFACECAFEBEEF, mcfg.manifest_records(m)[:3])
    common = mcfg.build_section("common", 0x0123456789ABCDEF, mcfg.manifest_records(m)[3:5])
    old = mcfg.Config(4, bytes(32), bytes(32), (common, mcfg.factory_config(m).sections[0], rocket))
    old_path = tmp_path / "old.mcfg"
    old_path.write_bytes(mcfg.encode(old))
    # an image that reads only quad rewrites its section and carries the others
    new_path = tmp_path / "new.mcfg"
    assert cli("encode", "--manifest", extremes[0], "--sequence", 5, "--carry-from", old_path,
               "--out", new_path).returncode == 0
    new = mcfg.decode(new_path.read_bytes())
    assert [s.kind for s in new.sections] == ["common", "quad", "rocket"]
    assert new.section("rocket").body == rocket.body and new.section("common").body == common.body
    assert new.section("rocket") == rocket
    # and its own hash does not move with them
    assert mcfg.config_hash(new, ("quad",)) == mcfg.config_hash(old, ("quad",))
    assert mcfg.config_hash(new) == mcfg.config_hash(old)
    changed = mcfg.build_section("rocket", 0xFEEDFACECAFEBEEF, mcfg.manifest_records(m)[:2])
    other = mcfg.Config(4, bytes(32), bytes(32), (common, old.sections[1], changed))
    assert mcfg.config_hash(other, ("quad",)) == mcfg.config_hash(old, ("quad",))
    assert mcfg.config_hash(other) != mcfg.config_hash(old)


def test_every_corrupted_payload_byte_is_detected(extremes):
    data = mcfg.encode(mcfg.factory_config(manifest(extremes[0])))
    for i in range(mcfg.HEADER_SIZE, len(data)):
        bad = bytearray(data)
        bad[i] ^= 0x01
        with pytest.raises(mcfg.McfgError, match="payload SHA-256"):
            mcfg.decode(bytes(bad))


def test_header_and_length_damage_is_detected(extremes):
    data = mcfg.encode(mcfg.factory_config(manifest(extremes[0])))
    for i in (0, 7, 8, 10, 16, 20, 88, 120):
        bad = bytearray(data)
        bad[i] ^= 0x01
        with pytest.raises(mcfg.McfgError):
            mcfg.decode(bytes(bad))
    with pytest.raises(mcfg.McfgError, match="file is"):
        mcfg.decode(data[:-1])
    with pytest.raises(mcfg.McfgError, match="file is"):
        mcfg.decode(data + b"\0")
    with pytest.raises(mcfg.McfgError, match="shorter"):
        mcfg.decode(data[:100])


def test_unknown_format_version_is_refused(extremes):
    data = bytearray(mcfg.encode(mcfg.factory_config(manifest(extremes[0]))))
    data[8] = 2
    with pytest.raises(mcfg.McfgError, match="format version 2"):
        mcfg.decode(bytes(data))


def test_inconsistent_sigma_is_refused():
    bad = mcfg.Record("f32", 0, 0x3F800000, "Choice", "scenario", "Manual", False, "s")
    with pytest.raises(mcfg.McfgError, match="sigma"):
        mcfg.build_section("quad", 1, [bad])


def test_corrupt_directory_is_refused_even_with_a_valid_hash(extremes):
    cfg = mcfg.factory_config(manifest(extremes[0]))
    data = bytearray(mcfg.encode(cfg))
    payload = bytearray(data[mcfg.HEADER_SIZE:])
    payload[0] = 9  # an unknown section kind
    data[88:120] = hashlib.sha256(bytes(payload)).digest()
    data[mcfg.HEADER_SIZE:] = payload
    with pytest.raises(mcfg.McfgError, match="kind 9"):
        mcfg.decode(bytes(data))


def test_view_round_trips_through_encode(extremes, tmp_path):
    out = tmp_path / "e.mcfg"
    view = tmp_path / "e.json"
    again = tmp_path / "again.mcfg"
    assert cli("encode", "--manifest", extremes[0], "--sequence", 3, "--out", out).returncode == 0
    assert cli("decode", out, "--out", view).returncode == 0
    assert cli("encode", "--json", view, "--out", again).returncode == 0
    assert again.read_bytes() == out.read_bytes()


def test_dump_names_the_records(extremes, tmp_path):
    out = tmp_path / "e.mcfg"
    assert cli("encode", "--manifest", extremes[0], "--out", out).returncode == 0
    doc = json.loads(cli("dump", out, "--names", extremes[0]).stdout)
    records = doc["sections"][0]["records"]
    assert doc["sections"][0]["named"] and [r["name"] for r in records] == list(manifest(extremes[0])["params"])
    assert records[3]["value"] == -(2**31) and records[3]["link_id"] == "int_min"
    assert doc["config_hash"] == mcfg.config_hash(mcfg.decode(out.read_bytes())).hex()
    assert "name" not in json.loads(cli("dump", out).stdout)["sections"][0]["records"][0]


def test_migrate_carries_by_name_and_reports_the_rest(tmp_path):
    old_m = generate(tmp_path, entry("keep", 1.0) + entry("retype", 2.0) + entry("reunit", 3.0, unit="m")
                     + entry("gone", 4.0) + entry("pinned", 5.0, extra=", " + LOCK), "old")
    new_m = generate(tmp_path, entry("new_one", 9.0) + entry("keep", 1.0) + entry("retype", 2, "i32")
                     + entry("reunit", 3.0, unit="mm") + entry("pinned", 5.0, extra=", " + LOCK), "new")
    old = mcfg.factory_config(manifest(old_m), sequence=6, image_sha256=b"\x11" * 32)
    records = mcfg.section_records(old.section("quad"))
    edited = [mcfg.Record(r.type, r.value_bits ^ (0x100 if i in (0, 4) else 0), r.sigma_bits, r.sigma_kind, r.method,
                          "Manual" if i in (0, 4) else r.origin, r.locked, r.source) for i, r in enumerate(records)]
    schema = int(manifest(old_m)["schema_hash"], 16)
    rocket = mcfg.build_section("rocket", 5, records[:1])
    old = mcfg.Config(6, b"\x11" * 32, b"\x22" * 32, (mcfg.build_section("quad", schema, edited), rocket))
    old_path = tmp_path / "old.mcfg"
    old_path.write_bytes(mcfg.encode(old))
    new_path = tmp_path / "new.mcfg"
    report_path = tmp_path / "report.json"
    r = cli("migrate", old_path, "--old-manifest", old_m, "--manifest", new_m, "--out", new_path, "--report",
            report_path)
    assert r.returncode == 0, r.stderr
    report = json.loads(report_path.read_text())
    assert report["carried"] == ["keep"] and report["added"] == ["new_one"] and report["dropped"] == ["gone"]
    assert [c["name"] for c in report["changed"]] == ["retype", "reunit"]
    assert report["locked_reset"] == ["pinned"]
    new = mcfg.decode(new_path.read_bytes())
    assert new.sequence == 7 and new.image_sha256 == bytes(32) and new.report_id == bytes(32)
    assert new.section("quad").schema_hash == int(manifest(new_m)["schema_hash"], 16)
    got = dict(zip(manifest(new_m)["params"], mcfg.section_records(new.section("quad")), strict=True))
    assert got["keep"] == edited[0] and got["keep"].origin == "Manual"
    defaults = dict(zip(manifest(new_m)["params"], mcfg.manifest_records(manifest(new_m)), strict=True))
    for name in ("new_one", "retype", "reunit", "pinned"):
        assert got[name] == defaults[name], name
    assert new.section("rocket").body == rocket.body, "an unread section is carried byte for byte"


def test_migrate_refuses_a_file_that_is_not_the_old_schema(tmp_path):
    a = generate(tmp_path, entry("a", 1.0), "a")
    b = generate(tmp_path, entry("b", 1.0), "b")
    path = tmp_path / "a.mcfg"
    assert cli("encode", "--manifest", a, "--out", path).returncode == 0
    r = cli("migrate", path, "--old-manifest", b, "--manifest", a, "--out", tmp_path / "x.mcfg")
    assert r.returncode == 1 and "schema hash" in r.stderr
