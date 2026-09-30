"""Card flattening (tools/card/flatten.py) and the L1 run report (tools/card/report.py). Frozen L01 tests.

The oracles are read from the committed card, profile and budget independently of the tools: the spin text by a line
scan of the raw file, the entry sets by a walk of the YAML, the hashes by a second implementation of the framing.
"""

import hashlib
import re
import shutil
import struct
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[5]
FLATTEN = ROOT / "tools" / "card" / "flatten.py"
REPORT = ROOT / "tools" / "card" / "report.py"
GEN = ROOT / "tools" / "gen" / "params_gen.py"
CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
PROFILE = ROOT / "sensors" / "profiles" / "marv_v2_board_default.yaml"
BUDGET = ROOT / "design" / "budget.yaml"

sys.path.insert(0, str(ROOT / "tools" / "card"))
import report  # noqa: E402
import schema  # noqa: E402

MOTORS = (1, 2, 3, 4)
VALUE_UNKNOWN_BUDGET = ("p", "k", "b", "E", "false_alarm_budgets", "cv_phase_budget", "batch_plan")


def f32(x):
    return struct.unpack("<f", struct.pack("<f", x))[0]


def run(*args):
    return subprocess.run([sys.executable, *map(str, args)], capture_output=True, text=True, check=False)


def flatten(tmp_path, card=CARD, budget=BUDGET, root=ROOT, name="flat"):
    out_card = tmp_path / name / "card.yaml"
    out_reg = tmp_path / name / "register.yaml"
    r = run(FLATTEN, "--card", card, "--budget", budget, "--out-card", out_card, "--out-register", out_reg,
            "--root", root)
    return r, out_card, out_reg


def generate(tmp_path, out_card, out_reg, name="gen"):
    out = tmp_path / name
    r = run(GEN, "--card", out_card, "--register", out_reg, "--out-dir", out)
    assert r.returncode == 0, r.stderr
    return out


DEFAULT_RE = re.compile(
    r"    // (\w+)\n"
    r"    \{\{ParamType::(F32|I32), ([-+0-9.e]+)f?, (-?\d+)\}, ([-+0-9.e]+)f, "
    r"ParamOrigin::(\w+), ParamMethod::(\w+), (true|false),\n"
    r"     SigmaKind::(\w+),\n"
)


def parse_defaults(text):
    rows = {}
    for m in DEFAULT_RE.finditer(text):
        name, ptype, fval, ival, sigma, origin, method, locked, kind = m.groups()
        rows[name] = {"type": ptype, "f": float(fval), "i": int(ival), "sigma": float(sigma), "origin": origin,
                      "method": method, "locked": locked == "true", "kind": kind}
    return rows


@pytest.fixture(scope="module")
def real(tmp_path_factory):
    tmp = tmp_path_factory.mktemp("real")
    r, out_card, out_reg = flatten(tmp)
    assert r.returncode == 0, r.stderr
    gen = generate(tmp, out_card, out_reg)
    return {
        "card_text": out_card.read_text(encoding="utf-8"),
        "register_text": out_reg.read_text(encoding="utf-8"),
        "ids": (gen / "param_ids.hpp").read_text(),
        "rows": parse_defaults((gen / "param_defaults.cpp").read_text()),
        "manifest": yaml.safe_load((gen / "params_manifest.json").read_text(encoding="utf-8")),
        "provenance": yaml.safe_load((gen / "params_provenance.json").read_text(encoding="utf-8")),
        "tmp": tmp,
    }


def card_spins():
    """Spin direction per motor from a line scan of the raw card, not from YAML."""
    spins, motor, in_spin = {}, None, False
    for line in CARD.read_text(encoding="utf-8").splitlines():
        m = re.fullmatch(r"  m([1-4]):", line)
        if m:
            motor, in_spin = int(m.group(1)), False
        elif re.fullmatch(r"    spin:", line):
            in_spin = True
        elif re.fullmatch(r"    position:", line):
            in_spin = False
        elif in_spin and motor:
            v = re.fullmatch(r"      value: (ccw|cw)", line)
            if v:
                spins[motor] = v.group(1)
    return spins


def read_param_ids(path):
    """One id per line; blank lines and '#' comments ignored."""
    lines = (ln.split("#", 1)[0].strip() for ln in path.read_text(encoding="utf-8").splitlines())
    return [ln for ln in lines if ln]


# Each layer that adds product-set parameters commits its own manifest, tests/regression/quad/Lnn/param_ids
# (decision 0004). EXPECTED_IDS is L1's: the card and budget ids that flatten.py writes by default.
EXPECTED_IDS = read_param_ids(ROOT / "tests" / "regression" / "quad" / "L01" / "param_ids")


def product_set_ids():
    """The union of every layer's manifest; the manifests must be disjoint."""
    ids = [i for p in sorted((ROOT / "tests" / "regression" / "quad").glob("L[0-9][0-9]/param_ids"))
           for i in read_param_ids(p)]
    assert len(ids) == len(set(ids)), "a parameter id is listed in more than one layer manifest"
    return ids


def test_generated_ids_are_the_card_and_budget_ids(real):
    ids = re.findall(r"^  (\w+) = \d+,$", real["ids"], flags=re.M)
    assert sorted(ids) == sorted(EXPECTED_IDS)
    assert len(ids) == len(set(ids))


def test_value_unknown_budget_entries_are_absent(real):
    for name in VALUE_UNKNOWN_BUDGET:
        assert name not in real["rows"], name
        assert not re.search(rf"^{re.escape(name)}:", real["register_text"], flags=re.M)
    assert "esc_map" not in real["card_text"].replace("rotors.esc_map", "")


def test_yaw_sign_follows_the_card_spin_text(real):
    spins = card_spins()
    assert spins == {1: "ccw", 2: "ccw", 3: "cw", 4: "cw"}
    for n in MOTORS:
        row = real["rows"][f"rotor_yaw_sign_m{n}"]
        assert row["type"] == "I32"
        assert row["i"] == (1 if spins[n] == "ccw" else -1)
        assert row["kind"] == "Exact"
        assert row["method"] == "Derived"
    assert [real["rows"][f"rotor_yaw_sign_m{n}"]["i"] for n in MOTORS] == [1, 1, -1, -1]


def test_skipped_entries_are_listed_in_the_header(real):
    head = [ln for ln in real["card_text"].splitlines() if ln.startswith("#")]
    text = "\n".join(head)
    assert "rotors.esc_map" in text and "rotors.motor_lag.model" in text
    assert "not flattened at L1" in text and "L6" in text
    reg_head = "\n".join(ln for ln in real["register_text"].splitlines() if ln.startswith("#"))
    for name in VALUE_UNKNOWN_BUDGET:
        assert f"  {name}: value UNKNOWN" in reg_head


def expected_numbers():
    """(generated name, f32 value, unit, sigma kind, sigma or None) per generated parameter, from the YAML."""
    card = schema.load_yaml(CARD)
    out = {}

    def scalar(name, e):
        s = e["sigma"]
        kind = "Unknown" if s == "UNKNOWN" else "Known"
        out[name] = (f32(e["value"]), e["unit"], kind, f32(s) if kind == "Known" else 0.0)

    def vector(name, e, suffixes):
        for i, suf in enumerate(suffixes):
            s = e["sigma"][i] if isinstance(e["sigma"], list) else e["sigma"]
            kind = "Unknown" if s == "UNKNOWN" else "Known"
            out[name + suf] = (f32(e["value"][i]), e["unit"], kind, f32(s) if kind == "Known" else 0.0)

    scalar("mass", card["mass"])
    vector("inertia", card["inertia_diag"], ("_xx", "_yy", "_zz"))
    r = card["rotors"]
    scalar("rotor_thrust_coeff", r["thrust_coeff"])
    scalar("rotor_torque_ratio", r["torque_ratio"])
    vector("rotor_speed", r["speed_range"], ("_min", "_max"))
    scalar("motor_tau", r["motor_lag"]["tau"])
    for n in MOTORS:
        vector(f"rotor_position_m{n}", r[f"m{n}"]["position"], ("_x", "_y", "_z"))
    budget = schema.load_yaml(BUDGET)
    for name in ("PM_min", "chi2_gate_quantile"):
        out[name] = (f32(budget[name]["value"]), budget[name]["unit"], "Choice", 0.0)
    return out


def test_every_generated_value_equals_the_card_value_bit_exactly(real):
    expected = expected_numbers()
    assert len(expected) == len(EXPECTED_IDS) - 4
    for name, (value, unit, kind, sigma) in expected.items():
        row = real["rows"][name]
        assert row["type"] == "F32"
        assert struct.pack("<f", f32(row["f"])) == struct.pack("<f", value), name
        assert row["kind"] == kind, name
        assert struct.pack("<f", f32(row["sigma"])) == struct.pack("<f", sigma), name
        assert real["manifest"]["params"][name]["unit"] == unit, name
        assert real["manifest"]["params"][name]["type"] == "f32", name
        assert row["origin"] == ("DefaultFromRegister" if kind == "Choice" else "DefaultFromCard"), name
        assert not row["locked"], name


def test_provenance_matches_the_card_entries(real):
    by_name = {e["name"]: e for e in real["provenance"]["entries"]}
    assert by_name["inertia"]["shape"] == "diag3"
    assert by_name["rotor_speed"]["shape"] == "range"
    assert by_name["rotor_position_m3"]["components"] == [f"rotor_position_m3_{a}" for a in "xyz"]
    assert by_name["mass"]["sigma_kind"] == ["Known"]
    assert by_name["inertia"]["sigma_kind"] == ["Unknown"] * 3
    assert by_name["rotor_yaw_sign_m2"]["sigma_kind"] == ["Exact"]
    assert by_name["PM_min"]["sigma_kind"] == ["Choice"]
    assert all(e["lock"] is None for e in by_name.values())
    assert by_name["mass"]["source_file"] != by_name["PM_min"]["source_file"]


def test_flatten_is_byte_stable(tmp_path, real):
    r, out_card, out_reg = flatten(tmp_path, name="again")
    assert r.returncode == 0, r.stderr
    assert out_card.read_text(encoding="utf-8") == real["card_text"]
    assert out_reg.read_text(encoding="utf-8") == real["register_text"]


def test_locked_card_entry_reaches_the_generated_records(tmp_path):
    doc = schema.load_yaml(CARD)
    doc["mass"]["lock"] = {"by": "luis", "on": "2026-09-29", "via": "manual"}
    card = tmp_path / "locked_card.yaml"
    card.write_text(yaml.safe_dump(doc, allow_unicode=True, sort_keys=False), encoding="utf-8")
    r, out_card, out_reg = flatten(tmp_path, card=card)
    assert r.returncode == 0, r.stderr
    gen = generate(tmp_path, out_card, out_reg)
    rows = parse_defaults((gen / "param_defaults.cpp").read_text())
    assert rows["mass"]["locked"] and not rows["motor_tau"]["locked"]
    prov = {e["name"]: e for e in yaml.safe_load((gen / "params_provenance.json").read_text())["entries"]}
    assert prov["mass"]["lock"] == {"by": "luis", "on": "2026-09-29", "via": "manual"}


def failing_card(tmp_path, mutate, name="bad_card.yaml"):
    doc = schema.load_yaml(CARD)
    mutate(doc)
    p = tmp_path / name
    p.write_text(yaml.safe_dump(doc, allow_unicode=True, sort_keys=False), encoding="utf-8")
    return p


@pytest.mark.parametrize("mutate,needle", [
    (lambda d: d["rotors"]["m1"].pop("spin"), "rotors.m1.spin"),
    (lambda d: d["mass"].update(sigma=0), "mass: sigma"),
    (lambda d: d.update(sensor_profile="no_such_profile"), "sensor_profile"),
])
def test_a_card_failing_the_linter_writes_nothing(tmp_path, mutate, needle):
    bad = failing_card(tmp_path, mutate)
    r, out_card, out_reg = flatten(tmp_path, card=bad)
    assert r.returncode == 1, r.stdout + r.stderr
    assert needle in r.stderr
    assert not out_card.exists() and not out_reg.exists() and not out_card.parent.exists()


def test_a_budget_failing_the_linter_writes_nothing(tmp_path):
    doc = schema.load_yaml(BUDGET)
    doc["PM_min"]["sigma"] = 0.1
    bad = tmp_path / "bad_budget.yaml"
    bad.write_text(yaml.safe_dump(doc, allow_unicode=True, sort_keys=False), encoding="utf-8")
    r, out_card, out_reg = flatten(tmp_path, budget=bad)
    assert r.returncode == 1, r.stdout + r.stderr
    assert "PM_min" in r.stderr
    assert not out_card.exists() and not out_reg.exists()


# ---- the report ------------------------------------------------------------------------------------------------


def framed(files, root):
    h = hashlib.sha256()
    for f in files:
        data = Path(f).read_bytes()
        rel = Path(f).resolve().relative_to(Path(root).resolve()).as_posix()
        h.update(rel.encode() + b"\x1f" + str(len(data)).encode() + b"\x1f" + data)
    return h.hexdigest()


def header_value(text, key):
    m = re.search(rf"^{key}: (.*)$", text, flags=re.M)
    assert m, f"no {key} line"
    return m.group(1)


def walk(node, path=""):
    """(path, entry) for each mapping with a method and a value or model, in the raw YAML."""
    if isinstance(node, dict):
        if "method" in node and ("value" in node or "model" in node):
            yield path, node
            return
        for k, v in node.items():
            yield from walk(v, f"{path}.{k}" if path else str(k))


def expected_sets():
    docs = {"card": yaml.safe_load(CARD.read_text(encoding="utf-8")),
            "profile": yaml.safe_load(PROFILE.read_text(encoding="utf-8")),
            "budget": yaml.safe_load(BUDGET.read_text(encoding="utf-8"))}
    sets = {"UNVERIFIED": set(), "INFERRED": set(), "sigma UNKNOWN": set(), "value UNKNOWN": set()}
    for label, doc in docs.items():
        for path, e in walk(doc):
            key = f"{label}:{path}"
            for tag in ("UNVERIFIED", "INFERRED"):
                if tag in (e.get("status") or []):
                    sets[tag].add(key)
            s = e.get("sigma")
            if s == "UNKNOWN" or (isinstance(s, list) and "UNKNOWN" in s):
                sets["sigma UNKNOWN"].add(key)
            if e.get("value") == "UNKNOWN":
                sets["value UNKNOWN"].add(key)
    return sets


def sections(text):
    out, title = {}, None
    for line in text.splitlines():
        m = re.fullmatch(r"(UNVERIFIED|INFERRED|sigma UNKNOWN|value UNKNOWN|design-budget|locked) \((\d+)\)", line)
        if m:
            title = m.group(1)
            out[title] = []
        elif line.startswith("  ") and title and line.strip() != "(none)":
            out[title].append(line)
        elif not line.strip():
            title = None
    return out


def listing_problems(text, expected):
    """Every difference between the report's four doubt lists and the expected sets."""
    secs = sections(text)
    problems = []
    for title, want in expected.items():
        got = {ln.split()[0] for ln in secs.get(title, [])}
        for missing in sorted(want - got):
            problems.append(f"{title}: missing {missing}")
        for extra in sorted(got - want):
            problems.append(f"{title}: unexpected {extra}")
    return problems


@pytest.fixture(scope="module")
def real_report():
    return report.build_report(CARD, BUDGET, ROOT)


def test_report_hashes_are_computed_independently(real_report):
    assert header_value(real_report, "card hash") == framed([CARD, PROFILE], ROOT)
    assert header_value(real_report, "budget hash") == framed([BUDGET], ROOT)
    assert len(header_value(real_report, "card hash")) == 64


def test_report_lists_every_doubt_of_the_real_card(real_report):
    expected = expected_sets()
    assert {"card:mass", "card:rotors.thrust_coeff", "card:rotors.m1.spin", "card:inertia_diag"} <= set().union(
        *expected.values())
    assert listing_problems(real_report, expected) == []
    for title in ("UNVERIFIED", "INFERRED", "sigma UNKNOWN", "value UNKNOWN"):
        for line in sections(real_report)[title]:
            assert "source: " in line and len(line.split("source: ", 1)[1]) > 0, line


def test_negative_control_the_listing_checker_catches_a_dropped_line(real_report):
    expected = expected_sets()
    lines = real_report.splitlines()
    for i, line in enumerate(lines):
        if line.startswith("  card:mass  source:"):
            del lines[i]
            break
    else:
        pytest.fail("card:mass not listed")
    problems = listing_problems("\n".join(lines) + "\n", expected)
    assert problems == ["UNVERIFIED: missing card:mass"]


def test_report_lists_design_budget_entries_with_value_or_unknown(real_report):
    lines = sections(real_report)["design-budget"]
    budget = yaml.safe_load(BUDGET.read_text(encoding="utf-8"))
    assert [ln.split()[0] for ln in lines] == [f"budget:{n}" for n in budget]
    by = {ln.split()[0]: ln for ln in lines}
    assert f"value: {budget['PM_min']['value']}" in by["budget:PM_min"]
    assert "value: UNKNOWN" in by["budget:batch_plan"]


def test_report_states_the_model_and_is_stable(real_report):
    assert real_report.splitlines()[-1] == (
        "model: perfect-model (truth and firmware from the same card, not dispersed; core §6)")
    assert report.build_report(CARD, BUDGET, ROOT) == real_report


def test_report_generator_commit_line(real_report):
    assert re.fullmatch(r"[0-9a-f]{40}( dirty)?|UNKNOWN", header_value(real_report, "generator commit"))


@pytest.fixture
def copy_root(tmp_path):
    root = tmp_path / "repo"
    for src in (CARD, PROFILE, BUDGET):
        dst = root / src.relative_to(ROOT)
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(src, dst)
    return root


def test_report_outside_a_git_tree_says_unknown(copy_root):
    text = report.build_report(copy_root / "vehicles" / CARD.name, copy_root / "design" / "budget.yaml", copy_root)
    assert header_value(text, "generator commit") == "UNKNOWN"
    assert header_value(text, "card hash") == framed(
        [copy_root / "vehicles" / CARD.name, copy_root / "sensors" / "profiles" / PROFILE.name], copy_root)
    assert header_value(text, "card hash") == framed([CARD, PROFILE], ROOT)


def test_changing_one_profile_byte_changes_the_card_hash_only(copy_root):
    card, budget = copy_root / "vehicles" / CARD.name, copy_root / "design" / "budget.yaml"
    profile = copy_root / "sensors" / "profiles" / PROFILE.name
    before = report.build_report(card, budget, copy_root)
    data = bytearray(profile.read_bytes())
    i = data.index(b"a")
    data[i] = ord("b")
    profile.write_bytes(bytes(data))
    after = report.build_report(card, budget, copy_root)
    assert header_value(after, "card hash") != header_value(before, "card hash")
    assert header_value(after, "card hash") == framed([card, profile], copy_root)
    assert header_value(after, "budget hash") == header_value(before, "budget hash")


def test_changing_one_budget_byte_changes_the_budget_hash_only(copy_root):
    card, budget = copy_root / "vehicles" / CARD.name, copy_root / "design" / "budget.yaml"
    before = report.build_report(card, budget, copy_root)
    data = bytearray(budget.read_bytes())
    data[data.index(b"0.999")] = ord("1")
    budget.write_bytes(bytes(data))
    after = report.build_report(card, budget, copy_root)
    assert header_value(after, "budget hash") != header_value(before, "budget hash")
    assert header_value(after, "card hash") == header_value(before, "card hash")


def test_report_shows_a_locked_entry_with_by_on_via(copy_root):
    card, budget = copy_root / "vehicles" / CARD.name, copy_root / "design" / "budget.yaml"
    doc = schema.load_yaml(card)
    doc["mass"]["lock"] = {"by": "luis", "on": "2026-09-29", "via": "manual"}
    card.write_text(yaml.safe_dump(doc, allow_unicode=True, sort_keys=False), encoding="utf-8")
    text = report.build_report(card, budget, copy_root)
    locked = sections(text)["locked"]
    assert len(locked) == 1
    assert locked[0].startswith("  card:mass  lock by luis on 2026-09-29 via manual  source: ")
    assert "locked (1)" in text
    assert "locked (0)" in report.build_report(CARD, BUDGET, ROOT)


def test_report_cli_writes_the_same_text(tmp_path, real_report):
    out = tmp_path / "sub" / "l1_report.txt"
    r = run(REPORT, "--card", CARD, "--budget", BUDGET, "--root", ROOT, "--out", out)
    assert r.returncode == 0, r.stderr
    assert out.read_text(encoding="utf-8") == real_report
    r = run(REPORT, "--card", tmp_path / "missing.yaml", "--budget", BUDGET, "--root", ROOT)
    assert r.returncode == 1


# ---- CMake -----------------------------------------------------------------------------------------------------


def cmake_configure(build, *defs):
    cmd = ["cmake", "-S", str(ROOT), "-B", str(build), "-G", "Ninja", "-DMARV_TARGET=host",
           "-DCMAKE_BUILD_TYPE=Debug", f"-DMARV_PYTHON={sys.executable}", *defs]
    return subprocess.run(cmd, capture_output=True, text=True, check=False)


def cmake_build(build):
    return subprocess.run(["cmake", "--build", str(build), "--target", "marv_params_generated", "marv_params_report"],
                          capture_output=True, text=True, check=False)


needs_cmake = pytest.mark.skipif(shutil.which("cmake") is None or shutil.which("ninja") is None,
                                 reason="cmake and ninja are needed")


@needs_cmake
def test_cmake_product_set_comes_from_the_card_and_writes_the_report(tmp_path):
    build = tmp_path / "build"
    r = cmake_configure(build)
    assert r.returncode == 0, r.stderr + r.stdout
    r = cmake_build(build)
    assert r.returncode == 0, r.stderr + r.stdout
    header = (build / "generated" / "marv_params" / "marv" / "params" / "param_ids.hpp").read_text()
    ids = re.findall(r"^  (\w+) = \d+,$", header, flags=re.M)
    assert sorted(ids) == sorted(product_set_ids())
    text = (build / "generated" / "marv_params" / "l1_report.txt").read_text(encoding="utf-8")
    assert header_value(text, "card hash") == framed([CARD, PROFILE], ROOT)


@needs_cmake
def test_cmake_build_fails_when_the_card_fails_the_linter(tmp_path):
    bad = failing_card(tmp_path, lambda d: d["rotors"]["m3"].pop("spin"))
    build = tmp_path / "build"
    r = cmake_configure(build, f"-DMARV_VEHICLE_CARD={bad}")
    assert r.returncode == 0, r.stderr + r.stdout
    r = cmake_build(build)
    assert r.returncode != 0
    assert "rotors.m3.spin" in r.stdout + r.stderr
    assert not (build / "generated" / "marv_params" / "marv" / "params" / "param_ids.hpp").exists()
