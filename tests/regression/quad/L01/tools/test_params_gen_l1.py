"""Quad L1 parameter generator rules: sigma kinds, lock, vector shapes and params_provenance.json.

The generator is run as a subprocess, like the L0 generator tests, so this file stands alone.
"""

import hashlib
import json
import re
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
GEN = ROOT / "tools" / "gen" / "params_gen.py"
L0_FIXTURES = ROOT / "tests" / "regression" / "quad" / "L00" / "fixtures"
L1_FIXTURES = ROOT / "tests" / "regression" / "quad" / "L01" / "fixtures"

LOCK = "{by: luis, on: 2026-09-29, via: manual}"


def entry(name, **fields):
    """One YAML entry; values are raw YAML text, None drops the field."""
    base = {"type": "f32", "value": "0.75", "unit": "kg", "method": "measured", "source": '"test data"',
            "sigma": "0.01"}
    base.update(fields)
    body = ", ".join(f"{k}: {v}" for k, v in base.items() if v is not None)
    return f"{name}: {{{body}}}\n"


def run_gen(tmp_path, *sources, out="out"):
    args = [sys.executable, str(GEN)]
    for kind, path in sources:
        args += [f"--{kind}", str(path)]
    args += ["--out-dir", str(tmp_path / out)]
    return subprocess.run(args, capture_output=True, text=True, check=False)


def write(tmp_path, text, name="src.yaml"):
    p = tmp_path / name
    p.write_text(text)
    return p


def generate(tmp_path, text, out="out"):
    r = run_gen(tmp_path, ("card", write(tmp_path, text, f"{out}.yaml")), out=out)
    assert r.returncode == 0, r.stderr
    d = tmp_path / out
    return {f.name: f.read_text() for f in d.iterdir()}


def refused(tmp_path, text, *needles):
    r = run_gen(tmp_path, ("card", write(tmp_path, text)))
    assert r.returncode == 1, r.stdout + r.stderr
    for needle in needles:
        assert needle in r.stderr, r.stderr
    assert not (tmp_path / "out").exists(), "a refused run must write nothing"


def records(defaults_cpp):
    """(name, sigma literal, locked, SigmaKind) for every record of param_defaults.cpp, in order."""
    names = re.findall(r"^    // (\w+)$", defaults_cpp, re.M)
    rows = re.findall(
        r"\{\{ParamType::\w+, [^}]*\}, (\S+), ParamOrigin::\w+, ParamMethod::\w+, (true|false),\s+SigmaKind::(\w+),",
        defaults_cpp)
    assert len(names) == len(rows)
    return [(n, s, lk == "true", k) for n, (s, lk, k) in zip(names, rows, strict=True)]


# --- sigma kind mapping -------------------------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("fields", "kind"),
    [
        ({"sigma": "0.01"}, "Known"),
        ({"method": "scenario", "sigma": "0"}, "Exact"),
        ({"method": "scenario", "sigma": "0.0"}, "Exact"),
        ({"type": "i32", "value": "4", "sigma": "exact"}, "Exact"),
        ({"sigma": "UNKNOWN"}, "Unknown"),
        ({"method": "design-budget", "sigma": "choice"}, "Choice"),
        ({"method": "scenario", "sigma": "choice"}, "Choice"),
        ({"method": "scenario", "sigma": "0.5"}, "Known"),
        ({"method": "design-budget", "sigma": "0"}, "Exact"),
        ({"method": '"derived(2 * m)"', "sigma": "0"}, "Exact"),
        ({"method": '"derived(2 * m)"', "sigma": "UNKNOWN"}, "Unknown"),
    ],
)
def test_sigma_maps_to_kind_and_zero_sigma_is_positive_zero(tmp_path, fields, kind):
    out = generate(tmp_path, entry("p", **fields))
    ((_, sigma, locked, got),) = records(out["param_defaults.cpp"])
    assert got == kind
    assert locked is False
    assert (sigma == "0.0f") == (kind != "Known"), sigma


def test_negative_zero_sigma_is_emitted_as_positive_zero(tmp_path):
    out = generate(tmp_path, entry("p", method="scenario", sigma="-0.0"))
    ((_, sigma, _, kind),) = records(out["param_defaults.cpp"])
    assert (sigma, kind) == ("0.0f", "Exact")


# --- sigma policy refusals ----------------------------------------------------------------------------------------


@pytest.mark.parametrize("method", ["published", "measured", "identified", "datasheet"])
def test_zero_sigma_is_refused_for_measured_kinds(tmp_path, method):
    refused(tmp_path, entry("p", method=method, sigma="0"), "p", "field 'sigma'", "exact", method)


def test_zero_sigma_control_the_same_entry_passes_as_scenario(tmp_path):
    generate(tmp_path, entry("p", method="scenario", sigma="0"))


@pytest.mark.parametrize("method", ["published", "measured", "identified", "datasheet", "derived(r)"])
def test_choice_is_refused_unless_design_budget_or_scenario(tmp_path, method):
    refused(tmp_path, entry("p", method=json.dumps(method), sigma="choice"), "p", "field 'sigma'", "choice")


@pytest.mark.parametrize("method", ["design-budget", "scenario"])
def test_unknown_is_refused_for_design_budget_and_scenario(tmp_path, method):
    refused(tmp_path, entry("p", method=method, sigma="UNKNOWN"), "p", "field 'sigma'", "UNKNOWN", method)


def test_unknown_control_the_same_entry_passes_as_measured(tmp_path):
    generate(tmp_path, entry("p", method="measured", sigma="UNKNOWN"))


@pytest.mark.parametrize("token", ["unknown", "Choice", "CHOICE", "Exact", '"0.1"', '""', "null", "[0.1]"])
def test_other_sigma_strings_are_refused(tmp_path, token):
    refused(tmp_path, entry("p", method="scenario", sigma=token), "p", "field 'sigma'")


# --- the exact token (integer counts) -----------------------------------------------------------------------------


def test_exact_token_is_refused_for_f32(tmp_path):
    refused(tmp_path, entry("p", method="datasheet", sigma="exact"), "p", "field 'sigma'", "exact", "i32")


@pytest.mark.parametrize("method", ["design-budget", "scenario"])
def test_exact_token_is_refused_for_design_budget_and_scenario(tmp_path, method):
    refused(tmp_path, entry("p", type="i32", value="4", method=method, sigma="exact"), "p", "field 'sigma'", "exact",
            method)


@pytest.mark.parametrize("method", ["published", "measured", "identified", "datasheet", '"derived(r)"'])
def test_exact_token_is_accepted_for_i32_with_every_other_method(tmp_path, method):
    out = generate(tmp_path, entry("poles", type="i32", value="14", unit='"1"', method=method, sigma="exact"))
    ((_, sigma, _, kind),) = records(out["param_defaults.cpp"])
    assert (sigma, kind) == ("0.0f", "Exact")


def test_exact_token_for_a_whole_i32_vector(tmp_path):
    out = generate(tmp_path, entry("n", type="i32", value="[1, 2]", shape="range", unit='"1"', method="datasheet",
                                   sigma="exact"))
    assert [k for *_, k in records(out["param_defaults.cpp"])] == ["Exact", "Exact"]


def test_exact_token_for_an_f32_vector_is_refused(tmp_path):
    refused(tmp_path, entry("n", value="[1.0, 2.0]", shape="range", method="datasheet", sigma="exact"), "n",
            "field 'sigma'", "i32")


# --- lock ---------------------------------------------------------------------------------------------------------


def test_lock_sets_locked_and_provenance_records_it(tmp_path):
    out = generate(tmp_path, entry("a", lock=LOCK) + entry("b"))
    assert [(n, lk) for n, _, lk, _ in records(out["param_defaults.cpp"])] == [("a", True), ("b", False)]
    prov = json.loads(out["params_provenance.json"])
    lock_of = {e["name"]: e["lock"] for e in prov["entries"]}
    assert lock_of == {"a": {"by": "luis", "on": "2026-09-29", "via": "manual"}, "b": None}


def test_lock_date_may_be_a_quoted_string(tmp_path):
    out = generate(tmp_path, entry("a", lock='{by: "L. M.", on: "2026-02-28", via: manual}'))
    prov = json.loads(out["params_provenance.json"])
    assert prov["entries"][0]["lock"] == {"by": "L. M.", "on": "2026-02-28", "via": "manual"}


@pytest.mark.parametrize(
    "lock",
    [
        "{on: 2026-09-29, via: manual}",
        "{by: luis, via: manual}",
        "{by: luis, on: 2026-09-29}",
        '{by: "", on: 2026-09-29, via: manual}',
        '{by: "  ", on: 2026-09-29, via: manual}',
        "{by: 7, on: 2026-09-29, via: manual}",
        "{by: luis, on: 2026-09-29, via: auto}",
        "{by: luis, on: 2026-09-29, via: Manual}",
        "{by: luis, on: 2026-13-45, via: manual}",
        '{by: luis, on: "2026-02-30", via: manual}',
        '{by: luis, on: "29/09/2026", via: manual}',
        '{by: luis, on: "2026-9-29", via: manual}',
        "{by: luis, on: 20260929, via: manual}",
        "{by: luis, on: 2026-09-29T10:00:00, via: manual}",
        "{by: luis, on: true, via: manual}",
        "{by: luis, on: 2026-09-29, via: manual, why: because}",
        "null",
        "true",
        "luis",
        "[luis, 2026-09-29, manual]",
        "{}",
    ],
)
def test_malformed_lock_is_refused(tmp_path, lock):
    refused(tmp_path, entry("a", lock=lock), "a", "field 'lock'")


def test_lock_control_the_well_formed_lock_passes(tmp_path):
    generate(tmp_path, entry("a", lock=LOCK))


def test_lock_covers_every_component_of_a_vector(tmp_path):
    out = generate(tmp_path, entry("v", value="[1.0, 2.0, 3.0]", shape="frd3", sigma="UNKNOWN", lock=LOCK))
    assert [lk for _, _, lk, _ in records(out["param_defaults.cpp"])] == [True, True, True]
    prov = json.loads(out["params_provenance.json"])
    assert len(prov["entries"]) == 1 and prov["entries"][0]["lock"]["by"] == "luis"


# --- shapes -------------------------------------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("shape", "values", "names"),
    [
        ("frd3", "[1.0, 2.0, 3.0]", ["v_x", "v_y", "v_z"]),
        ("diag3", "[1.0, 2.0, 3.0]", ["v_xx", "v_yy", "v_zz"]),
        ("range", "[1.0, 2.0]", ["v_min", "v_max"]),
        ("motors", "[1.0, 2.0, 3.0, 4.0]", ["v_m1", "v_m2", "v_m3", "v_m4"]),
        ("motors", "[1.0]", ["v_m1"]),
        ("motors", "[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]", ["v_m1", "v_m2", "v_m3", "v_m4", "v_m5", "v_m6"]),
    ],
)
def test_shape_expands_to_named_scalar_records(tmp_path, shape, values, names):
    out = generate(tmp_path, entry("v", value=values, shape=shape, sigma="UNKNOWN"))
    assert [n for n, *_ in records(out["param_defaults.cpp"])] == names
    assert list(json.loads(out["params_manifest.json"])["params"]) == names
    prov = json.loads(out["params_provenance.json"])["entries"]
    assert len(prov) == 1 and prov[0]["name"] == "v" and prov[0]["shape"] == shape
    assert prov[0]["components"] == names and prov[0]["sigma_kind"] == ["Unknown"] * len(names)


def test_expanded_vector_is_identical_to_hand_written_scalars(tmp_path):
    vec = generate(tmp_path, entry("v", value="[1.5, -2.5, 0.0]", shape="frd3", sigma="[0.1, UNKNOWN, 0.3]"),
                   out="vec")
    scalars = generate(tmp_path,
                       entry("v_x", value="1.5", sigma="0.1") + entry("v_y", value="-2.5", sigma="UNKNOWN") +
                       entry("v_z", value="0.0", sigma="0.3"), out="sca")
    for name in ("param_ids.hpp", "param_defaults.cpp", "params_manifest.json"):
        assert vec[name] == scalars[name], name


def test_schema_hash_runs_over_the_expanded_scalars(tmp_path):
    out = generate(tmp_path, entry("m", value="[1.0, 2.0]", shape="motors", unit="s", sigma="UNKNOWN"))
    h = hashlib.sha256(b"m_m1\x1ff32\x1fs\x1em_m2\x1ff32\x1fs\x1e").digest()
    assert json.loads(out["params_manifest.json"])["schema_hash"] == f"0x{int.from_bytes(h[:8], 'big'):016x}"


def test_per_component_sigma_kinds_and_values(tmp_path):
    out = generate(tmp_path, entry("v", value="[1.0, 2.0, 3.0]", shape="diag3", sigma="[0.5, UNKNOWN, 0.25]"))
    assert [(s, k) for _, s, _, k in records(out["param_defaults.cpp"])] == [
        ("0.5f", "Known"), ("0.0f", "Unknown"), ("0.25f", "Known")]
    assert json.loads(out["params_provenance.json"])["entries"][0]["sigma_kind"] == ["Known", "Unknown", "Known"]


def test_single_token_sigma_applies_to_every_component(tmp_path):
    out = generate(tmp_path, entry("v", value="[1.0, 2.0]", shape="range", method="design-budget", sigma="choice"))
    assert [k for *_, k in records(out["param_defaults.cpp"])] == ["Choice", "Choice"]


def test_integer_vector(tmp_path):
    out = generate(tmp_path, entry("n", type="i32", value="[1, 2]", shape="range", unit='"1"', sigma="UNKNOWN"),
                   out="ok")
    assert out["param_defaults.cpp"].count("ParamType::I32") == 2
    refused(tmp_path, entry("n", type="i32", value="[1, 2.5]", shape="range", sigma="UNKNOWN"), "n", "field 'value'")


@pytest.mark.parametrize(
    ("fields", "needle"),
    [
        ({"shape": "frd3", "value": "[1.0, 2.0]", "sigma": "UNKNOWN"}, "field 'value'"),
        ({"shape": "frd3", "value": "[1.0, 2.0, 3.0, 4.0]", "sigma": "UNKNOWN"}, "field 'value'"),
        ({"shape": "diag3", "value": "[1.0]", "sigma": "UNKNOWN"}, "field 'value'"),
        ({"shape": "range", "value": "[1.0, 2.0, 3.0]", "sigma": "UNKNOWN"}, "field 'value'"),
        ({"shape": "motors", "value": "[]", "sigma": "UNKNOWN"}, "field 'value'"),
        ({"shape": "frd3", "value": "1.0", "sigma": "UNKNOWN"}, "field 'value'"),
        ({"shape": "quat", "value": "[1.0, 2.0]", "sigma": "UNKNOWN"}, "field 'shape'"),
        ({"shape": "null", "value": "[1.0, 2.0]", "sigma": "UNKNOWN"}, "field 'shape'"),
        ({"shape": "frd3", "value": "[1.0, 2.0, 3.0]", "sigma": "0.1"}, "field 'sigma'"),
        ({"shape": "range", "value": "[1.0, 2.0]", "sigma": "0"}, "field 'sigma'"),
        ({"shape": "frd3", "value": "[1.0, 2.0, 3.0]", "sigma": "[0.1, 0.2]"}, "field 'sigma'"),
        ({"shape": "frd3", "value": "[1.0, 2.0, 3.0]", "sigma": "[0.1, 0.2, 0.3, 0.4]"}, "field 'sigma'"),
        ({"shape": "motors", "value": "[1.0, 2.0]", "sigma": "[0.1]"}, "field 'sigma'"),
        ({"shape": "range", "value": "[1.0, 2.0]", "sigma": "[choice, choice]", "method": "scenario"}, "field 'sigma'"),
        ({"shape": "range", "value": "[1.0, 2.0]", "sigma": "[0.1, -0.2]"}, "field 'sigma'"),
        ({"shape": "range", "value": "[1.0, 2.0]", "sigma": "[0.1, exact]"}, "field 'sigma'"),
        ({"shape": "range", "value": "[1.0, 2.0]", "sigma": "[0, 0.2]"}, "field 'sigma'"),
        ({"shape": "range", "value": "[1.0, 2.0]", "sigma": "[UNKNOWN, 0.2]", "method": "scenario"}, "field 'sigma'"),
        ({"shape": "range", "value": "[1.0, .nan]", "sigma": "UNKNOWN"}, "field 'value'"),
        ({"shape": "range", "value": '[1.0, "2.0"]', "sigma": "UNKNOWN"}, "field 'value'"),
    ],
)
def test_bad_vector_entry_is_refused(tmp_path, fields, needle):
    refused(tmp_path, entry("v", **fields), "v", needle)


def test_value_list_without_shape_is_refused(tmp_path):
    refused(tmp_path, entry("v", value="[1.0, 2.0]", sigma="UNKNOWN"), "v", "field 'value'")


def test_sigma_list_without_shape_is_refused(tmp_path):
    refused(tmp_path, entry("v", sigma="[0.1]"), "v", "field 'sigma'")


def test_duplicate_after_expansion_is_refused(tmp_path):
    text = entry("v", value="[1.0, 2.0, 3.0]", shape="frd3", sigma="UNKNOWN") + entry("v_y", sigma="UNKNOWN")
    refused(tmp_path, text, "v_y", "duplicate")


def test_duplicate_after_expansion_across_files_is_refused(tmp_path):
    a = write(tmp_path, entry("v", value="[1.0, 2.0]", shape="range", sigma="UNKNOWN"), "a.yaml")
    b = write(tmp_path, entry("v_min", sigma="UNKNOWN"), "b.yaml")
    r = run_gen(tmp_path, ("card", a), ("register", b))
    assert r.returncode == 1
    assert "v_min" in r.stderr and "duplicate" in r.stderr and "a.yaml" in r.stderr
    assert not (tmp_path / "out").exists()


def test_two_vectors_that_expand_to_the_same_name_are_refused(tmp_path):
    text = entry("v", value="[1.0, 2.0]", shape="range", sigma="UNKNOWN") + \
        entry("v", value="[1.0, 2.0]", shape="range", sigma="UNKNOWN")
    refused(tmp_path, text, "duplicate key 'v'")


def test_expanded_name_must_be_a_valid_enumerator(tmp_path):
    refused(tmp_path, entry("v_", value="[1.0, 2.0]", shape="range", sigma="UNKNOWN"), "v_", "field 'name'")


# --- params_provenance.json and the unchanged outputs ---------------------------------------------------------------


def test_provenance_lists_every_source_entry_in_order(tmp_path):
    card = write(tmp_path, entry("s", sigma="UNKNOWN") +
                 entry("v", value="[1.0, 2.0, 3.0]", shape="frd3", sigma="[0.1, UNKNOWN, 0.3]", lock=LOCK),
                 "card.yaml")
    reg = write(tmp_path, entry("c", method="design-budget", sigma="choice"), "reg.yaml")
    r = run_gen(tmp_path, ("card", card), ("register", reg))
    assert r.returncode == 0, r.stderr
    prov = json.loads((tmp_path / "out" / "params_provenance.json").read_text())
    manifest = json.loads((tmp_path / "out" / "params_manifest.json").read_text())
    assert prov["schema_hash"] == manifest["schema_hash"]
    assert prov["entries"] == [
        {"name": "s", "source_file": "card.yaml", "shape": None, "components": ["s"], "sigma_kind": ["Unknown"],
         "lock": None},
        {"name": "v", "source_file": "card.yaml", "shape": "frd3", "components": ["v_x", "v_y", "v_z"],
         "sigma_kind": ["Known", "Unknown", "Known"], "lock": {"by": "luis", "on": "2026-09-29", "via": "manual"}},
        {"name": "c", "source_file": "reg.yaml", "shape": None, "components": ["c"], "sigma_kind": ["Choice"],
         "lock": None},
    ]


def test_manifest_entry_format_is_unchanged(tmp_path):
    out = generate(tmp_path, entry("v", value="[1.0, 2.0]", shape="range", unit="m", sigma="UNKNOWN", lock=LOCK))
    manifest = json.loads(out["params_manifest.json"])
    assert manifest["params"]["v_min"] == {"id": 0, "type": "f32", "unit": "m"}
    assert set(manifest) == {"schema_hash", "schema_hash_algorithm", "count", "params"}


def test_header_text_is_unchanged_by_kind_lock_and_shape_metadata(tmp_path):
    plain = generate(tmp_path, entry("p", sigma="0.01"), out="plain")
    rich = generate(tmp_path, entry("p", sigma="UNKNOWN", lock=LOCK), out="rich")
    assert plain["param_ids.hpp"] == rich["param_ids.hpp"]
    assert plain["params_manifest.json"] == rich["params_manifest.json"]


def test_l1_fixture_set_generates_and_covers_every_kind(tmp_path):
    r = run_gen(tmp_path, ("card", L1_FIXTURES / "l1_card.yaml"), ("register", L1_FIXTURES / "l1_register.yaml"))
    assert r.returncode == 0, r.stderr
    cpp = (tmp_path / "out" / "param_defaults.cpp").read_text()
    assert {k for *_, k in records(cpp)} == {"Known", "Exact", "Unknown", "Choice"}
    assert any(lk for _, _, lk, _ in records(cpp))


def test_l0_fixtures_still_generate_with_only_known_and_exact(tmp_path):
    r = run_gen(tmp_path, ("card", L0_FIXTURES / "l0_card.yaml"), ("register", L0_FIXTURES / "l0_register.yaml"))
    assert r.returncode == 0, r.stderr
    kinds = {k for *_, k in records((tmp_path / "out" / "param_defaults.cpp").read_text())}
    assert kinds == {"Known", "Exact"}
    assert (tmp_path / "out" / "params_provenance.json").exists()
