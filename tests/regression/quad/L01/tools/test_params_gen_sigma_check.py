"""Build-time sigma check of the parameter generator (docs/decisions/0001, invariant I1).

The generator validates every final (sigma kind, sigma) record after the token to kind mapping. These tests inject a
fault into that mapping in the loaded module and require the generator to refuse and write nothing; the unpatched
generator must accept the same input.
"""

import importlib.util
import re
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
GEN = ROOT / "tools" / "gen" / "params_gen.py"

SOURCE = (
    'alpha_unknown: {type: f32, value: 0.75, unit: kg, method: measured, source: "test data", sigma: UNKNOWN}\n'
    'beta_known: {type: f32, value: 0.75, unit: kg, method: measured, source: "test data", sigma: 0.5}\n'
    'gamma_exact: {type: i32, value: 4, unit: "1", method: published, source: "test data", sigma: exact}\n'
)


@pytest.fixture
def gen(monkeypatch):
    spec = importlib.util.spec_from_file_location("params_gen_under_test", GEN)
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, module)
    spec.loader.exec_module(module)
    return module


def run(gen, tmp_path):
    src = tmp_path / "src.yaml"
    src.write_text(SOURCE)
    return gen.main(["--card", str(src), "--out-dir", str(tmp_path / "out")])


def fault(gen, monkeypatch, wrong):
    """Replaces the token to kind mapping so that the entries in `wrong` (sigma text -> kind) get that kind."""
    real = gen._parse_sigma

    def faulty(sigma, ptype, method_key, what, tokens):
        kind, value = real(sigma, ptype, method_key, what, tokens)
        return wrong.get(sigma, kind), value

    monkeypatch.setattr(gen, "_parse_sigma", faulty)


def test_unpatched_generator_accepts_the_same_input(gen, tmp_path):
    assert run(gen, tmp_path) == 0
    text = (tmp_path / "out" / "param_defaults.cpp").read_text()
    assert "static_assert(sigma_table_consistent(kParamDefaults)," in text
    assert re.search(r"\n    // alpha_unknown\n.*\n     SigmaKind::Unknown,", text)


def test_known_kind_with_sigma_zero_is_refused(gen, tmp_path, monkeypatch, capsys):
    fault(gen, monkeypatch, {"UNKNOWN": "Known"})
    assert run(gen, tmp_path) == 1
    err = capsys.readouterr().err
    assert "alpha_unknown" in err and "Known" in err and "invariant I1" in err
    assert "beta_known" not in err and "gamma_exact" not in err
    assert not (tmp_path / "out").exists(), "a refused run must write nothing"


def test_unknown_kind_with_sigma_above_zero_is_refused(gen, tmp_path, monkeypatch, capsys):
    fault(gen, monkeypatch, {0.5: "Unknown"})
    assert run(gen, tmp_path) == 1
    err = capsys.readouterr().err
    assert "beta_known" in err and "Unknown" in err and "invariant I1" in err
    assert "alpha_unknown" not in err and "gamma_exact" not in err
    assert not (tmp_path / "out").exists(), "a refused run must write nothing"


def test_every_mismatch_is_named_in_one_run(gen, tmp_path, monkeypatch, capsys):
    fault(gen, monkeypatch, {"UNKNOWN": "Known", 0.5: "Choice", "exact": "Bogus"})
    assert run(gen, tmp_path) == 1
    err = capsys.readouterr().err
    assert "alpha_unknown" in err and "beta_known" in err and "gamma_exact" in err
    assert not (tmp_path / "out").exists()


@pytest.mark.parametrize(
    ("kind", "sigma", "ok"),
    [
        ("Known", 0.5, True),
        ("Known", 0.0, False),
        ("Known", float("inf"), False),
        ("Known", float("nan"), False),
        ("Known", -0.5, False),
        ("Exact", 0.0, True),
        ("Unknown", 0.0, True),
        ("Choice", 0.0, True),
        ("Exact", -0.0, False),
        ("Unknown", 0.5, False),
        ("Choice", float("nan"), False),
        ("Bogus", 0.0, False),
    ],
)
def test_python_mirror_of_i1(gen, kind, sigma, ok):
    assert gen.sigma_consistent(kind, sigma) is ok
