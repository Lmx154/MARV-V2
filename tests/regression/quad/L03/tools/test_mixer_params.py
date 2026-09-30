"""L3 mixer parameters from the vehicle card (tools/card/mixer.py, flatten.py --out-mixer; decision 0004 items 1, 2, 5).

The oracles are the card read through schema.py and the plant configuration read through gen_plant_config.py; the
generated parameter set is read back from params_gen's param_defaults.cpp.
"""

import re
import struct
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
import gen_plant_config as gpc  # noqa: E402
import mixer  # noqa: E402
import schema  # noqa: E402

FLATTEN = ROOT / "tools" / "card" / "flatten.py"
GEN = ROOT / "tools" / "gen" / "params_gen.py"
CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
BUDGET = ROOT / "design" / "budget.yaml"

MOTORS = ("m1", "m2", "m3", "m4")
AXES = ("thrust", "roll", "pitch", "yaw")
MIXER_NAMES = ["idle_speed"] + [f"mixer_m{i}_{a}" for i in (1, 2, 3, 4) for a in AXES]
N = len(MOTORS)
EPS32 = 2.0 ** -(24 - 1)  # float32 epsilon: 24 significand bits
EPS64 = sys.float_info.epsilon

# The four rotor positions of the committed card, as written in it (one line each, unique in the file).
POS = {"m1": "[0.075, 0.10, 0.0]", "m2": "[-0.075, -0.10, 0.0]", "m3": "[0.075, -0.10, 0.0]",
       "m4": "[-0.075, 0.10, 0.0]"}
SPEED = "value: [150, 2800]"


def r32(x):
    return struct.unpack("<f", struct.pack("<f", x))[0]


def gamma(n, eps):
    return n * eps / (1 - n * eps)


def run(*args):
    return subprocess.run([sys.executable, *map(str, args)], capture_output=True, text=True, check=False)


def flatten(tmp, card, name="flat"):
    d = tmp / name
    r = run(FLATTEN, "--card", card, "--budget", BUDGET, "--out-card", d / "card.yaml",
            "--out-register", d / "register.yaml", "--out-mixer", d / "mixer.yaml", "--root", ROOT)
    return r, d


def generate(tmp, d, name="gen"):
    out = tmp / name
    r = run(GEN, "--card", d / "card.yaml", "--card", d / "mixer.yaml", "--register", d / "register.yaml",
            "--out-dir", out)
    assert r.returncode == 0, r.stderr
    return out


DEFAULT_RE = re.compile(r"    // (\w+)\n    \{\{ParamType::F32, ([-+0-9.e]+)f?, ")


def f32_defaults(gen):
    return {n: float(v) for n, v in DEFAULT_RE.findall((gen / "param_defaults.cpp").read_text())}


def craft(tmp, name, edits):
    """A copy of the committed card with each (old, new) text replaced; every old text must occur exactly once."""
    text = CARD.read_text(encoding="utf-8")
    for old, new in edits:
        assert text.count(old) == 1, old
        text = text.replace(old, new)
    path = tmp / f"{name}.yaml"
    path.write_text(text, encoding="utf-8")
    return path


@pytest.fixture(scope="module")
def real(tmp_path_factory):
    tmp = tmp_path_factory.mktemp("real")
    r, d = flatten(tmp, CARD)
    assert r.returncode == 0, r.stderr
    gen = generate(tmp, d)
    card = schema.load_yaml(CARD)
    return {"tmp": tmp, "dir": d, "gen": gen, "values": f32_defaults(gen), "card": card,
            "entries": dict(mixer.mixer_entries(card, CARD))}


def card_b(card):
    r = card["rotors"]
    return mixer.effectiveness([[float(v) for v in r[m]["position"]["value"]] for m in MOTORS],
                               [gpc.SPIN_SIGN[r[m]["spin"]["value"]] for m in MOTORS],
                               float(r["torque_ratio"]["value"]))


# (a) idle_speed


def test_idle_speed_is_the_card_speed_min_and_the_plant_omega_min(real):
    card_min = real["card"]["rotors"]["speed_range"]["value"][0]
    omega_min = gpc.plant_config(CARD)["omega_min_rad_s"]
    assert real["values"]["idle_speed"] == card_min == omega_min == 150.0
    e = real["entries"]["idle_speed"]
    assert e["type"] == "f32" and e["unit"] == "rad/s" and e["value"] == card_min
    assert e["method"].startswith("derived(") and "speed_range.min" in e["method"] and "rotor_speed_min" in e["method"]
    assert "rotors.speed_range" in e["source"] and "QF-6" in e["source"] and "replace" in e["source"]
    assert e["sigma"] == schema.UNKNOWN


def test_generated_idle_speed_is_a_derived_unknown_sigma_parameter(real):
    p = re.search(r"    // idle_speed\n(.*?)\n    // ", (real["gen"] / "param_defaults.cpp").read_text(), flags=re.S)
    assert "ParamMethod::Derived" in p.group(1) and "SigmaKind::Unknown" in p.group(1)
    assert '"rad/s"' in p.group(1)


# (b) M B = I


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(N)) for j in range(N)] for i in range(N)]


def abs_matmul(a, b):
    return matmul([[abs(x) for x in r] for r in a], [[abs(x) for x in r] for r in b])


def test_m_times_b_is_identity_in_double_to_the_derived_bound(tmp_path):
    """Rule: |M B - I|_ij <= gamma_2n(eps64) (|M||B|)_ij, n = 4. Gauss-Jordan (n-term row updates, one backward stable
    solve) plus the checking product (n-term dot products) each contribute gamma_n (Higham §3.5, §9, §14)."""
    cards = [CARD]
    cards.append(craft(tmp_path, "asym", [(POS["m1"], "[0.2, 0.10, 0.0]")]))
    cards.append(craft(tmp_path, "asym2", [(POS["m4"], "[-0.09, 0.13, 0.0]")]))
    for path in cards:
        card = schema.load_yaml(path)
        b = card_b(card)
        m, _, _ = mixer.mixer_matrix(card, path)
        prod, mag = matmul(m, b), abs_matmul(m, b)
        for i in range(N):
            for j in range(N):
                assert abs(prod[i][j] - (1.0 if i == j else 0.0)) <= gamma(2 * N, EPS64) * mag[i][j], (path, i, j)
    perturbed = [list(r) for r in m]
    perturbed[1][2] *= 1 + 8 * gamma(2 * N, EPS64) * N
    prod = matmul(perturbed, b)
    assert any(abs(prod[i][j] - (1.0 if i == j else 0.0)) > gamma(2 * N, EPS64) * mag[i][j]
               for i in range(N) for j in range(N))


def test_every_m_entry_round_trips_into_the_generated_parameter_set(real):
    m, _, _ = mixer.mixer_matrix(real["card"], CARD)
    for i in range(N):
        for k, axis in enumerate(AXES):
            name = f"mixer_m{i + 1}_{axis}"
            assert r32(real["values"][name]) == r32(m[i][k]), name
            assert real["entries"][name]["value"] == m[i][k]
            assert real["entries"][name]["unit"] == ("1" if axis == "thrust" else "1/m")
            assert real["entries"][name]["sigma"] == schema.UNKNOWN
            assert real["entries"][name]["method"].startswith("derived(")
    assert [n for n in real["values"] if n.startswith("mixer_") or n == "idle_speed"] == MIXER_NAMES


def test_float_mixer_times_float_b_is_identity_to_the_decision_0004_bound(real):
    """Decision 0004 item 2: |fl(M^ B^) - I| <= (gamma_n + eps)(|M||B|) per element, hats the float values, n = 4,
    eps the float32 epsilon; fl is a float32 dot product accumulated in order."""
    mh = [[real["values"][f"mixer_m{i + 1}_{a}"] for a in AXES] for i in range(N)]
    bh = [[r32(x) for x in row] for row in card_b(real["card"])]

    def fl_product(a, b):
        out = [[0.0] * N for _ in range(N)]
        for i in range(N):
            for j in range(N):
                acc = 0.0
                for k in range(N):
                    acc = r32(acc + r32(a[i][k] * b[k][j]))
                out[i][j] = acc
        return out

    mag = abs_matmul(mh, bh)
    bound = [[(gamma(N, EPS32) + EPS32) * mag[i][j] for j in range(N)] for i in range(N)]
    prod = fl_product(mh, bh)
    for i in range(N):
        for j in range(N):
            assert abs(prod[i][j] - (1.0 if i == j else 0.0)) <= bound[i][j], (i, j)
    bad = [list(r) for r in mh]
    bad[0][0] += 4 * bound[0][0]
    prod = fl_product(bad, bh)
    assert abs(prod[0][0] - 1.0) > bound[0][0]


# (c) sign sanity


def test_mixer_signs(real):
    v = real["values"]
    col = lambda axis: [v[f"mixer_m{i}_{axis}"] for i in (1, 2, 3, 4)]  # noqa: E731
    assert all(x > 0 for x in col("thrust"))
    assert [x > 0 for x in col("roll")] == [False, True, True, False]
    assert [x > 0 for x in col("pitch")] == [True, False, True, False]
    ccw = [real["card"]["rotors"][m]["spin"]["value"] == "ccw" for m in MOTORS]
    assert ccw == [True, True, False, False]
    assert [x > 0 for x in col("yaw")] == ccw


def test_signs_follow_the_forward_map(real):
    """Torque about x_FRD is -y f, so a positive roll request raises the motors at y < 0; torque about y_FRD is x f,
    so a positive pitch request raises those at x > 0."""
    r = real["card"]["rotors"]
    for i, m in enumerate(MOTORS, start=1):
        x, y, _ = r[m]["position"]["value"]
        assert (real["values"][f"mixer_m{i}_roll"] > 0) == (y < 0)
        assert (real["values"][f"mixer_m{i}_pitch"] > 0) == (x > 0)


# (d) refusals, each with a control that differs in one thing only


def refused(tmp, path, needle):
    r, d = flatten(tmp, path, name=f"out_{path.stem}")
    assert r.returncode == 1, r.stdout + r.stderr
    assert needle in r.stderr, r.stderr
    assert not (d / "mixer.yaml").exists() and not (d / "card.yaml").exists()
    with pytest.raises(gpc.GenError) as e:
        mixer.mixer_entries(schema.load_yaml(path), path)
    assert needle in str(e.value)


def accepted(tmp, path):
    r, d = flatten(tmp, path, name=f"ok_{path.stem}")
    assert r.returncode == 0, r.stderr
    assert (d / "mixer.yaml").exists()


def test_singular_b_is_refused(tmp_path):
    """The exact zero pivot (c_q = 0) and a pivot below n eps max|B| (c_q = 1e-20) are refused; c_q = 0.022 is not."""
    for name, cq in (("cq_zero", "0.0"), ("cq_tiny", "1.0e-20")):
        refused(tmp_path, craft(tmp_path, name, [("value: 0.022", f"value: {cq}")]), "singular")
    accepted(tmp_path, craft(tmp_path, "cq_small", [("value: 0.022", "value: 0.0001")]))


NEG_THRUST = [(POS["m1"], "[0.08, 0.28, 0.0]"), (POS["m2"], "[-0.27, -0.02, 0.0]"),
              (POS["m3"], "[0.02, -0.17, 0.0]"), (POS["m4"], "[-0.28, 0.12, 0.0]")]


def test_nonpositive_thrust_entry_is_refused(tmp_path):
    refused(tmp_path, craft(tmp_path, "neg_thrust", NEG_THRUST), "is not positive")
    accepted(tmp_path, craft(tmp_path, "pos_thrust", [(POS["m1"], "[0.1, 0.1, 0.0]")] + NEG_THRUST[1:]))


def test_zero_torque_infeasible_is_refused(tmp_path):
    edits = [(POS["m1"], "[0.5, 0.10, 0.0]")]
    refused(tmp_path, craft(tmp_path, "infeasible", edits + [(SPEED, "value: [2000, 2800]")]), "zero torque")
    accepted(tmp_path, craft(tmp_path, "feasible", edits))


# (e) the real card


def test_real_card_is_not_refused(real):
    m, _, idle = mixer.mixer_matrix(real["card"], CARD)
    assert idle == 150.0 and all(row[0] > 0 for row in m)
    text = (real["dir"] / "mixer.yaml").read_text(encoding="utf-8")
    assert re.findall(r"^(\w+):", text, flags=re.M) == MIXER_NAMES


def test_flatten_without_out_mixer_writes_the_two_l1_files_only(tmp_path):
    d = tmp_path / "l1"
    r = run(FLATTEN, "--card", CARD, "--budget", BUDGET, "--out-card", d / "card.yaml",
            "--out-register", d / "register.yaml", "--root", ROOT)
    assert r.returncode == 0, r.stderr
    assert sorted(p.name for p in d.iterdir()) == ["card.yaml", "register.yaml"]
