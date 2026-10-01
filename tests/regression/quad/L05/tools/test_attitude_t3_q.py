"""L5 T3 quantisation term Q of the T4 tolerance (owner decision 19): the oracle's quantiser and its negative controls.

Q is the maximum over the time and the tau x J box of |quantised - unquantised| of the design-model trajectory, the quantiser
being the firmware's allocation and thrust_to_dshot rounding followed by marv_plant's ESC map (attitude_t3_oracle.py,
Quantiser). Checked here, on a short segment (the rule is the same at any length):
  1. control: with the quantiser disabled (identity map) Q = 0 exactly, for a tilt script and both yaw scripts;
  2. the quantiser enabled gives Q > 0 and finite on every channel (the positive control of 1);
  3. the quantiser's request dead bands and the hover DShot equal the values step_cause.py derived independently
     (tests/regression/quad/L05/results/step_cause/cause.txt: hover DShot 765.06 -> 765; dead bands 8.03e-4 roll, 6.02e-4 pitch N m);
  4. the committed attitude_t3_q.txt holds a finite positive q for every script and channel.
"""

import importlib.util
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "refdata"))
import refdata  # noqa: E402

REFERENCE = Path(__file__).resolve().parents[1] / "t3" / "reference"
T3_GENERATED = refdata.reference_dir("quad/L05/t3")  # the generated golden, envelope and q (decision 0011)
SHORT_SEGMENT = 1200  # attitude executions of the hold and of the release segment: long enough for every yaw member to lock (att_yaw_t_cross / T_a = 820 after the release at N = 1; 410 at N = 2)
DEAD_BAND_REL_TOL = 1e-3  # scenario test value: the two derivations agree to this relative difference (cause.txt prints 4 digits)
BISECTIONS = 80  # scenario test value: the dead-band search halves an interval of 1e-2 N m this many times


def _load():
    spec = importlib.util.spec_from_file_location("attitude_t3_oracle", REFERENCE / "attitude_t3_oracle.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


ORACLE = _load()


@pytest.fixture(scope="module")
def setup():
    p = ORACLE.read_inputs(REFERENCE / "attitude_t3_inputs.txt")
    q = ORACLE.read_q_inputs(REFERENCE / "attitude_t3_q_inputs.txt")
    return ORACLE.Setup(p), q


@pytest.mark.parametrize("name", ORACLE.Q_SCRIPTS)
def test_disabled_quantiser_gives_zero_q(setup, name):
    su, q = setup
    r = ORACLE.q_script(su, q, name, SHORT_SEGMENT, identity=True)
    for ch in ORACLE.Q_CHANNELS[name]:
        assert r[ch]["used"][0] == 0.0, (name, ch)
    assert r["saturated"] == 0


@pytest.mark.parametrize("name", ORACLE.Q_SCRIPTS)
def test_enabled_quantiser_gives_positive_finite_q(setup, name):
    su, q = setup
    r = ORACLE.q_script(su, q, name, SHORT_SEGMENT)
    for ch in ORACLE.Q_CHANNELS[name]:
        v = r[ch]["used"][0]
        assert math.isfinite(v) and v > 0.0, (name, ch, v)
        assert v == max(r[ch]["corners"][0], r[ch]["grid"][0])


def _dead_band(qz):
    lo, hi = 0.0, 1e-2
    for _ in range(BISECTIONS):
        mid = (lo + hi) / 2
        if qz(mid) != 0.0:
            hi = mid
        else:
            lo = mid
    return hi


def test_quantiser_reproduces_the_diagnosis_quantum(setup):
    _, q = setup
    facts = ORACLE.q_hover_facts(q)
    assert facts["hover_dshot_rounded"] == 765
    assert abs(facts["hover_dshot_real"] - 765.06) < 5e-3
    assert facts["zero_request_torque"] == [0.0, 0.0, 0.0]
    assert _dead_band(ORACLE.Quantiser(q, 0)) == pytest.approx(8.03e-4, rel=DEAD_BAND_REL_TOL)
    assert _dead_band(ORACLE.Quantiser(q, 1)) == pytest.approx(6.02e-4, rel=DEAD_BAND_REL_TOL)
    # Control: the identity map has no dead band (a request of one ulp passes through).
    assert ORACLE.Quantiser(q, 0, identity=True)(1e-12) == 1e-12


def test_committed_q_file_is_recorded_for_every_script_and_channel():
    rows = {}
    scenario = None
    for line in (T3_GENERATED / "attitude_t3_q.txt").read_text().splitlines():
        w = line.split()
        if not w or w[0].startswith("#"):
            continue
        if w[0] == "scenario":
            scenario = w[1]
        elif w[0] == "channel" and w[2] == "q":
            rows[(scenario, w[1])] = float(w[3])
    assert set(rows) == {(n, c) for n, chs in ORACLE.Q_CHANNELS.items() for c in chs}
    assert all(math.isfinite(v) and v > 0.0 for v in rows.values())
