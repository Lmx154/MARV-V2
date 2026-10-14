"""The truth vertical feed of the L11 plugin (decision 0020 section 1, item c, 'Plugin' and 'Adapter and log').

Skipped only as conftest.py says. One short run of the l11_vertical_scripted composition on the host-gz-l11 build with
<attitude_source>truth</attitude_source> and <vertical_source>truth</vertical_source> (l11_gz.world): the hover thrust and
one height reference set by sil_override, the rotors at rest at the start, so the vehicle falls and z and v_z evolve.

Claims. The element is accepted and the run completes. Every tick has exactly one record 8 (VERTICAL), directly after its
TRUTH record (the reader's rule), whose tick is the tick, whose flags are the valid bit only, and whose z and v_z are, bit
for bit, the float casts of the logged plant body's pos_ned[2] and vel_ned[2] of the host step the tick belongs to (held
over the step's m ticks). At the ticks where the fed state is the reference (z = z_ref, v_z = 0: the first host step), every
motor's DShot is in the card's hover bracket [D_lo, D_hi] at the scenario site (tools/sim/hover.py): the vertical loop's
output there is exactly the hover feed-forward vert_hover_thrust_n.

Negative controls: (a) the expectation taken from the host step one earlier or one later fails; (b) the expectation left in
binary64 (no float cast) fails; (c) the same world with the hover thrust 2 % high (labelled control value, the scale of
tests/regression/quad/L02/tools/test_hover.py's wrong-mass control) puts the DShot at the reference outside the bracket.
"""

import struct
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import l11_gz  # noqa: E402
from l11_gz import M  # noqa: E402

import gen_plant_config as gpc  # noqa: E402
import hover  # noqa: E402
import l5_scenario as l5s  # noqa: E402
import lockstep_log  # noqa: E402
import run_scenario  # noqa: E402

HOVER_SCALE_CONTROL = 1.02


def bits(x):
    return struct.pack("<f", x)


def r32(x):
    return struct.unpack("<f", struct.pack("<f", x))[0]


def flown_log(tmp, hover_scale=1):
    log = tmp / "run.bin"
    r = l11_gz.run(l11_gz.world(tmp, log, hover_scale=hover_scale), tmp, l11_gz.PLUGIN_DIR_L11)
    assert r.returncode == 0 and "REFUSED" not in r.stderr, run_scenario.stderr_tail(r.stderr)
    d = lockstep_log.read(log)
    assert run_scenario.complete_trailer(d, l11_gz.ITERATIONS, M)
    return d


@pytest.fixture(scope="module")
def flown(tmp_path_factory):
    return flown_log(tmp_path_factory.mktemp("vertical"))


def mismatches(d, shift=0, cast=True):
    """Ticks whose VERTICAL record is not (bit for bit) the expected state of step tick // M + shift, or is missing, or
    has another tick or other flags."""
    bad = []
    for t in d["ticks"]:
        v = t.get("vertical")
        j = min(max(t["tick"] // M + shift, 0), len(d["steps"]) - 1)
        step = d["steps"][j]
        z, vz = step["body_pos_ned"][2], step["body_vel_ned"][2]
        if cast:
            ok = v is not None and bits(v["z_ned_m"]) == bits(z) and bits(v["vz_ned_m_s"]) == bits(vz)
        else:
            ok = v is not None and v["z_ned_m"] == z and v["vz_ned_m_s"] == vz
        if not (ok and v["tick"] == t["tick"] and v["flags"] == l11_gz.VALID):
            bad.append(t["tick"])
    return bad


def bracket():
    vals = l5s.values(l5s.load(l11_gz.SOURCE))
    _, _, d_lo, d_hi = hover.bracket_for_card(gpc.plant_config(l11_gz.CARD), vals["site_latitude_rad"],
                                              vals["site_height_m"])
    return d_lo, d_hi


def dshot_at_reference(d):
    z_ref = l11_gz.z_ref_m()
    return [t["dshot"] for t in d["ticks"] if t["vertical"]["z_ned_m"] == z_ref and t["vertical"]["vz_ned_m_s"] == 0]


def test_every_tick_has_one_vertical_record_with_the_float_cast_of_the_step_body_state(flown):
    assert len(flown["ticks"]) == len(flown["truths"]) == l11_gz.ITERATIONS * M
    assert all("vertical" in t for t in flown["ticks"])
    assert mismatches(flown) == []


def test_the_state_evolves_and_exercises_the_cast(flown):
    steps = flown["steps"]
    for k in ("body_pos_ned", "body_vel_ned"):
        values = [s[k][2] for s in steps]
        assert len(set(values)) > len(values) // 2, f"{k}[2] barely changes"
        assert any(r32(x) != x for x in values), f"every {k}[2] is already binary32"


def test_dshot_at_the_reference_is_in_the_hover_bracket(flown):
    lo, hi = bracket()
    at_ref = dshot_at_reference(flown)
    assert at_ref, "no tick was fed the reference state"
    assert all(lo <= x <= hi for row in at_ref for x in row), (at_ref, (lo, hi))


@pytest.mark.parametrize("shift", (-1, 1))
def test_control_expectation_from_another_step_fails(flown, shift):
    assert mismatches(flown, shift=shift) != []


def test_control_expectation_without_the_float_cast_fails(flown):
    assert len(mismatches(flown, cast=False)) > len(flown["ticks"]) // 2


def test_control_a_hover_thrust_off_by_two_percent_leaves_the_bracket(tmp_path):
    lo, hi = bracket()
    at_ref = dshot_at_reference(flown_log(tmp_path, HOVER_SCALE_CONTROL))
    assert at_ref and all(x > hi for row in at_ref for x in row), (at_ref, (lo, hi))
