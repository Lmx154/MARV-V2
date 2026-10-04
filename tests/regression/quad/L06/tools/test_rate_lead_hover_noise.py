"""Hover noise of the stage (c) rate loop (decision 0014 "Stage (c) close", pass bar (c) noise line; lands with decision 0016's
commit): "at hover omega = 0, so the FF path's noise contribution is zero to first order and the combined value equals the D
path's 0.494 mN". test_rate_lead.py asserts only the maximum over the operating points (noise_floor_max <= budget_n); this
file asserts the hover statement itself, on the model pieces test_rate_lead_checks.py builds without the N* search.

The inputs are the checks file's: the committed card's gains at its pinned N_STAR, OMEGA_C and T_FF (test_rate_lead.py
asserts design() returns them), the f32 per-axis kd and T_f, the firmware model design() evaluates the noise on (the 312/313
us stamp pattern, expf -LIBM_EXP_ULPS ulp: the largest alpha), the chain at the hover rotor speeds chain["omega_hover"].

The D path alone is the tool's own: rate_lead.Noise.rms returns "d" (|G_D|^2 P_i, no FF term) next to "combined" in the same
call. Equality is exact (bitwise): at w0 = 0 gyroscopic_jacobian is 0, so Q_i = R_i = 0 and the combined variance is
dd P_i + ff * 0 - 2 df * 0 = dd P_i + 0.0 - 0.0, which is dd P_i in double arithmetic (x + 0.0 and x - 0.0 are x, ff and df
finite), and the same per-phase maximum, scaled and square-rooted the same way as "d". So the tolerance of check 1 is 0.

Control (core 7.2): at the rate_max operating point (|w0_a| = rate_max_a, one sign pattern; the tool's own operating point) the
combined value is strictly greater than the D-path-only value, so the equality that holds at hover does not hold by construction.
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
import gen_plant_config as gpc  # noqa: E402
import rate_lead as rl  # noqa: E402
import test_rate_lead_checks as checks  # noqa: E402  (this directory; pytest's default import mode puts it on sys.path)
from test_rate_lead_checks import setup  # noqa: E402,F401  (the module fixture, imported so it is found here)

# The record's value: decision 0014 "Stage (c) close", noise line: D path at hover 0.494 mN, printed to 3 significant figures.
RECORDED_HOVER_MN = 0.494
MN = 1e3                                  # N -> mN (the record's unit)
RECORD_HALF_UNIT_MN = 0.0005              # half of the record's last printed digit (0.001 mN): its rounding interval


def _rms(setup, speeds, w0):
    """rate_lead.Noise.rms as design() evaluates it for the noise record (`at`): f32 kd and T_f, T_ff, firmware model fw_max."""
    axes32 = setup["axes32"]
    kd32, tf32 = [a[2] for a in axes32], axes32[0][3]
    fw = (setup["dts"], -rl.LIBM_EXP_ULPS)
    return setup["noise"].rms(speeds, w0, kd32, tf32, checks.T_FF, fw=fw)


def _hover(setup):
    return _rms(setup, [setup["chain"]["omega_hover"]] * rl.MOTORS, [0.0, 0.0, 0.0])


def _budget_n():
    card, budget, _, _ = rl.load(checks.CARD, checks.BUDGET, checks.SCENARIO, checks.PROFILE)
    rotors = card["rotors"]
    k = float(gpc._known(rotors["thrust_coeff"], "rotors.thrust_coeff", checks.CARD))
    omega_min, omega_max = (float(v) for v in gpc._known(rotors["speed_range"], "rotors.speed_range", checks.CARD))
    mass = float(gpc._known(card["mass"], "mass", checks.CARD))
    m_matrix = rl.mixer.mixer_matrix(card, checks.CARD)[0]
    unit, _, _ = rl.dshot_step(m_matrix, mass, k, omega_min, omega_max)
    return float(rl.rate._value(budget, "d_path_noise_budget", "budget entry", checks.CARD)) * unit


def test_hover_combined_noise_equals_the_d_path_alone_exactly(setup):
    hover = _hover(setup)
    assert len(hover["d"]) == len(hover["combined"]) == len(setup["m"]) > 0
    # Exact: see the module docstring (tolerance 0).
    assert hover["combined"] == hover["d"]
    assert all(x > 0 for x in hover["d"])     # a zero D path would make the equality vacuous
    assert all(x == 0.0 for x in hover["ff"])


def test_hover_value_is_within_the_budget_and_is_the_recorded_0_494_mn(setup):
    d_max = max(_hover(setup)["d"])
    budget_n = _budget_n()
    print(f"hover D path {d_max * MN!r} mN; budget {budget_n * MN!r} mN")
    assert d_max <= budget_n
    assert abs(d_max * MN - RECORDED_HOVER_MN) <= RECORD_HALF_UNIT_MN


def test_control_at_rate_max_the_combined_noise_exceeds_the_d_path(setup):
    rate_max = setup["rate_max"]
    nz = _rms(setup, None, list(rate_max))     # |w0_a| = rate_max_a, all signs +; the rate_max chain has every notch bypassed
    print(f"rate_max combined {max(nz['combined']) * MN!r} mN, D only {max(nz['d']) * MN!r} mN")
    assert nz["combined"] != nz["d"]           # the check-1 equality breaks (tolerance 0)
    assert max(nz["combined"]) > max(nz["d"])
