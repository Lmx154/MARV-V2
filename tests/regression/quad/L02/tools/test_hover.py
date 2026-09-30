"""Hover bracket and WGS 84 normal gravity (tools/sim/hover.py, tools/sim/prim_constants.py). L02 tests.

Gravity is compared with the L1 golden CSV (tests/regression/quad/L01/unit/prim/gravity_golden.csv, read only) under
the tolerance rule of tests/regression/quad/L01/unit/prim/gravity_test.cpp: relative tolerance kOps * epsilon(double)
with kOps = 40, plus the bound on the toolbox's e^2 (0.00669437999013) against TR8350.2 Table 3.3 (6.69437999014e-3),
delta_e2 / (2 (1 - e2)). The hover bracket is compared with an independent evaluation in decimal arithmetic (50
digits) from constants restated here from NIMA TR8350.2 3rd ed. Amdt 1 (2000), as the C++ test restates them.
"""

import decimal
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
sys.path.insert(0, str(ROOT / "tools" / "sim"))
import gen_plant_config as gpc  # noqa: E402
import hover  # noqa: E402
import prim_constants  # noqa: E402
import schema  # noqa: E402

GOLDEN = ROOT / "tests" / "regression" / "quad" / "L01" / "unit" / "prim" / "gravity_golden.csv"
CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
SCENARIO = ROOT / "scenarios" / "quad" / "L02" / "hover.yaml"
K_OPS = 40
EPS = 2.0 ** -52
TOOLBOX_E2 = 0.00669437999013
TABLE_E2 = 6.69437999014e-3
GOLDEN_ROWS = 13 * 6

# TR8350.2 restated as decimal strings, independent of constants.hpp.
D = decimal.Decimal
decimal.getcontext().prec = 50
REF = {
    "a": D("6378137.0"), "inv_f": D("298.257223563"), "e2": D("6.69437999014e-3"), "gamma_e": D("9.7803253359"),
    "k": D("0.00193185265241"), "m": D("0.00344978650684"),
}


def golden_rows():
    rows = []
    header = False
    for line in GOLDEN.read_text(encoding="utf-8").splitlines():
        if not line or line.startswith("#"):
            continue
        if not header:
            header = True
            continue
        c = [float(x) for x in line.split(",")]
        rows.append({"phi": c[1], "h": c[2], "gamma": c[3], "gamma_h": c[4]})
    return rows


def golden_tol():
    return K_OPS * EPS + abs(TABLE_E2 - TOOLBOX_E2) / (2.0 * (1.0 - TABLE_E2))


def within(got, want, rel):
    return abs(got - want) <= rel * abs(want)


def dec_gamma_h(phi, h):
    s = D(math.sin(phi))  # sin evaluated in double, then carried exactly: the input phi is a double
    s2 = s * s
    f = 1 / REF["inv_f"]
    gamma = REF["gamma_e"] * (1 + REF["k"] * s2) / (1 - REF["e2"] * s2).sqrt()
    return gamma * (1 - (2 / REF["a"]) * (1 + f + REF["m"] - 2 * f * s2) * D(h) + (3 / (REF["a"] * REF["a"])) * D(h) ** 2)


def test_constants_are_read_from_the_header():
    c = prim_constants.load()
    assert c["kDshotThrottleMin"] == 48 and c["kDshotThrottleMax"] == 2047
    assert c["kWgs84A"] == 6378137.0 and c["kWgs84InvF"] == 298.257223563
    assert c["kWgs84E2"] == 6.69437999014e-3 and c["kWgs84GammaE"] == 9.7803253359
    assert c["kWgs84K"] == 0.00193185265241 and c["kWgs84M"] == 0.00344978650684
    assert c["kWgs84HeightQuadCoeff"] == 3.0 and c["kWgs84F"] == 1.0 / 298.257223563


def test_golden_grid_is_complete():
    assert len(golden_rows()) == GOLDEN_ROWS


def test_gravity_matches_the_l1_golden_vectors():
    tol = golden_tol()
    for r in golden_rows():
        assert within(hover.normal_gravity_ellipsoid(r["phi"]), r["gamma"], tol), r
        assert within(hover.normal_gravity(r["phi"], r["h"]), r["gamma_h"], tol), r


def test_gravity_matches_an_independent_decimal_evaluation():
    for lat in range(-90, 91, 5):
        phi = lat * math.pi / 180.0
        for h in (0.0, 1.0, 100.0, 1000.0, 10000.0, 50000.0):
            want = float(dec_gamma_h(phi, h))
            assert within(hover.normal_gravity(phi, h), want, K_OPS * EPS), (lat, h)


def test_gravity_known_answers():
    assert within(hover.normal_gravity_ellipsoid(0.0), 9.7803253359, K_OPS * EPS)
    assert abs(hover.normal_gravity_ellipsoid(math.pi / 2) - 9.8321849378) <= 1e-10 + K_OPS * EPS * 9.83
    assert hover.normal_gravity(0.5, 0.0) == hover.normal_gravity_ellipsoid(0.5)


def test_negative_control_perturbed_gamma_e_breaks_the_golden_comparison(monkeypatch):
    tol = golden_tol()
    rows = golden_rows()
    monkeypatch.setitem(hover._C, "kWgs84GammaE", hover._C["kWgs84GammaE"] * (1.0 + 1e-9))
    assert all(not within(hover.normal_gravity_ellipsoid(r["phi"]), r["gamma"], tol) for r in rows)
    assert all(not within(hover.normal_gravity(r["phi"], r["h"]), r["gamma_h"], tol) for r in rows)


def test_negative_control_dropped_height_term_breaks_the_golden_comparison(monkeypatch):
    tol = golden_tol()
    rows = [r for r in golden_rows() if r["h"] >= 100.0]
    assert rows
    monkeypatch.setitem(hover._C, "kWgs84HeightQuadCoeff", 0.0)
    assert all(not within(hover.normal_gravity(r["phi"], r["h"]), r["gamma_h"], tol) for r in rows)


def card_and_site():
    cfg = gpc.plant_config(CARD)
    doc = schema.load_yaml(SCENARIO)
    return cfg, doc["site_latitude_rad"]["value"], doc["site_height_m"]["value"]


def dec_bracket(cfg, lat, h):
    g = dec_gamma_h(lat, h)
    omega_h = (D(cfg["mass_kg"]) * g / (4 * D(cfg["thrust_coeff"]))).sqrt()
    wmin, wmax = D(cfg["omega_min_rad_s"]), D(cfg["omega_max_rad_s"])
    d_star = 48 + (omega_h - wmin) * (2047 - 48) / (wmax - wmin)
    return omega_h, d_star, int(d_star.to_integral_value(rounding=decimal.ROUND_FLOOR))


def test_hover_bracket_for_the_committed_card():
    cfg, lat, h = card_and_site()
    omega_h, d_star, d_lo, d_hi = hover.bracket_for_card(cfg, lat, h)
    want_omega, want_star, want_lo = dec_bracket(cfg, lat, h)
    assert (d_lo, d_hi) == (want_lo, want_lo + 1) == (765, 766)
    assert abs(D(d_star) - want_star) <= D("1e-9") and abs(D(omega_h) - want_omega) <= D("1e-9")

    def omega(d):
        return D(cfg["omega_min_rad_s"]) + (D(cfg["omega_max_rad_s"]) - D(cfg["omega_min_rad_s"])) * (d - 48) / (2047 - 48)

    assert omega(d_lo) < want_omega < omega(d_hi)


def test_negative_control_wrong_mass_moves_the_bracket():
    cfg, lat, h = card_and_site()
    _, _, lo, hi = hover.bracket_for_card(cfg, lat, h)
    heavier = dict(cfg, mass_kg=cfg["mass_kg"] * 1.02)
    _, _, lo2, _ = hover.bracket_for_card(heavier, lat, h)
    assert lo2 > lo
    lighter = dict(cfg, mass_kg=cfg["mass_kg"] * 0.98)
    _, _, lo3, _ = hover.bracket_for_card(lighter, lat, h)
    assert lo3 < lo


def test_negative_control_bracket_follows_latitude():
    # g differs between the equator and the pole (5.2e-3 relative), which is more than the 2.4e-3 of one DShot step
    # at this hover point (one step is 2650 / 1999 rad/s of omega_h, and g goes as omega^2): the bracket must move
    cfg, _, h = card_and_site()
    lo_eq = hover.bracket_for_card(cfg, 0.0, h)[2]
    lo_pole = hover.bracket_for_card(cfg, math.pi / 2, h)[2]
    assert lo_pole > lo_eq


def test_bracket_outside_the_throttle_range_is_refused():
    cfg, lat, h = card_and_site()
    with pytest.raises(hover.HoverError):
        hover.bracket_for_card(dict(cfg, thrust_coeff=cfg["thrust_coeff"] * 100.0), lat, h)  # omega_h below omega_min
    with pytest.raises(hover.HoverError):
        hover.bracket_for_card(dict(cfg, thrust_coeff=cfg["thrust_coeff"] / 100.0), lat, h)  # above omega_max
