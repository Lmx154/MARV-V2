"""Hover DShot bracket for a card and a site (docs/decisions/0003 item 7), and WGS 84 normal gravity.

  g(phi, h)   NIMA TR8350.2 3rd ed. Amdt 1 (2000) eq. (4-1) and (4-3), mirroring fw/prim/include/marv/prim/gravity.hpp
              operation for operation, with the constants read from fw/prim/include/marv/prim/constants.hpp:
                gamma   = gamma_e (1 + k sin^2 phi) / sqrt(1 - e^2 sin^2 phi)                    (4-1)
                gamma_h = gamma [1 - (2/a)(1 + f + m - 2 f sin^2 phi) h + (3/a^2) h^2]           (4-3)
              phi is geodetic latitude in rad, h the geodetic height above the ellipsoid in m, positive up.
  omega_h     sqrt(mass g(phi, h0) / (4 k)), with the card's mass and thrust coefficient k. (Not the timestep
              multiplier m of the scenarios.)
  D*          the inverse of the ESC map of the card (linear in omega: DShot 48 -> omega_min, 2047 -> omega_max):
              D* = 48 + (omega_h - omega_min) (2047 - 48) / (omega_max - omega_min), the same expression order as
              tests/regression/quad/L01/unit/plant_card/hover_card_test.cpp.
  D_lo, D_hi  floor(D*) and floor(D*) + 1. Both must be legal throttle values (48..2047).
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import prim_constants  # noqa: E402

_C = prim_constants.load()


def normal_gravity_ellipsoid(phi):
    s = math.sin(phi)
    s2 = s * s
    return _C["kWgs84GammaE"] * (1.0 + _C["kWgs84K"] * s2) / math.sqrt(1.0 - _C["kWgs84E2"] * s2)


def normal_gravity(phi, h):
    s = math.sin(phi)
    s2 = s * s
    inv_a = 1.0 / _C["kWgs84A"]
    f = _C["kWgs84F"]
    linear = 2.0 * inv_a * (1.0 + f + _C["kWgs84M"] - 2.0 * f * s2) * h
    quad = _C["kWgs84HeightQuadCoeff"] * inv_a * inv_a * h * h
    return normal_gravity_ellipsoid(phi) * (1.0 - linear + quad)


class HoverError(Exception):
    pass


def bracket(mass_kg, thrust_coeff, omega_min, omega_max, lat_rad, height_m):
    """(omega_h, d_star, d_lo, d_hi); raises HoverError if the bracket is not inside the throttle range."""
    dmin = _C["kDshotThrottleMin"]
    dmax = _C["kDshotThrottleMax"]
    g = normal_gravity(lat_rad, height_m)
    omega_h = math.sqrt(mass_kg * g / (4.0 * thrust_coeff))
    d_star = dmin + (omega_h - omega_min) * (dmax - dmin) / (omega_max - omega_min)
    d_lo = math.floor(d_star)
    d_hi = d_lo + 1
    if not (dmin <= d_lo and d_hi <= dmax):
        raise HoverError(f"hover DShot bracket [{d_lo}, {d_hi}] (D* = {d_star!r}) is outside {dmin}..{dmax}")
    return omega_h, d_star, d_lo, d_hi


def bracket_for_card(plant_cfg, lat_rad, height_m):
    """`plant_cfg` is gen_plant_config.plant_config(card): the linted card values."""
    return bracket(plant_cfg["mass_kg"], plant_cfg["thrust_coeff"], plant_cfg["omega_min_rad_s"],
                   plant_cfg["omega_max_rad_s"], lat_rad, height_m)
