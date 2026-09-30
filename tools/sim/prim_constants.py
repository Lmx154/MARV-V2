"""Constants of fw/prim/include/marv/prim/constants.hpp, read from that file so the Python tools cannot drift from the
firmware. The header is the single place where each constant carries its citation (DShot: Betaflight DShot notes;
WGS 84: NIMA TR8350.2 3rd ed. Amdt 1 (2000) Tables 3.1, 3.3, 3.4 and eq. (4-3)); this module restates none of them.

Only plain `inline constexpr <type> kName = <decimal literal>;` lines are read. kWgs84F is the header's derived
constant, f = 1 / (1/f), restated here as the same expression.
"""

from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
HEADER = ROOT / "fw" / "prim" / "include" / "marv" / "prim" / "constants.hpp"
_LINE = re.compile(r"^inline constexpr [\w:]+ (k\w+) = ([-+0-9.eE]+);", re.MULTILINE)
NEEDED = (
    "kDshotThrottleMin", "kDshotThrottleMax", "kWgs84A", "kWgs84InvF", "kWgs84E2", "kWgs84GammaE", "kWgs84K",
    "kWgs84M", "kWgs84HeightQuadCoeff",
)


def load(header=HEADER):
    """dict name -> float (int for the DShot bounds) of every needed constant; raises KeyError if one is missing."""
    text = Path(header).read_text(encoding="utf-8")
    raw = dict(_LINE.findall(text))
    out = {}
    for name in NEEDED:
        lit = raw[name]
        out[name] = int(lit) if name.startswith("kDshot") else float(lit)
    out["kWgs84F"] = 1.0 / out["kWgs84InvF"]
    return out
