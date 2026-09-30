#!/usr/bin/env python3
"""L3 mixer parameters from a vehicle card (quad spec L3, decision 0004 items 1, 2 and 5).

Used by flatten.py (--out-mixer). For a linted card it yields the params_gen entries

  idle_speed                    rad/s  the card's speed_range.min (QF-6's measured minimum stable rotor speed replaces
                                       it later without mixer changes)
  mixer_m<i>_<axis>             M[i-1, axis], axis in thrust, roll, pitch, yaw: row i of M, so f_i = sum_axis
                                       M[i, axis] * u_axis, f_i the thrust of motor i in N

B is the effectiveness matrix in per-motor thrust, rows [thrust, roll, pitch, yaw]; column i is
[1, -y_i, x_i, s_i * c_q]^T with (x_i, y_i) the FRD rotor position of motor i, s_i = its yaw sign (ccw +1, cw -1) and
c_q = rotor torque ratio. That is the forward map of marv_plant v0: force -T z_FRD and torque r_i x f + z_FRD s_i c_q T.
M = B^-1 is computed in double by Gauss-Jordan elimination with partial pivoting. Units: thrust "1" (N per N), roll,
pitch, yaw "1/m" (N per N m). The mixer entries take sigma UNKNOWN: they inherit the sigma of their inputs.

The generator refuses (GenError) a card when
  * B is singular: the largest pivot magnitude left in some elimination step is <= n * eps * max|B|, n = 4 and eps the
    double epsilon. A pivot that small is not distinguishable from the rounding error of the elimination itself
    (Higham, Accuracy and Stability of Numerical Algorithms, ch. 9), so the inverse would be noise;
  * any M[i, thrust] <= 0: the collective would push a motor the wrong way;
  * zero torque is not achievable: max_i f_min / M[i, thrust] > min_i f_max / M[i, thrust], with f_min = k idle_speed^2
    and f_max = k speed_max^2. Pure collective c gives f_i = c M[i, thrust], so c must satisfy both bounds on every
    motor and the interval of c is empty.
"""

from __future__ import annotations

import sys

import gen_plant_config as gpc
import schema

MOTORS = gpc.MOTORS
SPIN_SIGN = gpc.SPIN_SIGN
AXES = ("thrust", "roll", "pitch", "yaw")
IDLE_METHOD = "derived(idle_speed = speed_range.min, the card's rotors.speed_range minimum, i.e. rotor_speed_min)"
MIXER_METHOD = ("derived(M = B^-1, B the effectiveness matrix in per-motor thrust with rows [thrust, roll, pitch, yaw] "
                "and column i = [1, -y_i, x_i, s_i c_q] from the FRD rotor position of motor i, its yaw sign and "
                "rotor_torque_ratio; inverted in double)")


def _known(entry, path, card_path):
    return gpc._known(entry, path, card_path)


def effectiveness(positions, yaw_signs, torque_ratio):
    """B as a 4x4 list of rows (double)."""
    cols = [[1.0, -p[1], p[0], s * torque_ratio] for p, s in zip(positions, yaw_signs)]
    return [[cols[j][i] for j in range(len(cols))] for i in range(len(AXES))]


def inverse(b):
    """(M, smallest pivot, threshold): B^-1 by Gauss-Jordan with partial pivoting; M is None when B is singular."""
    n = len(b)
    eps = sys.float_info.epsilon
    threshold = n * eps * max(abs(x) for row in b for x in row)
    a = [list(row) + [1.0 if i == j else 0.0 for j in range(n)] for i, row in enumerate(b)]
    smallest = float("inf")
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(a[r][c]))
        smallest = min(smallest, abs(a[p][c]))
        if abs(a[p][c]) <= threshold:
            return None, smallest, threshold
        a[c], a[p] = a[p], a[c]
        piv = a[c][c]
        a[c] = [x / piv for x in a[c]]
        for r in range(n):
            if r != c and a[r][c] != 0.0:
                f = a[r][c]
                a[r] = [x - f * y for x, y in zip(a[r], a[c])]
    return [row[n:] for row in a], smallest, threshold


def mixer_matrix(card, card_path):
    """(M, B, idle_speed): raises gpc.GenError when the card is refused."""
    rotors = card["rotors"]
    positions = [[float(v) for v in _known(rotors[m]["position"], f"rotors.{m}.position", card_path)]
                 for m in MOTORS]
    signs = [SPIN_SIGN[rotors[m]["spin"]["value"]] for m in MOTORS]
    c_q = float(_known(rotors["torque_ratio"], "rotors.torque_ratio", card_path))
    k = float(_known(rotors["thrust_coeff"], "rotors.thrust_coeff", card_path))
    speed = _known(rotors["speed_range"], "rotors.speed_range", card_path)
    idle, top = float(speed[0]), float(speed[1])
    b = effectiveness(positions, signs, c_q)
    m, smallest, threshold = inverse(b)
    if m is None:
        gpc.refuse(card_path, "rotors", f"the effectiveness matrix B is singular: pivot {smallest:.3g} <= "
                                        f"n eps max|B| = {threshold:.3g}")
    for i in range(len(MOTORS)):
        if not m[i][0] > 0.0:
            gpc.refuse(card_path, f"rotors.m{i + 1}", f"mixer thrust entry M[{i + 1},thrust] = {m[i][0]:.6g} is not "
                                                      "positive")
    f_min, f_max = k * idle * idle, k * top * top
    lo = max(f_min / m[i][0] for i in range(len(MOTORS)))
    hi = min(f_max / m[i][0] for i in range(len(MOTORS)))
    if lo > hi:
        gpc.refuse(card_path, "rotors", f"zero torque is not achievable: max f_min/M[i,thrust] = {lo:.6g} N exceeds "
                                        f"min f_max/M[i,thrust] = {hi:.6g} N")
    return m, b, idle


def mixer_entries(card, card_path):
    """The ordered (parameter name, params_gen entry) list: idle_speed, then mixer_m<i>_<axis> row by row."""
    m, _, idle = mixer_matrix(card, card_path)
    rotors = card["rotors"]
    speed = rotors["speed_range"]
    out = [("idle_speed", {
        "type": "f32", "value": idle, "unit": speed["unit"], "method": IDLE_METHOD,
        "source": "vehicle card rotors.speed_range value[0]; stands in for the measured minimum stable rotor speed "
                  "(quad spec QF-6, decision 0004 item 1), which will replace it",
        "sigma": schema.UNKNOWN})]
    status, check = [], []
    for n in MOTORS:
        spin = rotors[n]["spin"]
        status += [s for s in spin.get("status", []) if s not in status]
        if "check" in spin and spin["check"] not in check:
            check.append(spin["check"])
    for i, row in enumerate(m):
        for axis, v in zip(AXES, row):
            entry = {
                "type": "f32", "value": v, "unit": "1" if axis == "thrust" else "1/m", "method": MIXER_METHOD,
                "source": f"vehicle card rotors m1..m4 position and spin, rotors.torque_ratio; row {i + 1}, {axis} "
                          "column of M (decision 0004 item 2)",
                "sigma": schema.UNKNOWN}
            if status:
                entry["status"] = status
            if check:
                entry["check"] = "; ".join(check)
            out.append((f"mixer_m{i + 1}_{axis}", entry))
    return out
