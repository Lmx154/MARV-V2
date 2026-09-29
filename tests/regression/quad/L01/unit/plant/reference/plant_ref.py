# Frozen path: tests/regression/quad/L01/unit/plant/reference/plant_ref.py
# Independent reference of the marv_plant v0 wrench at steady rotor speed, plain Python math, no numpy, no code shared
# with the C++ implementation. Reads plant_ref_inputs.txt, writes plant_ref_expected.txt (both beside this script).
#   uv run python tests/regression/quad/L01/unit/plant/reference/plant_ref.py
# Every value in the inputs file is a scenario value (L1 test fixture). The outputs are reproducible byte for byte on
# one machine; another libm may differ in the last digit of sin, exp or sqrt, which the C++ test's tolerance absorbs.
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# DShot range of the ESC map (Betaflight DShot notes): the map is linear in omega between these (marv_plant.h).
DSHOT_MIN = 48
DSHOT_MAX = 2047

# WGS 84 normal gravity, NIMA TR8350.2 3rd ed. Amdt 1 (2000). Restated here from the tables, not from constants.hpp.
A = 6378137.0                    # Table 3.1, semi-major axis, m
INV_F = 298.257223563            # Table 3.1, inverse flattening
F = 1.0 / INV_F
E2 = 6.69437999014e-3            # Table 3.3, first eccentricity squared
GAMMA_E = 9.7803253359           # Table 3.4, equatorial normal gravity, m/s^2
K = 0.00193185265241             # Table 3.4, Somigliana's constant
M = 0.00344978650684             # Table 3.4, m = omega^2 a^2 b / GM


def gravity(lat, h):
    s2 = math.sin(lat) ** 2
    gamma = GAMMA_E * (1.0 + K * s2) / math.sqrt(1.0 - E2 * s2)                          # eq. (4-1)
    series = 1.0 - (2.0 / A) * (1.0 + F + M - 2.0 * F * s2) * h + (3.0 / A**2) * h * h  # eq. (4-3)
    return gamma * series


def rotation(q):
    # R = (w^2 - u.u) I + 2 u u^T + 2 w [u]x for a unit quaternion (w, u), body -> NED.
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    d = w * w - (x * x + y * y + z * z)
    u = (x, y, z)
    skew = ((0.0, -z, y), (z, 0.0, -x), (-y, x, 0.0))
    return [[d * (1.0 if i == j else 0.0) + 2.0 * u[i] * u[j] + 2.0 * w * skew[i][j] for j in range(3)]
            for i in range(3)]


def matvec(r, v):
    return [sum(r[i][j] * v[j] for j in range(3)) for i in range(3)]


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def read_inputs(path):
    cfg, cases = {}, []
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if parts[0] == "case":
                cases.append((parts[1], [float(p) for p in parts[2:9]], [int(p) for p in parts[9:13]]))
            else:
                cfg[parts[0]] = [float(p) for p in parts[1:]]
    return cfg, cases


def omega_cmd(cfg, dshot):
    if dshot == 0:
        return 0.0
    lo, hi = cfg["omega_min_rad_s"][0], cfg["omega_max_rad_s"][0]
    return lo + (hi - lo) * (dshot - DSHOT_MIN) / (DSHOT_MAX - DSHOT_MIN)


def solve(cfg, case):
    _, num, dshot = case
    pos, q = num[0:3], num[3:7]
    k, ratio, mass = cfg["thrust_coeff"][0], cfg["torque_ratio_m"][0], cfg["mass_kg"][0]
    rp = cfg["rotor_position_frd_m"]
    omega = [omega_cmd(cfg, d) for d in dshot]
    force_b, torque_b = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
    for i in range(4):
        thrust = k * omega[i] ** 2
        f = [0.0, 0.0, -thrust]                       # thrust along -z_FRD
        r = rp[3 * i:3 * i + 3]
        rxf = cross(r, f)
        spin = cfg["yaw_sign"][i] * ratio * thrust    # reaction torque about +z_FRD, positive for ccw
        force_b = [force_b[j] + f[j] for j in range(3)]
        torque_b = [torque_b[j] + rxf[j] + (spin if j == 2 else 0.0) for j in range(3)]
    g = gravity(cfg["site_lat_rad"][0], cfg["site_height_m"][0] - pos[2])
    rot = rotation(q)
    fn = matvec(rot, force_b)
    fn[2] += mass * g                                 # gravity along +z_NED (down)
    tn = matvec(rot, torque_b)
    pairs = cfg["pole_count"][0] / 2.0
    erpm = [w * (60.0 / (2.0 * math.pi)) * pairs for w in omega]
    return omega, erpm, fn, tn, g


def main():
    cfg, cases = read_inputs(os.path.join(HERE, "plant_ref_inputs.txt"))
    lines = ["# Frozen path: tests/regression/quad/L01/unit/plant/reference/plant_ref_expected.txt",
             "# Written by plant_ref.py from plant_ref_inputs.txt; do not edit by hand.",
             "# name omega[4] erpm[4] force_ned[3] torque_ned[3] g"]
    for case in cases:
        omega, erpm, fn, tn, g = solve(cfg, case)
        vals = omega + erpm + fn + tn + [g]
        lines.append(case[0] + " " + " ".join(repr(v) for v in vals))
    with open(os.path.join(HERE, "plant_ref_expected.txt"), "w") as fh:
        fh.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
