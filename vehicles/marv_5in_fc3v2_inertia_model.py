#!/usr/bin/env python3
"""Point-mass inertia and centre-of-gravity model of Luis's 7-inch quad (card vehicles/marv_5in_fc3v2.yaml, M4 P1).

Every mass is Luis's (2026-10-16/17) or the remainder of his 732 g all-up mass; every height above the arm plane and the
split of the frame between arms and plates is an assumption of this model (labelled ASSUMED below), not a measurement.
The model is INFERRED until the bifilar pendulum measures the inertia (quad spec 4.0 M4).

Run: python3 vehicles/marv_5in_fc3v2_inertia_model.py   (prints the nominal result and the sensitivity variants)

Frame: body FRD, x forward, y right, z down. Heights are metres above the arm mid-plane (up positive); the output z are
FRD relative to the centre of gravity (down positive), so a point above the CG has a negative z.
"""

import itertools

X, Y = 0.112, 0.1275  # m: Luis's ruler spans 224 mm (front-rear) and 255 mm (left-right), halved
MASS_TOTAL = 0.732    # kg, Luis: "the 732 includes the battery"
PACK, PROP, MOTOR, FRAME, ESC = 0.172, 0.009, 0.0479, 0.217, 0.015  # kg: pack, one prop, one motor, frame only, ESC

NOMINAL = {
    "arm_fraction": 0.5,   # ASSUMED: share of the 217 g frame in the four arms (rest in the plates); no drawing
    "h_plates": 0.005,     # ASSUMED m: plates (2/3/3 mm) mass-weighted height above the arm mid-plane
    "h_stack": 0.012,      # ASSUMED m: FC, receiver, wiring (the remainder mass) height
    "h_esc": 0.008,        # ASSUMED m: 4-in-1 ESC height
    "h_motor": 0.015,      # ASSUMED m: motor mass centre (2807 on top of a 6 mm arm: 3 + ~12 mm)
    "h_prop": 0.040,       # ASSUMED m: prop plane (thrust application point) above the arm mid-plane
    "h_pack": 0.035,       # ASSUMED m: pack mass centre (pack on the top plate, ~30 mm tall: top plate ~20 mm + 15 mm)
    "r_stack": 0.0,        # ASSUMED m: radius of the remainder mass about the centre (0: a point at the centre)
}


def model(p):
    pts = []  # (mass, x, y, h)
    arm_m = FRAME * p["arm_fraction"] / 4.0
    plate_m = FRAME * (1.0 - p["arm_fraction"])
    corners = [(sx * X, sy * Y) for sx in (1, -1) for sy in (1, -1)]
    for cx, cy in corners:
        pts.append((arm_m, cx / 2.0, cy / 2.0, 0.0))        # arm: point mass at its midpoint
        pts.append((MOTOR, cx, cy, p["h_motor"]))
        pts.append((PROP, cx, cy, p["h_prop"]))
    rest = MASS_TOTAL - PACK - 4 * PROP - 4 * MOTOR - FRAME - ESC
    pts += [(plate_m, 0.0, 0.0, p["h_plates"]), (ESC, 0.0, 0.0, p["h_esc"]), (rest, 0.0, 0.0, p["h_stack"]),
            (PACK, 0.0, 0.0, p["h_pack"])]
    m = sum(q[0] for q in pts)
    cg = sum(q[0] * q[3] for q in pts) / m
    ixx = sum(q[0] * (q[2] ** 2 + (q[3] - cg) ** 2) for q in pts)
    iyy = sum(q[0] * (q[1] ** 2 + (q[3] - cg) ** 2) for q in pts)
    izz = sum(q[0] * (q[1] ** 2 + q[2] ** 2) for q in pts)
    return {"mass": m, "rest": rest, "cg_h": cg, "motor_z_frd": -(p["h_prop"] - cg), "ixx": ixx, "iyy": iyy, "izz": izz}


def main():
    nom = model(NOMINAL)
    print(f"remainder (stack/wiring) = {nom['rest'] * 1e3:.1f} g, total {nom['mass'] * 1e3:.1f} g")
    print(f"CG height above the arm plane {nom['cg_h'] * 1e3:.2f} mm; thrust plane z (FRD, rel. CG) {nom['motor_z_frd'] * 1e3:.2f} mm")
    print(f"nominal J = ({nom['ixx']:.4e}, {nom['iyy']:.4e}, {nom['izz']:.4e}) kg m^2")
    lo, hi = {}, {}
    variants = {"arm_fraction": (0.3, 0.7), "h_plates": (0.0, 0.010), "h_stack": (0.0, 0.025), "h_esc": (0.0, 0.016),
                "h_motor": (0.005, 0.025), "h_prop": (0.030, 0.050), "h_pack": (0.025, 0.045), "r_stack": (0.0, 0.04)}
    for key, (a, b) in variants.items():
        for val in (a, b):
            p = dict(NOMINAL, **{key: val})
            if key == "r_stack":
                continue
            r = model(p)
            print(f"  {key}={val}: J = ({r['ixx']:.4e}, {r['iyy']:.4e}, {r['izz']:.4e}) CG {r['cg_h'] * 1e3:.1f} mm")
    # all-corner spread, one-at-a-time variants root-sum-squared
    for axis in ("ixx", "iyy", "izz"):
        dev = 0.0
        for key, (a, b) in variants.items():
            if key == "r_stack":
                continue
            ra, rb = model(dict(NOMINAL, **{key: a}))[axis], model(dict(NOMINAL, **{key: b}))[axis]
            dev += ((rb - ra) / 2.0) ** 2
        print(f"{axis}: rss of one-at-a-time half-spreads = {dev ** 0.5:.3e} ({100 * dev ** 0.5 / nom[axis]:.1f} %)")
    print("position-uncertainty effect: x,y +-5 mm each")
    for dx, dy in itertools.product((-0.005, 0.005), repeat=2):
        global X, Y
        x0, y0 = X, Y
        X, Y = x0 + dx, y0 + dy
        r = model(NOMINAL)
        X, Y = x0, y0
        print(f"  dx={dx:+.3f} dy={dy:+.3f}: J = ({r['ixx']:.4e}, {r['iyy']:.4e}, {r['izz']:.4e})")


if __name__ == "__main__":
    main()
