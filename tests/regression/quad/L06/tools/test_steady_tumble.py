"""T1: R2's steady-tumble rotor speeds hold the body rates on the plant itself (decision 0015; decision 0014 owner decision 5,
condition 2, and second round item 1).

The rule is tools/sim/run_l5.py steady_tumble on scenarios/quad/L05/recover_tumble.yaml (rotor_speed_rad_s: steady_tumble).
marv_plant is run through its C ABI (plant_torque_tool.cpp, compiled here, as the other L06 tool checks compile theirs) with
the card's plant configuration, the scenario's site, seed and tick, the rule's speeds as initial_omega_rad_s and the scenario's
initial body state. One marv_plant_step with every DShot 0 and dt the smallest positive double leaves the rotor speeds exactly
as they were (omega <- 0 + (omega - 0) exp(-dt/tau), and exp(-dt/tau) is 1 in binary64; asserted bit for bit), so the torque
it returns is the plant's torque at t = 0. R2 starts at [0, 1, 0, 0], whose rotation matrix is diag(1, -1, -1) exactly in
binary64 (prim Quat::to_rotation_matrix: every entry is 1 - 2 (1 + 0), 1 - 2 (0 + 0) or 2 (0 - 0)), so the NED torque maps
back to the body without rounding.

The check. w' = J^-1 (tau_plant - w0 x J w0) = 0 within tol_a / J_a on every axis a, evaluated exactly (fractions) from the
doubles: the plant's body torque, w0 and the card's J. tol_a bounds every rounding between the exact balance and the plant's
output (u = 2^-53, gamma_n = n u / (1 - n u), Higham, Accuracy and Stability of Numerical Algorithms, 2nd ed., section 3.1):
  plant      gamma_6 sum_i |B_ai| k w_i^2: T_i = (k w_i) w_i (2 roundings), B_ai T_i (1; the plant's r_i x (0, 0, -T_i) and
             s_i c_q T_i are B_ai T_i with the same single rounding, B the card's effectiveness matrix, tools/card/mixer.py),
             the sum over 4 motors (3); the plant's other components add exact zeros;
  speeds     gamma_3 sum_i |B_ai| T^_i: w_i = sqrt(T^_i / k) (2 roundings), so k w_i^2 = T^_i (1 + d1)(1 + d2)^2;
  allocation gamma_4 sum_i |B_ai| sum_j |M_ij| |v_j|: T^_i = M_i0 c* + sum_axis M_i,axis tau^_axis, v = (c*, tau^), 4 products
             and 3 sums, at most 4 roundings per term;
  inverse    |((B M - I) v)_a|, exact: M = B^-1 is computed in binary64 (mixer.inverse), so B M is not I; its residual is
             taken exactly, not bounded;
  coupling   gamma_3 (|w_b J_c w_c| + |w_c J_b w_b|): the rule's tau^ = w0 x (J w0) (J w, the product and the difference).
No other torque enters: marv_plant v0 applies the rotor thrust moments and the rotor yaw reaction only (no rotor inertia, so
no rotor gyroscopic torque; no drag; no position-dependent torque), at the centre of mass. A torque the rule left out would
show here at its own size, far above tol.

Negative control (owner decision 5, condition 2): hover rotors (run_l5.hover_rotor_speeds, the R2 start before decision 0015),
under the same check with the tolerance of their own allocation (v = (m g, 0, 0, 0)), must fail it.
"""

import math
import shutil
import subprocess
import sys
from fractions import Fraction
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
sys.path.insert(0, str(ROOT / "tools" / "sim"))
import gen_plant_config as gpc  # noqa: E402
import l5_scenario as l5s  # noqa: E402
import mixer  # noqa: E402
import run_l5  # noqa: E402
import scenario as scn  # noqa: E402

CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
SCEN = ROOT / "scenarios" / "quad" / "L05" / "recover_tumble.yaml"
TOOL_SRC = Path(__file__).resolve().parent / "plant_torque_tool.cpp"
INVERTED = [0.0, 1.0, 0.0, 0.0]
R_INVERTED_DIAG = (1, -1, -1)  # the exact rotation matrix of INVERTED (module docstring)
U = Fraction(1, 2 ** 53)  # binary64 unit roundoff
STOP = 0  # DShot 0: commanded rotor speed 0


def gamma(n):
    return n * U / (1 - n * U)


@pytest.fixture(scope="module")
def tool(tmp_path_factory):
    cxx = shutil.which("c++") or shutil.which("g++") or shutil.which("clang++")
    if cxx is None:
        pytest.fail("no C++ compiler on this host: the T1 plant check cannot run (it is a required check)")
    exe = tmp_path_factory.mktemp("plant_torque_tool") / "plant_torque_tool"
    inc = [f"-I{ROOT / p}" for p in ("sim/plant/include", "sim/plant/src", "fw/prim/include", "fw/types/include")]
    build = subprocess.run([cxx, "-std=c++20", "-ffp-contract=off", "-Wall", "-Wextra", "-Werror", "-Wdouble-promotion",
                            "-Wfloat-conversion", *inc, str(TOOL_SRC), str(ROOT / "sim/plant/src/marv_plant.cpp"),
                            str(ROOT / "sim/plant/src/noise/noise.cpp"), "-o", str(exe)],
                           capture_output=True, text=True, check=False)
    assert build.returncode == 0, build.stderr
    return exe


@pytest.fixture(scope="module")
def case():
    card_doc, profile = gpc.load_linted(CARD)
    cfg, _ = gpc.config_from(card_doc, profile, CARD)
    m, b, _ = mixer.mixer_matrix(card_doc, CARD)
    vals = l5s.values(l5s.load(SCEN))
    assert vals["initial_state"]["rotor_speed_rad_s"] == l5s.ROTOR_SPEED_STEADY_TUMBLE
    assert vals["initial_state"]["attitude_q_wxyz"] == INVERTED
    rule = run_l5.steady_tumble(CARD, vals)
    hover = run_l5.l4.hover_thrust(CARD, vals)
    return {"cfg": cfg, "M": m, "B": b, "vals": vals, "rule": rule, "J": [float(x) for x in card_doc["inertia_diag"]["value"]],
            "hover": {"speeds": run_l5.hover_rotor_speeds(CARD, vals), "v": (hover, 0.0, 0.0, 0.0),
                      "thrusts": [m[i][0] * hover for i in range(scn.MOTORS)]}}


def plant_body_torque(tool, case, speeds):
    """(body torque, rotor speeds) the plant returns after one step from `speeds` (module docstring)."""
    cfg, vals = case["cfg"], case["vals"]
    st = vals["initial_state"]
    tick = float(Fraction(vals["tick_period_num_us"], vals["tick_period_den"] * scn.MICROSECONDS_PER_SECOND))
    h = float.hex
    words = [str(cfg["esc_map"]), str(cfg["pole_count"]), *map(str, cfg["yaw_sign"]), h(cfg["mass_kg"]),
             *(h(x) for r in cfg["rotor_position_frd_m"] for x in r),
             *map(h, (cfg["thrust_coeff"], cfg["torque_ratio_m"], cfg["omega_min_rad_s"], cfg["omega_max_rad_s"],
                      cfg["motor_tau_s"], tick, vals["site_latitude_rad"], vals["site_height_m"], math.ulp(0.0))),
             str(vals["seed"]), *map(h, speeds),
             *(h(float(x)) for k in ("position_ned_m", "velocity_ned_m_s", "attitude_q_wxyz", "body_rates_frd_rad_s")
               for x in st[k]),
             *[str(STOP)] * scn.MOTORS]
    run = subprocess.run([str(tool)], input=" ".join(words) + "\n", capture_output=True, text=True, check=False)
    assert run.returncode == 0, run.stderr
    out = [float.fromhex(x) for x in run.stdout.split()]
    return [s * x for s, x in zip(R_INVERTED_DIAG, out[:3])], out[3:]


def check(case, torque_body, speeds, thrusts, v):
    """Per axis (w' exact, its tolerance tol_a / J_a), both in rad/s^2 (module docstring), as Fractions."""
    b, m, j = case["B"], case["M"], case["J"]
    w = [Fraction(x) for x in case["vals"]["initial_state"]["body_rates_frd_rad_s"]]
    jf = [Fraction(x) for x in j]
    k = Fraction(case["cfg"]["thrust_coeff"])
    c0 = (w[1] * jf[2] * w[2] - w[2] * jf[1] * w[1], w[2] * jf[0] * w[0] - w[0] * jf[2] * w[2],
          w[0] * jf[1] * w[1] - w[1] * jf[0] * w[0])
    pairs = ((1, 2), (2, 0), (0, 1))
    vf = [Fraction(x) for x in v]
    bm = [[sum(Fraction(b[r][i]) * Fraction(m[i][c]) for i in range(scn.MOTORS)) - (r == c) for c in range(len(vf))]
          for r in range(len(vf))]
    out = []
    for a in range(len(c0)):
        row = [abs(Fraction(x)) for x in b[a + 1]]
        tol = (gamma(6) * sum(row[i] * k * Fraction(speeds[i]) ** 2 for i in range(scn.MOTORS))
               + gamma(3) * sum(row[i] * abs(Fraction(thrusts[i])) for i in range(scn.MOTORS))
               + gamma(4) * sum(row[i] * sum(abs(Fraction(m[i][c])) * abs(vf[c]) for c in range(len(vf)))
                                for i in range(scn.MOTORS))
               + abs(sum(bm[a + 1][c] * vf[c] for c in range(len(vf))))
               + gamma(3) * (abs(w[pairs[a][0]] * jf[pairs[a][1]] * w[pairs[a][1]])
                             + abs(w[pairs[a][1]] * jf[pairs[a][0]] * w[pairs[a][0]])))
        out.append(((Fraction(torque_body[a]) - c0[a]) / jf[a], tol / jf[a]))
    return out


def report(capsys, label, res):
    with capsys.disabled():
        print()
        print(f"{label}: " + "; ".join(f"axis {a}: w' {float(x):+.3e} rad/s^2, tol {float(t):.3e}" for a, (x, t) in enumerate(res)))


def test_t1_steady_tumble_rotor_speeds_hold_the_rates_on_the_plant(tool, case, capsys):
    rule = case["rule"]
    lo, hi = rule["speed_range"]
    assert all(lo <= x <= hi for x in rule["speeds"])
    assert list(rule["speeds"]) == [math.sqrt(t / case["cfg"]["thrust_coeff"]) for t in rule["thrusts"]]
    torque, after = plant_body_torque(tool, case, rule["speeds"])
    assert after == list(rule["speeds"]), "the plant's rotor state moved: the torque is not the one at t = 0"
    res = check(case, torque, rule["speeds"], rule["thrusts"], (rule["collective"], *rule["torque"]))
    report(capsys, f"steady tumble (c* {rule['collective']!r} N, hover {rule['hover_collective']!r} N)", res)
    assert all(abs(x) <= t for x, t in res), res


def test_control_hover_rotors_fail_the_t1_check(tool, case, capsys):
    hv = case["hover"]
    torque, after = plant_body_torque(tool, case, hv["speeds"])
    assert after == list(hv["speeds"])
    res = check(case, torque, hv["speeds"], hv["thrusts"], hv["v"])
    report(capsys, "control, hover rotors", res)
    assert any(abs(x) > t for x, t in res), res
