"""The T4e design models of the refdata set quad/L06/t4e (tools/sim/l6_t4e_model.py; decision 0019, the noise suite
design memo of 2026-10-04, F-a and section 1, commit C6a), per push.

  (1) T4c identity: every generalised model on the T4c chain (latency 0, the low-pass alone) is its frozen original bit
      for bit: L4 l4_loop against rate_t3_oracle.closed_loop (trajectory and rho), l1_norm, the TOL and band_envelope;
      L5 L5Lin against attitude_t3_oracle.Lin.run (every injection kind, both Jacobian ends, a lock, a nonzero initial
      state with its recorded states, both drives), Member4e's tilt and yaw runs (with the quantiser) against the
      oracle's, recovery_run against recovery_model.member_run (quantised, two sub-steps), simulate4e against the oracle's
      simulate, l5_tolerance against the oracle's tolerance (the frozen-gain and the yaw lock regimes). Controls: the
      same models with latency 1 differ;
  (2) the T4e L4 model on the acro script equals l6_ff_eval.linear_response with the same sensor (C4's model) bit for
      bit; the T4e chain is gyro_chain_design.chain_stages at omega_hover on the fixture's values;
  (3) notch_f32_error bounds a binary32 emulation of notch_coeffs (every operation rounded, sinf and cosf as the correctly
      rounded result); control: a thousandth of the bound does not;
  (4) the L5 noise sums (l5_unit_sums: the seed run, the phase-class runs, the step recursion) equal the direct
      computation, one run per injection tick, within test_l6_noise_term.direct_sums's first-order bound, at both
      Jacobian ends; controls: the dt phases swapped and the seed path dropped each break it.
"""

import math
import struct
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "sim"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import l6_t4e_model as m  # noqa: E402
import test_l6_noise_term as c4  # noqa: E402  (its direct-sum helpers)

lo4, ao, rm, fe, nt, gcd = m.lo4, m.ao, m.rm, m.fe, m.nt, m.gcd
REF = ROOT / "tests" / "regression" / "quad"
P4 = lo4.read_inputs(str(REF / "L04" / "t3" / "reference" / "rate_t3_inputs.txt"))
P5 = ao.read_inputs(str(REF / "L05" / "t3" / "reference" / "attitude_t3_inputs.txt"))
QFIX = ao.read_q_inputs(str(REF / "L05" / "t3" / "reference" / "attitude_t3_q_inputs.txt"))
T4E_INPUTS = REF / "L06" / "t4e" / "reference" / "t4e_inputs.txt"
N_L4 = 300          # labelled test value: L4 executions of the identity checks
H_TILT = 40         # labelled test value: L5 segment executions of the tilt identity checks
H_YAW = 1000        # labelled test value: the shortest yaw segment whose release locks inside the run (2 H + 1)
N_LIN = 150         # labelled test value: L5Lin boundaries of the identity checks
N_REC = 80          # labelled test value: recovery executions of the identity checks
WINDOW = 40         # labelled test value: observations of the L5 direct noise sums
MEMBER = (0.5, -0.25)  # a grid member off the nominal
CORNER = (1.0, -1.0)   # a box corner
EMULATION_SHARE = 1.0e-3  # control (3): the bound scaled by this must fail the emulation


def _omega_hover():
    for line in T4E_INPUTS.read_text().splitlines():
        f = line.split("#")[0].split()
        if f and f[0] == "omega_hover":
            return float.fromhex(f[1])
    raise AssertionError("t4e_inputs.txt has no omega_hover")


OMEGA_HOVER = _omega_hover()
T4C4 = m.Chain("T4c", 0, m.t4c_stages(P4))
T4C5 = m.Chain("T4c", 0, m.t4c_stages(P5))
LAT1 = m.Chain("latency 1", 1, m.t4c_stages(P5))
T4E4 = m.Chain("T4e", 1, m.t4e_stages(P4, OMEGA_HOVER))
T4E5 = m.Chain("T4e", 1, m.t4e_stages(P5, OMEGA_HOVER))


# ---- (1) T4c identity: L4 -------------------------------------------------------------------------------------------

def _l4_args(axis, chain=T4C4):
    su = lo4.Setup(P4)
    return su, m.l4_law(P4, axis), su.tick_map(P4[lo4.INERTIA[axis]], P4["motor_tau"]), m.oracle_stamps(su, N_L4)


def test_l4_loop_is_the_oracle_on_t4c():
    for axis in lo4.AXES:
        su, law, tmap, stamps = _l4_args(axis)
        sp = P4[f"rate_max_{axis}"]
        want = lo4.closed_loop(su, axis, P4[lo4.INERTIA[axis]], P4["motor_tau"], N_L4, sp, want_rho=True)
        assert m.l4_loop(law, T4C4, tmap, [sp] * N_L4, stamps, su.divisor, want_rho=True) == want, axis
        for node in lo4.NODES:
            for k in (2, 3):
                ats = [(node, k - 1, t) for t in range(su.divisor)] if node in lo4.TICK_NODES else [(node, k - 1)]
                got = 0.0
                for at in ats:
                    got += sum(abs(v) for v in m.l4_loop(law, T4C4, tmap, [0.0] * N_L4, stamps, su.divisor, inject=at)[k:])
                assert got == lo4.l1_norm(su, axis, node, k, N_L4), (axis, node, k)
        tol, _, _ = m.l4_tol(P4, axis, T4C4, [sp] * N_L4, stamps, N_L4)
        l1 = {n: max(lo4.l1_norm(su, axis, n, k, N_L4) for k in (2, 3)) for n in lo4.NODES}
        assert tol == sum(l1[n] * want[1][n] for n in lo4.NODES), axis
    su, _, _, stamps = _l4_args("roll")
    band = m.l4_band(P4, "roll", T4C4, [P4["rate_max_roll"]] * N_L4, stamps)
    assert (band["lo"], band["hi"]) == lo4.band_envelope(su, "roll", N_L4, P4["rate_max_roll"], m.GRID)


def test_l4_loop_control_latency_1_differs():
    su, law, tmap, stamps = _l4_args("roll")
    sp = [P4["rate_max_roll"]] * N_L4
    lat1 = m.Chain("latency 1", 1, T4C4.stages)
    assert m.l4_loop(law, lat1, tmap, sp, stamps, su.divisor) != lo4.closed_loop(
        su, "roll", P4["inertia_xx"], P4["motor_tau"], N_L4, P4["rate_max_roll"])


# ---- (1) T4c identity: L5 -------------------------------------------------------------------------------------------

def test_l5_lin_is_the_oracle_lin_on_t4c():
    su = ao.Setup(P5)
    for axis in range(3):
        lin, mine = ao.Lin(su, axis), m.L5Lin(su, axis, T4C5)
        zero = [0.0] * ao.STATE
        for kind, at in (("gyro", 7), ("lpf", 9), ("e", 3), ("I", 4), ("D", 5), ("u", 6), ("ref", 2)):
            for fb in m.l5_fb_ends(su, axis):
                want, _ = lin.run(0, N_LIN, zero, 0.0, fb, kind=kind, at=at)
                assert mine.run(0, N_LIN, zero, 0.0, fb, kind=kind, at=at)[0] == want, (axis, kind, fb)
                assert mine.run(0, N_LIN, zero, 0.0, fb, kind=kind, at=at, seed=True)[0] == want, (axis, kind, fb)
        want, _ = lin.run(0, N_LIN, zero, 0.0, 0.0, kind="gyro", at=11, lock=(40, su.cfg.kp))
        assert mine.run(0, N_LIN, zero, 0.0, 0.0, kind="gyro", at=11, lock=(40, su.cfg.kp))[0] == want
        state = [0.1 * (k + 1) for k in range(ao.STATE)]
        want, sts = lin.run(40, N_LIN, state, 0.3, su.cfg.kp, kind="gyro", at=55, record_states=True)
        got, rec = mine.run(40, N_LIN, state, 0.3, su.cfg.kp, kind="gyro", at=55, record=set(range(len(sts))))
        assert got == want and all(tuple(rec[k]) == tuple(sts[k]) for k in range(len(sts)))
        seq = [((k * 7) % 13 - 6) / 100 for k in range(N_LIN * su.divisor)]
        for drive, lock in ((("lpf", seq), None), (("D", seq[:N_LIN]), (60, su.cfg.kp))):
            want, _ = lin.run(0, N_LIN, zero, 0.0, su.cfg.kp, drive=drive, lock=lock)
            assert mine.run(0, N_LIN, zero, 0.0, su.cfg.kp, drive=drive, lock=lock)[0] == want, drive[0]


def test_l5_lin_control_latency_1_differs():
    su = ao.Setup(P5)
    want, _ = ao.Lin(su, 0).run(0, N_LIN, [0.0] * ao.STATE, 0.0, su.cfg.kp, kind="gyro", at=7)
    got, _ = m.L5Lin(su, 0, LAT1).run(0, N_LIN, [0.0] * m.l5_state_size(LAT1), 0.0, su.cfg.kp, kind="gyro", at=7)
    assert got != want


def test_l5_band_members_are_the_oracles_on_t4c():
    su = ao.Setup(P5)
    s, t = MEMBER
    for axis in (0, 1):
        assert m.tilt_member_run(su, axis, s, t, H_TILT, T4C5) == ao.tilt_member_run(su, axis, s, t, H_TILT)
    assert (m.tilt_member_run(su, 0, *CORNER, H_TILT, T4C5, ao.Quantiser(QFIX, 0))
            == ao.tilt_member_run(su, 0, *CORNER, H_TILT, ao.Quantiser(QFIX, 0)))
    for d in (0.0, P5["tau_held_yaw_nm"]):
        assert (m.yaw_member_run(su, s, t, H_TILT, 1.0, d, T4C5, ao.Quantiser(QFIX, 2))
                == ao.yaw_member_run(su, s, t, H_TILT, 1.0, d, ao.Quantiser(QFIX, 2)))
    q0, w0 = (0.0, 1.0, 0.0, 0.0), (P5["rate_max_roll"], P5["rate_max_pitch"], P5["rate_max_yaw"])
    assert m.recovery_run(su, *CORNER, q0, w0, N_REC, T4C5) == rm.member_run(su, *CORNER, q0, w0, N_REC)
    assert (m.recovery_run(su, *CORNER, q0, w0, N_REC, T4C5, rm.Quant3(QFIX), m_sub=2)
            == rm.member_run(su, *CORNER, q0, w0, N_REC, rm.Quant3(QFIX), m_sub=2))
    assert m.tilt_member_run(su, 0, s, t, H_TILT, LAT1) != ao.tilt_member_run(su, 0, s, t, H_TILT)
    assert m.recovery_run(su, *CORNER, q0, w0, N_REC, LAT1) != rm.member_run(su, *CORNER, q0, w0, N_REC)


def test_l5_golden_and_tolerance_are_the_oracles_on_t4c():
    su = ao.Setup(P5)
    for name, h in (("step_roll", H_TILT), ("yaw_release", H_YAW)):
        want = ao.simulate(su, name, h)
        got = m.simulate4e(su, name, h, T4C5)
        assert got == want, name
        if name == "yaw_release":
            assert want["lock_a"] is not None, "the identity check needs the yaw lock inside the run"
        assert m.l5_tolerance(su, name, got, T4C5) == ao.tolerance(su, name, want), name
    assert m.simulate4e(su, "step_roll", H_TILT, LAT1)["th"] != ao.simulate(su, "step_roll", H_TILT)["th"]


# ---- (2) the T4e chain and C4's model -------------------------------------------------------------------------------

def test_t4e_chain_and_the_c4_model():
    su = lo4.Setup(P4)
    qs = [P4[f"gyro_notch_q_h{i}"] for i in (1, 2, 3)]
    want = tuple(st for st in gcd.chain_stages(su.tick_s, P4["gyro_lpf_cutoff_hz"], qs, P4["gyro_notch_omega_min"],
                                               [OMEGA_HOVER] * gcd.MOTORS) if st != gcd.IDENTITY)
    assert T4E4.coeffs == want and T4E4.coeffs == T4E5.coeffs
    assert len(T4E4.stages) == gcd.MOTORS * len(gcd.HARMONICS) + 1, "every notch active at omega_hover"
    plan = fe.run_l4.plan_acro(fe.run_l4.l4s.load_acro(fe.ACRO), fe.plan_params(P4), fe.CARD)
    sps, stamps = fe.run_l4.acro_setpoints(plan, 0)
    jt, tt = P4["inertia_xx"] * (1 + MEMBER[0] * su.j_band), P4["motor_tau"] * (1 + MEMBER[1] * su.tau_band)
    su1 = lo4.Setup({**P4, "rate_loop_divisor": 1})
    maps = (su.plant_map(jt, tt), su1.tick_map(jt, tt))
    kp, ki, kd, tau_ref, t_f = m.l4_law(P4, "roll")
    want = fe.linear_response(kp, ki, kd, tau_ref, t_f, T4E4.sensor(), maps, sps, stamps, su.divisor)
    assert m.l4_loop(m.l4_law(P4, "roll"), T4E4, su.tick_map(jt, tt), sps, stamps, su.divisor) == want


# ---- (3) the notch coefficient bound ---------------------------------------------------------------------------------

def _r32(x):
    return struct.unpack("<f", struct.pack("<f", x))[0]


def _notch_f32(omega, harmonic, q, num, den):
    """notch_coeffs in binary32 (gyro_chain.hpp update_notches and notch_coeffs), every operation rounded."""
    two_pi = _r32(2 * _r32(math.pi))
    period = _r32(_r32(num / den) / _r32(1.0e6))
    f0 = _r32(_r32(harmonic * _r32(omega)) / two_pi)
    w0 = _r32(_r32(two_pi * f0) * period)
    alpha = _r32(_r32(math.sin(w0)) / _r32(2 * q))
    cw = _r32(math.cos(w0))
    norm = _r32(1 / _r32(1 + alpha))
    b1 = _r32(-_r32(2 * cw) * norm)
    return norm, b1, norm, b1, _r32(_r32(1 - alpha) * norm)


def test_notch_f32_error_bounds_a_float_emulation():
    num, den = P4["tick_period_num_us"], P4["tick_period_den"]
    worst = 0.0
    for st in T4E4.stages[:-1]:
        harmonic = int(st.node[-1])
        q = P4[f"gyro_notch_q_h{harmonic}"]
        f32 = _notch_f32(OMEGA_HOVER, harmonic, q, num, den)
        for c, e, b in zip(st.coeffs, f32, st.err):
            assert abs(e - c) <= b, (st.node, c, e, b)
            worst = max(worst, abs(e - c) / b)
        assert any(abs(e - c) > EMULATION_SHARE * b for c, e, b in zip(st.coeffs, f32, st.err)), st.node
    assert worst > 0


# ---- (4) the L5 noise sums against direct injection -------------------------------------------------------------------

def _l5_direct(fb, s, t):
    su = ao.Setup(P5)
    lin = m.L5Lin(su, 0, T4E5, s, t)
    per_obs = su.divisor * su.ratio
    period = m.l5_period_ticks(su)
    n_run = (2 * period - 1 + per_obs * WINDOW) // per_obs + 1
    zero = [0.0] * lin.size

    def run(at):
        return [o[0] for o in lin.run(0, n_run, zero, 0.0, fb, kind="gyro", at=at, seed=True)[0]]

    resp = [run(i) for i in range(per_obs * WINDOW + 1)]
    classes = [run(period + k) for k in range(period)]
    points = tuple(range(WINDOW + 1))
    return su, per_obs, period, points, resp, classes


def test_l5_noise_formula_equals_direct_injection():
    for fb_index in (0, 1):
        su = ao.Setup(P5)
        fb = m.l5_fb_ends(su, 0)[fb_index]
        su, per_obs, period, points, resp, classes = _l5_direct(fb, *CORNER)
        assert c4._class_runs_are_shifts(resp, classes, period, per_obs)
        direct = c4.direct_sums(resp, points, per_obs)
        vw, vr = m.l5_unit_sums(su, 0, T4E5, *CORNER, fb, WINDOW + 1)["theta"]
        assert c4._worst(c4._within(vw, vr, direct)) <= 1, fb
        swapped = c4._phase_swapped(classes, period, per_obs)
        vw, vr = nt.unit_sums(resp[0], swapped, period, period, per_obs, points)
        assert c4._worst(c4._within(vw, vr, direct)) > 1, f"control: the dt phases swapped pass (fb {fb})"
        vw, vr = nt.unit_sums([0.0] * len(resp[0]), classes, period, period, per_obs, points)
        assert c4._worst(c4._within(vw, vr, direct)) > 1, f"control: the seed path dropped passes (fb {fb})"
