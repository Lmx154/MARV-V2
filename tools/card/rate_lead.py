#!/usr/bin/env python3
"""L6 stage (c) rate-loop gain rule: PI x lead on the exact discrete loop with the gyro chain and the IMU latency (quad
spec L6 stage (c); decisions 0009 D1-D5, 0014 "the design" and owner decisions 3, 6 and 8).

A tool, and the source of the product rate parameters: flatten.py --out-rate-lead calls design() and entries_from()
(decision 0014, commit 3). Plain Python double (math, cmath), no numpy, like rate.py. Imported, not copied: rate.py (plant constants,
spacing a, the band-box corner list, the sup rule's scan and bisection constants, r32, r32_up), gyro_chain_design.py (the
exact discrete loop Loop.f, crossovers, the chain response and stages), gyro_chain_params.py (the flown chain's parameters
from the committed card, budget, register and profile), gen_imu_config.py (sigma_d), mixer.py (M) and tools/sim's
prim_constants (the DShot throttle range).

Rule (every number comes from the card, the budget, the scenario register or the sensor profile; the method constants
below set resolution and termination only)

  1. Loop model: gyro_chain_design's exact discrete loop at tick T_s and rate-loop period T = D T_s, with the flown chain
     (gyro_chain_params.design) at its design point, every notch at omega_th on all four motors (decision 0014,
     "Feasibility"), and the profile's latency (1 sample). Step 13 asserts the result over the whole configuration set
     instead of assuming that this point is the worst. Corners: rate.corner_list (nominal and J -+ b_J x tau -+ b_tau,
     each axis a SISO loop with J normalised to 1, rate.py's per-axis convention).
  2. Controller: the firmware law (rate_loop.hpp), deferred forward-Euler I, D on the measurement by backward difference
     through the first-order D low-pass discretised like the prefilter (decision 0009 D2):
         C(z) = kp + ki T/(z - 1) + kd alpha (z - 1)/(T (z - beta)),  alpha = 1 - e^(-T/T_f), beta = e^(-T/T_f).
     That is the feedback path; the D term acts on y only, so the loop L = C P is the same as for D on the error.
  3. Prototype C(s) = K (1 + w_i/s)(1 + s/w_z)/(1 + s/w_p), w_i = w/a (a = rate.spacing(PM_min), 2.414 at 45 deg),
     w_z = w/sqrt(N), w_p = w sqrt(N). Parallel gains (matching s^2, s and s^0 of the numerators over s (1 + s/w_p)):
         kp = K (1 + w_i/w_z - w_i/w_p),  ki = K w_i,  kd = K/w_z - kp/w_p,  T_f = 1/w_p (the D low-pass is the lead pole).
     kd = (K (N - 1)/w)(1/sqrt(N) - 1/(a N)) >= 0, and kd = 0 exactly at N = 1 (the rule is rate.py's PI there).
     K from |L_nom(e^{jwT})| = 1 on the exact loop of step 1 (nominal corner). Gains are J-normalised (kappa, J = 1);
     axis a flies J_a kappa (decision 0009 D5: the same rule on every axis), T_f is J-independent.
  4. w_c(N): rate.py's sup rule (rate.py step 5) on this loop: geometric scan from pi/T 2^SCAN_LOG2_START in steps of
     2^(1/SCAN_STEPS_PER_OCTAVE) to the first point whose worst-corner PM (gyro_chain_design.crossovers, the minimum over
     a corner's crossovers; no crossover or an unwrapping failure is infeasible) is below PM_min, then bisection until the
     f32 parameters (per axis f32(J kp), f32(J ki), f32(J kd); f32(T_f) when kd != 0) of the midpoint equal those of a
     bracket end. The feasible end is taken.
  5. Ms(N): the largest |1/(1 + L)| over the corners, on gyro_chain_design's grid, refined by golden section between the
     neighbours of the grid maximum.
  6. Noise (owner decision 8): RMS per motor of the D path plus the omega x J omega feed-forward path, in N, from the
     gyro's white noise (decision 0012). Steps, in the order the signal takes them:
       a. sigma_d = N_gyro sqrt(f_s/2) per tick sample and axis (gen_imu_config.derived_report), independent axes;
       b. the chain at the operating point's rotor speeds: tick-rate PSD sigma_d^2 |H(e^{j theta_s})|^2 (normalised so
          the variance is (1/2 pi) times its integral over (-pi, pi]); the latency is a pure delay and drops out;
       c. decimation by D (the rate loop reads every D-th chain output): PSD sigma_d^2 A(theta),
          A(theta) = (1/D) sum_{k<D} |H(e^{j (theta + 2 pi k)/D})|^2;
       d. torque on axis b from gyro axis a: T_ba = -kd_b G_D(z) delta_ab + G0_ba (1 + tau_m G_F(z)), with
          G_D = alpha (z - 1)/(T (z - beta)) (step 2's D path without kd), G_F the same with T_ff (the lag compensation's
          first-order derivative of the chain output), tau_m the card's motor lag, and G0 = d(w x J w)/dw at the
          operating w0 = [w0]x J - [J w0]x (the feed-forward term w x J w + tau_m d/dt(w x J w) linearised; d/dt(w x J w)
          = G0 w-dot, and the term in w-dot0 vanishes at the steady operating point). J is the card's diagonal (0009 F3);
       e. thrust of motor i: f_i = sum_b M[i, b] tau_b (mixer.mixer_matrix, columns roll, pitch, yaw), so
          f_i = sum_a W_ia y_a, W_ia = -m_ia kd_a G_D + g_ia (1 + tau_m G_F), m_ia = M[i, a], g_ia = sum_b M[i, b] G0_ba;
       f. variance_i = sigma_d^2 (1/pi) int_0^pi A sum_a |W_ia|^2 dtheta (real filters: the integrand is even), by the
          midpoint rule on NOISE_POINTS points (spectrally accurate for this smooth periodic integrand; the report gives
          the change on doubling). Expanded: sum_a |W_ia|^2 = |G_D|^2 P_i + |1 + tau_m G_F|^2 Q_i
          - 2 Re(G_D conj(1 + tau_m G_F)) R_i with P_i = sum_a (m_ia kd_a)^2, Q_i = sum_a g_ia^2, R_i = sum_a m_ia kd_a g_ia.
          The D and FF paths share the noise, so the combined RMS is this coherent sum, not a root-sum-square.
     Operating points. Hover: w0 = 0 (G0 = 0, no FF noise), every rotor at omega_hover (standard gravity, as
     gyro_chain_design's omega_hover). Rate_max: |w0_a| = rate_max_a on every axis, each of the 8 sign patterns (the cross
     term R_i changes sign with w0); the chain with every notch bypassed. This is an upper bound, chosen because the rotor
     speeds at rate_max are undefined: the steady tumble's thrusts f = M [m g, w0 x J w0] at the hover collective need one
     motor below f_min (on the committed card -0.301 N; the report prints them), so no rotor-speed operating point exists
     to place the notches at. The bound holds at any rotor speeds, because each notch has |H| <= 1 at every frequency. The
     value at rate_max is the largest over the sign patterns and motors.
     Two evaluations of steps d-f: the design model (steps 4-7), LTI at T with alpha in double; and the firmware model
     (steps 8-9), the 2-periodic law of step 8 with the per-phase response W_ia^(p) in place of W_ia and the larger of the
     two phase variances (y_n = Y_p z^n for the input z^n at phase p = n mod 2, so a phase's variance is the integral of
     |Y_p|^2 A).
     Unit: the hover DShot step's thrust (decisions 0006 and 0013, step_cause.py quantum): per motor f_h = M[i, thrust] m g,
     omega_h = sqrt(f_h/k), D* = d_min + (omega_h - omega_min)(d_max - d_min)/(omega_max - omega_min) rounded,
     dw = (omega_max - omega_min)/(d_max - d_min), w_q = omega_min + dw (D - d_min), dT = k((w_q + dw)^2 - w_q^2); the
     smallest over motors. Budget = d_path_noise_budget x dT.
  7. N*: the largest N with worst-corner Ms <= Ms_max and noise floor <= budget, where the noise floor is the combined
     noise at rate_max with the lag term off (tau_m G_F = 0: the D path plus the plain FF term, the part no T_ff can
     remove). Ordering (decision recorded here, owner decision 8 couples N and T_ff through one budget): N* first, on the
     floor; then T_ff (step 9) spends what remains. The loop's margins and crossover take priority over the feed-forward,
     the coupling is one way (T_ff never moves N*), and a T_ff exists by construction whenever N* does (the combined noise
     falls to the floor as T_ff grows). Search: N_k = 2^(k/N_SCAN_STEPS_PER_OCTAVE) from N = 1 to the first infeasible
     point, N_MONOTONE_EXTRA more grid points beyond it, Ms and the floor asserted non-decreasing over the whole grid (the
     rule assumes it), then geometric bisection until hi/lo <= 1 + N_REL_STEP; N* = lo. The evaluations run in --procs
     worker processes; the result is the serial one for any count (n_search).
  8. f32 guard (owner decision 6: the floors are the margin, no second margin): worst PM >= PM_min and worst Ms <= Ms_max
     on every axis's normalised loop with the coefficients the firmware computes at run time (rate_loop.hpp step()):
       - the per-axis f32 kp, ki, kd and T_f;
       - dt_n = float(dt_us)/float(1e6) per execution, with dt_us the stamp difference of the SIL (hal_sim stamp_us,
         fw/hal/sim/include/marv/hal_sim/hal_sim.hpp:21-23): at 625/4 us and D = 2 the executions alternate 312 and 313 us
         (execution_dts; a pattern longer than 2 is refused);
       - alpha_n = 1 - exp(-dt_n/T_f) in float: the division and the subtraction rounded to f32 (computing in double then
         rounding is exact for one operation), exp correctly rounded and then moved by -LIBM_EXP_ULPS, 0, +LIBM_EXP_ULPS f32
         steps (the target libm's expf, INFERRED within that bound); every variant must pass. 1 - exp cancels, so the f32
         alpha carries a relative error near 4e-6 per half ulp: it moves PM by about 2e-7 rad at N*, above the f32 gain
         rounding;
       - the law I_n = I_(n-1) + ki dt_n e_(n-1), Df_n = (1 - alpha_n) Df_(n-1) + alpha_n kd (y_n - y_(n-1))/dt_n (the
         float rounding of the products is signal noise, not a coefficient).
     The alternating dt makes the law 2-periodic. Its exact frequency analysis: the per-phase responses Y_p (closed forms
     periodic_integral, periodic_derivative) give the harmonic transfer K = [[a(z), b(-z)], [b(z), a(-z)]], a = (Y_0 + Y_1)/2,
     b = (Y_0 - Y_1)/2, on the components at z and -z; with the plant diag(G(z), G(-z)) the loop L = K G, and PM and Ms
     are those of the effective loop at z with the mirror loop closed, L_eff = L00 - L01 L10/(1 + L11) (1/(1 + L_eff) is
     the z-to-z entry of (I + L)^-1). Freezing dt at 312 or 313 us is not a bound: a frozen loop misreads every step by
     0.16 % and moves PM by 6e-4 rad, while the alternation averages out (the report prints both effects).
     A candidate that fails is replaced by the previous feasible point of w_c(N*)'s scan-and-bisection history, then by
     those of the next lower feasible N. None passes: refused.
  9. T_ff: the smallest T_ff >= 0 with combined noise at rate_max <= budget at the guarded f32 design, on the firmware
     model (step 6: f32 T_ff, the dt pattern, the f32 alpha with exp moved by -LIBM_EXP_ULPS, the largest alpha; every libm
     variant is checked at the result). 0 is the unfiltered backward difference. Doubling from T to the first feasible point (refused past T 2^T_FF_LOG2_MAX), the combined noise
     asserted non-increasing over the doubling grid and T_FF_MONOTONE_EXTRA doublings beyond, bisection to relative
     T_FF_REL_TOL; the feasible end, rounded up to f32 (rate.r32_up: more filtering, less noise).
 10. Checks: w_c^2 T tau_lo < 1/2 (rate.py step 7) with w_c the (J-, tau-) corner's largest crossover; QF-8: the verdict is
     UNKNOWN while the card's motor tau sigma is UNKNOWN (rate_qf8.py records the same; the chain's low-pass rule refuses
     D < 2, so the faster rate of the QF-8 curve is not designable with the chain either).
 11. tau_cl: t63 of the closed loop per corner, the first rate-loop sample time n T at which the true body rate reaches
     1 - e^-1 after a unit rate step at n = 0 (no prefilter; D on the measurement, so no setpoint kick), simulated at the
     tick: the exact ZOH motor-lag plant (rate.py rise_time's update at T_s), the gyro sample delayed by the latency, the
     chain (step 1's operating point, direct form II transposed), the firmware law every D ticks, u held D ticks; double
     gains. With H = 1, latency 0 and kd = 0 this is rate.py's rise_time.
 12. Physical J corners (owner decision 3): the vertices of {J : (1 - b_J) J0_i <= J_i <= (1 + b_J) J0_i, J_i <= J_j + J_k}
     (the band box intersected with the triangle inequalities), by enumerating every triple of the 9 bounding planes.
     The box vertices that survive (including the common-scale corners (1 -+ b_J) J0 when J0 is physical) and the new
     vertices where a triangle plane cuts the box are both returned, each marked. For the feed-forward evaluation; the
     loop margins keep rate.py's per-axis corners.
 13. The configuration set (owner decision 1 of decision 0014's third round: "Assert the rate loop's PM and Ms over the
     same configuration set, rather than relying on 'omega_th is the worst case'"): every chain configuration the firmware
     flies or is tested in. The notches of all four motors at one rotor speed omega in [omega_th, omega_max] (omega_max the
     card's speed_range maximum; below omega_th the firmware bypasses a notch), and every notch bypassed (the chain its
     low-pass: no rotor speed, the stage (c) T4 configuration); each at the profile's latency (flight) and at latency 0 (the
     truth gyro of T4 and the T3 harness). At every configuration, step 8's evaluation of the chosen f32 design (the
     firmware loop, the dt pattern, the f32 alpha, expf -+LIBM_EXP_ULPS) at nominal and the four corners must give worst
     PM >= PM_min and worst Ms <= Ms_max; otherwise the card is refused (the design is not changed: a failure goes to the
     owner). Notch grid (core 7.5): level L is omega_i = omega_th (omega_max/omega_th)^(i/2^L), i = 0..2^L, the two ends
     exact, so level 0 is the two ends and every level contains the one before. Level L is resolved when one halving of
     the log step (level L + 1) leaves both the worst PM and the worst Ms over the set unchanged: no added speed is worse
     than the worst of level L, the strictest form of 7.5's rule, with no tolerance to choose. L rises from 0 until
     resolved (refused past NOTCH_GRID_HALVINGS_MAX); the floors are asserted on every configuration evaluated (level
     L + 1 and the bypassed ones). The configurations run in --procs worker processes; each evaluation is a function of
     its configuration alone and the results come back in input order, so the result is the serial one for any count.
     The four motors share the speed, as in step 1; motors at different speeds are not swept.

    uv run python tools/card/rate_lead.py --report
"""

from __future__ import annotations

import argparse
import cmath
import collections
import itertools
import math
import multiprocessing
import struct
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[0] / "sim"))
import cpu_quota  # noqa: E402
import gen_imu_config as gic  # noqa: E402
import gen_plant_config as gpc  # noqa: E402
import gyro_chain_design as gcd  # noqa: E402
import gyro_chain_params as gcp  # noqa: E402
import mixer  # noqa: E402
import prim_constants  # noqa: E402
import rate  # noqa: E402
import schema  # noqa: E402

ROOT = HERE.parents[1]
AXES = rate.AXES
MOTORS = gcd.MOTORS

# Method constants of the numerical search, not vehicle or design numbers: resolution and termination only.
N_SCAN_STEPS_PER_OCTAVE = 4       # N grid 2^(k/4)
N_SCAN_LOG2_MAX = 10              # no bracket below N = 2^10: refused
N_MONOTONE_EXTRA = 4              # grid points past the bracket (one octave of N) in the monotonicity check
N_REL_STEP = 1e-3                 # the labelled step: N bisection stops at hi/lo <= 1 + N_REL_STEP
T_FF_LOG2_MAX = 24                # no T_ff below T 2^24: refused
T_FF_MONOTONE_EXTRA = 4           # doublings past the bracket in the monotonicity check
T_FF_REL_TOL = 1e-6               # T_ff bisection stops at (hi - lo) <= T_FF_REL_TOL hi
NOISE_POINTS = 8192               # midpoint rule over (0, pi) at the rate loop
GOLDEN_ITERATIONS = 80            # golden-section refinement of the Ms peak (shrinks the bracket by 0.618^80)
MAX_STEP_TICKS = 2 ** 24          # step-response simulation cap
CORNER_TOL_ULPS = 8               # triangle-inequality and duplicate tolerance, in units of eps x sum(J0)
# The target libm's expf error, in f32 steps: the bound the chain's coefficient check assumes for float libm calls
# (tests/regression/quad/L06/gyro_chain/support.hpp kLibmUlps, INFERRED from glibc's documented float maxima).
LIBM_EXP_ULPS = 2
MICRO_PER_UNIT = 1e6              # SI prefix: microseconds per second (fw prim::kMicrosecondsPerSecond)
NOTCH_GRID_HALVINGS_MAX = 6       # no resolved notch grid by level 6 (65 speeds): refused (rule step 13)

SIGN_PATTERNS = tuple(itertools.product((1, -1), repeat=3))  # the signs of w0 at rate_max (rule step 6)


def refuse(card_path, reason):
    gpc.refuse(card_path, "rate_lead", f"{reason} (decision 0014)")


# ---- the controller and the loop -------------------------------------------------------------------------------------


def prototype_gains(omega, n, a):
    """(kp, ki, kd, T_f) of the prototype with K = 1 (rule step 3)."""
    wi, wz, wp = omega / a, omega / math.sqrt(n), omega * math.sqrt(n)
    kp = 1 + wi * (1 / wz - 1 / wp)
    return kp, wi, 1 / wz - kp / wp, 1 / wp


def lowpass_pole(t, tf):
    """(alpha, beta) of the first-order low-pass discretised like the prefilter: alpha = 1 - e^(-T/T_f), beta = 1 - alpha;
    T_f = 0 is the unfiltered difference (alpha 1, beta 0)."""
    if tf == 0:
        return 1.0, 0.0
    return -math.expm1(-t / tf), math.exp(-t / tf)


def derivative(z, t, alpha, beta):
    """alpha (z - 1)/(T (z - beta)): backward difference through the first-order low-pass."""
    return alpha * (z - 1) / (t * (z - beta))


# ---- the firmware's run-time coefficients (rule step 8) -------------------------------------------------------------


def stamp_us(num, den, n):
    """hal_sim::stamp_us (fw/hal/sim/include/marv/hal_sim/hal_sim.hpp:21-23): the SIL's integer stamp of tick n."""
    return (n // den) * num + ((n % den) * num) // den


def execution_dts(num, den, divisor):
    """The rate executions' dt_us pattern over its minimal period (the stamps repeat with D num every den executions);
    None when that period exceeds 2."""
    seq = [stamp_us(num, den, (m + 1) * divisor) - stamp_us(num, den, m * divisor) for m in range(2 * den)]
    for period in (1, 2):
        if all(seq[i] == seq[i % period] for i in range(len(seq))):
            return seq[:period]
    return None


def f32_step(x, k):
    """The positive float32 value x moved by k float32 steps."""
    bits = struct.unpack("<I", struct.pack("<f", x))[0] + k
    return struct.unpack("<f", struct.pack("<I", bits))[0]


def f32_alpha(dt32, tf32, ulps=0):
    """T(1) - exp(-dt / tau) in float (rate_loop.hpp step()), exp correctly rounded then moved by `ulps` f32 steps; tau = 0
    is the unfiltered path, alpha 1."""
    if tf32 == 0:
        return 1.0
    return rate.r32(1 - f32_step(rate.r32(math.exp(rate.r32(-dt32 / tf32))), ulps))


def firmware_phases(dts_us, tau, ulps):
    """[(dt_n, alpha_n)] per phase: dt_n = float(dt_us) / float(1e6) and alpha_n at the f32 tau."""
    tau32 = rate.r32(tau)
    out = []
    for d in dts_us:
        dt = rate.r32(rate.r32(d) / rate.r32(MICRO_PER_UNIT))
        out.append((dt, f32_alpha(dt, tau32, ulps)))
    return out


def periodic_derivative(z, phases):
    """(X_0, X_1): the steady-state response x_n = X_(n mod 2) z^n of x_n = (1 - alpha_n) x_(n-1) + (alpha_n/dt_n)(y_n - y_(n-1))
    to y_n = z^n; phases [(dt, alpha)] for even and odd n (one entry: constant). From X_1 = p_1 X_0/z + q_1 and
    X_0 = p_0 X_1/z + q_0, p = 1 - alpha, q = (alpha/dt)(1 - 1/z)."""
    zi = 1 / z
    v = 1 - zi
    if len(phases) == 1:
        dt, al = phases[0]
        x = al / dt * v / (1 - (1 - al) * zi)
        return x, x
    (d0, a0), (d1, a1) = phases
    p0, p1, q0, q1 = 1 - a0, 1 - a1, a0 / d0 * v, a1 / d1 * v
    x0 = (q0 + p0 * q1 * zi) / (1 - p0 * p1 * zi * zi)
    return x0, p1 * zi * x0 + q1


def periodic_integral(z, dts):
    """(J_0, J_1): the per-phase response of I_n = I_(n-1) + dt_n e_(n-1) to e_n = z^n (dts for even and odd n). From
    J_1 = (J_0 + dt_1)/z and J_0 = (J_1 + dt_0)/z."""
    zi = 1 / z
    if len(dts) == 1:
        j = dts[0] * zi / (1 - zi)
        return j, j
    d0, d1 = dts
    j0 = (d1 * zi * zi + d0 * zi) / (1 - zi * zi)
    return j0, zi * (j0 + d1)


_Z = {th: cmath.exp(1j * th) for th in gcd._THETA}   # e^{j theta} on gyro_chain_design's grid


class LeadLoop(gcd.Loop):
    """gyro_chain_design's exact discrete loop with the firmware's PID law in place of its PI (rule step 2). gains = the
    normalised (kp, ki, kd, T_f); jt the corner's true inertia; `cache` maps a grid theta to the plant part (1/D) sum_k F."""

    def __init__(self, gains, jt, model, tau, cache=None):
        kp, ki, kd, tf = gains
        super().__init__(kp / jt, ki / jt, model.t_s, model.divisor, tau, model.h, model.latency)
        self.kd = kd / jt
        self.alpha, self.beta = lowpass_pole(self.t, tf) if kd else (0.0, 0.0)
        self.cache = cache or {}

    def plant(self, theta):
        v = self.cache.get(theta)
        if v is None:
            v = sum(self.f((theta + 2 * math.pi * k) / self.divisor) for k in range(self.divisor)) / self.divisor
        return v

    def controller(self, z):
        c = self.kp + self.ki * self.t / (z - 1)
        if self.kd:
            c += self.kd * derivative(z, self.t, self.alpha, self.beta)
        return c

    def __call__(self, theta):
        # controller(z) * plant(theta) inlined, the same operations in the same order (the hot path of the sup scan).
        z = _Z.get(theta)
        if z is None:
            z = cmath.exp(1j * theta)
        v = self.cache.get(theta)
        if v is None:
            v = self.plant(theta)
        c = self.kp + self.ki * self.t / (z - 1)
        if self.kd:
            c += self.kd * (self.alpha * (z - 1) / (self.t * (z - self.beta)))
        return c * v


class FirmwareLoop(LeadLoop):
    """Rule step 8's loop: L_eff of the 2-periodic firmware law with per-phase (dt, alpha) `phases`, normalised gains."""

    def __init__(self, gains, jt, model, tau, phases):
        super().__init__(gains, jt, model, tau, model.plant_cache(tau))
        self.phases, self.mirror = phases, model.mirror_cache(tau)

    def harmonic(self, z):
        """(a, b) of the controller (input y, output -u): Y_p = kp + ki J_p + kd X_p, a = (Y_0 + Y_1)/2, b = (Y_0 - Y_1)/2."""
        j0, j1 = periodic_integral(z, [ph[0] for ph in self.phases])
        x0, x1 = periodic_derivative(z, self.phases)
        y0, y1 = self.kp + self.ki * j0 + self.kd * x0, self.kp + self.ki * j1 + self.kd * x1
        return (y0 + y1) / 2, (y0 - y1) / 2

    def __call__(self, theta):
        z = cmath.exp(1j * theta)
        a, b = self.harmonic(z)
        a2, b2 = self.harmonic(-z)
        g = self.plant(theta)
        g2 = self.mirror.get(theta)
        if g2 is None:
            g2 = self.plant(theta + math.pi)
        l00, l01, l10, l11 = a * g, b2 * g2, b * g, a2 * g2
        return l00 - l01 * l10 / (1 + l11)


class Model:
    """The loop model of rule step 1: tick, divisor, latency, chain stages (None = H = 1), corners, spacing a."""

    def __init__(self, t_s, divisor, latency, stages, corners, a, inertia):
        self.t_s, self.divisor, self.latency, self.stages = t_s, divisor, latency, stages
        self.t = divisor * t_s
        self.h = None if stages is None else (lambda th: gcd.chain_response(stages, th))
        self.corners, self.a, self.inertia = corners, a, inertia
        self._plant, self._mirror = {}, {}

    def plant_cache(self, tau):
        if tau not in self._plant:
            lp = LeadLoop((1.0, 0.0, 0.0, 0.0), 1.0, self, tau)
            self._plant[tau] = {th: lp.plant(th) for th in gcd._THETA}
        return self._plant[tau]

    def mirror_cache(self, tau):
        """theta -> the plant part at theta + pi (the component at -z), on the grid."""
        if tau not in self._mirror:
            lp = LeadLoop((1.0, 0.0, 0.0, 0.0), 1.0, self, tau)
            self._mirror[tau] = {th: lp.plant(th + math.pi) for th in gcd._THETA}
        return self._mirror[tau]

    def loop(self, gains, jt, tau, phases=None):
        """The design loop (phases None) or the firmware loop of rule step 8."""
        if phases is None:
            return LeadLoop(gains, jt, self, tau, self.plant_cache(tau))
        return FirmwareLoop(gains, jt, self, tau, phases)

    def gains(self, omega, n):
        """Normalised (kp, ki, kd, T_f) at crossover omega and lead ratio n, K from |L_nom| = 1 (rule step 3)."""
        proto = prototype_gains(omega, n, self.a)
        _, jt, tau = self.corners[0]
        m = abs(self.loop(proto, jt, tau)(omega * self.t))
        return proto[0] / m, proto[1] / m, proto[2] / m, proto[3]

    def axis_gains(self, gains):
        """Per-axis double (J kp, J ki, J kd, T_f)."""
        kp, ki, kd, tf = gains
        return [(j * kp, j * ki, j * kd, tf) for j in self.inertia]

    def f32_key(self, gains):
        per = self.axis_gains(gains)
        key = tuple((rate.r32(a[0]), rate.r32(a[1]), rate.r32(a[2])) for a in per)
        return key + ((rate.r32(gains[3]),) if gains[2] else ())

    def margins(self, gains, phases=None):
        """[(corner, PM, crossover rad/s of that PM, all crossovers rad/s)]; PM -inf where the loop has no valid crossover."""
        out = []
        for name, jt, tau in self.corners:
            try:
                xs = gcd.crossovers(self.loop(gains, jt, tau, phases))
            except gcd.DesignError:
                xs = []
            if not xs:
                out.append((name, -math.inf, None, []))
                continue
            th, pm = min(xs, key=lambda x: x[1])
            out.append((name, pm, th / self.t, [x[0] / self.t for x in xs]))
        return out

    def sensitivity(self, gains, phases=None):
        """[(corner, Ms, frequency rad/s of the peak)] (rule step 5)."""
        out = []
        grid = gcd._THETA
        for name, jt, tau in self.corners:
            lp = self.loop(gains, jt, tau, phases)

            def s(th, lp=lp):
                return 1 / abs(1 + lp(th))

            vals = [s(th) for th in grid]
            i = max(range(len(grid)), key=vals.__getitem__)
            lo, hi = grid[max(i - 1, 0)], grid[min(i + 1, len(grid) - 1)]
            g = (math.sqrt(5) - 1) / 2
            x1, x2 = hi - g * (hi - lo), lo + g * (hi - lo)
            f1, f2 = s(x1), s(x2)
            for _ in range(GOLDEN_ITERATIONS):
                if f1 >= f2:
                    hi, x2, f2 = x2, x1, f1
                    x1 = hi - g * (hi - lo)
                    f1 = s(x1)
                else:
                    lo, x1, f1 = x1, x2, f2
                    x2 = lo + g * (hi - lo)
                    f2 = s(x2)
            best = max((vals[i], grid[i]), (f1, x1), (f2, x2))
            out.append((name, best[0], best[1] / self.t))
        return out


def worst(rows, key=1, largest=False):
    pick = max if largest else min
    return pick(rows, key=lambda r: r[key])


def sup_crossover(model, n, pm_min):
    """Rule step 4: dict(omega, history, scan_bracket, bisections, width), or None when the scan start already fails."""
    t = model.t
    ratio = 2.0 ** (1 / rate.SCAN_STEPS_PER_OCTAVE)

    def feasible(w):
        return worst(model.margins(model.gains(w, n)))[1] >= pm_min

    omega = math.pi / t * 2.0 ** rate.SCAN_LOG2_START
    if not feasible(omega):
        return None
    history, lo, hi = [], None, None
    while lo is None:
        history.append(omega)
        nxt = omega * ratio
        if nxt * t >= math.pi:
            return None
        if feasible(nxt):
            omega = nxt
        else:
            lo, hi = omega, nxt
    scan_bracket = (lo, hi)
    bisections = 0
    for _ in range(rate.MAX_BISECTIONS):
        mid = (lo + hi) / 2
        if mid <= lo or mid >= hi:
            break
        if model.f32_key(model.gains(mid, n)) in (model.f32_key(model.gains(lo, n)), model.f32_key(model.gains(hi, n))):
            break
        bisections += 1
        if feasible(mid):
            lo = mid
            history.append(mid)
        else:
            hi = mid
    return {"omega": lo, "history": history, "scan_bracket": scan_bracket, "bisections": bisections, "width": hi - lo}


# ---- noise -----------------------------------------------------------------------------------------------------------


def skew(v):
    return [[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]]


def gyroscopic_jacobian(w0, inertia):
    """G0 = d(w x J w)/dw at w0 = [w0]x J - [J w0]x (rule step 6d)."""
    sw, sjw = skew(w0), skew([j * w for j, w in zip(inertia, w0)])
    return [[sw[r][c] * inertia[c] - sjw[r][c] for c in range(3)] for r in range(3)]


def cross(u, v):
    return [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]


class Noise:
    """The noise model of rule step 6 for one card: the integration grid, the chain's aliased power per operating point,
    the mixer and the unit."""

    def __init__(self, chain, m_matrix, inertia, tau_m, sigma_d, points=NOISE_POINTS):
        self.chain, self.m, self.inertia, self.tau_m, self.sigma_d = chain, m_matrix, inertia, tau_m, sigma_d
        self.points = points
        self.t = chain["divisor"] * chain["t_s"]
        self.theta = [math.pi * (i + 0.5) / points for i in range(points)]
        self.z = [cmath.exp(1j * th) for th in self.theta]
        self._power = {}

    def aliased_power(self, speeds):
        """A(theta) on the grid for the chain at rotor speeds `speeds`, None = every notch bypassed (rule step 6c)."""
        key = None if speeds is None else tuple(speeds)
        if key not in self._power:
            c, d = self.chain, self.chain["divisor"]
            st = gcd.chain_stages(c["t_s"], c["f_c"], c["q"], c["omega_th"], [c["omega_th"]] * MOTORS if key is None
                                  else list(speeds), bypass_all=key is None)
            self._power[key] = [sum(abs(gcd.chain_response(st, (th + 2 * math.pi * k) / d)) ** 2 for k in range(d)) / d
                                for th in self.theta]
        return self._power[key]

    def tumble_thrusts(self, w0, mass, g):
        """Motor thrusts of the steady tumble at w0 with the hover collective: f = M [m g, w0 x J w0] (N)."""
        tau = cross(w0, [j * w for j, w in zip(self.inertia, w0)])
        return [row[0] * mass * g + sum(row[1 + b] * tau[b] for b in range(3)) for row in self.m]

    def rms(self, speeds, w0, kd_axes, tf, t_ff, lag=True, sigma_d=None, fw=None):
        """Per-motor RMS (N): dict d, ff, combined, each a list over motors (rule steps 6d-6f). kd_axes are the flown
        per-axis kd; tf the D low-pass; t_ff the FF derivative's time constant; lag False drops tau_m G_F. fw None: the
        design model; fw = (dt_us pattern, libm ulps): the firmware model, the larger of the phase variances."""
        sig = self.sigma_d if sigma_d is None else sigma_d
        a_th = self.aliased_power(speeds)
        if fw is None:
            alpha, beta = lowpass_pole(self.t, tf)
            alpha_f, beta_f = lowpass_pole(self.t, t_ff)

            def responses(z):
                return (derivative(z, self.t, alpha, beta),), (derivative(z, self.t, alpha_f, beta_f),)
        else:
            ph_d, ph_f = firmware_phases(fw[0], tf, fw[1]), firmware_phases(fw[0], t_ff, fw[1])

            def responses(z):
                return periodic_derivative(z, ph_d), periodic_derivative(z, ph_f)
        lag_gain = self.tau_m if lag else 0.0
        phases = len(responses(self.z[0])[0])
        i_dd, i_ff, i_df = [0.0] * phases, [0.0] * phases, [0.0] * phases
        for z, a in zip(self.z, a_th):
            gds, gfs = responses(z)
            for p in range(phases):
                gd, gf = gds[p], 1 + lag_gain * gfs[p]
                i_dd[p] += a * abs(gd) ** 2
                i_ff[p] += a * abs(gf) ** 2
                i_df[p] += a * (gd * gf.conjugate()).real
        g0 = gyroscopic_jacobian(w0, self.inertia)
        out = {"d": [], "ff": [], "combined": []}
        for row in self.m:
            mi = [row[1 + a] for a in range(3)]
            gi = [sum(row[1 + b] * g0[b][a] for b in range(3)) for a in range(3)]
            p_ = sum((mi[a] * kd_axes[a]) ** 2 for a in range(3))
            q_ = sum(gi[a] ** 2 for a in range(3))
            r_ = sum(mi[a] * kd_axes[a] * gi[a] for a in range(3))
            per = [(i_dd[p] / self.points, i_ff[p] / self.points, i_df[p] / self.points) for p in range(phases)]
            out["d"].append(sig * math.sqrt(max(dd * p_ for dd, _, _ in per)))
            out["ff"].append(sig * math.sqrt(max(ff * q_ for _, ff, _ in per)))
            out["combined"].append(sig * math.sqrt(max(max(dd * p_ + ff * q_ - 2 * df * r_, 0.0) for dd, ff, df in per)))
        return out


def dshot_step(m_matrix, mass, k, omega_min, omega_max):
    """(dT, per-motor detail) of the hover DShot step (rule step 6, unit)."""
    c = prim_constants.load()
    dmin, dmax = c["kDshotThrottleMin"], c["kDshotThrottleMax"]
    dw = (omega_max - omega_min) / (dmax - dmin)
    rows = []
    for row in m_matrix:
        f = row[0] * mass * gic.G
        w = math.sqrt(f / k)
        d_star = dmin + (w - omega_min) * (dmax - dmin) / (omega_max - omega_min)
        d0 = round(d_star)
        wq = omega_min + dw * (d0 - dmin)
        rows.append({"f": f, "omega": w, "d_star": d_star, "d": d0, "dT": k * ((wq + dw) ** 2 - wq ** 2)})
    return min(r["dT"] for r in rows), dw, rows


# ---- the step response -----------------------------------------------------------------------------------------------


def rise_time(model, gains, jt, tau):
    """t63 of rule step 11 (None when not reached within MAX_STEP_TICKS)."""
    kp, ki, kd, tf = gains[0] / jt, gains[1] / jt, gains[2] / jt, gains[3]
    t, t_s, d = model.t, model.t_s, model.divisor
    e, one_minus_e, _, _ = rate.plant_constants(tau, t_s)
    alpha, _ = lowpass_pole(t, tf) if kd else (0.0, 0.0)
    target = 1 - math.exp(-1)
    stages = model.stages or []
    state = [[0.0, 0.0] for _ in stages]
    delay = [0.0] * model.latency
    w = m = u = integ = e_prev = y_prev = dfilt = 0.0
    for tick in range(MAX_STEP_TICKS):
        sample = w
        if delay:
            delay.append(w)
            sample = delay.pop(0)
        y = sample
        for (b0, b1, b2, a1, a2), s in zip(stages, state):
            x = y
            y = b0 * x + s[0]
            s[0] = b1 * x - a1 * y + s[1]
            s[1] = b2 * x - a2 * y
        if tick % d == 0:
            if w >= target:
                return tick // d * t
            err = 1 - y
            integ += ki * t * e_prev
            dfilt += alpha * (-kd * (y - y_prev) / t - dfilt)
            u = kp * err + integ + dfilt
            e_prev, y_prev = err, y
        w, m = w + t_s * u + (m - u) * tau * one_minus_e, e * m + one_minus_e * u
    return None


# ---- the physical J corners ------------------------------------------------------------------------------------------


def _solve3(a, b):
    det = (a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
           + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]))
    if det == 0:
        return None
    out = []
    for c in range(3):
        m = [[b[r] if k == c else a[r][k] for k in range(3)] for r in range(3)]
        out.append((m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                    + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])) / det)
    return out


def physical_corners(inertia, band):
    """Rule step 12: [dict(J (tuple), box (a box vertex), common_scale)], sorted. Half-spaces a.x <= c."""
    j0 = [float(j) for j in inertia]
    tol = CORNER_TOL_ULPS * sys.float_info.epsilon * sum(j0)
    lo, hi = [(1 - band) * j for j in j0], [(1 + band) * j for j in j0]
    planes = []
    for i in range(3):
        e = [1.0 if k == i else 0.0 for k in range(3)]
        planes.append((e, hi[i]))
        planes.append(([-x for x in e], -lo[i]))
    for i in range(3):
        planes.append(([1.0 if k == i else -1.0 for k in range(3)], 0.0))
    found = []
    for tri in itertools.combinations(planes, 3):
        x = _solve3([p[0] for p in tri], [p[1] for p in tri])
        if x is None or any(sum(a * v for a, v in zip(p[0], x)) > p[1] + tol for p in planes):
            continue
        if any(all(abs(a - b) <= tol for a, b in zip(x, y)) for y in found):
            continue
        found.append(x)
    out = []
    for x in sorted(found):
        on_box = all(min(abs(v - lo[i]), abs(v - hi[i])) <= tol for i, v in enumerate(x))
        if on_box:
            x = [lo[i] if abs(v - lo[i]) <= tol else hi[i] for i, v in enumerate(x)]
        common = on_box and (x == lo or x == hi)
        out.append({"J": tuple(x), "box": on_box, "common_scale": common})
    return out


def triangle_ok(j, tol=0.0):
    return all(j[i] <= j[(i + 1) % 3] + j[(i + 2) % 3] + tol for i in range(3))


# ---- the rule --------------------------------------------------------------------------------------------------------


def evaluate_n(model, noise, n, pm_min, op_floor):
    """One N of rule step 7: the sup crossover, the margins, Ms and the noise floor. op_floor: [(speeds, w0)]."""
    sup = sup_crossover(model, n, pm_min)
    if sup is None:
        return {"n": n, "sup": None, "ms": math.inf, "floor": math.inf}
    gains = model.gains(sup["omega"], n)
    margins = model.margins(gains)
    sens = model.sensitivity(gains)
    kd_axes = [a[2] for a in model.axis_gains(gains)]
    floor = max(max(noise.rms(sp, w0, kd_axes, gains[3], 0.0, lag=False)["combined"]) for sp, w0 in op_floor)
    return {"n": n, "sup": sup, "gains": gains, "margins": margins, "sens": sens, "pm": worst(margins)[1],
            "ms": worst(sens, largest=True)[1], "floor": floor}


_CONTEXT = {}   # the N search's model, noise model, PM_min and operating points, set before the workers fork


def _evaluate_job(n):
    c = _CONTEXT
    return evaluate_n(c["model"], c["noise"], n, c["pm_min"], c["ops"])


def _grid_n(step):
    return 2.0 ** (step / N_SCAN_STEPS_PER_OCTAVE)


def n_search(evaluate, feasible, procs, card_path):
    """Rule step 7's search: (grid, k_first, lo record, hi record, feasible history, bisection count). evaluate(n) -> record
    with key n (a module-level function: the workers receive it by name); feasible(record) -> bool, in this process.

    The result is the serial search's for any procs, as in attitude_t3_oracle.py (--procs): every evaluation depends on its
    n alone, the workers only evaluate (Pool.map returns the results in input order), and every decision is taken here in
    the serial order on the same n values (the same expressions). Parallel work is speculative and only discarded:
      grid       the serial loop evaluates steps 0, 1, ... up to k_first + N_MONOTONE_EXTRA; here batches of `procs` steps,
                 truncated to that set (the cap 2^N_SCAN_LOG2_MAX refuses where the serial loop would);
      bisection  the serial loop evaluates mid = sqrt(lo hi) and moves lo or hi; here the tree of the next L levels of
                 possible mids (2^L - 1 <= procs points, each node stopping where the serial condition stops) is evaluated,
                 then replayed with the serial decisions."""
    cap = 2.0 ** N_SCAN_LOG2_MAX
    procs = max(1, procs)

    def run(mapper):
        grid, k_first = [], None
        while not (k_first is not None and len(grid) > k_first + N_MONOTONE_EXTRA):
            nxt = len(grid)
            if _grid_n(nxt) > cap:
                refuse(card_path, f"no N up to 2^{N_SCAN_LOG2_MAX} violates Ms_max or the noise budget: no bracket")
            count = procs if k_first is None else k_first + N_MONOTONE_EXTRA + 1 - nxt
            for r in mapper([_grid_n(k) for k in range(nxt, nxt + count) if _grid_n(k) <= cap]):
                grid.append(r)
                if k_first is None and not feasible(r):
                    k_first = len(grid) - 1
            if k_first is not None:
                del grid[k_first + N_MONOTONE_EXTRA + 1:]
        if k_first == 0:
            return grid, 0, None, None, [], 0
        lo, hi = grid[k_first - 1], grid[k_first]
        history, bisections = list(grid[:k_first]), 0
        levels = max(1, (procs + 1).bit_length() - 1)

        def tree(lo_n, hi_n, depth, out):
            if depth == levels or not hi_n / lo_n > 1 + N_REL_STEP:
                return
            mid = math.sqrt(lo_n * hi_n)
            out.append(mid)
            tree(mid, hi_n, depth + 1, out)
            tree(lo_n, mid, depth + 1, out)

        while hi["n"] / lo["n"] > 1 + N_REL_STEP:
            points = []
            tree(lo["n"], hi["n"], 0, points)
            found = {r["n"]: r for r in mapper(points)}
            for _ in range(levels):
                if not hi["n"] / lo["n"] > 1 + N_REL_STEP:
                    break
                mid = found[math.sqrt(lo["n"] * hi["n"])]
                bisections += 1
                if feasible(mid):
                    lo = mid
                    history.append(mid)
                else:
                    hi = mid
        return grid, k_first, lo, hi, history, bisections

    if procs == 1:
        return run(lambda ns: [evaluate(n) for n in ns])
    with multiprocessing.get_context("fork").Pool(procs) as pool:
        return run(lambda ns: pool.map(evaluate, ns, chunksize=1))


def f32_axes(model, gains):
    """Per-axis f32 (kp, ki, kd, T_f) as doubles."""
    return [tuple(rate.r32(x) for x in a) for a in model.axis_gains(gains)]


def guard(model, axes32, dts_us, stop=None):
    """Rule step 8 evaluation: per axis (worst PM, its corner, worst Ms, its corner) of the firmware loop over the libm
    variants (the largest alpha first). stop = (PM_min, Ms_max) returns at the first failing variant."""
    out = []
    for j, (kp, ki, kd, tf) in zip(model.inertia, axes32):
        g = (kp / j, ki / j, kd / j, tf)
        pm, ms = (math.inf, None), (-math.inf, None)
        for ulps in (-LIBM_EXP_ULPS, 0, LIBM_EXP_ULPS):
            ph = firmware_phases(dts_us, tf, ulps)
            p, m = worst(model.margins(g, ph)), worst(model.sensitivity(g, ph), largest=True)
            pm, ms = min(pm, (p[1], p[0])), max(ms, (m[1], m[0]))
            if stop and not (pm[0] >= stop[0] and ms[0] <= stop[1]):
                return out + [(pm[0], pm[1], ms[0], ms[1])]
        out.append((pm[0], pm[1], ms[0], ms[1]))
    return out


def guard_passes(rows, pm_min, ms_max):
    return all(r[0] >= pm_min and r[2] <= ms_max for r in rows)


def _guard_job(axes32):
    c = _CONTEXT
    return guard(c["model"], axes32, c["dts"], stop=c["stop"])


def first_passing_guard(candidates, procs):
    """(candidate, guard rows) of the first of `candidates` (an iterator of (..., axes32), in the step-down order) whose
    guard passes, or None. The result is the serial step-down's for any procs: each guard depends on its axes32 alone
    (_CONTEXT: model, dts, stop = (PM_min, Ms_max), set before the workers fork), up to `procs` candidates ahead of the
    first undecided one run speculatively on the pool, and the decision is taken here in the step-down order."""
    stop = _CONTEXT["stop"]
    if procs <= 1:
        for cand in candidates:
            rows = _guard_job(cand[-1])
            if guard_passes(rows, *stop):
                return cand, rows
        return None
    with multiprocessing.get_context("fork").Pool(procs) as pool:
        window = collections.deque()
        for cand in itertools.chain(candidates, [None]):
            while len(window) >= procs or (cand is None and window):
                head, job = window.popleft()
                rows = job.get()
                if guard_passes(rows, *stop):
                    return head, rows
            if cand is not None:
                window.append((cand, pool.apply_async(_guard_job, (cand[-1],))))
    return None


# ---- the configuration set (rule step 13) ----------------------------------------------------------------------------


def notch_grid(omega_th, omega_max, level):
    """[(i / 2^level, omega_i)] of rule step 13's notch grid, the two ends exact (i / 2^level is exact in binary, so a level's
    points recur bit for bit in the next)."""
    n = 2 ** level
    return [(i / n, omega_th if i == 0 else omega_max if i == n else omega_th * (omega_max / omega_th) ** (i / n))
            for i in range(n + 1)]


def configuration_set(chain, latency, level):
    """Rule step 13's configurations at notch-grid level `level`: [(key, label, latency, stages)], key = (grid fraction, or
    None with every notch bypassed, latency). Speed-major, each at `latency` then 0, then every notch bypassed; the first is
    step 1's design point (its stages equal design()'s)."""
    lats = (latency, 0) if latency else (0,)
    t_s, f_c, q, th = chain["t_s"], chain["f_c"], chain["q"], chain["omega_th"]
    out = []
    for frac, w in notch_grid(th, chain["omega_max"], level):
        st = gcd.chain_stages(t_s, f_c, q, th, [w] * MOTORS)
        out += [((frac, lat), f"notches at {w:.4f} rad/s, latency {lat}", lat, st) for lat in lats]
    st = gcd.chain_stages(t_s, f_c, q, th, [th] * MOTORS, bypass_all=True)
    return out + [((None, lat), f"notches bypassed, latency {lat}", lat, st) for lat in lats]


def _set_job(cfg):
    c = _CONTEXT
    model = Model(c["t_s"], c["divisor"], cfg[2], cfg[3], c["corners"], c["a"], c["inertia"])
    return guard(model, c["set_axes32"], c["dts"])


def set_worst(rows):
    """((worst PM, label, axis, corner), (worst Ms, label, axis, corner)) over [(key, label, guard rows)]."""
    pm = min((r[0], label, axis, r[1]) for _, label, g in rows for axis, r in zip(AXES, g))
    ms = max((r[2], label, axis, r[3]) for _, label, g in rows for axis, r in zip(AXES, g))
    return pm, ms


def set_passes(rows, pm_min, ms_max):
    return all(guard_passes(g, pm_min, ms_max) for _, _, g in rows)


def configuration_rows(model, chain, axes32, dts, procs, card_path):
    """Rule step 13's evaluation of the f32 design axes32: dict(level (the resolved one), levels [(level, speeds, worst PM,
    worst Ms)], rows [(key, label, guard rows)] over every configuration evaluated, in level L + 1's order)."""
    _CONTEXT.update(t_s=model.t_s, divisor=model.divisor, corners=model.corners, a=model.a, inertia=model.inertia,
                    set_axes32=axes32, dts=dts)
    done, levels, level = {}, [], 0

    def rows_of(cfgs):
        return [(c[0], c[1], done[c[0]]) for c in cfgs]

    def run(mapper):
        nonlocal level
        while True:
            coarse, finer = (configuration_set(chain, model.latency, lv) for lv in (level, level + 1))
            todo = [c for c in finer if c[0] not in done]
            for c, g in zip(todo, mapper(todo)):
                done[c[0]] = g
            if not levels:
                levels.append((level, 2 ** level + 1, *set_worst(rows_of(coarse))))
            levels.append((level + 1, 2 ** (level + 1) + 1, *set_worst(rows_of(finer))))
            if levels[-1][2][0] == levels[-2][2][0] and levels[-1][3][0] == levels[-2][3][0]:
                return rows_of(finer)
            level += 1
            if level > NOTCH_GRID_HALVINGS_MAX:
                refuse(card_path, f"the configuration set's worst PM or Ms still changes past notch-grid level "
                                  f"{NOTCH_GRID_HALVINGS_MAX} (rule step 13)")

    if procs <= 1:
        rows = run(lambda cs: [_set_job(c) for c in cs])
    else:
        with multiprocessing.get_context("fork").Pool(procs) as pool:
            rows = run(lambda cs: pool.map(_set_job, cs, chunksize=1))
    return {"level": level, "levels": levels, "rows": rows}


def design(card, budget, scenario, profile, card_path, profile_path, procs=None):
    """The whole rule: a dict of results (raises gpc.GenError when the card is refused). procs: worker processes of the
    N search and the f32 guard's step-down (default cpu_quota.usable_cpus()); the result does not depend on it (n_search,
    first_passing_guard)."""
    procs = procs or cpu_quota.usable_cpus()
    pi = rate.design(card, budget, scenario, card_path)
    inp = pi["inputs"]
    pm_min, a = inp["PM_min"], pi["a"]
    ms_max = float(rate._value(budget, "Ms_max", "budget entry", card_path))
    noise_budget = float(rate._value(budget, "d_path_noise_budget", "budget entry", card_path))
    rate_max = [float(rate._value(scenario, f"rate_max_{ax}", "scenario entry", card_path)) for ax in AXES]
    chain = gcp.design(card, budget, scenario, profile, profile_path, card_path)
    si, _, _ = gic.imu_config(profile_path)
    num, den = scenario["tick_period_num_us"]["value"], scenario["tick_period_den"]["value"]
    sigma_d = gic.derived_report(si, num, den)[0]["gyro"][4]
    dts = execution_dts(num, den, chain["divisor"])
    if dts is None:
        refuse(card_path, "the rate executions' stamp pattern is longer than 2: the guard's periodic analysis covers 1 or 2")
    corners = rate.corner_list(inp["tau"], inp["b_tau"], inp["b_J"])
    stages = gcd.chain_stages(chain["t_s"], chain["f_c"], chain["q"], chain["omega_th"], [chain["omega_th"]] * MOTORS)
    model = Model(chain["t_s"], chain["divisor"], si["latency_samples"], stages, corners, a, inp["inertia"])

    m_matrix, _, idle = mixer.mixer_matrix(card, card_path)
    rotors = card["rotors"]
    k = float(gpc._known(rotors["thrust_coeff"], "rotors.thrust_coeff", card_path))
    speed = gpc._known(rotors["speed_range"], "rotors.speed_range", card_path)
    mass = float(gpc._known(card["mass"], "mass", card_path))
    unit, dw, dshot_rows = dshot_step(m_matrix, mass, k, float(speed[0]), float(speed[1]))
    budget_n = noise_budget * unit
    noise = Noise(chain, m_matrix, inp["inertia"], inp["tau"], sigma_d)
    f_min, f_max = k * idle * idle, k * float(speed[1]) ** 2
    ops = [(None, [s * r for s, r in zip(signs, rate_max)]) for signs in SIGN_PATTERNS]
    tumble = noise.tumble_thrusts(rate_max, mass, gic.G)
    hover_speeds = [chain["omega_hover"]] * MOTORS

    # Rule step 7: the N grid, its monotonicity, the bisection (in `procs` processes; the result does not depend on it).
    def feasible(r):
        return r["ms"] <= ms_max and r["floor"] <= budget_n

    for _, _, tau in corners:
        model.plant_cache(tau)                  # filled before the fork, so every worker inherits them
    noise.aliased_power(None)
    _CONTEXT.update(model=model, noise=noise, pm_min=pm_min, ops=ops)
    grid, k_first, lo_r, hi_r, n_history, n_bisections = n_search(_evaluate_job, feasible, procs, card_path)
    if k_first == 0:
        refuse(card_path, f"N = 1 (PI) already violates Ms_max or the noise budget: Ms {grid[0]['ms']!r}, noise floor "
                          f"{grid[0]['floor']!r} N against {budget_n!r} N")
    for prev, nxt in zip(grid, grid[1:]):
        if not (nxt["ms"] >= prev["ms"] and nxt["floor"] >= prev["floor"]):
            refuse(card_path, f"Ms or the noise floor is not non-decreasing in N between N = {prev['n']!r} and "
                              f"{nxt['n']!r}: the N* rule assumes it")

    # Rule step 8: the f32 guard, stepping down through w_c(N*)'s history, then through the N history.
    def step_down():
        for cand_n in reversed(n_history):
            for cand_w in reversed(cand_n["sup"]["history"]):
                g = model.gains(cand_w, cand_n["n"])
                yield cand_n, cand_w, g, f32_axes(model, g)

    _CONTEXT.update(model=model, dts=dts, stop=(pm_min, ms_max))
    found = first_passing_guard(step_down(), procs)
    chosen = None if found is None else (*found[0][:3], found[0][3], found[1])
    if chosen is None:
        refuse(card_path, "no feasible design passes the f32 guard (PM >= PM_min, Ms <= Ms_max on the f32 gains)")
    star, omega_c, gains, axes32, guard_rows = chosen
    stepped_down = (star is not lo_r) or (omega_c != star["sup"]["omega"])
    margins, sens = model.margins(gains), model.sensitivity(gains)
    tf32 = axes32[0][3]
    if any(ax[3] != tf32 for ax in axes32):
        refuse(card_path, "the per-axis f32 T_f differ")
    kd32 = [ax[2] for ax in axes32]

    # Rule step 9: T_ff at the guarded f32 design, on the firmware model with the largest alpha.
    fw_max = (dts, -LIBM_EXP_ULPS)

    def combined(t_ff, fw=fw_max):
        return max((max(noise.rms(sp, w0, kd32, tf32, t_ff, fw=fw)["combined"]), sp, w0) for sp, w0 in ops)

    t = model.t
    ff_grid = [(0.0, combined(0.0)[0])]
    if ff_grid[0][1] <= budget_n:
        t_ff = 0.0
    else:
        cand, first = t, None
        while first is None or len(ff_grid) <= first + T_FF_MONOTONE_EXTRA:
            if cand > t * 2.0 ** T_FF_LOG2_MAX:
                refuse(card_path, f"no T_ff up to T 2^{T_FF_LOG2_MAX} meets the noise budget")
            ff_grid.append((cand, combined(cand)[0]))
            if first is None and ff_grid[-1][1] <= budget_n:
                first = len(ff_grid) - 1
            cand *= 2
        for (_, a_), (_, b_) in zip(ff_grid, ff_grid[1:]):
            if not b_ <= a_:
                refuse(card_path, "the combined noise is not non-increasing in T_ff: the T_ff rule assumes it")
        ff_lo, hi = ff_grid[first - 1][0], ff_grid[first][0]
        while hi - ff_lo > T_FF_REL_TOL * hi:
            mid = (ff_lo + hi) / 2
            if combined(mid)[0] <= budget_n:
                hi = mid
            else:
                ff_lo = mid
        t_ff = rate.r32_up(hi)

    def at(speeds, w0, t_ff_, lag=True):
        return noise.rms(speeds, w0, kd32, tf32, t_ff_, lag, fw=fw_max)

    hover = at(hover_speeds, [0.0, 0.0, 0.0], t_ff)
    worst_ff = combined(t_ff)
    at_max = at(worst_ff[1], worst_ff[2], t_ff)
    floor_max = max(max(at(sp, w0, t_ff, lag=False)["combined"]) for sp, w0 in ops)
    variants = [combined(t_ff, (dts, u))[0] for u in (-LIBM_EXP_ULPS, 0, LIBM_EXP_ULPS)]
    if not max(variants) <= budget_n:
        refuse(card_path, f"the combined noise at T_ff {t_ff!r} s is {max(variants)!r} N, above the budget {budget_n!r} N")
    doubled = Noise(chain, m_matrix, inp["inertia"], inp["tau"], sigma_d, 2 * NOISE_POINTS)
    conv = max(doubled.rms(worst_ff[1], worst_ff[2], kd32, tf32, t_ff, fw=fw_max)["combined"]) / worst_ff[0] - 1

    # Rule step 8's two effects at the chosen design, roll axis (report only): the firmware model against the design
    # model, and a frozen dt (one phase) at each pattern value.
    g_roll = tuple(x / inp["inertia"][0] for x in axes32[0][:3]) + (tf32,)
    effects = [("design model (double alpha, T)", worst(model.margins(g_roll))[1] - pm_min),
               ("firmware model (f32, alternating dt)", worst(model.margins(g_roll, firmware_phases(dts, tf32, 0)))[1] - pm_min)]
    for d in dts:
        effects.append((f"dt frozen at {d} us", worst(model.margins(g_roll, firmware_phases([d], tf32, 0)))[1] - pm_min))

    # Rule step 10.
    lo_corner = next(m for m in margins if m[0] == "J-,tau-")
    tau_lo = (1 - inp["b_tau"]) * inp["tau"]
    condition = max(lo_corner[3]) ** 2 * t * tau_lo
    if not condition < 0.5:
        refuse(card_path, f"w_c^2 T tau_lo = {condition!r} is not below 1/2 at the corner (J-, tau-)")
    tau_sigma = card["rotors"]["motor_lag"]["tau"].get("sigma")
    qf8 = ("UNKNOWN: the card's rotors.motor_lag.tau sigma is UNKNOWN (rate_qf8.py records the same verdict)"
           if tau_sigma == schema.UNKNOWN else "open: motor tau sigma known; run the QF-8 curve")

    # Rule step 11.
    t63 = []
    for name, jt, tau in corners:
        r = rise_time(model, gains, jt, tau)
        if r is None:
            refuse(card_path, f"the step response at {name} does not reach 1 - e^-1 within {MAX_STEP_TICKS} ticks")
        t63.append((name, r))

    # Rule step 13: the chosen f32 design over the configuration set (in `procs` processes; the result does not depend on it).
    cset = configuration_rows(model, chain, axes32, dts, procs, card_path)
    if not set_passes(cset["rows"], pm_min, ms_max):
        (pm, pl, pa, pc), (ms, ml, ma, mc) = set_worst(cset["rows"])
        refuse(card_path, f"the f32 design fails a floor over the configuration set (rule step 13): worst PM {pm!r} rad at "
                          f"{pl}, {pa} {pc} (PM_min {pm_min!r}); worst Ms {ms!r} at {ml}, {ma} {mc} (Ms_max {ms_max!r})")

    return {
        "set": cset,
        "inputs": {"inertia": inp["inertia"], "tau": inp["tau"], "PM_min": pm_min, "Ms_max": ms_max, "b_tau": inp["b_tau"],
                   "b_J": inp["b_J"], "noise_budget": noise_budget, "rate_max": rate_max, "sigma_d": sigma_d,
                   "latency": si["latency_samples"], "T": t, "T_s": chain["t_s"], "a": a},
        "chain": chain, "model": model, "noise": noise, "ops": ops, "hover_speeds": hover_speeds,
        "tumble_thrusts": tumble, "f_range": (f_min, f_max),
        "dshot": {"dT": unit, "dw": dw, "rows": dshot_rows}, "budget_n": budget_n,
        "grid": grid, "k_first": k_first, "n_bisections": n_bisections, "n_lo": lo_r, "n_hi": hi_r,
        "n_star": star["n"], "omega_c": omega_c, "sup": star["sup"], "gains": gains, "stepped_down": stepped_down,
        "margins": margins, "sens": sens, "pm_worst": worst(margins), "ms_worst": worst(sens, largest=True),
        "axes32": axes32, "guard": guard_rows, "dts": dts, "effects": effects, "noise_variants": variants,
        "t_ff": t_ff, "ff_grid": ff_grid, "noise_hover": hover, "noise_max": at_max, "noise_max_w0": worst_ff[2],
        "noise_floor_max": floor_max, "noise_convergence": conv,
        "condition": condition, "qf8": qf8, "t63": t63, "tau_cl": max(x[1] for x in t63),
        "pi_with_chain": grid[0], "pi_l4": pi,
        "j_corners": physical_corners(inp["inertia"], inp["b_J"]),
    }


# ---- the report ------------------------------------------------------------------------------------------------------


def report_lines(r):
    deg = math.degrees
    i, c = r["inputs"], r["chain"]
    mn = 1e3  # N to mN, display only
    lines = [
        "MARV L6 stage (c) rate-loop gain rule, PI x lead with the chain (tools/card/rate_lead.py, decision 0014)",
        f"T {i['T']!r} s (T_s {i['T_s']!r} s, D {c['divisor']}), latency {i['latency']} sample(s), a {i['a']!r}, "
        f"PM_min {deg(i['PM_min']):.4f} deg, Ms_max {i['Ms_max']!r}, bands tau +-{i['b_tau']!r} J +-{i['b_J']!r}",
        f"chain (flown ESC {c['esc_clock_error']!r}): f_c {c['f_c']:.4f} Hz, Q {c['q'][0]:.4f} {c['q'][1]:.4f} "
        f"{c['q'][2]:.4f}, omega_th {c['omega_th']:.4f} rad/s (all notches there in the loop), omega_hover "
        f"{c['omega_hover']:.4f} rad/s",
        f"L4 PI without the chain (rate.py): w_c {r['pi_l4']['omega_c']:.6f} rad/s; PI with the chain (N = 1): "
        f"w_c {r['pi_with_chain']['sup']['omega']:.6f} rad/s",
        "N grid (rule step 7): N, w_c,nom rad/s, worst PM deg, worst Ms, noise floor at rate_max mN",
    ]
    for g in r["grid"]:
        if g["sup"] is None:
            lines.append(f"  {g['n']:9.5f}  no feasible crossover")
        else:
            lines.append(f"  {g['n']:9.5f}  {g['sup']['omega']:10.5f}  {deg(g['pm']):8.4f}  {g['ms']:.6f}  "
                         f"{g['floor'] * mn:.6f}")
    pmw, msw = r["pm_worst"], r["ms_worst"]
    kp, ki, kd, tf = r["gains"]
    lines += [
        f"N bisection: {r['n_bisections']} steps, bracket {r['n_lo']['n']!r} .. {r['n_hi']['n']!r} (hi/lo - 1 "
        f"{r['n_hi']['n'] / r['n_lo']['n'] - 1:.3e}); binding: "
        + ("Ms" if r["n_hi"]["ms"] > i["Ms_max"] else "noise"),
        f"N* {r['n_star']!r}",
        f"w_c,nom {r['omega_c']!r} rad/s (scan bracket {r['sup']['scan_bracket'][0]:.6f} .. "
        f"{r['sup']['scan_bracket'][1]:.6f}, {r['sup']['bisections']} bisections, width {r['sup']['width']!r}); "
        f"stepped down by the guard {r['stepped_down']}",
        "margins (double gains): corner, crossover rad/s, PM deg, Ms, Ms peak rad/s",
    ]
    for (name, pm, wx, _), (_, ms, wm) in zip(r["margins"], r["sens"]):
        lines.append(f"  {'*' if name in (pmw[0], msw[0]) else ' '} {name:<8} {wx:10.5f}  {deg(pm):9.5f}  {ms:.6f}  "
                     f"{wm:9.4f}")
    lines += [
        f"worst PM {deg(pmw[1]):.6f} deg at {pmw[0]} (PM - PM_min {pmw[1] - i['PM_min']:.3e} rad); worst Ms "
        f"{msw[1]:.6f} at {msw[0]}",
        f"kappa (J = 1): kp {kp!r}  ki {ki!r}  kd {kd!r}  T_f {tf!r} s (w_z {r['omega_c'] / math.sqrt(r['n_star']):.5f}, "
        f"w_p {1 / tf:.5f} rad/s)",
    ]
    for ax, j, g32, (pm, pmc, ms, msc) in zip(AXES, i["inertia"], r["axes32"], r["guard"]):
        lines.append(f"  {ax:<5} J {j!r}: kp {g32[0]!r}  ki {g32[1]!r}  kd {g32[2]!r}  T_f {g32[3]!r}  (f32); guard: "
                     f"PM {deg(pm):.6f} deg ({pmc}), Ms {ms:.6f} ({msc})")
    lines.append(f"f32 guard (rule step 8: PM >= PM_min and Ms <= Ms_max on the firmware loop, dt pattern {r['dts']} us, "
                 f"f32 alpha, expf -+{LIBM_EXP_ULPS} ulp): "
                 f"{'pass' if guard_passes(r['guard'], i['PM_min'], i['Ms_max']) else 'FAIL'}")
    lines.append("  effects at the chosen design, roll, worst PM - PM_min (rad): "
                 + "; ".join(f"{name} {v:+.3e}" for name, v in r["effects"]))
    cset = r["set"]
    (pm, pl, pa, pc), (ms, ml, ma, mc) = set_worst(cset["rows"])
    lines.append("configuration set (rule step 13: step 8's evaluation at every configuration; notch grid halvings: level, "
                 "speeds, worst PM deg, worst Ms):")
    lines += [f"  level {lv} ({n} speeds): PM {deg(p[0]):.7f} ({p[1]}, {p[2]} {p[3]}), Ms {m[0]:.7f} ({m[1]}, {m[2]} {m[3]})"
              for lv, n, p, m in cset["levels"]]
    lines.append(f"  resolved at level {cset['level']} (level {cset['level'] + 1} leaves both unchanged); per configuration "
                 "evaluated: worst PM deg (axis corner), PM - PM_min rad; worst Ms (axis corner), Ms_max - Ms")
    for _, label, g in cset["rows"]:
        (p, _, a1, c1), (m, _, a2, c2) = set_worst([(None, label, g)])
        lines.append(f"  {label:<40} {deg(p):.7f} ({a1} {c1}) {p - i['PM_min']:+.3e}; {m:.7f} ({a2} {c2}) "
                     f"{i['Ms_max'] - m:+.3e}")
    lines.append(f"  worst over the set: PM {deg(pm):.7f} deg at {pl}, {pa} {pc}; Ms {ms:.7f} at {ml}, {ma} {mc}: "
                 f"{'pass' if set_passes(cset['rows'], i['PM_min'], i['Ms_max']) else 'FAIL'}")
    d = r["dshot"]
    lines += [
        f"noise: sigma_d {i['sigma_d']!r} rad/s per tick sample; hover DShot step: D {d['rows'][0]['d']} (D* "
        f"{d['rows'][0]['d_star']:.4f}), dw {d['dw']!r} rad/s, dT {d['dT'] * mn:.6f} mN; budget {i['noise_budget']!r} x dT "
        f"= {r['budget_n'] * mn:.6f} mN",
        "rate_max operating point: every notch bypassed (bound); the steady tumble at w0 = rate_max on every axis with the "
        "hover collective needs motor thrusts " + " ".join(f"{x:.4f}" for x in r["tumble_thrusts"])
        + f" N against [{r['f_range'][0]:.4f}, {r['f_range'][1]:.4f}] N",
        f"T_ff {r['t_ff']!r} s (rule step 9; combined noise over the doubling grid: "
        + ", ".join(f"{tff * 1e3:.4g} ms {v * mn:.4f} mN" for tff, v in r["ff_grid"]) + ")",
    ]
    for label, nz in (("hover", r["noise_hover"]), (f"rate_max, worst sign pattern w0 {r['noise_max_w0']}", r["noise_max"])):
        lines.append(f"  {label}: per motor mN  D " + " ".join(f"{x * mn:.5f}" for x in nz["d"])
                     + "  FF " + " ".join(f"{x * mn:.5f}" for x in nz["ff"])
                     + "  combined " + " ".join(f"{x * mn:.5f}" for x in nz["combined"]))
    lines += [
        f"  max combined: hover {max(r['noise_hover']['combined']) * mn:.6f} mN, rate_max {max(r['noise_max']['combined']) * mn:.6f} "
        f"mN ({max(r['noise_max']['combined']) / r['budget_n']:.6f} of the budget); floor (lag off) at rate_max "
        f"{r['noise_floor_max'] * mn:.6f} mN; integration points doubled: relative change {r['noise_convergence']:.2e}",
        f"  (firmware model, expf -{LIBM_EXP_ULPS} ulp: the largest alpha) rate_max combined over the expf variants -{LIBM_EXP_ULPS}, 0, +{LIBM_EXP_ULPS} ulp: "
        + " ".join(f"{v * mn:.7f}" for v in r["noise_variants"]) + " mN",
        f"condition w_c^2 T tau_lo {r['condition']!r} (< 1/2, (J-, tau-) crossover)",
        f"QF-8: {r['qf8']}",
        "t63 (s, rule step 11): " + "  ".join(f"{n} {t:.6f}" for n, t in r["t63"]) + f"; tau_cl (max) {r['tau_cl']:.6f}",
        f"physical J corners (rule step 12; band +-{i['b_J']!r}): {len(r['j_corners'])} vertices "
        f"({sum(v['box'] for v in r['j_corners'])} box vertices, {sum(v['common_scale'] for v in r['j_corners'])} common-scale)",
    ]
    for v in r["j_corners"]:
        lines.append(f"  J {v['J'][0]:.6g} {v['J'][1]:.6g} {v['J'][2]:.6g}  "
                     + ("box" if v["box"] else "cut") + (" common-scale" if v["common_scale"] else ""))
    return lines


# ---- the product parameters (decision 0014, commit 3) ------------------------------------------------------------------

GAIN_METHOD = (
    "derived(rate PID by the PI x lead rule of decision 0014 (rule steps 1-12 of tools/card/rate_lead.py): the 0005 PI "
    "structure (integral zero w_c,nom/a, a = sqrt((1 + sin PM_min)/(1 - sin PM_min))) with the derivative-on-measurement "
    "lead at N* = the largest N, on the grid 2^(k/4) refined by bisection, whose worst-case phase margin over the "
    "tau_robustness_band x inertia_robustness_band box stays >= PM_min, whose worst Ms stays <= Ms_max and whose noise floor "
    "at rate_max stays within d_path_noise_budget x the DShot step; w_c,nom the largest such crossover on the design model "
    "(flown gyro chain with every notch at omega_th, the profile's latency, the firmware's forward-Euler PID with the D "
    "low-pass); kp, ki, kd = J x kappa rounded to f32 and stepped down until the f32 guard (PM >= PM_min, Ms <= Ms_max on "
    "the firmware's run-time f32 coefficients, 2-periodic dt, expf +-2 ulp) passes, and that guard's PM >= PM_min and "
    "Ms <= Ms_max asserted over the configuration set (all notches at each rotor speed of a halving-resolved grid from "
    "omega_th to omega_max, and every notch bypassed, each at the profile's latency and at 0; rule step 13); "
    "tools/card/rate_lead.py)")
GAIN_SOURCE = ("vehicle card inertia_diag, rotors.motor_lag.tau, thrust_coeff, speed_range and the mixer M; design-budget "
               "PM_min, Ms_max, d_path_noise_budget, tau_robustness_band, inertia_robustness_band; scenario register "
               "tick_period_num_us, tick_period_den, rate_loop_divisor, rate_max_<axis>; the sensor profile the card names "
               "(the gyro chain, decision 0013); decision 0014")
TF_METHOD = ("derived(T_f = 1/w_p, w_p = w_c,nom sqrt(N*), the pole of the PI x lead rule's prototype (rule step 3, "
             "tools/card/rate_lead.py), rounded to f32 with the gains and stepped down with them by the f32 guard; decision "
             "0014)")
TAU_REF_METHOD = (
    "derived(tau_ref = max(tau_cl, rate_max/alpha_max) of decision 0005 QF-2 (tools/card/rate.py rule step 9) with tau_cl the "
    "largest 63.2 % rise time over the robustness box of the stage (c) closed-loop step response (rate_lead.py rule step "
    "11, the PID with the gyro chain and latency) in place of the PI's; alpha_max = tau_max/(J (1 + "
    "inertia_robustness_band)), tau_max the air-mode envelope torque of the mixer M with the collective free; rounded up "
    "to f32; tools/card/rate_lead.py tau_ref_lead)")
TAU_REF_SOURCE = ("the gain rule's inputs (decision 0014) and the vehicle card rotors thrust_coeff and speed_range, the mixer "
                  "M and the scenario register rate_max_<axis>; decision 0005 QF-2; lead decision 2026-10-01 (decision 0014 "
                  "c5)")
T_FF_METHOD = (
    "derived(T_ff = the smallest T_ff >= 0 whose combined RMS of the D path and the lag-compensated omega x J omega "
    "feed-forward path per motor at rate_max (every sign pattern, every notch bypassed) is at most d_path_noise_budget x "
    "the hover DShot step's thrust, on the firmware model at the guarded f32 design (rule step 9 of "
    "tools/card/rate_lead.py), rounded up to f32; read by the rate loop only while rate_ff_enable is 1; decision 0014, "
    "owner decision 8)")


def tau_ref_lead(result):
    """tau_ref per axis: rate.py rule step 9, r32_up(max(tau_cl, rate_max_a / alpha_max_a)), with this design's tau_cl."""
    pi = result["pi_l4"]
    return [rate.r32_up(max(result["tau_cl"], pi["axes"][ax]["authority_term"])) for ax in AXES]


def entries_from(result):
    """The 16 rate-loop parameters (ordered (name, params_gen entry) pairs) of a design() result, in rate.entries_from's
    order with rate_d_filter_tau_<axis> after rate_kd_<axis>, then rate_ff_filter_tau (T_ff)."""
    ax32, tau_ref = result["axes32"], tau_ref_lead(result)
    out = []
    for column, name, unit in ((0, "kp", "N m s/rad"), (1, "ki", "N m/rad"), (2, "kd", "N m s^2/rad"), (3, "d_filter_tau", "s")):
        for i, axis in enumerate(AXES):
            method = TF_METHOD if column == 3 else GAIN_METHOD
            out.append((f"rate_{name}_{axis}", {"type": "f32", "value": ax32[i][column], "unit": unit, "method": method,
                                                 "source": GAIN_SOURCE, "sigma": schema.UNKNOWN}))
    for i, axis in enumerate(AXES):
        out.append((f"rate_tau_ref_{axis}", {"type": "f32", "value": tau_ref[i], "unit": "s", "method": TAU_REF_METHOD,
                                              "source": TAU_REF_SOURCE, "sigma": schema.UNKNOWN}))
    out.append(("rate_ff_filter_tau", {"type": "f32", "value": result["t_ff"], "unit": "s", "method": T_FF_METHOD,
                                       "source": GAIN_SOURCE, "sigma": schema.UNKNOWN}))
    return out


def load(card_path, budget_path, scenario_path, profile_path):
    card, budget, scenario, profile = (schema.load_yaml(p) for p in (card_path, budget_path, scenario_path, profile_path))
    return card, budget, scenario, profile


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--report", action="store_true", required=True, help="print the derivation report")
    ap.add_argument("--card", default=str(ROOT / "vehicles" / "uzh_neurobem_5in.yaml"))
    ap.add_argument("--budget", default=str(ROOT / "design" / "budget.yaml"))
    ap.add_argument("--scenario", default=str(ROOT / "design" / "scenario_values.yaml"))
    ap.add_argument("--profile", default=str(ROOT / "sensors" / "profiles" / "marv_v2_board_default.yaml"))
    ap.add_argument("--procs", type=int, default=cpu_quota.usable_cpus(),
                    help="worker processes of the N search (the output does not depend on it)")
    args = ap.parse_args(argv)
    try:
        card, budget, scenario, profile = load(args.card, args.budget, args.scenario, args.profile)
        for line in report_lines(design(card, budget, scenario, profile, args.card, args.profile, args.procs)):
            print(line)
    except (gpc.GenError, gcd.DesignError) as e:
        print(e, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
