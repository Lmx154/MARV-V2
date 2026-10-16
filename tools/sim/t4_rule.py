"""The L2 T4 pass rule under SIM-7 (docs/decisions/0003 items 9 and 11), pure Python, no simulator.

Step sequence. H_k = 2^(K-k) t_tick for k = 0..K (K = 2 in the committed scenarios), one run per H_k, q_k the
quantity read from run k. The caller passes q = [q_0, ..., q_K], the reference q_ref, the roundoff floor F and the
reference's own error F_ref (both >= 0, from floor() and from tools/sim/reference.py).

Terms. d_k = q_k - q_{k-1} for k = 1..K, E = |d_K|, rho = d_K / d_{K-1}, p_obs = -log2 rho (the observed order: the
step halves from k-1 to k, so rho = 2^-p for an error c H^p). No integrator order is assumed.

Pass rule. A quantity passes if either holds:
  (i)  E <= F and |q_K - q_ref| <= F + F_ref
  (ii) rho in (0, 1) and |q_hat - q_ref| <= E + F (1 + rho) / (1 - rho) + F_ref,
       q_hat = q_K + d_K rho / (1 - rho)   (Aitken / Richardson extrapolation with the observed order)
Branch (i) is tried first. When d_{K-1} = 0, rho is undefined and (ii) is not available (no division is made); with
K = 1 there is no d_{K-1} either. Non-finite inputs fail. Otherwise the quantity fails.

Roundoff floor. F = (N + extra_roundings) u Q, see floor(). u = 2^-53 is the unit roundoff of IEEE 754 binary64 in
round-to-nearest, Higham (2002) "Accuracy and Stability of Numerical Algorithms" 2nd ed. sec. 2.1, read here from
sys.float_info.mant_dig (53). The N term is the first-order bound of recursive summation of N terms, Higham (2002)
sec. 4.2, |s_hat - s| <= (N - 1) u sum|x_i| + O(u^2), taken as N u Q (one extra u covers the first rounding of the
running sum); Q is sum|x_i|, the magnitude scale the caller states for the quantity. extra_roundings is the count of
roundings in the quantity's own formula, each of relative size <= u, counted by the caller from the formula.

Freshness (item 11). A gz read is rewritten only when the link has moved, so a step's raw read can be stale. The
quantity is evaluated at fresh_index(reads): the last index whose raw read differs bitwise from the previous one. A
scenario fails if there is no fresh read in the last half of the run, or (hover) none after the settling time t_s.
"""

from __future__ import annotations

import math
import struct
import sys
from dataclasses import dataclass, field

U = math.ldexp(1.0, -sys.float_info.mant_dig)  # IEEE 754 binary64 unit roundoff 2^-53, Higham (2002) sec. 2.1
BRANCH_I = "(i)"
BRANCH_II = "(ii)"


def floor(n, q, extra_roundings=0):
    """Roundoff floor F = (N + extra_roundings) u Q, see the module docstring. N >= 0 integer, Q >= 0."""
    if n < 0 or extra_roundings < 0 or q < 0 or not math.isfinite(q):
        raise ValueError("floor needs N >= 0, extra_roundings >= 0 and a finite Q >= 0")
    return (n + extra_roundings) * U * q


@dataclass(frozen=True)
class Verdict:
    passed: bool
    branch: str | None  # BRANCH_I, BRANCH_II, or None when the quantity fails
    d: tuple  # d_1..d_K
    E: float  # |d_K|
    rho: float | None  # d_K / d_{K-1}; None when d_{K-1} = 0 or K = 1
    p_obs: float | None  # -log2 rho; None unless rho > 0
    q_hat: float | None  # extrapolated value; None unless rho in (0, 1)
    bound: float  # tolerance of the branch taken; on failure that of (ii) if available, else of (i)
    error: float  # |q_K - q_ref| for branch (i), |q_hat - q_ref| for branch (ii); as for bound on failure
    F: float
    F_ref: float
    pass_i: bool
    pass_ii: bool
    reasons: tuple = field(default_factory=tuple)

    def report(self):
        """Every term the run report lists (item 9): d_k, rho, p_obs, E, q_hat, bound, F."""
        return {
            "passed": self.passed, "branch": self.branch, "d": list(self.d), "rho": self.rho, "p_obs": self.p_obs,
            "E": self.E, "q_hat": self.q_hat, "bound": self.bound, "error": self.error, "F": self.F,
            "F_ref": self.F_ref, "pass_i": self.pass_i, "pass_ii": self.pass_ii, "reasons": list(self.reasons),
        }


def evaluate(q, q_ref, F, F_ref):
    q = [float(x) for x in q]
    if len(q) < 2:
        raise ValueError("evaluate needs q_0..q_K with K >= 1")
    if F < 0 or F_ref < 0:
        raise ValueError("F and F_ref must be >= 0")
    K = len(q) - 1
    d = tuple(q[k] - q[k - 1] for k in range(1, K + 1))
    E = abs(d[-1])
    qK = q[-1]
    reasons = []
    finite = all(math.isfinite(x) for x in (*q, q_ref, F, F_ref)) and all(math.isfinite(x) for x in d)
    if not finite:
        return Verdict(False, None, d, E, None, None, None, math.nan, math.nan, F, F_ref, False, False,
                       ("non-finite input",))

    err_i = abs(qK - q_ref)
    bound_i = F + F_ref
    pass_i = E <= F and err_i <= bound_i
    if not pass_i:
        reasons.append("(i): E > F" if E > F else "(i): |q_K - q_ref| > F + F_ref")

    rho = p_obs = q_hat = None
    err_ii = bound_ii = None
    pass_ii = False
    if K < 2 or d[-2] == 0.0:
        reasons.append("(ii): unavailable, no d_{K-1} or d_{K-1} = 0")
    else:
        rho = d[-1] / d[-2]
        if rho > 0.0:
            p_obs = -math.log2(rho)
        if not (0.0 < rho < 1.0):
            reasons.append("(ii): rho not in (0, 1)")
        else:
            q_hat = qK + d[-1] * rho / (1.0 - rho)
            err_ii = abs(q_hat - q_ref)
            bound_ii = E + F * (1.0 + rho) / (1.0 - rho) + F_ref
            pass_ii = err_ii <= bound_ii
            if not pass_ii:
                reasons.append("(ii): |q_hat - q_ref| > bound")

    if pass_i:
        branch, bound, error = BRANCH_I, bound_i, err_i
    elif pass_ii:
        branch, bound, error = BRANCH_II, bound_ii, err_ii
    else:
        branch = None
        bound, error = (bound_ii, err_ii) if bound_ii is not None else (bound_i, err_i)
    passed = branch is not None
    return Verdict(passed, branch, d, E, rho, p_obs, q_hat, bound, error, F, F_ref, pass_i, pass_ii,
                   () if passed else tuple(reasons))


def _bits(read):
    if isinstance(read, (int, float)):
        read = (read,)
    return struct.pack(f"<{len(read)}d", *[float(x) for x in read])


def fresh_index(reads):
    """The last index i >= 1 whose raw read differs bitwise from read i-1 (item 11), or None if there is none.
    A read is a float or a sequence of floats (the raw gz pose and velocity of one step); -0.0 differs from 0.0."""
    bits = [_bits(r) for r in reads]
    for i in range(len(bits) - 1, 0, -1):
        if bits[i] != bits[i - 1]:
            return i
    return None


@dataclass(frozen=True)
class Freshness:
    index: int | None  # fresh_index(reads)
    stale_steps: int  # steps i >= 1 whose read equals the previous one bitwise (the run report lists it)
    ok: bool
    reasons: tuple


def freshness(reads, times, t_settle=None):
    """Item 11 failure conditions. `times` is the simTime of each read, seconds, nondecreasing; the run spans
    times[0]..times[-1]. Fails when there is no fresh read at a time >= the midpoint of the run (the last half), or,
    when t_settle is given (hover), none at a time > t_settle."""
    if len(reads) != len(times) or len(reads) < 2:
        raise ValueError("freshness needs reads and times of equal length >= 2")
    bits = [_bits(r) for r in reads]
    stale = sum(1 for i in range(1, len(bits)) if bits[i] == bits[i - 1])
    idx = fresh_index(reads)
    mid = times[0] + (times[-1] - times[0]) / 2
    reasons = []
    if idx is None or times[idx] < mid:
        reasons.append("no fresh read in the last half of the run")
    if t_settle is not None and (idx is None or times[idx] <= t_settle):
        reasons.append("no fresh read after the settling time")
    return Freshness(idx, stale, not reasons, tuple(reasons))


# ---------------------------------------------------------------------------------------------------------------------
# T4 behaviour predicates S, B and T (docs/decisions/0029; quad spec 4.1). Evaluated on the m = 1 run.
#
# A segment is {name: (unit, errors)}: errors[i] = q(n_i) - q_ref(n_i) at the segment's executions n_0..n_last, plain
# floats. Delta is the largest |error| in the segment among the quantities of the same unit, so a roll angle and a
# heading share a Delta and a rate does not. s is the register's t4_settle_fraction, read by the caller; E[name] is
# the quantity's |m1 - m2| (SIM-7), >= 0. A predicate returns one Check per quantity. Non-finite input fails.
# ---------------------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Check:
    name: str
    ok: bool
    value: float  # the error the predicate compared
    bound: float  # what it was compared with


def _peak(errors):
    magnitudes = [abs(float(e)) for e in errors]
    return math.nan if not all(math.isfinite(m) for m in magnitudes) else max(magnitudes)


def _segment_deltas(segment):
    deltas = {}
    for name, (unit, errors) in segment.items():
        if len(errors) == 0:
            raise ValueError(f"segment quantity {name!r} has no executions")
        peak = _peak(errors)
        previous = deltas.get(unit, 0.0)
        deltas[unit] = math.nan if math.isnan(peak) or math.isnan(previous) else max(previous, peak)
    return deltas


def _checked(name, value, bound):
    ok = math.isfinite(value) and math.isfinite(bound) and value <= bound
    return Check(name, ok, value, bound)


def settles(segment, s, E):
    """S: at the segment's last execution |error| <= s Delta + E, Delta per unit group."""
    deltas = _segment_deltas(segment)
    return [_checked(name, abs(float(errors[-1])), s * deltas[unit] + E[name])
            for name, (unit, errors) in segment.items()]


def bounded(segment, s, E):
    """B: for a segment without a disturbance onset, |error(n)| <= |error(n_0)| + s Delta + E at every n. The value
    reported is the largest excess |error(n)| - |error(n_0)|, so it is compared with s Delta + E."""
    deltas = _segment_deltas(segment)
    out = []
    for name, (unit, errors) in segment.items():
        first = abs(float(errors[0]))
        excess = max(abs(float(e)) - first for e in errors) if math.isfinite(_peak(errors)) else math.nan
        out.append(_checked(name, excess, s * deltas[unit] + E[name]))
    return out


def shrinks(errors):
    """T: the stepped axis's |error| at the segment end is smaller than at its start. No number."""
    first, last = abs(float(errors[0])), abs(float(errors[-1]))
    return math.isfinite(first) and math.isfinite(last) and last < first


# ---------------------------------------------------------------------------------------------------------------------
# Roles (docs/decisions/0031): B applies only to what a scenario steps or holds. A role is static, read from the scenario
# file and the documented firmware logic, never from run data.
#   STEPPED  the error is nonzero at n_0 by a reference change or an initial offset.
#   HELD     a constant reference, neither stepped nor driven.
#   DRIVEN   the error is 0 at n_0 by construction but the scenario sets it moving: the rate of a stepped quantity, a
#            quantity given a nonzero rate by the initial state, a reference set by a firmware event inside its own axis
#            response (the heading relative to the yaw lock).
# S applies to every quantity; B to STEPPED and HELD quantities of segments without a disturbance onset. Delta is 0029's
# rule, except where every quantity of a unit in a segment is HELD: Delta is then the largest Delta of that unit over the
# scenario's segments that have a STEPPED or DRIVEN quantity of that unit (a HELD quantity has no excursion of its own to
# scale by); ValueError when there is none (the quantity is not checked).
# ---------------------------------------------------------------------------------------------------------------------

STEPPED, HELD, DRIVEN = "stepped", "held", "driven"
ROLES = (STEPPED, HELD, DRIVEN)


def _unit_deltas(segment):
    """{unit: Delta} of a role segment, and the set of units that have a STEPPED or DRIVEN quantity."""
    deltas, moving = {}, set()
    for name, (unit, role, errors) in segment.items():
        if role not in ROLES:
            raise ValueError(f"quantity {name!r}: unknown role {role!r}")
        if len(errors) == 0:
            raise ValueError(f"segment quantity {name!r} has no executions")
        peak = _peak(errors)
        previous = deltas.get(unit, 0.0)
        deltas[unit] = math.nan if math.isnan(peak) or math.isnan(previous) else max(previous, peak)
        if role != HELD:
            moving.add(unit)
    return deltas, moving


def scenario_checks(segments, s, E_end, E_max, disturbed=frozenset()):
    """{segment: {"S": [Check], "B": [Check]}} for segments {segment: {name: (unit, role, errors)}}, s the register's
    t4_settle_fraction, E_end[segment][name] the |m1 - m2| of the segment's last execution (S) and E_max[segment][name]
    its largest over the segment (B). A segment in `disturbed` has a disturbance onset: S only."""
    per = {seg: _unit_deltas(quantities) for seg, quantities in segments.items()}
    scale = {}
    for deltas, moving in per.values():
        for unit in moving:
            scale[unit] = max(scale.get(unit, 0.0), deltas[unit])
    out = {}
    for seg, quantities in segments.items():
        deltas, moving = per[seg]
        delta = {}
        for unit in deltas:
            if unit in moving:
                delta[unit] = deltas[unit]
            elif unit in scale:
                delta[unit] = scale[unit]
            else:
                raise ValueError(f"segment {seg!r}: every {unit!r} quantity is HELD and no segment has a STEPPED or "
                                 f"DRIVEN one to scale by")
        checks = {"S": [], "B": []}
        for name, (unit, role, errors) in quantities.items():
            checks["S"].append(_checked(name, abs(float(errors[-1])), s * delta[unit] + E_end[seg][name]))
            if role != DRIVEN and seg not in disturbed:
                first = abs(float(errors[0]))
                excess = max(abs(float(e)) - first for e in errors) if math.isfinite(_peak(errors)) else math.nan
                checks["B"].append(_checked(name, excess, s * delta[unit] + E_max[seg][name]))
        out[seg] = checks
    return out
