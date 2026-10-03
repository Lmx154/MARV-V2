"""Per-motor notch-speed mixes against the configuration set (decision 0014, fifth round items 3 and 4: the shared notch speed
of the design model is a simplification, the firmware tracks each rotor, fw/gyro_chain/include/marv/gyro_chain/gyro_chain.hpp
update_notches).

rate_lead's configuration set (rule step 13) and attitude_lead's (rule step 6') put all four motors' notches at one speed of
the notch grid, or all bypassed. In flight each motor has its own speed. The 12 notches and the low-pass are a cascade of
commuting stages, so the chain's H depends only on the multiset of the four motors' states, each a speed of the notch grid
(omega_th .. omega_max, the grid of the set's resolved level + 1) or bypassed (below omega_th: chain_stages makes the notches the
identity). This file evaluates EVERY multiset (35 on the 3 speeds of level 1 and bypassed), at the profile's latency and at 0, on
the same evaluations the set uses (rate_lead.guard: the firmware loop, the SIL's dt pattern, f32 alpha, expf -+2 ulp, nominal and
the four corners, three axes; attitude_lead's loops at the final f32 k, yaw at its effective gain), and asserts
  - every rate guard row (axis) has PM >= the set's worst PM and Ms <= the set's worst Ms, and the attitude loops' PM >= the
    set's worst PM. The comparisons are exact: the multisets of four equal states are the set's own configurations and reproduce
    its rows bit for bit (asserted); every other multiset clears the set's worst by far more than the rounding of reordering
    the cascade's 13 stage products (the nearest is 0.9 degrees away), so the exact comparison carries no tolerance;
  - the premise of the argument that the extremes bind (decision 0014): below each notch's centre, the phase and the magnitude of
    the notch are non-decreasing in its own speed (less lag, |H| closer to 1), and the crossovers and Ms peaks of the set lie
    below omega_th, the lowest centre. Checked on gyro_chain_design's notch_coeffs / stage_response at the chain's own Q.
Controls (core 7.2): a configuration the firmware cannot fly, one motor's notches forced active at 0.9 omega_th (below the bypass
threshold), breaks the rate guard's floors PM_min and Ms_max and the set's worst; the attitude PM check breaks at the attitude
gain x 1.1 (the forced-active motor does not lower the attitude PM, which the bypassed, latency-0 loop binds); the premise's
monotonicity fails at the first grid frequency above the notch centre, and the location check fails on a loop whose gains are
scaled to put a crossover above omega_th.

rate_lead.design and the stage (c) attitude design run once per pytest session (conftest.py). The mixes run in worker processes
(about 105 s of CPU, 13 s on 16 CPUs).
"""

import cmath
import itertools
import math
import multiprocessing
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
import attitude  # noqa: E402
import attitude_lead as al  # noqa: E402
import gyro_chain_design as gcd  # noqa: E402
import rate_lead as rl  # noqa: E402

# A motor below omega_th: chain_stages makes its three notches the identity (gyro_chain.hpp update_notches).
BYPASS = 0.0
# The control motor's speed as a fraction of omega_th (the packet's labelled value; below the threshold, so the firmware never
# flies it: the notches would be bypassed).
FORCED_ACTIVE_FRACTION = 0.9
# The attitude control's gain factor (the labelled x 1.1 of the L6 controls).
ATT_GAIN_CONTROL = 1.1
# The location control's gain factor (labelled): rate gains x 1000 put the loop's crossover at about 500 rad/s, above omega_th.
LOCATION_GAIN_CONTROL = 1000.0
# The premise scan's resolution: 2^10 intervals of rotor speed on a geometric grid over [omega_th, omega_max], and every 4th
# frequency of gyro_chain_design's grid (monotonicity is a continuous property; this is its resolution on the grid).
PREMISE_SPEEDS = 2 ** 10 + 1
PREMISE_STRIDE = 4

_CONTEXT = {}


def evaluate(speeds, latency, omega_th=None):
    """{"rate": rl.guard rows, "att": [(axis, corner, PM or -inf, margin valid)]} of the mix `speeds` (the four motors' rotor
    speeds, BYPASS for a bypassed motor) at `latency`; omega_th the chain's bypass threshold (default: the design's)."""
    c, m = _CONTEXT["chain"], _CONTEXT["model"]
    stages = gcd.chain_stages(c["t_s"], c["f_c"], c["q"], c["omega_th"] if omega_th is None else omega_th, list(speeds))
    rows = rl.guard(rl.Model(m.t_s, m.divisor, latency, stages, m.corners, m.a, m.inertia), _CONTEXT["axes32"], _CONTEXT["dts"])
    inner = al.Inner(f"mix {speeds}", _CONTEXT["axes32"], m.t_s, m.divisor, latency, [s for s in stages if s != gcd.IDENTITY])
    return {"rate": rows, "att": attitude_margins(al.build_loops(_CONTEXT["rr"], inner), _CONTEXT["k32"])}


def attitude_margins(loops, k32):
    """[(axis, corner, PM or -inf without a unique crossover, margin valid)] at the P gain k32 (yaw at its effective gain);
    valid: the unwrapped phase passed attitude's start and branch checks and the crossover is unique."""
    out = []
    for axis, name, loop in loops:
        _, pm, why = loop.margin(attitude.axis_k(axis, k32, _CONTEXT["w32"]))
        out.append((axis, name, -math.inf if pm is None else pm, loop.start_ok and loop.branch_ok and not why))
    return out


def job(args):
    return evaluate(*args)


@pytest.fixture(scope="module")
def setup(rate_lead_design, attitude_lead_design):
    lead, att = rate_lead_design, attitude_lead_design
    _CONTEXT.update(chain=lead["chain"], model=lead["model"], axes32=lead["axes32"], dts=lead["dts"], rr=att["rate"],
                    k32=att["k32"], w32=att["w32"])
    level = lead["set"]["level"] + 1
    grid = [w for _, w in rl.notch_grid(lead["chain"]["omega_th"], lead["chain"]["omega_max"], level)]
    options = sorted(grid, reverse=True) + [BYPASS]
    latencies = (lead["model"].latency, 0)
    jobs = [(mix, lat) for mix in itertools.combinations_with_replacement(options, 4) for lat in latencies]
    with multiprocessing.get_context("fork").Pool(rl.cpu_quota.usable_cpus()) as pool:
        results = pool.map(job, jobs, chunksize=1)
    return {"lead": lead, "att": att, "level": level, "grid": grid, "jobs": jobs, "results": dict(zip(jobs, results)),
            "latencies": latencies}


def set_floors(lead):
    """(worst PM, worst Ms) of rate_lead's configuration set."""
    (pm, *_), (ms, *_) = rl.set_worst(lead["set"]["rows"])
    return pm, ms


def test_the_mixes_are_all_multisets_and_the_equal_ones_reproduce_the_sets_own_rows(setup):
    lead, att, grid = setup["lead"], setup["att"], setup["grid"]
    n = len(grid) + 1
    assert len(setup["jobs"]) == math.comb(n + 3, 4) * len(setup["latencies"]) == len(setup["results"])
    # The equal mixes are the set's configurations: the set's rate rows bit for bit ...
    chain, level = lead["chain"], lead["set"]["level"] + 1
    cfgs = rl.configuration_set(chain, lead["model"].latency, level)
    rows = {key: guard for key, _, guard in lead["set"]["rows"]}
    labels = {key: label for key, label, _, _ in cfgs}
    assert set(rows) == set(labels) and len(rows) == (len(grid) + 1) * len(setup["latencies"])
    for (frac, lat), guard in rows.items():
        speed = BYPASS if frac is None else grid[round(frac * 2 ** level)]
        assert setup["results"][((speed,) * 4, lat)]["rate"] == guard, (frac, lat)
    # ... and the attitude design's PMs, loop for loop (final level's loops and the loops its halving added).
    design = {(a, name): pm for a, name, pm, *_ in att["final"]["detail"] + att["levels"][-1]["added_detail"]}
    for (frac, lat), _ in rows.items():
        speed = BYPASS if frac is None else grid[round(frac * 2 ** level)]
        for axis, corner, pm, valid in setup["results"][((speed,) * 4, lat)]["att"]:
            assert valid and design[axis, f"{labels[frac, lat]}: {corner}"] == pm, (frac, lat, axis, corner)
    assert min(design.values()) == att["pm_worst"]


def test_every_mix_holds_the_rate_guards_worst_pm_and_ms_of_the_set(setup):
    lead = setup["lead"]
    set_pm, set_ms = set_floors(lead)
    pm_min, ms_max = lead["inputs"]["PM_min"], lead["inputs"]["Ms_max"]
    assert set_pm >= pm_min and set_ms <= ms_max
    worst_pm, worst_ms = math.inf, -math.inf
    for (mix, lat), res in setup["results"].items():
        assert len(res["rate"]) == len(rl.AXES)
        assert rl.guard_passes(res["rate"], set_pm, set_ms), (mix, lat, res["rate"])
        worst_pm, worst_ms = min(worst_pm, *(r[0] for r in res["rate"])), max(worst_ms, *(r[2] for r in res["rate"]))
    # The set's worst is attained by a mix (the equal ones), so the bound is tight, not slack.
    assert (worst_pm, worst_ms) == (set_pm, set_ms)
    print(f"{len(setup['results'])} mixes: worst PM {math.degrees(worst_pm)!r} deg, worst Ms {worst_ms!r} (the set's own)")


def test_every_mix_holds_the_attitude_loops_worst_pm_of_the_set(setup):
    att = setup["att"]
    worst = math.inf
    for (mix, lat), res in setup["results"].items():
        assert len(res["att"]) == len(rl.AXES) * len(_CONTEXT["model"].corners)
        for axis, corner, pm, valid in res["att"]:
            assert valid and pm >= att["pm_worst"], (mix, lat, axis, corner, pm)
            worst = min(worst, pm)
    assert worst == att["pm_worst"]


def test_control_a_motor_forced_active_below_the_threshold_breaks_the_rate_guards_floors(setup):
    lead = setup["lead"]
    set_pm, set_ms = set_floors(lead)
    pm_min, ms_max = lead["inputs"]["PM_min"], lead["inputs"]["Ms_max"]
    th = lead["chain"]["omega_th"]
    mix = (FORCED_ACTIVE_FRACTION * th, th, th, th)
    rows = evaluate(mix, lead["model"].latency, omega_th=FORCED_ACTIVE_FRACTION * th)["rate"]
    assert not rl.guard_passes(rows, set_pm, set_ms)
    assert not rl.guard_passes(rows, pm_min, math.inf) and not rl.guard_passes(rows, -math.inf, ms_max)
    # The flown chain bypasses that motor at that speed, which is the set's configuration of the three others only.
    flown = evaluate(mix, lead["model"].latency)["rate"]
    assert rl.guard_passes(flown, set_pm, set_ms)
    print(f"control: PM {math.degrees(min(r[0] for r in rows))!r} deg (PM_min {math.degrees(pm_min)!r}), Ms "
          f"{max(r[2] for r in rows)!r} (Ms_max {ms_max!r})")


def test_control_the_attitude_gain_times_1_1_breaks_the_attitude_pm_check(setup):
    att = setup["att"]
    # The loops of the binding mix (the lowest PM of the mixes): the same evaluation at k x 1.1 falls below the set's worst.
    mix, lat = min(setup["results"], key=lambda j: min(r[2] for r in setup["results"][j]["att"]))
    c, m = _CONTEXT["chain"], _CONTEXT["model"]
    stages = gcd.chain_stages(c["t_s"], c["f_c"], c["q"], c["omega_th"], list(mix))
    inner = al.Inner("control", _CONTEXT["axes32"], m.t_s, m.divisor, lat, [s for s in stages if s != gcd.IDENTITY])
    loops = al.build_loops(_CONTEXT["rr"], inner)
    assert attitude_margins(loops, att["k32"]) == setup["results"][mix, lat]["att"]
    up = attitude_margins(loops, att["k32"] * ATT_GAIN_CONTROL)
    assert min(pm for *_, pm, _ in up) < att["pm_worst"]


# ---- the premise: monotone below each notch's centre, and the loop's frequencies lie there ------------------------------


@pytest.fixture(scope="module")
def notches(rate_lead_design):
    """Per harmonic h and rotor speed on a geometric grid of PREMISE_SPEEDS over [omega_th, omega_max], the notch's coefficients
    (gyro_chain_design.notch_coeffs at the chain's Q_h), and the frequencies (rad/s) of the grid scanned."""
    c, m = rate_lead_design["chain"], rate_lead_design["model"]
    speeds = [c["omega_th"] * (c["omega_max"] / c["omega_th"]) ** (i / (PREMISE_SPEEDS - 1)) for i in range(PREMISE_SPEEDS)]
    coeffs = {h: [gcd.notch_coeffs(h * w / (2 * math.pi), qh, c["t_s"]) for w in speeds] for h, qh in zip(gcd.HARMONICS, c["q"])}
    freqs = [th / m.t for th in gcd._THETA[::PREMISE_STRIDE] if th / m.t * c["t_s"] < math.pi]
    return coeffs, freqs


def monotone_in_speed(coeffs_h, w, t_s):
    """True iff the notch's phase and |H| at loop frequency w are non-decreasing along the increasing rotor speeds."""
    zinv = cmath.exp(-1j * w * t_s)
    vals = [gcd.stage_response(cf, zinv) for cf in coeffs_h]
    phase, mag = [cmath.phase(v) for v in vals], [abs(v) for v in vals]
    return all(b >= a for a, b in zip(phase, phase[1:])) and all(b >= a for a, b in zip(mag, mag[1:]))


def test_premise_each_notchs_phase_and_magnitude_are_monotone_in_its_speed_below_its_centre(rate_lead_design, notches):
    c = rate_lead_design["chain"]
    coeffs, freqs = notches
    for h in gcd.HARMONICS:
        first_bad = next((w for w in freqs if not monotone_in_speed(coeffs[h], w, c["t_s"])), None)
        # Control: the predicate fails on this grid, and no lower than the lowest centre h omega_th (it is the control that
        # the check can fail; below the centre, every grid frequency passed).
        assert first_bad is not None and first_bad >= h * c["omega_th"], (h, first_bad)
        assert all(w < first_bad for w in freqs if w < h * c["omega_th"])
        print(f"h {h}: monotone at every scanned frequency below {first_bad!r} rad/s (lowest centre {h * c['omega_th']!r})")


def loop_frequencies(model, axes32, gain=1.0):
    """(all crossovers, Ms peak frequencies) in rad/s of the design loop over the corners and axes, rate gains x `gain`."""
    xs, peaks = [], []
    for j, (kp, ki, kd, tf) in zip(model.inertia, axes32):
        g = (gain * kp / j, gain * ki / j, gain * kd / j, tf)
        xs += [w for _, _, _, allx in model.margins(g) for w in allx]
        peaks += [w for _, _, w in model.sensitivity(g)]
    return xs, peaks


def test_premise_the_sets_crossovers_and_ms_peaks_lie_below_omega_th(rate_lead_design):
    lead = rate_lead_design
    m, th = lead["model"], lead["chain"]["omega_th"]
    cfgs = rl.configuration_set(lead["chain"], m.latency, lead["set"]["level"] + 1)
    xs, peaks = [], []
    for _, _, lat, stages in cfgs:
        x, p = loop_frequencies(rl.Model(m.t_s, m.divisor, lat, stages, m.corners, m.a, m.inertia), lead["axes32"])
        xs, peaks = xs + x, peaks + p
    assert xs and peaks and max(xs) < th and max(peaks) < th
    print(f"crossovers {min(xs)!r} .. {max(xs)!r} rad/s, Ms peaks {min(peaks)!r} .. {max(peaks)!r} rad/s, omega_th {th!r}")
    # Control: the same extraction on a loop with its gains scaled by the labelled factor puts a frequency above omega_th.
    x, p = loop_frequencies(rl.Model(m.t_s, m.divisor, m.latency, cfgs[0][3], m.corners, m.a, m.inertia), lead["axes32"],
                            LOCATION_GAIN_CONTROL)
    assert max(x + p) >= th
