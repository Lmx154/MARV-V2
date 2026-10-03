"""R1X's alpha excess in gz is the coupling w x Jw (decision 0014, owner decisions third round, item 3):
tests/regression/quad/L06/results/r1x_coupling/.

Re-runs r1x_coupling.py once (its main, the README's command). gz is not run: gz_new.txt and gz_old.txt, the compact gz
summaries capture_gz.py wrote, are its committed inputs. Checks:
  - the raw output reproduces coupling.txt byte for byte; control: a changed result renders differently;
  - the new gains' design and model, re-run here, equal gz_new.txt's sections line for line, and both summaries were captured
    with today's r1x_model.py; control: a one-character change in a captured model line is caught;
  - every dist row is |gz - run| at its n (new: the run's value is the re-run's); control: a planted row is caught;
  - (i) new gains: gz leaves the envelope; D (no w x Jw) on gz's branch stays inside; C (+ w x Jw) leaves it; C is closer to
    gz than D on alpha, w_x, w_y, w_z, err_z, its alpha distance within the test's F + Q and D's beyond;
  - (ii) old gains: gz and C stay inside, D inside, the same distance claims;
  - control (core 7.2) of (i) and (ii): C0, the coupled model from the exact start (not on gz's branch), is not closer than D
    on any of those channels;
  - (iii) gz's first-step attitude q(1), which picks the branch, is bit-identical under both gain sets;
  - (iv) CF (C with the ideal lag-compensated feed-forward) equals D within K + R and stays inside, under both gain sets;
    control: C is beyond K + R from D on every channel.
"""

import copy
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
DIR = ROOT / "tests" / "regression" / "quad" / "L06" / "results" / "r1x_coupling"
sys.path.insert(0, str(DIR))
import r1x_coupling as rc  # noqa: E402


@pytest.fixture(scope="module")
def run(tmp_path_factory):
    out = tmp_path_factory.mktemp("r1x_coupling") / "coupling.txt"
    r, ev, text = rc.main(["--out", str(out)])
    return r, ev, text, out


def test_raw_output_reproduces_byte_for_byte(run):
    *_, out = run
    assert out.read_bytes() == (DIR / "coupling.txt").read_bytes()


def test_control_a_changed_result_renders_differently(run):
    r, ev, text, _ = run
    planted = copy.deepcopy(ev)
    planted["new"]["C_closer_on_all"] = not planted["new"]["C_closer_on_all"]
    assert rc.render(r, planted) != text


def test_rerun_equals_the_capture_and_the_model_is_the_captured_one(run):
    r, *_ = run
    assert r["model_sha_ok"] and r["rerun_equal"]
    for s in (1, 3):
        assert rc.first_difference(r["rerun"]["blocks"][s], r["gz"]["new"]["blocks"][s]) is None


def test_control_a_changed_captured_model_line_is_caught(run):
    r, *_ = run
    captured = list(r["gz"]["new"]["blocks"][3])
    i = next(i for i, ln in enumerate(captured) if ln.startswith("model.series_sha256"))
    captured[i] = captured[i][:-3] + ("0" if captured[i][-3] != "0" else "1") + captured[i][-2:]
    assert rc.first_difference(r["rerun"]["blocks"][3], captured) is not None


def test_dist_rows_are_consistent(run):
    r, *_ = run
    assert rc.check_dist(r["gz"]["new"], r["rerun"]["runs"]) is None
    assert rc.check_dist(r["gz"]["old"]) is None


def test_control_a_planted_dist_row_is_caught(run):
    r, *_ = run
    planted = copy.deepcopy(r["gz"]["new"])
    row = planted["facts"]["dist.C"]["alpha"]
    row[0] = row[0] * 2
    assert rc.check_dist(planted, r["rerun"]["runs"]) is not None


def test_claim_i_new_gains_the_coupled_model_reproduces_gz_and_its_excess(run):
    _, ev, *_ = run
    e = ev["new"]
    assert e["verdict_consistent"] and e["gz_outside"], e
    assert e["D_inside"] and e["C_outside"] and e["C_does_what_gz_does"], e
    assert e["C_closer_on_all"] and e["alpha_C_within_tol"] and e["alpha_D_beyond_tol"], e


def test_claim_ii_old_gains_the_same(run):
    _, ev, *_ = run
    e = ev["old"]
    assert e["verdict_consistent"] and not e["gz_outside"], e
    assert e["D_inside"] and not e["C_outside"] and e["C_does_what_gz_does"], e
    assert e["C_closer_on_all"] and e["alpha_C_within_tol"] and e["alpha_D_beyond_tol"], e


def test_control_the_coupled_model_off_gz_branch_is_not_closer_than_d(run):
    _, ev, *_ = run
    for tag in ("new", "old"):
        assert not ev[tag]["control_C0_closer_on_any"], (tag, ev[tag]["control_C0_closer"])


def test_claim_iii_gz_first_step_attitude_is_bit_identical_under_both_gain_sets(run):
    _, ev, *_ = run
    assert all(ev["identity"]["q1_identical"].values()), ev["identity"]["q1_identical"]


def test_claim_iv_ideal_ff_equals_d_within_k_plus_r_and_stays_inside(run):
    _, ev, *_ = run
    for tag in ("new", "old"):
        assert ev[tag]["ff_within"] and ev[tag]["ff_inside"], tag


def test_control_without_the_feed_forward_the_coupled_model_is_beyond_k_plus_r(run):
    _, ev, *_ = run
    for tag in ("new", "old"):
        assert ev[tag]["ff_control"], tag


def test_the_combined_claim(run):
    _, ev, text, _ = run
    assert ev["claim"]
    assert text.rstrip("\n").endswith("the excess is the coupling: True")
