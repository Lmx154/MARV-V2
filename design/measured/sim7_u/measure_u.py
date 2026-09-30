"""SIM-7 seed U^0: the L4 rate-loop chirp's margin-measurement uncertainty U, re-measured (decision 0006 section E,
"U and the circularity", step 1; decision 0005 "T4 chirp margins").

U = min over axes of (E_H + U_A + U_d), in rad, with, per axis, from the frozen L4 chirp harness (run_l4.run_chirp,
run_l4.chirp_margin, run_l4.float_input_term, exactly as tests/regression/quad/L04/gz/test_t4_chirp.py calls them):
  E_H = |PM(m = 1) - PM(m = 2)|, U_A = |PM(A) - PM(A/2)|, U_d = run_l4.float_input_term on the (m = 1, A) run.

Two steps, run from the repository root:
  measure_u.py measure [--out-dir DIR]   runs Gazebo (needs the host-gz-l4 build); writes raw.json and inputs.json
  measure_u.py post [--out-dir DIR]      raw.json -> u.yaml; no Gazebo, deterministic
The run matrix (RUNS, AXES, HALF), the card and the scenario directory are the test module's own, imported, not copied.
The test's fourth run (half duration) only checks the duration's convergence and does not enter U; it is not run here.
"""

import argparse
import hashlib
import importlib.util
import json
import math
import os
import string
import sys
import tempfile
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
TEST = ROOT / "tests" / "regression" / "quad" / "L04" / "gz" / "test_t4_chirp.py"
GIT_SHA_LENGTH = 40
RAW, INPUTS, OUT = "raw.json", "inputs.json", "u.yaml"
USED_RUNS = ("m1", "m2", "half_amplitude")
RULE = ("U = min over axes of (E_H + U_A + U_d), in rad (decision 0006 section E, the chirp rule of decision 0005 "
        "'T4 chirp margins'). Per axis: E_H = |PM(m = 1) - PM(m = 2)|, U_A = |PM(A) - PM(A/2)|, U_d = the float32 "
        "input term run_l4.float_input_term of the (m = 1, A) run. The minimum, because a smaller U gives the "
        "higher, safer rate.")


def load_test_module():
    spec = importlib.util.spec_from_file_location("test_t4_chirp", TEST)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def commit():
    """The commit the clean copy was made from: $MARV_COMMIT (a worktree's .git file does not resolve in the container)."""
    value = os.environ.get("MARV_COMMIT", "")
    if len(value) != GIT_SHA_LENGTH or any(c not in string.hexdigits for c in value):
        raise SystemExit("set MARV_COMMIT to the full git commit hash the copy was made from")
    return value.lower()


def measure(out_dir):
    t = load_test_module()
    run_l4 = t.run_l4
    runs_spec = [r for r in t.RUNS if r[0] in USED_RUNS]
    raw = {"method": "measured", "axes": {}}
    inputs = {"command": "see README.md", "git_commit": commit(),
              "script_sha256": sha256(__file__), "test_module": str(TEST.relative_to(ROOT)),
              "test_module_sha256": sha256(TEST), "card": str(t.CARD.relative_to(ROOT)), "card_sha256": sha256(t.CARD),
              "plugin_dir": str(Path(t.PLUGIN_DIR).relative_to(ROOT)),
              "plugin_sha256": {p.name: sha256(p) for p in sorted(Path(t.PLUGIN_DIR).glob("*.so"))},
              "run_l4_sha256": sha256(ROOT / "tools" / "sim" / "run_l4.py"), "scenarios": {}}
    for axis in t.AXES:
        scenario = t.SCEN / f"chirp_{axis}.yaml"
        inputs["scenarios"][axis] = {"path": str(scenario.relative_to(ROOT)), "sha256": sha256(scenario)}
        with tempfile.TemporaryDirectory() as tmp:
            runs = {name: run_l4.run_chirp(t.CARD, scenario, m, Path(tmp), t.PLUGIN_DIR, amp_scale=scale, halved=halved)
                    for name, m, scale, halved in runs_spec}
            margins = {name: run_l4.chirp_margin(s) for name, s in runs.items()}
            assert all(r["pm"] is not None for r in margins.values()), {n: r["reason"] for n, r in margins.items()}
            design = runs["m1"].plan.design
            raw["axes"][axis] = {
                "runs": {name: {"m": runs[name].run.m, "amp_scale": scale, "amplitude_nm": runs[name].plan.amp_nm,
                                "pm_rad": margins[name]["pm"], "crossover_rad_s": margins[name]["crossover"],
                                "stale_reads_in_window": len(runs[name].window_stale)}
                         for name, _, scale, _ in runs_spec},
                "U_d_terms_of_m1_A": run_l4.float_input_term(runs["m1"], margins["m1"]),
                "design_pm_rad": dict(design["design_pm"]), "pm_min_rad": design["pm_min"]}
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / RAW).write_text(json.dumps(raw, indent=2) + "\n", encoding="utf-8")
    (out_dir / INPUTS).write_text(json.dumps(inputs, indent=2) + "\n", encoding="utf-8")


def post(out_dir):
    raw = json.loads((out_dir / RAW).read_text(encoding="utf-8"))
    per_axis = {}
    for axis, a in raw["axes"].items():
        pm = {n: r["pm_rad"] for n, r in a["runs"].items()}
        e_h, u_a = abs(pm["m1"] - pm["m2"]), abs(pm["m1"] - pm["half_amplitude"])
        u_d = a["U_d_terms_of_m1_A"]["U_d"]
        u_axis = e_h + u_a + u_d
        per_axis[axis] = {"E_H_rad": e_h, "U_A_rad": u_a, "U_d_rad": u_d, "U_rad": u_axis,
                          "E_H_deg": math.degrees(e_h), "U_A_deg": math.degrees(u_a), "U_d_deg": math.degrees(u_d),
                          "U_deg": math.degrees(u_axis)}
    axis_min = min(per_axis, key=lambda k: per_axis[k]["U_rad"])
    u = per_axis[axis_min]["U_rad"]
    doc = {"U": u, "unit": "rad", "U_deg": math.degrees(u), "method": "measured", "minimum_over_axes_at": axis_min,
           "rule": RULE, "source": {"script": "design/measured/sim7_u/measure_u.py", "raw": f"design/measured/sim7_u/{RAW}",
                                    "inputs": f"design/measured/sim7_u/{INPUTS}"},
           "per_axis": per_axis}
    (out_dir / OUT).write_text(yaml.safe_dump(doc, sort_keys=False, width=120), encoding="utf-8")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("step", choices=("measure", "post"))
    ap.add_argument("--out-dir", type=Path, default=HERE)
    args = ap.parse_args(argv)
    (measure if args.step == "measure" else post)(args.out_dir)
    return 0


if __name__ == "__main__":
    sys.exit(main())
