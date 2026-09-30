#!/usr/bin/env python3
"""SIM-3 throughput measurement of the gz-sim 8 lockstep plugin (quad spec 3.3 SIM-3, docs/decisions/0003 item 2).

  measure_sim3.py [--card FILE] [--scenario FILE] [--m N] [--iterations N] [--plugin-dir DIR] [--out-dir DIR] [--output FILE]

Runs run_scenario.measure_rtf on the free_fall scenario at m = 1 (one gz process for the full run and one for a 10-iteration
startup run, the test world, RTF 0), by default on the Release plugin build (`cmake --preset host-gz-release && cmake --build
--preset host-gz-release`, plugin in build/host-gz-release/sim/gz/plugin), and prints the SIM-3 block: per-core RTF, startup
overhead, CPU / wall, cores, build type, and the required value and the comparison, both UNKNOWN (design/budget.yaml
batch_plan is UNKNOWN, the item stays open until it is set; core C-6).

The simulated duration is a labelled scenario value: by default the scenario's own T (free_fall: duration_ticks 6400 at
m = 1, 1 s). It may be shortened with --iterations but not lengthened (a 10 s run hits a DART assert, seen at the determinism
scenario), so it never exceeds the scenario's T. --output also writes the block to a file. Nothing is committed.
"""

from __future__ import annotations

import argparse
import os
import platform
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
import run_scenario  # noqa: E402
import scenario as scn  # noqa: E402

DEFAULT_CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
DEFAULT_SCENARIO = ROOT / "scenarios" / "quad" / "L02" / "free_fall.yaml"
DEFAULT_PLUGIN_DIR = ROOT / "build" / "host-gz-release" / "sim" / "gz" / "plugin"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--card", default=str(DEFAULT_CARD), metavar="FILE")
    ap.add_argument("--scenario", default=str(DEFAULT_SCENARIO), metavar="FILE")
    ap.add_argument("--m", type=int, default=1)
    ap.add_argument("--iterations", type=int, help="host steps of the full run; default and maximum: the scenario's T")
    ap.add_argument("--plugin-dir", default=str(DEFAULT_PLUGIN_DIR), metavar="DIR")
    ap.add_argument("--out-dir", metavar="DIR", help="default: a temporary directory")
    ap.add_argument("--output", metavar="FILE")
    args = ap.parse_args(argv)
    doc = scn.load(args.scenario, args.card)
    limit = scn.iterations(doc, args.m)
    iterations = limit if args.iterations is None else args.iterations
    if not 0 < iterations <= limit:
        ap.error(f"--iterations must be in 1..{limit} (the scenario's T at m = {args.m})")
    if not run_scenario.plugin_present(args.plugin_dir):
        ap.error(f"no {run_scenario.PLUGIN_FILE} in {args.plugin_dir}")
    out_dir = args.out_dir or tempfile.mkdtemp(prefix="marv_sim3_")
    try:
        s = run_scenario.measure_rtf(args.card, args.scenario, args.m, None, out_dir, args.plugin_dir, iterations=iterations)
    except run_scenario.RunError as e:
        print(f"measure_sim3: {e}", file=sys.stderr)
        return 1
    tick = scn.tick_period_s(doc)
    header = [
        f"command: measure_sim3.py --scenario {Path(args.scenario).name} --m {args.m} --iterations {iterations} "
        f"--plugin-dir {Path(args.plugin_dir).relative_to(ROOT) if Path(args.plugin_dir).is_relative_to(ROOT) else args.plugin_dir}",
        f"simulated duration: {float(iterations * args.m * tick)!r} s (a labelled scenario value: {iterations} host steps of "
        f"m = {args.m} ticks; the scenario's own T is {limit * args.m} ticks)",
        f"host: {platform.platform()}, {platform.machine()}, python {platform.python_version()}",
        f"nproc: {os.cpu_count()} online, {len(os.sched_getaffinity(0))} in this process's affinity",
        f"build type: {s.plugin_build_type} (CMAKE_BUILD_TYPE of the plugin build)",
    ]
    text = "\n".join(header) + "\n" + run_scenario.format_sim3(s)
    sys.stdout.write(text)
    if args.output:
        Path(args.output).parent.mkdir(parents=True, exist_ok=True)
        Path(args.output).write_text(text, encoding="utf-8", newline="\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
