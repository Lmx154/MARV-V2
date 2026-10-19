"""Invariant S-1 (ground-segment unit GX-8): the import-boundary check tools/ci/check_s1.py, on the real tree and on planted trees
(its negative controls: a SIM file that imports the vehicle client, and a vehicle file that imports the sim channel, each fail it)."""

import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
CHECK = ROOT / "tools" / "ci" / "check_s1.py"


def run(root=None):
    cmd = [sys.executable, str(CHECK)] + (["--root", str(root)] if root else [])
    r = subprocess.run(cmd, capture_output=True, text=True, check=False)
    return r.returncode, r.stdout + r.stderr


def tree(tmp_path, files):
    for name, text in files.items():
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")
    return tmp_path


CLEAN = {
    "lib/link/vehicle.ts": "export const v = 1;\n",
    "lib/state/helper_api.ts": "export const h = 1;\n",
    "lib/state/live_state.ts": "import { v } from '$link/vehicle';\n",
    "lib/components/ui/Panel.svelte": "<script lang=\"ts\"></script>\n",
    "lib/sim/record.ts": "export const r = 1;\n",
    "lib/sim/channel.ts": "import { r } from './record';\nimport { writable } from 'svelte/store';\n",
    "lib/sim/instance.ts": "import { h } from '$lib/state/helper_api';\nimport { r } from './record';\n",
    "lib/sim/SimPanel.svelte": "<script lang=\"ts\">\n\timport Panel from '$lib/components/ui/Panel.svelte';\n</script>\n",
    "routes/sim/+page.svelte": "<script lang=\"ts\">\n\timport { r } from '$lib/sim/instance';\n</script>\n",
    "routes/vehicle/+page.svelte": "<script lang=\"ts\">\n\timport { v } from '$lib/state/live_state';\n</script>\n",
    "routes/+layout.svelte": "<script lang=\"ts\">\n\timport { r } from '$lib/sim/instance';\n</script>\n",
}


def test_the_real_tree_is_clean():
    code, out = run()
    assert code == 0 and "S-1 clean" in out, out


def test_a_clean_planted_tree_passes(tmp_path):
    code, out = run(tree(tmp_path, CLEAN))
    assert code == 0, out


@pytest.mark.parametrize("name,text,expect", [
    # a SIM file reaching the vehicle's side, every way
    ("lib/sim/channel.ts", "import { VehicleClient } from '$link/vehicle';\n", "lib/sim/channel.ts:1"),
    ("lib/sim/channel.ts", "import { v } from '$lib/link/vehicle';\n", "lib/sim/channel.ts:1"),
    ("lib/sim/channel.ts", "import { v } from '../link/vehicle';\n", "lib/sim/channel.ts:1"),
    ("lib/sim/sim_view.ts", "import { computeLive } from '$lib/state/live_state';\n", "lib/sim/sim_view.ts:1"),
    ("lib/sim/sim_view.ts", "import { app } from '$lib/state/app';\n", "lib/sim/sim_view.ts:1"),
    ("lib/sim/sim_view.ts", "const m = await import('$link/session');\n", "lib/sim/sim_view.ts:1"),
    ("lib/sim/SimPanel.svelte", "<script lang=\"ts\">\n\timport LivePanel from '$lib/components/LivePanel.svelte';\n</script>\n", "lib/sim/SimPanel.svelte:2"),
    ("routes/sim/+page.svelte", "<script lang=\"ts\">\n\timport { app } from '$lib/state/app';\n</script>\n", "routes/sim/+page.svelte:2"),
    # the vehicle's side reaching the sim channel
    ("lib/state/live_state.ts", "import { decodeSimRecord } from '$lib/sim/record';\n", "lib/state/live_state.ts:1"),
    ("lib/link/vehicle.ts", "export * from '../sim/record';\n", "lib/link/vehicle.ts:1"),
    ("routes/vehicle/+page.svelte", "<script lang=\"ts\">\n\timport SimPanel from '$lib/sim/SimPanel.svelte';\n</script>\n", "routes/vehicle/+page.svelte:2"),
    ("lib/state/app.ts", "import { simView } from '$lib/sim/instance';\n", "lib/state/app.ts:1"),
])
def test_a_planted_crossing_of_the_boundary_fails_and_names_the_line(tmp_path, name, text, expect):
    code, out = run(tree(tmp_path, {**CLEAN, name: text}))
    assert code == 1, out
    assert f"S1-VIOLATION: gcs/src/{expect}" in out, out


def test_an_empty_or_missing_tree_is_a_tool_failure_not_a_pass(tmp_path):
    assert run(tmp_path)[0] == 2
    assert run(tmp_path / "nowhere")[0] == 2
