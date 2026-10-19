#!/usr/bin/env python3
"""The configuration validator (core G7; ground spec §2 "Prove"; decision 0053 Q2 and unit 9): a config file and the image it
is for in, a validation report out. The GCS activates a config only with a passing report for (image hash, config hash), and
the activation carries the report's id, which the FC records in the slot it boots from (the binding of the store).

  validate_config.py --config <file.mcfg> [--image <file>] [--build <dir>] [--card <card>] [--budget <budget>]
                     [--required <required_checks.yaml>] [--work-dir <dir>] [--out <report.json>]

  --config    the candidate config (.mcfg, tools/config/mcfg.py); its section of the image's schema is checked
  --image     the image the config is for; its SHA-256 is the report's image hash. Default: the build's SIL library
              <build>/sim/quad_fc/libmarv_sil_quad_fc.so (the hash the SIL reports as its image identity)
  --build     the image's host build tree with the Gazebo plugin (default build/host-gz-quadfc): the parameter set's
              manifest, marv_validate_table, the config-validation ctests and the plugin of check 4
  --card      the config's card (default vehicles/marv_7in_fc3v2.yaml, the card of the FC3v2 set)
  --required  the required-check list (default tools/validate/required_checks.yaml, signed by Luis)
  --work-dir  where the runs go (default: a new directory under the system temporary directory)
  --out       the report (default <work-dir>/report.json)

The checks are those of the required-check list, run in its order (each one's rule is in that file): records,
composition (marv_validate_table: the firmware's store::decode with composition::validate), ctest (ctest -L
config-validation with MARV_VALIDATE_CONFIG naming the config), sil (tools/validate/sil_check.py: a quad_fc Gazebo run on
the card with the existing behaviour predicates). Each is run even when an earlier one failed, as far as it can be.

The report (format marv-validation-report/1) is canonical JSON: sorted keys, separators "," and ":", UTF-8, one final newline.
Its id is the SHA-256 of exactly those bytes, so it carries no timestamp and no absolute path: the same inputs, tools and
commit give the same id. Fields: format; verdict (PASS only if every required check passes, a check that could not run
fails); image {file, sha256, kind (sil: the build's SIL library; file: any other)}; sil_library_sha256 (the library the
ctests and check 4 ran with); config {file, file_sha256, config_hash (core IF-1, of the image's section), schema_hash};
card {file, sha256}; budget {file, sha256}; required_checks {file, sha256}; tools {commit, dirty, python, ctest, gz};
checks [{id, title, result, values [text], failures [text]}] in the list's order.

Progress goes to stderr, one line per check ("validate_config: <id>: PASS|FAIL"); the last stdout lines are
"report: <path>", "report_id: <hex>" and "verdict: PASS|FAIL". Exit 0 on PASS, 1 on FAIL, 2 when an input is missing.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import platform
import re
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "tools" / "config"))
sys.path.insert(0, str(HERE))
import mcfg  # noqa: E402
import yaml  # noqa: E402

FORMAT = "marv-validation-report/1"
ENV = "MARV_VALIDATE_CONFIG"
DEFAULT_BUILD = ROOT / "build" / "host-gz-quadfc"
DEFAULT_CARD = ROOT / "vehicles" / "marv_7in_fc3v2.yaml"
DEFAULT_BUDGET = ROOT / "design" / "budget.yaml"
DEFAULT_REQUIRED = HERE / "required_checks.yaml"
PARAMETER_SET = "marv_params_fc3v2"
SECTION = "quad"
SIL_LIBRARY = Path("sim") / "quad_fc" / "libmarv_sil_quad_fc.so"
PLUGIN_DIR = Path("sim") / "gz" / "plugin"
TABLE_TOOL = Path("tools") / "validate" / "marv_validate_table"
CTEST_TIMEOUT_S = 1800  # labelled harness value: bounds a hung ctest run only
CTEST_OUTPUT_BYTES = 1 << 20  # labelled harness value: per-test output kept in the junit file (ctest truncates at 1 KiB by default)
QF3_LINE = re.compile(r"^qf3_table: (rate|attitude) .*$", re.M)
L07_LINE = re.compile(r"^\s*(\(a\) .*|control \((a|c')\): theta_rot .*)$", re.M)


class InputError(Exception):
    pass


def sha256_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def canonical(doc):
    return (json.dumps(doc, sort_keys=True, separators=(",", ":"), ensure_ascii=False) + "\n").encode("utf-8")


def rel(path):
    p = Path(path).resolve()
    try:
        return p.relative_to(ROOT).as_posix()
    except ValueError:
        return p.name


def progress(text):
    print(f"validate_config: {text}", file=sys.stderr, flush=True)


def git_state():
    def git(*args):
        return subprocess.run(["git", "-C", str(ROOT), *args], capture_output=True, text=True, check=False)

    head = git("rev-parse", "HEAD")
    if head.returncode != 0:
        return "UNKNOWN", None
    dirty = git("status", "--porcelain")
    return head.stdout.strip(), bool(dirty.stdout.strip())


def tool_version(*cmd):
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=60, check=False)
    except (OSError, subprocess.TimeoutExpired):
        return "UNKNOWN"
    first = (out.stdout or out.stderr).strip().splitlines()
    return first[0] if first else "UNKNOWN"


def gz8_version():
    try:
        out = subprocess.run(["gz", "sim", "--force-version", "8", "--versions"], capture_output=True, text=True, timeout=60,
                             check=False)
    except (OSError, subprocess.TimeoutExpired):
        return "UNKNOWN"
    return next((v.strip() for v in out.stdout.splitlines() if v.strip().startswith("8.")), "UNKNOWN")


def result(check, failures, values):
    return {"id": check["id"], "title": check["title"], "result": "FAIL" if failures else "PASS", "values": values,
            "failures": failures}


# ---- check: records ------------------------------------------------------------------------------------------------

def run_table_tool(tool, config):
    try:
        out = subprocess.run([str(tool), str(config)], capture_output=True, text=True, timeout=60, check=False)
    except (OSError, subprocess.TimeoutExpired) as e:
        return {"error": f"marv_validate_table did not run: {e}"}
    fields = dict(line.split(": ", 1) for line in out.stdout.splitlines() if ": " in line)
    if out.returncode not in (0, 1) or "verdict" not in fields:
        return {"error": f"marv_validate_table exited {out.returncode}: {out.stderr.strip() or out.stdout.strip()}"}
    return fields


def check_records(check, cfg, cfg_error, manifest, table):
    failures, values = [], []
    if cfg is None:
        return result(check, [f"the file is not a valid .mcfg: {cfg_error}"], values)
    schema_hash = mcfg.manifest_schema_hash(manifest)
    section = next((s for s in cfg.sections if s.schema_hash == schema_hash), None)
    if section is None:
        return result(check, [f"no section has the image's schema hash {manifest['schema_hash']} (sections: "
                              + ", ".join(f"{s.kind} 0x{s.schema_hash:016x}" for s in cfg.sections) + ")"], values)
    if section.kind != SECTION:
        failures.append(f"the image's schema is in a {section.kind} section, not {SECTION}")
    records = mcfg.section_records(section)
    names = list(manifest["params"])
    if len(records) != len(names):
        return result(check, [f"{len(records)} records, the image has {len(names)}"], values)
    defaults = mcfg.manifest_records(manifest)
    changed, locked = [], 0
    for name, r, d in zip(names, records, defaults, strict=True):
        if r.type != d.type:
            failures.append(f"{name}: type {r.type}, the image's is {d.type}")
            continue
        if r.type == "f32" and not math.isfinite(mcfg.f32_of(r.value_bits)):
            failures.append(f"{name}: value bits 0x{r.value_bits:08x} are not a finite f32")
        if not mcfg.sigma_ok(r.sigma_kind, r.sigma_bits):
            failures.append(f"{name}: sigma kind {r.sigma_kind} with sigma bits 0x{r.sigma_bits:08x} breaks I1")
        if not r.source.strip():
            failures.append(f"{name}: the source text is empty")
        if d.locked:
            locked += 1
            if r != d:
                failures.append(f"{name} is locked: the config's record {r} differs from the compiled {d}")
        elif r.locked:
            failures.append(f"{name}: the lock flag is set, the image's parameter is not locked")
        if r.value_bits != d.value_bits:
            changed.append(name)
    values.append(f"{len(records)} records of schema {manifest['schema_hash']} ({SECTION} section); "
                  f"config hash {mcfg.config_hash(cfg, (section.kind,)).hex()}")
    values.append(f"{locked} locked parameters equal to the compiled values and provenance")
    values.append("values changed from the compiled table: " + (", ".join(changed) if changed else "none"))
    if "error" in table:
        failures.append(table["error"])
    else:
        values.append(f"the firmware's reader (store::decode): {table['fault']}")
        if table["validator"] == "not run":
            failures.append(f"the firmware's reader refuses the records: {table['fault']}")
    return result(check, failures, values)


# ---- check: composition ----------------------------------------------------------------------------------------------

def check_composition(check, table):
    if "error" in table:
        return result(check, [table["error"]], [])
    values = [f"composition::validate: {table['validator']}", f"store::decode: {table['fault']}; verdict {table['verdict']}"]
    if table["validator"] == "not run":
        return result(check, [f"the validator did not run: the reader refused the file first ({table['fault']})"], values)
    if table["verdict"] != "accepted":
        return result(check, [f"composition::validate rejects the table: {table['validator']}"], values)
    return result(check, [], values)


# ---- check: ctest ----------------------------------------------------------------------------------------------------

def check_ctest(check, build, config, work_dir):
    junit = Path(work_dir) / "ctest.xml"
    env = dict(os.environ, **{ENV: str(Path(config).resolve())})
    cmd = ["ctest", "--test-dir", str(build), "-L", check["label"], "--output-junit", str(junit),
           "--test-output-size-passed", str(CTEST_OUTPUT_BYTES), "--test-output-size-failed", str(CTEST_OUTPUT_BYTES)]
    try:
        out = subprocess.run(cmd, env=env, capture_output=True, text=True, timeout=CTEST_TIMEOUT_S, check=False)
    except (OSError, subprocess.TimeoutExpired) as e:
        return result(check, [f"ctest did not run: {e}"], [])
    (Path(work_dir) / "ctest.out").write_text(out.stdout + out.stderr, encoding="utf-8")
    if not junit.exists():
        return result(check, [f"ctest wrote no result (exit {out.returncode}): {out.stderr.strip()[-400:]}"], [])
    cases = ET.parse(junit).getroot().findall("testcase")
    failures, values, ran = [], [], {}
    for c in cases:
        name, status = c.get("name"), c.get("status")
        text = c.findtext("system-out") or ""
        ok = status == "run" and c.find("failure") is None
        ran[name] = ok
        values.append(f"{name}: {'passed' if ok else 'FAILED (' + str(status) + ')'}")
        lines = [m.group(0).strip() for m in QF3_LINE.finditer(text)] + [m.group(1).strip() for m in L07_LINE.finditer(text)]
        values += [f"  {line}" for line in lines]
        if not ok:
            failed = [line for line in lines if "FAIL" in line]
            failures.append(f"{name} failed with the config injected (status {status})"
                            + (": " + " | ".join(failed) if failed else ""))
    if not cases:
        failures.append(f"no ctest is labelled {check['label']} in {rel(build)} (or none was built)")
    for t in check.get("tests", []):
        if t["name"] not in ran:
            failures.append(f"the required test {t['name']} did not run")
    for na in check.get("not_applicable", []):
        values.append(f"not applicable: {na['test']}: {na['reason']}")
    return result(check, failures, values)


# ---- check: sil --------------------------------------------------------------------------------------------------------

def check_sil(check, cfg, manifest, card, build, budget, work_dir):
    import sil_check  # noqa: PLC0415 (imports the gz tooling only when the check runs)

    if cfg is None:
        return result(check, ["the file is not a valid .mcfg: no run"], [])
    try:
        r = sil_check.run_check(manifest, cfg, card, Path(build) / PLUGIN_DIR, Path(work_dir) / "sil", budget)
    except sil_check.CheckError as e:
        return result(check, [str(e)], [])
    values = [f"form: {' '.join(check['form'].split())}",
              f"script: hover stick {r.get('hover_stick')!r} (derived from the card), step stick {r.get('step_stick')!r}, "
              f"s {r.get('settle_fraction')!r}"]
    for run in r["runs"]:
        values.append(f"run m = {run['m']}: {run['steps']} host steps, {run['ticks']} ticks, pad release tick "
                      f"{run['release_tick']}, log sha256 {run['log_sha256']}, gz {run.get('gz', 'UNKNOWN')}")
    failures = []
    if "error" in r:
        failures.append(r["error"])
    got = {p["name"]: p for p in r["predicates"]}
    for name in check["predicates"]:
        p = got.get(name)
        if p is None:
            if "error" not in r:
                failures.append(f"predicate {name} was not evaluated")
            continue
        values.append(f"{name}: {'PASS' if p['ok'] else 'FAIL'}: {p['detail']}")
        if not p["ok"]:
            failures.append(f"predicate {name} fails: {p['detail']}")
    return result(check, failures, values)


# ---- the report -----------------------------------------------------------------------------------------------------

def validate(config, image, build, card, budget, required, work_dir):
    """(report bytes, report id hex, report dict)."""
    build = Path(build)
    manifest_path = build / "generated" / PARAMETER_SET / "marv" / "params" / "params_manifest.json"
    tool = build / TABLE_TOOL
    sil_library = build / SIL_LIBRARY
    for p, what in ((manifest_path, "the parameter set's manifest"), (tool, "marv_validate_table"),
                    (sil_library, "the SIL library"), (Path(config), "the config"), (Path(image), "the image"),
                    (Path(card), "the card"), (Path(budget), "the budget"), (Path(required), "the required-check list")):
        if not p.exists():
            raise InputError(f"{what} is missing: {p}")
    manifest = mcfg.load_manifest(manifest_path)
    spec = yaml.safe_load(Path(required).read_text(encoding="utf-8"))
    data = Path(config).read_bytes()
    cfg, cfg_error = None, None
    try:
        cfg = mcfg.decode(data)
    except mcfg.McfgError as e:
        cfg_error = str(e)
    work_dir = Path(work_dir)
    work_dir.mkdir(parents=True, exist_ok=True)
    table = run_table_tool(tool, config)

    checks = []
    for check in spec["checks"]:
        progress(f"{check['id']}: {check['title']} ...")
        if check["id"] == "records":
            c = check_records(check, cfg, cfg_error, manifest, table)
        elif check["id"] == "composition":
            c = check_composition(check, table)
        elif check["id"] == "ctest":
            c = check_ctest(check, build, config, work_dir)
        elif check["id"] == "sil":
            c = check_sil(check, cfg, manifest, card, build, budget, work_dir)
        else:
            c = result(check, [f"the validator has no implementation of the required check {check['id']!r}"], [])
        progress(f"{check['id']}: {c['result']}" + (f" ({c['failures'][0]})" if c["failures"] else ""))
        checks.append(c)

    commit, dirty = git_state()
    image_sha = sha256_file(image)
    lib_sha = sha256_file(sil_library)
    schema_hash = mcfg.manifest_schema_hash(manifest)
    section = None if cfg is None else next((s for s in cfg.sections if s.schema_hash == schema_hash), None)
    doc = {
        "format": FORMAT,
        "verdict": "PASS" if checks and all(c["result"] == "PASS" for c in checks) else "FAIL",
        "image": {"file": Path(image).name, "sha256": image_sha, "kind": "sil" if image_sha == lib_sha else "file"},
        "sil_library_sha256": lib_sha,
        "config": {"file": Path(config).name, "file_sha256": hashlib.sha256(data).hexdigest(),
                   "config_hash": None if section is None else mcfg.config_hash(cfg, (section.kind,)).hex(),
                   "schema_hash": manifest["schema_hash"]},
        "card": {"file": rel(card), "sha256": sha256_file(card)},
        "budget": {"file": rel(budget), "sha256": sha256_file(budget)},
        "required_checks": {"file": rel(required), "sha256": sha256_file(required),
                            "signed": spec.get("signed")},
        "tools": {"commit": commit, "dirty": dirty, "python": platform.python_version(),
                  "ctest": tool_version("ctest", "--version"), "gz": gz8_version()},
        "checks": checks,
    }
    body = canonical(doc)
    return body, hashlib.sha256(body).hexdigest(), doc


def main(argv=None):
    ap = argparse.ArgumentParser(description="MARV configuration validator (module docstring)")
    ap.add_argument("--config", type=Path, required=True)
    ap.add_argument("--image", type=Path)
    ap.add_argument("--build", type=Path, default=DEFAULT_BUILD)
    ap.add_argument("--card", type=Path, default=DEFAULT_CARD)
    ap.add_argument("--budget", type=Path, default=DEFAULT_BUDGET)
    ap.add_argument("--required", type=Path, default=DEFAULT_REQUIRED)
    ap.add_argument("--work-dir", type=Path)
    ap.add_argument("--out", type=Path)
    a = ap.parse_args(argv)
    work = a.work_dir or Path(tempfile.mkdtemp(prefix="marv-validate-"))
    image = a.image or a.build / SIL_LIBRARY
    try:
        body, report_id, doc = validate(a.config, image, a.build, a.card, a.budget, a.required, work)
    except (InputError, mcfg.McfgError, OSError, yaml.YAMLError) as e:
        print(f"validate_config: cannot validate: {e}", file=sys.stderr)
        return 2
    out = a.out or work / "report.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_bytes(body)
    print(f"report: {out}")
    print(f"report_id: {report_id}")
    print(f"verdict: {doc['verdict']}")
    return 0 if doc["verdict"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
