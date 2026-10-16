#!/usr/bin/env python3
"""Blast-radius test selector: given a git diff, say which test groups the change can affect.

    uv run python ci/affected.py [--base REV] [--head REV] [--json] [--check-manifest]

The map is ci/blast_radius.yaml (its header says how a path is classified). A changed path that matches no full root, no
rule and no ignore glob selects everything (fail-safe). design/budget.yaml is diffed key by key against the base and a
changed key selects the files that name it. --check-manifest fails when the map has gone stale against
ci/run_ci.sh, ci/run_ci_gz.sh and the repository.
"""

from __future__ import annotations

import argparse
import ast
import json
import os
import posixpath
import re
import shlex
import subprocess
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent
MAP_PATH = ROOT / "ci" / "blast_radius.yaml"
RUN_CI = ROOT / "ci" / "run_ci.sh"
RUN_CI_GZ = ROOT / "ci" / "run_ci_gz.sh"
QUAD_TESTS = ROOT / "tests" / "regression" / "quad"

# `uv sync --frozen` is the step whose function is `uv`; it always runs and is not selectable.
ALWAYS_RUN_STEPS = {"uv"}
SELECTION_KEYS = ("ctest_labels", "run_ci_steps", "pytest_paths", "gz_groups")
RULE_KEYS = ("paths", "why") + SELECTION_KEYS
OPTIONAL_RULE_KEYS = ("like",)
TOP_KEYS = {"full_roots", "rules", "ignore", "ignore_first", "merge_only_steps", "step_groups", "budget_register",
            "card_files"}
REGISTER_KEYS = {"file", "text_fields", "scan_min_name_length", "scan_exclude", "mentions", "extra_readers", "any_change",
                 "structural"}
REGISTER_SELECTIONS = ("any_change", "structural")
SCHEMA_FIELDS = ("unit", "sigma", "method", "status", "check")
TEXT_SUFFIX_SKIP = {".png", ".jpg", ".jpeg", ".gif", ".pdf", ".bin", ".so", ".o", ".a", ".gz", ".zip", ".npz", ".npy", ".ico"}


# ------------------------------------------------------------------------------------------------ globs

def glob_to_regex(pattern: str) -> re.Pattern:
    """`*` and `?` stay inside one path component; `**/` is any number of directories; a trailing `**` is the rest."""
    out = []
    i, n = 0, len(pattern)
    while i < n:
        c = pattern[i]
        if c == "*":
            if pattern[i:i + 2] == "**":
                i += 2
                if i < n and pattern[i] == "/":
                    out.append("(?:.*/)?")
                    i += 1
                else:
                    out.append(".*")
                continue
            out.append("[^/]*")
        elif c == "?":
            out.append("[^/]")
        elif c == "[":
            j = pattern.find("]", i + 2)
            if j < 0:
                out.append(re.escape(c))
            else:
                body = pattern[i + 1:j]
                if body.startswith("!"):
                    body = "^" + body[1:]
                out.append("[" + body + "]")
                i = j
        else:
            out.append(re.escape(c))
        i += 1
    return re.compile("".join(out) + r"\Z")


def first_match(path: str, globs: list[str]) -> str | None:
    for g in globs:
        if glob_to_regex(g).match(path):
            return g
    return None


# ------------------------------------------------------------------------------------------------ the repository

def parse_steps(text: str) -> list[str]:
    """The step ids of ci/run_ci.sh: the function name of each `step "<title>" <function> [args]` line."""
    text = re.sub(r"\\\n", " ", text)
    ids = []
    for m in re.finditer(r'^[ \t]*step[ \t]+"(?:[^"\\]|\\.)*"[ \t]+(\S+)(?:[ \t]+(\S+))?', text, re.M):
        func, arg = m.group(1), m.group(2)
        ids.append(f"{func}:{arg}" if func == "host_preset" and arg else func)
    return ids


def parse_gz_groups(text: str) -> list[str]:
    return re.findall(r"^group_(\w+)\(\)", text, re.M)


def repo_universe(root: Path = ROOT) -> dict:
    """Everything a full selection holds, read from the repository."""
    steps = parse_steps((root / "ci" / "run_ci.sh").read_text())
    quad = root / "tests" / "regression" / "quad"
    labels = sorted(p.name for p in quad.glob("L[0-9][0-9]") if p.is_dir())
    nightly = re.search(r"^nightly_tools_tests=\(\n(.*?)^\)", (root / "ci" / "run_ci.sh").read_text(), re.M | re.S)
    return {
        "nightly_pytest_files": sorted(re.findall(r"^\s*(tests/\S+\.py)", nightly.group(1), re.M)) if nightly else [],
        "ctest_labels": labels,
        "run_ci_steps": sorted(set(steps) - ALWAYS_RUN_STEPS),
        "gz_groups": sorted(parse_gz_groups((root / "ci" / "run_ci_gz.sh").read_text())),
        "pytest_paths": [],  # filled from the map by select()
    }


def git(args: list[str], root: Path = ROOT) -> str:
    r = subprocess.run(["git", "-C", str(root), *args], capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(f"git {' '.join(args)}: {r.stderr.strip()}")
    return r.stdout


def resolve_base(base: str | None, root: Path = ROOT) -> str:
    if base:
        return git(["rev-parse", "--verify", base + "^{commit}"], root).strip()
    for master in ("master", "origin/master"):
        try:
            return git(["merge-base", "HEAD", master], root).strip()
        except RuntimeError:
            continue
    raise RuntimeError("cannot resolve the default base: no merge-base of HEAD with master or origin/master")


def changed_paths(base_sha: str, head: str | None, root: Path = ROOT) -> list[str]:
    """base..head, or (no head) base to the working tree: committed, staged, unstaged and untracked non-ignored files.

    Renames are reported as a deletion and an addition, so both names count.
    """
    if head:
        names = git(["diff", "--name-only", "--no-renames", "-z", base_sha, head], root)
    else:
        names = git(["diff", "--name-only", "--no-renames", "-z", base_sha], root)
        names += git(["ls-files", "--others", "--exclude-standard", "-z"], root)
    return sorted({p for p in names.split("\0") if p})


# ------------------------------------------------------------------------------------------------ the register

def file_at(rev: str | None, path: str, root: Path = ROOT) -> str | None:
    """The text of `path` at `rev` (the working tree when rev is None), or None when the file is not there."""
    if rev is None:
        f = root / path
        return f.read_text(encoding="utf-8") if f.is_file() else None
    try:
        return git(["show", f"{rev}:{path}"], root)
    except RuntimeError:
        return None


def is_number(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def register_scan_candidates(cfg: dict, mapping: dict, tracked: list[str]) -> list[str]:
    """The tracked files whose text names the keys of the register: not the register, not an ignored path, not a copy of values."""
    skip = list(cfg.get("scan_exclude") or []) + list(mapping.get("ignore") or []) + list(mapping.get("ignore_first") or [])
    return [p for p in tracked if p != cfg["file"] and Path(p).suffix not in TEXT_SUFFIX_SKIP and first_match(p, skip) is None]


def key_regex(key: str) -> re.Pattern:
    return re.compile(r"(?<![A-Za-z0-9_])" + re.escape(key) + r"(?![A-Za-z0-9_])")


def register_readers(keys: list[str], cfg: dict, mapping: dict, tracked: list[str], root: Path = ROOT
                     ) -> tuple[dict[str, list[str] | None], dict[str, str]]:
    """key -> its reader files (None: not mappable), and why not. A reader is a candidate file that names the key as a whole
    word, minus the `mentions`, plus the `extra_readers`."""
    min_len = int(cfg.get("scan_min_name_length", 1))
    mentions, extra = cfg.get("mentions") or [], cfg.get("extra_readers") or []
    candidates = register_scan_candidates(cfg, mapping, tracked) if keys else []
    texts: dict[str, str] = {}

    def text(p: str) -> str:
        if p not in texts:
            try:
                texts[p] = (root / p).read_text(encoding="utf-8", errors="ignore")
            except OSError:
                texts[p] = ""
        return texts[p]

    readers: dict[str, list[str] | None] = {}
    unmapped: dict[str, str] = {}
    for key in keys:
        if len(key) < min_len:
            readers[key] = None
            unmapped[key] = (f"its name has {len(key)} character(s), fewer than scan_min_name_length {min_len}: a scan by name "
                             f"cannot find its readers")
            continue
        rx = key_regex(key)
        hits = {p for p in candidates if rx.search(text(p))}
        hits = {p for p in hits if not any(key in m["keys"] and first_match(p, m["paths"]) for m in mentions)}
        for e in extra:
            if key in e["keys"]:
                hits.update(e["paths"])
        readers[key] = sorted(hits)
    return readers, unmapped


def consumed_card_files(mapping: dict, paths: list[str], tracked: list[str], root: Path = ROOT) -> set[str]:
    """Those of `paths` (card or profile files) that some tracked file consumes: names the path or the file name stem as a
    whole word, in a file that is not ignored or excluded; another card or profile only counts when it is itself consumed."""
    cfg = mapping.get("card_files") or {}
    skip = list(cfg.get("scan_exclude") or []) + list(mapping.get("ignore") or []) + list(mapping.get("ignore_first") or [])
    candidates = [p for p in tracked if Path(p).suffix not in TEXT_SUFFIX_SKIP and first_match(p, skip) is None]
    texts: dict[str, str] = {}

    def text(p: str) -> str:
        if p not in texts:
            try:
                texts[p] = (root / p).read_text(encoding="utf-8", errors="ignore")
            except OSError:
                texts[p] = ""
        return texts[p]

    def is_card(p: str) -> bool:
        return first_match(p, cfg.get("globs") or []) is not None

    memo: dict[str, bool] = {}

    def consumed(path: str, visiting: tuple[str, ...]) -> bool:
        if path in memo:
            return memo[path]
        rx = re.compile(r"(?<![A-Za-z0-9_])(?:" + re.escape(path) + "|" + re.escape(Path(path).stem) + r")(?![A-Za-z0-9_])")
        result = False
        for r in candidates:
            if r == path or r in visiting or not rx.search(text(r)):
                continue
            if not is_card(r) or consumed(r, visiting + (path,)):
                result = True
                break
        if not visiting:
            memo[path] = result
        return result

    return {p for p in paths if consumed(p, ())}


def register_change(mapping: dict, base_sha: str, head: str | None, changed: list[str], tracked: list[str],
                    root: Path = ROOT) -> dict | None:
    """The key-level diff of the design-budget register between base and head (the working tree when head is None), or None
    when the map has no register section or the file did not change. Keys: file, keys (key -> added/removed/changed),
    text_only (keys whose only changed fields are the text fields), structural (a parameter appeared, went or changed
    schema), readers and unmapped (register_readers), errors (reasons the diff could not be made)."""
    cfg = mapping.get("budget_register")
    if not cfg or cfg["file"] not in changed:
        return None
    info = {"file": cfg["file"], "keys": {}, "text_only": [], "structural": False, "readers": {}, "unmapped": {}, "errors": []}
    docs = []
    for rev in (base_sha, head):
        where = rev[:12] if rev else "the working tree"
        text = file_at(rev, cfg["file"], root)
        try:
            doc = yaml.safe_load(text) if text is not None else {}
        except yaml.YAMLError as e:
            info["errors"].append(f"cannot parse the register at {where}: {str(e).splitlines()[0]}")
            return info
        doc = {} if doc is None else doc
        if not isinstance(doc, dict) or any(not isinstance(v, dict) for v in doc.values()):
            info["errors"].append(f"the register at {where} is not a mapping of entries")
            return info
        docs.append(doc)
    old, new = docs
    text_fields = set(cfg.get("text_fields") or [])
    for key in sorted(set(old) | set(new)):
        a, b = old.get(key), new.get(key)
        if a == b:
            continue
        info["keys"][key] = "added" if a is None else "removed" if b is None else "changed"
        if a is not None and b is not None and {f for f in set(a) | set(b) if a.get(f) != b.get(f)} <= text_fields:
            info["text_only"].append(key)
        has_a, has_b = a is not None and is_number(a.get("value")), b is not None and is_number(b.get("value"))
        if has_a != has_b or (has_b and any(a.get(f) != b.get(f) for f in SCHEMA_FIELDS)):
            info["structural"] = True
    info["readers"], info["unmapped"] = register_readers(
        [k for k in info["keys"] if k not in info["text_only"]], cfg, mapping, tracked, root)
    return info


# ------------------------------------------------------------------------------------------------ the map

def load_map(path: Path = MAP_PATH) -> dict:
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def expand_steps(steps: list[str], groups: dict) -> list[str]:
    out = []
    for s in steps:
        if s.startswith("@"):
            out.extend(groups[s[1:]])
        else:
            out.append(s)
    return out


def rule_selection(rule: dict, mapping: dict) -> dict:
    sel = {k: list(rule.get(k) or []) for k in SELECTION_KEYS}
    sel["run_ci_steps"] = expand_steps(sel["run_ci_steps"], mapping.get("step_groups") or {})
    return sel


def map_pytest_paths(mapping: dict) -> list[str]:
    return sorted({p for r in mapping.get("rules") or [] for p in r.get("pytest_paths") or []})


def classify(path: str, mapping: dict):
    """('full', glob), ('rules', [rule, ...]), ('ignore', glob) or ('unmapped', None); ignore_first wins over all."""
    g = first_match(path, mapping.get("ignore_first") or [])
    if g is not None:
        return "ignore", g
    g = first_match(path, mapping.get("full_roots") or [])
    if g is not None:
        return "full", g
    hits = [r for r in mapping.get("rules") or [] if first_match(path, r["paths"]) is not None]
    if hits:
        return "rules", hits
    g = first_match(path, mapping.get("ignore") or [])
    if g is not None:
        return "ignore", g
    return "unmapped", None


def select(changed: list[str], mapping: dict, universe: dict, base: str = "", forced_full: list[str] | None = None,
           pyindex=None, per_push: bool = True, notes: list[str] | None = None, register: dict | None = None,
           consumed: set[str] | None = None) -> dict:
    """The selection for a list of changed paths.

    With a PythonIndex, a changed Python file under tools/ or tests/regression/ contributes the test files whose import or
    spawn closure holds it instead of its rule's pytest directories (a directory stays when the closure analysis cannot
    resolve something in it). `notes` collects what the caller should tell the user.

    `register` is register_change() of the design-budget register: when the register file is among the changed paths and
    `register` describes it, the file selects its any_change selection, its structural selection when a parameter appeared
    or went, and each changed key's readers, instead of the full root it is without that description. A rule's `like` paths
    are selected as if they had changed too. A card or profile file (map section card_files) selects what the committed card
    selects when it is in `consumed` (consumed_card_files; None: unknown, so consumed).
    """
    notes = notes if notes is not None else []
    full_reasons = list(forced_full or [])
    chosen = {k: set() for k in SELECTION_KEYS}
    reg_cfg = mapping.get("budget_register") or {}
    work: list[tuple[str, str]] = []
    for path in sorted(set(changed)):
        if register and reg_cfg and path == reg_cfg.get("file") == register.get("file"):
            expand_register(register, reg_cfg, mapping, chosen, full_reasons, work, notes)
        else:
            work.append((path, ""))
            card_cfg = mapping.get("card_files") or {}
            if card_cfg and first_match(path, card_cfg["globs"]) is not None and path != card_cfg["consumed_like"] \
                    and (consumed is None or path in consumed):
                work.append((card_cfg["consumed_like"], f" ({path} is consumed)"))
    seen: set[str] = set()
    i = 0
    while i < len(work):
        path, origin = work[i]
        i += 1
        if path in seen:
            continue
        seen.add(path)
        kind, what = classify(path, mapping)
        if kind == "full":
            full_reasons.append(f"{path}: {what}{origin}")
        elif kind == "unmapped":
            full_reasons.append(f"{path}: unmapped{origin}")
        elif kind == "rules":
            file_level = pyindex.files_for(path) if pyindex and is_python_candidate(path) else None
            if pyindex and is_python_candidate(path) and file_level is None:
                notes.append(f"{path}: closure analysis unavailable ({pyindex.why_not(path)}); pytest selected at directory level")
            for rule in what:
                work.extend((p, f" (like {path})") for p in rule.get("like") or [])
                for k, v in rule_selection(rule, mapping).items():
                    if k == "pytest_paths" and file_level is not None:
                        files, unresolved = file_level
                        rule_dirs, v = set(v), set(files)
                        for d in sorted(rule_dirs & set(unresolved)):
                            for f, why in unresolved[d]:
                                whole_dir = Path(f).name == "conftest.py"
                                v.add(d if whole_dir else f)
                                notes.append(f"{path}: {f} has {why}, which the closure analysis cannot follow; "
                                             f"{'its directory' if whole_dir else 'it'} is selected")
                    chosen[k].update(v)
    dirs = {p for p in chosen["pytest_paths"] if not p.endswith(".py")}
    files = {p for p in chosen["pytest_paths"] if p.endswith(".py") and not any(p.startswith(d + "/") for d in dirs)}
    nightly = set(universe.get("nightly_pytest_files") or [])
    if per_push and not full_reasons and files & nightly:
        notes.append(f"{sorted(files & nightly)} run in the full run only (nightly list), not selected")
        files -= nightly
    chosen["pytest_paths"] = dirs | files
    result = {"base": base, "changed": sorted(set(changed)), "full": bool(full_reasons),
              "full_reasons": sorted(set(full_reasons))}
    everything = {
        "ctest_labels": universe["ctest_labels"],
        "run_ci_steps": universe["run_ci_steps"],
        "pytest_paths": sorted(set(universe.get("pytest_paths") or []) | set(map_pytest_paths(mapping))),
        "gz_groups": universe["gz_groups"],
    }
    for k in ("ctest_labels", "run_ci_steps", "pytest_paths", "gz_groups"):
        result[k] = sorted(set(everything[k]) if result["full"] else chosen[k])
    return result


def expand_register(register: dict, cfg: dict, mapping: dict, chosen: dict, full_reasons: list[str],
                    work: list[tuple[str, str]], notes: list[str]) -> None:
    """Add what a described change of the register file selects: chosen (selections), full_reasons, work (reader paths)."""
    file = cfg["file"]
    for err in register.get("errors") or []:
        full_reasons.append(f"{file}: {err}")
    parts = ["any_change"] + (["structural"] if register.get("structural") else [])
    for part in parts:
        for k, v in rule_selection(cfg[part], mapping).items():
            chosen[k].update(v)
    keys = register.get("keys") or {}
    text_only = [k for k in keys if k in (register.get("text_only") or [])]
    if keys:
        notes.append(f"{file}: changed keys " + ", ".join(
            f"{k} ({keys[k]}{', text only' if k in text_only else ''})" for k in sorted(keys)))
    for key in sorted(keys):
        if key in text_only:
            continue
        readers = (register.get("readers") or {}).get(key)
        if readers is None:
            full_reasons.append(f"{file}: key {key}: {(register.get('unmapped') or {}).get(key, 'no readers known')}")
            continue
        notes.append(f"{file}: key {key} is read by {', '.join(readers) if readers else 'no tracked file'}")
        work.extend((r, f" (reads budget key {key})") for r in readers)


# ------------------------------------------------------------------------------------------------ the manifest check

def check_manifest(mapping: dict, universe: dict, root: Path = ROOT) -> list[str]:
    errors = []
    unknown = set(mapping) - TOP_KEYS
    if unknown:
        errors.append(f"unknown top-level keys: {sorted(unknown)}")
    groups = mapping.get("step_groups") or {}
    merge_only = set(mapping.get("merge_only_steps") or [])
    steps_in_ci = set(universe["run_ci_steps"])

    for key in ("full_roots", "ignore"):
        if not isinstance(mapping.get(key), list) or not mapping.get(key):
            errors.append(f"{key}: missing or empty")

    selected_steps = set()
    selected_pytest = set()
    register_errors, pseudo_rules = check_register_section(mapping, root)
    errors += register_errors + check_card_files_section(mapping, root)
    for i, rule in enumerate((mapping.get("rules") or []) + pseudo_rules):
        name = f"rule {i} ({', '.join(rule.get('paths') or ['no paths'])})" if "_name" not in rule else rule["_name"]
        for k in RULE_KEYS:
            if k not in rule:
                errors.append(f"{name}: missing key {k}")
        for k in sorted(set(rule) - set(RULE_KEYS) - set(OPTIONAL_RULE_KEYS) - {"_name"}):
            errors.append(f"{name}: unknown key {k}")
        for p in rule.get("like") or []:
            if not (root / p).exists():
                errors.append(f"{name}: like path '{p}' does not exist")
        if not rule.get("why"):
            errors.append(f"{name}: empty why")
        for k in SELECTION_KEYS + ("paths",):
            if not isinstance(rule.get(k), list):
                errors.append(f"{name}: {k} is not a list")
        if not rule.get("paths"):
            errors.append(f"{name}: no paths")
        for s in rule.get("run_ci_steps") or []:
            if s.startswith("@"):
                if s[1:] not in groups:
                    errors.append(f"{name}: unknown step group {s}")
                else:
                    selected_steps.update(groups[s[1:]])
        for s in rule.get("run_ci_steps") or []:
            if s.startswith("@"):
                continue
            selected_steps.add(s)
            if s not in steps_in_ci:
                errors.append(f"{name}: run_ci_steps id '{s}' is not a step of ci/run_ci.sh")
        for p in rule.get("pytest_paths") or []:
            selected_pytest.add(p)
            if not (root / p).exists():
                errors.append(f"{name}: pytest path '{p}' does not exist")
        for g in rule.get("gz_groups") or []:
            if g not in universe["gz_groups"]:
                errors.append(f"{name}: gz group '{g}' is not a group of ci/run_ci_gz.sh ({universe['gz_groups']})")
        for lab in rule.get("ctest_labels") or []:
            if lab not in universe["ctest_labels"]:
                errors.append(f"{name}: ctest label '{lab}' has no tests/regression/quad/{lab} directory")
    for gname, steps in groups.items():
        for s in steps:
            selected_steps.add(s)
            if s not in steps_in_ci:
                errors.append(f"step_groups.{gname}: id '{s}' is not a step of ci/run_ci.sh")

    for s in sorted(steps_in_ci - selected_steps - merge_only):
        errors.append(f"step '{s}' of ci/run_ci.sh is selected by no rule and is not in merge_only_steps")

    # A tools-test directory that no rule names and that no gz group runs would never be selected.
    gz_text = (root / "ci" / "run_ci_gz.sh").read_text()
    for d in sorted(QUAD_TESTS.glob("L[0-9][0-9]/tools")) if root == ROOT else []:
        rel = d.relative_to(root).as_posix()
        if any(d.glob("test_*.py")) and rel not in selected_pytest and rel not in gz_text:
            errors.append(f"tools-test directory {rel} is named by no rule and no gz group runs it")

    for key in ("full_roots", "ignore", "ignore_first"):
        for g in mapping.get(key) or []:
            glob_to_regex(g)
    if not isinstance(mapping.get("ignore_first"), list) or not mapping.get("ignore_first"):
        errors.append("ignore_first: missing or empty")
    return errors


def check_card_files_section(mapping: dict, root: Path = ROOT) -> list[str]:
    cfg = mapping.get("card_files")
    if not isinstance(cfg, dict):
        return ["card_files: section missing"]
    errors = []
    for k in sorted({"globs", "consumed_like", "scan_exclude"} - set(cfg)):
        errors.append(f"card_files: missing key {k}")
    for g in list(cfg.get("globs") or []) + list(cfg.get("scan_exclude") or []):
        glob_to_regex(g)
    like = cfg.get("consumed_like")
    if not like or not (root / like).is_file():
        errors.append(f"card_files: consumed_like '{like}' does not exist")
    elif not any(like in (r.get("paths") or []) for r in mapping.get("rules") or []):
        errors.append(f"card_files: consumed_like '{like}' has no rule of its own")
    return errors


def check_register_section(mapping: dict, root: Path = ROOT) -> tuple[list[str], list[dict]]:
    """Errors in the budget_register section, and its two selections as rules (so the rule checks validate them)."""
    reg = mapping.get("budget_register")
    if reg is None:
        return ["budget_register: section missing"], []
    errors = []
    if not isinstance(reg, dict):
        return ["budget_register: not a mapping"], []
    for k in sorted(REGISTER_KEYS - set(reg)):
        errors.append(f"budget_register: missing key {k}")
    for k in sorted(set(reg) - REGISTER_KEYS):
        errors.append(f"budget_register: unknown key {k}")
    file = reg.get("file")
    keys: set[str] = set()
    if not file or not (root / file).is_file():
        errors.append(f"budget_register: file '{file}' does not exist")
    else:
        try:
            keys = set(yaml.safe_load((root / file).read_text(encoding="utf-8")) or {})
        except yaml.YAMLError as e:
            errors.append(f"budget_register: {file} does not parse: {e}")
    if not isinstance(reg.get("scan_min_name_length"), int):
        errors.append("budget_register: scan_min_name_length is not an integer")
    for g in list(reg.get("scan_exclude") or []):
        glob_to_regex(g)
    scan_keys = [k for k in keys if len(k) >= int(reg.get("scan_min_name_length") or 1)]
    for section in ("mentions", "extra_readers"):
        for j, entry in enumerate(reg.get(section) or []):
            where = f"budget_register.{section}[{j}]"
            for f in ("keys", "paths", "why"):
                if not entry.get(f):
                    errors.append(f"{where}: missing or empty {f}")
            for key in entry.get("keys") or []:
                if keys and key not in keys:
                    errors.append(f"{where}: key '{key}' is not in {file}")
            for pth in entry.get("paths") or []:
                if section == "extra_readers" and not (root / pth).is_file():
                    errors.append(f"{where}: path '{pth}' does not exist")
                if section == "mentions" and not any(c in pth for c in "*?[") and (root / pth).is_file():
                    text = (root / pth).read_text(encoding="utf-8", errors="ignore")
                    if not any(key_regex(key).search(text) for key in entry.get("keys") or [] if key in scan_keys):
                        errors.append(f"{where}: '{pth}' no longer names any of {entry.get('keys')}: remove the entry")
    pseudo = []
    for k in REGISTER_SELECTIONS:
        if isinstance(reg.get(k), dict):
            pseudo.append({**reg[k], "paths": [file or "?"], "_name": f"budget_register.{k}"})
    return errors, pseudo


def map_notes(mapping: dict, tracked: list[str]) -> list[str]:
    """Information only: tracked files the map does not know (they would select everything) and globs that match nothing."""
    notes = []
    unmapped = [p for p in tracked if classify(p, mapping)[0] == "unmapped"]
    if unmapped:
        notes.append(f"{len(unmapped)} tracked files are unmapped and would select everything: {unmapped[:5]}")
    every = [g for g in (mapping.get("full_roots") or [])] + [g for r in mapping.get("rules") or [] for g in r["paths"]]
    dead = [g for g in every if not any(glob_to_regex(g).match(p) for p in tracked)]
    if dead:
        notes.append(f"globs that match no tracked file: {dead}")
    return notes


# ------------------------------------------------------------------------------------------------ cross-checks
# The map is written by hand from what was read; these checks recompute the dependencies mechanically and fail when the
# map selects less than they find. Not found by them (so still a judgement of the map): runtime reads in C++ sources,
# paths built from names the checks cannot resolve, links that are not include paths, and imports across test directories.

TOOLS_DIRS = ("card", "sim", "ci", "refdata", "gen")
CXX_SUFFIXES = {".h", ".hpp", ".hh", ".cpp", ".cc", ".inl"}
REPO_TOP = ("tests", "scenarios", "design", "sensors", "vehicles", "fw", "sim", "tools", "ci")
UNKNOWN = "<?>"


class Selector:
    """select() for one path, remembered."""

    def __init__(self, mapping: dict, universe: dict, pyindex=None, consumed: set[str] | None = None):
        self.mapping, self.universe, self.pyindex, self.consumed, self.cache = mapping, universe, pyindex, consumed, {}

    def __call__(self, path: str) -> dict:
        if path not in self.cache:
            self.cache[path] = select([path], self.mapping, self.universe, pyindex=self.pyindex, per_push=False,
                                      consumed=self.consumed)
        return self.cache[path]


def parse_gz_test_paths(text: str) -> dict[str, set[str]]:
    """tests path -> the gz groups that run it, from the functions and group_* lists of ci/run_ci_gz.sh."""
    funcs = {m.group(1): m.group(2) for m in re.finditer(r"^(\w+)\(\) \{\n(.*?)^\}", text, re.M | re.S)}
    out: dict[str, set[str]] = {}
    for g in parse_gz_groups(text):
        for step_fn in re.findall(r"step \w+ (\w+)", funcs.get(f"group_{g}", "")):
            for p in re.findall(r"tests/regression/[\w/.-]+", funcs.get(step_fn, "")):
                out.setdefault(p, set()).add(g)
    return out


def reader_groups(rel: str, universe: dict, gz_paths: dict[str, set[str]]) -> dict | None:
    """What must be selected when `rel` (a test .py file) changes or one of its inputs does: pytest dir, gz groups or label."""
    pytest_dir = next((p for p in universe["pytest_paths"] if rel.startswith(p + "/")), None)
    groups = set()
    for p, gs in gz_paths.items():
        if rel == p or rel.startswith(p + "/"):
            groups |= gs
    if not groups:
        parent = rel.rsplit("/", 1)[0] + "/"
        for p, gs in gz_paths.items():
            if p.startswith(parent):
                groups |= gs
    if pytest_dir or groups:
        return {"pytest": pytest_dir, "groups": groups, "label": None, "file": rel}
    m = re.match(r"tests/regression/quad/(L\d\d)/", rel)
    if m and "/results/" not in rel:
        return {"pytest": None, "groups": set(), "label": m.group(1), "file": rel}
    return None


def lacks(sel: dict, need: dict) -> str | None:
    if sel["full"]:
        return None
    whole_dir = need["file"] is not None and need["file"].endswith("conftest.py")  # a conftest applies to every test beside it
    if need["pytest"] and not any(p == need["pytest"] or p == need["file"] or (whole_dir and p.startswith(need["pytest"] + "/"))
                                  for p in sel["pytest_paths"]):
        return f"pytest {need['pytest']}"
    missing = sorted(need["groups"] - set(sel["gz_groups"]))
    if missing:
        return f"gz group {','.join(missing)}"
    if need["label"] and need["label"] not in sel["ctest_labels"]:
        return f"label {need['label']}"
    return None


def cxx_dependencies(entries: list[dict], root: Path) -> dict[tuple[str, str], str]:
    """(layer label, repo path of a source file or an include directory) -> the source file that needs it."""
    need: dict[tuple[str, str], str] = {}
    for e in entries:
        try:
            src = Path(e["file"]).resolve().relative_to(root).as_posix()
        except ValueError:
            continue
        m = re.match(r"tests/regression/quad/(L\d\d)/", src)
        if not m:
            continue
        label = m.group(1)
        args = e.get("arguments") or shlex.split(e["command"])
        paths = [src]
        for i, a in enumerate(args):
            for flag in ("-I", "-isystem", "-iquote", "-idirafter"):
                if a == flag and i + 1 < len(args):
                    paths.append(args[i + 1])
                elif a.startswith(flag) and a != flag:
                    paths.append(a[len(flag):])
        for p in paths[1:] + paths[:1]:
            try:
                rel = (Path(e["directory"]) / p).resolve().relative_to(root).as_posix()
            except ValueError:
                continue
            if rel.startswith(("fw/", "sim/", "tests/regression/")):
                need.setdefault((label, rel), src)
    return need


def cross_check_cxx(select_one: Selector, tracked: list[str], cc_path: Path, root: Path) -> tuple[list[str], list[str]]:
    """Every source file and include directory a test target compiles with must select that target's layer label."""
    if not cc_path.exists():
        return [], ["no host-debug compile_commands.json, C++ cross-check skipped"]
    errors = []
    for (label, rel), src in sorted(cxx_dependencies(json.loads(cc_path.read_text()), root).items()):
        files = [p for p in tracked if (p == rel or p.startswith(rel + "/")) and Path(p).suffix in CXX_SUFFIXES]
        for f in files:
            sel = select_one(f)
            if not sel["full"] and label not in sel["ctest_labels"]:
                errors.append(f"{rel} → missing label {label} (from {src}; e.g. {f})")
                break
    return errors, []


# --- Python

def tool_module_index(tracked: list[str]) -> dict[str, list[str]]:
    """Importable modules by stem: tools/<dir>/*.py, and the helper modules of the test directories (oracles, runners) that
    other tests import by putting their directory on sys.path. Test files and conftest.py are not modules others import."""
    index: dict[str, list[str]] = {}
    for p in tracked:
        stem = Path(p).stem
        is_tool = re.fullmatch(r"tools/(?:%s)/\w+\.py" % "|".join(TOOLS_DIRS), p)
        is_helper = (re.fullmatch(r"tests/regression/quad/L\d\d/.+\.py", p)
                     and not stem.startswith("test_") and stem != "conftest")
        if is_tool or is_helper:
            index.setdefault(stem, []).append(p)
    return index


DYNAMIC_CALLS = {"exec", "__import__", "import_module", "run_path", "run_module", "spec_from_file_location", "load_source"}
SPAWN_CALLS = {"run", "Popen", "check_call", "check_output", "call"}


def is_python_candidate(path: str) -> bool:
    """A Python file whose pytest selection can be made at file level."""
    return path.endswith(".py") and path.startswith(("tools/", "tests/regression/"))


def unresolved_constructs(tree) -> list[str]:
    """Constructs through which a test could reach a Python file the import closure does not see: dynamic imports and
    `exec`, and Python subprocesses whose script is not a path the checks can see written in the file."""
    names_with_py = set()
    for n in tree.body:
        if isinstance(n, ast.Assign) and any(isinstance(c, ast.Constant) and isinstance(c.value, str) and c.value.endswith(".py")
                                             for c in ast.walk(n.value)):
            names_with_py.update(t.id for t in n.targets if isinstance(t, ast.Name))

    def visible(node) -> bool:
        return any((isinstance(c, ast.Constant) and isinstance(c.value, str) and c.value.endswith(".py"))
                   or (isinstance(c, ast.Name) and c.id in names_with_py) for c in ast.walk(node))

    enclosing = {}
    for fn in ast.walk(tree):
        if isinstance(fn, (ast.FunctionDef, ast.AsyncFunctionDef)):
            for c in ast.walk(fn):
                enclosing[id(c)] = fn

    def param_of(node, fn) -> int | None:
        """Position in fn's arguments of the parameter `node` passes through (`*map(str, args)`, `str(script)`, `script`)."""
        while isinstance(node, (ast.Starred, ast.Call)):
            node = node.value if isinstance(node, ast.Starred) else (node.args[-1] if node.args else None)
        if not isinstance(node, ast.Name) or fn is None:
            return None
        names = [a.arg for a in fn.args.posonlyargs + fn.args.args]
        if node.id in names:
            return names.index(node.id)
        return len(names) if fn.args.vararg and node.id == fn.args.vararg.arg else None

    def callers_resolved(fn, pos, depth=0) -> bool:
        """Every call of `fn` passes, at `pos`, a script the file names (or forwards its own parameter, checked in turn)."""
        calls = [c for c in ast.walk(tree) if isinstance(c, ast.Call) and getattr(c.func, "id", "") == fn.name]
        if not calls or depth > 3:
            return False
        for c in calls:
            if isinstance(c.args[pos] if pos < len(c.args) else None, ast.Constant):
                continue
            if pos < len(c.args) and not isinstance(c.args[pos], ast.Starred) and visible(c.args[pos]):
                continue
            outer = enclosing.get(id(c))
            last = c.args[-1] if c.args else None
            if pos == len(c.args) - 1 and isinstance(last, ast.Starred) and isinstance(last.value, ast.Name) and outer is not None:
                lists = [x.value for x in ast.walk(outer) if isinstance(x, ast.Assign) and isinstance(x.value, (ast.List, ast.Tuple))
                         and any(isinstance(t, ast.Name) and t.id == last.value.id for t in x.targets)]
                if lists and all(v.elts and (isinstance(v.elts[0], ast.Constant) or visible(v.elts[0])) for v in lists):
                    continue
            forwarded = param_of(c.args[pos], outer) if pos < len(c.args) else None
            starred_from = param_of(c.args[-1], outer) if c.args and isinstance(c.args[-1], ast.Starred) and pos >= len(c.args) - 1 else None
            fwd = forwarded if forwarded is not None else starred_from
            if outer is None or fwd is None or not callers_resolved(outer, fwd, depth + 1):
                return False
        return True

    found = []
    for n in ast.walk(tree):
        if not isinstance(n, ast.Call):
            continue
        name = getattr(n.func, "id", getattr(n.func, "attr", ""))
        if name in DYNAMIC_CALLS:
            literal_module = name in ("import_module", "__import__", "run_module") and n.args and isinstance(n.args[0], ast.Constant)
            if name == "exec" or not (literal_module or visible(n)):
                found.append(f"{name}() at line {n.lineno}")
        elif name in SPAWN_CALLS and isinstance(n.func, ast.Attribute) and getattr(n.func.value, "id", "") == "subprocess":
            argv = n.args[0] if n.args else None
            if isinstance(argv, (ast.List, ast.Tuple)):
                for i, e in enumerate(argv.elts[:-1]):
                    is_python = (isinstance(e, ast.Attribute) and e.attr == "executable") or \
                                (isinstance(e, ast.Constant) and e.value in ("python", "python3"))
                    script = argv.elts[i + 1]
                    if is_python and not (isinstance(script, ast.Constant) or visible(script)):
                        fn = enclosing.get(id(n))
                        pos = param_of(script, fn)
                        if pos is not None and callers_resolved(fn, pos):
                            break
                        found.append(f"a Python subprocess with a computed script at line {n.lineno}")
                        break
    return found


def python_refs(source: str, index: dict[str, list[str]]) -> set[str]:
    """Tools files a Python source imports (the tests put the tools directories on sys.path) or names as a script."""
    return python_refs_ex(source, index)[0]


def python_refs_ex(source: str, index: dict[str, list[str]]) -> tuple[set[str], list[str]]:
    """(referenced files, reasons the references may be incomplete)."""
    try:
        tree = ast.parse(source)
    except SyntaxError as e:
        return set(), [f"a parse error ({e.msg}, line {e.lineno})"]
    stems = set()
    for n in ast.walk(tree):
        if isinstance(n, ast.Import):
            stems.update(a.name.split(".")[0] for a in n.names)
        elif isinstance(n, ast.ImportFrom):
            if n.module:
                stems.add(n.module.split(".")[0])
            stems.update(a.name for a in n.names)
        elif isinstance(n, ast.Constant) and isinstance(n.value, str) and n.value.endswith(".py"):
            stems.add(Path(n.value).stem)
    found = set()
    for s in stems & set(index):
        found.update(index[s])
    return found, unresolved_constructs(tree)


class PythonIndex:
    """Which per-push test files reach each Python file (import closure plus named scripts), for file-level selection."""

    def __init__(self, root: Path, tracked: list[str], universe: dict):
        self.root = root
        self.modules = tool_module_index(tracked)
        self.known = {p for p in tracked if p.endswith(".py") and (root / p).exists()}
        self.analysis: dict[str, tuple[set[str], list[str]]] = {}
        self.tests: set[str] = set()
        self.reverse: dict[str, set[str]] = {}
        self.unresolved: dict[str, list[str]] = {}
        for d in universe["pytest_paths"]:
            tests = [p for p in self.known if p.startswith(d + "/") and p.count("/") == d.count("/") + 1
                     and Path(p).name.startswith("test_")]
            conftest = f"{d}/conftest.py"
            for t in tests:
                self.tests.add(t)
                roots = [t] + ([conftest] if conftest in self.known else [])
                seen = set(roots)
                stack = list(roots)
                while stack:
                    f = stack.pop()
                    refs, why = self.refs(f)
                    if f in roots and why:
                        self.unresolved.setdefault(d, []).append((f, why[0]))
                    for g in refs - seen:
                        seen.add(g)
                        stack.append(g)
                for f in seen:
                    self.reverse.setdefault(f, set()).add(t)

    def refs(self, f: str) -> tuple[set[str], list[str]]:
        if f not in self.analysis:
            source = (self.root / f).read_text(encoding="utf-8")
            refs, why = python_refs_ex(source, self.modules)
            refs |= {path for path, _, _ in python_reads(source) if path in self.known and path != f}
            self.analysis[f] = (refs, why)
        return self.analysis[f]

    def why_not(self, path: str) -> str:
        if path not in self.known:
            return "not a tracked file on disk"
        return self.refs(path)[1][0] if self.refs(path)[1] else "a conftest.py applies to its whole directory"

    def files_for(self, path: str):
        """(test files, {pytest dir: [reasons]}) for a changed Python file, or None to select by directory."""
        if path not in self.known or Path(path).name == "conftest.py":
            return None
        if self.refs(path)[1] and any(r.startswith("a parse error") for r in self.refs(path)[1]):
            return None
        if Path(path).name.startswith("test_"):
            return ({path} if path in self.tests else set()) | self.reverse.get(path, set()), {}
        return set(self.reverse.get(path, set())), self.unresolved


def tool_closure(roots: set[str], root: Path, index: dict[str, list[str]]) -> dict[str, str]:
    """Transitive tools files of `roots` (tools files): file -> the root that reaches it."""
    seen: dict[str, str] = {}
    stack = [(r, r) for r in roots]
    while stack:
        f, origin = stack.pop()
        for g in python_refs((root / f).read_text(encoding="utf-8"), index):
            if g not in seen:
                seen[g] = origin
                stack.append((g, origin))
    return seen


def test_python_files(tracked: list[str], universe: dict, gz_paths: dict[str, set[str]]) -> dict[str, dict]:
    """The Python files of the pytest directories and of the gz-run directories, with what must select them."""
    out = {}
    for p in tracked:
        if not p.endswith(".py") or not p.startswith("tests/"):
            continue
        need = reader_groups(p, universe, gz_paths)
        if need and (need["pytest"] or need["groups"]):
            out[p] = need
    return out


def cross_check_python(select_one: Selector, tracked: list[str], universe: dict, root: Path,
                       gz_paths: dict[str, set[str]]) -> list[str]:
    index = tool_module_index(tracked)
    errors = []
    reported = set()
    for test, need in sorted(test_python_files(tracked, universe, gz_paths).items()):
        direct = python_refs((root / test).read_text(encoding="utf-8"), index)
        for tool, origin in [(d, d) for d in direct] + list(tool_closure(direct, root, index).items()):
            missing = lacks(select_one(tool), need)
            if missing and (tool, missing) not in reported:
                reported.add((tool, missing))
                errors.append(f"{tool} → missing {missing} (imported by {test}" + (f" via {origin})" if origin != tool else ")"))
    flatten = "tools/card/flatten.py"
    if flatten in tracked:
        for tool in sorted(tool_closure({flatten}, root, index)):
            sel = select_one(tool)
            if not sel["full"] and (set(universe["ctest_labels"]) - set(sel["ctest_labels"])
                                    or "product_table_host" not in sel["run_ci_steps"]):
                errors.append(f"{tool} → in the import closure of {flatten} but does not select every ctest label and product_table_host")
    return errors


# --- runtime reads

def eval_parts(node, env: dict) -> list[str] | None:
    """The path parts of an expression like ROOT / "tests" / "x", UNKNOWN for a base that cannot be resolved."""
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return [node.value]
    if isinstance(node, ast.Name):
        return env.get(node.id, [UNKNOWN])
    if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Div):
        left, right = eval_parts(node.left, env), eval_parts(node.right, env)
        return None if left is None or right is None else left + right
    if isinstance(node, ast.JoinedStr):
        return ["".join(v.value if isinstance(v, ast.Constant) else "*" for v in node.values)]
    if isinstance(node, ast.Call):
        name = getattr(node.func, "id", getattr(node.func, "attr", ""))
        if name in ("Path", "str", "join", "joinpath", "resolve") and node.args:
            parts: list[str] = []
            for a in node.args:
                p = eval_parts(a, env)
                if p is None:
                    return None
                parts += p
            return parts
        return [UNKNOWN]
    if isinstance(node, (ast.Attribute, ast.Subscript)):
        return [UNKNOWN]
    return None


def repo_path_of(parts: list[str] | None) -> str | None:
    if not parts:
        return None
    while UNKNOWN in parts:
        parts = parts[parts.index(UNKNOWN) + 1:] if parts.count(UNKNOWN) == 1 else parts[len(parts) - parts[::-1].index(UNKNOWN):]
    path = posixpath.normpath("/".join(p.strip("/") for p in parts if p))
    return path if path.split("/")[0] in REPO_TOP and "/" in path else None


def python_reads(source: str) -> list[tuple[str, int, bool]]:
    """(repo path, line, built with `/`) of every path expression in a Python source that resolves to a repository path.
    A plain string naming a directory is usually an argument to a checker, not a read, so only built paths count as such."""
    try:
        tree = ast.parse(source)
    except SyntaxError:
        return []
    env: dict[str, list[str]] = {}
    for n in tree.body:
        if isinstance(n, ast.Assign) and len(n.targets) == 1 and isinstance(n.targets[0], ast.Name):
            parts = eval_parts(n.value, env)
            if parts is not None:
                env[n.targets[0].id] = parts
    reads = []
    inner = set()
    for n in ast.walk(tree):
        if isinstance(n, ast.BinOp) and isinstance(n.op, ast.Div):
            inner.update(id(c) for c in ast.walk(n) if c is not n and isinstance(c, ast.BinOp))
    for n in ast.walk(tree):
        if (isinstance(n, ast.BinOp) and isinstance(n.op, ast.Div) and id(n) not in inner) or \
           (isinstance(n, ast.Constant) and isinstance(n.value, str) and "/" in n.value and not re.search(r"\s", n.value)):
            p = repo_path_of(eval_parts(n, env))
            if p:
                reads.append((p, n.lineno, isinstance(n, ast.BinOp)))
    return reads


def cmake_reads(text: str, cmake_rel: str) -> list[tuple[str, int, bool]]:
    here = posixpath.dirname(cmake_rel)
    reads = []
    for lineno, line in enumerate(text.splitlines(), 1):
        line = line.split("#", 1)[0]
        for m in re.finditer(r"(\$\{(?:PROJECT_SOURCE_DIR|CMAKE_SOURCE_DIR|CMAKE_CURRENT_SOURCE_DIR)\}/|(?<![\w/.-])(?=(?:tests|scenarios|design|sensors|vehicles|fw|sim|tools)/))([^\s\"')$]+)", line):
            base = here if "CURRENT" in m.group(1) else ""
            path = posixpath.normpath(posixpath.join(base, m.group(2)))
            if path.split("/")[0] in REPO_TOP and "/" in path:
                reads.append((path, lineno, True))
    return reads


def read_targets(path: str, tracked: list[str], is_chain: bool) -> list[str]:
    """The tracked files a read names: a file exactly, a glob, or the C++ files under a fw/ or sim/ directory (an include path
    or a source a test compiles). Other directory reads are search paths or listings and are left to the other checks, as are
    tools/ files (imports) and CMakeLists.txt files."""
    if path.startswith("tools/") or path.endswith("CMakeLists.txt"):
        return []
    if any(c in path for c in "*?["):
        rx = glob_to_regex(path)
        return [t for t in tracked if rx.match(t) and not t.endswith("CMakeLists.txt")]
    if path in tracked:
        return [path]
    if is_chain and path.startswith(("fw/", "sim/")):
        return [t for t in tracked if t.startswith(path + "/") and Path(t).suffix in CXX_SUFFIXES | {".py"}]
    return []


def cross_check_reads(select_one: Selector, tracked: list[str], universe: dict, root: Path,
                      gz_paths: dict[str, set[str]]) -> list[str]:
    errors = []
    reported = set()
    for reader in tracked:
        if not reader.startswith("tests/regression/") or "/results/" in reader:
            continue
        if reader.endswith(".py"):
            reads = python_reads((root / reader).read_text(encoding="utf-8"))
        elif reader.endswith("CMakeLists.txt") or reader.endswith(".cmake"):
            reads = cmake_reads((root / reader).read_text(encoding="utf-8"), reader)
        else:
            continue
        need = reader_groups(reader, universe, gz_paths)
        if not need:
            continue
        for path, lineno, is_chain in reads:
            for target in read_targets(path, tracked, is_chain):
                if target == reader or classify(target, select_one.mapping)[0] == "ignore":
                    continue
                missing = lacks(select_one(target), need)
                if missing and (path, missing) not in reported:
                    reported.add((path, missing))
                    errors.append(f"{target} → missing {missing} (read as {path} at {reader}:{lineno})")
                    break
    return errors


# --- the register, the card and the profile

def cross_check_register(mapping: dict, universe: dict, tracked: list[str], select_one: Selector, root: Path,
                         gz_paths: dict[str, set[str]]) -> list[str]:
    """Recompute from the files what the register section and the card rules must select; fail when they select less.

    1. A file that builds a key's name from a prefix (`t4_confidence_{klass}`, `t4_confidence_*`) reads keys the whole-word
       scan cannot see: it must be an extra reader, a mention or already a reader of that key.
    2. Every test that reads a param_ids manifest is selected by a structural change of the register.
    3. Every test that names a card or a sensor profile (its path or its name) is selected by that file's own selection."""
    errors = []
    reg = mapping.get("budget_register") or {}
    if reg.get("file") and (root / reg["file"]).is_file():
        keys = [k for k in (yaml.safe_load((root / reg["file"]).read_text(encoding="utf-8")) or {})
                if len(k) >= int(reg.get("scan_min_name_length") or 1)]
        candidates = register_scan_candidates(reg, mapping, tracked)
        texts = {p: (root / p).read_text(encoding="utf-8", errors="ignore") for p in candidates}
        readers, _ = register_readers(keys, reg, mapping, tracked, root)
        for key in keys:
            known = set(readers[key] or []) | {p for m in reg.get("mentions") or [] if key in m["keys"]
                                               for p in candidates if first_match(p, m["paths"])}
            for cut in [m.end() for m in re.finditer("_", key)]:
                prefix = key[:cut]
                if len(prefix) < 12:
                    continue
                rx = re.compile(r"(?<![A-Za-z0-9_])" + re.escape(prefix) + r"(?:\{|\*)")
                for p, t in sorted(texts.items()):
                    if p not in known and rx.search(t):
                        errors.append(f"{p} builds the name of {key} from '{prefix}...': add it to budget_register extra_readers or mentions")
        structural = select([reg["file"]], mapping, universe, per_push=False,
                            register={"file": reg["file"], "structural": True, "keys": {}})
        param_ids_word = re.compile(r"\bparam_ids\b(?!\.hpp)")
        for p in tracked:
            if not p.startswith("tests/regression/") or "/results/" in p or Path(p).suffix not in {".py", ".txt", ".cmake"} \
                    or p.endswith("/param_ids"):
                continue
            need = reader_groups(p, universe, gz_paths)
            if need and param_ids_word.search((root / p).read_text(encoding="utf-8", errors="ignore")):
                missing = lacks(structural, need)
                if missing:
                    errors.append(f"budget_register structural → missing {missing} (reads param_ids at {p})")
    cards = [p for p in tracked if p.startswith(("vehicles/", "sensors/profiles/")) and p.endswith(".yaml")]
    for card in cards:
        names = [card, Path(card).stem]
        sel = select_one(card)
        for p in tracked:
            if not p.startswith("tests/regression/") or "/results/" in p or p == card or Path(p).suffix == ".md":
                continue
            need = reader_groups(p, universe, gz_paths)
            if need is None:
                continue
            text = (root / p).read_text(encoding="utf-8", errors="ignore")
            if any(n in text for n in names):
                missing = lacks(sel, need)
                if missing:
                    errors.append(f"{card} → missing {missing} (named by {p})")
        # a CMakeLists.txt, *.cmake, fw/, sim/ or tools/ file that names it makes it a build or tool input: it must select
        # what the committed card selects, whatever rule the file has of its own
        like = (mapping.get("card_files") or {}).get("consumed_like")
        if like and card != like and not sel["full"]:
            for p in tracked:
                if p == card or not (p.endswith("CMakeLists.txt") or p.endswith(".cmake") or p.startswith(("fw/", "sim/", "tools/"))):
                    continue
                if first_match(p, mapping.get("ignore") or []) or p in ("ci/affected.py",):
                    continue
                if any(re.search(r"(?<![A-Za-z0-9_])" + re.escape(n) + r"(?![A-Za-z0-9_])", (root / p).read_text(encoding="utf-8", errors="ignore"))
                       for n in names):
                    want = select_one(like)
                    short = [k for k in SELECTION_KEYS if not set(want[k]) <= set(sel[k])]
                    if short and not want["full"]:
                        errors.append(f"{card} → selects less than {like} for {short} (named by {p})")
                    break
    return errors


def repo_files(root: Path = ROOT) -> list[str]:
    """Tracked files plus untracked files that are not ignored."""
    names = git(["ls-files", "-z"], root) + git(["ls-files", "--others", "--exclude-standard", "-z"], root)
    return sorted({p for p in names.split("\0") if p})


def cross_checks(mapping: dict, universe: dict, root: Path = ROOT) -> tuple[list[str], list[str]]:
    tracked = repo_files(root)
    cards = [p for p in tracked if first_match(p, (mapping.get("card_files") or {}).get("globs") or []) is not None]
    select_one = Selector(mapping, universe, PythonIndex(root, tracked, universe),
                          consumed_card_files(mapping, cards, tracked, root))
    gz_paths = parse_gz_test_paths((root / "ci" / "run_ci_gz.sh").read_text())
    errors, notes = cross_check_cxx(select_one, tracked, root / "build" / "host-debug" / "compile_commands.json", root)
    errors += cross_check_python(select_one, tracked, universe, root, gz_paths)
    errors += cross_check_reads(select_one, tracked, universe, root, gz_paths)
    errors += cross_check_register(mapping, universe, tracked, select_one, root, gz_paths)
    return errors, notes


# ------------------------------------------------------------------------------------------------ output

def summary(result: dict, notes: list[str] | None = None) -> str:
    lines = [f"base {result['base'] or '(unresolved)'}, {len(result['changed'])} changed files"]
    if result["full"]:
        lines.append("FULL: everything is selected")
        lines.extend(f"  reason {r}" for r in result["full_reasons"])
    for key, label in (("ctest_labels", "ctest label"), ("run_ci_steps", "run_ci step"),
                       ("pytest_paths", "pytest path"), ("gz_groups", "gz group")):
        lines.extend(f"{label} {v}" for v in result[key])
    if not result["full"] and not any(result[k] for k in SELECTION_KEYS):
        lines.append("nothing selected")
    lines.extend(f"note: {n}" for n in notes or [])
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--base", help="base revision (default: git merge-base HEAD master)")
    ap.add_argument("--head", help="head revision (default: the working tree, with untracked files)")
    ap.add_argument("--json", action="store_true", help="machine output")
    ap.add_argument("--check-manifest", action="store_true", help="check the map against ci/run_ci.sh and the repository")
    args = ap.parse_args(argv)

    mapping = load_map()
    universe = repo_universe()
    universe["pytest_paths"] = map_pytest_paths(mapping)

    if args.check_manifest:
        errors = check_manifest(mapping, universe)
        cross_errors, cross_notes = cross_checks(mapping, universe)
        errors += cross_errors
        tracked = [p for p in git(["ls-files", "-z"]).split("\0") if p]
        for note in map_notes(mapping, tracked) + cross_notes:
            print(f"note: {note}", file=sys.stderr)
        for e in errors:
            print(f"error: {e}", file=sys.stderr)
        if errors:
            return 1
        print("manifest OK")
        return 0

    notes: list[str] = []
    per_push = os.environ.get("MARV_CI_MODE", "per-push") != "full"
    try:
        base = resolve_base(args.base)
        tracked = repo_files()
        pyindex = PythonIndex(ROOT, tracked, universe)
        changed = changed_paths(base, args.head)
        register = register_change(mapping, base, args.head, changed, tracked)
        card_globs = (mapping.get("card_files") or {}).get("globs") or []
        consumed = consumed_card_files(mapping, [p for p in changed if first_match(p, card_globs) is not None], tracked)
        result = select(changed, mapping, universe, base, pyindex=pyindex, per_push=per_push, notes=notes, register=register,
                        consumed=consumed)
    except RuntimeError as e:
        result = select([], mapping, universe, forced_full=[f"git: {e}"])
    if args.json:
        print(json.dumps(result, indent=2))
        for note in notes:
            print(f"note: {note}", file=sys.stderr)
    else:
        print(summary(result, notes))
    return 0


if __name__ == "__main__":
    sys.exit(main())
