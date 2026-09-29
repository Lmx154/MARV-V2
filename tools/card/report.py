#!/usr/bin/env python3
"""The L1 run report (core contracts 2.1, 2.2, 6): what a run's card and budget say about themselves.

  report.py --card <card> --budget <budget> [--root <repo>] [--out <file>]

Plain text, stable order, written to --out or stdout. Also usable as a function: build_report(card, budget, root).
It reads the files as given and does not lint them (flatten.py lints); a file that cannot be parsed is an error.

  card hash     SHA-256, full hex, over the card closure: the card file, then each profile it references in
                reference order, each fed as <repo-relative path> 0x1f <byte length in decimal> 0x1f <raw bytes>
  budget hash   the same framing over the budget file alone
  generator commit   `git rev-parse HEAD` in the root, "UNKNOWN" when git cannot say; "dirty" is appended when a
                tracked file differs from HEAD

Lists, each entry as `<file>:<path>  source: <source>` (file is card, profile or budget): UNVERIFIED, INFERRED, sigma
UNKNOWN (any component), value UNKNOWN, design-budget entries (value or UNKNOWN), locked entries (by, on, via). The
last line states the model: perfect-model (truth and firmware from the same card, not dispersed; core 6).
"""

from __future__ import annotations

import argparse
import hashlib
import subprocess
import sys
from pathlib import Path

import yaml

import lint
import schema

MODEL_LINE = "model: perfect-model (truth and firmware from the same card, not dispersed; core §6)"
SEP = b"\x1f"


def _rel(path, root):
    p = Path(path).resolve()
    try:
        return p.relative_to(Path(root).resolve()).as_posix()
    except ValueError:
        return p.as_posix()


def framed_hash(files, root):
    """SHA-256 hex over files, each as <repo-relative path> 0x1f <byte length> 0x1f <raw bytes>."""
    h = hashlib.sha256()
    for f in files:
        data = Path(f).read_bytes()
        h.update(_rel(f, root).encode("utf-8") + SEP + str(len(data)).encode("ascii") + SEP + data)
    return h.hexdigest()


def profile_files(card_doc, root):
    """The profiles the card references, in reference order."""
    pid = card_doc.get("sensor_profile")
    if isinstance(pid, str) and pid:
        return [Path(root) / lint.PROFILE_DIR / f"{pid}.yaml"]
    return []


def generator_commit(root):
    def git(*args):
        return subprocess.run(["git", "-C", str(root), *args], capture_output=True, text=True, check=False)

    try:
        head = git("rev-parse", "HEAD")
        if head.returncode != 0 or not head.stdout.strip():
            return "UNKNOWN"
        dirty = git("diff-index", "--quiet", "HEAD", "--")
    except OSError:
        return "UNKNOWN"
    return head.stdout.strip() + (" dirty" if dirty.returncode != 0 else "")


def _is_entry(node):
    return isinstance(node, dict) and ("value" in node or "model" in node) and "method" in node


def walk_entries(node, path=""):
    """Yield (dotted path, entry) for every entry mapping below node, in file order."""
    if _is_entry(node):
        yield path, node
    elif isinstance(node, dict):
        for k, v in node.items():
            yield from walk_entries(v, f"{path}.{k}" if path else str(k))


def _source(entry):
    text = entry.get("source")
    if not isinstance(text, str):
        text = "(design-budget entry; the rationale is in the register)"
    return " ".join(text.split())


def _line(label, path, entry):
    return f"  {label}:{path}  source: {_source(entry)}"


def _sigma_unknown(entry):
    s = entry.get("sigma")
    return s == schema.UNKNOWN or (isinstance(s, list) and schema.UNKNOWN in s)


def _lock_text(lock):
    return f"lock by {lock.get('by')} on {lock.get('on')} via {lock.get('via')}"


def build_report(card, budget, root=lint.ROOT):
    card_doc = schema.load_yaml(card)
    budget_doc = schema.load_yaml(budget)
    profiles = profile_files(card_doc, root)
    files = [("card", card_doc), *(("profile", schema.load_yaml(p)) for p in profiles), ("budget", budget_doc)]
    entries = [(label, path, e) for label, doc in files for path, e in walk_entries(doc)]

    def section(title, rows):
        return [f"{title} ({len(rows)})", *(rows or ["  (none)"]), ""]

    def where(pred, fmt=None):
        return [fmt(label, path, e) if fmt else _line(label, path, e)
                for label, path, e in entries if pred(e)]

    def status(tag):
        return lambda e: isinstance(e.get("status"), list) and tag in e["status"]

    out = [
        "MARV L1 run report",
        "",
        f"card: {_rel(card, root)}",
        *[f"profile: {_rel(p, root)}" for p in profiles],
        f"budget: {_rel(budget, root)}",
        f"card hash: {framed_hash([card, *profiles], root)}",
        f"budget hash: {framed_hash([budget], root)}",
        f"generator commit: {generator_commit(root)}",
        "",
        *section("UNVERIFIED", where(status("UNVERIFIED"))),
        *section("INFERRED", where(status("INFERRED"))),
        *section("sigma UNKNOWN", where(_sigma_unknown)),
        *section("value UNKNOWN", where(lambda e: e.get("value") == schema.UNKNOWN)),
        *section("design-budget", [
            f"  {label}:{path}  value: {e['value']}  unit: {e.get('unit')}"
            for label, path, e in entries if label == "budget"]),
        *section("locked", where(
            lambda e: isinstance(e.get("lock"), dict),
            lambda label, path, e: f"  {label}:{path}  {_lock_text(e['lock'])}  source: {_source(e)}")),
        MODEL_LINE,
    ]
    return "\n".join(out) + "\n"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--card", required=True)
    ap.add_argument("--budget", required=True)
    ap.add_argument("--root", default=str(lint.ROOT), help="repository root (resolves sensor_profile)")
    ap.add_argument("--out")
    args = ap.parse_args(argv)
    try:
        text = build_report(args.card, args.budget, args.root)
    except (OSError, yaml.YAMLError) as e:
        print(f"report.py: {e}", file=sys.stderr)
        return 1
    if args.out:
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(text, encoding="utf-8", newline="\n")
    else:
        sys.stdout.write(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
