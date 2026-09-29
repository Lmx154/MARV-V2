#!/usr/bin/env python3
"""Fail a change to frozen regression tests that lacks a decision record (core 7.3)."""

import argparse
import re
import subprocess
import sys

REGRESSION_PREFIX = "tests/regression/"
RECORD_RE = re.compile(r"^docs/decisions/(?!0000-)\d{4}-[a-z0-9]+(?:-[a-z0-9]+)*\.md$")
TEMPLATE = "docs/decisions/0000-template.md"
HEADINGS = ("## What changed", "## Why", "## Evidence", "## Approval")


def git(*args):
    proc = subprocess.run(["git", *args], capture_output=True, text=True, check=False)
    if proc.returncode != 0:
        raise RuntimeError(f"git {' '.join(args)} failed ({proc.returncode}): {proc.stderr.strip()}")
    return proc.stdout


def parse_name_status(raw):
    """Return (status letter, [paths]) entries from `git diff --name-status -z`."""
    tokens = raw.split("\0")
    if tokens and tokens[-1] == "":
        tokens.pop()
    entries = []
    i = 0
    while i < len(tokens):
        status = tokens[i][0]
        count = 2 if status in "RC" else 1
        entries.append((status, tokens[i + 1 : i + 1 + count]))
        i += 1 + count
    return entries


def references(text, path):
    pattern = r"(?<![\w./-])" + re.escape(path) + r"(?![\w/-]|\.\w)"
    return re.search(pattern, text) is not None


def check(base, head):
    merge_base = git("merge-base", base, head).strip()
    head_commit = git("rev-parse", "--verify", head + "^{commit}").strip()
    raw = git("diff", "--name-status", "-z", "-M", merge_base, head_commit)

    frozen_paths = []
    record_paths = []
    for status, paths in parse_name_status(raw):
        path = paths[-1]
        if status in "AMT" and RECORD_RE.match(path) and path != TEMPLATE:
            record_paths.append(path)
        if status in "MTD" and path.startswith(REGRESSION_PREFIX):
            frozen_paths.append(path)
        elif status == "R" and paths[0].startswith(REGRESSION_PREFIX):
            frozen_paths.append(paths[0])
            if path.startswith(REGRESSION_PREFIX):
                frozen_paths.append(path)

    violations = []
    record_texts = []
    for path in record_paths:
        text = git("show", f"{head_commit}:{path}")
        record_texts.append(text)
        for heading in HEADINGS:
            if not re.search(r"^" + re.escape(heading) + r"[ \t]*$", text, re.MULTILINE):
                violations.append(f"{path}: decision record is missing heading '{heading}'")

    for path in frozen_paths:
        if not any(references(text, path) for text in record_texts):
            violations.append(
                f"{path}: frozen file modified, deleted or renamed, but no decision "
                f"record in this change references it"
            )

    return frozen_paths, record_paths, violations


def main(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", required=True, help="base ref; the merge base with head is diffed")
    parser.add_argument("--head", required=True, help="head ref")
    args = parser.parse_args(argv)

    try:
        frozen_paths, record_paths, violations = check(args.base, args.head)
    except RuntimeError as err:
        print(f"ERROR: {err}", file=sys.stderr)
        return 2

    if violations:
        print("FAIL: regression change check")
        for violation in violations:
            print(f"  - {violation}")
        return 1
    print(
        f"PASS: regression change check ({len(frozen_paths)} frozen file(s) changed, "
        f"{len(record_paths)} decision record(s))"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
