#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Fetch the standard MAVLink message definitions at the commit pinned in link/mavlink_pin.json into a directory.

The XML files are never vendored: the CMake configure step and the tools tests call this script, which downloads each file
from the pinned commit and checks it against the SHA-256 in the pin. A file already in the directory with the right hash is
not fetched again. The marv dialect (link/marv.xml) is copied beside them so that its `<include>` resolves.

Exit 0 when the directory holds every file with the pinned hash, 1 on a hash mismatch or a failed download.
"""

import argparse
import hashlib
import json
import shutil
import sys
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PIN = ROOT / "link" / "mavlink_pin.json"
DIALECT = ROOT / "link" / "marv.xml"
TIMEOUT_S = 60


def load_pin() -> dict:
    return json.loads(PIN.read_text())


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def fetch(out_dir: Path) -> None:
    pin = load_pin()
    out_dir.mkdir(parents=True, exist_ok=True)
    for name, want in pin["files"].items():
        target = out_dir / name
        if target.exists() and sha256(target) == want:
            continue
        url = pin["url_template"].format(commit=pin["commit"], file=name)
        with urllib.request.urlopen(url, timeout=TIMEOUT_S) as response:  # noqa: S310 (pinned https URL)
            data = response.read()
        got = hashlib.sha256(data).hexdigest()
        if got != want:
            raise SystemExit(f"{name}: SHA-256 {got} from {url} is not the pinned {want}")
        target.write_bytes(data)
    shutil.copyfile(DIALECT, out_dir / DIALECT.name)


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    fetch(args.out_dir)
    print(f"mavlink definitions at {load_pin()['commit']} in {args.out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
