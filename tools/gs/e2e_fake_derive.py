#!/usr/bin/env python3
"""The stand-in derivation of the GX-13 end-to-end run (tools/gs/e2e.sh light mode, gcs/e2e/derive.spec.ts). A test tool, never flown.

The helper's derive command (tools/gs/derive.py) runs three repository scripts with its Python; `--derive-python` gives it the wrapper
`e2e_fake_derive.py --install DIR` writes (DIR/python), which runs this script with the script path and the arguments the helper meant for
Python. By the script's name:
  lint.py, mcfg.py   the real script (the lint of the card copy and the .mcfg encoder are fast)
  flatten_cached.py  writes the flattened files and the four derivation reports at once (the real flatten takes a minute on 16 CPUs), after
                     MARV_FAKE_DERIVE_STEP_S seconds; the card file records whether the card copy differs from the repository's card
  params_gen.py      writes $MARV_FAKE_DERIVE_MANIFEST as params_manifest.json; when the card copy was edited the first unlocked f32 default has
                     one more in its bits (so the derived config differs from the image's factory config)
  validate           the validator stand-in: a PASS report for the config and the image file, in the validator's command-line contract
"""

from __future__ import annotations

import hashlib
import json
import os
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def install(directory: Path) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / "python"
    path.write_text(f'#!/bin/sh\nexec "{sys.executable}" "{Path(__file__).resolve()}" "$@"\n')
    path.chmod(0o755)


def arg(args: list[str], flag: str) -> str:
    return args[args.index(flag) + 1]


def flatten(args: list[str]) -> int:
    time.sleep(float(os.environ.get("MARV_FAKE_DERIVE_STEP_S", "0")))
    card = Path(arg(args, "--card"))
    root = Path(arg(args, "--root"))
    repo_card = ROOT / card.relative_to(root)
    edited = card.read_bytes() != repo_card.read_bytes()
    for flag in ("--out-card", "--out-register", "--out-mixer", "--out-scenario", "--out-rate-lead", "--out-attitude-lead", "--out-gyro-chain", "--out-vertical"):
        out = Path(arg(args, flag))
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(f"# stand-in flatten output\n# edited: {'yes' if edited else 'no'}\n")
        if flag in ("--out-rate-lead", "--out-attitude-lead", "--out-gyro-chain", "--out-vertical"):
            out.with_name(out.stem + "_report.txt").write_text(
                f"stand-in derivation report {out.stem}\ncard {card.relative_to(root).as_posix()} sha256 {hashlib.sha256(card.read_bytes()).hexdigest()}\n"
                f"card copy differs from the repository card: {edited}\n")
    return 0


def params_gen(args: list[str]) -> int:
    first_card = Path(arg(args, "--card"))
    edited = "edited: yes" in first_card.read_text()
    manifest = json.loads(Path(os.environ["MARV_FAKE_DERIVE_MANIFEST"]).read_text())
    if edited:
        for entry in manifest["params"].values():
            if entry["type"] == "f32" and not entry["lock"]:
                entry["default_bits"] = f"0x{int(entry['default_bits'], 16) + 1:08x}"
                break
    out = Path(arg(args, "--out-dir"))
    out.mkdir(parents=True, exist_ok=True)
    (out / "params_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return 0


def validate(args: list[str]) -> int:
    sys.path.insert(0, str(ROOT / "tools" / "config"))
    import mcfg
    image = Path(arg(args, "--image")).read_bytes()
    cfg = mcfg.decode(Path(arg(args, "--config")).read_bytes())
    body = (json.dumps({"format": "marv-validation-report/1", "verdict": "PASS", "image": {"sha256": hashlib.sha256(image).hexdigest()},
                        "config": {"config_hash": mcfg.config_hash(cfg).hex()}, "checks": []}, sort_keys=True) + "\n").encode()
    Path(arg(args, "--out")).write_bytes(body)
    print("stand-in validator: PASS", file=sys.stderr)
    print("report_id: " + hashlib.sha256(body).hexdigest())
    print("verdict: PASS")
    return 0


def main(argv: list[str]) -> int:
    if argv[:1] == ["--install"]:
        install(Path(argv[1]))
        return 0
    script, args = Path(argv[0]).name, argv[1:]
    if script == "flatten_cached.py":
        return flatten(args)
    if script == "params_gen.py":
        return params_gen(args)
    if script == "validate":
        return validate(args)
    if script in ("lint.py", "mcfg.py"):
        os.execv(sys.executable, [sys.executable, argv[0], *args])
    print(f"stand-in derive: unsupported script {argv[0]}", file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
