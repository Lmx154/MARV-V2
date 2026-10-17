# Checks a composition's extended parameter manifest: product ids unchanged, ids added, added ids start with a prefix.
# Consolidated from the L04/L05/L11 composition scripts, decision 0024.
"""A composition's parameter set extends the product set: every product parameter has the same record (id, type and
unit) in the extended manifest (the product libraries are compiled against the product ids and run on the extended
table), and the extended manifest adds at least one id. With prefixes, every id the extended manifest adds must also
start with one of them (decision 0020 section 4: l11_ for the script and disturbances, vert_ for the hover thrust).

Usage: check_param_ids.py <product params_manifest.json> <extended params_manifest.json> [<prefix> ...]
Exit 0 iff the product set is carried unchanged, the extended set adds at least one id and, when prefixes are given,
every added id starts with a prefix. Exit 2 on a usage error.
"""

import json
import sys

# The fields that identify a record across sets; the manifests' provenance fields (source file, lock, sigma ...) are
# per set and may differ.
IDENTITY = ("id", "type", "unit")


def main(argv: list[str]) -> int:
    if len(argv) < 3:
        print(__doc__, file=sys.stderr)
        return 2
    with open(argv[1], encoding="utf-8") as f:
        product = json.load(f)["params"]
    with open(argv[2], encoding="utf-8") as f:
        extended = json.load(f)["params"]
    prefixes = tuple(argv[3:])
    failed = False
    for name, rec in product.items():
        got = extended.get(name)
        if got is None or {k: got.get(k) for k in IDENTITY} != {k: rec.get(k) for k in IDENTITY}:
            print(f"product parameter {name}: {rec} != extended {got}", file=sys.stderr)
            failed = True
    added = [name for name in extended if name not in product]
    if not added:
        print("the extended set has no additional parameters", file=sys.stderr)
        failed = True
    if prefixes:
        for name in added:
            if not name.startswith(prefixes):
                print(f"added parameter {name} does not start with any of {prefixes}", file=sys.stderr)
                failed = True
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
