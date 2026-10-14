"""The l11_vertical_scripted composition's parameter set extends the product set and names what it adds: every product
parameter has the same record in the extended manifest (marv_rate, marv_mixer, marv_attitude and marv_vertical are
compiled against the product ids and run on the extended table), and every id the extended manifest adds starts with
one of the given prefixes (decision 0020 section 4: l11_ for the script and disturbances, vert_ for the hover thrust).

Usage: check_param_ids.py <product params_manifest.json> <extended params_manifest.json> <prefix> [<prefix> ...]
Exit 0 iff the product set is carried unchanged, the extended set adds at least one id and every added id starts with
a prefix.
"""

import json
import sys


def main(argv: list[str]) -> int:
    if len(argv) < 4:
        print(__doc__, file=sys.stderr)
        return 2
    with open(argv[1], encoding="utf-8") as f:
        product = json.load(f)["params"]
    with open(argv[2], encoding="utf-8") as f:
        extended = json.load(f)["params"]
    prefixes = tuple(argv[3:])
    failed = False
    for name, rec in product.items():
        if extended.get(name) != rec:
            print(f"product parameter {name}: {rec} != extended {extended.get(name)}", file=sys.stderr)
            failed = True
    added = [name for name in extended if name not in product]
    if not added:
        print("the extended set has no additional parameters", file=sys.stderr)
        failed = True
    for name in added:
        if not name.startswith(prefixes):
            print(f"added parameter {name} does not start with any of {prefixes}", file=sys.stderr)
            failed = True
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
