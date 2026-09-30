"""The composition's parameter set extends the product set: every product parameter has the same id, type and unit in
the extended manifest (marv_rate and marv_mixer are compiled against the product ids and run on the extended table).

Usage: check_param_prefix.py <product params_manifest.json> <extended params_manifest.json>
Exit 0 iff every product entry is in the extended manifest with an equal record and the extended set has more entries.
"""

import json
import sys


def main(argv: list[str]) -> int:
    if len(argv) != 3:
        print(__doc__, file=sys.stderr)
        return 2
    with open(argv[1], encoding="utf-8") as f:
        product = json.load(f)["params"]
    with open(argv[2], encoding="utf-8") as f:
        extended = json.load(f)["params"]
    bad = [name for name, rec in product.items() if extended.get(name) != rec]
    for name in bad:
        print(f"product parameter {name}: {product[name]} != extended {extended.get(name)}", file=sys.stderr)
    if len(extended) <= len(product):
        print("the extended set has no additional parameters", file=sys.stderr)
        return 1
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
