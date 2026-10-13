"""CI check: viewer/data/index.json regenerated from code must equal the committed copy. Strings,
integers and booleans (including the fleet hashes) must match exactly; floats may differ by at most
TOL, because the last printed digit of a minimum separation can differ between numpy/BLAS builds.
Prints every difference. Exit 1 on any difference."""

from __future__ import annotations

import json
import subprocess
import sys

TOL = 1e-5


def diff(a, b, path="$"):
    out = []
    if isinstance(a, dict) and isinstance(b, dict):
        for k in sorted(set(a) | set(b)):
            if k not in a or k not in b:
                out.append(f"{path}.{k}: key missing on one side")
            else:
                out += diff(a[k], b[k], f"{path}.{k}")
    elif isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            out.append(f"{path}: length {len(a)} vs {len(b)}")
        else:
            for i, (x, y) in enumerate(zip(a, b)):
                out += diff(x, y, f"{path}[{i}]")
    elif isinstance(a, float) or isinstance(b, float):
        if not (isinstance(a, (int, float)) and isinstance(b, (int, float))) or abs(a - b) > TOL:
            out.append(f"{path}: {a!r} vs {b!r}")
    elif a != b:
        out.append(f"{path}: {a!r} vs {b!r}")
    return out


def main() -> int:
    new = json.load(open("viewer/data/index.json"))
    old = json.loads(subprocess.check_output(["git", "show", "HEAD:viewer/data/index.json"]))
    d = diff(new, old)
    for line in d:
        print(line)
    print("differences:", len(d))
    return 1 if d else 0


if __name__ == "__main__":
    sys.exit(main())
