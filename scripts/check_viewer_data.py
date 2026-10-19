"""CI check on viewer/data/index.json.

1. Determinism on one machine (hard check): build twice, outputs must be identical.
2. Agreement with the committed copy (hard check since phase 6). Targets are snapped to a binary
   grid and assignment costs are exact integers (docs/REPRODUCIBILITY.md), so a copy committed from
   another machine must agree: fleet hashes exactly, numbers within TOL. Before that change hashes
   differed between machines and this check was a warning only.
3. Every regenerated show verifies PASS (hard check).

Usage: python scripts/check_viewer_data.py     (after python scripts/build_viewer_data.py)
"""

from __future__ import annotations

import json
import shutil
import subprocess
import sys
from pathlib import Path

TOL = 1e-5
IDX = Path("viewer/data/index.json")


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
    first = json.loads(IDX.read_text())
    shutil.copy(IDX, "/tmp/index_first.json")
    subprocess.check_call([sys.executable, "scripts/build_viewer_data.py"],
                          stdout=subprocess.DEVNULL)
    second = json.loads(IDX.read_text())
    rc = 0
    d = diff(first, second)
    print("determinism (two builds on this machine), differences:", len(d))
    for line in d:
        print(line)
        rc = 1
    bad = [s["id"] for s in second["shows"] if s["verdict"] != "PASS"]
    if bad:
        print("shows that do not verify PASS:", bad)
        rc = 1
    old = json.loads(subprocess.check_output(["git", "show", "HEAD:viewer/data/index.json"]))
    d2 = diff(second, old)
    print("agreement with committed copy, differences:", len(d2))
    for line in d2[:8]:
        print("::error::" + line[:200])
    if d2:
        rc = 1
    return rc


if __name__ == "__main__":
    sys.exit(main())
