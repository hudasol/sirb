"""Compare a rerun of the differential report with the committed results/differential.json.

    python scripts/compare_differential.py RERUN.json      # writes results/differential_linux_vs_committed.json

Timing fields are ignored (the rule of scripts/make_changes_doc.py). The output lists every changed
value with its path, so a difference can be traced to an adversarial family and seed.
"""

from __future__ import annotations

import json
import platform
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from make_changes_doc import TIMING, leaves  # noqa: E402

OUT = Path("results/differential_linux_vs_committed.json")


def main() -> int:
    new = json.loads(Path(sys.argv[1]).read_text())
    old = json.loads(subprocess.run(["git", "show", "HEAD:results/differential.json"],
                                    capture_output=True, text=True, check=True).stdout)
    keep = lambda d: {k: v for k, v in leaves(d)  # noqa: E731
                      if not TIMING.search(k.split(".")[-1].split("[")[0])
                      and ".environment" not in k and ".python" not in k}
    a, b = keep(old), keep(new)
    changed = sorted(k for k in a.keys() & b.keys() if a[k] != b[k])
    res = {"compared": len(a.keys() & b.keys()), "changed": len(changed),
           "only_committed": sorted(a.keys() - b.keys()), "only_rerun": sorted(b.keys() - a.keys()),
           "changes": [{"path": k, "committed": a[k], "rerun": b[k]} for k in changed],
           "rerun_platform": platform.platform(), "rerun_python": platform.python_version()}
    OUT.write_text(json.dumps(res, indent=1) + "\n")
    print(res["compared"], "compared;", res["changed"], "changed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
