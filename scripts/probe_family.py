"""Record, bit for bit, what both verifiers return for one adversarial family (stream F, phase 6).

Used to locate the cause of a difference between two differential report runs: run it in each
environment and compare the outputs with ``--compare``. Floats are stored with ``float.hex`` so
equality is exact. Per seed: the exact minimum of every pair, verify2's lower bound, the verdicts and
the disagreement records, exactly as ``scripts/differential_report.py`` computes them.

    python scripts/probe_family.py degenerate_collision --out probe.json
    python scripts/probe_family.py --compare a.json b.json
"""

from __future__ import annotations

import argparse
import hashlib
import json
import platform
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sirb.diffcases import adversarial_case, compare  # noqa: E402


def hexify(o):
    if isinstance(o, float):
        return float(o).hex()
    if isinstance(o, dict):
        return {str(k): hexify(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [hexify(v) for v in o]
    if isinstance(o, np.generic):
        return hexify(o.item())
    return o


def kernel_hashes() -> dict:
    """SHA-256 of eigenvalues and of matrix products of 2000 fixed random matrices (n = 2..12):
    the companion matrix root finder of sirb.verify uses np.linalg.eigvals (LAPACK)."""
    rng = np.random.default_rng(12345)
    he, hm = hashlib.sha256(), hashlib.sha256()
    for _ in range(2000):
        n = int(rng.integers(2, 13))
        A = rng.standard_normal((n, n))
        he.update(np.sort_complex(np.linalg.eigvals(A)).tobytes())
        hm.update((A @ A.T).tobytes())
    return {"eigvals_sha256": he.hexdigest(), "matmul_sha256": hm.hexdigest()}


def probe(family: str, seeds: int) -> dict:
    rows = []
    for s in range(seeds):
        fleet, d_min, meta = adversarial_case(family, s)
        stats, dis = compare(fleet, d_min, meta.get("d_true"), np.random.default_rng(s))
        rows.append({"seed": s, "stats": hexify(stats), "disagreements": hexify(dis)})
    return {"family": family, "seeds": seeds,
            "environment": {"python": platform.python_version(), "platform": platform.platform(),
                            "numpy": np.__version__, "compiler": platform.python_compiler()},
            "kernels": kernel_hashes(), "rows": rows}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("family", nargs="?")
    ap.add_argument("--seeds", type=int, default=250)
    ap.add_argument("--out")
    ap.add_argument("--compare", nargs=2)
    a = ap.parse_args()
    if a.compare:
        x, y = (json.loads(Path(p).read_text()) for p in a.compare)
        res = {"family": x["family"], "a": x["environment"], "b": y["environment"],
               "seeds_compared": len(x["rows"]),
               "seeds_differing": [rx["seed"] for rx, ry in zip(x["rows"], y["rows"]) if rx != ry],
               "seeds_with_different_disagreement_records": [
                   rx["seed"] for rx, ry in zip(x["rows"], y["rows"])
                   if rx["disagreements"] != ry["disagreements"]],
               "eigvals_identical": x["kernels"]["eigvals_sha256"] == y["kernels"]["eigvals_sha256"],
               "matmul_identical": x["kernels"]["matmul_sha256"] == y["kernels"]["matmul_sha256"]}
        print(json.dumps(res, indent=1))
        if a.out:
            Path(a.out).write_text(json.dumps(res, indent=1) + "\n")
        return 0
    Path(a.out).write_text(json.dumps(probe(a.family, a.seeds)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
