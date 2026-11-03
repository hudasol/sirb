"""WP6 study (predictions P48 to P50 in docs/PREDICTIONS.md, committed before this ran).

Planner capacity on a crossing (random) assignment: prioritized planner against the opt in delay
planner, judged by the exact verifier. SYNTHETIC two stage shows; planning only.

    python scripts/capacity_study.py       # writes results/capacity_study.json, resumable
"""

from __future__ import annotations

import json
import signal
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from sirb.envinfo import environment  # noqa: E402
from sirb.planner.baseline import plan_baseline  # noqa: E402
from sirb.planner.delays import plan_delay_cbs  # noqa: E402
from sirb.planner.prioritized import plan_prioritized  # noqa: E402
from sirb.spec import new_show  # noqa: E402
from sirb.verify import verify  # noqa: E402

COUNTS = (50, 100, 200, 300, 400)
SEEDS = range(5)
WALL_S = 300
CACHE = Path("results/.cache")


def make_show(n: int, seed: int) -> dict:
    g = {"type": "launch_grid", "spacing_m": 3.2, "center": [0, -85, 0]}
    return new_show(f"capacity {n} seed {seed}", n, [
        {"name": "launch", "shape": g, "hold_s": 2},
        {"name": "orb", "shape": {"type": "sphere", "spacing_m": 3.2, "center": [0, 0, 45]}, "hold_s": 3}],
        seed=seed, limits={"geofence": {"x": [-200, 200], "y": [-200, 150]}})


class Timeout(Exception):
    pass


def _alarm(signum, frame):
    raise Timeout()


def run(planner: str, show: dict) -> dict:
    signal.signal(signal.SIGALRM, _alarm)
    signal.alarm(WALL_S)
    t = time.time()
    try:
        fleet, info = (plan_prioritized(show, assign="random") if planner == "prioritized"
                       else plan_delay_cbs(show, assign="random"))
        signal.alarm(0)
        rep = verify(fleet, show["limits"])
        return {"ok": bool(rep.ok), "min_separation_m": rep.min_separation, "completion_s": float(fleet.t_end),
                "plan_s": time.time() - t, "violation_counts": rep.violation_counts}
    except Timeout:
        return {"ok": False, "timeout": True, "plan_s": time.time() - t}
    except Exception as exc:   # a planner failure is a result
        signal.alarm(0)
        return {"ok": False, "error": f"{type(exc).__name__}: {exc}", "plan_s": time.time() - t}


def main() -> None:
    CACHE.mkdir(parents=True, exist_ok=True)
    rows = []
    for n in COUNTS:
        for seed in SEEDS:
            show = make_show(n, seed)
            f = CACHE / f"capacity_{n}_{seed}.json"
            if not f.exists():
                ref, _ = plan_baseline(show)       # Hungarian reference completion time
                rec = {"n": n, "seed": seed, "reference_completion_s": float(ref.t_end)}
                for p in ("prioritized", "delays"):
                    rec[p] = run(p, show)
                f.write_text(json.dumps(rec, default=float))
            rows.append(json.loads(f.read_text()))
            print(n, seed, {p: rows[-1][p]["ok"] for p in ("prioritized", "delays")}, flush=True)

    def frac(p, n):
        r = [x[p]["ok"] for x in rows if x["n"] == n]
        return sum(r) / len(r)

    med = [x["delays"]["completion_s"] / x["reference_completion_s"] for x in rows
           if x["n"] == 200 and x["delays"]["ok"]]
    res = {"label": "SYNTHETIC two stage shows; planning only; not flight evidence",
           "assignment": "random", "counts": list(COUNTS), "seeds": list(SEEDS), "wall_limit_s": WALL_S,
           "rows": rows,
           "success_fraction": {p: {str(n): frac(p, n) for n in COUNTS} for p in ("prioritized", "delays")},
           "P50": {"median_completion_ratio_n200": float(np.median(med)) if med else None,
                   "n_successful_n200": len(med)},
           "environment": environment({"show_seeds": list(SEEDS)})}
    Path("results/capacity_study.json").write_text(json.dumps(res, indent=1, default=float))
    print(json.dumps(res["success_fraction"], indent=1), res["P50"])


if __name__ == "__main__":
    main()
