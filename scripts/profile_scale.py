"""G5: runtime and memory scaling of planning and exact verification, 10 to 4000 drones.

Computational scaling study only. The shows use relaxed altitude and geofence limits so that every
size is a *valid* show (the volumetric shapes at n >= 2000 exceed the default 0 to 120 m band); this
says nothing about an operational envelope. Fixed seeds. Each size runs in its own process (peak RSS
via resource.getrusage) with a timeout; finished sizes are cached in results/.cache so an
interrupted run resumes.

    python scripts/profile_scale.py                  # all sizes
    python scripts/profile_scale.py --one 500        # one size, prints JSON (used internally)
"""

from __future__ import annotations

import argparse
import cProfile
import json
import pstats
import resource
import subprocess
import sys
import time
from pathlib import Path

SIZES = [10, 20, 50, 100, 200, 500, 1000, 2000, 4000]
TIMEOUT_S = 3000


def scale_show(n: int, spacing: float = 3.2) -> dict:
    from sirb.spec import new_show
    grid = {"type": "launch_grid", "spacing_m": spacing, "center": [0.0, -250.0, 0.0]}
    big = {"limits": {"geofence": {"x": [-500.0, 500.0], "y": [-450.0, 300.0]},
                      "z_max": 500.0, "crowd": {"y": -800.0, "min_distance_m": 50.0}}}
    return new_show(f"scale n={n}", n, [
        {"name": "launch", "shape": grid, "hold_s": 2},
        {"name": "orb", "shape": {"type": "sphere", "spacing_m": spacing,
                                   "center": [0, 0, 200]}, "hold_s": 4},
        {"name": "wall", "shape": {"type": "wall", "aspect": 1.6, "spacing_m": spacing,
                                    "center": [0, 0, 200]}, "hold_s": 4},
        {"name": "land", "shape": grid, "hold_s": 2},
    ], seed=7, **big)


def one(n: int) -> dict:
    from sirb.assign import METHODS
    from sirb.planner.baseline import plan_baseline
    from sirb.spec import build_targets, validate_show
    from sirb.verify import verify
    show = scale_show(n)
    errs = validate_show(show, check_targets=False)
    t = time.perf_counter()
    targets = build_targets(show)
    t_targets = time.perf_counter() - t
    t = time.perf_counter()
    METHODS["hungarian_sq"](targets[0], targets[1])
    t_hungarian_one = time.perf_counter() - t
    t = time.perf_counter()
    fleet, _ = plan_baseline(show)
    t_plan = time.perf_counter() - t
    prof = cProfile.Profile()
    t = time.perf_counter()
    prof.enable()
    rep = verify(fleet, show["limits"])
    prof.disable()
    t_verify = time.perf_counter() - t
    st = pstats.Stats(prof)
    top = sorted(((v[3], k) for k, v in st.stats.items()), reverse=True)[:8]
    return {"n": n, "validate_errors": errs[:3], "targets_s": t_targets,
            "hungarian_one_transition_s": t_hungarian_one, "plan_total_s": t_plan,
            "verify_s": t_verify, "verified_ok": rep.ok,
            "violation_counts": rep.violation_counts,
            "nominal_min_separation_m": rep.min_separation, "pairs_checked": rep.n_pairs_checked,
            "n_segments": rep.n_segments,
            "verify_top_cumulative": [{"cum_s": round(c, 3), "fn": f"{k[2]} ({Path(k[0]).name}:{k[1]})"}
                                      for c, k in top],
            "peak_rss_mb": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--one", type=int)
    ap.add_argument("--sizes", type=int, nargs="*")
    a = ap.parse_args()
    if a.one:
        print(json.dumps(one(a.one)))
        return
    cache = Path("results/.cache")
    cache.mkdir(parents=True, exist_ok=True)
    rows = []
    for n in a.sizes or SIZES:
        f = cache / f"profile_scale_{n}.json"
        if f.exists():
            rows.append(json.loads(f.read_text()))
            continue
        try:
            p = subprocess.run([sys.executable, __file__, "--one", str(n)], capture_output=True,
                               text=True, timeout=TIMEOUT_S)
            row = json.loads(p.stdout.strip().splitlines()[-1]) if p.returncode == 0 else {
                "n": n, "failed": True, "returncode": p.returncode, "stderr_tail": p.stderr[-400:]}
        except subprocess.TimeoutExpired:
            row = {"n": n, "failed": True, "timeout_s": TIMEOUT_S}
        f.write_text(json.dumps(row))
        rows.append(row)
        print(json.dumps({k: row.get(k) for k in ("n", "plan_total_s", "verify_s", "verified_ok",
                                                  "peak_rss_mb", "failed")}), flush=True)
    Path("results/scaling_profile.json").write_text(json.dumps(
        {"note": "computational scaling only; relaxed altitude/geofence limits; see script docstring",
         "rows": rows}, indent=1))


if __name__ == "__main__":
    main()
