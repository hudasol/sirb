"""WP1 study (predictions P34 and P35 in docs/PREDICTIONS.md, committed before this ran).

Does the exact margin of a pair explain how often it violates in the stress simulation?
Cases: the four demo shows and 20 random fleets (seeds 3400 to 3419). SYNTHETIC fleets and an
ASSUMED disturbance profile; nothing here is flight evidence.

    SIRB_WORKERS=2 python scripts/doctor_study.py     # writes results/doctor_study.json
"""

from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))

from sirb.disturbance import StressConfig  # noqa: E402
from sirb.doctor import diagnose  # noqa: E402
from sirb.envinfo import environment  # noqa: E402
from sirb.planner.baseline import plan_baseline  # noqa: E402
from sirb.spec import load_show, new_show, validate_show  # noqa: E402

TRIALS = 200
SEED = 20261016
MIN_VIOLATING = 20
WORKERS = int(os.environ.get("SIRB_WORKERS", "1"))
PROFILE = dict(dt=0.2, w20=6.0, gnss_bias_sigma=0.5, gnss_deg_fraction=0.25)
DEMOS = {"gallery_120": "shows/gallery_120.json", "oryx_500": "shows/oryx_500.json",
         "torus_300": "shows/torus_300.json", "sirb_text_100": "shows/sirb_text_100.json"}
RANDOM_SEEDS = list(range(3400, 3420))
SHAPES = ("sphere", "wall", "ring", "torus")


def random_show(seed: int) -> dict:
    rng = np.random.default_rng(seed)
    n = int(rng.integers(16, 41))
    sp = float(rng.uniform(3.0, 4.2))
    grid = {"type": "launch_grid", "spacing_m": sp, "center": [0.0, -85.0, 0.0]}
    stages = [{"name": "launch", "shape": grid, "hold_s": 2}]
    for k, name in enumerate(rng.choice(SHAPES, size=3)):
        shape = {"type": str(name), "spacing_m": sp, "center": [0, 0, 60]}
        if name == "wall":
            shape["aspect"] = 1.6
        stages.append({"name": f"{name}{k}", "shape": shape, "hold_s": 3})
    stages.append({"name": "land", "shape": grid, "hold_s": 2})
    show = new_show(f"random fleet {seed}", n, stages, seed=seed,
                    limits={"geofence": {"x": [-200.0, 200.0], "y": [-200.0, 150.0]}})
    return show


def run_case(name: str, show: dict) -> dict:
    errs = validate_show(show)
    if errs:
        return {"name": name, "status": "invalid show", "errors": errs}
    t = time.time()
    fleet, _ = plan_baseline(show)
    d = diagnose(show, fleet, StressConfig(**PROFILE), trials=TRIALS, seed=SEED, workers=WORKERS)
    st = d["stress"]
    n_viol = round(st["p_violation"] * TRIALS)
    rank = st["pair_ranking"]
    out = {"name": name, "n_drones": show["n_drones"], "exact_verdict": d["exact"]["verdict"],
           "exact_min_separation_m": d["exact"]["min_separation_m"],
           "violating_trials": n_viol, "p_violation": st["p_violation"],
           "violation_events": st["violation_events"], "wall_s": time.time() - t,
           "included": n_viol >= MIN_VIOLATING}
    if st["violation_events"]:
        cov = st["coverage_by_exact_rank"]
        out["top1_share"], out["top5_share"], out["top10_share"] = (
            cov["top_1_exact_share_of_events"], cov["top_5_exact_share_of_events"],
            cov["top_10_exact_share_of_events"])
        top = rank[0]
        out["most_frequent_pair"] = top["pair"]
        out["most_frequent_pair_exact_rank"] = top["exact_rank"]
        out["exact_rank1_is_most_frequent"] = top["exact_rank"] == 1
        out["pairs_with_violations"] = st["pairs_with_violations"]
    return out


def main() -> None:
    run1 = Path("results/doctor_study_run1.json")
    if run1.exists():   # the demo cases are deterministic and unchanged between the two runs
        old = {c["name"]: c for c in json.loads(run1.read_text())["cases"]}
        cases = [old[k] for k in DEMOS]
    else:
        cases = [run_case(k, load_show(v)) for k, v in DEMOS.items()]
    cases += [run_case(f"random_{s}", random_show(s)) for s in RANDOM_SEEDS]
    inc = [c for c in cases if c.get("included")]
    p34 = [c["top10_share"] >= 0.8 for c in inc]
    p35 = [c["exact_rank1_is_most_frequent"] for c in inc]
    res = {
        "label": "SIMULATED; synthetic fleets; assumed disturbance profile; not flight evidence",
        "profile": PROFILE, "trials": TRIALS, "seed": SEED, "min_violating_trials": MIN_VIOLATING,
        "cases": cases,
        "n_cases": len(cases), "n_included": len(inc),
        "excluded": [{"name": c["name"], "violating_trials": c.get("violating_trials")}
                     for c in cases if not c.get("included")],
        "P34": {"fraction_with_top10_share_ge_0.8": (sum(p34) / len(p34)) if p34 else None,
                "threshold": 0.75,
                "median_top10_share": float(np.median([c["top10_share"] for c in inc])) if inc else None,
                "median_top5_share": float(np.median([c["top5_share"] for c in inc])) if inc else None,
                "median_top1_share": float(np.median([c["top1_share"] for c in inc])) if inc else None},
        "P35": {"fraction_exact_rank1_most_frequent": (sum(p35) / len(p35)) if p35 else None,
                "threshold": 0.5},
        "environment": environment({"stress_seed": SEED, "random_fleet_seeds": RANDOM_SEEDS}),
    }
    Path("results/doctor_study.json").write_text(json.dumps(res, indent=1))
    print(json.dumps({k: res[k] for k in ("n_cases", "n_included", "P34", "P35")}, indent=1))


if __name__ == "__main__":
    main()
