"""Baseline benchmark: sirb planners and independent comparators on shared scenarios.

Every plan is judged by the exact verifier (sirb.verify) with the same threshold. Output:
results/baselines.json. Fixed seeds; no hand edited numbers.
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from sirb.baselines.orca import plan_orca  # noqa: E402
from sirb.baselines.sampled import sampled_min_separation  # noqa: E402
from sirb.planner.baseline import plan_baseline  # noqa: E402
from sirb.planner.waves import plan_waves  # noqa: E402
from sirb.shapes import snap  # noqa: E402
from sirb.verify import pairwise_min_separation  # noqa: E402

D_MIN = 2.0
SAMPLE_STEPS = (0.5, 0.2, 0.1, 0.05)


def template(name: str, start: np.ndarray, goal: np.ndarray) -> dict:
    n = len(start)
    return {
        "schema_version": 1, "name": name, "n_drones": n, "seed": 1,
        "vehicle": {"name": "bench", "mass_kg": 0.83, "radius_m": 0.24,
                    "sources": {"mass_kg": "assumption", "radius_m": "assumption"}},
        "limits": {"vmax_h": 4.0, "vmax_up": 4.0, "vmax_down": 4.0, "amax": 4.0, "jmax": 8.0,
                   "z_min": -1000.0, "z_max": 1000.0,
                   "geofence": {"x": [-1000.0, 1000.0], "y": [-1000.0, 1000.0]},
                   "min_separation_m": D_MIN,
                   "crowd": {"y": -5000.0, "min_distance_m": 1.0},
                   "sources": {"all": "assumption"}},
        "stages": [{"name": "a", "shape": {"type": "points", "points": start.tolist()},
                    "hold_s": 0.0},
                   {"name": "b", "shape": {"type": "points", "points": goal.tolist()},
                    "hold_s": 0.0}],
        "disclaimer": "simulation-validated only; not flight-certified",
    }


def fibonacci_sphere(n: int, radius: float, centre=(0.0, 0.0, 30.0)) -> np.ndarray:
    k = np.arange(n) + 0.5
    phi = np.arccos(1 - 2 * k / n)
    th = np.pi * (1 + 5 ** 0.5) * k
    p = np.stack([np.cos(th) * np.sin(phi), np.sin(th) * np.sin(phi), np.cos(phi)], 1)
    return snap(radius * p + np.asarray(centre))


def sphere_radius(n: int) -> float:
    return max(10.0, 2.2 * D_MIN * (n / 4.0) ** 0.5 * 1.0)


def spaced_random(n: int, box: float, seed: int, z=(10.0, 40.0)) -> np.ndarray:
    rng = np.random.default_rng(seed)
    pts: list[np.ndarray] = []
    while len(pts) < n:
        c = np.array([rng.uniform(-box, box), rng.uniform(-box, box), rng.uniform(*z)])
        if all(np.linalg.norm(c - q) >= 1.6 * D_MIN for q in pts):
            pts.append(c)
    return snap(np.array(pts))


def scenarios() -> list[dict]:
    out = []
    for n in (8, 16, 32, 64):
        s = fibonacci_sphere(n, sphere_radius(n))
        c = np.array([0.0, 0.0, 30.0])
        out.append({"name": f"antipodal_sphere_{n}", "n": n, "start": s,
                    "goal": snap(2 * c - s)})
    for n in (16, 32, 64):
        box = 12.0 * (n / 16) ** 0.5 * 2.0
        out.append({"name": f"random_exchange_{n}", "n": n,
                    "start": spaced_random(n, box, 100 + n),
                    "goal": spaced_random(n, box, 200 + n)})
    return out


def judge(fleet) -> dict:
    d, i, j, t, lb, _ = pairwise_min_separation(fleet, 1000.0)
    return {"min_separation_m": float(d), "pass": bool(d >= D_MIN - 1e-6),
            "pair": [int(i), int(j)], "time_s": float(t)}


def makespan(fleet) -> float:
    return float(max(tr.segments[-1].t1 for tr in fleet.trajectories))


def run_sirb(show: dict, planner: str, assign: str):
    t = time.perf_counter()
    try:
        if planner == "waves":
            fleet, info = plan_waves(show, assign=assign)
        else:
            fleet, info = plan_baseline(show, assign=assign)
    except Exception as exc:  # planners raise when they cannot produce a plan
        return {"status": "no_plan", "reason": str(exc)[:120]}
    r = {"status": "ok", "makespan_s": makespan(fleet), "plan_s": time.perf_counter() - t}
    r.update(judge(fleet))
    r["_fleet"] = fleet
    return r


def sampled_false_pass(trials: int = 3000, seed: int = 7) -> dict:
    """Random two drone crossings with a miss distance below the threshold. Counts how often a
    frame sampled check at each step passes a fleet that the exact verifier fails."""
    from sirb.trajectory import Fleet, Segment, Trajectory

    rng = np.random.default_rng(seed)

    def line(p0, v, T):
        return Trajectory([Segment(0.0, T, np.stack([p0, v]))])

    failing = 0
    false_pass = {str(dt): 0 for dt in SAMPLE_STEPS}
    for _ in range(trials):
        T = 10.0
        tc = rng.uniform(2.0, 8.0)
        va = rng.normal(size=3)
        va = 4.0 * va / np.linalg.norm(va)
        vb = rng.normal(size=3)
        vb = 4.0 * vb / np.linalg.norm(vb)
        m = rng.normal(size=3)
        m = rng.uniform(0.0, D_MIN) * m / np.linalg.norm(m)
        pa = np.array([0.0, 0.0, 0.0]) - va * tc
        pb = m - vb * tc
        fleet = Fleet([line(pa, va, T), line(pb, vb, T)])
        if judge(fleet)["pass"]:
            continue
        failing += 1
        for dt in SAMPLE_STEPS:
            if sampled_min_separation(fleet, dt, D_MIN).ok:
                false_pass[str(dt)] += 1
    return {"trials": trials, "failing_fleets": failing, "false_pass": false_pass,
            "false_pass_rate": {k: v / failing for k, v in false_pass.items()},
            "relative_speed_range_mps": [0.0, 8.0]}


def main() -> None:
    results = []
    for sc in scenarios():
        show = template(sc["name"], sc["start"], sc["goal"])
        row = {"scenario": sc["name"], "n": sc["n"], "methods": {}}
        straight = run_sirb(show, "baseline", "identity")
        row["methods"]["straight_labeled"] = straight
        row["methods"]["waves_labeled"] = run_sirb(show, "waves", "identity")
        row["methods"]["straight_unlabeled_hungarian"] = run_sirb(show, "baseline", "hungarian_sq")
        row["methods"]["waves_unlabeled_hungarian"] = run_sirb(show, "waves", "hungarian_sq")
        for rs in (2.0, 2.4):
            t = time.perf_counter()
            res = plan_orca(sc["start"], sc["goal"], radius_sum=rs, vmax=4.0,
                            max_time_s=300.0, dim=3)
            m = {"status": "ok" if res.arrived else "timeout", "makespan_s": res.makespan_s,
                 "plan_s": time.perf_counter() - t}
            m.update(judge(res.fleet))
            row["methods"][f"orca_r{rs}_labeled"] = m
        # sampled checking of the failing straight line fleet
        if straight.get("status") == "ok":
            fl = straight["_fleet"]
            row["sampled_on_straight_labeled"] = {
                str(dt): (lambda r: {"min_separation_m": float(r.min_separation),
                                     "ok": bool(r.ok)})(sampled_min_separation(fl, dt, D_MIN))
                for dt in SAMPLE_STEPS}
        for m in row["methods"].values():
            m.pop("_fleet", None)
        results.append(row)
        print(row["scenario"], {k: (v.get("pass"), round(v.get("makespan_s", -1), 1))
                                for k, v in row["methods"].items()}, flush=True)
    out = ROOT / "results" / "baselines.json"
    sampled = sampled_false_pass()
    print("sampled false pass", sampled["false_pass_rate"])
    out.write_text(json.dumps({"d_min_m": D_MIN, "vmax": 4.0, "results": results,
                               "sampled_false_pass": sampled}, indent=1))


if __name__ == "__main__":
    main()
