"""G8 (exploratory, proposal seed): the same planners and exact verifier applied to ground robots on
a floor grid, in simulation. Benign multi robot coordination (warehouse style). NOT a result about
any real robot: the vehicle limits below are ASSUMED, robots are points, floors are flat, there is no
localisation error and no wheel dynamics beyond the minimum jerk speed profile.

Tasks (all start on a square grid at 2.0 m pitch):
  translate : the whole grid moves 20 m (Hungarian assignment, parallel straight lines)
  shuffle   : every robot must end in a different random slot (assignment 'random': many crossings)
  swap15    : 15% of robots are forced into crossing slots (assignment 'pinned15')
Planners: baseline (synchronous), waves, prioritized (sideways lanes only: robots cannot fly).
Fixed seeds. Writes results/ground_robots.json.
"""

from __future__ import annotations

import json
import time
from pathlib import Path

from sirb.planner.baseline import plan_baseline
from sirb.planner.prioritized import plan_prioritized
from sirb.planner.waves import plan_waves
from sirb.spec import new_show
from sirb.verify import verify

LIMITS = {"vmax_h": 1.5, "vmax_up": 0.05, "vmax_down": 0.05, "amax": 1.0, "jmax": 3.0,
          "z_min": -0.05, "z_max": 0.05, "min_separation_m": 1.0,
          "geofence": {"x": [-60.0, 60.0], "y": [-60.0, 60.0]},
          "crowd": {"y": -500.0, "min_distance_m": 50.0},
          "sources": {k: "assumption" for k in ("vmax_h", "vmax_up", "vmax_down", "amax", "jmax",
                                               "z_min", "z_max", "geofence", "min_separation_m")}
          | {"crowd": "not used on the floor study"}}


def ground_show(n: int, task: str) -> dict:
    a = {"type": "launch_grid", "spacing_m": 2.0, "center": [-10.0, 0.0, 0.0]}
    b = {"type": "launch_grid", "spacing_m": 2.0,
         "center": [10.0, 0.0, 0.0] if task == "translate" else [-10.0, 0.0, 0.0]}
    return new_show(f"ground {task} n={n}", n, [
        {"name": "A", "shape": a, "hold_s": 1}, {"name": "B", "shape": b, "hold_s": 1}],
        seed=11, limits=LIMITS)


def run_planner(name, show, assign):
    t = time.perf_counter()
    try:
        if name == "baseline":
            fleet, _ = plan_baseline(show, assign=assign)
        elif name == "waves":
            fleet, _ = plan_waves(show, assign=assign)
        else:
            fleet, _ = plan_prioritized(show, assign=assign, lane_dz=(0.0,), lane_dh=(1.5, 3.0),
                                        time_budget_s=30.0)
    except Exception as e:  # a planner may legitimately fail to produce a plan
        return {"planner": name, "plan": False, "error": f"{type(e).__name__}: {str(e)[:160]}",
                "plan_s": time.perf_counter() - t}
    plan_s = time.perf_counter() - t
    rep = verify(fleet, show["limits"])
    return {"planner": name, "plan": True, "plan_s": plan_s, "verified_ok": rep.ok,
            "min_separation_m": rep.min_separation, "violation_counts": rep.violation_counts,
            "duration_s": fleet.t_end}


def main():
    rows = []
    for n in (16, 36, 64, 100):
        for task, assign in (("translate", "hungarian_sq"), ("shuffle", "random"),
                             ("swap15", "pinned15")):
            show = ground_show(n, task)
            for pl in ("baseline", "waves", "prioritized"):
                r = {"n": n, "task": task, "assignment": assign, **run_planner(pl, show, assign)}
                rows.append(r)
                print({k: r.get(k) for k in ("n", "task", "planner", "plan", "verified_ok",
                                              "min_separation_m", "error")}, flush=True)
    Path("results/ground_robots.json").write_text(json.dumps(
        {"note": "exploratory simulation, assumed limits, points on a flat floor; see docstring",
         "limits": LIMITS, "rows": rows}, indent=1))


if __name__ == "__main__":
    main()
