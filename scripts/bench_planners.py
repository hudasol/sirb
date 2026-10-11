"""M2/M3 benchmark: assignment methods and planners on demo shows, judged by the verifier.

Writes results/assignment.json and results/planners.json. Deterministic (fixed seeds).
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

from sirb import assign as A
from sirb.planner.baseline import plan_baseline
from sirb.planner.prioritized import plan_prioritized
from sirb.planner.waves import plan_waves
from sirb.spec import build_targets, load_show
from sirb.verify import verify


def assignment_table(show: dict) -> list[dict]:
    tg = build_targets(show)
    d_sep = show["limits"]["min_separation_m"]
    rows = []
    for k in range(len(tg) - 1):
        res = A.evaluate(tg[k], tg[k + 1], d_sep, seed=show["seed"] + k)
        rows.append({"transition": f"{show['stages'][k]['name']}->{show['stages'][k+1]['name']}",
                     **{m: v for m, v in res.items()}})
    return rows


def planner_row(show: dict, label: str, fn) -> dict:
    t0 = time.perf_counter()
    try:
        fleet, info = fn(show)
    except RuntimeError as exc:
        return {"planner": label, "planning_failed": True, "reason": str(exc),
                "plan_runtime_s": time.perf_counter() - t0}
    plan_s = time.perf_counter() - t0
    rep = verify(fleet, show["limits"])
    return {
        "planner": label,
        "verified_ok": rep.ok,
        "violation_counts": rep.violation_counts,
        "completion_time_s": fleet.t_end,
        "min_separation_m": rep.min_separation,
        "min_separation_is_lower_bound": rep.min_separation_is_lower_bound,
        "max_speed_h": rep.max_speed_h, "max_accel": rep.max_accel, "max_jerk": rep.max_jerk,
        "plan_runtime_s": plan_s, "verify_runtime_s": rep.runtime_s,
    }


def main(shows_dir: str = "shows", out_dir: str = "results") -> int:
    Path(out_dir).mkdir(exist_ok=True)
    asg, plans = {}, {}
    for f in sorted(Path(shows_dir).glob("*.json")):
        show = load_show(f)
        asg[f.stem] = assignment_table(show)
        plans[f.stem] = [
            planner_row(show, "baseline sync (hungarian)", lambda s: plan_baseline(s)),
            planner_row(show, "baseline async (hungarian)",
                        lambda s: plan_baseline(s, sync=False)),
            planner_row(show, "baseline sync (identity)",
                        lambda s: plan_baseline(s, assign="identity")),
            planner_row(show, "baseline 4 waves x 3 s", lambda s: plan_baseline(
                s, waves=4, stagger_s=3.0)),
            planner_row(show, "waves (hungarian)", lambda s: plan_waves(s)),
            planner_row(show, "prioritized (hungarian)",
                        lambda s: plan_prioritized(s, time_budget_s=30)),
            planner_row(show, "baseline sync (pinned15)",
                        lambda s: plan_baseline(s, assign="pinned15")),
            planner_row(show, "waves (pinned15)", lambda s: plan_waves(s, assign="pinned15")),
            planner_row(show, "prioritized (pinned15)",
                        lambda s: plan_prioritized(s, assign="pinned15", time_budget_s=30)),
        ]
        for r in plans[f.stem]:
            if r.get("planning_failed"):
                print(f"{f.stem:16s} {r['planner']:28s} PLANNING FAILED ({r['reason']}) "
                      f"after {r['plan_runtime_s']:.1f}s")
                continue
            print(f"{f.stem:16s} {r['planner']:28s} ok={r['verified_ok']!s:5s} "
                  f"T={r['completion_time_s']:7.1f} dmin={r['min_separation_m']:.2f} "
                  f"plan={r['plan_runtime_s']:.1f}s verify={r['verify_runtime_s']:.1f}s "
                  f"{r['violation_counts']}")
    Path(out_dir, "assignment.json").write_text(json.dumps(asg, indent=1))
    Path(out_dir, "planners.json").write_text(json.dumps(plans, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main(*sys.argv[1:]))
