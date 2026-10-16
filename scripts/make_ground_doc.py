"""Generate docs/GROUND_ROBOTS.md from results/ground_robots.json."""

from __future__ import annotations

import json
from pathlib import Path

R = json.loads(Path("results/ground_robots.json").read_text())


def _cause(r: dict) -> str:
    """Short failure cause for a row (empty when a plan was produced)."""
    if r["plan"]:
        return ""
    if r.get("cause"):
        return r["cause"]
    return r["error"].split(" at drone")[0]


def main() -> None:
    rows = R["rows"]
    g = R["grid_settings"]
    L = ["# Exploratory: ground robots on a grid (proposal seed)", "",
         "**Exploratory. Simulation only. A proposal seed, not an achievement.** Generated from "
         "`results/ground_robots.json` by `scripts/make_ground_doc.py`.", "",
         "Benign warehouse style coordination: robots start on a square grid at 2.0 m pitch and "
         "the planners and the exact verifier are applied with the altitude band closed. Robots "
         "are points on a flat floor with assumed limits "
         f"(max speed {R['limits']['vmax_h']} m/s, max acceleration {R['limits']['amax']} m/s², "
         f"required separation {R['limits']['min_separation_m']} m). There is no localisation "
         "error, no wheel dynamics and no obstacles. Nothing here says anything about a real "
         "robot.", "",
         "Planners: `baseline` (straight synchronous legs), `waves`, `prioritized` (lanes only, "
         "robots cannot fly) and `grid` (`sirb/planner/grid.py`, added later): prioritised space "
         "time A* on the lattice with waiting, vertex and edge swap reservations, parked robots "
         "reserving their goal, and lockstep rest to rest steps of one cell. Grid settings "
         f"(fixed before the recorded run): pitch {g['pitch_m']} m, margin {g['margin_cells']} "
         f"free cells around the start and goal slots, up to {g['restarts']} priority orders, "
         f"{g['time_budget_s']:.0f} s budget per transition.", "",
         "| n | Task | Planner | Plan produced | Verified | Min separation (m) | Duration (s) "
         "| Plan time (s) | Failure cause |",
         "|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        ms = r.get("min_separation_m")
        L.append(f"| {r['n']} | {r['task']} | {r['planner']} | {'yes' if r['plan'] else 'no'} | "
                 f"{'yes' if r.get('verified_ok') else 'no'} | "
                 f"{'' if ms is None else f'{ms:.3f}'} | "
                 f"{'' if r.get('duration_s') is None else f'{r['duration_s']:.1f}'} | "
                 f"{r['plan_s']:.2f} | {_cause(r)} |")
    tot = {}
    for r in rows:
        k = (r["task"], r["planner"])
        t = tot.setdefault(k, [0, 0])
        t[1] += 1
        t[0] += bool(r.get("verified_ok"))
    L += ["", "Verified plans out of runs, by task and planner:", "",
          "| Task | Planner | Verified / runs |", "|---|---|---|"]
    for (task, pl), (ok, n) in sorted(tot.items()):
        L.append(f"| {task} | {pl} | {ok} / {n} |")
    errs = {}
    for r in rows:
        if not r["plan"]:
            errs.setdefault((r["planner"], _cause(r)), []).append(r["n"])
    L += ["", "Planner failures (no plan produced), by cause:", "",
          "| Planner | Cause | Sizes affected |", "|---|---|---|"]
    for (pl, e), ns in sorted(errs.items()):
        L.append(f"| {pl} | {e} | {', '.join(map(str, sorted(set(ns))))} |")

    by = {(r["n"], r["task"], r["planner"]): r for r in rows}
    L += ["", "Grid planner against the baseline, same task and size. The baseline moves every "
          "robot in one straight minimum jerk leg; the grid planner stops at every cell centre "
          "(rest to rest per step) and so is slower even when nothing blocks it. For the "
          "shuffle and swap15 tasks the baseline plan does not pass the verifier, so its "
          "duration is for an invalid plan and the ratio is not a like for like comparison.", "",
          "| n | Task | Baseline duration (s) | Baseline verified | Grid duration (s) "
          "| Grid steps | Grid / baseline duration | Grid priority orders tried |",
          "|---|---|---|---|---|---|---|---|"]
    for (n, task, pl), gr in sorted(by.items(), key=lambda kv: (kv[0][1], kv[0][0])):
        if pl != "grid":
            continue
        b = by[(n, task, "baseline")]
        gd = gr.get("duration_s")
        ratio = "" if gd is None else f"{gd / b['duration_s']:.2f}"
        L.append(f"| {n} | {task} | {b['duration_s']:.1f} | "
                 f"{'yes' if b.get('verified_ok') else 'no'} | "
                 f"{'no plan' if gd is None else f'{gd:.1f}'} | {gr.get('steps', '')} | "
                 f"{ratio} | {gr.get('orders_tried', '')} |")

    L += ["", "The waves and prioritized planners separate drones mostly by altitude or by "
          "lateral lanes. With the altitude band closed and a dense 2 m grid they cannot find a "
          "schedule when assignments cross. This is a limitation of those planners on this task, "
          "not evidence about any robot.", "",
          "## The grid planner", "",
          "Every produced grid plan was passed to the exact verifier. Prioritised planning is "
          "incomplete: a robot planned late can be boxed in by robots already parked at their "
          "goals or by the reservations of earlier robots, and an earlier robot never steps "
          "aside for a later one. Where the grid planner has no row marked verified, the "
          "failure cause column gives the reason it reported after trying its priority orders. "
          "The cause `blocked_by_reservations` means the goal was reachable on the empty "
          "lattice but no conflict free path existed given the reservations already made; "
          "`enclosed_by_parked_robots` means parked robots cut the robot off even statically.", "",
          "Separation. In every step all robots follow the same profile, so the relative "
          "position of two robots moves along a straight segment between lattice points. With "
          "no shared cell and no head on swap in the discrete plan, the closest approach is "
          "reached by two perpendicular moves past diagonal neighbours, pitch divided by "
          "sqrt 2 (`worst_case_separation` in `sirb/planner/grid.py`, enumerated over all step "
          "pairs and checked against sampled legs in `tests/test_grid_planner.py`). The "
          "planner refuses a pitch for which that is below the required separation. The "
          "verifier remains the judge of each plan.", "",
          "Provenance. The restart policy of the grid planner (bumping a failed robot to the "
          "front of the priority order) was changed during development while looking at "
          "these same tasks, because an earlier policy with fewer bump restarts failed on more "
          "of them. The grid rows are therefore a development result on these tasks, not a "
          "held out measurement, and the sizes and seeds are too few to say how often the "
          "planner succeeds in general. With the final policy, many more priority orders and a "
          "wider margin (within a longer time budget) were tried by hand on the failing shuffle "
          "cases and did not produce a plan; that check is not recorded in the results file.",
          "",
          "## Reading", "",
          "A planner that fails here fails under the stated assumed limits and gets no credit or "
          "blame beyond this simulation. A verified plan is guaranteed only under the model: point "
          "robots, exact polynomial paths, the limits above.", ""]
    Path("docs/GROUND_ROBOTS.md").write_text("\n".join(L))


if __name__ == "__main__":
    main()
