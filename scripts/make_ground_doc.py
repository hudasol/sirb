"""Generate docs/GROUND_ROBOTS.md from results/ground_robots.json."""

from __future__ import annotations

import json
from pathlib import Path

R = json.loads(Path("results/ground_robots.json").read_text())


def main() -> None:
    rows = R["rows"]
    L = ["# Exploratory: ground robots on a grid (proposal seed)", "",
         "**Exploratory. Simulation only. A proposal seed, not an achievement.** Generated from "
         "`results/ground_robots.json` by `scripts/make_ground_doc.py`.", "",
         "Benign warehouse style coordination: robots start on a square grid at 2.0 m pitch and "
         "the same planners and exact verifier are applied with the altitude band closed. Robots "
         "are points on a flat floor with assumed limits "
         f"(max speed {R['limits']['vmax_h']} m/s, max acceleration {R['limits']['amax']} m/s², "
         f"required separation {R['limits']['min_separation_m']} m). There is no localisation "
         "error, no wheel dynamics and no obstacles. Nothing here says anything about a real "
         "robot.", "",
         "| n | Task | Planner | Plan produced | Verified | Min separation (m) | Duration (s) |",
         "|---|---|---|---|---|---|---|"]
    for r in rows:
        ms = r.get("min_separation_m")
        L.append(f"| {r['n']} | {r['task']} | {r['planner']} | {'yes' if r['plan'] else 'no'} | "
                 f"{'yes' if r.get('verified_ok') else 'no'} | "
                 f"{'' if ms is None else f'{ms:.3f}'} | "
                 f"{'' if r.get('duration_s') is None else f'{r['duration_s']:.1f}'} |")
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
    L += ["", "## Reading", "",
          "A planner that fails here fails under the stated assumed limits and gets no credit or "
          "blame beyond this simulation. A verified plan is guaranteed only under the model: point "
          "robots, exact polynomial paths, the limits above.", ""]
    Path("docs/GROUND_ROBOTS.md").write_text("\n".join(L))


if __name__ == "__main__":
    main()
