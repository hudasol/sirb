# Exploratory: ground robots on a grid (proposal seed)

**Exploratory. Simulation only. A proposal seed, not an achievement.** Generated from `results/ground_robots.json` by `scripts/make_ground_doc.py`.

Benign warehouse style coordination: robots start on a square grid at 2.0 m pitch and the planners and the exact verifier are applied with the altitude band closed. Robots are points on a flat floor with assumed limits (max speed 1.5 m/s, max acceleration 1.0 m/s², required separation 1.0 m). There is no localisation error, no wheel dynamics and no obstacles. Nothing here says anything about a real robot.

Planners: `baseline` (straight synchronous legs), `waves`, `prioritized` (lanes only, robots cannot fly) and `grid` (`sirb/planner/grid.py`, added later): prioritised space time A* on the lattice with waiting, vertex and edge swap reservations, parked robots reserving their goal, and lockstep rest to rest steps of one cell. Grid settings (fixed before the recorded run): pitch 2.0 m, margin 2 free cells around the start and goal slots, up to 30 priority orders, 60 s budget per transition.

| n | Task | Planner | Plan produced | Verified | Min separation (m) | Duration (s) | Plan time (s) | Failure cause |
|---|---|---|---|---|---|---|---|---|
| 16 | translate | baseline | yes | yes | 2.000 | 27.5 | 0.02 |  |
| 16 | translate | waves | yes | yes | 2.000 | 27.5 | 0.03 |  |
| 16 | translate | prioritized | yes | yes | 2.000 | 27.5 | 0.27 |  |
| 16 | translate | grid | yes | yes | 2.000 | 36.9 | 0.07 |  |
| 16 | shuffle | baseline | yes | no | 0.000 | 12.8 | 0.28 |  |
| 16 | shuffle | waves | no | no |  |  | 0.00 | RuntimeError: wave scheduling stuck |
| 16 | shuffle | prioritized | no | no |  |  | 0.96 | RuntimeError: transition 0: no feasible plan |
| 16 | shuffle | grid | yes | yes | 1.414 | 29.9 | 0.09 |  |
| 16 | swap15 | baseline | yes | no | 0.000 | 10.1 | 0.14 |  |
| 16 | swap15 | waves | no | no |  |  | 0.00 | RuntimeError: wave scheduling stuck |
| 16 | swap15 | prioritized | no | no |  |  | 0.32 | RuntimeError: transition 0: no feasible plan |
| 16 | swap15 | grid | yes | yes | 1.414 | 16.0 | 0.02 |  |
| 36 | translate | baseline | yes | yes | 2.000 | 27.5 | 0.02 |  |
| 36 | translate | waves | yes | yes | 2.000 | 27.5 | 0.03 |  |
| 36 | translate | prioritized | yes | yes | 2.000 | 27.5 | 0.10 |  |
| 36 | translate | grid | yes | yes | 2.000 | 36.9 | 0.11 |  |
| 36 | shuffle | baseline | yes | no | 0.000 | 16.9 | 0.01 |  |
| 36 | shuffle | waves | no | no |  |  | 0.00 | RuntimeError: wave scheduling stuck |
| 36 | shuffle | prioritized | no | no |  |  | 2.25 | RuntimeError: transition 0: no feasible plan |
| 36 | shuffle | grid | yes | yes | 1.414 | 68.3 | 1.30 |  |
| 36 | swap15 | baseline | yes | no | 0.000 | 15.0 | 0.02 |  |
| 36 | swap15 | waves | no | no |  |  | 0.00 | RuntimeError: wave scheduling stuck |
| 36 | swap15 | prioritized | no | no |  |  | 1.07 | RuntimeError: transition 0: no feasible plan |
| 36 | swap15 | grid | yes | yes | 1.414 | 26.4 | 0.04 |  |
| 64 | translate | baseline | yes | yes | 2.000 | 27.5 | 0.02 |  |
| 64 | translate | waves | yes | yes | 2.000 | 27.5 | 0.07 |  |
| 64 | translate | prioritized | yes | yes | 2.000 | 27.5 | 0.20 |  |
| 64 | translate | grid | yes | yes | 2.000 | 36.9 | 0.22 |  |
| 64 | shuffle | baseline | yes | no | 0.000 | 22.6 | 0.04 |  |
| 64 | shuffle | waves | no | no |  |  | 0.01 | RuntimeError: wave scheduling stuck |
| 64 | shuffle | prioritized | no | no |  |  | 14.55 | RuntimeError: transition 0: no feasible plan |
| 64 | shuffle | grid | no | no |  |  | 8.80 | blocked_by_reservations |
| 64 | swap15 | baseline | yes | no | 0.000 | 25.5 | 0.03 |  |
| 64 | swap15 | waves | no | no |  |  | 0.02 | RuntimeError: wave scheduling stuck |
| 64 | swap15 | prioritized | no | no |  |  | 4.19 | RuntimeError: transition 0: no feasible plan |
| 64 | swap15 | grid | yes | yes | 1.414 | 47.3 | 2.09 |  |
| 100 | translate | baseline | yes | yes | 2.000 | 27.5 | 0.32 |  |
| 100 | translate | waves | yes | yes | 2.000 | 27.5 | 0.22 |  |
| 100 | translate | prioritized | yes | yes | 2.000 | 27.5 | 1.11 |  |
| 100 | translate | grid | yes | yes | 2.000 | 36.9 | 0.26 |  |
| 100 | shuffle | baseline | yes | no | 0.000 | 29.1 | 0.05 |  |
| 100 | shuffle | waves | no | no |  |  | 0.03 | RuntimeError: wave scheduling stuck |
| 100 | shuffle | prioritized | no | no |  |  | 16.60 | RuntimeError: transition 0: no feasible plan |
| 100 | shuffle | grid | no | no |  |  | 10.57 | blocked_by_reservations |
| 100 | swap15 | baseline | yes | no | 0.000 | 25.5 | 0.02 |  |
| 100 | swap15 | waves | no | no |  |  | 0.01 | RuntimeError: wave scheduling stuck |
| 100 | swap15 | prioritized | no | no |  |  | 1.88 | RuntimeError: transition 0: no feasible plan |
| 100 | swap15 | grid | yes | yes | 1.414 | 47.3 | 0.78 |  |

Verified plans out of runs, by task and planner:

| Task | Planner | Verified / runs |
|---|---|---|
| shuffle | baseline | 0 / 4 |
| shuffle | grid | 2 / 4 |
| shuffle | prioritized | 0 / 4 |
| shuffle | waves | 0 / 4 |
| swap15 | baseline | 0 / 4 |
| swap15 | grid | 4 / 4 |
| swap15 | prioritized | 0 / 4 |
| swap15 | waves | 0 / 4 |
| translate | baseline | 4 / 4 |
| translate | grid | 4 / 4 |
| translate | prioritized | 4 / 4 |
| translate | waves | 4 / 4 |

Planner failures (no plan produced), by cause:

| Planner | Cause | Sizes affected |
|---|---|---|
| grid | blocked_by_reservations | 64, 100 |
| prioritized | RuntimeError: transition 0: no feasible plan | 16, 36, 64, 100 |
| waves | RuntimeError: wave scheduling stuck | 16, 36, 64, 100 |

Grid planner against the baseline, same task and size. The baseline moves every robot in one straight minimum jerk leg; the grid planner stops at every cell centre (rest to rest per step) and so is slower even when nothing blocks it. For the shuffle and swap15 tasks the baseline plan does not pass the verifier, so its duration is for an invalid plan and the ratio is not a like for like comparison.

| n | Task | Baseline duration (s) | Baseline verified | Grid duration (s) | Grid steps | Grid / baseline duration | Grid priority orders tried |
|---|---|---|---|---|---|---|---|
| 16 | shuffle | 12.8 | no | 29.9 | 8 | 2.33 | 3 |
| 36 | shuffle | 16.9 | no | 68.3 | 19 | 4.05 | 23 |
| 64 | shuffle | 22.6 | no | no plan |  |  |  |
| 100 | shuffle | 29.1 | no | no plan |  |  |  |
| 16 | swap15 | 10.1 | no | 16.0 | 4 | 1.59 | 1 |
| 36 | swap15 | 15.0 | no | 26.4 | 7 | 1.76 | 2 |
| 64 | swap15 | 25.5 | no | 47.3 | 13 | 1.86 | 3 |
| 100 | swap15 | 25.5 | no | 47.3 | 13 | 1.86 | 3 |
| 16 | translate | 27.5 | yes | 36.9 | 10 | 1.34 | 1 |
| 36 | translate | 27.5 | yes | 36.9 | 10 | 1.34 | 1 |
| 64 | translate | 27.5 | yes | 36.9 | 10 | 1.34 | 1 |
| 100 | translate | 27.5 | yes | 36.9 | 10 | 1.34 | 1 |

The waves and prioritized planners separate drones mostly by altitude or by lateral lanes. With the altitude band closed and a dense 2 m grid they cannot find a schedule when assignments cross. This is a limitation of those planners on this task, not evidence about any robot.

## The grid planner

Every produced grid plan was passed to the exact verifier. Prioritised planning is incomplete: a robot planned late can be boxed in by robots already parked at their goals or by the reservations of earlier robots, and an earlier robot never steps aside for a later one. Where the grid planner has no row marked verified, the failure cause column gives the reason it reported after trying its priority orders. The cause `blocked_by_reservations` means the goal was reachable on the empty lattice but no conflict free path existed given the reservations already made; `enclosed_by_parked_robots` means parked robots cut the robot off even statically.

Separation. In every step all robots follow the same profile, so the relative position of two robots moves along a straight segment between lattice points. With no shared cell and no head on swap in the discrete plan, the closest approach is reached by two perpendicular moves past diagonal neighbours, pitch divided by sqrt 2 (`worst_case_separation` in `sirb/planner/grid.py`, enumerated over all step pairs and checked against sampled legs in `tests/test_grid_planner.py`). The planner refuses a pitch for which that is below the required separation. The verifier remains the judge of each plan.

Provenance. The restart policy of the grid planner (bumping a failed robot to the front of the priority order) was changed during development while looking at these same tasks, because an earlier policy with fewer bump restarts failed on more of them. The grid rows are therefore a development result on these tasks, not a held out measurement, and the sizes and seeds are too few to say how often the planner succeeds in general. With the final policy, many more priority orders and a wider margin (within a longer time budget) were tried by hand on the failing shuffle cases and did not produce a plan; that check is not recorded in the results file.

## Reading

A planner that fails here fails under the stated assumed limits and gets no credit or blame beyond this simulation. A verified plan is guaranteed only under the model: point robots, exact polynomial paths, the limits above.
