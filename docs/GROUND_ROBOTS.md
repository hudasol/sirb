# Exploratory: ground robots on a grid (proposal seed)

**Exploratory. Simulation only. A proposal seed, not an achievement.** Generated from `results/ground_robots.json` by `scripts/make_ground_doc.py`.

Benign warehouse style coordination: robots start on a square grid at 2.0 m pitch and the same planners and exact verifier are applied with the altitude band closed. Robots are points on a flat floor with assumed limits (max speed 1.5 m/s, max acceleration 1.0 m/s², required separation 1.0 m). There is no localisation error, no wheel dynamics and no obstacles. Nothing here says anything about a real robot.

| n | Task | Planner | Plan produced | Verified | Min separation (m) | Duration (s) |
|---|---|---|---|---|---|---|
| 16 | translate | baseline | yes | yes | 2.000 | 27.5 |
| 16 | translate | waves | yes | yes | 2.000 | 27.5 |
| 16 | translate | prioritized | yes | yes | 2.000 | 27.5 |
| 16 | shuffle | baseline | yes | no | 0.000 | 12.8 |
| 16 | shuffle | waves | no | no |  |  |
| 16 | shuffle | prioritized | no | no |  |  |
| 16 | swap15 | baseline | yes | no | 0.000 | 10.1 |
| 16 | swap15 | waves | no | no |  |  |
| 16 | swap15 | prioritized | no | no |  |  |
| 36 | translate | baseline | yes | yes | 2.000 | 27.5 |
| 36 | translate | waves | yes | yes | 2.000 | 27.5 |
| 36 | translate | prioritized | yes | yes | 2.000 | 27.5 |
| 36 | shuffle | baseline | yes | no | 0.000 | 16.9 |
| 36 | shuffle | waves | no | no |  |  |
| 36 | shuffle | prioritized | no | no |  |  |
| 36 | swap15 | baseline | yes | no | 0.000 | 15.0 |
| 36 | swap15 | waves | no | no |  |  |
| 36 | swap15 | prioritized | no | no |  |  |
| 64 | translate | baseline | yes | yes | 2.000 | 27.5 |
| 64 | translate | waves | yes | yes | 2.000 | 27.5 |
| 64 | translate | prioritized | yes | yes | 2.000 | 27.5 |
| 64 | shuffle | baseline | yes | no | 0.000 | 22.6 |
| 64 | shuffle | waves | no | no |  |  |
| 64 | shuffle | prioritized | no | no |  |  |
| 64 | swap15 | baseline | yes | no | 0.000 | 25.5 |
| 64 | swap15 | waves | no | no |  |  |
| 64 | swap15 | prioritized | no | no |  |  |
| 100 | translate | baseline | yes | yes | 2.000 | 27.5 |
| 100 | translate | waves | yes | yes | 2.000 | 27.5 |
| 100 | translate | prioritized | yes | yes | 2.000 | 27.5 |
| 100 | shuffle | baseline | yes | no | 0.000 | 29.1 |
| 100 | shuffle | waves | no | no |  |  |
| 100 | shuffle | prioritized | no | no |  |  |
| 100 | swap15 | baseline | yes | no | 0.000 | 25.5 |
| 100 | swap15 | waves | no | no |  |  |
| 100 | swap15 | prioritized | no | no |  |  |

Verified plans out of runs, by task and planner:

| Task | Planner | Verified / runs |
|---|---|---|
| shuffle | baseline | 0 / 4 |
| shuffle | prioritized | 0 / 4 |
| shuffle | waves | 0 / 4 |
| swap15 | baseline | 0 / 4 |
| swap15 | prioritized | 0 / 4 |
| swap15 | waves | 0 / 4 |
| translate | baseline | 4 / 4 |
| translate | prioritized | 4 / 4 |
| translate | waves | 4 / 4 |

Planner failures (no plan produced), by cause:

| Planner | Error | Sizes affected |
|---|---|---|
| prioritized | RuntimeError: transition 0: no feasible plan | 16, 36, 64, 100 |
| waves | RuntimeError: wave scheduling stuck | 16, 36, 64, 100 |

The waves and prioritized planners separate drones mostly by altitude or by lateral lanes. With the altitude band closed and a dense 2 m grid they cannot find a schedule when assignments cross. This is a limitation of those planners on this task, not evidence about any robot.

## Reading

A planner that fails here fails under the stated assumed limits and gets no credit or blame beyond this simulation. A verified plan is guaranteed only under the model: point robots, exact polynomial paths, the limits above.
