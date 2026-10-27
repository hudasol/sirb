# Product readiness: gaps and plan

Written 2026-10-28 after an audit of the repository (code review of the verifier, planners, stress model, command line and viewer; a coverage run of 860 tests at 95 percent line coverage). Status is updated as work lands. Items are ordered by how much they would mislead or block a reader.

## What was found

| Id | Gap | Status |
|---|---|---|
| B1 | No external reference for any claim. Comparisons existed only against sirb's own baseline planner. | Done: sampled checker and ORCA comparators judged by the exact verifier (`docs/BASELINES.md`). ORCA matches the RVO2-3D reference on 2400 random states (1e-3 m/s). |
| B2 | A brief close approach could be missed by frame based checks; the size of that risk was unquantified. | Done: 4.13 percent false pass at 0.5 s, 0.17 percent at 0.2 s, 0 at 0.05 s on 3000 random crossings. |
| B3 | Labeled goals (drone i to point i) were not exercised; the Hungarian step hides the hard case. | Done: labeled and free assignment variants of every scenario. |
| B4 | No sequential convex programming baseline on a published scenario. | Open. Needs cvxpy (Apache-2.0, optional extra) and the Augugliaro et al. 2012 setup. |
| H7 | No caps on show size, so a hostile file could exhaust memory. | Done: 5000 drones, 64 stages, 3600 s holds. |
| M1 | Margin command accepted impossible arguments. | Done. |
| M7 | CI covered Linux and one Python only. | Done: Linux, Windows, macOS on 3.12 and 3.13, plus wheel build and clean install test. A release workflow attaches the build. |
| H3 | The wind headline could read as a limit. | Already worded as not a limit in the README; the model has no common mode wind failure (see limits below). |
| H4 | Stressed verdicts hinge on treating absolute GNSS error as relative error. | Open. Plan: add a common mode fraction (default 0 so existing results are unchanged) and label margin results UNCALIBRATED until a differential GNSS source is found. |
| H5 | Real shows cannot be loaded. | Partly done: a `points` shape takes explicit coordinates and `verify-external` checks a sampled CSV (piecewise linear, separation only). Open: Crazyswarm polynomial import and viewer file drop. |
| H6 | The stress simulator is reachable only through scripts. | Done: `sirb stress`. |
| M2 | No machine readable JSON Schema for show files; unknown keys are ignored. | Open. |
| M4 | Separation is isotropic; downwash is not modelled. | Open. |
| M5 | Type checking has 65 open errors and `py.typed` is shipped. | Open: fix or drop the marker. |
| M8 | Coverage gate and a nightly large differential run are absent. | Open. |

## Limits that remain, stated plainly

- Guarantees are under the model: the exact verifier proves separation of the planned polynomial paths, not of aircraft. Everything stressed is simulated with assumed or cited parameters.
- The second verifier shares the first verifier's semantics (same polynomial model, same units), so agreement is not independent evidence of a correct model.
- Planners other than the free assignment baseline add little on the demo shows; waves and prioritized produce slow plans (see `docs/BASELINES.md`: waves took 68 to 136 s where ORCA took 9 to 14 s) and no plan at 32 and 64 drones on the labeled antipodal sphere.
- ORCA here is a numpy reimplementation validated per step, and perfectly symmetric starts (a ring of drones swapping across the centre) resolve slowly in both it and the algorithm in general.
- No flight control, radio, or regulatory claim. Not flight certified.

## Acceptance tests for the remaining items

- B4: the SCP plan for 6 drones in the published 6 x 6 x 6 m scenario is judged by the exact verifier; the result file records feasibility, cost and runtime next to the published values, which are cited, not copied.
- H4: with common fraction 0 every existing result file reproduces bit for bit; with a nonzero fraction the margin falls monotonically.
- H5: a CSV with a time column and per drone positions round trips to a fleet whose verification result matches the original.
- H6: `sirb stress show.json --gnss file.json --trials N --seed S` reproduces `results/` rows for the same inputs.
