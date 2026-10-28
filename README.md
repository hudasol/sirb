# Sirb (سرب)

[![CI](https://github.com/hudasol/sirb/actions/workflows/ci.yml/badge.svg)](https://github.com/hudasol/sirb/actions/workflows/ci.yml)

Trajectory planner, exact separation verifier and Monte Carlo stress tester for multi drone shows, with a static bilingual (English, Arabic) viewer. *Sirb* is Arabic for a flock or squadron.

Live viewer: https://hudasol.github.io/sirb/

> **Simulation and planning only.** Not flight certified, no regulatory compliance claim, no flight control, radio or MAVLink code. A PASS verdict means the planned polynomial paths satisfy the limits declared in the show file. It is not an approval or a safety case. A real show needs an operator safety case and approvals from the relevant UAE aviation authorities. No real flight data is behind any result in this repository.

## What it does

1. **Plan.** Shape sequences (primitives, text, procedural point clouds) become piecewise quintic trajectories. Three planners: synchronous baseline, wave scheduling, prioritized planning with lanes. Assignment is Hungarian on exact integer costs.
2. **Verify.** Pairwise relative motion is a polynomial, so the minimum separation is found exactly at segment ends and real roots of the derivative. Speed, acceleration, jerk, altitude band, geofence and crowd line are checked on the same polynomials. A second verifier (interval branch and bound) shares no code with it and is used for differential testing.
3. **Stress.** Monte Carlo over Dryden turbulence, a second order tracking loop, Gauss-Markov GNSS bias, dropouts, battery, comms and motor failures, with contingency behaviours. Wilson 95% intervals, common random numbers across configurations.
4. **Report.** A verification record per plan (hash plus recomputation), a margin tool that gives the nominal separation needed for a stated simulated risk, and the viewer.

```
show.json -> validate -> assign -> plan -> verify -> export bundle -> viewer
                                     \-> stress (Monte Carlo) -> margin / report
```

## Install and use

Python 3.12 or newer, numpy and scipy only at runtime.

```
pip install -c constraints.txt -e ".[dev]"
make test                                   # ruff and pytest

python -m sirb validate shows/sirb_text_100.json
python -m sirb plan shows/sirb_text_100.json --out export.json
python -m sirb check export.json
python -m sirb margin --help

python -m http.server -d viewer 8000        # viewer at http://localhost:8000/
```

Exit codes: 0 success, 1 invalid or modified input, 2 plan verified but violates a declared limit. `check` detects accidental change (hash and recomputation). It is not authentication.

## Results

Generated from `results/*.json`; every number traces to a seeded script.

### Headlines

<!-- HEADLINES:START -->
- Demo show `sirb_text_100` (100 drones, 156 s): exact verified minimum separation 2.189 m against d_min 2.0 m, verdict PASS (model: drones as points on the planned polynomials).
- Demo show `torus_300` (300 drones, 107 s): exact verified minimum separation 2.196 m against d_min 2.0 m, verdict PASS (model: drones as points on the planned polynomials).
- Demo show `wall_500` (500 drones, 132 s): exact verified minimum separation 2.448 m against d_min 2.0 m, verdict PASS (model: drones as points on the planned polynomials).
- Simulated wind (ASSUMED models), 100 drone show: highest tested wind with a Wilson upper bound at most 5%: 12 m/s at 20 ft; first tested wind with an observed violation: 15 m/s. This is not a wind limit: the model has no actuator limit, the sweep goes beyond the vehicle's published wind limit, and the value moves with the tracking and GNSS assumptions (`docs/SENSITIVITY.md`).
- Simulated GNSS bias (25% of drones degraded, ASSUMED): P(violation) exceeds 5% already at bias sigma 0.1 m (35.5%); the nominal margin over d_min is only 0.189 m.
- Smallest tested formation spacing in the stated scenario whose Wilson upper bound is at most 5%: 5.0 m, nominal separation 3.54 m (300 trials, a coarse grid of spacings; the margin tool's search answer is in `docs/MARGIN.md`).
- Contingency runs: largest bystander to bystander violation rate over all rows: 0.0%. Violations involving the drone that had the event are common: up to 99.5% (gnss_dropout 10% x 10 s, descend), with contact (centres within two drone radii) up to 62.0%.
- Stress study scaling: largest verified size n = 1000: plan 7.9 s + exact verification 98.1 s (wall clock on a shared machine; not comparable with `docs/PERFORMANCE.md`). Sizes 2000 did not verify because the shapes exceed the altitude band (a limit of the test shapes, not a timing result).
<!-- HEADLINES:END -->

### Verification evidence

<!-- ASSURANCE:START -->
- Second, independent verifier (interval branch and bound, `sirb/verify2.py`): 10856 fleets (8000 random, 2750 adversarial in 11 families), 13100 verdict checks: lower bound above the exact value in 0 cases (max l minus e = 0.0e+00 m), FAIL verdicts with the exact value at or above the threshold: 6 (max margin 0.0e+00 m); 30 disagreement records, all in the high multiplicity root family, largest exact minus lower bound 7.3e-08 m (`docs/VERIFIER.md`). Brute force sampling at dt = 1e-4 s against the exact value: 3034 comparisons, largest sampled minus exact 1.1e-08 m (grid plus breakpoints), largest negative gap 3.6e-15 m.
- Mutation testing (mutmut, killed over total): fresh phase 6 run on the current code: `assign.py` 93.0% of 314; `planner/baseline.py` 98.4% of 188; `planner/common.py` 95.4% of 196; `planner/prioritized.py` 82.8% of 682; `planner/waves.py` 90.0% of 450; `verify.py` 92.9% of 2383; `verify2.py` 95.3% of 988. (`docs/MUTATION.md`).
- Viewer accessibility (axe-core 4.14.0, 8 page runs): 18 violated rules before, 0 after; this is an automated check and does not replace manual testing.
- Viewer behaviour checks (keyboard, reduced motion, light theme, file protocol, RTL, overflow): 40 passed, 0 failed.
- Scale (computational study): n = 4000: plan 233 s, exact verification 164 s, peak memory 665 MB (`docs/PERFORMANCE.md`).
- Viewer, 1000 drones, two panes, software WebGL: median frame interval 317 ms (3.2 fps), JavaScript update p95 1.2 ms; smoothness on a GPU is not measured.
- Largest one at a time swings of the median minimum separation (stressed baseline): gnss_bias_sigma (1.57 m), gnss_deg_fraction (0.43 m), gnss_bias_tau (0.09 m) (`docs/SENSITIVITY.md`).
<!-- ASSURANCE:END -->

These checks show that two implementations with different methods agree on large random and adversarial sets, that the test suite detects most small code changes, and that the viewer passes an automated accessibility scan. They do not prove the verifier correct, and they are statements about the plan as written, not about aircraft.

Stress probabilities are properties of the simulated model with assumed parameters (`docs/METHODS.md`). The stressed verdicts depend on one unsourced parameter, the GNSS bias that is not shared between nearby drones; see `docs/GNSS_SOURCED.md`. On the demo shows the wave and prioritized planners match the baseline and do not add capability (`docs/PREDICTIONS.md`).

| English | Arabic (RTL) | Arabic, phone width |
|---|---|---|
| ![English viewer](viewer/screenshots/index_en_1440.png) | ![Arabic viewer](viewer/screenshots/index_ar_1440.png) | ![Arabic phone](viewer/screenshots/index_ar_390.png) |

## Layout

| Path | Purpose |
|---|---|
| `sirb/spec.py`, `shapes.py` | show file, validator, procedural shapes |
| `sirb/assign.py`, `planner/` | assignment, baseline, wave and prioritized planners |
| `sirb/verify.py`, `verify2.py`, `certificate.py` | exact verifier, second verifier, verification record |
| `sirb/disturbance.py`, `stress.py`, `contingency.py` | stress model |
| `sirb/margin.py`, `gnss_params.py`, `preflight.py` | margin tool, GNSS parameter files, pre flight checks |
| `sirb/export.py`, `cli.py` | export bundle, command line |
| `viewer/` | static three.js viewer |
| `shows/`, `data/`, `results/` | demo shows, the one external table (FAA GPS statistics), generated results |
| `scripts/` | experiments, benchmarks and document generators |
| `tests/` | unit, property (hypothesis), differential and mutation killer tests |
| `docs/` | see `docs/README.md` |

## Reproducing

`make reproduce` runs install, tests, demo shows, planner benchmark, viewer data and the stress experiments with fixed seeds. The later studies (sensitivity, sourced GNSS, margin, scaling profile, differential report) run through `scripts/run_phase6_chain.sh`. Planned fleets are identical across machines (`docs/REPRODUCIBILITY.md`). Wall clock fields differ between runs, the prioritized and grid planners stop at a time budget, and the near degenerate verifier cases depend on the LAPACK build.

## Limitations

Drones are points following a simplified vehicle model. Vehicle numbers come from datasheets where available and are labelled assumptions otherwise. The 2.0 m separation is an assumption, and the 50 m crowd buffer follows cited guidance whose UAE applicability was not verified. Wind uses Dryden turbulence intended for piloted aircraft. The verifier is independent of the planners in code and method but not in authorship, and nobody outside the project has reviewed it. See `docs/METHODS.md` section 9.

## Related projects

`hudasol/marsad` and the Gulf GNSS interference project are separate. Sirb reads GNSS parameters from a versioned file (`docs/GNSS_PARAMS.md`) and neither project is imported or modified.

## Licence and citation

Apache-2.0 for the whole repository (`LICENSE`, `NOTICE`); the vendored three.js files keep their own licence. Third party licences are listed in `docs/DATA_SOURCES.md`; the reasoning is in `docs/LICENSE_PROPOSAL.md`. Citation metadata is in `CITATION.cff`.
