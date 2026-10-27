# Sirb architecture

Status: written as the Phase 0/1 design and reconciled with the code in critique round 3 (`dev/REVIEW_ROUNDS.md`). Where the build differs from the design, the row or sentence says so.

## Principles

1. **Planner and verifier are separate code paths.** They share only the trajectory data format (`sirb/trajectory.py` dataclasses, no logic). The verifier never imports planner code and uses a different method (exact polynomial root finding) from the planner's own checks (sampled safety margin).
2. **Everything reported comes from code with fixed seeds.** `make reproduce` regenerates `results/*.json` and the viewer data. `docs/REPORT.md` does not exist yet, so the planned test that fails when a report and the results disagree does not exist either; today only `tests/test_assign_planners.py::test_results_files_consistent_if_present` and the CI viewer-data diff check results against code. Wall-clock fields are machine dependent.
3. **Files only.** No MAVLink, no radio, no flight-control code. Exports are labeled "simulation-validated only".
4. **Precomputed artifacts for hosting.** The viewer is static: it loads `viewer/data/index.json`, per-scenario binary position files and `certificate.json`. No backend.

## Stack

| Layer | Choice | Why |
|---|---|---|
| Core | Python 3.13, numpy, scipy | `scipy.optimize.linear_sum_assignment` for Hungarian assignment, `scipy.spatial.cKDTree` for neighbor queries, `numpy.roots` for exact segment minima. No heavy solver dependency, so CI is fast and results are reproducible |
| Optional | none at first | cvxpy and OR-tools stay out until a method needs them (SCP is deferred to `NEXT_STEPS.md` unless time allows, to keep methods interpretable) |
| Tests | pytest, hypothesis | property-based tests to attack the verifier |
| Lint | ruff | fast, one tool |
| Viewer | three.js (ES module, pinned version, vendored or from a pinned CDN URL), `InstancedMesh` | 500+ drones in one draw call; static hosting |
| CI | GitHub Actions | ruff, pytest, regenerate viewer data and diff `index.json` (no full reproduction, no browser smoke test) |
| Hosting | Vercel (static site), Render as fallback | both connectors are available; static output needs no server |

## Data model

`show.json` (format in `docs/SHOW_SCHEMA.md`, enforced by `sirb.spec.validate_show`; no separate JSON Schema file): metadata, vehicle model, limits (speed, acceleration, jerk, altitude band, geofence box; polygons are not implemented), crowd line, stages (keyframes of shapes with hold durations), generator parameters and seed.

Trajectory: per drone, a list of polynomial segments `(t0, t1, coeffs)` where `coeffs` has shape `(deg+1, 3)` in local time `tau = t - t0`. Planner output uses quintic minimum-jerk segments along chosen waypoints; the verifier accepts any polynomial degree up to 7 so adversarial tests can feed it arbitrary curves.

## Modules

| Module | Responsibility |
|---|---|
| `sirb/spec.py` | schema validation, loading |
| `sirb/shapes.py` | parametric primitives, text-to-points (built-in stroke font, no external font files), procedural 3D point clouds, keyframed sequences |
| `sirb/assign.py` | Hungarian and baselines (identity, random, greedy nearest) with path-length and conflict metrics |
| `sirb/planner/baseline.py` | straight lines with time staggering |
| `sirb/planner/waves.py` | conflict-driven wave scheduling with closed-form pairwise constraints |
| `sirb/planner/prioritized.py` | prioritized planning with altitude and sideways lanes and delay search, using the planner's own sampled margin check |
| `sirb/verify.py` | independent verifier: exact per-segment pairwise minimum separation, dynamic limits (exact extrema of speed, acceleration, jerk via polynomial roots), geofence, crowd-line distance, certificate |
| `sirb/certificate.py` | machine-readable certificate with content hash of the trajectory file, parameters, margins, and an explicit statement of assumptions |
| `sirb/disturbance.py` | wind (mean profile plus Dryden-intensity first-order Gauss-Markov turbulence), GNSS error model (white noise, Gauss-Markov bias, dropout with dead reckoning; **step and ramp spoofing models are not implemented**), discrete comms-loss / low-battery / motor-failure events (**battery capacity variance is not modelled**) |
| `sirb/stress.py` | tracking-error model, Monte-Carlo runner, Wilson intervals, `required_value` (smallest tabulated spacing whose upper bound meets a risk target); the sweeps, spacing table and scaling benchmark are driven by `scripts/run_stress.py` |
| `sirb/contingency.py` | continue, hold, descend, corridor (clearest of 8 directions judged against others' nominal paths only; it does not check the geofence), rejoin; evaluated through `scripts/run_stress.py contingency` |
| `sirb/preflight.py` | consistency validator (centre, heading, geofence versus declared standard, crowd-line distance, fleet plan hash, perturbation tests) motivated by the NTSB Orlando findings |
| `sirb/export.py`, `sirb/cli.py` | JSON export of show, fleet and certificate; command line |
| `viewer/` | three.js player |

## Verifier method (the part that must be right)

For two drones with segments on a common sub-interval `[a, b]`, the relative position `d(tau)` is a vector polynomial. `|d|^2` is a scalar polynomial `P(tau)`. The minimum over `[a, b]` is attained at an endpoint or at a real root of `P'(tau)` inside `(a, b)`. Roots are found with `numpy.roots` on `P'`, and each candidate minimum is evaluated directly from the polynomial. A pair with no overlapping interval is skipped. Pair enumeration uses a conservative axis-aligned bounding-box prefilter (the built verifier uses boxes, not spheres) per segment (padded by the separation threshold) so scaling to thousands of drones does not require all-pairs work; the prefilter can only discard pairs that provably cannot violate, and a hypothesis test compares prefiltered and brute-force results.

Speed, acceleration and jerk limits: the maximum of `|v(tau)|^2` etc. is found the same way (endpoints plus real roots of the derivative of the squared norm).

**What is guaranteed:** under the declared model (the polynomial trajectories as written, drones as spheres of the declared radius plus margin), the reported minimum separation is the exact minimum up to floating-point and root-finding tolerance. **What is not guaranteed:** that real aircraft follow the polynomials (tracking error, wind, GNSS error are addressed only statistically in M5), anything about aerodynamics such as downwash, or any regulatory property.

## Stress-test model summary

Disturbance models are listed with sources or assumption labels in `docs/RESEARCH.md` section 3 and 5 and `docs/METHODS.md` section 5. Each Monte-Carlo run perturbs a sampled nominal trajectory (step 0.2 s) with: wind-driven tracking error (second-order closed-loop tracking model per drone with a wind disturbance input), GNSS-induced position-estimate error fed into the tracking loop, discrete comms-loss, low-battery and motor-failure events with a chosen contingency behaviour. Separation is then measured on the realized paths as the exact minimum over the *piecewise-linear interpolation* of the sampled paths (`sirb.stress.exact_pair_min`). This is not the verifier's method: the verifier works on polynomials and the stress code never imports it. The stress code is cross-checked against the verifier on noise-free runs in `tests/test_stress.py`; the remaining approximation is the linear interpolation (`docs/METHODS.md` section 5).

## Security and licensing

No secrets in the repo. `docs/DATA_SOURCES.md` records every dependency and data source with its license. No code is copied from Skybrush (GPL-3.0); the show format is Sirb's own. Any future Skybrush adapter would need a license review first.
