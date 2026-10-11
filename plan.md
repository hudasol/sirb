# Sirb plan

Definition of done for the whole project: live viewer, CI passing, meaningful tests including adversarial verifier tests, one-command reproduction of every reported number, honest validation, docs consistent with the code.

| Milestone | Tag | Scope | Definition of done |
|---|---|---|---|
| M0 | v0.0.1 | Research and plan | `docs/RESEARCH.md`, `docs/ARCHITECTURE.md`, `docs/DATA_SOURCES.md`, README with safety statement pushed |
| M1 | v0.1.0 | Show spec and shapes | JSON schema and validator; primitives, text, procedural point clouds, keyframed sequences; at least 4 original demo shows; tests pass |
| M2 | v0.2.0 | Assignment | Hungarian versus identity, random, greedy; reported total path length and peak conflicts on all demo transitions; tests |
| M3 | v0.3.0 | Trajectories | baseline and prioritized planner; both respect speed, acceleration, jerk, altitude, geofence; comparison table on feasibility, completion time, minimum separation |
| M4 | v0.4.0 | Verifier | exact segment minima, dynamic limits, geofence, crowd line; certificate; hypothesis tests; corrupted-trajectory tests all flagged |
| M5 | v0.5.0 | Stress testing | Monte-Carlo over wind, tracking, GNSS, comms, battery, failures; risk-versus-severity curves; separation required for a stated risk target; scaling from tens to thousands of drones with runtime |
| M6 | v0.6.0 | Contingencies | hold, safe-corridor descent, abort-and-rejoin evaluated; failure cases reported |
| M6b | v0.6.1 | Pre-flight validator | centre, heading, geofence, crowd-line and plan-hash checks; perturbation tests |
| M7 | v0.7.0 | Viewer | three.js player, scrubbing, separation overlay, nominal versus stressed, certificate panel, 500+ drones smooth, Arabic/English with RTL, every number links to its method |
| M8 | v0.8.0 | Export | documented JSON export of show and report, files only |
| M9 | v0.9.0 | Deploy | live URL, CI badge, `make reproduce` |
| Review | v0.9.x | Six critique rounds | `docs/REVIEW_ROUNDS.md` |
| Final | v1.0.0 | REPORT, summary, video script, NEXT_STEPS | docs match code and results |

## Rules carried through every milestone

Simulation and planning only, not flight-certified, no regulatory claim. No flight-control or radio code. Entertainment and benign coordination only. No fabricated numbers: every result from seeded code, vehicle parameters cited or labeled assumptions. Cite only retrieved sources. Licenses recorded. Original or procedural shapes only. Validation honesty and a sim-to-real section.

## Decisions made without asking (reversible)

- Repo name `sirb`, MIT license for Sirb's own code.
- numpy and scipy only for the core; cvxpy and OR-tools deferred. SCP is listed in `NEXT_STEPS.md` rather than built unless time remains; the stronger method is prioritized planning, chosen for interpretability.
- Pre-flight validator added after research showed the formally investigated incidents were configuration and wind failures.
- Static viewer with precomputed artifacts; Vercel first, Render as fallback.

## Status against the definitions of done (reconciled in review round 3)

- Met: M0 to M4, M7 (viewer exists; **"500+ drones smooth" was never measured**: no frame-rate measurement is in the repo), M8.
- Code present, experiment outputs generated but uncommitted and milestone tags not yet created: M5, M6, M6b. M5's "separation required for a stated risk target" exists only as the post-processing helper `sirb.stress.required_value`; no reported value yet. "Scaling from tens to thousands of drones" is the `scaling` experiment of `scripts/run_stress.py`.
- Not started: M9 (live URL, CI badge), Final (REPORT, summary, video script, NEXT_STEPS).
- Overall definition of done: "one-command reproduction of every reported number" holds for numbers, not for wall-clock timing fields; "docs consistent with the code" is checked for the items in `tests/test_review_regressions.py` only.
