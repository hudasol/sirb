# Roadmap

Scope: a planner, exact verifier and stress tester for multi drone show trajectories. Simulation and planning only.

## Delivered

| Version | Scope |
|---|---|
| 0.0.x | Research notes, show format and shapes, assignment, baseline, wave and prioritized planners |
| 0.0.x | Exact verifier with verification record, second verifier for differential testing |
| 0.0.x | Monte Carlo stress model, contingency behaviours, pre flight validator |
| 0.0.x | Static three.js viewer (English, Arabic), file export, command line |
| 0.1.0 | Sensitivity study, hazard table, GNSS parameter files, margin tool, scaling profile, mutation testing, reproducible planned fleets across platforms |

## Open

| Item | Notes |
|---|---|
| Relative GNSS error between show drones | The only unsourced parameter the stressed verdicts depend on. Needs measurements from real hardware (`docs/GNSS_SOURCED.md`). |
| Indoor validation | Proposed in `docs/PROPOSAL_SEED.md`, not started. |
| Planner capability | The wave and prioritized planners do not beat the baseline on the demo shows (`docs/PREDICTIONS.md`). |
| Viewer frame rate on a GPU | Only software rendering was measured (`docs/VIEWER.md`). |
| Native review of `docs/SUMMARY_AR.md` | Arabic text has not been reviewed by a native speaker. |
| Licence decision | Options in `docs/LICENSE_PROPOSAL.md`. |

## Principles

- No flight control, radio or MAVLink code; exports are files marked simulation validated only.
- Every reported number comes from a seeded script and a file under `results/`.
- What is guaranteed under the model, what is simulated and what is only planned stay separate in every document.
- Third party material and its licence is recorded in `docs/DATA_SOURCES.md`.
