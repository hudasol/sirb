# Sirb (سرب)

*Sirb* is Arabic for "flock" or "squadron".

A simulation-first suite that turns shape sequences into multi-drone show trajectories, checks them with an exact separation verifier that shares no logic with the planners, stress-tests them in Monte-Carlo simulation against wind, GNSS degradation, drone failures and comms/battery events, and writes a report with the measured margins.

> **Safety statement.** Sirb is a simulation and planning research tool. It is **not flight-certified** and makes **no claim of regulatory compliance**. It has no flight-control, radio or MAVLink code and cannot command real aircraft; exports are files labeled "simulation-validated only". A PASS verdict means only that the polynomial paths as written satisfy the limits declared in the show file; it is not an approval, a safety case or evidence that a real show is safe. A real drone show needs an operator safety case and approvals from the relevant UAE aviation authorities. There is no real flight data behind any result here.

## Status

Built and committed: show spec and shapes (M1), assignment (M2), planners (M3), verifier and certificate (M4), disturbance models, stress simulator, contingency behaviours and pre-flight validator (M5, M6, M6b code), static viewer (M7), file export (M8), command line, and the critique rounds in `docs/REVIEW_ROUNDS.md`. Tags exist for M0 to M4, M7 and M8. Not done: milestone tags for M5/M6/M6b (the stress experiments in `results/stress_*.json` are still being generated), deployment (M9: no live URL, no CI badge), `docs/REPORT.md` and the final summary. See `plan.md` for the milestone definitions.

## What the evidence supports

- The verifier is **independent in code and method** from the planners (exact polynomial root finding versus planner-side sampling; `sirb/verify.py` imports nothing from Sirb). It is **not independent in authorship**: the same people wrote both, and nobody outside the project has reviewed it. Its guarantee is exact minimum separation and limit checks for the trajectories *as written*, under the declared model (drones are points; limits come from the show file, many of which are labeled assumptions).
- On the four demo shows, Hungarian assignment with synchronous straight-line legs already verifies (`results/planners.json`). The wave scheduler and the prioritized planner match it there, and **neither produced a plan on the deliberately crossed `pinned15` assignment in any of the four shows** (wave scheduling got stuck; prioritized hit its 30 s budget). The benchmark therefore does not show that the stronger planners add capability.
- Stress-test probabilities are properties of the simulated model with assumed parameters (`docs/METHODS.md`, sections 5 and 9). They are not real-world risk. Several sweeps go beyond the vehicle's published wind limit and the model has no actuator limit; read the caveats in METHODS section 5 before quoting any row.
- Limited-search novelty claim: I found no open-source drone-show tool that publishes separation margins under modelled GNSS error or ships a verifier independent of the planner. That search was limited (`docs/RESEARCH.md` section 1.4).

Terminology: the "certificate" Sirb writes is a machine-readable verification record of what the exact verifier computed. It is not an airworthiness certificate, an approval or a safety clearance; the viewer calls it a verification record. A verdict is `PASS` only if nothing failed and every limit family was supplied (otherwise `INCOMPLETE`).

## Quickstart

```
pip install -e ".[dev]"
pytest -q && ruff check .

python -m sirb validate shows/sirb_text_100.json
python -m sirb plan shows/sirb_text_100.json --out /tmp/export.json   # plan + verify + export bundle
python -m sirb check /tmp/export.json                                 # re-check hash and certificate

python -m http.server -d viewer 8000     # viewer at http://localhost:8000/ (must be served over HTTP)
```

Exit codes: 0 ok, 1 invalid or tampered input, 2 the plan verified but failed a declared limit. The check on an export detects accidental change (hash plus recomputation); it is **not authentication**, because anyone who edits a file can also recompute its hash.

Reproduction: `make reproduce` runs install, tests, demo shows, planner benchmark, viewer data and the stress experiments with fixed seeds. On Windows without `make`, run the `python scripts/...` lines from the `Makefile` in the repository root. Numbers are deterministic given the library versions; wall-clock fields (`*_runtime_s`, `wall_s`, `*_cpu_s`) are machine dependent and will differ between runs. Floating-point results can differ in the last bits across CPUs or BLAS builds.

## Layout

| Path | Purpose |
|---|---|
| `sirb/spec.py`, `shapes.py` | show file, validator, procedural shapes |
| `sirb/assign.py`, `planner/` | assignment methods; baseline, wave and prioritized planners |
| `sirb/verify.py`, `certificate.py` | exact verifier and reproducible certificate |
| `sirb/disturbance.py`, `stress.py`, `contingency.py` | Monte-Carlo stress model |
| `sirb/preflight.py`, `export.py`, `cli.py` | consistency validator, file export, command line |
| `viewer/` | static three.js viewer, English/Arabic |
| `docs/` | `METHODS.md` (assumptions and sim-to-real gap), `VERIFIER.md`, `SHOW_SCHEMA.md`, `RESEARCH.md`, `DATA_SOURCES.md`, `REVIEW_ROUNDS.md` |

## Limitations

Simulated drones follow a simplified vehicle model. Vehicle numbers come from published datasheets where available and are labeled assumptions otherwise; in particular the 2.0 m separation threshold is an assumption, while the 50 m crowd buffer follows cited guidance whose UAE applicability was **not verified**. GNSS position-domain error parameters are assumptions. Wind uses Dryden-intensity turbulence built for piloted aircraft. See `docs/RESEARCH.md` sections 3 and 5 and `docs/METHODS.md` section 9.

## Related work

`hudasol/marsad` (GNSS position trust) and the Gulf GNSS interference project are separate repos; Sirb's GNSS model is designed to be parameterizable from their outputs later. They are not modified or imported.

## License

MIT for Sirb's own code. See `docs/DATA_SOURCES.md`.
