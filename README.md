# Sirb (سرب)

*Sirb* is Arabic for "flock" or "squadron".

A simulation-first suite that turns shape sequences into collision-free, dynamically feasible multi-drone show trajectories, stress-tests them against wind, GNSS degradation, drone failures and battery variance, and outputs a safety report with quantified margins and an independently verified separation certificate.

> **Safety statement.** Sirb is a simulation and planning research tool. It is **not flight-certified** and makes **no claim of regulatory compliance**. It has no flight-control, radio or MAVLink code and cannot command real aircraft; exports are files labeled "simulation-validated only". A real drone show needs an operator safety case and approvals from the relevant UAE aviation authorities. There is no real flight data behind any result here.

## Status

Phase 0 and 1 complete (research and plan). Build milestones follow `plan.md`. Nothing below the research layer is implemented until its milestone tag exists.

## Problem

Large drone shows are a visible UAE strength. The investigated failures I could retrieve (NTSB Orlando 2024, ATSB Melbourne 2023) came from configuration errors, wind beyond the airframe limit and weak oversight, with GNSS interference attributed in some other incidents without a published investigation. Planning tools check the nominal path; what I found no open-source tool publishing is how separation margins degrade under modelled disturbances, or a verifier independent of the planner. That claim is from a limited search; see `docs/RESEARCH.md` for evidence and caveats.

## Method (planned)

1. Show spec: JSON schema, procedural shapes, keyframed sequences.
2. Assignment: optimal drone-to-target matching per transition.
3. Trajectories: baseline and prioritized planner under speed, acceleration, jerk, altitude and geofence limits.
4. Independent verifier: exact continuous-time minimum separation per polynomial segment, limit checks, certificate.
5. Stress tests: Monte-Carlo over wind, GNSS, comms, battery and failures, with risk-versus-severity curves.
6. Contingencies and a pre-flight consistency validator.
7. Static three.js viewer, Arabic/English.

## Limitations

Simulated drones follow a simplified vehicle model. Vehicle numbers come from published datasheets where available and are labeled assumptions otherwise. GNSS position-domain error parameters are assumptions. Wind uses Dryden-form turbulence built for piloted aircraft. See `docs/RESEARCH.md` sections 3 and 5.

## Quickstart

```
pip install -e ".[dev]"
pytest
```

Further commands appear here as milestones land.

## Related work

`hudasol/marsad` (GNSS position trust) and the Gulf GNSS interference project are separate repos; Sirb's GNSS model is designed to be parameterizable from their outputs later. They are not modified or imported.

## License

MIT for Sirb's own code. See `docs/DATA_SOURCES.md`.
