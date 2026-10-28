# Changelog

Format follows Keep a Changelog. Versions before 0.1.0 were internal milestones.

## [Unreleased]

### Added
- Gallery show of 120 drones with original outline silhouettes (falcon, dhow, palm, figure, heart, SIRB, skyline), planned and verified like the other shows, and listed in the viewer.

### Changed
- Licence changed from MIT to Apache-2.0 (adds an explicit patent grant).

### Changed (certificate, version 2)
- A certificate now carries the formal property, the separation robustness value, the vehicle radius and surface clearance, and a plain statement of what a PASS means. Verification refuses a certificate whose tolerances are looser than the verifier's own, and an export whose vehicle radius differs from the show's.

### Added
- Ellipsoidal (downwash style) separation check, exact by scaling the vertical axis (`sirb/clearance.py`, `verify-external --d-min-vertical`).
- Baseline comparators (sampled separation check, ORCA validated against RVO2-3D) and a benchmark judged by the exact verifier (`docs/BASELINES.md`).
- `sirb stress` and `sirb verify-external` commands (`sirb/importers.py`).
- `gnss_common_fraction` option and the correlation study `docs/GNSS_COMMON.md`.
- `points` shape for loading explicit coordinates.
- Input caps (5000 drones, 64 stages, 3600 s holds) and argument checks for `margin`.
- CI on Linux, Windows and macOS with Python 3.12 and 3.13; wheel build check; release workflow.

## [0.1.0]

### Added
- Second verifier based on interval branch and bound, used for differential testing against the exact verifier (`sirb/verify2.py`).
- Margin tool: `python -m sirb margin` returns the nominal separation needed for a simulated risk target and can export the plan at that scale.
- Versioned GNSS parameter files with a JSON schema and mandatory provenance (`docs/GNSS_PARAMS.md`).
- Sensitivity study, hazard table, scaling profile to 4000 drones, mutation testing with killer tests, ground robot grid planner (exploratory).
- Viewer: Arabic right to left layout, automated accessibility check, frame rate bench.
- `--version` flag, `py.typed`, editorconfig, citation file, contributing guide, Makefile help.
- CI check that planned fleets and viewer data reproduce.

### Changed
- Targets are snapped to a 2^-20 m grid and assignment costs are exact integers, so planned fleets and their hashes are identical on Windows, Linux and the CI runner. Result files derived from planned fleets were regenerated (`docs/dev/CHANGES.md`).
- The verifier refines multiple roots locally; before this the reported minimum could be too large by up to 2.3e-7 m.
- Process records moved to `docs/dev/`; `plan.md` became `docs/ROADMAP.md`.

### Fixed
- A baseline planner defect found by mutation testing (`docs/MUTATION.md`).
