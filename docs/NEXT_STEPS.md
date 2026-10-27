# Next steps

Everything under "Backlog" is small enough for a short working session. Everything under "Future
robotics proposals" is **future work, not built, not claimed**; it is written as a starting point
for proposals only.

## Backlog (each item sized for one short session)
1. **Pin Actions by commit SHA**: the repository, CI and the Pages workflow exist; the Actions are still pinned by tag.
2. **Deploy the viewer** (`viewer/` is fully static; Vercel or Render static site, publish dir `viewer`).
3. **Fix the n=2000 scaling row**: the sphere target extends below the altitude floor; lift the centre or
   raise the floor in that experiment, then re-run `make stress`.
4. **More shapes**: extend the stroke font past the 13 letters (still original glyphs only).
5. **Planner capacity**: the prioritized planner fails on crossing assignments at 200+ drones; try
   per-wave replanning or a conflict-based search over delays.
6. **Calibrate the stress model** against any real log the owner can legitimately obtain (tracking
   response, wind response). Until then every stressed result is model-conditional.
7. **Downwash margin**: add an explicit vertical separation rule/model (literature-cited) to the verifier
   and viewer.
8. **Viewer tests in CI**: keyboard, reduced motion, light theme and `file://` checks exist and pass
   locally (`results/viewer_checks.json`); they are not run in CI yet.
9. **Export authentication**: the export hash detects accidents, not tampering by an adversary; add
   signing if exports are ever exchanged between organisations.
10. **Verify the current UAE UAS rules** from the authority's own pages (GCAA CAR-UAS status was not
    verified as current) and keep a dated summary in RESEARCH.md. Never claim compliance.

11. **Generated docs in CI**: run the doc generators and `scripts/check_doc_numbers.py` in CI and fail
    when a committed generated file differs from a fresh build.
12. **Measure the relative GNSS error** between nearby low cost receivers (the parameter every stressed
    verdict hinges on, `docs/GNSS_SOURCED.md`), for example two receivers on a short static baseline.

## Future robotics proposals (not built)
- **GNSS-aware show planning**: parameterize the GNSS error model from outputs of a separate
  Gulf-GNSS interference study so separation margins respond to regional interference levels.
  Sirb's `gnss_error` is isolated for this; no coupling exists today.
- **Multi-robot coordination beyond shows** (warehouse, inspection): reuse the exact polynomial
  separation verifier as an independent checker for any planner's output.
- **Hardware-in-the-loop evidence**: autopilot SITL/HIL runs to replace the generic tracking loop
  with an identified one, and to produce the first non-simulated comparison.
- **Fleet-level contingency optimization**: choose per-drone fallback behaviours to minimize
  simulated conflict probability, subject to the same stated assumptions.
- **Regulator-facing evidence packs**: structured, versioned verification records that an operator
  can attach to their own safety case (the operator, not Sirb, makes the case).
