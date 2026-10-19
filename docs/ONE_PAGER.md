# Sirb (سرب) in one page

**What it is.** A simulation first suite: shape sequences to multi drone show trajectories, an exact
separation verifier independent of the planners, Monte Carlo stress tests (wind, GNSS, failures), a
bilingual viewer. **Simulation and planning only. Not flight-certified. No regulatory compliance claim.
Real shows need an operator safety case and approvals from the relevant UAE aviation authorities.**

**Headline numbers (generated from result files).**
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

**What is guaranteed.** Under the model (polynomial paths as written, drones as points) the verifier's
minimum separation is exact up to floating point tolerance, cross checked by a second implementation.
**What is simulated.** All stressed probabilities, under assumed models, with no flight data.
**What is only planned.** See `docs/NEXT_STEPS.md` and `docs/PROPOSAL_SEED.md`.

**Links.** Repo: https://github.com/hudasol/sirb. Method and assumptions: `docs/METHODS.md`.
