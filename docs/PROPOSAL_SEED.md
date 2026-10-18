# Proposal seed: indoor validation of the Sirb planner and verifier

**Status: PROPOSAL, not an achievement.** Nothing described here has been done. No partner, host
laboratory, funder or aircraft has been approached or has agreed to anything, and nobody endorses this
document. No cost figures are given because none were retrieved from a source.

## 1. Problem
Sirb produces multi drone trajectories, an exact verification of their separation and limits, and
simulated stress results. Every stressed number depends on assumed disturbance and tracking models
(`docs/METHODS.md` section 9), and no real flight data exists. A credible next step is to replace
assumptions with measurements in a setting where failures are cheap and contained.

## 2. Aim
Test, indoors and at small scale, whether trajectories that Sirb verifies are followed closely enough
for the verified separation to hold, and measure the real tracking error that the simulator currently
assumes.

## 3. Questions (and what would count as an answer)
1. How large is the real position tracking error of small multirotors following Sirb polynomials, as a
   function of speed, acceleration and formation density? Answer: error distributions from motion
   capture, compared with the simulated `mean_max_tracking_error_m` under the same trajectories.
2. Does the verified minimum separation (exact, on the polynomials) predict the measured minimum
   separation once tracking error is added? Answer: paired comparison per show, with the gap reported.
3. Which assumed parameters (tracking bandwidth omega_n, damping zeta, drag gain) can be identified
   from the logs, and do the identified values change the simulated violation probabilities? Answer:
   refit, re run `scripts/sensitivity.py`, report the change.
4. Do the contingency behaviours (hold, descend, corridor) behave in practice as in simulation for the
   drone that has the event, in a scaled, netted setting? Answer: measured trajectories of an induced
   command loss with a safe, pre approved failure injection.

## 4. Method (planned)
- Scale the shows down (smaller formation, lower speeds) with `sirb` limits set from the borrowed
  aircraft's own documentation, so the verifier checks the real vehicle limits.
- Export plans as files (the existing export). Any conversion to a vehicle or ground station format,
  and any flight, is done by the host laboratory's own tools and procedures; Sirb contains no flight
  control, radio or MAVLink code and will not be extended to include it.
- Record positions with the host laboratory's motion capture system; compute the pairwise minimum
  distance with the same exact method on the measured, piecewise linear paths.
- Use fixed predictions registered before each batch (`docs/PREDICTIONS.md` style), then report hits and
  misses.

## 5. Resources needed (no costs stated)
Borrowed access to an indoor flight space with motion capture and a safety net, a small number of
multirotors with documented limits, and a supervising lab member who owns the safety case and
procedures. Funding scope, if any, is for the researcher's time and consumables; the amount is not
estimated here.

## 6. Safety and ethics
Indoor, netted, institution approved procedures, host laboratory responsible for all flight
operations. Scope is entertainment style formations and benign multi robot coordination only. No
payload, targeting or weapon related use. No outdoor flight and no flight near people is proposed.

## 7. Risks
Small indoor aircraft differ from outdoor show drones (size, downwash, GNSS free positioning), so
results transfer only partly; motion capture is not GNSS, so GNSS error models stay untested; the
sample of runs will be small, so intervals will be wide.

## 8. Outputs
An open dataset of trajectories and logs (licence to be agreed with the host), a refit of the tracking
model, an updated sim to real section, and a short report that separates what was measured from what
remains simulated.

## 9. What this document does not claim
No approval, no partnership, no endorsement, no cost estimate, no regulatory conclusion. A real
outdoor show would still need an operator safety case and approvals from the relevant UAE aviation
authorities.
