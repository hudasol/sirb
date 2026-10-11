# Methods, assumptions and the sim-to-real gap

Sirb is a **simulation and planning tool. It is not flight-certified and makes no claim of
regulatory compliance.** Real shows need an operator safety case and approvals from the relevant
UAE aviation authorities. Exports are files only, labeled "simulation-validated only".

Frame: x east, y north, z up, metres. Every reported number is produced by code with fixed seeds
(`make reproduce`). Items labeled **ASSUMPTION** are modelling choices, not measurements.

## 1. Trajectory format
Each drone flies a sequence of polynomial segments `(t0, t1, coeffs (deg+1, 3))`; a fleet has a
sha256 content hash. Rest-to-rest legs are quintic minimum-jerk: peak speed 1.875 L/T,
peak acceleration 5.7735 L/T^2, peak jerk 60 L/T^3 for leg length L and duration T.

## 2. Assignment (M2)
Drones are matched to formation points by the Hungarian algorithm on squared distance. For
synchronous straight-line motion this tends to avoid crossings; it is not a proof. Alternatives
(identity, random, greedy, linear-cost Hungarian, pinned) are benchmarked in `results/assignment.json`.

## 3. Planners (M3)
- *baseline*: synchronous straight legs, optional waves/stagger.
- *waves*: closed-form pairwise constraints to schedule departure waves.
- *prioritized*: per-drone routing with vertical/sideways lanes and delays, sampled at dt = 0.05 s with
  a Lipschitz safety margin dt/2 (vpeak_i + vpeak_j) plus fine refinement; 60 s time budget.
Measured limits (documented honestly in `results/planners.json`): on the demo shows the Hungarian +
synchronous baseline already verifies; the stronger planners help only on crossing assignments and
have limited capacity (they fail on 200+ drone pinned cases). Planners never import the verifier.

## 4. Independent verifier (M4) - what is *guaranteed*
Relative position of two drones is a vector polynomial per segment overlap, so |d|^2 is a scalar
polynomial. Its minimum lies at an interval endpoint or a real root of the derivative. The verifier
solves these exactly (numpy roots, AABB prefilter) and checks speed, acceleration, jerk, altitude,
geofence and crowd distance at exact extrema. **Guarantee (under the declared model):** drones are
points following the polynomials as written; the reported minimum separation is exact up to
floating-point and root-finding tolerance for every pair whose minimum is below the search radius.
Details: `docs/VERIFIER.md`. The certificate (`sirb/certificate.py`) contains no timestamps and can be
recomputed by `verify_certificate`. It does **not** cover tracking error, wind, GNSS, downwash,
failures or any regulatory property.

## 5. Stress model (M5) - what is only *simulated*
Tracking: `eps'' = -kd eps' - kp eps - kp e(t) + drag_gain * wind`, exact zero-order-hold
discretization. kp = omega_n^2, omega_n = 2 rad/s, zeta = 0.9, drag_gain = 0.3 /s: **ASSUMPTIONS**.
- Wind: mean wind at 20 ft with 1/7 power-law profile (**ASSUMPTION**); turbulence intensities from the
  Dryden low-altitude relations (MIL-HDBK-1797 as given in MathWorks documentation; their
  applicability to small multirotors is an **ASSUMPTION**); frozen-turbulence delay along the wind
  so neighbouring drones feel correlated, delayed wind, plus an independent local share 0.3
  (**ASSUMPTION**).
- GNSS: white noise 0.015 m per axis (**ASSUMPTION** informed by the ZED-F9P datasheet), optional
  Gauss-Markov bias (sigma, tau = 30 s) for a degraded fraction of drones (**ASSUMPTIONS**), dropout
  windows with dead-reckoning random walk 0.3 m/sqrt(s) (**ASSUMPTION**). The error model is isolated
  in `gnss_error` so it could later be parameterized from outputs of a separate Gulf-GNSS study;
  no such coupling exists today.
- Events: comms loss, low battery, motor failure at uniform times in [0.15 T, 0.85 T]. Ballistic fall
  uses horizontal velocity decay 0.5 /s (**ASSUMPTION**).
- Separation: exact minimum distance on the piecewise-linear realized paths (windowed, candidate
  pair pruning with a tracking-deviation bound), so sampling error between steps is not hidden
  beyond the linear-interpolation assumption at dt = 0.2 s.
- Statistics: Monte-Carlo with fixed seeds; proportions reported with Wilson 95% intervals. A
  zero-violation result with N trials only bounds the true rate by roughly 3/N.

Further assumptions and limits found in critique round 2 (none of these change a stored number;
they change how the numbers must be read):
- *Perfect feed-forward.* The error equation above is independent of the reference, i.e. the
  commanded acceleration is assumed to be tracked exactly and only disturbances excite the loop.
  A more aggressive plan is therefore not penalised by this model.
- *No authority limit.* There is no thrust, tilt or airframe wind limit: the loop can always
  cancel the modelled disturbance. With the 1/7 power law, a 20 ft wind W20 gives a mean wind of
  1.386 x W20 at 60 m (computed from the formula in `disturbance.py`). The X7 published sustained
  wind limit is 9 m/s (RESEARCH.md section 5), so W20 above about 6.5 m/s is already beyond the
  vehicle's published envelope at show altitude. Sweep rows for W20 = 9, 12, 15 and 20 m/s
  describe conditions in which a real aircraft would be outside its stated limits; the model's
  small tracking errors there are not predictions.
- *Sampling.* Realised paths are piecewise linear between steps of dt = 0.2 s. The chord of a
  reference with acceleration at most a lies within a dt^2 / 8 of the true path; with the default
  amax = 4 m/s^2 that is 0.02 m per drone (0.04 m on a pair distance). GNSS white noise is drawn
  once per step, so its effect on the loop depends on dt; at 0.015 m it is small next to the
  other terms.
- *GNSS error independence.* Per-drone errors are independent. For RTK-type errors, which are
  largely common to nearby aircraft, this overstates differential error (conservative for
  separation). For spoofing or jamming it is not conservative: a shared pull-off would be
  correlated in a way that this model cannot produce.
- *Turbulence.* Only the Dryden *intensities* are used. Each component is a first-order
  Gauss-Markov process with a fixed correlation time (`wind_tau`, ASSUMPTION), not a Dryden
  spectrum with altitude-dependent scale lengths.
- *Candidate pairs.* Pair pruning (`dev_bound`) is a heuristic pad, not a proof. Check run in
  review: all pairs versus pruned pairs on `sirb_text_100` for five stressed configurations
  (6 seeds each, 2,953 truly violating pairs in total): 0 missed. Kept as a regression test
  on a smaller case.
- *Trials and seeds.* Trial t of a row uses the stream `[seed, tag, t]`, so trials inside a row
  are independent draws. In `sweeps`, tag = index inside the sweep, so rows of one sweep are
  independent but the same stream is reused by row 0 of every sweep; in `spacing`, every row
  uses tag 0 (common random numbers across spacings, which sharpens comparisons between rows
  but means rows are not independent of each other). Wilson intervals are per row at 95%; they
  are not simultaneous over a table or a sweep. Rows of the scaling experiment use 6 to 20
  trials and their `p_violation` is not an estimate of anything.
- *Rejoin reference speed.* The reference for the GNSS-dropout `hold` / `rejoin` behaviours
  blends back to the moving nominal path over a fixed 6 s. On `sirb_text_100` with 30% of
  drones in dropout, the reference reached 23.4 m/s (6 seeds), against a 10 m/s horizontal
  limit. The tracking loop follows it without saturation, so the `hold` / `rejoin` rows for
  `gnss_loss` in `results/stress_contingency.json` are optimistic about how fast drones can
  rejoin. `StressConfig.rejoin_vmax` (default `None`, which reproduces the stored behaviour)
  lengthens the blend to cap the closing speed; `make stress` runs without it, so the shipped contingency rows carry this caveat.

## 6. Contingency behaviours (M6)
continue, hold, descend, corridor (clearest of 8 horizontal directions, judged against others'
nominal paths), rejoin (GNSS dropout), ballistic. They are expressed as modified references and flown
by the same tracking loop. Comparisons are in `results/stress_contingency.json`.

## 7. Pre-flight validator (M6b)
Compares what the safety case declares with what a ground station loaded (centre, heading, geofence
buffer, plan hashes, aircraft count), re-verifies the plan after any placement transform, and checks
forecast wind against a limit. Motivated by the published NTSB Orlando findings (see RESEARCH.md).
It checks software consistency of declared inputs only; it cannot know whether a declaration is true.

## 8. Vehicle and limit parameters
Defaults in `sirb/spec.py` are cited to retrieved datasheets or labeled assumptions in the show
spec (`limits` sources). The 2.0 m separation threshold and the 50 m crowd buffer follow cited
published guidance listed in RESEARCH.md; **the UAE rule set was not verified as current**, and
nothing here states that a show satisfies it.

## 9. Sim-to-real gap (read this before using any number)
1. **No real flight data was used.** Every stressed result depends on assumed disturbance models.
2. Tracking dynamics are a generic second-order loop, not an identified autopilot; attitude, motor
   saturation, latency and controller windup are not modelled.
3. Downwash and inter-vehicle aerodynamic interaction are not modelled; drones are points in the
   verifier and 0.24 m-radius spheres in contact checks.
4. Dryden turbulence is a piloted-aircraft model; real gusts near ground and structures differ.
5. GNSS errors are statistical and independent per drone except for the stated correlations; real
   interference (jamming, spoofing, multipath at a venue) is correlated across the fleet.
6. Battery variance is represented only as discrete low-battery events.
7. Radio links, show-control software, human factors and procedures are out of scope.
8. Monte-Carlo probabilities are properties of the simulated model; they are not real-world risk.
The verifier's guarantee concerns the plan as written, not the aircraft's actual motion.
