# Predictions (written before the runs) and outcomes

Rule: each prediction below was committed before the experiment it concerns was run. Outcomes are
filled in afterwards from result files (`results/*.json`); a miss is recorded as a miss. Nothing here
is a result until the outcome column says so.

Informed priors (disclosed): earlier in this project I saw that n = 2000 planning took about 19 s and
verification about 31 s, that the exact verifier agreed with the sampled planner check, and that the
noise-free stress simulator matched the verifier. Predictions for scaling are therefore not blind.

| # | Prediction (made before the run) | Outcome | Hit / miss |
|---|---|---|---|
| P1 | Second verifier (conservative interval bounds) never reports PASS where the exact verifier finds a separation violation larger than 1e-6 m. Zero such disagreements over at least 10,000 random and adversarial fleets. | 10856 fleets, 13100 verdict checks: 0 PASS/FAIL contradictions outside 1e-6 m; 0 PASS with exact below threshold (results/differential.json) | Hit |
| P2 | Interval verifier may report extra violations (it is conservative); I expect these only for fleets whose exact minimum is within 0.05 m of the threshold. | verify2 never reports extra FAILs (FAIL needs a witnessed distance below the threshold); near threshold it returns UNDECIDED: 2552 cases, all with abs(e - d_min) <= 8.97e-14 m; FAIL with exact at or above threshold: 6 | Hit (conservatism appears as UNDECIDED, not as extra violations) |
| P3 | Largest gap between brute-force sampled minimum (dt = 1e-4 s) and exact minimum is below 1e-6 m, and the sampled minimum is never below the exact minimum by more than floating-point noise (1e-9 m). | sampled minus exact: max 2.26e-05 m on the plain grid (3 of 3 cases above 1e-6 have the argmin at a breakpoint), 1.07e-08 m with breakpoints added; sampled below exact: max 3.55e-15 m | Miss as stated (plain grid), hit with breakpoints added |
| P4 | Mutation score on `sirb/verify.py` before new tests: 70 to 85 percent. Survivors concentrate in tolerance constants, report fields and message text. On the planners before new tests: 50 to 70 percent. After new tests I aim above 90 percent for the verifier but expect some equivalent mutants to remain. | pending | |
| P5 | Sensitivity ranking of P(separation violation) on the 100-drone show, over the ranges in the experiment: GNSS bias sigma first, tracking bandwidth omega_n second, mean wind w20 third, GNSS degraded fraction fourth, dropout rate fifth; GNSS white noise and battery variance negligible. | pending | |
| P6 | Conclusions that will not survive a range test: any single "safe up to X m/s wind" statement, because the answer shifts with the tracking-loop assumption. | pending | |
| P7 | Verifier runtime grows roughly as n^1.5 to n^2 from 100 to 2000 drones; the baseline planner is dominated by the O(n^3) Hungarian step above about 1000 drones; at 4000 drones the Hungarian step takes more than 100 s and full verification more than 100 s, and memory, not CPU, becomes the limit for the stress simulator. | pending | |
| P8 | Headless software-rendered WebGL (swiftshader) will not hold 30 fps with two panes of 1000 drones; the JavaScript update cost per frame alone will stay under 16 ms. axe will report at least one issue (contrast or landmark/region). | pending | |
| P9 | For grid ground robots, naive synchronous straight-line plans fail the separation verifier on most dense swap-type tasks; the existing planners will fix some but not all. | pending | |
