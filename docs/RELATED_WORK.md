# Related work and research gap

Written 2026-10-28. This is a targeted reading of the closest sources I could retrieve, not an exhaustive survey. Facts below come from the retrieved pages (read through a summariser, so quotations are the summariser's extracts); anything not stated by the retrieved page is marked as not retrieved. Venues and years are given only where a source stated them.

## What already exists

| Source | What it establishes | Consequence for Sirb |
|---|---|---|
| Gravell and Summers, "Centralized Collision-free Polynomial Trajectories and Goal Assignment for Aerial Swarms" ([arXiv 2101.08829](https://arxiv.org/pdf/2101.08829)) | Exact pairwise separation of polynomial trajectories by building the squared distance polynomial and minimising it with root finding; segment pairs are screened first by a cheap line segment test. Experiments with 20 quadrotors in a 2 m by 3 m room and simulations up to 1000 agents. The guarantee relies on inflated collision volumes (30 cm cylinders for a 14 cm vehicle) and assumes disturbances stay inside them. Venue and year: not retrieved. | The exact check is not new. Sirb must not claim it as an invention. |
| Bucki and Mueller, "Rapid Collision Detection for Multicopter Trajectories" ([arXiv 1904.04223](https://arxiv.org/pdf/1904.04223)) | Continuous time collision detection of fifth order polynomial trajectories against convex obstacles by recursive bisection with closed form critical points; outcomes feasible, infeasible or indeterminable; the method extends to two quadcopters through their relative position. Microsecond scale per trajectory in their Monte Carlo benchmark. | Closest relative of the second (interval) verifier, including the undecided outcome. Speed per pair is a reasonable comparison to report. |
| Preiss, Hönig, Ayanian and Sukhatme, "Downwash-Aware Trajectory Planning for Large Quadrotor Teams" ([arXiv 1704.04852](https://arxiv.org/pdf/1704.04852)) | Ellipsoidal (taller than wide) vehicle model for downwash, a discrete grid planner followed by safe corridors that guarantee separation by construction; simulations up to 200 robots, 32 flown. | Defines the anisotropic separation model Sirb plans to add as an option; their guarantee is by construction, not by checking an arbitrary plan. |
| Augugliaro, Schoellig and D'Andrea, IROS 2012, sequential convex programming for a quadrocopter fleet | Separation enforced only at discrete time steps; the paper itself notes that interpolated trajectories may violate it between steps and reduces the step size and re-solves when a check of the interpolation finds violations. 4 to 12 vehicles in simulation. | A published instance of the sampled versus continuous gap that Sirb measures. Used as a planned comparator (not yet implemented here). |
| Skybrush Studio safety check ([documentation](https://docs.skybrush.io/public/skybrush-studio-for-blender/latest/panels/safety_and_export/safety_check.html)) | Proximity is checked on animation frames, measured between drone centres; by default one violating pair per frame is flagged. The page does not state a frame rate or whether frames are interpolated. | The practical show workflow is frame sampled. This is the realistic baseline for the sampled checker. |
| van den Berg, Guy, Lin and Manocha, "Reciprocal n-body collision avoidance" (2011) | Reactive velocity based collision avoidance (ORCA). | Implemented here as a comparator and validated against its reference implementation. |
| Corso, Moss, Koren, Lee and Kochenderfer, "A Survey of Algorithms for Black-Box Safety Validation of Cyber-Physical Systems" ([arXiv 2005.02979](https://arxiv.org/pdf/2005.02979)) | Separates falsification (finding counterexamples by optimisation, path planning or reinforcement learning) from estimating failure probability (needs importance sampling for rare events). | Sirb's stress tester estimates probabilities by plain Monte Carlo. Worst case search and importance sampling are separate, missing capabilities. |

## What Sirb does and does not contribute

Established elsewhere, so not claimed: exact continuous time separation of polynomial trajectories; root finding on the squared distance; undecided outcomes from interval methods; the sampled versus continuous gap in principle.

Candidate contributions, stated as hypotheses to be tested, not as results (see `docs/PREDICTIONS.md`):

1. A second, independently written verifier (interval branch and bound) checked against the first by differential testing on 10,856 fleets, a brute force check and mutation testing. I did not find the verifier of a published method audited this way in the sources above; absence from a short reading is not proof of absence.
2. A measured false pass rate of frame sampled checks against the exact verifier, by frame step, with the benchmark scenarios and code released.
3. A risk targeted margin: the nominal separation needed for a stated Wilson upper bound under a declared disturbance model, with its dependence on an unsourced assumption (GNSS error correlation) measured, not hidden.
4. Cross platform reproducibility of plans and verdicts (identical fleet hashes on Windows, Linux and the CI runner).

These are engineering and evaluation contributions. They do not amount to a new algorithm, and the project should not be presented as one.

## The gap statement

In the sources retrieved, the exact polynomial check appears as a component inside planners and obstacle avoidance routines. I found none that reports (a) an independent cross validation of the checker itself, (b) the quantified error of the sampled check practitioners use for shows, and (c) a margin synthesis under a stated disturbance model, together, on show scale fleets. Sirb's question is therefore narrow: how often does the common frame sampled check pass a plan that fails continuously, how does the answer depend on frame step and compute budget, and how much extra separation does a stated disturbance model require.

## Hypotheses that can fail

- H1. At matched wall clock budget, a frame sampled checker misses a small but non zero share of violations on near boundary cases, so the exact verifier gives a guarantee the sampler cannot. This fails if the sampler at matched budget catches every case on every test set, in which case the exact verifier's advantage is the guarantee only, not a measured detection gain.
- H2. The exact verifier is no more than one order of magnitude slower per fleet than a sampler that reaches a 99 percent detection rate on the adversarial set. This fails if the exact verifier is much slower than that.
- H3. The stressed verdict at frame step 0.2 s agrees with a finer step (0.05 s) within the Monte Carlo interval. This fails if between sample violations change the verdict.

## Not covered

Fault tolerant replanning methods, PX4 or ArduPilot based show systems beyond the Skybrush page, chance constrained planning, and the literature on formal verification of hybrid systems were not read. A reviewer from those areas may know closer work; the next step is to ask one.
