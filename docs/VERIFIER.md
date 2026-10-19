# Sirb verifier

Independent checker for piecewise-polynomial drone trajectories (`sirb/verify.py`,
`sirb/certificate.py`). It shares only the data classes in `sirb/trajectory.py` with the
planner and uses a different method: exact polynomial root finding, never time sampling.

## Method

**Pieces.** Each drone is tiled over all of time by pieces: a constant piece holding the
start position before the first segment, the segments themselves, and a constant piece at
the final position after the last segment (the drone keeps occupying space). A gap between
segments is a structural error but is bridged by holding the last position so geometry
stays defined.

**Separation.** For two drones, every pair of overlapping pieces gives a sub-interval
`[a, b]`. Time is normalised to `s in [0, 1]` and both pieces are re-expanded exactly
(Taylor shift with binomial weights). The relative position `D(s)` is a vector polynomial,
`|D|^2 = P(s)` a scalar polynomial of degree up to 2*deg. The minimum lies at an endpoint or
at a real root of `P'`. Roots come from batched companion-matrix eigenvalues (the same
algorithm as `numpy.roots`), roots with small imaginary part are kept, polished with two
Newton steps, and every candidate is evaluated directly as `|D(s)|` (never trusting the root
alone, and never via `sqrt(P)`, which would lose precision near zero). A coarse safety grid
(k/8) is added to the candidates; extra candidates can only reduce a reported minimum toward
the true value, never hide it.

**Prefilter.** Every piece gets an exact axis-aligned bounding box (per-coordinate extrema
via derivative roots; constant pieces are points). Pieces are sorted by start time; for each
piece only later-starting, time-overlapping pieces of other drones are examined, and a pair
is kept only if the box-to-box distance is at most the search radius. The box distance is a
lower bound on the true distance, so the filter only discards pairs that provably cannot be
closer than the radius. Surviving piece pairs are solved in vectorised batches and reduced to
one minimum per drone pair. `pair_minima(..., prefilter=False)` is the brute-force path used
in tests.

**Search radius.** Default `4 * min_separation_m`. Pairs whose minimum is below it are
reported exactly; if none is, `min_separation = search_radius` with
`min_separation_is_lower_bound = True`. The radius is raised to at least
`1.01*min_separation_m + 1e-3` so the threshold is always inside the searched range.

**Dynamics.** Per segment (time normalised to `[0, 1]`), the extrema of the relevant scalar
polynomials are found the same way: `|v_h|^2`, `v_z` (max and min), `|a|^2`, `|j|^2`, and
the coordinates x, y, z. Note that z, x, y extrema inside a segment are checked, not just
endpoints.

**Structure.** Positive duration; contiguity within 1e-9 s (`time_gap`, also overlap);
position continuity within 1e-4 m (`position_jump`); velocity continuity within 1e-3 m/s
(`velocity_discontinuity`, since a jump means unbounded acceleration that the polynomial
accel check cannot see); acceleration continuity within 1e-3 m/s^2 (`accel_discontinuity`,
added in review round 2: a step means unbounded jerk that the in-segment jerk check cannot
see); finite times/coefficients and well-formed shape (`nonfinite`;
the drone is excluded from geometry but never crashes the verifier). A drone with no
segments is reported as `time_gap`.

## Tolerance policy

- A dynamic or geometric limit is violated only if exceeded by more than
  `tol * (1 + |limit|)` (default `tol = 1e-6`): above for upper limits, below for lower ones.
  Separation is violated if `d < min_separation_m - tol*(1 + min_separation_m)`.
- Structural tolerances are fixed absolute values (above), independent of `tol`.
- Reported numbers (max speed, margins, minimum separation) are untoleranced exact values.
- Violation lists are capped at 1000 per kind (worst first); `violation_counts` holds the
  true counts and `violations_truncated` says if a cap applied. `ok` uses the true counts.
- `VerifyReport` has two extra defaulted fields beyond the specified API
  (`violation_counts`, `violations_truncated`); `tolerances` also records `search_radius`.

## Guarantees and non-guarantees

Guaranteed, under the declared model (polynomial paths exactly as written; drones are points
whose required centre-to-centre separation is `min_separation_m`): the minimum separation
over all time for each pair below the search radius, and the maximum/minimum of every checked
quantity, are exact up to floating-point and root-finding error (about 1e-9 in practice for
well-scaled data; see tests). Not guaranteed: real aircraft tracking, wind, GNSS error,
downwash, vehicle size and shape, failures/contingencies, any regulatory property. Numerical
caveat: monomial-basis root finding degrades for badly scaled polynomials (huge coefficients,
very long segments). The safety grid and direct evaluation bound the damage but this is not a
proof-grade interval arithmetic method.

Naming: a "certificate" here is a machine-readable verification record. It is not an airworthiness
certificate, an approval or a safety clearance, and the viewer labels it "verification record".
`verify()` skips any limit that is absent from the limits dict (only `min_separation_m` is
mandatory), so `make_certificate` reports verdict `INCOMPLETE` (and lists `unchecked_limits`)
instead of `PASS` when a limit family is missing; `FAIL` still takes precedence.

Certificates (`make_certificate` / `verify_certificate`) carry the fleet SHA-256, limits, all
results (except wall-clock runtime, omitted so output is reproducible), a verdict, method
text, and the statement "simulation-validated only; not flight-certified; no regulatory
compliance claim". There are no timestamps. `verify_certificate` checks the hash and
recomputes the verification, comparing verdict and numbers.

## Spec flaws found and fixes

- The architecture text says "bounding-sphere prefilter"; boxes are used instead (exact per
  coordinate extrema, tighter for elongated segments). The guarantee is the same.
- Velocity continuity is required in addition to the stated checks because acceleration
  limits are only meaningful within a polynomial segment.
- A search radius below the separation threshold would make violations undetectable; it is
  clamped as described above.

## How it was tested (`tests/test_verify.py`, `tests/test_certificate.py`)

- Hypothesis: random polynomial pairs of degree 1..7 against a 200k-sample brute force, with
  the sampling gap bounded by the Lipschitz constant of the distance.
- Analytic cases: head-on, exact collision, perpendicular crossing at a known time, a grazing
  pass whose minimum lies between coarse samples, cubic pair against `scipy` bounded
  minimisation, degree-7 Taylor circles with known radial gap.
- Different time spans (landed drone, parked drone passed after everything else ended, drone
  occupying its start before its first segment).
- Hypothesis: prefiltered equals brute force on random multi-segment fleets, and checked
  against dense sampling of the whole fleet.
- Corrupted trajectories, one per violation kind, plus grazing just under/over the threshold.
- A 1000-drone rest-to-rest quintic fleet (about 1 s) and the same with an injected collision.

<!-- differential:begin -->
## Differential verification

Generated by `scripts/differential_report.py` (results in `results/differential.json`) and
copied into this section by `scripts/update_verifier_doc.py`; the numbers below are not typed
by hand. Fixed seeds only; rerun with `PYTHONPATH=. python scripts/differential_report.py`.

**Second verifier (`sirb/verify2.py`).** It imports only the data classes of
`sirb.trajectory` and shares no code with `sirb/verify.py`; it uses no root finding. Pieces
(hold before, segments, hold after, gaps held) are paired when their time overlap has positive
length. For a pair of pieces on an overlap `[a, b]` the vector polynomial difference is
re-expanded by Horner synthetic division about a cell midpoint, its squared norm is formed
as a scalar polynomial in the local variable `r`, and a cell `[m - w, m + w]` gets the lower
bound `p0 + min(p1 r + p2 r^2) - sum_(k>=3) |p_k| w^k` (the quadratic part is minimised
exactly, the rest is bounded by absolute values), minus a rounding allowance of
`gamma * eps` times the magnitude of the terms (not directed rounding). The value at the
midpoint is an upper bound (a witnessed distance). Cells are bisected until their bound
reaches `best witness - tol` (`tol = 1e-9` m) or the cap; the result is a lower bound `l`, a
witness `u`, and the verdict at `d_min`: PASS only if `l >= d_min`, FAIL if a witnessed
distance (plus the allowance) is below `d_min`, otherwise UNDECIDED. A conservative box
prefilter (centred-form boxes per piece) discards piece pairs whose boxes are at least `cap`
apart. Cells still open at depth 48 would be reported as undecided leaves: there were
0 in this run.

**Agreement.** With `e` the exact minimum of `sirb.verify`, `l` and `u` the lower bound and
witness of `sirb.verify2`, per drone pair and overall:
C1 `l <= e + 1e-9`; C2 `e - l <= gap_bound + 1e-9`, where `gap_bound` is verify2's `tol`
plus its rounding allowance (the extra 1e-9 is the accuracy the exact verifier claims for
itself, because `e - l = (e - true) + (true - l)`); C3 `u >= e - 1e-9` (a witnessed distance
is a real distance and cannot beat the true minimum); C4 no PASS/FAIL contradiction outside
+/-1e-6 m of the threshold. A disagreement is any violated criterion, or a mismatch with the
analytic minimum of a constructed case by more than 1e-8 m.

**Counts.** 10856 fleets: 8000 random polynomial fleets (degree 1
to 5, 2 to 5 drones, 1 to 4 segments, some with jumps, gaps, overlaps), 2750
adversarial fleets (boundary_min 250, degenerate_collision 250, degenerate_roots 250, graze_curved 250, graze_linear 250, long_high_degree 250, parallel_double 250, split_smooth 250, stationary_threshold 250, tiny_random 250, tiny_segments 250), 100 fleets of 12 drones and
6 fleets of 100 drones; 51251 drone pairs compared
and 13100 verdicts checked.

**Largest discrepancies.** `max (e - l)` = 7.31e-08 m; `max (l - e)` =
0 m (C1 limit 1e-9); `max (e - u)` = 7.31e-08 m;
largest abs(exact - analytic construction) = 7.31e-08 m;
largest `gap_bound` = 1.29e-09 m. Verdict counts: 4990 PASS,
5558 FAIL, 2552 UNDECIDED (largest abs(e - d_min) among
undecided: 8.88e-14 m); PASS with the exact
minimum below the threshold: 0; FAIL with the
exact minimum at or above the threshold: 6
(largest margin 0 m; a margin of 0
means the threshold equals the exact minimum, where the witnessed distance can sit a few rounding
units below it).

**Disagreements (30 records).** Every record, with seed, values and
a reference computation (extended precision grid scan plus zoom), is in
`results/differential.json`.

| family | kind | records | largest difference (m) | first seeds |
|---|---|---|---|---|
| degenerate_collision | C2_gap_too_wide | 11 | 7.31e-08 | [88, 91, 94, 112, 116] |
| degenerate_collision | C3_witness_below_exact | 12 | 7.31e-08 | [88, 91, 94, 112, 116] |
| degenerate_collision | exact_vs_construction | 7 | 7.31e-08 | [94, 112, 116, 132, 150] |

**Findings.** 12 fleets (all in the families degenerate_collision) produced 30 records of kinds ['C2_gap_too_wide', 'C3_witness_below_exact', 'exact_vs_construction']; 0 records violate C1 or C4, so verify2 never reported a lower bound above the exact value and no verdict contradicted the exact value outside 1e-6 m. Cause (labelled by the extended precision reference): float conditioning in sirb.verify. In these constructed collisions or near collisions, `|D|^2` has a root of high multiplicity at the minimum, so the derivative has a root of multiplicity up to 2m-1; companion matrix eigenvalues of a multiple root carry an error of order `eps^(1/(2m-1))`, Newton polishing converges only linearly there, and the best candidate lands off the flat minimum. `sirb.verify` then reports a minimum that is too large by up to 7.31e-08 m (the reference value is the true minimum of the float64 coefficients). This is above the 'about 1e-9' stated under Guarantees and below the 1e-6 tolerance of the violation policy, so no verdict changes unless a pair sits within that band of the threshold. It is float conditioning of root finding, not a logic error: `sirb/verify.py` was not changed. A remedy, not applied here, would be a bounded local minimisation of `|D|` around the best candidate.

**Brute-force sampling** (`dt = 1e-4` s, windows of +/-0.05 s around the exact argmin and
around the coarse argmin, plus a coarse 1e-2 s global pass, plus the whole span when it is
at most 1 s): 3034 comparisons (1500 random
single segment pairs, 1534 pairs from random continuous multi segment fleets).
`max (sampled - exact)` = 2.26e-05 m; largest case where the sampled
value is below the exact minimum: 3.55e-15 m
(0 cases beyond 1e-9). Sampled minus exact
exceeded 1e-6 m in 3 cases, of which
3 had the exact argmin at a segment breakpoint (grid
points miss the kink by up to `v * dt / 2`) and
0 did not; with breakpoint times added to the
samples the largest `sampled - exact` is 1.07e-08 m.
The V shaped minima of 300 near collision passes: largest `sampled - exact`
5.51e-14 m (sampling error bound `v*dt/2`).
verify2's lower bound exceeded a sampled distance by more than 1e-9 in
0 cases.

**Timing.** 100 drone fleets: verify2 0.0236 to
0.108 s, `sirb.verify` 0.0573 to
0.617 s. Whole report: 518 s.

**Limits.** The lower bound is rigorous in exact arithmetic; in floating point it is
rigorous up to an explicit allowance (`gamma * eps` times term magnitudes), not directed
rounding, so it is not a formal proof. Both verifiers read the float64 coefficients as exact
and share the modelling choices (hold before and after, gaps held, later segment wins in an
overlap, drones as points); the differential test cannot find an error in those shared
choices, and the two programs have one author. Pieces are paired only when their overlap has
positive length, so for a trajectory with a position jump the mixed instant (end of one piece,
start of another) is not examined by either verifier. The cases are generated by the
families in `sirb/diffcases.py`; other structures could behave differently. Fleets are small
(2 to 12 drones; 100 only for timing and bound checks). Degrees above 5 appear only in the
long high degree family (up to 7). The reference used for diagnosis is a sampled
extended precision search, not a proof. Dynamics, structure and geometry checks are not
covered by this comparison.
<!-- differential:end -->
