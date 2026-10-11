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
