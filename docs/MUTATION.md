# Mutation testing of the verifier and planners

Goal: find out whether the test suite would notice if the verifier or the planners were subtly
wrong. Mutation testing makes small automatic edits (mutants) to the source, runs the tests, and
counts a mutant as killed when a test fails. A surviving mutant is either a real gap in the tests
or a change that does not alter behaviour (an equivalent mutant). All numbers below are generated
from `results/mutation.json` by `scripts/mutation_summary.py`; none is typed by hand.

## Method

* Tool: mutmut 3.8.0 (BSD-3-Clause, dev extra, listed in `docs/DATA_SOURCES.md`).
* Targets (`[tool.mutmut]` in `pyproject.toml`): `sirb/verify.py`, `sirb/verify2.py`,
  `sirb/assign.py`, `sirb/planner/common.py`, `baseline.py`, `waves.py`, `prioritized.py`.
* Test selection: the fast part of the suite only (verifier, certificate, regression, assignment and
  planner, second-verifier differential tests, and the two `test_mutation_killers_*` files). Nine
  slow tests are deselected (brute-force comparisons, 1000-drone fleets, the CLI subprocess test);
  they are listed in `pyproject.toml`. mutmut runs, per mutant, only the selected tests whose
  coverage touches the mutated function and stops at the first failure.
* Machine: 2 CPUs shared with an unrelated experiment, so everything ran under `nice -n 15` and in
  chunks of at most about 8 minutes. mutmut keeps its progress in `mutants/` (git-ignored), so each
  chunk resumes where the last stopped.

Commands (from the repository root; `pip install -e '.[dev]'` first):

```
nice -n 15 mutmut run --max-children 8            # repeat until it reports nothing left to do
python scripts/mutation_summary.py --label before            # counts, before adding the killer tests
python scripts/mutation_summary.py --label after --diffs --tables   # counts, survivors, tables below
pytest tests/test_mutation_killers_verify.py tests/test_mutation_killers_planners.py -q
```

The "before" run used the existing suite only (the two killer files did not exist). The "after" run
re-ran every mutant that had not been killed with the killer tests present. Mutants killed before
stay killed, because the earlier tests are still in the selection.

Score formula (also stored in `results/mutation.json`):

```
score          = killed / (total - skipped)
score_detected = (killed + timeout) / (total - skipped)
```

`total` counts every generated mutant. A mutant in code the selected tests never run ("no tests") and
a timeout both count against `score`; timeouts count in `score_detected`. Nothing is excluded
because it looks equivalent: equivalent mutants stay in the denominator, so 100 percent is not
reachable.

## Results

| file | mutants | before: killed | before: score | after: killed | after: survived | after: timeout | after: score |
|---|---|---|---|---|---|---|---|
| sirb/assign.py | 265 | 188 | 70.9% | 241 | 24 | 0 | 90.9% |
| sirb/planner/baseline.py | 176 | 107 | 60.8% | 174 | 2 | 0 | 98.9% |
| sirb/planner/common.py | 186 | 111 | 59.7% | 174 | 12 | 0 | 93.5% |
| sirb/planner/prioritized.py | 682 | 371 | 54.4% | 565 | 110 | 7 | 82.8% |
| sirb/planner/waves.py | 450 | 307 | 68.2% | 400 | 50 | 0 | 88.9% |
| sirb/verify.py | 2204 | 1645 | 74.6% | 2051 | 150 | 3 | 93.1% |
| sirb/verify2.py | 988 | 668 | 67.6% | 668 | 316 | 4 | 67.6% |

`verify2.py` was measured (it is part of the selection through its differential tests) but no tests
were written for it; its before and after are the same run.

Before this work `assign.py` (43 mutants) and `common.py` (41) were also mostly "no tests"; the
killer tests execute them, which accounts for part of the planner gain.

## What the survivors of the first run showed (verifier)

Real test gaps, now closed by `tests/test_mutation_killers_verify.py`: the exact tolerance boundary
`limit + tol*(1+|limit|)` of every limit family (including non-zero lower limits, where a wrong
`1+|limit|` factor only shows when the limit is not 0); every field of every violation record (kind,
drones, time, value, limit, text); the numeric report fields (speeds, accelerations, margins, minimum
separation and its time and pair); the structural-check boundaries; exclusion of a drone with a
malformed segment from geometry and dynamics, for each of the four malformed-segment kinds; the hold
before the first segment, across gaps and after the last segment; the search radius, the prefilter and
the closest-pair selection; violation ordering and the cap; the polynomial kernels (`_binom`, `_shift`,
`_extremes`, `_critical_points`) against hand-derived answers. No bug in the verifier was found: every
new test passes on the unmutated code and every expected number was derived by hand, not read back
from the verifier.

## Survivors after the new tests

Classes (assigned by `classify` in `scripts/mutation_summary.py` from the text of the change, plus
a table of hand-checked rules for `verify.py`; the reasons are arguments from reading the code, not
proofs):

* equivalent: the changed code computes the same thing (default argument, dtype, case-insensitive
  string, masked slot that is replaced by a valid candidate, norm of a negated vector, ...).
* no-effect: changes cost or warnings only (warning filters, a superset box in the prefilter).
* tolerance: strict versus non-strict comparison, which differs only when two floats are exactly equal.
* constant: a numeric constant changed and no selected test distinguishes it. This is a weaker label
  than equivalent; for the planners it means the tests accept plans produced with the other constant.
* message: report or error text.
* deep-numerics: root-finding and refinement details in `verify.py` that the 9-point grid, the Newton
  polish and the direct evaluation of every candidate make insensitive on the polynomials I tried; a
  near-tangent or ill-conditioned known-answer suite would be needed to separate them.
* heuristic: `prioritized.py` search parameters; any value still produces a plan that the verifier
  judges, so only plan quality or runtime changes.
* open: not shown to be equivalent and no test distinguishes it. These are real, untriaged gaps.

| file | equivalent | no-effect | tolerance | constant | message | deep-numerics | heuristic | open |
|---|---|---|---|---|---|---|---|---|
| sirb/assign.py | 6 | 4 | 2 | 5 | 0 | 0 | 0 | 7 |
| sirb/planner/baseline.py | 0 | 0 | 1 | 0 | 0 | 0 | 0 | 1 |
| sirb/planner/common.py | 0 | 0 | 3 | 3 | 0 | 0 | 0 | 6 |
| sirb/planner/prioritized.py | 3 | 0 | 12 | 36 | 1 | 0 | 58 | 0 |
| sirb/planner/waves.py | 3 | 8 | 8 | 15 | 0 | 0 | 0 | 16 |
| sirb/verify.py | 85 | 6 | 17 | 21 | 1 | 20 | 0 | 0 |
| sirb/verify2.py | 26 | 4 | 17 | 72 | 2 | 0 | 0 | 195 |

Grouped list (one row per function and class; the full per-mutant list with the textual change is
in `results/mutation.json`):

Survivors after
| file | function | class | n | mutants | reason |
|---|---|---|---|---|---|
| assign.py | assign_greedy | equivalent | 1 | 9 | case-insensitive argument (scipy/numpy/einsum/float parsing) |
| assign.py | assign_greedy | open | 5 | 7, 23, 25, 41, 42 | planner internals not pinned by a known-answer test |
| assign.py | assign_hungarian_len | equivalent | 1 | 9 | case-insensitive argument (scipy/numpy/einsum/float parsing) |
| assign.py | assign_hungarian_len | open | 1 | 7 | planner internals not pinned by a known-answer test |
| assign.py | assign_hungarian_sq | equivalent | 1 | 9 | case-insensitive argument (scipy/numpy/einsum/float parsing) |
| assign.py | assign_pinned | equivalent | 1 | 1 | docstring or comment removed |
| assign.py | assign_pinned | open | 1 | 25 | planner internals not pinned by a known-answer test |
| assign.py | assign_random | equivalent | 1 | 1 | docstring or comment removed |
| assign.py | evaluate | equivalent | 1 | 1 | docstring or comment removed |
| assign.py | linear_conflicts | constant | 5 | 1, 56, 72, 73, 76 | numeric constant changed; no input in the tests distinguishes it |
| assign.py | linear_conflicts | no-effect | 4 | 40, 41, 42, 43 | floating-point warning filter; changes warnings, not values |
| assign.py | linear_conflicts | tolerance | 2 | 55, 71 | strict vs non-strict comparison; differs only when the two sides are exactly equal |
| planner/baseline.py | plan_baseline | open | 1 | 72 | planner internals not pinned by a known-answer test |
| planner/baseline.py | plan_baseline | tolerance | 1 | 67 | strict vs non-strict comparison; differs only when the two sides are exactly equal |
| planner/common.py | leg_segment | open | 4 | 4, 6, 8, 10 | planner internals not pinned by a known-answer test |
| planner/common.py | min_leg_time | tolerance | 1 | 22 | strict vs non-strict comparison; differs only when the two sides are exactly equal |
| planner/common.py | path_duration | constant | 1 | 13 | numeric constant changed; no input in the tests distinguishes it |
| planner/common.py | path_segments | constant | 2 | 8, 16 | numeric constant changed; no input in the tests distinguishes it |
| planner/common.py | path_segments | open | 2 | 21, 22 | planner internals not pinned by a known-answer test |
| planner/common.py | stitch | tolerance | 2 | 7, 21 | strict vs non-strict comparison; differs only when the two sides are exactly equal |
| planner/prioritized.py | _plan_window | constant | 21 | 12, 15, 37, 39, 59, 70, 105, 106, 108, 109, 117, 122, ... | numeric constant changed; no input in the tests distinguishes it |
| planner/prioritized.py | _plan_window | heuristic | 41 | 13, 14, 33, 35, 36, 38, 64, 67, 69, 92, 93, 95, ... | search-heuristic or schedule parameter; any variant still yields a plan the verifier judges |
| planner/prioritized.py | _plan_window | message | 1 | 42 | message or report text |
| planner/prioritized.py | _plan_window | tolerance | 7 | 40, 58, 115, 151, 175, 180, 242 | strict vs non-strict comparison; differs only when the two sides are exactly equal |
| planner/prioritized.py | _sample_path | tolerance | 3 | 8, 20, 26 | strict vs non-strict comparison; differs only when the two sides are exactly equal |
| planner/prioritized.py | plan_prioritized | constant | 15 | 4, 5, 6, 7, 8, 9, 34, 85, 116, 122, 125, 129, ... | numeric constant changed; no input in the tests distinguishes it |
| planner/prioritized.py | plan_prioritized | equivalent | 1 | 72 | case-insensitive argument (scipy/numpy/einsum/float parsing) |
| planner/prioritized.py | plan_prioritized | equivalent | 1 | 69 | sort kind only matters for exact ties (argument removed) |
| planner/prioritized.py | plan_prioritized | equivalent | 1 | 67 | sort kind only matters for exact ties (argument set to None = default) |
| planner/prioritized.py | plan_prioritized | heuristic | 17 | 28, 30, 82, 83, 84, 92, 114, 115, 133, 152, 153, 175, ... | search-heuristic or schedule parameter; any variant still yields a plan the verifier judges |
| planner/prioritized.py | plan_prioritized | tolerance | 2 | 113, 130 | strict vs non-strict comparison; differs only when the two sides are exactly equal |
| planner/waves.py | _point_seg_dist | constant | 2 | 38, 39 | numeric constant changed; no input in the tests distinguishes it |
| planner/waves.py | _point_seg_dist | no-effect | 4 | 9, 10, 11, 12 | floating-point warning filter; changes warnings, not values |
| planner/waves.py | _point_seg_dist | tolerance | 2 | 24, 36 | strict vs non-strict comparison; differs only when the two sides are exactly equal |
| planner/waves.py | _sync_min_dist | constant | 3 | 54, 55, 58 | numeric constant changed; no input in the tests distinguishes it |
| planner/waves.py | _sync_min_dist | no-effect | 4 | 22, 23, 24, 25 | floating-point warning filter; changes warnings, not values |
| planner/waves.py | _sync_min_dist | tolerance | 2 | 37, 53 | strict vs non-strict comparison; differs only when the two sides are exactly equal |
| planner/waves.py | plan_waves | constant | 4 | 32, 83, 114, 155 | numeric constant changed; no input in the tests distinguishes it |
| planner/waves.py | plan_waves | open | 5 | 26, 28, 105, 148, 150 | planner internals not pinned by a known-answer test |
| planner/waves.py | plan_waves | tolerance | 2 | 82, 101 | strict vs non-strict comparison; differs only when the two sides are exactly equal |
| planner/waves.py | schedule_waves | constant | 6 | 84, 89, 90, 91, 98, 102 | numeric constant changed; no input in the tests distinguishes it |
| planner/waves.py | schedule_waves | equivalent | 1 | 50 | case-insensitive argument (scipy/numpy/einsum/float parsing) |
| planner/waves.py | schedule_waves | equivalent | 1 | 47 | sort kind only matters for exact ties (argument removed) |
| planner/waves.py | schedule_waves | equivalent | 1 | 45 | sort kind only matters for exact ties (argument set to None = default) |
| planner/waves.py | schedule_waves | open | 11 | 25, 28, 30, 33, 35, 38, 53, 86, 87, 88, 99 | planner internals not pinned by a known-answer test |
| planner/waves.py | schedule_waves | tolerance | 2 | 15, 23 | strict vs non-strict comparison; differs only when the two sides are exactly equal |
| verify.py | _build_pieces | equivalent | 1 | 120 | chunk size only changes memory use |
| verify.py | _build_pieces | equivalent | 3 | 15, 31, 55 | constant (hold) piece: the time origin does not enter a degree-0 polynomial |
| verify.py | _build_pieces | equivalent | 4 | 63, 68, 73, 78 | default dtype of the values involved is the same (argument removed) |
| verify.py | _build_pieces | equivalent | 4 | 61, 66, 71, 76 | default dtype of the values involved is the same (argument set to None = default) |
| verify.py | _build_pieces | equivalent | 1 | 85 | only reached with no pieces, which pair_minima handles before using K |
| verify.py | _build_pieces | equivalent | 1 | 21 | overwritten by the first segment of the loop before any use |
| verify.py | _build_pieces | no-effect | 1 | 132 | box over a wider span is a superset; prefilter is only less selective |
| verify.py | _build_pieces | tolerance | 1 | 26 | strict vs non-strict comparison; differs only when the two sides are exactly equal |
| verify.py | _candidate_pairs | equivalent | 2 | 7, 67 | case-insensitive argument (scipy/numpy/einsum/float parsing) |
| verify.py | _candidate_pairs | equivalent | 1 | 78 | default dtype of the values involved is the same (argument removed) |
| verify.py | _candidate_pairs | equivalent | 1 | 75 | default dtype of the values involved is the same (argument set to None = default) |
| verify.py | _candidate_pairs | equivalent | 5 | 85, 87, 90, 92, 93 | empty result; dtype of an empty index array is irrelevant |
| verify.py | _candidate_pairs | equivalent | 1 | 34 | ends[i] >= i + 1 always (hi > lo), so the branch is the same |
| verify.py | _candidate_pairs | equivalent | 1 | 21 | prefilter/cap switch only widens an optional pre-selection |
| verify.py | _candidate_pairs | equivalent | 1 | 16 | searchsorted default side is 'left' (argument removed) |
| verify.py | _candidate_pairs | equivalent | 1 | 5 | sort kind only matters for exact ties (argument removed) |
| verify.py | _candidate_pairs | equivalent | 1 | 3 | sort kind only matters for exact ties (argument set to None = default) |
| verify.py | _candidate_pairs | no-effect | 1 | 58 | same-drone pairs are discarded later by the pair key |
| verify.py | _candidate_pairs | tolerance | 1 | 23 | prefilter radius slack of 1e-9 m |
| verify.py | _candidate_pairs | tolerance | 2 | 33, 68 | strict vs non-strict comparison; differs only when the two sides are exactly equal |
| verify.py | _critical_points | constant | 8 | 65, 67, 79, 104, 143, 157, 162, 184 | numeric constant changed; no input in the tests distinguishes it |
| verify.py | _critical_points | deep-numerics | 14 | 27, 42, 45, 48, 53, 61, 63, 64, 71, 74, 92, 130, ... | root-finding details (degree selection, Newton polish, masks); the 9-point grid and direct evaluation make results insensitive on the test polynomials |
| verify.py | _critical_points | equivalent | 7 | 107, 108, 109, 110, 111, 112, 113 | defensive LinAlgError fallback; eigvals does not raise for the finite input built here |
| verify.py | _critical_points | equivalent | 1 | 156 | nan_to_num default is 0.0 (argument removed) |
| verify.py | _critical_points | no-effect | 1 | 29 | floating-point warning filter; changes warnings, not values |
| verify.py | _critical_points | tolerance | 8 | 44, 82, 119, 120, 123, 142, 181, 201 | strict vs non-strict comparison; differs only when the two sides are exactly equal |
| verify.py | _dynamics | constant | 1 | 415 | numeric constant changed; no input in the tests distinguishes it |
| verify.py | _eval_vec | constant | 1 | 8 | numeric constant changed; no input in the tests distinguishes it |
| verify.py | _eval_vec | equivalent | 1 | 6 | default dtype of the values involved is the same (argument removed) |
| verify.py | _eval_vec | equivalent | 1 | 4 | default dtype of the values involved is the same (argument set to None = default) |
| verify.py | _extremes | constant | 2 | 13, 15 | numeric constant changed; no input in the tests distinguishes it |
| verify.py | _extremes | equivalent | 4 | 21, 27, 34, 41 | an invalid slot evaluated at s=0 (or 1) is a genuine endpoint candidate |
| verify.py | _extremes | equivalent | 1 | 4 | prefilter/cap switch only widens an optional pre-selection |
| verify.py | _extremes | tolerance | 1 | 12 | strict vs non-strict comparison; differs only when the two sides are exactly equal |
| verify.py | _horner_rows | equivalent | 1 | 19 | negates the whole polynomial; only its norm is used |
| verify.py | _refine_minimum | constant | 2 | 19, 28 | numeric constant changed; no input in the tests distinguishes it |
| verify.py | _refine_minimum | deep-numerics | 6 | 3, 13, 22, 85, 100, 118 | golden-section refinement only lowers a minimum towards the true one; needs a near-tangent known-answer case |
| verify.py | _refine_minimum | tolerance | 2 | 5, 55 | strict vs non-strict comparison; differs only when the two sides are exactly equal |
| verify.py | _shift | constant | 2 | 19, 31 | numeric constant changed; no input in the tests distinguishes it |
| verify.py | _shift | equivalent | 1 | 44 | case-insensitive argument (scipy/numpy/einsum/float parsing) |
| verify.py | _shift | equivalent | 2 | 12, 15 | dtype/fill of the integer exponent table; masked entries are zeroed afterwards |
| verify.py | _shift | no-effect | 1 | 20 | floating-point warning filter; changes warnings, not values |
| verify.py | _solve_pairs | constant | 1 | 131 | numeric constant changed; no input in the tests distinguishes it |
| verify.py | _solve_pairs | equivalent | 5 | 125, 142, 148, 150, 159 | invalid-root masking; invalid slots are replaced by candidates already present |
| verify.py | _solve_pairs | equivalent | 10 | 4, 37, 42, 45, 57, 64, 66, 69, 71, 72 | point evaluation of unbounded/degenerate overlaps; dominated by the adjacent finite pair for continuous trajectories (argued, not proven) |
| verify.py | _solve_pairs | equivalent | 4 | 105, 106, 107, 110 | slice(None,3), slice(0,None), slice(3) select the same three columns |
| verify.py | _solve_pairs | no-effect | 1 | 29 | floating-point warning filter; changes warnings, not values |
| verify.py | _solve_pairs | tolerance | 1 | 38 | strict vs non-strict comparison; differs only when the two sides are exactly equal |
| verify.py | _sq_poly | constant | 1 | 6 | numeric constant changed; no input in the tests distinguishes it |
| verify.py | _sq_poly | equivalent | 1 | 4 | one extra all-zero coefficient column |
| verify.py | _structure | constant | 1 | 148 | numeric constant changed; no input in the tests distinguishes it |
| verify.py | _structure | equivalent | 3 | 60, 92, 125 | case-insensitive argument (scipy/numpy/einsum/float parsing) |
| verify.py | _structure | equivalent | 1 | 41 | default dtype of the values involved is the same (argument removed) |
| verify.py | _structure | equivalent | 1 | 39 | default dtype of the values involved is the same (argument set to None = default) |
| verify.py | _structure | equivalent | 3 | 29, 30, 337 | drone is excluded anyway because it has no usable segment |
| verify.py | _structure | tolerance | 1 | 226 | strict vs non-strict comparison; differs only when the two sides are exactly equal |
| verify.py | pair_minima | constant | 1 | 16 | numeric constant changed; no input in the tests distinguishes it |
| verify.py | pair_minima | equivalent | 4 | 48, 49, 50, 52 | all variants of the pair key remain injective for pieces of drones <= max index |
| verify.py | pair_minima | no-effect | 1 | 36 | refining every pair instead of only those below the radius changes cost, not the reported minima below the radius |
| verify.py | verify | constant | 1 | 152 | numeric constant changed; no input in the tests distinguishes it |
| verify.py | verify | equivalent | 1 | 133 | case-insensitive argument (scipy/numpy/einsum/float parsing) |
| verify.py | verify | equivalent | 4 | 40, 46, 48, 51 | pair_minima rebuilds the same pieces from the fleet when none are passed |
| verify.py | verify | equivalent | 1 | 147 | prefilter/cap switch only widens an optional pre-selection |
| verify.py | verify | message | 1 | 132 | message or report text |
| verify2.py | _abs_eval | constant | 3 | 13, 15, 17 | numeric constant changed; no input in the tests distinguishes it |
| verify2.py | _abs_eval | open | 1 | 19 | not shown to be equivalent; no test distinguishes it |
| verify2.py | _candidate_piece_pairs | constant | 3 | 52, 83, 88 | numeric constant changed; no input in the tests distinguishes it |
| verify2.py | _candidate_piece_pairs | equivalent | 2 | 7, 62 | case-insensitive argument (scipy/numpy/einsum/float parsing) |
| verify2.py | _candidate_piece_pairs | equivalent | 1 | 73 | default dtype of the values involved is the same (argument removed) |
| verify2.py | _candidate_piece_pairs | equivalent | 1 | 70 | default dtype of the values involved is the same (argument set to None = default) |
| verify2.py | _candidate_piece_pairs | equivalent | 2 | 20, 21 | prefilter/cap switch only widens an optional pre-selection |
| verify2.py | _candidate_piece_pairs | equivalent | 1 | 16 | searchsorted default side is 'left' (argument removed) |
| verify2.py | _candidate_piece_pairs | equivalent | 1 | 5 | sort kind only matters for exact ties (argument removed) |
| verify2.py | _candidate_piece_pairs | equivalent | 1 | 3 | sort kind only matters for exact ties (argument set to None = default) |
| verify2.py | _candidate_piece_pairs | open | 16 | 28, 33, 50, 51, 53, 54, 75, 76, 79, 80, 81, 82, ... | not shown to be equivalent; no test distinguishes it |
| verify2.py | _candidate_piece_pairs | tolerance | 3 | 27, 40, 63 | strict vs non-strict comparison; differs only when the two sides are exactly equal |
| verify2.py | _cell_bounds | constant | 20 | 20, 31, 32, 42, 43, 44, 46, 53, 55, 62, 94, 104, ... | numeric constant changed; no input in the tests distinguishes it |
| verify2.py | _cell_bounds | equivalent | 1 | 15 | case-insensitive argument (scipy/numpy/einsum/float parsing) |
| verify2.py | _cell_bounds | equivalent | 6 | 34, 35, 51, 58, 132, 133 | prefilter/cap switch only widens an optional pre-selection |
| verify2.py | _cell_bounds | no-effect | 4 | 78, 79, 80, 81 | floating-point warning filter; changes warnings, not values |
| verify2.py | _cell_bounds | open | 38 | 18, 23, 27, 29, 30, 36, 37, 38, 39, 40, 41, 69, ... | not shown to be equivalent; no test distinguishes it |
| verify2.py | _cell_bounds | tolerance | 4 | 45, 93, 114, 142 | strict vs non-strict comparison; differs only when the two sides are exactly equal |
| verify2.py | _hold_pos | open | 2 | 18, 19 | not shown to be equivalent; no test distinguishes it |
| verify2.py | _piece_boxes | constant | 7 | 14, 16, 25, 27, 40, 48, 69 | numeric constant changed; no input in the tests distinguishes it |
| verify2.py | _piece_boxes | open | 23 | 2, 7, 13, 15, 18, 24, 26, 33, 39, 46, 47, 50, ... | not shown to be equivalent; no test distinguishes it |
| verify2.py | _pieces | constant | 8 | 22, 43, 45, 60, 83, 91, 92, 98 | numeric constant changed; no input in the tests distinguishes it |
| verify2.py | _pieces | equivalent | 2 | 11, 102 | default dtype of the values involved is the same (argument removed) |
| verify2.py | _pieces | equivalent | 2 | 9, 100 | default dtype of the values involved is the same (argument set to None = default) |
| verify2.py | _pieces | open | 19 | 12, 13, 14, 25, 27, 28, 33, 36, 41, 54, 56, 57, ... | not shown to be equivalent; no test distinguishes it |
| verify2.py | _pieces | tolerance | 3 | 16, 53, 55 | strict vs non-strict comparison; differs only when the two sides are exactly equal |
| verify2.py | _taylor_shift | constant | 4 | 11, 19, 21, 23 | numeric constant changed; no input in the tests distinguishes it |
| verify2.py | _taylor_shift | equivalent | 1 | 5 | default dtype of the values involved is the same (argument removed) |
| verify2.py | _taylor_shift | equivalent | 1 | 3 | default dtype of the values involved is the same (argument set to None = default) |
| verify2.py | _taylor_shift | open | 8 | 10, 15, 16, 17, 20, 22, 25, 27 | not shown to be equivalent; no test distinguishes it |
| verify2.py | verify2 | constant | 27 | 15, 16, 21, 24, 27, 28, 63, 64, 65, 66, 67, 68, ... | numeric constant changed; no input in the tests distinguishes it |
| verify2.py | verify2 | equivalent | 2 | 59, 62 | case-insensitive argument (scipy/numpy/einsum/float parsing) |
| verify2.py | verify2 | equivalent | 2 | 56, 57 | prefilter/cap switch only widens an optional pre-selection |
| verify2.py | verify2 | message | 2 | 58, 61 | message or report text |
| verify2.py | verify2 | open | 88 | 2, 13, 14, 23, 25, 26, 29, 30, 31, 32, 33, 34, ... | not shown to be equivalent; no test distinguishes it |
| verify2.py | verify2 | tolerance | 7 | 60, 328, 335, 337, 340, 426, 430 | strict vs non-strict comparison; differs only when the two sides are exactly equal |


## Honest limits

* A high score is not a proof. Mutants are small local edits; they do not model a missing feature or a
  wrong algorithm.
* The equivalent labels are arguments from reading the code. A mutant labelled equivalent that is
  not would be a missed gap. The riskiest are the `_solve_pairs` point-evaluation rows (argued
  from continuity of the trajectories) and the `deep-numerics` rows.
* `tolerance` and `constant` survivors are only harmless if the exact value does not matter. For the
  verifier tolerances it does matter and the boundary tests pin them; the survivors listed under
  those labels are other constants (search slack, grid size, polish iterations).
* `verify2.py` has 316 survivors (67.6 percent killed) and was not worked on; most are
  labelled open because I did not read them one by one.
* The planner survivors (`waves.py`, `prioritized.py`, `assign.py`, `common.py`) were triaged
  only by the rules above. `prioritized.py` is a heuristic search whose output is checked by the verifier,
  so many of its mutants change quality, not safety; this is why the tests do not pin them.
* Three verifier mutants and several `prioritized.py` mutants time out or were not decided in the
  run (`other_undetected` in the json); they are not counted as killed in `score`.
* The selection leaves out nine slow tests. A survivor might be killed by one of them; I did not
  re-run survivors against the full suite.
* mutmut forks workers after a statistics run, so module-level caches such as `_binom`'s cache
  are warm in every worker. The `_binom` test empties the cache explicitly for that reason.
