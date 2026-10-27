"""Tests added after triaging the survivors of the mutation run on ``sirb/verify2.py`` (phase 6).

Every expected number is derived by hand (the derivation is in the comment above each test) or by an
independent computation (exact integer or ``Fraction`` arithmetic, dense sampling, ``math``), never
read back from ``sirb.verify2``. Groups: the polynomial kernels, the piece construction, the piece
boxes, the candidate piece pairs, the cell bound (known polynomials with known minima, the rounding
allowance, the tail), and the driver (verdict, bounds, counts, fields, per-pair mode, caps, the
depth and size limits with the module constants lowered so that the counts can be derived by hand).
"""

from __future__ import annotations

import math
import time
import warnings
from fractions import Fraction

import numpy as np
import pytest

import sirb.verify2 as V2
from sirb.trajectory import Fleet, Segment, Trajectory
from sirb.verify2 import verify2

EPS = float(np.finfo(float).eps)
INF = math.inf


def seg(t0, t1, *rows):
    return Segment(float(t0), float(t1), np.array(rows, float))


def fleet(*drones):
    return Fleet([Trajectory(list(d)) for d in drones])


def parked(p, t0=0.0, t1=1.0):
    return [seg(t0, t1, list(p))]


# ---------------------------------------------------------------------------------------------
# 1. Polynomial kernels
# ---------------------------------------------------------------------------------------------
def test_taylor_shift_hand_example():
    # x(tau) = 1 + 2 tau + 3 tau^2, shifted by 2: 1 + 2 (2 + r) + 3 (4 + 4 r + r^2) = 17 + 14 r + 3 r^2
    # z(tau) = 4 - tau, shifted by 2: 2 - r
    c = np.zeros((1, 3, 3))
    c[0, :, 0] = [1, 2, 3]
    c[0, :, 2] = [4, -1, 0]
    q = V2._taylor_shift(c, np.array([2.0]))
    assert q[0, :, 0].tolist() == [17.0, 14.0, 3.0]
    assert q[0, :, 1].tolist() == [0.0, 0.0, 0.0]
    assert q[0, :, 2].tolist() == [2.0, -1.0, 0.0]


@pytest.mark.parametrize("K", [1, 2, 3, 4, 5])
def test_taylor_shift_matches_binomial_expansion_in_exact_integers(K):
    # c(d + r) = sum_k r^k sum_{j >= k} C(j, k) d^(j-k) c_j; small integers are exact in floats.
    rng = np.random.default_rng(100 + K)
    c = rng.integers(-5, 6, size=(5, K, 3))                 # integer dtype on purpose
    d = np.array([-3.0, -1.0, 0.0, 2.0, 4.0])
    c_before = c.copy()
    q = V2._taylor_shift(c, d)
    assert q.shape == (5, K, 3)
    for n in range(5):
        for a in range(3):
            for k in range(K):
                exact = sum(Fraction(math.comb(j, k)) * Fraction(d[n]) ** (j - k) * int(c[n, j, a])
                            for j in range(k, K))
                assert Fraction(float(q[n, k, a])) == exact, (n, a, k)
    assert np.array_equal(c, c_before)                      # the input is not modified


def test_abs_eval_hand_sums():
    # row 0 (x = 2): |1| + |-4| 2 + |7| 4 = 37; (y) 2 + 10 + 32 = 44; (z) 3 + 12 + 36 = 51
    # row 1 (x = 3): 0 + 1 * 3 + 0 = 3; (y) 3; (z) 3 + 2 * 9 = 21
    c = np.zeros((2, 3, 3))
    c[0] = [[1, -2, 3], [-4, 5, -6], [7, -8, 9]]
    c[1] = [[0, 0, 0], [-1, -1, -1], [0, 0, -2]]
    out = V2._abs_eval(c, np.array([2.0, 3.0]))
    assert out.tolist() == [[37.0, 44.0, 51.0], [3.0, 3.0, 21.0]]
    # x = 0 keeps only the constant term
    assert V2._abs_eval(c, np.array([0.0, 0.0])).tolist() == [[1.0, 2.0, 3.0], [0.0, 0.0, 0.0]]


def test_hold_pos_hand_values():
    # c0 + 3 c1 + 9 c2 = (1 + 0 + 18, 2 + 3 + 0, 3 - 3 + 9)
    c = np.array([[1.0, 2.0, 3.0], [0.0, 1.0, -1.0], [2.0, 0.0, 1.0]])
    assert V2._hold_pos(c, 3.0).tolist() == [19.0, 5.0, 9.0]
    assert V2._hold_pos(c, 0.0).tolist() == [1.0, 2.0, 3.0]
    assert V2._hold_pos(c[:1], 7.0).tolist() == [1.0, 2.0, 3.0]


# ---------------------------------------------------------------------------------------------
# 2. Piece construction
# ---------------------------------------------------------------------------------------------
def test_pieces_holds_gaps_and_time_origins():
    # drone 0: x = 2 tau, z = tau^2 on [1, 3] (tau = t - 1), end value (4, 0, 4); a gap [3, 5];
    #          then a parked segment on [5, 6] at (10, 0, 0).  drone 1: parked on [0, 1].
    f = fleet([seg(1, 3, [0, 0, 0], [2, 0, 0], [0, 0, 1]), seg(5, 6, [10, 0, 0])],
              [seg(0, 1, [7, 7, 7])])
    drone, lo, hi, t0r, cf = V2._pieces(f)
    assert drone.tolist() == [0, 0, 0, 0, 0, 1, 1, 1]
    assert lo.tolist() == [-INF, 1.0, 3.0, 5.0, 6.0, -INF, 0.0, 1.0]
    assert hi.tolist() == [1.0, 3.0, 5.0, 6.0, INF, 0.0, 1.0, INF]
    assert t0r.tolist() == [0.0, 1.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0]
    assert cf.shape == (8, 3, 3)
    expect = np.zeros((8, 3, 3))
    expect[0, 0] = [0, 0, 0]
    expect[1] = [[0, 0, 0], [2, 0, 0], [0, 0, 1]]
    expect[2, 0] = [4, 0, 4]                       # held end value of the first segment
    expect[3, 0] = [10, 0, 0]
    expect[4, 0] = [10, 0, 0]
    expect[5, 0] = expect[6, 0] = expect[7, 0] = [7, 7, 7]
    assert np.array_equal(cf, expect)


def test_pieces_row_count_and_padding():
    # linear segments (2 rows): K = 2, hold pieces are padded with a zero row (not repeated)
    f = fleet([seg(0, 1, [0, 0, 0], [1, 0, 0])])
    drone, lo, hi, t0r, cf = V2._pieces(f)
    assert cf.shape == (3, 2, 3)
    assert cf[0].tolist() == [[0, 0, 0], [0, 0, 0]]
    assert cf[1].tolist() == [[0, 0, 0], [1, 0, 0]]
    assert cf[2].tolist() == [[1, 0, 0], [0, 0, 0]]           # held end value (1, 0, 0)


def test_pieces_no_usable_segment():
    bad = Segment(2.0, 2.0, np.array([[1.0, 1.0, 1.0]]))
    drone, lo, hi, t0r, cf = V2._pieces(fleet([bad], []))
    assert len(drone) == len(lo) == len(hi) == len(t0r) == 0
    assert cf.shape == (0, 1, 3)


_NAN, _BADROWS = float("nan"), np.array([[9.0, 9.0, 9.0]])
MALFORMED = {
    "t1_equals_t0": Segment(2.0, 2.0, _BADROWS),
    "t1_before_t0": Segment(3.0, 1.0, _BADROWS),
    "coeffs_1d": Segment(0.0, 1.0, np.array([9.0, 9.0, 9.0])),
    "coeffs_two_columns": Segment(0.0, 1.0, np.full((2, 2), 9.0)),
    "coeffs_no_rows": Segment(0.0, 1.0, np.zeros((0, 3))),
    "t0_nan": Segment(_NAN, 1.0, _BADROWS),
    "t0_minus_inf": Segment(-INF, 1.0, _BADROWS),
    "t1_inf": Segment(0.0, INF, _BADROWS),
    "coeff_nan": Segment(0.0, 1.0, np.array([[9.0, _NAN, 9.0]])),
    "coeff_inf": Segment(0.0, 1.0, np.array([[9.0, 9.0, INF]])),
}


@pytest.mark.parametrize("kind", sorted(MALFORMED))
def test_pieces_skip_one_malformed_segment_and_keep_the_next(kind):
    f = fleet([MALFORMED[kind], seg(5, 6, [1, 2, 3])], [seg(0, 1, [0, 0, 0])])
    drone, lo, hi, t0r, cf = V2._pieces(f)
    assert drone.tolist() == [0, 0, 0, 1, 1, 1]
    assert lo.tolist() == [-INF, 5.0, 6.0, -INF, 0.0, 1.0]
    assert hi.tolist() == [5.0, 6.0, INF, 0.0, 1.0, INF]
    assert cf[:3, 0].tolist() == [[1, 2, 3], [1, 2, 3], [1, 2, 3]]


@pytest.mark.parametrize("kind", sorted(MALFORMED))
def test_pieces_drone_with_only_malformed_segments_has_no_pieces(kind):
    # the drone comes first, so a loop that stopped at it would also lose drone 1
    f = fleet([MALFORMED[kind]], [seg(0, 1, [0, 0, 0])])
    drone, lo, hi, t0r, cf = V2._pieces(f)
    assert drone.tolist() == [1, 1, 1]


def test_pieces_overlapped_segments_are_skipped_and_the_next_one_is_kept():
    # A [0, 4]; B [1, 3] ends before the cursor 4; B' [2, 4] ends exactly at the cursor; C [4, 6]
    # starts exactly at the cursor (no gap piece).
    f = fleet([seg(0, 4, [0, 0, 0]), seg(1, 3, [9, 9, 9]), seg(2, 4, [8, 8, 8]),
               seg(4, 6, [1, 1, 1])], [seg(0, 1, [5, 5, 5])])
    drone, lo, hi, t0r, cf = V2._pieces(f)
    assert drone.tolist() == [0, 0, 0, 0, 1, 1, 1]
    assert lo.tolist()[:4] == [-INF, 0.0, 4.0, 6.0]
    assert hi.tolist()[:4] == [0.0, 4.0, 6.0, INF]
    assert t0r.tolist()[:4] == [0.0, 0.0, 4.0, 0.0]
    assert cf[:4, 0].tolist() == [[0, 0, 0], [0, 0, 0], [1, 1, 1], [1, 1, 1]]


# ---------------------------------------------------------------------------------------------
# 3. Piece boxes (centred form about the midpoint)
# ---------------------------------------------------------------------------------------------
def test_piece_boxes_known_quadratic_pieces():
    # x = tau^2 on t in [0, 2], t0r = 0.  About the midpoint 1: (1 + r)^2 = 1 + 2 r + r^2, w = 1,
    # radius 2 + 1 = 3, allowance g * eps * (1 * 2^2); box [1 - 3 - a, 1 + 3 + a] = [-2 - a, 4 + a].
    # Same parabola on t in [3, 5] with t0r = 2 (tau in [1, 3], midpoint tau = 2): (2 + r)^2 =
    # 4 + 4 r + r^2, radius 4 + 1 = 5, allowance g eps 3^2 (|2| + 1 = 3), box [-1 - 9 g eps, 9 + 9 g eps].
    # A piece with lo = -inf holds the constant 5: box 5 +- g eps 5.
    cf = np.zeros((3, 3, 3))
    cf[0, 2, 0] = cf[1, 2, 0] = 1.0
    cf[2, 0, 0] = 5.0
    lo = np.array([0.0, 3.0, -INF])
    hi = np.array([2.0, 5.0, 2.0])
    t0r = np.array([0.0, 2.0, 0.0])
    g = 1e6
    blo, bhi = V2._piece_boxes(lo, hi, t0r, cf, g)
    ge = g * EPS
    np.testing.assert_allclose(blo[0], [-2 - 4 * ge, 0, 0], rtol=0, atol=1e-13)
    np.testing.assert_allclose(bhi[0], [4 + 4 * ge, 0, 0], rtol=0, atol=1e-13)
    np.testing.assert_allclose(blo[1], [4 - (5 + 9 * ge), 0, 0], rtol=0, atol=1e-13)
    np.testing.assert_allclose(bhi[1], [4 + 5 + 9 * ge, 0, 0], rtol=0, atol=1e-13)
    np.testing.assert_allclose(blo[2], [5 - 5 * ge, 0, 0], rtol=0, atol=1e-13)
    np.testing.assert_allclose(bhi[2], [5 + 5 * ge, 0, 0], rtol=0, atol=1e-13)
    # the boxes contain the true range of the parabola (0..4 and 1..9)
    assert blo[0, 0] <= 0.0 and bhi[0, 0] >= 4.0 and blo[1, 0] <= 1.0 and bhi[1, 0] >= 9.0
    assert (blo[:, 1:] == 0.0).all() and (bhi[:, 1:] == 0.0).all()


# ---------------------------------------------------------------------------------------------
# 4. Candidate piece pairs
# ---------------------------------------------------------------------------------------------
def _cand_arrays():
    # idx: drone, lo, hi, box x-range (y = z = 0 unless stated)
    rows = [
        (0, 0.0, 4.0, (0.0, 1.0)),
        (1, 1.0, 3.0, (2.5, 3.5)),
        (2, 2.0, 6.0, (10.0, 11.0)),
        (0, 4.0, 8.0, (0.0, 1.0)),
        (1, 5.0, 9.0, (8.0, 9.0)),
        (3, 0.0, 1.0, (0.5, 0.7)),
    ]
    drone = np.array([r[0] for r in rows], dtype=np.int64)
    lo = np.array([r[1] for r in rows])
    hi = np.array([r[2] for r in rows])
    blo = np.zeros((6, 3))
    bhi = np.zeros((6, 3))
    blo[:, 0] = [r[3][0] for r in rows]
    bhi[:, 0] = [r[3][1] for r in rows]
    # piece 4 also sits at y in [4, 5]: its gap to piece 2 (y = 0) is 4 in y and 1 in x
    blo[4, 1], bhi[4, 1] = 4.0, 5.0
    return drone, lo, hi, blo, bhi


def _pairs(result):
    ia, ib = result
    assert len(ia) == len(ib)
    return {frozenset((int(a), int(b))) for a, b in zip(ia, ib)}


def _fs(*pairs):
    return {frozenset(p) for p in pairs}


# Time overlaps of positive length between different drones: 0-1, 0-2, 1-2, 2-3, 2-4, 3-4, 0-5.
def test_candidate_pairs_without_a_cap_are_all_time_overlaps_of_different_drones():
    d, lo, hi, blo, bhi = _cand_arrays()
    got = V2._candidate_piece_pairs(d, lo, hi, blo, bhi, INF)
    assert _pairs(got) == _fs((0, 1), (0, 2), (1, 2), (2, 3), (2, 4), (3, 4), (0, 5))
    assert len(got[0]) == 7                                  # each pair exactly once


def test_candidate_pairs_box_gap_against_the_cap():
    d, lo, hi, blo, bhi = _cand_arrays()
    # gaps: 0-1 1.5 (squared 2.25); 0-2 9; 1-2 6.5; 2-3 9; 2-4 sqrt(1 + 16) (x 1, y 4);
    #       3-4 sqrt(49 + 16) (x 7, y 4); 0-5 0 (boxes overlap)
    assert _pairs(V2._candidate_piece_pairs(d, lo, hi, blo, bhi, 5.0)) == _fs((0, 1), (0, 5), (2, 4))
    # the pair 3-4 has squared gap 65, 1-2 has 42.25, 0-2 and 2-3 have 81: strict comparison
    assert _pairs(V2._candidate_piece_pairs(d, lo, hi, blo, bhi, 9.0)) == _fs(
        (0, 1), (0, 5), (1, 2), (2, 4), (3, 4))
    assert _pairs(V2._candidate_piece_pairs(d, lo, hi, blo, bhi, 9.0 + 1e-9)) == _fs(
        (0, 1), (0, 5), (1, 2), (2, 4), (3, 4), (0, 2), (2, 3))
    # overlapping boxes only: a gap is never negative, and with cap 1 only the touching pair stays
    assert _pairs(V2._candidate_piece_pairs(d, lo, hi, blo, bhi, 1.0)) == _fs((0, 5))


def test_candidate_pairs_never_pair_a_drone_with_itself():
    d = np.array([0, 0, 1], dtype=np.int64)
    lo = np.array([0.0, 1.0, 2.0])
    hi = np.array([5.0, 6.0, 7.0])                           # all three overlap in time
    z = np.zeros((3, 3))
    assert _pairs(V2._candidate_piece_pairs(d, lo, hi, z, z, 1.0)) == _fs((0, 2), (1, 2))
    assert _pairs(V2._candidate_piece_pairs(d, lo, hi, z, z, INF)) == _fs((0, 2), (1, 2))


def test_candidate_pairs_infinite_cap_does_not_prune_by_distance():
    d = np.array([0, 1], dtype=np.int64)
    lo = np.array([0.0, 0.0])
    hi = np.array([1.0, 1.0])
    blo = np.array([[0.0, 0.0, 0.0], [1e200, 0.0, 0.0]])
    bhi = blo.copy()                                         # squared gap overflows to inf
    assert _pairs(V2._candidate_piece_pairs(d, lo, hi, blo, bhi, INF)) == _fs((0, 1))


def test_candidate_pairs_empty_result_is_two_empty_index_arrays():
    d = np.array([0, 1], dtype=np.int64)
    lo = np.array([0.0, 2.0])
    hi = np.array([1.0, 3.0])                                # no time overlap
    z = np.zeros((2, 3))
    ia, ib = V2._candidate_piece_pairs(d, lo, hi, z, z, INF)
    assert len(ia) == 0 and len(ib) == 0
    assert ia.dtype == np.int64 and ib.dtype == np.int64


# ---------------------------------------------------------------------------------------------
# 5. Cell bounds
# ---------------------------------------------------------------------------------------------
def cb(D, pid, mid, w, E=None, gamma=0.0):
    pid = np.array(pid)
    return V2._cell_bounds(np.array(D, float), pid, np.array(mid, float), np.array(w, float),
                           np.zeros(len(pid)) if E is None else np.array(E, float), gamma)


# D0(s) = (s - 1, 1, 0): |D0|^2 = (s - 1)^2 + 1, minimum 1 at s = 1.   D1 = (3, 4, 0) constant.
D01 = [[[-1, 1, 0], [1, 0, 0]], [[3, 4, 0], [0, 0, 0]]]


def test_cell_bounds_linear_relative_motion_is_exact():
    # cell [0, 3] (mid 1.5, w 1.5): q0 = (0.5, 1), q1 = (1, 0): P = 1.25 + r + r^2, vertex r = -1/2
    #   (interior) value -1/4: lb^2 = 1.25 - 0.25 = 1, lbd = 1 (the true minimum, at s = 1)
    # cell [1.5, 3] (mid 2.25, w 0.75): minimum at the left end s = 1.5: |(0.5, 1)| = sqrt(1.25)
    # cell [0, 0.5] (mid 0.25, w 0.25): minimum at the right end s = 0.5: |(-0.5, 1)| = sqrt(1.25)
    # constant D1 = (3, 4, 0) anywhere: 5
    lbd, ub, ue = cb(D01, [0, 0, 0, 1], [1.5, 2.25, 0.25, 7.0], [1.5, 0.75, 0.25, 3.0])
    s125 = math.sqrt(1.25)
    np.testing.assert_allclose(lbd, [1.0, s125, s125, 5.0], rtol=0, atol=1e-14)
    # midpoint values: |(0.5, 1)|, |(1.25, 1)|, |(-0.75, 1)| = 1.25, |(3, 4)| = 5
    np.testing.assert_allclose(ub, [s125, math.sqrt(2.5625), 1.25, 5.0], rtol=0, atol=1e-14)
    assert (lbd <= ub + 1e-15).all()
    np.testing.assert_array_equal(ue, ub)                    # E = 0


def test_cell_bounds_never_exceed_the_true_minimum_of_the_cell():
    # dense sampling of |D0(s)| on the cell [0, 3] and on [1.5, 3]
    for mid, w in [(1.5, 1.5), (2.25, 0.75), (0.25, 0.25)]:
        s = np.linspace(mid - w, mid + w, 20001)
        true_min = float(np.min(np.hypot(s - 1.0, 1.0)))
        lbd = float(cb(D01, [0], [mid], [w])[0][0])
        assert lbd <= true_min + 1e-12
        # linear: the bound is the exact minimum (the sampled minimum overshoots by < 3e-9)
        assert lbd >= true_min - 1e-8


def test_cell_bounds_inflate_by_E_and_clip_at_zero():
    lbd, ub, ue = cb(D01, [0, 0], [1.5, 1.5], [1.5, 1.5], E=[0.25, 5.0])
    assert lbd[0] == pytest.approx(0.75, abs=1e-14)          # 1 - 0.25
    assert lbd[1] == 0.0                                     # 1 - 5 < 0 is clipped at zero
    assert ue[0] == pytest.approx(math.sqrt(1.25) + 0.25, abs=1e-14)
    assert ue[1] == pytest.approx(math.sqrt(1.25) + 5.0, abs=1e-14)
    assert ub[0] == pytest.approx(math.sqrt(1.25), abs=1e-14)


def test_cell_bounds_rounding_allowance_formula_for_linear_motion():
    # allowance gamma * eps * (|q0| + |q1| w)^2 is subtracted from lb^2 = 1 (cell mid 1.5, w 1.5):
    # |q0| = sqrt(1.25), |q1| = 1
    gamma = 1e9
    lbd = cb(D01, [0], [1.5], [1.5], gamma=gamma)[0][0]
    expected = math.sqrt(1.0 - gamma * EPS * (math.sqrt(1.25) + 1.5) ** 2)
    assert lbd == pytest.approx(expected, abs=1e-12)
    assert lbd < 1.0 - 1e-7                                  # the allowance is visible at this gamma


def test_cell_bounds_allowance_is_per_cell():
    # two cells of the same polynomial with different widths and midpoints get different allowances
    # gamma eps (|q0| + |q1| w)^2:  cell 0: (sqrt(1.25) + 1.5)^2, cell 1: (sqrt(2.5625) + 0.75)^2
    gamma = 1e9
    lbd = cb(D01, [0, 0], [1.5, 2.25], [1.5, 0.75], gamma=gamma)[0]
    e0 = math.sqrt(1.0 - gamma * EPS * (math.sqrt(1.25) + 1.5) ** 2)
    e1 = math.sqrt(1.25 - gamma * EPS * (math.sqrt(2.5625) + 0.75) ** 2)
    np.testing.assert_allclose(lbd, [e0, e1], rtol=0, atol=1e-12)


# Quartic |D|^2 cases. D(r) with the cell centred at 0 (mid = 0), so q = the coefficients.
#   Q1: D = (1 + r^2, r + r^2, 0):  |D|^2 = 1 + 3 r^2 + 2 r^3 + 2 r^4
#   Q2: D = (1 + r - r^2, 0, 0):    |D|^2 = 1 + 2 r - r^2 - 2 r^3 + r^4
#   Q3: D = (1 - r - r^2, 0, 0):    the mirror image
Q1 = [[[1, 0, 0], [0, 1, 0], [1, 1, 0]]]
Q2 = [[[1, 0, 0], [1, 0, 0], [-1, 0, 0]]]
Q3 = [[[1, 0, 0], [-1, 0, 0], [-1, 0, 0]]]


def test_cell_bounds_cubic_and_quartic_tail():
    # Q1, w = 1/2: p = (1, 0, 3, 2, 2); quadratic part min is 0 at r = 0; tail 2 w^3 + 2 w^4 =
    # 1/4 + 1/8 = 3/8; lb^2 = 5/8.  True minimum of |D|^2 is 1.
    lbd, ub, _ = cb(Q1, [0], [0.0], [0.5])
    assert lbd[0] == pytest.approx(math.sqrt(5 / 8), abs=1e-14)
    assert ub[0] == pytest.approx(1.0, abs=1e-15)
    # w = 1: tail 4 > 1, so lb^2 = 1 - 4 < 0 and the bound is clipped to 0
    assert cb(Q1, [0], [0.0], [1.0])[0][0] == 0.0
    assert cb(Q1, [0], [0.0], [1.0], E=[0.5])[0][0] == 0.0


def test_cell_bounds_quartic_allowance_uses_all_orders():
    # Q1, w = 1/2: |q0| = 1, |q1| = 1, |q2| = sqrt 2; S = 1/2 + sqrt(2)/4
    gamma = 1e9
    S = 0.5 + math.sqrt(2) / 4
    expected = math.sqrt(5 / 8 - gamma * EPS * (1.0 + S) ** 2)
    assert cb(Q1, [0], [0.0], [0.5], gamma=gamma)[0][0] == pytest.approx(expected, abs=1e-12)


@pytest.mark.parametrize("poly", [Q2, Q3])
def test_cell_bounds_concave_quadratic_part_uses_the_endpoints(poly):
    # w = 1/4: p = (1, +-2, -1, -+2, 1).  p2 < 0, so the minimum of p1 r + p2 r^2 is at an end:
    # -1/2 - 1/16 = -9/16.  tail = 2 w^3 + w^4 = 1/32 + 1/256 = 9/256.  lb^2 = 1 - 9/16 - 9/256
    # = 103/256.  True minimum of |D|^2 on [-1/4, 1/4]: |D| = 11/16 at the end where D is smallest,
    # so 121/256 >= 103/256.
    lbd = cb(poly, [0], [0.0], [0.25])[0][0]
    assert lbd == pytest.approx(math.sqrt(103 / 256), abs=1e-14)
    assert lbd <= 11 / 16


def test_cell_bounds_constant_polynomial_and_single_row():
    # K = 1 (a point): bound = norm; K = 2 with zero slope
    D = [[[3, 4, 0]]]
    lbd, ub, ue = cb(D, [0], [0.7], [0.2], E=[0.5])
    assert lbd[0] == pytest.approx(4.5, abs=1e-14)
    assert ub[0] == pytest.approx(5.0, abs=1e-14)
    assert ue[0] == pytest.approx(5.5, abs=1e-14)


def test_cell_bounds_emit_no_numpy_warnings():
    # p2 = 0 with p1 != 0 (divide) and p1 = p2 = 0 (invalid): D = 1 + r - r^2/2, and a constant
    zero_p2 = [[[1, 0, 0], [1, 0, 0], [-0.5, 0, 0]]]
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        lbd = cb(zero_p2, [0], [0.0], [0.5])[0][0]
        cb(D01, [1], [1.0], [1.0])
    # |D|^2 = (1 + r - r^2/2)^2 = 1 + 2 r + 0 r^2 - r^3 + r^4/4; with w = 1/2 the end value of 2 r
    # is -1 and the tail is 1/8 + 1/64, so the bound is clipped to 0
    assert lbd == 0.0
    # with w = 1/4: min of 2 r is -1/2, tail 1/64 + 1/1024; lb^2 = 1 - 1/2 - 17/1024 = 495/1024
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        lbd = cb(zero_p2, [0], [0.0], [0.25])[0][0]
    assert lbd == pytest.approx(math.sqrt(495 / 1024), abs=1e-14)
    assert lbd <= 0.71875                                    # |D(-1/4)| = 1 - 1/4 - 1/32


# ---------------------------------------------------------------------------------------------
# 6. The driver: verdicts, bounds, counts, fields
# ---------------------------------------------------------------------------------------------
# Two parked drones, A (0, 0, 4) and B (3, 0, 4) on [0, 1]: |A| = 4, |B| = 5, D = A - B = (-3, 0, 0).
# Pieces: head, segment, tail for each drone; three time-overlapping pairs (head-head, segment-
# segment, tail-tail), all with distance 3.  K = 1, gamma = 4 (1 + 1)^2 = 16, mag = 4 + 5 + 3 = 12,
# E = 16 * 12 * eps = 192 eps.  Level 0: every lower bound (about 3 - 2.4e-14) is above
# u - tol = 3 - 1e-9, so all three cells are discarded: 3 cells, 0 leaves.
def parked_pair():
    return fleet(parked((0, 0, 4)), parked((3, 0, 4)))


def test_two_parked_drones_pass_with_all_fields_known():
    r = verify2(parked_pair(), 2.0)
    assert r.verdict == "PASS"
    assert r.d_min == 2.0
    assert r.cap == 5.0                                      # 2 d_min + 1
    assert r.tol == 1e-9
    assert r.upper_bound == 3.0
    assert r.lower_bound == pytest.approx(3.0 - 1e-9, abs=1e-14)
    assert r.max_rounding_allowance == pytest.approx(192 * EPS, rel=1e-12, abs=0.0)
    assert r.gap_bound == pytest.approx(1e-9 + 192 * EPS, rel=1e-12, abs=0.0)
    assert (r.n_drones, r.n_piece_pairs, r.n_cells, r.n_leaves) == (2, 3, 3, 0)
    assert r.pair_lower == {} and r.pair_upper == {}
    assert 0.0 <= r.runtime_s < 600.0


def test_runtime_is_the_elapsed_time():
    t0 = time.perf_counter()
    r = verify2(parked_pair(), 2.0)
    elapsed = time.perf_counter() - t0
    assert 0.0 < r.runtime_s <= elapsed
    t0 = time.perf_counter()
    r = verify2(fleet(parked((0, 0, 0))), 2.0)               # one drone: no pair, early return
    assert 0.0 < r.runtime_s <= time.perf_counter() - t0


def test_verdict_thresholds_around_the_known_minimum():
    f = parked_pair()
    # lower = 3 - 1e-9, witness = 3 + E with E = 192 eps (exactly representable)
    assert verify2(f, 2.5).verdict == "PASS"
    assert verify2(f, 3.0).verdict == "UNDECIDED"            # not provably >= 3, not witnessed < 3
    assert verify2(f, 3.0 - 1e-6).verdict == "PASS"
    assert verify2(f, 3.0 + 1e-6).verdict == "FAIL"
    assert verify2(f, 3.5).verdict == "FAIL"
    # FAIL needs the witness (3 + E) strictly below d_min: at equality it is undecided
    assert verify2(f, 3.0 + 192 * EPS).verdict == "UNDECIDED"


def test_pass_at_exact_equality():
    # cap = d_min = 2.5: no pair is a candidate, the result is PASS although cap == d_min
    r = verify2(parked_pair(), 2.5, cap=2.5)
    assert r.cap == 2.5 and r.lower_bound == 2.5 and r.verdict == "PASS"
    # the lower bound is u - tol = 3.0 - 1e-9; asking for exactly that value is a PASS (>=)
    d_min = 3.0 - 1e-9
    r = verify2(parked_pair(), d_min)
    assert r.lower_bound == d_min and r.verdict == "PASS"


def test_cap_default_explicit_and_floor():
    f = parked_pair()
    assert verify2(f, 2.0).cap == 5.0
    assert verify2(f, 4.0).cap == 9.0
    assert verify2(f, 2.0, cap=7.5).cap == 7.5
    assert verify2(f, 3.0, cap=1.0).cap == 3.0               # never below d_min
    # the lower bound is capped: pieces at distance 3 with cap 2.75 -> lower = 2.75
    r = verify2(f, 2.0, cap=2.75)
    assert r.lower_bound == 2.75 and r.cap == 2.75


def test_tol_parameter_sets_the_pruning_margin_and_the_gap():
    r = verify2(parked_pair(), 2.0, tol=0.5)
    assert r.tol == 0.5
    assert r.lower_bound == pytest.approx(2.5, abs=1e-14)    # u - tol
    assert r.upper_bound == 3.0
    assert r.gap_bound == pytest.approx(0.5 + 192 * EPS, rel=1e-12, abs=0.0)
    assert r.n_leaves == 0


def test_lower_bound_floor_is_zero_not_one_for_close_drones():
    # distance 0.5: lower = 0.5 - 1e-9 (a floor of 1 would raise it, then be clipped to 0.5)
    f = fleet(parked((0, 0, 0)), parked((0.5, 0, 0)))
    for per_pair in (False, True):
        r = verify2(f, 0.25, per_pair=per_pair)
        assert r.lower_bound == pytest.approx(0.5 - 1e-9, abs=1e-14)
        assert r.upper_bound == 0.5
    r = verify2(f, 0.25, per_pair=True)
    assert r.pair_lower[(0, 1)] == pytest.approx(0.5 - 1e-9, abs=1e-14)


def test_per_pair_mode_dictionaries_and_default_off():
    # A (0, 0, 4), B (3, 0, 4), C (0, 4, 4): distances 3 (A-B), 4 (A-C), 5 (B-C)
    f = fleet(parked((0, 0, 4)), parked((3, 0, 4)), parked((0, 4, 4)))
    r = verify2(f, 2.0, cap=10.0, per_pair=True)
    assert r.pair_upper == {(0, 1): 3.0, (0, 2): 4.0, (1, 2): 5.0}
    assert set(r.pair_lower) == {(0, 1), (0, 2), (1, 2)}
    for key, d in {(0, 1): 3.0, (0, 2): 4.0, (1, 2): 5.0}.items():
        assert r.pair_lower[key] == pytest.approx(d - 1e-9, abs=1e-14)
    assert r.lower_bound == pytest.approx(3.0 - 1e-9, abs=1e-14)
    assert r.upper_bound == 3.0
    assert (r.n_piece_pairs, r.n_cells, r.n_leaves) == (9, 9, 0)
    assert all(isinstance(i, int) and isinstance(j, int) for i, j in r.pair_lower)
    # global mode (the default): same overall bounds, no dictionaries
    g = verify2(f, 2.0, cap=10.0)
    assert g.pair_lower == {} and g.pair_upper == {}
    assert g.lower_bound == pytest.approx(3.0 - 1e-9, abs=1e-14) and g.upper_bound == 3.0
    assert (g.n_piece_pairs, g.n_cells, g.n_leaves) == (9, 9, 0)


def test_far_pair_is_not_resolved_beyond_the_cap():
    f = fleet(parked((0, 0, 0)), parked((100, 0, 0)))
    r = verify2(f, 2.0)                                      # cap 5: the pair is never a candidate
    assert (r.n_piece_pairs, r.n_cells, r.n_leaves) == (0, 0, 0)
    assert r.verdict == "PASS" and r.lower_bound == 5.0 and r.upper_bound == INF
    assert r.gap_bound == 0.0 and r.max_rounding_allowance == 0.0
    r = verify2(f, 2.0, cap=99.9)
    assert r.n_piece_pairs == 0 and r.lower_bound == 99.9
    r = verify2(f, 2.0, cap=100.5)                           # resolved: 100 - tol, capped above
    assert r.n_piece_pairs == 3 and r.upper_bound == 100.0
    assert r.lower_bound == pytest.approx(100.0 - 1e-9, abs=1e-12)
    assert r.verdict == "PASS"


def test_no_usable_segments_gives_the_empty_result():
    bad = Segment(2.0, 2.0, np.array([[1.0, 1.0, 1.0]]))
    r = verify2(fleet([bad], []), 2.0, tol=1e-6)
    assert r.verdict == "PASS"
    assert r.d_min == 2.0 and r.cap == 5.0 and r.lower_bound == 5.0
    assert r.upper_bound == INF and r.gap_bound == 0.0 and r.tol == 1e-6
    assert r.max_rounding_allowance == 0.0
    assert (r.n_drones, r.n_piece_pairs, r.n_cells, r.n_leaves) == (2, 0, 0, 0)
    assert r.runtime_s == 0.0
    assert r.pair_lower == {} and r.pair_upper == {}
    # cap equal to d_min still reports PASS (nothing to compare)
    assert verify2(fleet([bad], []), 2.0, cap=2.0).verdict == "PASS"
    assert verify2(fleet([bad], []), 2.0, cap=2.0).cap == 2.0


def test_malformed_segment_drone_is_ignored_by_the_verdict():
    nan_seg = Segment(0.0, 1.0, np.array([[float("nan"), 0.0, 0.0]]))
    r = verify2(fleet(parked((0, 0, 0)), [nan_seg]), 2.0)    # B would sit on top of A if it counted
    assert r.verdict == "PASS" and r.n_drones == 2 and r.n_piece_pairs == 0
    assert r.lower_bound == 5.0 and r.upper_bound == INF and r.n_cells == 0
    assert 0.0 <= r.runtime_s < 600.0


def test_single_candidate_pair_is_resolved_not_skipped():
    # A parked at the origin on [0, 1]; B(tau) = (10 - 40 tau + 40 tau^2, 3, 0) = (40 (tau - 1/2)^2, 3, 0)
    # dips to (0, 3, 0) at tau = 1/2 and is at (10, 3, 0) (distance sqrt(109) > cap) at both ends.
    # Only the segment-segment pair survives the cap: n_piece_pairs == 1.  Minimum distance 3.
    f = fleet(parked((0, 0, 0)), [seg(0, 1, [10, 3, 0], [-40, 0, 0], [40, 0, 0])])
    r = verify2(f, 2.0)
    assert r.n_piece_pairs == 1
    assert r.verdict == "PASS"
    assert r.upper_bound == 3.0                              # the midpoint is the minimiser
    assert r.lower_bound == pytest.approx(3.0 - 1e-9, abs=1e-13)
    # K = 3: gamma = 4 * 4^2 = 64; mag = |A| + |B| + |D| with abs_eval at |x| + h = 1:
    # B -> (10 + 40 + 40, 3, 0), D -> the same, A -> 0:  2 sqrt(8109)
    assert r.max_rounding_allowance == pytest.approx(64 * EPS * 2 * math.sqrt(8109), rel=1e-12, abs=0.0)
    assert r.gap_bound == pytest.approx(1e-9 + 64 * EPS * 2 * math.sqrt(8109), rel=1e-12, abs=0.0)
    assert verify2(f, 4.0).verdict == "FAIL"                 # 3 < 4


def test_rounding_allowance_for_linear_pieces_uses_gamma_36():
    # A parked at the origin on [0, 2]; B = (1 - 2 tau, -3, 0) on [0, 2], so D = A - B = (2 tau - 1, 3, 0)
    # K = 2: gamma = 4 * 3^2 = 36.  Segment pair: h = 2, mag = 0 + |(1 + 2*2, 3, 0)| + |(1 + 2*2, 3, 0)|
    #   = 2 sqrt(34), the largest over the three pairs (the hold pairs give 2 sqrt(10), 2 sqrt(18)).
    f = fleet(parked((0, 0, 0), 0.0, 2.0), [seg(0, 2, [1, -3, 0], [-2, 0, 0])])
    r = verify2(f, 2.0)
    assert r.max_rounding_allowance == pytest.approx(36 * EPS * 2 * math.sqrt(34), rel=1e-12, abs=0.0)


def test_rounding_allowance_pins_every_term_of_the_magnitude():
    # A(tau) = (4 - 2 tau, 0, 0), B(tau) = (4 - 2 tau, 3, 0) on [0, 1] (parallel, distance 3 always).
    # K = 2, gamma = 36.  Magnitudes mag = |A-term| + |B-term| + |D-term|:
    #   segment pair (h = 1, origins 0): |(4+2, 0, 0)| + |(4+2, 3, 0)| + |(0, 3, 0)| = 6 + sqrt(45) + 3
    #   head pair: |(4,0,0)| + |(4,3,0)| + 3 = 4 + 5 + 3 = 12;  tail pair: 2 + sqrt(13) + 3 = 8.6
    # The segment pair is the largest, 9 + 3 sqrt(5); a wrong sign inside either abs_eval makes it smaller.
    f = fleet([seg(0, 1, [4, 0, 0], [-2, 0, 0])], [seg(0, 1, [4, 3, 0], [-2, 0, 0])])
    r = verify2(f, 2.0)
    assert r.n_piece_pairs == 3 and r.n_cells == 3 and r.n_leaves == 0
    assert r.upper_bound == 3.0
    assert r.max_rounding_allowance == pytest.approx(36 * EPS * (9 + 3 * math.sqrt(5)), rel=1e-12, abs=0.0)


def test_nan_threshold_is_never_decided():
    # nothing is comparable with NaN: the empty path (no usable segment) and the main path agree
    bad = Segment(2.0, 2.0, np.array([[1.0, 1.0, 1.0]]))
    assert verify2(fleet([bad], []), float("nan")).verdict == "UNDECIDED"
    assert verify2(parked_pair(), float("nan")).verdict == "UNDECIDED"


def test_interior_minimum_bisection_counts_and_bounds():
    # D(s) = (2 s - 1, 3, 0) on s in [0, 2]; |D| is smallest, 3, at s = 1/2, not at the midpoint s = 1.
    # Level 0: three cells (hold-hold at s = a with a zero width, the segment cell [0, 2], the other
    # hold-hold).  u = sqrt(10) from the head/segment-midpoint; the segment cell has lbd 3 < u - tol
    # and is bisected; the hold cells have lbd ~ sqrt(10), sqrt(18) >= u - tol and are discarded.
    # Level 1: cells [0, 1] (mid 1/2, ub = 3) and [1, 2]; both are discarded against u - tol = 3 - 1e-9.
    # Total 3 + 2 = 5 cells and no leaves.
    f = fleet(parked((0, 0, 0), 0.0, 2.0), [seg(0, 2, [1, -3, 0], [-2, 0, 0])])
    r = verify2(f, 2.0)
    assert (r.n_piece_pairs, r.n_cells, r.n_leaves) == (3, 5, 0)
    assert r.upper_bound == 3.0
    assert r.lower_bound == pytest.approx(3.0 - 1e-9, abs=1e-13)
    assert r.verdict == "PASS"
    assert verify2(f, 3.0 - 1e-6).verdict == "PASS"
    assert verify2(f, 3.0).verdict == "UNDECIDED"
    assert verify2(f, 3.0 + 1e-6).verdict == "FAIL"
    assert verify2(f, 3.2).verdict == "FAIL"


def test_head_on_crossing_is_a_fail_with_zero_distance():
    # A: x = -5 + 5 t, B: x = 5 - 5 t on [0, 2]; they meet at the origin at t = 1 (the midpoint).
    f = fleet([seg(0, 2, [-5, 0, 0], [5, 0, 0])], [seg(0, 2, [5, 0, 0], [-5, 0, 0])])
    r = verify2(f, 1.0)
    assert r.verdict == "FAIL"
    assert r.upper_bound == 0.0 and r.lower_bound == 0.0
    assert r.d_min == 1.0


def test_hold_across_a_gap_is_used_for_the_gap_interval():
    # A flies x = 2 tau on [10, 11], holds (2, 0, 0) on the gap [11, 13], then is parked on [13, 14].
    # B(tau) = (7 - 20 tau + 20 tau^2, 1, 0), tau = t - 11.5, on [11.5, 12.5]: starts and ends at
    # (7, 1, 0) (distance > 5 from everything A does) and passes (2, 1, 0) at tau = 1/2, i.e. at
    # distance 1 from A's held point, during A's gap only.  The minimum distance is 1, at t = 12.
    f = fleet([seg(10, 11, [0, 0, 0], [2, 0, 0]), seg(13, 14, [2, 0, 0])],
              [seg(11.5, 12.5, [7, 1, 0], [-20, 0, 0], [20, 0, 0])])
    r = verify2(f, 1.5)
    assert r.verdict == "FAIL"
    assert r.upper_bound == 1.0
    assert r.lower_bound == pytest.approx(1.0 - 1e-9, abs=1e-12)
    assert verify2(f, 0.9).verdict == "PASS"


def test_segments_with_nonzero_time_origin_and_partial_overlap():
    # A: x = t - 1 on [1, 5].  B: (3, 1 + (t - 2)/2, 0) on [2, 6].  Overlap [2, 5].
    # |D|^2 = (t - 4)^2 + (1 + (t - 2)/2)^2, derivative 2.5 t - 8 = 0 at t = 3.2, value 0.64 + 2.56 = 3.2
    # The other piece pairs are farther (>= 2.2).
    f = fleet([seg(1, 5, [0, 0, 0], [1, 0, 0])], [seg(2, 6, [3, 1, 0], [0, 0.5, 0])])
    true_min = math.sqrt(3.2)
    r = verify2(f, 1.0)
    assert true_min - 3e-9 <= r.lower_bound <= true_min + 1e-12
    assert true_min - 1e-12 <= r.upper_bound <= true_min + 3e-9
    assert r.n_leaves == 0
    assert verify2(f, 1.7).verdict == "PASS"
    assert verify2(f, 1.8).verdict == "FAIL"


def test_overlapped_segment_piece_starts_at_the_cursor_not_at_its_own_origin():
    # Drone 0: a far parked segment on [0, 4], then x = tau (tau = t - 2) on [2, 6]; only the part
    # t in [4, 6] is a piece (start 4, time origin 2, so tau runs over [2, 4] there).  Drone 1 is
    # parked at (3, 2, 0) on [3, 7].  Over t in [4, 6]: D = (t - 2 - 3, -2) = (t - 5, -2), minimum
    # distance 2 at t = 5; every other pair of pieces is farther (the far segment, the holds at
    # (4, 0, 0) and (3, 2, 0): distance sqrt 5).
    f = fleet([seg(0, 4, [100, 100, 100]), seg(2, 6, [0, 0, 0], [1, 0, 0])],
              [seg(3, 7, [3, 2, 0])])
    r = verify2(f, 1.0)
    assert r.upper_bound == 2.0
    assert r.lower_bound == pytest.approx(2.0 - 1e-9, abs=1e-12)
    assert r.verdict == "PASS"
    assert verify2(f, 2.5).verdict == "FAIL"


def test_cell_discarded_when_its_bound_equals_the_pruning_level(monkeypatch):
    # Parked pair at distance 3, K = 1: the cell bound is sqrt(9 - 144 eps) - 192 eps in IEEE
    # arithmetic (lb^2 = 9 - 144 eps is exact; sqrt is correctly rounded).  Choose tol so that the
    # pruning level u - tol is exactly that number: the cells are discarded (bound reaches the level).
    monkeypatch.setattr(V2, "_MAX_LEVEL", 3)
    bound = math.sqrt(9.0 - 144 * EPS) - 192 * EPS
    tol = 3.0 - bound
    assert 3.0 - tol == bound
    r = verify2(parked_pair(), 2.0, tol=tol)
    assert (r.n_cells, r.n_leaves) == (3, 0)


def test_negative_threshold_keeps_the_lower_bound_below_the_cap():
    # lower_bound <= cap is part of the result's contract; with d_min = -5 the default cap is -5
    r = verify2(parked_pair(), -5.0)
    assert r.cap == -5.0
    assert r.lower_bound <= r.cap
    assert r.verdict == "PASS"


def test_segment_shorter_than_one_second_does_not_extend_past_its_end():
    # A parked at the origin; B: x = 3 - 4 tau on [0, 0.5] (ends at x = 1, then holds there).  The
    # polynomial reaches 0 at tau = 3/4, outside its segment; the true minimum distance is 1.
    f = fleet(parked((0, 0, 0), 0.0, 0.5), [seg(0, 0.5, [3, 0, 0], [-4, 0, 0])])
    r = verify2(f, 0.5)
    assert r.verdict == "PASS"
    assert r.upper_bound == pytest.approx(1.0, abs=2e-9)
    assert r.lower_bound == pytest.approx(1.0, abs=2e-9)


def test_results_are_sound_against_dense_sampling():
    # single segment on [0, 3] for every drone, so holds add nothing beyond the end values
    rng = np.random.default_rng(7)
    ts = np.linspace(0.0, 3.0, 12001)
    for trial in range(6):
        deg = 1 + trial % 3
        drones = [[seg(0, 3, *(rng.integers(-4, 5, size=(deg + 1, 3)).astype(float)))]
                  for _ in range(3)]
        f = fleet(*drones)
        pos = [sum(d[0].coeffs[k][None, :] * ts[:, None] ** k for k in range(deg + 1))
               for d in drones]
        sampled = min(float(np.min(np.linalg.norm(pos[i] - pos[j], axis=1)))
                      for i in range(3) for j in range(i + 1, 3))
        r = verify2(f, 0.5, cap=1e6)
        assert r.lower_bound <= sampled + 1e-12
        assert r.upper_bound >= sampled - 1e-4               # sampling can only overshoot the minimum
        assert r.upper_bound - r.lower_bound <= 1e-8
        assert r.n_leaves == 0


def test_no_numpy_warning_on_a_typical_run():
    f = fleet(parked((0, 0, 0), 0.0, 2.0), [seg(0, 2, [1, -3, 0], [-2, 0, 0])])
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        verify2(f, 2.0)
        verify2(parked_pair(), 2.0, per_pair=True)


# ---------------------------------------------------------------------------------------------
# 7. Depth and size limits (the module constants are lowered so the counts can be derived by hand)
# ---------------------------------------------------------------------------------------------
# Parked pair with tol = 0: u = 3 = |D| exactly, every cell's lower bound (3 - 216 eps) is below
# u, so no cell is ever discarded.  Level 0 has the two zero-width hold cells (always leaves) and
# the segment cell [0, 1]; each level doubles the segment cells.
def test_depth_limit_counts(monkeypatch):
    monkeypatch.setattr(V2, "_MAX_LEVEL", 3)
    r = verify2(parked_pair(), 2.0, tol=0.0)
    # cells: level 0: 3, level 1: 2, level 2: 4, level 3: 8 -> 17; leaves: 2 hold cells + 8
    assert (r.n_cells, r.n_leaves, r.n_piece_pairs) == (17, 10, 3)
    # leaf lower bound = sqrt(9 - 144 eps) - 192 eps, about 3 - 216 eps
    assert r.lower_bound == pytest.approx(3.0 - 216 * EPS, abs=3e-15)
    assert r.lower_bound < 3.0
    assert r.upper_bound == 3.0
    assert r.verdict == "PASS"
    assert verify2(parked_pair(), 3.0, tol=0.0).verdict == "UNDECIDED"   # touching: neither side


def test_depth_limit_chunked_evaluation_gives_the_same_counts(monkeypatch):
    monkeypatch.setattr(V2, "_MAX_LEVEL", 3)
    monkeypatch.setattr(V2, "_CHUNK", 3)
    r = verify2(parked_pair(), 2.0, tol=0.0)
    assert (r.n_cells, r.n_leaves) == (17, 10)
    assert r.lower_bound == pytest.approx(3.0 - 216 * EPS, abs=3e-15)
    assert r.upper_bound == 3.0


def test_size_limit_counts(monkeypatch):
    # 2 * (number of live cells) > _MAX_ITEMS makes all live cells leaves.  With _MAX_ITEMS = 6:
    # level 0: 3 live cells, 6 > 6 is false; the two hold cells are leaves (zero width), the segment
    # cell is split.  Level 1: 2 live cells (4 > 6 false), split.  Level 2: 4 live cells, 8 > 6: all leaves.
    monkeypatch.setattr(V2, "_MAX_ITEMS", 6)
    monkeypatch.setattr(V2, "_MAX_LEVEL", 6)
    r = verify2(parked_pair(), 2.0, tol=0.0)
    assert (r.n_cells, r.n_leaves) == (9, 6)
    assert r.lower_bound == pytest.approx(3.0 - 216 * EPS, abs=3e-15)
    # _MAX_ITEMS = 5: level 0 has 6 > 5: the three live cells are all leaves immediately
    monkeypatch.setattr(V2, "_MAX_ITEMS", 5)
    r = verify2(parked_pair(), 2.0, tol=0.0)
    assert (r.n_cells, r.n_leaves) == (3, 3)
