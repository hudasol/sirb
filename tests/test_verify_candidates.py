"""The vectorised candidate search must return exactly the arrays of the loop reference
(`_candidate_pairs_loop`): same elements, same order, same dtype. Downstream `np.lexsort`
ties depend on the order, so equality is checked with array_equal, not as sets."""

import numpy as np
import pytest

import sirb.verify as V
from sirb.diffcases import random_fleet, random_large_fleet
from sirb.trajectory import Fleet, Segment, Trajectory


def _pieces(fleet):
    good, bad = V._structure(fleet, [])
    return V._build_pieces(good, bad)


def _same(P, R, prefilter):
    ia, ib = V._candidate_pairs(P, R, prefilter)
    ja, jb = V._candidate_pairs_loop(P, R, prefilter)
    assert ia.dtype == ja.dtype and ib.dtype == jb.dtype
    assert np.array_equal(ia, ja) and np.array_equal(ib, jb)
    return len(ia)


def _holds_fleet(seed, n=40):
    """Drones that sit still before/after short, staggered moves (many hold pieces and equal
    start times, so ties in the stable sort and long time overlaps both occur)."""
    rng = np.random.default_rng(seed)
    trs = []
    for _ in range(n):
        p = rng.uniform(-4, 4, 3)
        t0 = float(rng.choice([0.0, 0.5, 1.0, 1.0, 2.0]))
        T = float(rng.uniform(0.2, 1.5))
        v = rng.uniform(-2, 2, 3)
        c = np.array([p, v])
        segs = [Segment(t0, t0 + T, c)]
        if rng.random() < 0.5:                        # gap, then a held segment
            end = c[0] + c[1] * T
            segs.append(Segment(t0 + T + 0.3, t0 + T + 0.6, end[None, :]))
        trs.append(Trajectory(segs))
    return Fleet(trs)


def test_loop_reference_is_the_old_algorithm_on_a_hand_case():
    a = Trajectory([Segment(0.0, 1.0, np.array([[0.0, 0, 0], [1, 0, 0]]))])
    b = Trajectory([Segment(0.5, 1.5, np.array([[2.5, 0, 0], [0, 0, 0]]))])
    far = Trajectory([Segment(0.0, 2.0, np.array([[500.0, 0, 0]]))])
    P = _pieces(Fleet([a, b, far]))
    assert _same(P, 2.0, True) > 0
    ia, ib = V._candidate_pairs(P, 2.0, True)
    assert not np.any(P.drone[ia] == 2) and not np.any(P.drone[ib] == 2)    # far drone pruned
    assert _same(P, 2.0, False) > len(ia)                                   # not pruned without prefilter


@pytest.mark.parametrize("prefilter", [True, False])
@pytest.mark.parametrize("R", [0.0, 0.5, 2.0, 6.0, 1e9])
def test_random_small_fleets_identical(R, prefilter):
    n_pairs = 0
    for s in range(60):
        fleet, _, _ = random_fleet(s)
        n_pairs += _same(_pieces(fleet), R, prefilter)
    assert n_pairs > 0


@pytest.mark.parametrize("prefilter", [True, False])
def test_larger_fleets_identical(prefilter):
    for s in range(10):
        n = 12 + 10 * (s % 5)
        fleet, _, _ = random_large_fleet(s, n=n, spread=3.0 + 10.0 * (s % 3))
        P = _pieces(fleet)
        for R in (1.0, 4.0, 25.0):
            _same(P, R, prefilter)


@pytest.mark.parametrize("prefilter", [True, False])
def test_fleets_with_holds_and_gaps_identical(prefilter):
    for s in range(40):
        P = _pieces(_holds_fleet(s))
        for R in (0.5, 3.0, 10.0):
            _same(P, R, prefilter)


@pytest.mark.parametrize("prefilter", [True, False])
def test_empty_single_and_all_bad_fleets(prefilter):
    for fleet in (Fleet([]), Fleet([Trajectory([Segment(0.0, 1.0, np.array([[0.0, 0, 0]]))])])):
        P = _pieces(fleet)
        assert _same(P, 5.0, prefilter) == 0
    # Every drone structurally bad (a time gap would be bridged, so use non-finite coefficients).
    nan = np.array([[np.nan, 0, 0]])
    P = _pieces(Fleet([Trajectory([Segment(0.0, 1.0, nan)]), Trajectory([Segment(0.0, 1.0, nan)])]))
    assert P.n == 0 and _same(P, 5.0, prefilter) == 0


@pytest.mark.parametrize("block,tile_rows", [(1, 1024), (3, 1024), (50, 1024), (10**9, 1), (10**9, 3),
                                              (200, 4)])
@pytest.mark.parametrize("prefilter", [True, False])
def test_chunk_boundaries(monkeypatch, block, tile_rows, prefilter):
    """Pair counts far above the cell budget, rows wider than one tile, and tiles of 1 to 4 rows."""
    monkeypatch.setattr(V, "_PAIR_BLOCK", block)
    monkeypatch.setattr(V, "_TILE_ROWS", tile_rows)
    exceeded = 0
    fleets = [_holds_fleet(s, n=30) for s in range(4)]
    fleets += [random_large_fleet(s, n=25, spread=5.0)[0] for s in range(4)]
    fleets += [random_fleet(s)[0] for s in range(20)]
    for fleet in fleets:
        P = _pieces(fleet)
        for R in (1.0, 8.0):
            exceeded += _same(P, R, prefilter) > min(block, tile_rows)
    assert exceeded > 0


def test_default_block_is_exercised_by_a_single_dense_fleet(monkeypatch):
    """With the real block size and a fleet whose pair count exceeds it (several blocks)."""
    fleet, _, _ = random_large_fleet(3, n=450, spread=40.0)
    P = _pieces(fleet)
    n = _same(P, 1e9, False)
    assert n > V._PAIR_BLOCK
    _same(P, 6.0, True)


def test_pair_minima_unchanged_end_to_end(monkeypatch):
    """The full per-pair minima are the same whichever implementation feeds them."""
    for s in range(20):
        fleet, _, _ = random_fleet(s)
        for prefilter in (True, False):
            new = V.pair_minima(fleet, 4.0, prefilter=prefilter)
            with monkeypatch.context() as m:
                m.setattr(V, "_candidate_pairs", V._candidate_pairs_loop)
                old = V.pair_minima(fleet, 4.0, prefilter=prefilter)
            assert new == old


def test_gap_sum_matches_einsum_bitwise_across_slice_lengths():
    """The squared box gap is the same float whether einsum sees a short or a long block."""
    rng = np.random.default_rng(0)
    g = np.abs(rng.normal(size=(5000, 3))) * 10 ** rng.uniform(-6, 3, (5000, 3))
    whole = np.einsum("ij,ij->i", g, g)
    for lo, hi in ((0, 1), (1, 8), (3, 4999), (17, 18), (100, 4100)):
        assert np.array_equal(np.einsum("ij,ij->i", g[lo:hi].copy(), g[lo:hi].copy()), whole[lo:hi])
