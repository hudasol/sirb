"""Tests added after the first mutation-testing run on the planners and ``sirb/assign.py``
(docs/MUTATION.md). The plan-level tests use tiny hand-made target sets (``build_targets`` is
replaced inside the planner module) so that every expected number can be derived by hand and the
tests run in milliseconds. Expected values come from the documented formulas (minimum-jerk
quintic, rest-to-rest legs, closed-form separation), not from reading back the planner output.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from sirb import assign as A
from sirb.planner import baseline as B
from sirb.planner import prioritized as P
from sirb.planner import waves as W
from sirb.planner.common import (
    K_A,
    K_J,
    K_V,
    hold_segment,
    leg_segment,
    min_leg_time,
    path_duration,
    path_segments,
    stitch,
)
from sirb.trajectory import Segment
from sirb.verify import verify

LIM = {"vmax_h": 10.0, "vmax_up": 5.0, "vmax_down": 2.0, "amax": 100.0, "jmax": 1000.0,
       "z_min": 0.0, "z_max": 100.0, "min_separation_m": 2.0,
       "geofence": {"x": [-200.0, 200.0], "y": [-200.0, 200.0]},
       "crowd": {"y": -500.0, "min_distance_m": 1.0}}


def lim_with(**kw):
    return {**LIM, **kw}


# ---------------------------------------------------------------------------------------------
# planner.common
# ---------------------------------------------------------------------------------------------
def test_constants_are_the_quintic_profile_peaks():
    assert K_V == 1.875 and K_J == 60.0
    assert K_A == pytest.approx(math.sqrt(100.0 / 3.0), rel=1e-15)
    # peak speed, accel, jerk of s(u) = 10u^3 - 15u^4 + 6u^5 (per unit displacement and T)
    u = np.linspace(0, 1, 200001)
    assert (30 * u**2 - 60 * u**3 + 30 * u**4).max() == pytest.approx(K_V, abs=1e-9)
    assert np.abs(60 * u - 180 * u**2 + 120 * u**3).max() == pytest.approx(K_A, abs=1e-6)


MARGIN = 1.02


@pytest.mark.parametrize("d,lim,expected", [
    # horizontal-limited, along x, along y, and 3-4-5 horizontal (z ignored by dh)
    ((100.0, 0.0, 0.0), LIM, MARGIN * K_V * 100 / 10.0),
    ((0.0, 100.0, 0.0), LIM, MARGIN * K_V * 100 / 10.0),
    ((30.0, 40.0, 0.0), LIM, MARGIN * K_V * 50 / 10.0),
    # climb uses vmax_up (also for exactly level flight), descent uses vmax_down
    ((0.0, 0.0, 50.0), LIM, MARGIN * K_V * 50 / 5.0),
    ((0.0, 0.0, -50.0), LIM, MARGIN * K_V * 50 / 2.0),
    # short climb of 0.5 m with the other limits very loose: vertical term dominates
    ((0.0, 0.0, 0.5), lim_with(amax=1e9, jmax=1e9), MARGIN * K_V * 0.5 / 5.0),
    ((0.0, 0.0, -0.5), lim_with(amax=1e9, jmax=1e9), MARGIN * K_V * 0.5 / 2.0),
    # accel-limited and jerk-limited
    ((100.0, 0.0, 0.0), lim_with(vmax_h=1e9, amax=1.0, jmax=1e9),
     MARGIN * math.sqrt(K_A * 100.0 / 1.0)),
    ((100.0, 0.0, 0.0), lim_with(vmax_h=1e9, amax=1e9, jmax=1.0),
     MARGIN * (K_J * 100.0 / 1.0) ** (1.0 / 3.0)),
    ((0.0, 3.0, 4.0), lim_with(vmax_h=1e9, vmax_up=1e9, amax=2.0, jmax=1e9),
     MARGIN * math.sqrt(K_A * 5.0 / 2.0)),
])
def test_min_leg_time_matches_the_limit_formulas(d, lim, expected):
    assert min_leg_time(np.array(d), lim) == pytest.approx(expected, rel=1e-12)


def test_min_leg_time_takes_the_slowest_constraint_and_scales_with_margin():
    d = np.array([100.0, 0.0, 0.0])
    base = min_leg_time(d, lim_with(vmax_h=1e9, amax=1.0, jmax=1e9))
    assert min_leg_time(d, lim_with(vmax_h=10.0, amax=1.0, jmax=1e9)) == pytest.approx(base)
    assert min_leg_time(d, lim_with(vmax_h=1.0, amax=1.0, jmax=1e9)) == pytest.approx(
        MARGIN * K_V * 100.0)
    assert min_leg_time(d, LIM, margin=1.5) == pytest.approx(1.5 * K_V * 100 / 10.0)
    assert min_leg_time(d, LIM, margin=1.0) == pytest.approx(K_V * 100 / 10.0)


def test_min_leg_time_zero_threshold():
    assert min_leg_time(np.array([0.0, 0.0, 0.0]), LIM) == 0.0
    assert min_leg_time(np.array([5e-13, 0.0, 0.0]), LIM) == 0.0
    assert min_leg_time(np.array([1e-12, 0.0, 0.0]), LIM) > 0.0         # exactly 1e-12 is a leg
    assert min_leg_time(np.array([1e-6, 0.0, 0.0]), LIM) > 0.0


def test_leg_segment_is_a_rest_to_rest_quintic():
    p0, p1 = np.array([1.0, 2.0, 3.0]), np.array([11.0, -6.0, 5.0])
    s = leg_segment(4.0, 2.5, p0, p1)
    assert (s.t0, s.t1) == (4.0, 6.5)
    assert s.coeffs.shape == (6, 3) and s.coeffs.dtype == float
    assert np.array_equal(s.coeffs[0], p0)
    assert np.all(s.coeffs[1] == 0) and np.all(s.coeffs[2] == 0)
    d, T = p1 - p0, 2.5
    assert np.allclose(s.coeffs[3], 10 * d / T**3, rtol=1e-14)
    assert np.allclose(s.coeffs[4], -15 * d / T**4, rtol=1e-14)
    assert np.allclose(s.coeffs[5], 6 * d / T**5, rtol=1e-14)
    # ends exactly at p1 with zero velocity and acceleration at both ends
    tau = np.array([T])
    c = s.coeffs
    pos = sum(c[k] * tau[0] ** k for k in range(6))
    vel = sum(k * c[k] * tau[0] ** (k - 1) for k in range(1, 6))
    acc = sum(k * (k - 1) * c[k] * tau[0] ** (k - 2) for k in range(2, 6))
    assert np.allclose(pos, p1, atol=1e-12) and np.allclose(vel, 0, atol=1e-12)
    assert np.allclose(acc, 0, atol=1e-12)


def test_leg_and_hold_segments_have_float_coefficients_for_integer_input():
    s = leg_segment(0.0, 2.0, np.array([0, 0, 0]), np.array([4, 0, 0]))
    assert s.coeffs.dtype == float
    h = hold_segment(1.0, 3.0, np.array([1, 2, 3]))
    assert h.coeffs.dtype == float and h.coeffs.shape == (1, 3)
    assert (h.t0, h.t1) == (1.0, 4.0) and np.array_equal(h.coeffs[0], [1.0, 2.0, 3.0])


def test_path_segments_and_duration_chain_legs_at_their_minimum_time():
    wps = [np.array([0.0, 0, 0]), np.array([100.0, 0, 0]), np.array([100.0, 0, 0]),
           np.array([100.0, 50.0, 0])]
    segs = path_segments(3.0, wps, LIM)
    T1 = min_leg_time(np.array([100.0, 0, 0]), LIM)
    T2 = min_leg_time(np.array([0.0, 50.0, 0]), LIM)
    assert len(segs) == 2                                  # the zero-length leg is skipped
    assert segs[0].t0 == 3.0 and segs[0].t1 == pytest.approx(3.0 + T1)
    assert segs[1].t0 == pytest.approx(3.0 + T1) and segs[1].t1 == pytest.approx(3.0 + T1 + T2)
    assert path_duration(wps, LIM) == pytest.approx(T1 + T2)
    assert path_duration(wps[:1], LIM) == 0


def test_stitch_pads_start_gaps_and_end_with_holds_and_adds_nothing_when_contiguous():
    a = leg_segment(2.0, 3.0, np.array([0.0, 0, 0]), np.array([9.0, 0, 0]))
    out = stitch([[a]], 10.0)
    segs = out[0].segments
    assert [(s.t0, s.t1) for s in segs] == [(0.0, 2.0), (2.0, 5.0), (5.0, 10.0)]
    assert np.array_equal(segs[0].coeffs, [[0.0, 0.0, 0.0]])          # held at its first position
    assert np.allclose(segs[2].coeffs, [[9.0, 0.0, 0.0]])             # held at its last position
    # contiguous pieces: no zero-length filler anywhere, and no padding when t_end is reached
    b = leg_segment(0.0, 3.0, np.array([0.0, 0, 0]), np.array([9.0, 0, 0]))
    c = leg_segment(3.0, 1.0, np.array([9.0, 0, 0]), np.array([9.0, 4.0, 0]))
    segs = stitch([[b, c]], 4.0)[0].segments
    assert [(s.t0, s.t1) for s in segs] == [(0.0, 3.0), (3.0, 4.0)]
    # a gap between pieces is bridged by holding the previous end position
    d = leg_segment(5.0, 1.0, np.array([9.0, 4.0, 0]), np.array([9.0, 8.0, 0]))
    segs = stitch([[b, d]], 6.0)[0].segments
    assert [(s.t0, s.t1) for s in segs] == [(0.0, 3.0), (3.0, 5.0), (5.0, 6.0)]
    assert np.allclose(segs[1].coeffs, [[9.0, 0.0, 0.0]])
    # padding at the end by a short interval still happens (not swallowed by a coarse tolerance)
    segs = stitch([[b]], 3.5)[0].segments
    assert [(s.t0, s.t1) for s in segs] == [(0.0, 3.0), (3.0, 3.5)]


def test_stitch_tolerance_is_a_picosecond():
    a = Segment(0.0, 1.0, np.zeros((1, 3)))
    near = Segment(1.0 + 5e-13, 2.0, np.zeros((1, 3)))
    assert len(stitch([[a, near]], 2.0)[0].segments) == 2             # sub-tolerance gap: no filler
    far = Segment(1.0 + 3e-12, 2.0, np.zeros((1, 3)))
    assert len(stitch([[a, far]], 2.0)[0].segments) == 3
    assert len(stitch([[a]], 1.0 + 5e-13)[0].segments) == 1
    assert len(stitch([[a]], 1.0 + 3e-12)[0].segments) == 2


# ---------------------------------------------------------------------------------------------
# sirb.assign
# ---------------------------------------------------------------------------------------------
def test_assign_methods_known_answers():
    src = np.array([[0.0, 0, 0], [10.0, 0, 0], [20.0, 0, 0]])
    dst = np.array([[21.0, 0, 0], [1.0, 0, 0], [11.0, 0, 0]])
    assert A.assign_hungarian_sq(src, dst).tolist() == [1, 2, 0]
    assert A.assign_hungarian_len(src, dst).tolist() == [1, 2, 0]
    assert A.assign_greedy(src, dst).tolist() == [1, 2, 0]
    assert A.assign_identity(src, dst).tolist() == [0, 1, 2]
    # squared cost prefers two medium moves over one long and one short; length cost ties
    s2 = np.array([[0.0, 0, 0], [3.0, 0, 0]])
    d2 = np.array([[2.0, 0, 0], [5.0, 0, 0]])
    assert A.assign_hungarian_sq(s2, d2).tolist() == [0, 1]
    s3 = np.array([[0.0, 0, 0], [1.0, 0, 0]])
    d3 = np.array([[1.0, 0, 0], [10.0, 0, 0]])
    assert A.assign_hungarian_sq(s3, d3).tolist() == [0, 1]


def test_greedy_takes_the_globally_closest_pair_first():
    src = np.array([[0.0, 0, 0], [4.0, 0, 0]])
    dst = np.array([[5.0, 0, 0], [2.9, 0, 0]])
    # closest pair is src1 -> dst0 (1.0), which forces src0 -> dst1 (2.9)
    assert A.assign_greedy(src, dst).tolist() == [1, 0]
    rng = np.random.default_rng(3)
    s, d = rng.random((30, 3)) * 100, rng.random((30, 3)) * 100
    perm = A.assign_greedy(s, d)
    assert sorted(perm.tolist()) == list(range(30))


def test_random_and_pinned_depend_on_the_seed_only():
    src = np.random.default_rng(1).random((40, 3)) * 50
    dst = np.random.default_rng(2).random((40, 3)) * 50
    assert A.assign_random(src, dst, 7).tolist() == np.random.default_rng(7).permutation(40).tolist()
    assert A.assign_random(src, dst, 7).tolist() == A.assign_random(src, dst, 7).tolist()
    assert A.assign_random(src, dst, 7).tolist() != A.assign_random(src, dst, 8).tolist()
    p1, p2 = A.assign_pinned(src, dst, 5), A.assign_pinned(src, dst, 5)
    assert p1.tolist() == p2.tolist() and p1.tolist() != A.assign_pinned(src, dst, 6).tolist()


def test_pinned_swaps_exactly_the_documented_number_of_targets():
    rng = np.random.default_rng(0)
    src, dst = rng.random((100, 3)) * 80, rng.random((100, 3)) * 80
    base = A.assign_hungarian_sq(src, dst)
    pinned = A.assign_pinned(src, dst, 3)                       # frac 0.15 of 100 -> 14 drones
    assert sorted(pinned.tolist()) == list(range(100))
    assert int((pinned != base).sum()) == 14
    # small fleets: at least one pair swaps
    s, d = src[:6], dst[:6]
    assert int((A.assign_pinned(s, d, 3) != A.assign_hungarian_sq(s, d)).sum()) == 2
    assert int((A.assign_pinned(src, dst, 3, frac=0.5) != base).sum()) == 50
    # swapped drones were chosen without replacement
    assert int((A.assign_pinned(src, dst, 11, frac=1.0) != base).sum()) == 100


def test_total_path_length_sums_over_drones():
    src = np.array([[0.0, 0, 0], [0.0, 0, 0]])
    dst = np.array([[3.0, 4.0, 0], [0.0, 0.0, 12.0]])
    assert A.total_path_length(src, dst, np.array([0, 1])) == pytest.approx(17.0)
    assert A.total_path_length(src, dst, np.array([1, 0])) == pytest.approx(17.0)
    assert isinstance(A.total_path_length(src, dst, np.array([0, 1])), float)


def test_linear_conflicts_counts_pairs_once_with_strict_threshold():
    # parallel drones, constant separation 2.0: conflict only if d_sep > 2.0
    src = np.array([[0.0, 0, 0], [0.0, 2.0, 0]])
    dst = np.array([[10.0, 0, 0], [10.0, 2.0, 0]])
    ident = np.array([0, 1])
    assert A.linear_conflicts(src, dst, ident, 2.0) == 0
    assert A.linear_conflicts(src, dst, ident, 2.0001) == 1
    # three mutually conflicting drones are 3 pairs, not 6
    src3 = np.array([[0.0, 0, 0], [1.0, 0, 0], [0.0, 1.0, 0]])
    assert A.linear_conflicts(src3, src3, np.arange(3), 5.0) == 3


def test_linear_conflicts_closest_approach_is_clipped_to_the_motion_interval():
    # B ahead of A on a line, A approaching but only to a distance of 1 by the end: d_min = 1
    src = np.array([[0.0, 0, 0], [10.0, 0, 0]])
    dst = np.array([[9.0, 0, 0], [10.0, 0, 0]])
    ident = np.array([0, 1])
    assert A.linear_conflicts(src, dst, ident, 1.5) == 1
    assert A.linear_conflicts(src, dst, ident, 0.9) == 0           # unclipped u=10 would be 0
    # moving apart: closest approach at the start (clip at 0), distance 3
    src = np.array([[0.0, 0, 0], [3.0, 0, 0]])
    dst = np.array([[-5.0, 0, 0], [3.0, 0, 0]])
    assert A.linear_conflicts(src, dst, ident, 3.5) == 1
    assert A.linear_conflicts(src, dst, ident, 2.9) == 0
    # head-on swap passes through each other
    src = np.array([[0.0, 0, 0], [10.0, 0, 0]])
    dst = np.array([[10.0, 0, 0], [0.0, 0, 0]])
    assert A.linear_conflicts(src, dst, ident, 0.1) == 1


def test_linear_conflicts_identical_motion_has_constant_distance():
    # zero relative velocity: the guard branch (a ~ 0) must use the start distance
    src = np.array([[0.0, 0, 0], [1.5, 0, 0]])
    dst = src + np.array([5.0, 5.0, 0])
    ident = np.array([0, 1])
    assert A.linear_conflicts(src, dst, ident, 1.6) == 1
    assert A.linear_conflicts(src, dst, ident, 1.4) == 0
    # tiny relative velocity below the guard still gives the start distance
    dst2 = dst + np.array([[0.0, 0, 0], [1e-10, 0, 0]])
    assert A.linear_conflicts(src, dst2, ident, 1.6) == 1


def test_linear_conflicts_blocks_give_the_same_count_as_a_single_block():
    rng = np.random.default_rng(5)
    src, dst = rng.random((40, 3)) * 20, rng.random((40, 3)) * 20
    perm = A.assign_random(src, dst, 1)
    full = A.linear_conflicts(src, dst, perm, 6.0, block=1000)
    for block in (1, 3, 7, 40):
        assert A.linear_conflicts(src, dst, perm, 6.0, block=block) == full
    assert full > 0


def test_evaluate_reports_every_method():
    rng = np.random.default_rng(1)
    src, dst = rng.random((12, 3)) * 30, rng.random((12, 3)) * 30
    out = A.evaluate(src, dst, 2.0, seed=4)
    assert set(out) == set(A.METHODS)
    for name, rec in out.items():
        assert set(rec) == {"total_path_length_m", "conflicting_pairs"}
        perm = (A.METHODS[name](src, dst, 4) if name in ("random", "pinned15")
                else A.METHODS[name](src, dst))
        assert rec["total_path_length_m"] == pytest.approx(A.total_path_length(src, dst, perm))
        assert rec["conflicting_pairs"] == A.linear_conflicts(src, dst, perm, 2.0)


# ---------------------------------------------------------------------------------------------
# planner.waves helpers
# ---------------------------------------------------------------------------------------------
def test_point_seg_dist_interior_and_clipped_ends():
    a = np.array([[0.0, 0, 0], [0.0, 0, 0], [0.0, 0, 1.0], [0.0, 0, 0]])
    b = np.array([[10.0, 0, 0], [0.0, 0, 0], [0.0, 0, 5.0], [0.5, 0, 0]])
    pts = np.array([[5.0, 3.0, 0], [-4.0, 3.0, 0], [13.0, 4.0, 0], [0.0, 0.0, 0.0],
                    [0.0, 3.0, 3.0], [0.5, 1.0, 0.0]])
    D = W._point_seg_dist(pts, a, b)
    assert D.shape == (4, 6)
    s34 = math.sqrt(34.0)
    assert D[0] == pytest.approx([3.0, 5.0, 5.0, 0.0, math.hypot(3, 3), 1.0])
    # degenerate (point) segment: plain distance to that point
    assert D[1] == pytest.approx([s34, 5.0, math.sqrt(185.0), 0.0, math.hypot(3, 3),
                                  math.hypot(0.5, 1.0)])
    # vertical segment from z=1 to z=5: point (0,3,3) is 3 away, point (0,0,0) is 1 below it
    assert D[2][4] == pytest.approx(3.0) and D[2][3] == pytest.approx(1.0)
    # short (0.5 m) segment: nearest point is its far end, not its start
    assert D[3][5] == pytest.approx(1.0) and D[3][2] == pytest.approx(math.hypot(12.5, 4.0))


def test_sync_min_dist_closed_form_and_clipping():
    # head-on swap passes through each other; parallel pair keeps 5 m; diagonal known case
    src = np.array([[0.0, 0, 0], [10.0, 0, 0], [0.0, 5.0, 0]])
    dst = np.array([[10.0, 0, 0], [0.0, 0, 0], [10.0, 5.0, 0]])
    D = W._sync_min_dist(src, dst)
    assert D.shape == (3, 3)
    assert D[0, 1] == pytest.approx(0.0, abs=1e-12) and D[1, 0] == D[0, 1]
    assert D[0, 2] == pytest.approx(5.0) and np.allclose(np.diag(D), 0.0)
    # approach that stops short of the meeting point: clipped to the end of the motion (u = 1)
    s = np.array([[0.0, 0, 0], [10.0, 0, 0]])
    d = np.array([[1.0, 0, 0], [10.0, 0, 0]])
    assert W._sync_min_dist(s, d)[0, 1] == pytest.approx(9.0)
    # diverging pair: clipped to the start (u = 0)
    s = np.array([[0.0, 0, 0], [3.0, 0, 0]])
    d = np.array([[-5.0, 0, 0], [3.0, 0, 0]])
    assert W._sync_min_dist(s, d)[0, 1] == pytest.approx(3.0)
    # tiny relative velocity below the guard still returns the starting distance
    s = np.array([[0.0, 0, 0], [3.0, 0, 0]])
    d = np.array([[1.0, 0, 0], [4.0 + 1e-10, 0, 0]])
    assert W._sync_min_dist(s, d)[0, 1] == pytest.approx(3.0, abs=1e-9)


def test_schedule_waves_parallel_pair_at_exactly_the_planning_distance_shares_a_wave():
    src = np.array([[0.0, 0, 50], [0.0, 2.0, 50]])
    dst = np.array([[10.0, 0, 50], [10.0, 2.0, 50]])
    assert W.schedule_waves(src, dst, 2.0).tolist() == [0, 0]
    # just above: they conflict, and no ordering in time can keep a 2 m lane clear either
    with pytest.raises(RuntimeError):
        W.schedule_waves(src, dst, 2.0001)


def test_schedule_waves_crossing_pair_splits_with_the_new_wave_placed_first():
    # A flies along y=0, B across it at x=5; both conflict.  Highest conflict degree first, ties
    # in index order: A takes wave 0.0, B is then tried in A's wave (no), before it (yes).
    src = np.array([[0.0, 0, 50], [5.0, -5.0, 50]])
    dst = np.array([[10.0, 0, 50], [5.0, 5.0, 50]])
    assert W.schedule_waves(src, dst, 2.05).tolist() == [1, 0]


def test_schedule_waves_orders_by_conflict_degree():
    # C flies up the x = 5 line and meets A (at t = 1/3) and B (at t = 2/3) head on; A and B are
    # compatible.  C has the highest conflict degree, so it is placed first; A and B are then put
    # together before it.
    src = np.array([[0.0, 0, 50], [-10.0, 10.0, 50], [5.0, -10.0, 50]])
    dst = np.array([[15.0, 0, 50], [12.5, 10.0, 50], [5.0, 20.0, 50]])
    sync = W._sync_min_dist(src, dst)
    assert sync[0, 2] < 0.1 and sync[1, 2] < 0.1 and sync[0, 1] > 9.0
    assert W.schedule_waves(src, dst, 2.05).tolist() == [0, 0, 1]


def test_schedule_waves_waiting_drones_must_clear_the_moving_drones_path():
    # B sits at x=5 and never moves; A flies through it, so any wave order collides
    src = np.array([[0.0, 0, 50], [5.0, 0, 50]])
    dst = np.array([[10.0, 0, 50], [5.0, 0, 50]])
    with pytest.raises(RuntimeError, match="wave scheduling stuck at drone"):
        W.schedule_waves(src, dst, 2.05)


def test_schedule_waves_enforces_clearance_between_waves_not_only_inside_them():
    # Two drones in different waves: the drone that waits at its start must be d_plan away from
    # the other's path, and the one that has arrived must be d_plan from it.  A flies y=0 from
    # x=0 to 10.  B starts at (5,2) -- exactly 2.0 from A's path -- and flies up to (5,12), so a
    # synchronous flight crosses nothing.  Put C across A's path to force at least two waves.
    src = np.array([[0.0, 0, 50], [5.0, 2.0, 50], [5.0, -8.0, 50]])
    dst = np.array([[10.0, 0, 50], [5.0, 12.0, 50], [5.0, 8.0, 50]])
    waves = W.schedule_waves(src, dst, 2.0)
    # every pair of drones is either in one wave with closed-form clearance or separated in time
    # with clearance from the other's static end point
    sync = W._sync_min_dist(src, dst)
    sd = W._point_seg_dist(src, src, dst)
    dd = W._point_seg_dist(dst, src, dst)
    for i in range(3):
        for j in range(3):
            if i == j:
                continue
            if waves[i] == waves[j]:
                assert sync[i, j] >= 2.0 - 1e-9
            elif waves[i] < waves[j]:
                assert sd[i, j] >= 2.0 - 1e-9 and dd[j, i] >= 2.0 - 1e-9
    assert len(set(waves.tolist())) >= 2


# ---------------------------------------------------------------------------------------------
# Plan-level tests with hand-made targets
# ---------------------------------------------------------------------------------------------
def make_show(stages_hold, name="tiny", seed=5, lim=None, n=None):
    return {"name": name, "seed": seed, "n_drones": n, "limits": lim or LIM,
            "stages": [{"hold_s": h} for h in stages_hold]}


def install_targets(monkeypatch, module, targets):
    monkeypatch.setattr(module, "build_targets", lambda show: [np.array(t, float) for t in targets])


T0 = [[0.0, 0, 20], [10.0, 0, 20], [20.0, 0, 20], [30.0, 0, 20]]
T1 = [[0.0, 40.0, 20], [10.0, 40.0, 20], [20.0, 40.0, 20], [30.0, 40.0, 20]]
T2 = [[0.0, 40.0, 60], [10.0, 40.0, 60], [20.0, 40.0, 60], [30.0, 40.0, 60]]


def legs_of(traj):
    return [s for s in traj.segments if s.coeffs.shape[0] == 6]


def holds_of(traj):
    return [s for s in traj.segments if s.coeffs.shape[0] == 1]


def test_baseline_timeline_info_and_metadata(monkeypatch):
    install_targets(monkeypatch, B, [T0, T1, T2])
    show = make_show([2.0, 1.5, 4.0], seed=5, n=4)
    fleet, info = B.plan_baseline(show, assign="identity")
    L1 = min_leg_time(np.array([0.0, 40.0, 0.0]), LIM)
    L2 = min_leg_time(np.array([0.0, 0.0, 40.0]), LIM)
    assert set(info) == {"transitions", "t_end", "plan_runtime_s"}
    assert 0.0 <= info["plan_runtime_s"] < 60.0
    tr = info["transitions"]
    assert len(tr) == 2
    assert set(tr[0]) == {"t_start", "window_s", "leg_s", "total_path_length_m",
                          "synchronous_conflicting_pairs"}
    assert tr[0]["t_start"] == 2.0 and tr[0]["window_s"] == pytest.approx(L1)
    assert tr[0]["leg_s"] == pytest.approx(L1)
    assert tr[0]["total_path_length_m"] == pytest.approx(160.0)
    assert tr[0]["synchronous_conflicting_pairs"] == 0
    assert tr[1]["t_start"] == pytest.approx(2.0 + L1 + 1.5)
    assert tr[1]["window_s"] == pytest.approx(L2) and tr[1]["total_path_length_m"] == 160.0
    assert info["t_end"] == pytest.approx(2.0 + L1 + 1.5 + L2 + 4.0)
    assert fleet.t_end == pytest.approx(info["t_end"])
    assert fleet.meta == {"planner": "baseline(assign=identity,waves=1,stagger_s=0.0,sync=True)",
                          "show": "tiny", "seed": 5}
    # every drone: hold, leg to stage 1, hold, leg to stage 2, hold
    t = fleet.trajectories[2]
    assert len(legs_of(t)) == 2
    assert np.allclose(t.pos(0.0), [20, 0, 20]) and np.allclose(t.pos(1.9), [20, 0, 20])
    assert np.allclose(t.pos(2.0 + L1), [20, 40, 20]) and np.allclose(t.pos(2.0 + L1 + 1.5),
                                                                       [20, 40, 20])
    assert np.allclose(t.pos(info["t_end"]), [20, 40, 60])


def test_baseline_single_window_defaults_and_conflict_count(monkeypatch):
    # first stage without a hold key starts at t = 0; swap pair conflicts
    install_targets(monkeypatch, B, [[[0.0, 0, 20], [10.0, 0, 20]], [[10.0, 0, 20], [0.0, 0, 20]]])
    show = {"name": "swap", "seed": 0, "n_drones": 2, "limits": LIM, "stages": [{}, {}]}
    fleet, info = B.plan_baseline(show, assign="identity")
    assert info["transitions"][0]["t_start"] == 0.0
    assert info["transitions"][0]["synchronous_conflicting_pairs"] == 1
    assert info["t_end"] == pytest.approx(info["transitions"][0]["window_s"])
    assert not verify(fleet, LIM).ok                 # the head-on swap really does collide
    # Hungarian assignment untangles it: nobody moves
    fleet2, info2 = B.plan_baseline(show)
    assert info2["transitions"][0]["total_path_length_m"] == 0.0
    assert info2["transitions"][0]["synchronous_conflicting_pairs"] == 0


def test_baseline_waves_stagger_and_windows(monkeypatch):
    install_targets(monkeypatch, B, [T0, T1])
    show = make_show([1.0, 0.0], n=4)
    fleet, info = B.plan_baseline(show, assign="identity", waves=2, stagger_s=3.0)
    T = min_leg_time(np.array([0.0, 40.0, 0.0]), LIM)
    tr = info["transitions"][0]
    assert tr["window_s"] == pytest.approx(T + 3.0) and tr["leg_s"] == pytest.approx(T)
    for i in range(4):
        (leg,) = legs_of(fleet.trajectories[i])
        assert leg.t0 == pytest.approx(1.0 + (i % 2) * 3.0)
        assert leg.t1 - leg.t0 == pytest.approx(T)
    assert info["t_end"] == pytest.approx(1.0 + T + 3.0)
    assert fleet.meta["planner"] == "baseline(assign=identity,waves=2,stagger_s=3.0,sync=True)"


def test_baseline_unsynchronised_legs_take_their_own_minimum_time(monkeypatch):
    t1 = [[0.0, 40.0, 20], [10.0, 10.0, 20], [20.0, 5.0, 20], [30.0, 40.0, 20]]
    install_targets(monkeypatch, B, [T0, t1])
    show = make_show([0.0, 0.0], n=4)
    fleet, info = B.plan_baseline(show, assign="identity", sync=False)
    own = [min_leg_time(np.array(t1[i]) - np.array(T0[i]), LIM) for i in range(4)]
    for i in range(4):
        (leg,) = legs_of(fleet.trajectories[i])
        assert leg.t1 - leg.t0 == pytest.approx(own[i])
    assert info["transitions"][0]["leg_s"] == pytest.approx(max(own))
    assert fleet.meta["planner"].endswith("sync=False)")
    sync_fleet, _ = B.plan_baseline(show, assign="identity", sync=True)
    (leg,) = legs_of(sync_fleet.trajectories[1])
    assert leg.t1 - leg.t0 == pytest.approx(max(own))


def test_baseline_stationary_drones_hold_for_the_whole_window_and_do_not_stop_the_loop(monkeypatch):
    t1 = [T0[0], [10.0, 40.0, 20], T0[2], [30.0, 40.0, 20]]           # drones 0 and 2 stay
    install_targets(monkeypatch, B, [T0, t1])
    show = make_show([1.0, 0.0], n=4)
    fleet, info = B.plan_baseline(show, assign="identity", waves=2, stagger_s=2.0)
    W_ = info["transitions"][0]["window_s"]
    for i in (0, 2):
        assert legs_of(fleet.trajectories[i]) == []
        inner = [s for s in holds_of(fleet.trajectories[i]) if s.t0 == 1.0]
        assert len(inner) == 1 and inner[0].t1 == pytest.approx(1.0 + W_)
    for i in (1, 3):
        assert len(legs_of(fleet.trajectories[i])) == 1
    # tiny displacement counts as stationary, a displacement of a millimetre does not
    t2 = [[0.0, 0.0 + 5e-10, 20], [10.0, 0.001, 20], T0[2], T0[3]]
    install_targets(monkeypatch, B, [T0, t2])
    fleet, _ = B.plan_baseline(show, assign="identity")
    assert legs_of(fleet.trajectories[0]) == [] and len(legs_of(fleet.trajectories[1])) == 1


def test_waves_timeline_info_and_metadata(monkeypatch):
    install_targets(monkeypatch, W, [T0, T1, T2])
    show = make_show([2.0, 1.5, 4.0], seed=9, n=4)
    fleet, info = W.plan_waves(show, assign="identity")
    L1 = min_leg_time(np.array([0.0, 40.0, 0.0]), LIM)
    L2 = min_leg_time(np.array([0.0, 0.0, 40.0]), LIM)
    assert set(info) == {"transitions", "t_end", "plan_runtime_s"}
    assert 0.0 <= info["plan_runtime_s"] < 60.0
    t0, t1 = info["transitions"]
    assert set(t0) == {"t_start", "window_s", "waves", "wave_s", "total_path_length_m",
                       "synchronous_conflicting_pairs"}
    assert t0["t_start"] == 2.0 and t0["waves"] == 1 and t0["wave_s"] == [pytest.approx(L1)]
    assert t0["window_s"] == pytest.approx(L1)
    assert t0["total_path_length_m"] == pytest.approx(160.0)
    assert t0["synchronous_conflicting_pairs"] == 0
    assert t1["t_start"] == pytest.approx(2.0 + L1 + 1.5) and t1["window_s"] == pytest.approx(L2)
    assert info["t_end"] == pytest.approx(2.0 + L1 + 1.5 + L2 + 4.0)
    assert fleet.meta == {"planner": "waves(assign=identity)", "show": "tiny", "seed": 9}
    assert np.allclose(fleet.trajectories[3].pos(fleet.t_end), [30, 40, 60])
    assert np.allclose(fleet.trajectories[3].pos(0.0), [30, 0, 20])


def test_waves_splits_a_crossing_pair_and_holds_waiters(monkeypatch):
    # A crosses B's path: two waves; the drone that waits holds still until its wave
    tg0 = [[0.0, 0, 50], [5.0, -5.0, 50]]
    tg1 = [[10.0, 0, 50], [5.0, 5.0, 50]]
    install_targets(monkeypatch, W, [tg0, tg1])
    show = make_show([0.0, 0.0], n=2)
    fleet, info = W.plan_waves(show, assign="identity")
    tr = info["transitions"][0]
    assert tr["waves"] == 2 and len(tr["wave_s"]) == 2
    TA = min_leg_time(np.array([10.0, 0, 0]), LIM)
    TB = min_leg_time(np.array([0.0, 10.0, 0]), LIM)
    # schedule_waves puts B (index 1) in wave 0, A in wave 1 (see the scheduler test)
    assert tr["wave_s"] == [pytest.approx(TB), pytest.approx(TA)]
    assert tr["window_s"] == pytest.approx(TA + TB)
    assert tr["synchronous_conflicting_pairs"] == 1
    (la,) = legs_of(fleet.trajectories[0])
    (lb,) = legs_of(fleet.trajectories[1])
    assert lb.t0 == 0.0 and lb.t1 == pytest.approx(TB)
    assert la.t0 == pytest.approx(TB) and la.t1 == pytest.approx(TB + TA)
    # the waiting drone holds its start position until its wave begins
    assert np.allclose(fleet.trajectories[0].pos(TB * 0.5), [0, 0, 50])
    assert np.allclose(fleet.trajectories[1].pos(TB + TA), [5, 5, 50])
    assert verify(fleet, lim_with()).ok


def test_waves_stationary_drones_get_a_hold_and_later_drones_are_still_planned(monkeypatch):
    t1 = [T0[0], [10.0, 40.0, 20], T0[2], [30.0, 40.0, 20]]
    install_targets(monkeypatch, W, [T0, t1])
    show = make_show([1.0, 0.0], n=4)
    fleet, info = W.plan_waves(show, assign="identity")
    for i in (1, 3):
        assert len(legs_of(fleet.trajectories[i])) == 1
    for i in (0, 2):
        assert legs_of(fleet.trajectories[i]) == []
        inner = [s for s in holds_of(fleet.trajectories[i]) if s.t0 == 1.0]
        assert len(inner) == 1 and inner[0].t1 == pytest.approx(1.0 + info["transitions"][0]["window_s"])
    # the contiguity check at the end of the plan: every drone covers [0, t_end] without gaps
    for tr in fleet.trajectories:
        for s0, s1 in zip(tr.segments[:-1], tr.segments[1:]):
            assert s1.t0 == pytest.approx(s0.t1) and s1.t1 > s1.t0


def test_waves_all_stationary_transition_gets_a_minimal_window(monkeypatch):
    install_targets(monkeypatch, W, [T0, T0])
    show = make_show([0.0, 0.0], n=4)
    fleet, info = W.plan_waves(show, assign="identity")
    assert info["transitions"][0]["window_s"] == 0.0 and info["transitions"][0]["waves"] == 1
    for tr in fleet.trajectories:
        assert all(s.t1 > s.t0 for s in tr.segments)
        assert len(tr.segments) >= 1


def test_waves_margin_enters_the_planning_distance(monkeypatch):
    # B crosses A's path at a synchronous closest approach of 2.871/sqrt(2) = 2.03 m: legal for a
    # 2.0 m minimum separation, but not with the default 0.05 m planning margin
    tg0 = [[0.0, 0, 50], [7.871, -5.0, 50]]
    tg1 = [[10.0, 0, 50], [7.871, 5.0, 50]]
    sync = W._sync_min_dist(np.array(tg0), np.array(tg1))[0, 1]
    assert sync == pytest.approx(2.871 / math.sqrt(2), abs=1e-3) and 2.0 < sync < 2.05
    install_targets(monkeypatch, W, [tg0, tg1])
    show = make_show([0.0, 0.0], n=2)
    fleet, info = W.plan_waves(show, assign="identity")
    assert info["transitions"][0]["waves"] == 2
    assert verify(fleet, LIM).ok
    _, info = W.plan_waves(show, assign="identity", margin=0.0)
    assert info["transitions"][0]["waves"] == 1
    _, info = W.plan_waves(show, assign="identity", margin=-0.5)
    assert info["transitions"][0]["waves"] == 1


# ---------------------------------------------------------------------------------------------
# planner.prioritized helpers
# ---------------------------------------------------------------------------------------------
def quintic_s(u):
    return 10 * u**3 - 15 * u**4 + 6 * u**5


def test_sample_path_follows_the_quintic_profile_with_delay_and_holds():
    wps = [np.array([0.0, 0, 0]), np.array([10.0, 0, 0]), np.array([10.0, 20.0, 0])]
    times = np.array([0.0, 0.5, 1.0, 2.0, 3.0, 5.0, 7.0, 8.0, 12.0])
    out = P._sample_path(wps, [2.0, 4.0], 1.0, times)
    s = quintic_s
    expected = [
        [0, 0, 0], [0, 0, 0],                       # before the delay: at the start
        [0, 0, 0],                                  # t == delay: u = 0
        [10 * s(0.5), 0, 0],                        # middle of leg 1
        [10, 0, 0],                                 # t == end of leg 1 == start of leg 2
        [10, 20 * s(0.5), 0],                       # middle of leg 2
        [10, 20, 0], [10, 20, 0], [10, 20, 0],      # at/after the end: final waypoint
    ]
    assert np.allclose(out, expected, atol=1e-12)
    q = P._sample_path(wps, [2.0, 4.0], 1.0, np.array([1.5]))
    assert np.allclose(q, [[10 * s(0.25), 0, 0]], atol=1e-12)
    q = P._sample_path(wps, [2.0, 4.0], 1.0, np.array([4.0]))
    assert np.allclose(q, [[10, 20 * s(0.25), 0]], atol=1e-12)


def test_sample_path_skips_zero_length_legs_and_keeps_going():
    wps = [np.array([0.0, 0, 0]), np.array([0.0, 0, 0]), np.array([6.0, 0, 0])]
    out = P._sample_path(wps, [0.0, 2.0], 0.0, np.array([0.0, 1.0, 2.0]))
    assert np.allclose(out, [[0, 0, 0], [6 * quintic_s(0.5), 0, 0], [6, 0, 0]])
    short = P._sample_path([np.array([0.0, 0, 0]), np.array([4.0, 0, 0])], [0.5], 0.0,
                           np.array([0.25]))
    assert np.allclose(short, [[4 * quintic_s(0.5), 0, 0]])
    # a single waypoint path (a hold) stays where it is
    hold = P._sample_path([np.array([1.0, 2.0, 3.0])], [], 0.0, np.array([0.0, 5.0]))
    assert np.allclose(hold, [[1, 2, 3], [1, 2, 3]])


def routes_of(src, dst, lane_dz=(), lane_dh=(), lim=None):
    return P._routes(np.array(src, float), np.array(dst, float), lim or LIM, lane_dz, lane_dh)


def test_routes_direct_first_then_vertical_then_sideways_lanes():
    r = routes_of([-5.0, 0, 50], [5.0, 0, 50], lane_dz=(6.0, 12.0), lane_dh=(3.0,))
    assert len(r) == 1 + 4 + 2
    assert np.array_equal(r[0][0], [-5, 0, 50]) and np.array_equal(r[0][1], [5, 0, 50]) and len(r[0]) == 2
    vias = [x[1] for x in r[1:]]
    expected = [[0, 0, 56], [0, 0, 44], [0, 0, 62], [0, 0, 38], [0, 3, 50], [0, -3, 50]]
    assert np.allclose(vias, expected)
    for x in r[1:]:
        assert np.array_equal(x[0], [-5, 0, 50]) and np.array_equal(x[2], [5, 0, 50])


def test_routes_sideways_lane_is_perpendicular_and_unit_length():
    r = routes_of([0.0, 0, 50], [3.0, 4.0, 50], lane_dh=(10.0,))
    vias = np.array([x[1] for x in r[1:]])
    mid = np.array([1.5, 2.0, 50.0])
    assert np.allclose(vias[0], mid + 10 * np.array([-4.0, 3.0, 0.0]) / 5.0)
    assert np.allclose(vias[1], mid - 10 * np.array([-4.0, 3.0, 0.0]) / 5.0)
    # a rising path still gets a purely horizontal perpendicular
    r = routes_of([0.0, 0, 40], [10.0, 0, 60], lane_dh=(4.0,))
    assert np.allclose([x[1] for x in r[1:]], [[5, 4, 50], [5, -4, 50]])


@pytest.mark.parametrize("dst,expected_dir", [
    ([0.5, 0.0, 50.0], [1.0, 0.0, 0.0]),     # horizontal length exactly 0.5: fallback direction
    ([0.4, 0.0, 50.0], [1.0, 0.0, 0.0]),
    ([0.0, 0.0, 60.0], [1.0, 0.0, 0.0]),     # vertical leg: fallback direction
    ([0.0, 1.0, 50.0], [-1.0, 0.0, 0.0]),    # 1 m along y: perpendicular is -x (unit)
    ([0.0, 0.6, 50.0], [-1.0, 0.0, 0.0]),    # just above the 0.5 threshold: normalised
])
def test_routes_perpendicular_fallback_threshold(dst, expected_dir):
    r = routes_of([0.0, 0.0, 50.0], dst, lane_dh=(2.0,))
    mid = (np.array([0.0, 0.0, 50.0]) + np.array(dst)) / 2
    assert np.allclose(r[1][1], mid + 2.0 * np.array(expected_dir))
    assert np.allclose(r[2][1], mid - 2.0 * np.array(expected_dir))


def test_routes_filter_by_altitude_and_geofence_with_inclusive_boundaries():
    lim = lim_with(z_min=0.0, z_max=100.0, geofence={"x": [-50.0, 50.0], "y": [-30.0, 20.0]})
    # vertical lanes at mid z = 50: z = 99.5 and 0.5 are the extreme allowed values
    r = routes_of([0, 0, 50], [0, 0, 50], lane_dz=(49.5,), lim=lim)
    assert [x[1][2] for x in r[1:]] == [99.5, 0.5]
    r = routes_of([0, 0, 50], [0, 0, 50], lane_dz=(49.75,), lim=lim)
    assert len(r) == 1                                       # 99.75 and 0.25 are both outside
    r = routes_of([0, 0, 50], [0, 0, 50], lane_dz=(50.0,), lim=lim)
    assert len(r) == 1
    # sideways lanes with a zero-length path use the x axis: x = +-50 are on the fence
    r = routes_of([0, 0, 50], [0, 0, 50], lane_dh=(50.0,), lim=lim)
    assert [x[1][0] for x in r[1:]] == [50.0, -50.0]
    r = routes_of([0, 0, 50], [0, 0, 50], lane_dh=(50.5,), lim=lim)
    assert len(r) == 1
    # lanes along y use the y range [-30, 20]: +30 is outside, -30 is exactly on the fence
    r = routes_of([-5.0, 0, 50], [5.0, 0, 50], lane_dh=(30.0,), lim=lim)
    assert [x[1][1] for x in r[1:]] == [-30.0]
    r = routes_of([-5.0, 0, 50], [5.0, 0, 50], lane_dh=(20.0,), lim=lim)
    assert [x[1][1] for x in r[1:]] == [20.0, -20.0]
    r = routes_of([-5.0, 0, 50], [5.0, 0, 50], lane_dh=(30.5,), lim=lim)
    assert len(r) == 1
    # a lane that is inside in x but outside in y is rejected (and vice versa)
    r = routes_of([-5.0, 0, 50], [5.0, 0, 50], lane_dh=(25.0,), lim=lim)
    assert [x[1][1] for x in r[1:]] == [-25.0]
    # z_min offset: lowest allowed via altitude is z_min + 0.5
    lim2 = lim_with(z_min=10.0, z_max=100.0, geofence={"x": [-50.0, 50.0], "y": [-30.0, 20.0]})
    r = routes_of([0, 0, 40], [0, 0, 40], lane_dz=(29.5,), lim=lim2)
    assert [x[1][2] for x in r[1:]] == [69.5, 10.5]


# ---------------------------------------------------------------------------------------------
# prioritized plans
# ---------------------------------------------------------------------------------------------
def test_prioritized_free_flight_keeps_the_synchronous_direct_legs(monkeypatch):
    install_targets(monkeypatch, P, [T0, T1])
    show = make_show([2.0, 1.0], seed=3, n=4)
    fleet, info = P.plan_prioritized(show, assign="identity")
    T = min_leg_time(np.array([0.0, 40.0, 0.0]), LIM)
    (tr,) = info["transitions"]
    assert set(tr) == {"t_start", "window_s", "planned_window_s", "direct_min_s", "promotions",
                       "widenings", "detoured_drones", "total_path_length_m",
                       "synchronous_conflicting_pairs"}
    assert tr["t_start"] == 2.0 and tr["direct_min_s"] == pytest.approx(T)
    assert tr["planned_window_s"] == pytest.approx(1.5 * T + 2.0)
    assert tr["window_s"] == pytest.approx(T)
    assert tr["promotions"] == 0 and tr["widenings"] == 0 and tr["detoured_drones"] == 0
    assert tr["total_path_length_m"] == pytest.approx(160.0)
    assert tr["synchronous_conflicting_pairs"] == 0
    assert info["t_end"] == pytest.approx(2.0 + T + 1.0)
    assert set(info) == {"transitions", "t_end", "plan_runtime_s"} and info["plan_runtime_s"] >= 0
    assert fleet.meta == {"planner": "prioritized(assign=identity)", "show": "tiny", "seed": 3}
    for i in range(4):
        (leg,) = legs_of(fleet.trajectories[i])
        assert leg.t0 == 2.0 and leg.t1 == pytest.approx(2.0 + T)
        assert np.allclose(fleet.trajectories[i].pos(fleet.t_end), T1[i])
    assert verify(fleet, LIM).ok


def test_prioritized_stationary_drone_holds_through_the_window(monkeypatch):
    t1 = [T0[0], [10.0, 40.0, 20], T0[2], [30.0, 40.0, 20]]
    install_targets(monkeypatch, P, [T0, t1])
    show = make_show([1.0, 0.0], n=4)
    fleet, info = P.plan_prioritized(show, assign="identity")
    W_ = info["transitions"][0]["window_s"]
    for i in (0, 2):
        assert legs_of(fleet.trajectories[i]) == []
        inner = [s for s in holds_of(fleet.trajectories[i]) if s.t0 == 1.0]
        assert len(inner) == 1 and inner[0].t1 == pytest.approx(1.0 + W_)
    assert verify(fleet, LIM).ok


def test_prioritized_head_on_swap_is_resolved_by_a_detour(monkeypatch):
    tg0 = [[-10.0, 0, 50], [10.0, 0, 50]]
    tg1 = [[10.0, 0, 50], [-10.0, 0, 50]]
    install_targets(monkeypatch, P, [tg0, tg1])
    show = make_show([0.0, 0.0], n=2)
    fleet, info = P.plan_prioritized(show, assign="identity")
    (tr,) = info["transitions"]
    assert tr["detoured_drones"] == 1
    assert tr["synchronous_conflicting_pairs"] == 1
    rep = verify(fleet, LIM)
    assert rep.ok, rep.violation_counts
    assert rep.min_separation >= LIM["min_separation_m"]
    # one drone flies the direct leg, the other a two-leg detour of the planned lane size
    n_legs = sorted(len(legs_of(t)) for t in fleet.trajectories)
    assert n_legs == [1, 2]
    d = [t for t in fleet.trajectories if len(legs_of(t)) == 2][0]
    via = legs_of(d)[0].end()
    assert abs(via[0]) < 1e-9
    assert (min(abs(abs(via[2] - 50) - h) for h in (6.0, 12.0, 18.0)) < 1e-9 and abs(via[1]) < 1e-9
            ) or (min(abs(abs(via[1]) - h) for h in (6.0, 12.0)) < 1e-9 and abs(via[2] - 50) < 1e-9)
    assert np.allclose([t.pos(fleet.t_end) for t in fleet.trajectories], tg1)


def test_prioritized_detour_chosen_for_the_lower_priority_drone(monkeypatch):
    # priority = longest straight path first: the 40 m drone keeps its direct leg, the 20 m one
    # detours
    tg0 = [[-20.0, 0, 50], [10.0, 0, 50]]
    tg1 = [[20.0, 0, 50], [-10.0, 0, 50]]
    install_targets(monkeypatch, P, [tg0, tg1])
    show = make_show([0.0, 0.0], n=2)
    fleet, info = P.plan_prioritized(show, assign="identity")
    assert len(legs_of(fleet.trajectories[0])) == 1
    assert len(legs_of(fleet.trajectories[1])) == 2
    assert verify(fleet, LIM).ok
    # the plan is deterministic
    again, _ = P.plan_prioritized(show, assign="identity")
    assert again.content_hash() == fleet.content_hash()


def test_prioritized_unplannable_transition_raises(monkeypatch):
    # a drone that never moves is parked on the straight path of another one inside a corridor
    # that leaves no room: z and fences leave no lane, delays cannot help
    tight = lim_with(z_min=49.0, z_max=51.0, geofence={"x": [-30.0, 30.0], "y": [-0.4, 0.4]})
    tg0 = [[-20.0, 0, 50], [0.0, 0, 50]]
    tg1 = [[20.0, 0, 50], [0.0, 0, 50]]
    install_targets(monkeypatch, P, [tg0, tg1])
    show = make_show([0.0, 0.0], n=2, lim=tight)
    with pytest.raises(RuntimeError, match="no feasible plan"):
        P.plan_prioritized(show, assign="identity", max_widen=1, promote_retries=1)


def test_prioritized_time_budget_is_enforced(monkeypatch):
    install_targets(monkeypatch, P, [T0, T1])
    show = make_show([0.0, 0.0], n=4)
    with pytest.raises(RuntimeError, match="planning time budget exceeded"):
        P.plan_prioritized(show, assign="identity", time_budget_s=-1.0)


# ---------------------------------------------------------------------------------------------
# Assignment hook shared by the three planners: which method gets a seed and which one.
# ---------------------------------------------------------------------------------------------
def _spy_methods(monkeypatch, module, calls):
    def make(name):
        def fn(*args):
            calls.append((name, args))
            return np.arange(len(args[0]))
        return fn
    monkeypatch.setattr(module, "METHODS", {n: make(n) for n in
                                            ("random", "pinned15", "greedy", "hungarian_len",
                                             "hungarian_sq", "identity")})


SEEDED = ("random", "pinned15")


@pytest.mark.parametrize("module,planner", [(B, "plan_baseline"), (W, "plan_waves"),
                                            (P, "plan_prioritized")])
@pytest.mark.parametrize("name", ["random", "pinned15", "greedy", "hungarian_len",
                                  "hungarian_sq", "identity"])
def test_planners_pass_source_target_and_seed_plus_transition_index(monkeypatch, module, planner,
                                                                    name):
    a = np.array(T0, float)
    b = a + [0.0, 40.0, 0.0]
    c = b + [0.0, 0.0, 40.0]
    install_targets(monkeypatch, module, [a, b, c])
    calls = []
    _spy_methods(monkeypatch, module, calls)
    show = make_show([0.0, 0.0, 0.0], seed=5, n=4)
    getattr(module, planner)(show, assign=name)
    assert [c_[0] for c_ in calls] == [name, name]
    first, second = (c_[1] for c_ in calls)
    assert np.array_equal(first[0], a) and np.array_equal(first[1], b)
    assert np.array_equal(second[0], b) and np.array_equal(second[1], c)
    if name in SEEDED:
        assert len(first) == 3 and len(second) == 3
        assert first[2] == 5 and second[2] == 6
    else:
        assert len(first) == 2 and len(second) == 2
