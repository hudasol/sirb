"""Tests added after the first mutation-testing run on ``sirb/verify.py`` (docs/MUTATION.md).

Each group below was written because a family of mutants survived the earlier suite: violation
records (kind, drones, time, value, limit, text), the exact tolerance boundary of every limit
family, the numeric report fields, the structural-check boundaries, piece construction (holds,
overlaps, excluded drones), the separation search radius and the prefilter. Expected numbers are
derived by hand from the polynomial (shown in the comments), never read back from the verifier.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

import sirb.verify as V
from sirb.trajectory import Fleet, Segment, Trajectory
from sirb.verify import pair_minima, pairwise_min_separation, verify


def seg(t0, t1, *rows):
    return Segment(float(t0), float(t1), np.array(rows, float))


def fleet(*drones):
    return Fleet([Trajectory(list(d)) for d in drones])


def const(p, t0=0.0, t1=10.0):
    return seg(t0, t1, list(p))


def of_kind(rep, kind):
    return [v for v in rep.violations if v.kind == kind]


# ---------------------------------------------------------------------------------------------
# 1. Violation records for every dynamic limit family.
#    Drone 1 holds p0 on [0, 1] and then flies one segment on [1, 3] (h = 2) whose coordinate is
#    a monomial a*tau^k, increasing (or decreasing) so the extremum is at the end, tau = 2,
#    s = 1, t = 3.  Drone 0 is parked far away inside every limit.
# ---------------------------------------------------------------------------------------------
def _mono(axis, power, a, p0=(0.0, 0.0, 0.0)):
    rows = [list(p0)] + [[0.0, 0.0, 0.0] for _ in range(power)]
    rows[power][axis] += a
    return rows


WIDE = {"x": [-100.0, 100.0], "y": [-100.0, 100.0]}
# name -> (rows of drone 1's flying segment, limits, park point of drone 0, expected tuple)
# expected = (kind, value, limit, detail)
CASES = {
    # x = 3 tau^2: dx/dtau = 6 tau -> 12 m/s at tau = 2
    "speed_h": (_mono(0, 2, 3.0), {"vmax_h": 10.0}, (0, 50, 0),
                ("speed_h", 12.0, 10.0, "segment 1: horizontal speed")),
    "speed_up": (_mono(2, 2, 3.0), {"vmax_up": 10.0}, (0, 50, 0),
                 ("speed_up", 12.0, 10.0, "segment 1: climb rate")),
    "speed_down": (_mono(2, 2, -3.0), {"vmax_down": 10.0}, (0, 50, 0),
                   ("speed_down", 12.0, 10.0, "segment 1: descent rate")),
    # x = tau^3: a = 6 tau -> 12 at tau = 2
    "accel": (_mono(0, 3, 1.0), {"amax": 10.0}, (0, 50, 0),
              ("accel", 12.0, 10.0, "segment 1: acceleration")),
    # x = tau^4: jerk = 24 tau -> 48 at tau = 2
    "jerk": (_mono(0, 4, 1.0), {"jmax": 40.0}, (0, 50, 0),
             ("jerk", 48.0, 40.0, "segment 1: jerk")),
    # z = 3 tau^2 -> 12 at tau = 2
    "alt_hi": (_mono(2, 2, 3.0), {"z_max": 10.0}, (0, 50, 0),
               ("altitude", 12.0, 10.0, "segment 1: altitude above z_max")),
    "alt_lo": (_mono(2, 2, -3.0), {"z_min": -10.0}, (0, 50, 0),
               ("altitude", -12.0, -10.0, "segment 1: altitude below z_min")),
    "geo_x_hi": (_mono(0, 2, 3.0), {"geofence": {"x": [-10.0, 10.0], "y": [-100.0, 100.0]}},
                 (0, 50, 0), ("geofence", 12.0, 10.0, "segment 1: x above geofence max")),
    "geo_x_lo": (_mono(0, 2, -3.0), {"geofence": {"x": [-10.0, 10.0], "y": [-100.0, 100.0]}},
                 (0, 50, 0), ("geofence", -12.0, -10.0, "segment 1: x below geofence min")),
    "geo_y_hi": (_mono(1, 2, 3.0), {"geofence": {"x": [-100.0, 100.0], "y": [-10.0, 10.0]}},
                 (50, 0, 0), ("geofence", 12.0, 10.0, "segment 1: y above geofence max")),
    "geo_y_lo": (_mono(1, 2, -3.0), {"geofence": {"x": [-100.0, 100.0], "y": [-10.0, 10.0]}},
                 (50, 0, 0), ("geofence", -12.0, -10.0, "segment 1: y below geofence min")),
    # y = 20 - 3 tau^2 reaches 8 m, the crowd needs y >= 0 + 10
    "crowd": (_mono(1, 2, -3.0, p0=(0.0, 20.0, 0.0)),
              {"crowd": {"y": 0.0, "min_distance_m": 10.0}}, (0, 50, 0),
              ("crowd", 8.0, 10.0, "segment 1: too close to crowd region")),
}


def _case_fleet(name):
    rows, lim, park, _ = CASES[name]
    p0 = tuple(rows[0])
    return fleet([const(park, 0.0, 3.0)], [seg(0.0, 1.0, list(p0)), seg(1.0, 3.0, *rows)]), \
        {"min_separation_m": 1.0, **lim}


@pytest.mark.parametrize("name", sorted(CASES))
def test_violation_record_fields_and_text(name):
    f, lim = _case_fleet(name)
    kind, value, limit, detail = CASES[name][3]
    rep = verify(f, lim)
    got = of_kind(rep, kind)
    assert len(got) == 1, [(v.kind, v.detail) for v in rep.violations]
    v = got[0]
    assert v.drones == (1,)
    assert v.t == pytest.approx(3.0, abs=1e-9)
    assert v.value == pytest.approx(value, abs=1e-9)
    assert v.limit == limit
    assert v.detail == detail
    assert rep.violation_counts[kind] == 1 and not rep.ok


# Tolerance boundary.  tol = 1 makes the threshold 2*L + 1 for L >= 0, so a value of exactly
# 2*L + 1 is allowed and anything above is flagged.  At the boundary each value below is exact in
# binary floating point (sqrt of a perfect square, integer coefficients).
BOUNDARY = {
    # name: (rows builder(a), limits, kind, exact at-threshold coefficient a_edge)
    "speed_h": (lambda a: _mono(0, 1, a), {"vmax_h": 2.0}, "speed_h", 5.0),            # 5 = 2*2+1
    "speed_up": (lambda a: _mono(2, 1, a), {"vmax_up": 2.0}, "speed_up", 5.0),
    "speed_down": (lambda a: _mono(2, 1, -a), {"vmax_down": 2.0}, "speed_down", 5.0),
    "accel": (lambda a: _mono(0, 2, a / 2), {"amax": 0.5}, "accel", 2.0),             # 2 = 2*.5+1
    "jerk": (lambda a: _mono(0, 3, a / 6), {"jmax": 2.5}, "jerk", 6.0),               # 6 = 2*2.5+1
    "alt_hi": (lambda a: _mono(2, 0, a), {"z_max": 2.0}, "altitude", 5.0),
    "alt_lo": (lambda a: _mono(2, 0, -a), {"z_min": 0.0}, "altitude", 1.0),           # -1 = 0-1
    "geo_x_hi": (lambda a: _mono(0, 0, a), {"geofence": {"x": [0.0, 2.0], "y": [-9.0, 9.0]}},
                 "geofence", 5.0),
    "geo_x_lo": (lambda a: _mono(0, 0, -a), {"geofence": {"x": [0.0, 20.0], "y": [-9.0, 9.0]}},
                 "geofence", 1.0),
    "geo_y_hi": (lambda a: _mono(1, 0, a), {"geofence": {"x": [-9.0, 9.0], "y": [0.0, 2.0]}},
                 "geofence", 5.0),
    "geo_y_lo": (lambda a: _mono(1, 0, -a), {"geofence": {"x": [-9.0, 9.0], "y": [0.0, 20.0]}},
                 "geofence", 1.0),
    "crowd": (lambda a: _mono(1, 0, -a), {"crowd": {"y": -9.0, "min_distance_m": 9.0}},
              "crowd", 1.0),                                                         # need 0 -> -1
    # lower limits that are not zero: the threshold is limit - tol*(1+|limit|) = 2 - 3 = -1
    "alt_lo_nonzero": (lambda a: _mono(2, 0, -a), {"z_min": 2.0}, "altitude", 1.0),
    "geo_x_lo_nonzero": (lambda a: _mono(0, 0, -a),
                         {"geofence": {"x": [2.0, 20.0], "y": [-9.0, 9.0]}}, "geofence", 1.0),
    "geo_y_lo_nonzero": (lambda a: _mono(1, 0, -a),
                         {"geofence": {"x": [-9.0, 9.0], "y": [2.0, 20.0]}}, "geofence", 1.0),
    "crowd_nonzero": (lambda a: _mono(1, 0, -a), {"crowd": {"y": -7.0, "min_distance_m": 9.0}},
                      "crowd", 1.0),                                                 # need 2 -> -1
}


@pytest.mark.parametrize("name", sorted(BOUNDARY))
def test_tolerance_boundary_is_strict_and_uses_one_plus_abs(name):
    build, lim, kind, edge = BOUNDARY[name]
    limits = {"min_separation_m": 1.0, **lim}

    def run(a):
        # constant-rate or constant-position single segment on [0, 1]
        rows = build(a)
        f = fleet([seg(0.0, 1.0, *rows)])
        return verify(f, limits, tol=1.0)

    assert not of_kind(run(edge), kind), "a value exactly at limit + tol*(1+|limit|) is allowed"
    assert of_kind(run(edge + 0.5), kind), "a value above the threshold is a violation"
    assert not of_kind(run(edge - 0.5), kind)


def test_dynamic_checks_use_the_default_tolerance_of_one_micro():
    # speed 10.000009 passes (limit 10 + 1e-6 * 11 = 10.000011); 10.00002 fails
    def rep(v):
        return verify(fleet([seg(0, 1, [0, 0, 0], [v, 0, 0])], [const((0, 50, 0), 0, 1)]),
                      {"min_separation_m": 1.0, "vmax_h": 10.0})
    assert not of_kind(rep(10.000009), "speed_h")
    assert of_kind(rep(10.00002), "speed_h")


def test_limits_absent_from_the_dict_are_skipped():
    f = fleet([seg(0, 1, [0, 0, 0], [500, 500, 500])], [const((0, 5000, 0), 0, 1)])
    rep = verify(f, {"min_separation_m": 1.0})
    assert rep.ok and rep.violations == [] and rep.violation_counts == {}
    assert math.isinf(rep.geofence_margin) and math.isinf(rep.crowd_margin)


# ---------------------------------------------------------------------------------------------
# 2. Report fields.
# ---------------------------------------------------------------------------------------------
def test_report_numeric_fields_match_hand_computation():
    # drone 0 on [2, 4] (h = 2):  p(tau) = (1 + tau^2, 2 + tau^3, 3 + tau^3 / 2)
    #   v = (2 tau, 3 tau^2, 1.5 tau^2): |v_h|max = sqrt(16 + 144), vz_max = 6, vz_min = 0
    #   a = (2, 6 tau, 3 tau):          |a|max = sqrt(4 + 144 + 36) at tau = 2
    #   j = (0, 6, 3):                  |j| = sqrt(45)
    #   z in [3, 7]
    d0 = [seg(2, 4, [1, 2, 3], [0, 0, 0], [1, 0, 0], [0, 1, 0.5])]
    d1 = [const((4, 15, 5), 0, 4)]
    lim = {"min_separation_m": 1.0, "geofence": {"x": [-2.0, 10.0], "y": [1.0, 20.0]},
           "crowd": {"y": -10.0, "min_distance_m": 2.0}}
    rep = verify(fleet(d0, d1), lim)
    assert rep.max_speed_h == pytest.approx(math.sqrt(160.0), abs=1e-9)
    assert rep.max_speed_up == pytest.approx(6.0, abs=1e-9)
    assert rep.max_speed_down == 0.0
    assert rep.max_accel == pytest.approx(math.sqrt(184.0), abs=1e-9)
    assert rep.max_jerk == pytest.approx(math.sqrt(45.0), abs=1e-9)
    assert rep.min_z == pytest.approx(3.0, abs=1e-9)
    assert rep.max_z == pytest.approx(7.0, abs=1e-9)
    # x in [1,5] and [4,4], y in [2,10] and [15,15]; margins: x lo 3, x hi 5, y lo 1, y hi 5
    assert rep.geofence_margin == pytest.approx(1.0, abs=1e-9)
    # crowd needs y >= -10 + 2 = -8; lowest y is 2 -> margin 10
    assert rep.crowd_margin == pytest.approx(10.0, abs=1e-9)
    assert rep.n_drones == 2 and rep.n_segments == 2
    assert rep.runtime_s >= 0.0 and rep.runtime_s < 120.0
    assert rep.ok and rep.violations == [] and rep.violation_counts == {}
    assert rep.violations_truncated is False


@pytest.mark.parametrize("pos,expected", [((1.0, 5.0), 1.0), ((8.5, 5.0), 1.5),
                                          ((5.0, 2.0), 2.0), ((5.0, 7.0), 3.0)])
def test_geofence_margin_is_the_tightest_side_on_either_axis(pos, expected):
    lim = {"min_separation_m": 1.0, "geofence": {"x": [0.0, 10.0], "y": [0.0, 10.0]}}
    rep = verify(fleet([const((pos[0], pos[1], 3.0), 0, 1)]), lim)
    assert rep.geofence_margin == pytest.approx(expected, abs=1e-12)


def test_down_and_up_speed_statistics_have_the_right_sign():
    # climbs at 2 m/s on [0,1], then descends at 5 m/s on [1,2]
    d = [seg(0, 1, [0, 0, 10], [0, 0, 2]), seg(1, 2, [0, 0, 12], [0, 0, -5])]
    rep = verify(fleet(d), {"min_separation_m": 1.0})
    assert rep.max_speed_up == pytest.approx(2.0, abs=1e-12)
    assert rep.max_speed_down == pytest.approx(5.0, abs=1e-12)
    assert rep.max_speed_h == 0.0 and rep.max_accel == 0.0 and rep.max_jerk == 0.0
    assert rep.min_z == pytest.approx(7.0, abs=1e-12) and rep.max_z == pytest.approx(12.0)


def test_speed_statistics_are_per_second_not_per_segment_parameter():
    # 10 m in 5 s: the normalised-time derivative is 10, the physical speed is 2
    rep = verify(fleet([seg(0, 5, [0, 0, 0], [2, 0, 0])]), {"min_separation_m": 1.0})
    assert rep.max_speed_h == pytest.approx(2.0, abs=1e-12)
    # accel 2*c2 = 2*0.5 = 1 m/s^2 and jerk 6*c3 = 6*0.125 = 0.75 m/s^3, on a 4 s segment
    rep = verify(fleet([seg(0, 4, [0, 0, 0], [0, 0, 0], [0.5, 0, 0], [0.125, 0, 0])]),
                 {"min_separation_m": 1.0})
    assert rep.max_jerk == pytest.approx(0.75, abs=1e-12)
    # accel(tau) = 2*0.5 + 6*0.125*tau = 1 + 0.75 tau, maximum 1 + 3 = 4 at tau = 4
    assert rep.max_accel == pytest.approx(4.0, abs=1e-12)


def test_horizontal_speed_ignores_vertical_and_accel_uses_all_three_axes():
    rep = verify(fleet([seg(0, 1, [0, 0, 0], [3, 4, 100])]), {"min_separation_m": 1.0})
    assert rep.max_speed_h == pytest.approx(5.0, abs=1e-12)           # x, y only
    assert rep.max_speed_up == pytest.approx(100.0, abs=1e-12)
    rep = verify(fleet([seg(0, 1, [0, 0, 0], [0, 0, 0], [1, 2, 2])]), {"min_separation_m": 1.0})
    assert rep.max_accel == pytest.approx(6.0, abs=1e-12)             # 2*|(1,2,2)| = 6, all axes
    rep = verify(fleet([seg(0, 1, [0, 0, 0], [0, 0, 0], [0, 0, 0], [1, 2, 2])]),
                 {"min_separation_m": 1.0})
    assert rep.max_jerk == pytest.approx(18.0, abs=1e-12)             # 6*|(1,2,2)| = 18


def test_empty_and_unusable_fleets_report_neutral_statistics():
    rep = verify(fleet([]), {"min_separation_m": 1.0, "geofence": {"x": [0, 1], "y": [0, 1]},
                            "crowd": {"y": 0.0, "min_distance_m": 1.0}})
    assert not rep.ok and rep.n_drones == 1 and rep.n_segments == 0
    assert rep.max_speed_h == 0.0 and rep.max_speed_up == 0.0 and rep.max_speed_down == 0.0
    assert rep.max_accel == 0.0 and rep.max_jerk == 0.0
    assert rep.min_z == math.inf and rep.max_z == -math.inf
    assert rep.geofence_margin == math.inf and rep.crowd_margin == math.inf
    assert rep.min_separation_is_lower_bound and rep.min_separation_pair is None
    assert rep.min_separation_time is None
    assert rep.n_pairs_checked == 0


def test_margins_are_infinite_without_the_corresponding_limit():
    rep = verify(fleet([const((0, 0, 3), 0, 1)]), {"min_separation_m": 1.0})
    assert rep.geofence_margin == math.inf and rep.crowd_margin == math.inf


def test_tolerances_record_and_counts():
    rep = verify(fleet([const((0, 0, 3), 0, 1)]), {"min_separation_m": 2.0}, tol=1e-3)
    t = rep.tolerances
    assert t["tol"] == 1e-3
    assert t["policy"] == "limit violated only if exceeded by more than tol*(1+|limit|)"
    assert t["search_radius"] == 8.0
    assert t["time_contiguity_s"] == 1e-9
    assert t["position_continuity_m"] == 1e-4
    assert t["velocity_continuity_mps"] == 1e-3
    assert t["acceleration_continuity_mps2"] == 1e-3
    assert t["violation_cap_per_kind"] == 1000
    assert set(t) == {"tol", "policy", "search_radius", "time_contiguity_s",
                      "position_continuity_m", "velocity_continuity_mps",
                      "acceleration_continuity_mps2", "violation_cap_per_kind"}


# ---------------------------------------------------------------------------------------------
# 3. Separation: records, boundary, search radius, choice of the closest pair.
# ---------------------------------------------------------------------------------------------
def test_separation_boundary_and_record():
    # min separation 4, tol 0.5: threshold 4 - 0.5 * 5 = 1.5.  Parked drones 1.5 apart are fine,
    # 1.4 apart are a violation with the documented text.
    def run(d):
        return verify(fleet([const((0, 0, 0))], [const((d, 0, 0))]), {"min_separation_m": 4.0},
                      tol=0.5)
    assert of_kind(run(1.5), "separation") == [] and run(1.5).ok
    rep = run(1.4)
    (v,) = of_kind(rep, "separation")
    assert v.drones == (0, 1) and v.value == pytest.approx(1.4, abs=1e-12) and v.limit == 4.0
    assert math.isfinite(v.t) and 0.0 <= v.t <= 10.0
    assert v.detail == "centre distance 1.4 m < 4.0 m"
    assert rep.min_separation == pytest.approx(1.4, abs=1e-12)
    assert rep.min_separation_pair == (0, 1) and not rep.min_separation_is_lower_bound
    assert rep.violation_counts == {"separation": 1}


def test_separation_threshold_scales_with_the_declared_minimum():
    # |limit| enters the tolerance: min_sep 100, tol 0.01 -> threshold 100 - 0.01 * 101 = 98.99
    def run(d):
        return verify(fleet([const((0, 0, 0))], [const((d, 0, 0))]), {"min_separation_m": 100.0},
                      tol=0.01)
    assert run(98.995).ok
    assert not run(98.98).ok


def test_search_radius_is_raised_above_the_separation_threshold():
    # asked for 0.1 but min_sep = 2 -> radius is raised to 1.01*2 + 1e-3 = 2.021
    lim = {"min_separation_m": 2.0}
    near = verify(fleet([const((0, 0, 0), 0, 5)], [const((2.0205, 0, 0), 0, 5)]), lim,
                  search_radius=0.1)
    assert near.tolerances["search_radius"] == pytest.approx(2.021, abs=1e-12)
    assert not near.min_separation_is_lower_bound
    assert near.min_separation == pytest.approx(2.0205, abs=1e-12)
    far = verify(fleet([const((0, 0, 0), 0, 5)], [const((2.025, 0, 0), 0, 5)]), lim,
                 search_radius=0.1)
    assert far.min_separation_is_lower_bound
    assert far.min_separation == pytest.approx(2.021, abs=1e-12)
    assert far.min_separation_pair is None and far.min_separation_time is None
    assert far.ok


def test_default_search_radius_is_four_times_the_separation():
    lim = {"min_separation_m": 2.0}
    r = verify(fleet([const((0, 0, 0), 0, 5)], [const((7.9, 0, 0), 0, 5)]), lim)
    assert r.tolerances["search_radius"] == 8.0 and not r.min_separation_is_lower_bound
    assert r.min_separation == pytest.approx(7.9, abs=1e-12)
    r = verify(fleet([const((0, 0, 0), 0, 5)], [const((8.1, 0, 0), 0, 5)]), lim)
    assert r.min_separation_is_lower_bound and r.min_separation == 8.0


def test_closest_pair_is_the_one_with_the_smallest_distance():
    # three parked drones: pairs (0,1) 5.0, (0,2) 3.0, (1,2) 4.0.  Lexicographic min would say
    # (0,1); smallest time would be arbitrary.
    f = fleet([const((0, 0, 0), 0, 5)], [const((5, 0, 0), 0, 5)], [const((0, 3, 0), 0, 5)])
    d, i, j, t, lb, n = pairwise_min_separation(f, 100.0)
    assert (i, j) == (0, 2) and d == pytest.approx(3.0, abs=1e-12) and not lb and n == 3
    rep = verify(f, {"min_separation_m": 2.0}, search_radius=100.0)
    assert rep.min_separation_pair == (0, 2) and rep.min_separation == pytest.approx(3.0)
    assert rep.n_pairs_checked == 3
    # the reported time is the time of the closest approach of that pair, not of another one
    g = fleet([seg(0, 10, [0, 0, 0], [1, 0, 0])], [seg(0, 10, [10, 0, 0], [-1, 0, 0])],
              [const((0, 50, 0), 0, 10)])
    d, i, j, t, lb, _ = pairwise_min_separation(g, 100.0)
    assert (i, j) == (0, 1) and d == pytest.approx(0.0, abs=1e-9)
    assert t == pytest.approx(5.0, abs=1e-9)
    res, _ = pair_minima(g, 100.0)
    assert res[(0, 1)][0] == pytest.approx(0.0, abs=1e-9)
    assert res[(0, 1)][1] == pytest.approx(5.0, abs=1e-9)


def test_pair_keys_do_not_collide_for_large_indices():
    # drones 1 and 4 vs 2 and 3 would share a key with a wrong multiplier; use 6 drones on a line
    pts = [(0, 0, 0), (10, 0, 0), (20, 0, 0), (30, 0, 0), (40, 0, 0), (45, 0, 0)]
    f = fleet(*[[const(p, 0, 5)] for p in pts])
    res, n = pair_minima(f, 12.0)
    assert set(res) == {(0, 1), (1, 2), (2, 3), (3, 4), (4, 5)}
    assert res[(4, 5)][0] == pytest.approx(5.0) and res[(0, 1)][0] == pytest.approx(10.0)
    assert n == 5


def test_pair_minima_is_strictly_below_the_radius_and_reports_pair_count():
    f = fleet([const((0, 0, 0), 0, 5)], [const((3, 0, 0), 0, 5)])
    res, n = pair_minima(f, 3.0)               # distance == radius -> not reported
    assert res == {} and n == 1
    res, n = pair_minima(f, 3.0000001)
    assert set(res) == {(0, 1)} and n == 1


def test_pair_minima_empty_inputs():
    assert pair_minima(fleet(), 5.0) == ({}, 0)
    assert pair_minima(fleet([]), 5.0) == ({}, 0)           # only an unusable drone
    one = fleet([const((0, 0, 0), 0, 5)])
    assert pair_minima(one, 5.0) == ({}, 0)                 # a single drone has no pair
    assert pairwise_min_separation(one, 5.0) == (5.0, None, None, None, True, 0)
    far = fleet([const((0, 0, 0), 0, 5)], [const((100, 0, 0), 0, 5)])
    assert pair_minima(far, 5.0) == ({}, 0)                 # prefilter drops it: nothing examined


def test_prefilter_discards_provably_distant_pieces_and_keeps_near_ones():
    far = fleet([seg(0, 5, [0, 0, 0], [1, 0, 0])], [seg(0, 5, [100, 100, 0], [1, 0, 0])])
    near = fleet([seg(0, 5, [0, 0, 0], [1, 0, 0])], [seg(0, 5, [0, 4.5, 0], [1, 0, 0])])
    assert pair_minima(far, 5.0)[1] == 0
    assert pair_minima(far, 5.0, prefilter=False)[1] == 1
    assert pair_minima(near, 5.0)[1] == 1
    res, _ = pair_minima(far, 5.0, prefilter=False)
    assert res == {}                                         # all pairs examined, none below 5
    # brute force and prefilter agree on distances
    a, _ = pair_minima(near, 5.0)
    b, _ = pair_minima(near, 5.0, prefilter=False)
    assert a.keys() == b.keys() and a[(0, 1)][0] == pytest.approx(b[(0, 1)][0], abs=1e-12)


def test_prefilter_radius_has_a_tiny_slack_but_is_not_loose():
    # boxes exactly R apart are kept (slack 1e-9), boxes 1e-6 beyond R are not
    base = [seg(0, 5, [0, 0, 0])]
    on = fleet(base, [seg(0, 5, [5.0, 0, 0])])
    off = fleet(base, [seg(0, 5, [5.000001, 0, 0])])
    assert pair_minima(on, 5.0)[1] == 1
    assert pair_minima(off, 5.0)[1] == 0
    just_inside = fleet(base, [seg(0, 5, [5.0 + 0.5e-9, 0, 0])])
    assert pair_minima(just_inside, 5.0)[1] == 1


def test_far_parked_drone_is_not_pruned_wrongly():
    # drone 0 sits at (30,0,0) until t=5 (its first segment starts later); drone 1 flies by at
    # distance 0.5 from it before that.  The held "before first segment" piece must have a box at
    # its true position, otherwise the prefilter would discard the pair.
    d0 = [seg(5, 6, [30, 0, 0])]
    d1 = [seg(0, 4, [30.5, -3, 0], [0, 1.5, 0])]
    f = fleet(d0, d1)
    res, n = pair_minima(f, 4.0)
    assert set(res) == {(0, 1)} and res[(0, 1)][0] == pytest.approx(0.5, abs=1e-9)
    assert res[(0, 1)][1] == pytest.approx(2.0, abs=1e-9)
    rep = verify(f, {"min_separation_m": 1.0})
    assert [v.kind for v in rep.violations] == ["separation"]


def test_real_pieces_get_exact_boxes_far_from_the_origin():
    # two drones moving 1 m/s parallel at 3 m apart, 1000 m from the origin: boxes must be real
    f = fleet([seg(0, 5, [1000, 1000, 0], [1, 0, 0])], [seg(0, 5, [1000, 1003, 0], [1, 0, 0])])
    assert pair_minima(f, 5.0)[1] == 1
    # and a distant mover 500 m away from the origin box is rejected (a box left at the origin
    # would wrongly keep it)
    g = fleet([seg(0, 5, [1000, 1000, 0], [1, 0, 0])], [seg(0, 5, [1000, 1100, 0], [1, 0, 0])])
    assert pair_minima(g, 5.0)[1] == 0


# ---------------------------------------------------------------------------------------------
# 4. Ordering, caps and counts of the violation list.
# ---------------------------------------------------------------------------------------------
def _speeders(speeds):
    drones = [[seg(0, 1, [0, 100.0 * k, 0], [s, 0, 0])] for k, s in enumerate(speeds)]
    return fleet(*drones)


def test_violations_sorted_worst_first_within_a_kind_and_by_kind_order():
    rep = verify(_speeders([11.0, 15.0, 12.0]), {"min_separation_m": 1.0, "vmax_h": 10.0})
    got = [(v.drones[0], v.value) for v in of_kind(rep, "speed_h")]
    assert got == [(1, pytest.approx(15.0)), (2, pytest.approx(12.0)), (0, pytest.approx(11.0))]
    # kind order: separation, speed_h, ..., then structure kinds
    f = fleet([seg(0, 1, [0, 0, 0], [20, 0, 0])], [seg(0, 1, [0, 0.5, 0], [20, 0, 0])],
              [seg(0, 1, [0, 900, 0]), seg(2, 3, [0, 900, 0])])
    rep = verify(f, {"min_separation_m": 2.0, "vmax_h": 10.0})
    kinds_in_order = [v.kind for v in rep.violations]
    assert kinds_in_order == ["separation", "speed_h", "speed_h", "time_gap"]


def test_separation_violations_are_listed_closest_first():
    f = fleet([const((0, 0, 0), 0, 5)], [const((1.5, 0, 0), 0, 5)], [const((0, 100, 0), 0, 5)],
              [const((0, 100.5, 0), 0, 5)])
    rep = verify(f, {"min_separation_m": 2.0})
    vals = [v.value for v in of_kind(rep, "separation")]
    assert vals == sorted(vals) and len(vals) == 2
    assert of_kind(rep, "separation")[0].drones == (2, 3)


def test_nonfinite_values_do_not_break_sorting():
    f = fleet([seg(0, 1, [float("nan"), 0, 0])], [seg(0, 1, [0, 0, float("inf")])],
              [seg(0, 1, [0, 500, 0], [30, 0, 0])])
    rep = verify(f, {"min_separation_m": 1.0, "vmax_h": 10.0})
    assert rep.violation_counts == {"speed_h": 1, "nonfinite": 2}
    assert [v.kind for v in rep.violations] == ["speed_h", "nonfinite", "nonfinite"]
    assert [v.drones for v in of_kind(rep, "nonfinite")] == [(0,), (1,)]


def test_violation_cap_truncates_per_kind_and_keeps_true_counts(monkeypatch):
    monkeypatch.setattr(V, "VIOLATION_CAP", 3)
    rep = verify(_speeders([11.0, 15.0, 12.0, 13.0, 14.0]),
                 {"min_separation_m": 1.0, "vmax_h": 10.0})
    assert rep.violation_counts == {"speed_h": 5}
    assert rep.violations_truncated is True and not rep.ok
    assert [v.drones[0] for v in rep.violations] == [1, 4, 3]            # 15, 14, 13
    assert rep.tolerances["violation_cap_per_kind"] == 3
    exact = verify(_speeders([11.0, 15.0, 12.0]), {"min_separation_m": 1.0, "vmax_h": 10.0})
    assert exact.violations_truncated is False and len(exact.violations) == 3


def test_cap_applies_to_each_kind_separately(monkeypatch):
    monkeypatch.setattr(V, "VIOLATION_CAP", 2)
    drones = [[seg(0, 1, [0, 100.0 * k, 0], [20, 0, 50])] for k in range(3)]
    rep = verify(fleet(*drones), {"min_separation_m": 1.0, "vmax_h": 10.0, "vmax_up": 10.0})
    assert rep.violation_counts == {"speed_h": 3, "speed_up": 3}
    assert len(of_kind(rep, "speed_h")) == 2 and len(of_kind(rep, "speed_up")) == 2


def test_ok_uses_true_counts_not_the_truncated_list(monkeypatch):
    monkeypatch.setattr(V, "VIOLATION_CAP", 0)
    rep = verify(_speeders([11.0]), {"min_separation_m": 1.0, "vmax_h": 10.0})
    assert rep.violations == [] and rep.violation_counts == {"speed_h": 1} and not rep.ok
    assert rep.violations_truncated is True


# ---------------------------------------------------------------------------------------------
# 5. Structure: boundaries, records and text.
# ---------------------------------------------------------------------------------------------
def test_drone_without_segments_record():
    rep = verify(fleet([const((0, 500, 0), 0, 1)], []), {"min_separation_m": 1.0})
    (v,) = rep.violations
    assert (v.kind, v.drones, v.t, v.value, v.limit, v.detail) == (
        "time_gap", (1,), 0.0, 0.0, 0.0, "drone has no segments")


def test_gap_and_overlap_records_and_text():
    gap = fleet([const((0, 0, 0), 0, 1), const((0, 0, 0), 1.5, 2)])
    (v,) = of_kind(verify(gap, {"min_separation_m": 1.0}), "time_gap")
    assert (v.drones, v.t, v.value, v.limit) == ((0,), 1.0, 0.5, 1e-9)
    assert v.detail == "between segments 0 and 1: gap of 0.5 s"
    over = fleet([const((0, 0, 0), 0, 1), const((0, 0, 0), 0.75, 2)])
    (v,) = of_kind(verify(over, {"min_separation_m": 1.0}), "time_gap")
    assert (v.drones, v.t, v.value, v.limit) == ((0,), 1.0, -0.25, 1e-9)
    assert v.detail == "between segments 0 and 1: overlap of -0.25 s"


def test_gap_of_exactly_the_tolerance_is_allowed_and_tiny_overshoot_is_not():
    ok = fleet([const((0, 0, 0), -1.0, 0.0), const((0, 0, 0), 1e-9, 1.0)])
    assert of_kind(verify(ok, {"min_separation_m": 1.0}), "time_gap") == []
    bad = fleet([const((0, 0, 0), -1.0, 0.0), const((0, 0, 0), 2e-9, 1.0)])
    assert len(of_kind(verify(bad, {"min_separation_m": 1.0}), "time_gap")) == 1
    neg = fleet([const((0, 0, 0), -1.0, 0.0), const((0, 0, 0), -2e-9, 1.0)])
    (v,) = of_kind(verify(neg, {"min_separation_m": 1.0}), "time_gap")
    assert "overlap" in v.detail


def test_non_positive_duration_record():
    f = fleet([const((0, 0, 0), 0, 1), seg(1, 1, [0, 0, 0])])
    (v,) = of_kind(verify(f, {"min_separation_m": 1.0}), "time_gap")
    assert (v.drones, v.t, v.value, v.limit) == ((0,), 1.0, 0.0, 0.0)
    assert v.detail == "segment 1: non-positive duration"
    rev = fleet([seg(3, 1, [0, 0, 0])])
    (v,) = of_kind(verify(rev, {"min_separation_m": 1.0}), "time_gap")
    assert v.t == 3.0 and v.value == -2.0 and v.detail == "segment 0: non-positive duration"


@pytest.mark.parametrize("kind,rows,tol,detail", [
    ("position_jump", [[1e-4, 0, 0]], 1e-4, "jump of 0.0001 m"),
    ("velocity_discontinuity", [[0, 0, 0], [1e-3, 0, 0]], 1e-3,
     "velocity jump of 0.001 m/s (unbounded acceleration)"),
    ("accel_discontinuity", [[0, 0, 0], [0, 0, 0], [5e-4, 0, 0]], 1e-3,
     "acceleration jump of 0.001 m/s^2 (unbounded jerk)"),
])
def test_continuity_tolerances_are_inclusive_and_records_are_exact(kind, rows, tol, detail):
    def run(scale):
        scaled = [[c * scale for c in r] for r in rows]
        if kind != "position_jump":
            scaled[0] = [0.0, 0.0, 0.0]
        return verify(fleet([const((0, 0, 0), 0, 1), seg(1, 2, *scaled)]),
                      {"min_separation_m": 1.0})
    assert of_kind(run(1.0), kind) == []                       # exactly at the tolerance
    (v,) = of_kind(run(2.0), kind)
    assert v.drones == (0,) and v.t == 1.0 and v.limit == tol
    assert v.value == pytest.approx(2 * tol, rel=1e-9)
    assert v.detail.startswith("between segments 0 and 1: ")
    assert detail.split(" of ")[0].split(" ")[0] in v.detail


def test_continuity_detail_text_for_each_discontinuity():
    f = fleet([const((0, 0, 0), 0, 1), seg(1, 2, [0.5, 0, 0])])
    (v,) = of_kind(verify(f, {"min_separation_m": 1.0}), "position_jump")
    assert v.detail == "between segments 0 and 1: jump of 0.5 m" and v.value == 0.5
    f = fleet([const((0, 0, 0), 0, 1), seg(1, 2, [0, 0, 0], [3, 4, 0])])
    (v,) = of_kind(verify(f, {"min_separation_m": 1.0}), "velocity_discontinuity")
    assert v.detail == "between segments 0 and 1: velocity jump of 5 m/s (unbounded acceleration)"
    assert v.value == 5.0
    f = fleet([const((0, 0, 0), 0, 1), seg(1, 2, [0, 0, 0], [0, 0, 0], [3, 0, 0])])
    (v,) = of_kind(verify(f, {"min_separation_m": 1.0}), "accel_discontinuity")
    assert v.detail == "between segments 0 and 1: acceleration jump of 6 m/s^2 (unbounded jerk)"
    assert v.value == 6.0


def test_velocity_and_acceleration_continuity_compare_the_right_derivative_orders():
    # moving segment into a segment with matching position and velocity but a different
    # acceleration: only accel_discontinuity; and matching accel but different velocity: only
    # velocity_discontinuity
    a = seg(0, 1, [0, 0, 0], [1, 0, 0], [0.5, 0, 0])           # at tau=1: p=1.5, v=2, a=1
    same = seg(1, 2, [1.5, 0, 0], [2, 0, 0], [0.5, 0, 0])
    rep = verify(fleet([a, same]), {"min_separation_m": 1.0})
    assert rep.violation_counts == {}
    diff_a = seg(1, 2, [1.5, 0, 0], [2, 0, 0], [1.5, 0, 0])
    rep = verify(fleet([a, diff_a]), {"min_separation_m": 1.0})
    assert rep.violation_counts == {"accel_discontinuity": 1}
    diff_v = seg(1, 2, [1.5, 0, 0], [3, 0, 0], [0.5, 0, 0])
    rep = verify(fleet([a, diff_v]), {"min_separation_m": 1.0})
    assert rep.violation_counts == {"velocity_discontinuity": 1}


def test_nonfinite_records():
    nan_seg = fleet([seg(2, 3, [float("nan"), 0, 0])])
    (v,) = of_kind(verify(nan_seg, {"min_separation_m": 1.0}), "nonfinite")
    assert v.drones == (0,) and v.t == 2.0 and math.isnan(v.value) and v.limit == 0.0
    assert v.detail == "segment 0: NaN/inf in times or coefficients"
    bad_time = fleet([seg(float("nan"), 3, [0, 0, 0])])
    (v,) = of_kind(verify(bad_time, {"min_separation_m": 1.0}), "nonfinite")
    assert v.t == 0.0 and math.isnan(v.value)
    inf_time = fleet([seg(0, float("inf"), [0, 0, 0])])
    (v,) = of_kind(verify(inf_time, {"min_separation_m": 1.0}), "nonfinite")
    assert v.t == 0.0
    shape = Segment(4.0, 5.0, np.zeros((2, 2)))
    (v,) = of_kind(verify(fleet([shape]), {"min_separation_m": 1.0}), "nonfinite")
    assert (v.drones, v.t, v.limit) == ((0,), 4.0, 0.0) and math.isnan(v.value)
    assert v.detail == "segment 0: malformed coefficient shape (2, 2)"
    empty = Segment(0.0, 1.0, np.zeros((0, 3)))
    (v,) = of_kind(verify(fleet([empty]), {"min_separation_m": 1.0}), "nonfinite")
    assert v.detail == "segment 0: malformed coefficient shape (0, 3)"


def test_unreadable_segment_record():
    class Broken:
        t0, t1 = 0.0, 1.0

        @property
        def coeffs(self):
            raise ValueError("boom")

    rep = verify(fleet([Broken()]), {"min_separation_m": 1.0})
    (v,) = of_kind(rep, "nonfinite")
    assert (v.drones, v.t, v.limit) == ((0,), 0.0, 0.0) and math.isnan(v.value)
    assert v.detail == "segment 0: unreadable (boom)"


def test_bad_segment_resets_continuity_and_excludes_drone_from_geometry():
    # segment 1 is NaN; the check between segment 0 and segment 2 must not run against it
    d = [const((0, 0, 0), 0, 1), seg(1, 2, [float("nan"), 0, 0]), const((50, 0, 0), 2, 3)]
    rep = verify(fleet(d), {"min_separation_m": 1.0})
    assert rep.violation_counts == {"nonfinite": 1}
    # a drone with a nonfinite segment does not take part in separation, even if it also has
    # good segments that pass right through another drone
    bad = [seg(0, 4, [0, 0, 0], [1, 0, 0]), seg(4, 5, [float("nan"), 0, 0])]
    other = [seg(0, 4, [0, 0.1, 0], [1, 0, 0])]
    rep = verify(fleet(bad, other), {"min_separation_m": 1.0})
    assert rep.violation_counts == {"nonfinite": 1}
    assert rep.min_separation_is_lower_bound and rep.n_pairs_checked == 0
    # dynamics of excluded drones are not evaluated either: this one is wildly over the limit
    fast = [seg(0, 1, [0, 0, 0], [500, 0, 0]), seg(1, 2, [float("nan"), 0, 0])]
    rep = verify(fleet(fast), {"min_separation_m": 1.0, "vmax_h": 10.0})
    assert rep.violation_counts == {"nonfinite": 1}
    assert rep.max_speed_h == 0.0


def test_unsupported_degree_record_and_trailing_zero_rows():
    deg8 = seg(0, 1, *([[0, 0, 0]] + [[0, 0, 0]] * 7 + [[1, 0, 0]]))     # degree 8, nonzero top
    rep = verify(fleet([deg8]), {"min_separation_m": 1.0})
    (v,) = of_kind(rep, "unsupported_degree")
    assert (v.drones, v.t, v.value, v.limit) == ((0,), 0.0, 8.0, 7.0)
    assert v.detail == "segment 0: degree 8 exceeds the supported maximum 7"
    assert rep.max_speed_h == 0.0                                          # excluded entirely
    # degree 7 is accepted; trailing zero rows above it are dropped, not counted
    ok7 = seg(0, 1, *([[0, 0, 0]] * 7 + [[1, 0, 0]]), [0, 0, 0])
    assert ok7.coeffs.shape == (9, 3)
    assert of_kind(verify(fleet([ok7]), {"min_separation_m": 1.0}), "unsupported_degree") == []
    ok7b = seg(0, 1, [0, 0, 0], *([[0, 0, 0]] * 6), [1, 0, 0])
    assert ok7b.coeffs.shape == (8, 3)
    assert of_kind(verify(fleet([ok7b]), {"min_separation_m": 1.0}), "unsupported_degree") == []
    assert of_kind(verify(fleet([deg8]), {"min_separation_m": 1.0}), "unsupported_degree")


def test_degree_error_marks_segment_unusable_and_resets_continuity():
    deg8 = seg(1, 2, *([[0, 0, 0]] * 8 + [[1, 0, 0]]))
    d = [const((0, 0, 0), 0, 1), deg8, const((0, 0, 0), 2, 3)]
    rep = verify(fleet(d), {"min_separation_m": 1.0})
    assert rep.violation_counts == {"unsupported_degree": 1}


def test_trailing_zero_rows_do_not_change_the_result():
    plain = fleet([seg(0, 2, [0, 0, 0], [1, 2, 3])], [seg(0, 2, [5, 0, 0], [0, 0, 0])])
    padded = fleet([seg(0, 2, [0, 0, 0], [1, 2, 3], [0, 0, 0], [0, 0, 0])],
                   [seg(0, 2, [5, 0, 0], [0, 0, 0], [0, 0, 0])])
    lim = {"min_separation_m": 1.0}
    a, b = verify(plain, lim), verify(padded, lim)
    assert a.max_speed_h == b.max_speed_h and a.min_separation == b.min_separation
    # an all-zero coefficient block is the origin, not an error
    z = verify(fleet([seg(0, 1, [0, 0, 0], [0, 0, 0])]), lim)
    assert z.ok and z.min_z == 0.0 and z.max_z == 0.0


# ---------------------------------------------------------------------------------------------
# 6. Piece construction: holds before/after/between, covered and overlapping segments.
# ---------------------------------------------------------------------------------------------
def _probe_at(p, t0=0.0, t1=100.0):
    return [const(p, t0, t1)]


def test_drone_occupies_its_final_position_after_the_last_segment():
    mover = [seg(2, 3, [0, 0, 0], [10, 0, 0])]                    # ends at (10,0,0) at t=3
    f = fleet(mover, _probe_at((10, 0.5, 0)))
    res, _ = pair_minima(f, 4.0)
    assert res[(0, 1)][0] == pytest.approx(0.5, abs=1e-9)
    assert res[(0, 1)][1] >= 3.0 - 1e-9                            # only after it arrives
    rep = verify(f, {"min_separation_m": 1.0})
    assert rep.violation_counts == {"separation": 1}


def test_gap_is_bridged_by_holding_the_last_position():
    # segment [2,3] moves (0,0,0) -> (10,0,0); gap on [3,5]; then [5,6] departs from (10,0,0).
    # The hold position is evaluated at tau = t1 - t0 = 1 (not t1 + t0 = 5).
    d = [seg(2, 3, [0, 0, 0], [10, 0, 0]), seg(5, 6, [10, 0, 0], [0, 7, 0])]
    probe = [seg(3.5, 4.5, [10, 0.5, 0])]
    rep = verify(fleet(d, probe), {"min_separation_m": 1.0})
    assert rep.violation_counts.get("separation") == 1
    assert rep.violation_counts.get("time_gap") == 1
    assert rep.min_separation_pair == (0, 1)


def test_position_before_the_first_segment_is_the_start_position():
    d = [seg(5, 6, [0, 0, 0], [1, 0, 0])]
    probe = [seg(0, 4, [0, 3, 0], [0, -0.75, 0])]               # reaches y=0 at t=4... passes 0.0
    res, _ = pair_minima(fleet(d, probe), 4.0)
    assert res[(0, 1)][0] == pytest.approx(0.0, abs=1e-9)
    assert res[(0, 1)][1] == pytest.approx(4.0, abs=1e-9)


def test_covered_segment_is_ignored_and_later_segments_still_count():
    # segment B [1,2] lies fully inside A [0,3]; C [3,4] continues.  Overlap is reported, A wins
    # inside it, and the drone still ends at C's end, not at B's end.
    A = seg(0, 3, [0, 0, 0], [1, 0, 0])               # ends at (3,0,0)
    B = seg(1, 2, [900, 0, 0])                         # covered: must not move the drone
    C = seg(3, 4, [3, 0, 0], [0, 5, 0])                # ends at (3,5,0)
    f = fleet([A, B, C], _probe_at((3, 5.5, 0)))
    res, _ = pair_minima(f, 4.0)
    assert res[(0, 1)][0] == pytest.approx(0.5, abs=1e-9)
    # B's end position is never occupied
    far = fleet([A, B, C], _probe_at((900, 0.2, 0)))
    res, _ = pair_minima(far, 4.0)
    assert res == {}


def test_covered_trailing_segment_does_not_move_the_final_hold():
    A = seg(0, 3, [0, 0, 0], [1, 0, 0])               # ends (3,0,0)
    B = seg(1, 3, [900, 0, 0])                         # ends exactly at A's end: fully covered
    rep = pair_minima(fleet([A, B], _probe_at((900, 0.2, 0))), 4.0)
    assert rep[0] == {}
    near = pair_minima(fleet([A, B], _probe_at((3, 0.2, 0))), 4.0)
    assert near[0][(0, 1)][0] == pytest.approx(0.2, abs=1e-9)


def test_partial_overlap_starts_the_later_segment_where_the_earlier_ends():
    A = seg(0, 2, [0, 0, 0], [1, 0, 0])               # ends (2,0,0) at t=2
    B = seg(1, 4, [100, 0, 0], [0, 1, 0])             # overlaps [1,2]; counts from t=2 on
    # during [2,4] B moves along x=100 -> drone is near (100, 1..3); before t=2 A governs
    probe_a = _probe_at((1.0, 0.3, 0), 0.9, 1.1)
    res, _ = pair_minima(fleet([A, B], probe_a), 4.0)
    assert res[(0, 1)][0] == pytest.approx(0.3, abs=1e-9)


def test_chunked_box_computation_covers_every_piece(monkeypatch):
    monkeypatch.setattr(V, "_CHUNK", 3)
    drones = [[seg(0, 5, [100.0 * k, 0, 0], [0, 1, 0])] for k in range(8)]
    drones.append([seg(0, 5, [300, 0.5, 0], [0, 1, 0])])        # parallel to drone 3, 0.5 apart
    res, n = pair_minima(fleet(*drones), 4.0)
    assert set(res) == {(3, 8)} and res[(3, 8)][0] == pytest.approx(0.5, abs=1e-9)
    assert n == 1


def test_chunked_pair_solving_matches_unchunked(monkeypatch):
    drones = [[seg(0, 5, [3.0 * k, 0, 0], [0, 0.5 * k, 0])] for k in range(9)]
    f = fleet(*drones)
    full, nf = pair_minima(f, 100.0)
    monkeypatch.setattr(V, "_CHUNK", 4)
    part, np_ = pair_minima(f, 100.0)
    assert full.keys() == part.keys() and nf == np_
    for k in full:
        assert full[k][0] == pytest.approx(part[k][0], abs=1e-12)
        assert full[k][1] == pytest.approx(part[k][1], abs=1e-9)


def test_point_overlaps_between_pieces_are_evaluated_directly():
    # drone 1 starts exactly when drone 0's only segment ends: the shared instant is a single
    # point; the pair must still be considered at that instant (positions 0.2 apart)
    a = [seg(0, 1, [0, 0, 0])]
    b = [seg(1, 2, [0, 0.2, 0])]
    res, _ = pair_minima(fleet(a, b), 4.0)
    assert res[(0, 1)][0] == pytest.approx(0.2, abs=1e-12)


def test_point_overlap_uses_each_pieces_own_time_origin():
    # both pieces start late (t0 = 40) with a nonzero linear term; they meet a single instant at
    # t = 50, so the evaluation at tau = t - t0 matters (a sign error would use tau = 90)
    a = [seg(40, 50, [0, 0, 0], [1, 0, 0])]                    # at t=50: x = 10
    b = [seg(50, 60, [10, 0.4, 0], [0, 1, 0])]                 # starts at (10,0.4)
    res, _ = pair_minima(fleet(a, b), 4.0)
    assert res[(0, 1)][0] == pytest.approx(0.4, abs=1e-12)
    assert res[(0, 1)][1] == pytest.approx(50.0, abs=1e-12)


def test_minimum_time_is_absolute_even_for_late_segments():
    a = [seg(100, 110, [-5, 0, 0], [1, 0, 0])]
    b = [seg(100, 110, [0, 1, 0])]
    d, i, j, t, lb, _ = pairwise_min_separation(fleet(a, b), 10.0)
    assert d == pytest.approx(1.0, abs=1e-12) and t == pytest.approx(105.0, abs=1e-9)


# ---------------------------------------------------------------------------------------------
# 9. Numerical kernels, tested directly against hand-derived answers.
# ---------------------------------------------------------------------------------------------

def test_binom_matrix_and_cache(monkeypatch):
    # B[k, j] = C(j, k) for k <= j, zero below the diagonal (hand-written Pascal triangle).
    monkeypatch.setattr(V, "_BINOM_CACHE", {})
    want = np.array([[1, 1, 1, 1], [0, 1, 2, 3], [0, 0, 1, 3], [0, 0, 0, 1]], float)
    got = V._binom(4)
    assert np.array_equal(got, want)
    assert V._binom(4) is got                       # second call is served from the cache
    assert np.array_equal(V._binom(2), [[1, 1], [0, 1]])
    assert set(V._BINOM_CACHE) == {2, 4}


def test_shift_quadratic_and_cubic(monkeypatch):
    monkeypatch.setattr(V, "_BINOM_CACHE", {})
    # p(tau) = tau^2 in every coordinate, q(s) = p(1 + 2 s) = 1 + 4 s + 4 s^2.
    c = np.zeros((1, 3, 3))
    c[0, 2, :] = 1.0
    q = V._shift(c, np.array([1.0]), np.array([2.0]))
    assert np.allclose(q[0], [[1] * 3, [4] * 3, [4] * 3], atol=1e-12)
    # p(tau) = tau^3 (x only), q(s) = (2 + s/2)^3 = 8 + 6 s + 1.5 s^2 + 0.125 s^3.
    c = np.zeros((1, 4, 3))
    c[0, 3, 0] = 1.0
    q = V._shift(c, np.array([2.0]), np.array([0.5]))
    assert np.allclose(q[0, :, 0], [8, 6, 1.5, 0.125], atol=1e-12)
    assert np.allclose(q[0, :, 1:], 0.0)
    # tau0 = 0 must not produce 0**0 problems: q(s) = (h s)^2.
    c = np.zeros((1, 3, 3))
    c[0, 2, 1] = 1.0
    q = V._shift(c, np.array([0.0]), np.array([3.0]))
    assert np.allclose(q[0, :, 1], [0, 0, 9], atol=1e-12)
    # constant polynomials (K = 1) are returned as an independent copy.
    c = np.array([[[1.0, 2.0, 3.0]]])
    q = V._shift(c, np.array([5.0]), np.array([7.0]))
    assert np.array_equal(q, c) and q is not c


def test_extremes_interior_minimum_is_found_off_the_grid():
    # (s - 0.3)^2 = 0.09 - 0.6 s + s^2: minimum 0 at s = 0.3 (not a multiple of 1/8), max 0.49 at s = 1.
    fmin, smin, fmax, smax = V._extremes(np.array([[0.09, -0.6, 1.0]]))
    assert fmin[0] == pytest.approx(0.0, abs=1e-12)
    assert smin[0] == pytest.approx(0.3, abs=1e-9)
    assert fmax[0] == pytest.approx(0.49, abs=1e-12)
    assert smax[0] == pytest.approx(1.0, abs=1e-12)


def test_extremes_interior_maximum_and_mixed_degrees():
    # Row 0: 3 s - 3 s^2, maximum 0.75 at 0.5. Row 1: s^3 + s (no real stationary point), range [0, 2].
    c = np.array([[0.0, 3.0, -3.0, 0.0], [0.0, 1.0, 0.0, 1.0]])
    fmin, smin, fmax, smax = V._extremes(c)
    assert fmax[0] == pytest.approx(0.75, abs=1e-12) and smax[0] == pytest.approx(0.5, abs=1e-9)
    assert fmin[0] == pytest.approx(0.0, abs=1e-12)
    assert fmin[1] == pytest.approx(0.0, abs=1e-12) and smin[1] == pytest.approx(0.0, abs=1e-12)
    assert fmax[1] == pytest.approx(2.0, abs=1e-12) and smax[1] == pytest.approx(1.0, abs=1e-12)


def test_extremes_ignores_stationary_points_outside_the_interval():
    # (s - 1.5)^2 on [0, 1]: decreasing, so min 0.25 at s = 1 and max 2.25 at s = 0.
    fmin, smin, fmax, smax = V._extremes(np.array([[2.25, -3.0, 1.0]]))
    assert (fmin[0], smin[0]) == (pytest.approx(0.25), pytest.approx(1.0))
    assert (fmax[0], smax[0]) == (pytest.approx(2.25), pytest.approx(0.0))
    # a constant polynomial has no derivative terms at all
    fmin, smin, fmax, smax = V._extremes(np.array([[4.0]]))
    assert fmin[0] == 4.0 and fmax[0] == 4.0


def test_critical_points_root_range_and_complex_rejection():
    # derivative 2 s - 0.6 -> single root 0.3
    cp = V._critical_points(np.array([[-0.6, 2.0]]))
    assert cp[0, 0] == pytest.approx(0.3, abs=1e-12)
    # root at 1.0005 is within the 1e-3 slack and is clipped into [0, 1]
    cp = V._critical_points(np.array([[-1.0005, 1.0]]))
    assert cp[0, 0] == 1.0
    # root at -0.0005 likewise clips to 0
    cp = V._critical_points(np.array([[0.0005, 1.0]]))
    assert cp[0, 0] == 0.0
    # roots beyond the slack are rejected (nan)
    assert np.isnan(V._critical_points(np.array([[-1.01, 1.0]]))[0, 0])
    assert np.isnan(V._critical_points(np.array([[0.01, 1.0]]))[0, 0])
    # complex roots (s^2 + 0.25) are rejected
    assert np.isnan(V._critical_points(np.array([[0.25, 0.0, 1.0]]))[0, :2]).all()
    # the output always ends with the 9-point grid 0, 1/8, ..., 1
    grid = V._critical_points(np.array([[-0.6, 2.0]]))[0, -(V._GRID + 1):]
    assert np.allclose(grid, np.linspace(0, 1, V._GRID + 1))


def test_over_and_under_use_a_relative_tolerance():
    tol = 1e-9
    assert V._over(1.0 + 3e-9, 1.0, tol) and not V._over(1.0 + 1e-9, 1.0, tol)
    assert not V._over(1.0 - 1.0, 1.0, tol)
    # |limit| enters the tolerance: limit -3 gives 4e-9, not 2e-9 or 0
    assert not V._over(-3.0 + 3e-9, -3.0, tol) and V._over(-3.0 + 5e-9, -3.0, tol)
    assert V._under(1.0 - 3e-9, 1.0, tol) and not V._under(1.0 - 1e-9, 1.0, tol)
    assert not V._under(-3.0 - 3e-9, -3.0, tol) and V._under(-3.0 - 5e-9, -3.0, tol)
    assert not V._under(1.0 + 3e-9, 1.0, tol)


def test_over_under_exact_boundary_is_not_a_violation():
    # 0 + 1e-9 * (1 + 0) is exactly representable as the same double as the literal
    assert not V._over(1e-9, 0.0, 1e-9) and not V._under(-1e-9, 0.0, 1e-9)
    assert V._over(2e-9, 0.0, 1e-9) and V._under(-2e-9, 0.0, 1e-9)


def test_speed_up_statistic_is_zero_for_a_purely_descending_fleet():
    rep = verify(fleet([seg(0, 1, [0, 0, 10], [0, 0, -3])]), {"min_separation_m": 1.0})
    assert rep.max_speed_up == 0.0 and rep.max_speed_down == pytest.approx(3.0)
    rep = verify(fleet([seg(0, 1, [0, 0, 0], [0, 0, 4])]), {"min_separation_m": 1.0})
    assert rep.max_speed_down == 0.0 and rep.max_speed_up == pytest.approx(4.0)


class _Unreadable:
    def __init__(self, t0, t1):
        self.t0, self.t1 = t0, t1

    @property
    def coeffs(self):
        raise ValueError("x")


BAD_SEGMENTS = {
    "unreadable": (_Unreadable, "nonfinite"),
    "shape": (lambda t0, t1: Segment(float(t0), float(t1), np.zeros((2, 2))), "nonfinite"),
    "nonfinite": (lambda t0, t1: seg(t0, t1, [float("nan"), 0, 0]), "nonfinite"),
    "degree": (lambda t0, t1: seg(t0, t1, *([[0, 0, 0]] * 8 + [[1, 0, 0]])), "unsupported_degree"),
}


@pytest.mark.parametrize("name", sorted(BAD_SEGMENTS))
def test_every_kind_of_bad_segment_is_reported_each_time_and_excludes_the_drone(name):
    make, kind = BAD_SEGMENTS[name]
    # good, bad, bad, good: both bad segments must be reported (the loop continues after the
    # first), nothing is compared across them, and the drone leaves the geometry although it
    # sits exactly on top of drone 1.
    d0 = [const((0, 0, 0), 0, 1), make(1, 2), make(2, 3), const((0, 0, 0), 3, 4)]
    d1 = [const((0, 0, 0), 0, 4)]
    d2 = [seg(0, 1, [50, 50, 0], [100, 0, 0])]                   # speeding drone after the bad one
    rep = verify(fleet(d0, d1, d2), {"min_separation_m": 1.0, "vmax_h": 10.0})
    assert rep.violation_counts == {kind: 2, "speed_h": 1}
    assert not of_kind(rep, "separation")
    assert not any(v.kind == "speed_h" and 0 in v.drones for v in rep.violations)


def test_empty_drone_does_not_stop_the_checks_of_later_drones():
    empty = Trajectory([])
    f = Fleet([empty, Trajectory([seg(0, 1, [0, 0, 0], [100, 0, 0])])])
    rep = verify(f, {"min_separation_m": 1.0, "vmax_h": 10.0})
    assert rep.violation_counts == {"time_gap": 1, "speed_h": 1}
    (g,) = of_kind(rep, "time_gap")
    assert g.drones == (0,) and g.detail == "drone has no segments"


def test_pieces_survive_a_bad_drone_in_front():
    # drone 0 has only a bad segment; drone 1 and 2 must still be compared with each other
    d0 = [seg(0, 1, [float("nan"), 0, 0])]
    d1 = [const((0, 0, 0), 0, 5)]
    d2 = [const((0.5, 0, 0), 0, 5)]
    rep = verify(fleet(d0, d1, d2), {"min_separation_m": 1.0})
    assert [v.drones for v in of_kind(rep, "separation")] == [(1, 2)]


def test_hold_after_the_last_segment_is_at_the_segment_end_position():
    # A flies x = 10 * tau on [0, 1] (tau = t - t0) and then hovers at x = 10.  B arrives at x = 10
    # at t = 5 and sits 0.5 m away from the hover point.
    # (A's segment starts at t0 = 2, so evaluating the end at t1 + t0 instead of t1 - t0 would differ)
    a = [seg(2, 3, [0, 0, 0], [10, 0, 0])]
    b = [seg(0, 5, [10.5, 50, 0], [0, -10, 0])]                              # y = 50 - 10 tau; y(5) = 0
    rep = verify(fleet(a, b), {"min_separation_m": 1.0})
    assert rep.min_separation == pytest.approx(0.5, abs=1e-6)
    assert rep.min_separation_pair == (0, 1) and rep.min_separation_time == pytest.approx(5.0, abs=1e-5)
    assert [v.drones for v in of_kind(rep, "separation")] == [(0, 1)]


def test_hold_across_a_time_gap_is_at_the_previous_segment_end():
    # A: x = 4 tau on [0, 1], gap during (1, 3), then x = 4 (constant) on [3, 4].  The gap is bridged by
    # holding at x = 4.  B passes through x = 4 at t = 2 (inside the gap), 0.25 m to the side.
    a = [seg(0, 1, [0, 0, 0], [4, 0, 0]), seg(3, 4, [4, 0, 0])]
    b = [seg(0, 4, [4.25, 8, 0], [0, -4, 0])]                                  # y(2) = 0
    rep = verify(fleet(a, b), {"min_separation_m": 1.0})
    assert rep.min_separation == pytest.approx(0.25, abs=1e-6)
    assert rep.min_separation_time == pytest.approx(2.0, abs=1e-5)
    assert [v.drones for v in of_kind(rep, "time_gap")] == [(0,)]
    # and the same drone before its first segment is held at the first position
    c = [seg(2, 3, [4, 0, 0])]
    d = [seg(0, 3, [4, 0.25, 0])]
    rep = verify(fleet(c, d), {"min_separation_m": 1.0})
    assert rep.min_separation == pytest.approx(0.25, abs=1e-9) and rep.min_separation_time == pytest.approx(0.0, abs=1e-6)


def test_violations_of_one_kind_are_listed_by_excess_over_the_limit():
    # both are 'altitude' violations: 101 against z_max = 100 (excess 1) and -5 against z_min = 0
    # (excess 5); the larger excess is listed first although |101 + 100| is the larger number.
    f = fleet([const((0, 0, 101))], [const((50, 0, -5))])
    rep = verify(f, {"min_separation_m": 1.0, "z_max": 100.0, "z_min": 0.0})
    assert [(v.drones, v.value) for v in of_kind(rep, "altitude")] == [((1,), -5.0), ((0,), 101.0)]
