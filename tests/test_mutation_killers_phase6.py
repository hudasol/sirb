"""Known answer tests for survivors of the fresh phase 6 mutmut run (docs/MUTATION.md).

Expected values are derived by hand or with exact Python integer arithmetic, never read back from
the code under test.
"""

import numpy as np
import pytest

from sirb.assign import _exact_sq_cost, assign_hungarian_len, assign_hungarian_sq
from sirb.planner.baseline import plan_baseline
from sirb.planner.common import path_segments
from sirb.planner.waves import plan_waves
from sirb.spec import new_show
from sirb.verify import verify

Q = 2 ** 20   # grid cells per metre in sirb.shapes and sirb.assign


def _exact(src, dst):
    """Squared distances in Python integers on the 2^-20 m grid, converted once to float."""
    out = np.empty((len(src), len(dst)))
    for i, a in enumerate(src):
        for j, b in enumerate(dst):
            ka = [round(x * Q) for x in a]
            kb = [round(x * Q) for x in b]
            out[i, j] = float(sum((p - q) ** 2 for p, q in zip(ka, kb))) / 2 ** 40
    return out


def test_exact_cost_matches_integer_arithmetic_far_from_the_origin():
    # Coordinates of a few hundred metres make each squared difference exceed 2^53 grid units,
    # where accumulating in floats would round differently from the exact integer sum.
    src = np.array([[311.25, -287.5, 92.75], [-401.0, 12.5, 3.0]]) + np.array([1, 3, 5]) / Q
    dst = np.array([[-299.0, 305.125, 1.5], [400.0, -410.0, 88.0], [0.0, 0.0, 0.0]])
    c = _exact_sq_cost(src, dst)
    assert c.dtype == np.float64
    assert np.array_equal(c, _exact(src, dst))


def test_exact_cost_known_small_values():
    c = _exact_sq_cost(np.array([[0.0, 0.0, 0.0]]), np.array([[3.0, 4.0, 0.0], [1.0, 1.0, 1.0]]))
    assert c.tolist() == [[25.0, 3.0]]


def test_exact_cost_rejects_coordinates_beyond_512_m():
    with pytest.raises(ValueError, match="beyond 512 m"):
        _exact_sq_cost(np.array([[512.0, 0.0, 0.0]]), np.array([[0.0, 0.0, 0.0]]))
    _exact_sq_cost(np.array([[511.9, 0.0, 0.0]]), np.array([[0.0, 0.0, 0.0]]))   # inside: fine


def test_exact_cost_of_empty_inputs_is_an_empty_matrix():
    c = _exact_sq_cost(np.zeros((0, 3)), np.zeros((2, 3)))
    assert c.shape == (0, 2)


def test_hungarian_variants_pick_the_known_optimum():
    src = np.array([[0.0, 0.0, 0.0], [10.0, 0.0, 0.0]])
    dst = np.array([[10.0, 1.0, 0.0], [0.0, 1.0, 0.0]])
    assert assign_hungarian_sq(src, dst).tolist() == [1, 0]
    assert assign_hungarian_len(src, dst).tolist() == [1, 0]


def _grid_stage(name, spacing=4.0, center=(0.0, 0.0, 0.0), **extra):
    return {"name": name, "shape": {"type": "launch_grid", "spacing_m": spacing,
                                    "center": list(center)}, **extra}


def test_baseline_transition_where_nobody_moves_verifies():
    # Found by this mutation work: two equal stages gave a leg time of 0 and zero length holds,
    # which the verifier flags as structural errors.
    show = new_show("stationary", 4, [_grid_stage("a", hold_s=1), _grid_stage("b", hold_s=1)])
    fleet, _ = plan_baseline(show)
    assert verify(fleet, show["limits"]).ok
    for tr in fleet.trajectories:
        assert all(s.t1 > s.t0 for s in tr.segments)
        assert np.allclose(tr.pos(1.5), tr.pos(0.0))


def test_baseline_holds_a_drone_whose_target_is_its_position():
    # 3 x 3 grids at 4 m and 8 m pitch share the centre point (0, 0): that drone stays while the
    # other eight move, so the planner must hold it at its own position for the whole leg.
    show = new_show("one stays", 9, [_grid_stage("a", hold_s=1),
                                     _grid_stage("b", spacing=8.0, hold_s=1)])
    fleet, info = plan_baseline(show)
    assert verify(fleet, show["limits"]).ok
    leg = info["transitions"][0]["leg_s"]
    assert leg > 0
    still = [tr for tr in fleet.trajectories if np.allclose(tr.pos(0.0), [0.0, 0.0, 0.0])]
    assert len(still) == 1
    for t in np.linspace(1.0, 1.0 + leg, 7):
        assert np.allclose(still[0].pos(t), [0.0, 0.0, 0.0])


def test_waves_accepts_stages_without_hold_time():
    show = new_show("no holds", 4, [_grid_stage("a"), _grid_stage("b", center=(0.0, 0.0, 10.0))])
    fleet, _ = plan_waves(show)
    assert verify(fleet, show["limits"]).ok
    assert fleet.trajectories[0].segments[0].t0 == 0.0


def test_path_segments_chain_rest_to_rest_legs_through_waypoints():
    lim = {"vmax_h": 5.0, "vmax_up": 2.0, "vmax_down": 2.0, "amax": 3.0, "jmax": 6.0}
    w = [np.array([0.0, 0.0, 0.0]), np.array([10.0, 0.0, 0.0]), np.array([10.0, 10.0, 0.0])]
    segs = path_segments(2.0, w, lim)
    assert len(segs) == 2 and segs[0].t0 == 2.0 and segs[1].t0 == segs[0].t1
    assert np.allclose(segs[0].start(), w[0]) and np.allclose(segs[0].end(), w[1])
    assert np.allclose(segs[1].start(), w[1]) and np.allclose(segs[1].end(), w[2])
    # a repeated waypoint adds no leg
    assert len(path_segments(0.0, [w[0], w[0], w[1]], lim)) == 1
