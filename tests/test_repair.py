"""Repair moves: they keep waypoints, respect the exact verifier, and are logged."""

import numpy as np

from sirb.planner.baseline import plan_baseline
from sirb.repair import apply_fleet_moves, cost, locate, path_length, scale_stage, stage_windows
from sirb.spec import new_show
from sirb.verify import verify


def _show():
    grid = {"type": "launch_grid", "spacing_m": 3.2, "center": [0.0, -40.0, 0.0]}
    return new_show("repair test", 9, [
        {"name": "launch", "shape": grid, "hold_s": 1},
        {"name": "orb", "shape": {"type": "sphere", "spacing_m": 3.2, "center": [0, 0, 40]},
         "hold_s": 2},
        {"name": "land", "shape": grid, "hold_s": 1}], seed=3)


def test_delay_keeps_endpoints_and_adds_duration():
    show = _show()
    fleet, _ = plan_baseline(show)
    moved = apply_fleet_moves(fleet, [{"type": "start_delay", "drone": 2, "delay_s": 1.5}])
    a, b = fleet.trajectories[2], moved.trajectories[2]
    assert np.allclose(a.pos(a.t_end), b.pos(b.t_end))
    assert np.allclose(a.pos(0.0), b.pos(0.0))
    assert b.t_end == a.t_end + 1.5
    assert np.allclose(fleet.trajectories[3].pos(5.0), moved.trajectories[3].pos(5.0))


def test_dilation_stretches_the_leg_and_keeps_waypoints():
    show = _show()
    fleet, info = plan_baseline(show)
    w = [x for x in stage_windows(show, info) if x[0] == "transition"][0]
    tref = (w[2] + w[3]) / 2
    moved = apply_fleet_moves(fleet, [{"type": "dilate_leg", "drone": 0, "t_ref": tref,
                                       "factor": 1.5}])
    a, b = fleet.trajectories[0], moved.trajectories[0]
    assert np.allclose(a.pos(a.t_end), b.pos(b.t_end))
    assert np.isclose(b.t_end - a.t_end, 0.5 * (w[3] - w[2]))
    assert np.allclose(b.pos(w[2] + 1.5 * (w[3] - w[2])), a.pos(w[3]))
    assert verify(moved, show["limits"]).max_speed_h <= verify(fleet, show["limits"]).max_speed_h


def test_stage_spacing_scales_only_that_stage():
    show = _show()
    s2 = scale_stage(show, 1, 1.1)
    assert s2["stages"][1]["shape"]["spacing_m"] == 3.2 * 1.1
    assert s2["stages"][0]["shape"]["spacing_m"] == 3.2
    assert show["stages"][1]["shape"]["spacing_m"] == 3.2


def test_cost_and_windows():
    show = _show()
    fleet, info = plan_baseline(show)
    assert cost(fleet, fleet) == {"extra_duration_s": 0.0, "extra_path_length_m": 0.0}
    assert path_length(fleet) > 0
    w = stage_windows(show, info)
    assert locate(w, 0.5)[0] == "hold" and locate(w, w[1][2] + 0.1)[0] == "transition"
