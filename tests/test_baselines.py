import numpy as np
import pytest

from sirb.baselines.orca import constraint_planes, orca_velocities, plan_orca
from sirb.baselines.sampled import sampled_min_separation
from sirb.shapes import generate_points
from sirb.trajectory import Fleet, Segment, Trajectory
from sirb.verify import pairwise_min_separation


def line(p0, p1, t1):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    return Trajectory([Segment(0.0, t1, np.stack([p0, (p1 - p0) / t1]))])


def head_on():
    return Fleet([line([-10, 0, 5], [10, 0, 5], 10.0), line([10, 0, 5], [-10, 0, 5], 10.0)])


def test_sampled_checker_misses_what_the_exact_checker_sees():
    # Two drones cross at the midpoint at t=5; a frame step of 10 s never sees it.
    fleet = head_on()
    exact = pairwise_min_separation(fleet, 1000.0)[0]
    assert exact < 1e-9
    coarse = sampled_min_separation(fleet, dt=10.0, d_min=1.0)
    assert coarse.ok and coarse.min_separation > 15.0
    fine = sampled_min_separation(fleet, dt=0.01, d_min=1.0)
    assert not fine.ok and fine.min_separation < 0.2


def test_sampled_checker_never_below_exact():
    rng = np.random.default_rng(1)
    fleet = Fleet([line(rng.uniform(0, 20, 3), rng.uniform(0, 20, 3), 8.0) for _ in range(6)])
    exact = pairwise_min_separation(fleet, 1000.0)[0]
    for dt in (1.0, 0.3, 0.05):
        assert sampled_min_separation(fleet, dt=dt, d_min=1.0).min_separation >= exact - 1e-9


def test_orca_planes_are_unit_normals_and_point_away():
    p = np.array([[5.0, 0.0, 0.0]])
    vrel = np.array([[2.0, 0.0, 0.0]])
    u, n = constraint_planes(p, vrel, 2.0, 3.0, 0.1)
    assert np.isclose(np.linalg.norm(n[0]), 1.0)
    assert n[0] @ u[0] > 0 or np.isclose(u[0] @ n[0], np.linalg.norm(u[0]))


def test_orca_head_on_keeps_clear():
    start = np.array([[-10.0, 0.3, 5], [10.0, -0.3, 5]])
    goal = start[::-1].copy()
    res = plan_orca(start, goal, radius_sum=2.0, vmax=3.0, max_time_s=60.0)
    assert res.arrived
    assert pairwise_min_separation(res.fleet, 1000.0)[0] > 1.5


def test_orca_single_agent_goes_straight():
    v = orca_velocities(np.zeros((1, 3)), np.zeros((1, 3)), np.array([[1.0, 2.0, 0.0]]),
                        radius_sum=2.0, tau=3.0, dt=0.1, vmax=4.0)
    assert np.allclose(v, [[1.0, 2.0, 0.0]])


def test_points_shape_roundtrip_and_validation():
    pts = [[0, 0, 5], [3, 0, 5], [0, 3, 6]]
    out = generate_points({"type": "points", "points": pts}, 3)
    assert out.shape == (3, 3) and np.allclose(out, pts)
    with pytest.raises(ValueError):
        generate_points({"type": "points", "points": pts}, 4)
    with pytest.raises(ValueError):
        generate_points({"type": "points", "points": [[0, 0, float("nan")]] * 3}, 3)
