import json
from pathlib import Path

import numpy as np
import pytest

from sirb import assign as A
from sirb.planner.baseline import plan_baseline
from sirb.planner.prioritized import plan_prioritized
from sirb.planner.waves import plan_waves, schedule_waves
from sirb.spec import load_show
from sirb.verify import verify

SHOW = "shows/sirb_text_100.json"


def test_assignment_validity_and_optimality():
    rng = np.random.default_rng(0)
    src, dst = rng.random((40, 3)) * 50, rng.random((40, 3)) * 50
    sq = lambda p: float(((dst[p] - src) ** 2).sum())
    best = sq(A.assign_hungarian_sq(src, dst))
    for name, fn in A.METHODS.items():
        perm = fn(src, dst, 1) if name in ("random", "pinned15") else fn(src, dst)
        assert sorted(perm.tolist()) == list(range(40))
        assert sq(perm) >= best - 1e-6
    # length-optimal minimises total length
    L = A.total_path_length
    assert L(src, dst, A.assign_hungarian_len(src, dst)) <= L(src, dst, A.assign_greedy(src, dst)) + 1e-9


def test_linear_conflicts_exact():
    src = np.array([[0.0, 0, 0], [10, 0, 0]])
    dst = np.array([[10.0, 0, 0], [0, 0, 0]])
    assert A.linear_conflicts(src, dst, np.array([0, 1]), 1.0) == 1   # head-on swap
    assert A.linear_conflicts(src, dst, np.array([1, 0]), 1.0) == 0   # each stays


@pytest.mark.parametrize("planner", [plan_baseline, plan_waves, plan_prioritized])
def test_planners_verified_on_demo(planner):
    show = load_show(SHOW)
    fleet, info = planner(show)
    rep = verify(fleet, show["limits"])
    assert rep.ok, rep.violation_counts
    lim = show["limits"]
    assert rep.max_speed_h <= lim["vmax_h"] + 1e-6
    assert rep.max_accel <= lim["amax"] + 1e-6
    assert rep.max_jerk <= lim["jmax"] + 1e-6
    assert rep.min_separation >= lim["min_separation_m"]
    assert info["t_end"] == pytest.approx(fleet.t_end)


def test_unsynchronised_and_identity_baselines_are_flagged():
    show = load_show(SHOW)
    for kwargs in ({"sync": False}, {"assign": "identity"}, {"waves": 4, "stagger_s": 3.0}):
        fleet, _ = plan_baseline(show, **kwargs)
        assert not verify(fleet, show["limits"]).ok, kwargs


def test_wave_scheduler_splits_and_reports_infeasible():
    # Two drones swapping on one line: no straight-line wave order is collision free.
    src = np.array([[0.0, 0, 50], [10, 0, 50]])
    dst = np.array([[10.0, 0, 50], [0, 0, 50]])
    with pytest.raises(RuntimeError):
        schedule_waves(src, dst, 2.05)
    # Two parallel drones moving apart sideways share a wave; perpendicular crossers split.
    src = np.array([[0.0, 0, 50], [30, 0, 50]])
    dst = np.array([[0.0, 30, 50], [30, 30, 50]])
    assert list(schedule_waves(src, dst, 2.05)) == [0, 0]


def test_deterministic_plans():
    show = load_show(SHOW)
    f1, _ = plan_waves(show)
    f2, _ = plan_waves(show)
    assert f1.content_hash() == f2.content_hash()


def test_results_files_consistent_if_present():
    p = Path("results/planners.json")
    if not p.exists():
        pytest.skip("run scripts/bench_planners.py")
    data = json.loads(p.read_text())
    for show, rows in data.items():
        sync = next(r for r in rows if r["planner"] == "baseline sync (hungarian)")
        assert sync["verified_ok"]
