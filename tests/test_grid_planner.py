"""Tests for the exploratory grid planner (sirb/planner/grid.py). Every plan is judged by the
independent exact verifier under the assumed ground robot limits."""

import itertools
import math

import numpy as np
import pytest

import sirb.planner.grid as G
from sirb.planner.common import leg_segment, min_leg_time
from sirb.spec import new_show
from sirb.verify import verify

# Same assumed limits as scripts/ground_robots.py (assumptions, not measurements).
LIMITS = {"vmax_h": 1.5, "vmax_up": 0.05, "vmax_down": 0.05, "amax": 1.0, "jmax": 3.0,
          "z_min": -0.05, "z_max": 0.05, "min_separation_m": 1.0,
          "geofence": {"x": [-60.0, 60.0], "y": [-60.0, 60.0]},
          "crowd": {"y": -500.0, "min_distance_m": 50.0},
          "sources": {k: "assumption" for k in ("vmax_h", "vmax_up", "vmax_down", "amax", "jmax",
                                               "z_min", "z_max", "geofence", "min_separation_m")}
          | {"crowd": "not used on the floor study"}}
PITCH = 2.0


def grid_show(n, task):
    a = {"type": "launch_grid", "spacing_m": PITCH, "center": [-10.0, 0.0, 0.0]}
    b = {"type": "launch_grid", "spacing_m": PITCH,
         "center": [10.0, 0.0, 0.0] if task == "translate" else [-10.0, 0.0, 0.0]}
    return new_show(f"test {task} n={n}", n, [
        {"name": "A", "shape": a, "hold_s": 1}, {"name": "B", "shape": b, "hold_s": 1}],
        seed=11, limits=LIMITS)


def lattice_show(starts, goals, monkeypatch):
    """A show whose stage targets are the given lattice cells (identity assignment)."""
    A = np.array([[x * PITCH, y * PITCH, 0.0] for x, y in starts])
    B = np.array([[x * PITCH, y * PITCH, 0.0] for x, y in goals])
    monkeypatch.setattr(G, "build_targets", lambda show: [A, B])
    return new_show("lattice", len(starts), [
        {"name": "A", "shape": {"type": "launch_grid", "spacing_m": PITCH}, "hold_s": 1},
        {"name": "B", "shape": {"type": "launch_grid", "spacing_m": PITCH}, "hold_s": 1}],
        seed=1, limits=LIMITS)


def corridor_blocked(x_lo, x_hi, pocket):
    """Cells of the margin 1 box around a straight corridor y = 0, x in [x_lo, x_hi], that are
    walls, leaving the corridor and one pocket cell free."""
    free = {(x, 0) for x in range(x_lo, x_hi + 1)} | {pocket}
    return [(x * PITCH, y * PITCH) for x in range(x_lo - 1, x_hi + 2) for y in (-1, 0, 1)
            if (x, y) not in free]


def test_worst_case_separation_is_pitch_over_root_two():
    assert G.worst_case_separation(PITCH) == pytest.approx(PITCH / math.sqrt(2))


def test_worst_case_separation_matches_sampled_legs():
    """Independent check of the geometric claim with real quintic legs: for every pair of step
    choices without a vertex conflict or edge swap, the sampled distance never drops below the
    claimed worst case, and the claimed worst case is attained."""
    T = min_leg_time(np.array([PITCH, 0.0, 0.0]), LIMITS)
    u = np.linspace(0.0, T, 401)
    smallest = math.inf
    for rx, ry in itertools.product(range(-3, 4), repeat=2):
        if (rx, ry) == (0, 0):
            continue
        for a, b in itertools.product(G._MOVES, repeat=2):
            if (rx + a[0] - b[0], ry + a[1] - b[1]) == (0, 0):
                continue
            if a != (0, 0) and a == (-b[0], -b[1]) and (rx, ry) == (-a[0], -a[1]):
                continue
            pa0 = np.array([rx * PITCH, ry * PITCH, 0.0])
            pb0 = np.zeros(3)
            sa = leg_segment(0.0, T, pa0, pa0 + PITCH * np.array([*a, 0.0]))
            sb = leg_segment(0.0, T, pb0, pb0 + PITCH * np.array([*b, 0.0]))
            d = np.linalg.norm(sa.pos(u) - sb.pos(u), axis=1).min()
            smallest = min(smallest, d)
    assert smallest >= G.worst_case_separation(PITCH) - 1e-6
    assert smallest == pytest.approx(G.worst_case_separation(PITCH), rel=2e-3)


def test_translate_is_straight_and_verified():
    show = grid_show(16, "translate")
    fleet, info = G.plan_grid(show, assign="hungarian_sq")
    rep = verify(fleet, show["limits"])
    assert rep.ok
    assert info["transitions"][0]["steps"] == 10  # 20 m at 2 m per cell, no waiting
    assert info["transitions"][0]["discrete_conflicts"] == 0
    assert info["t_end"] == pytest.approx(10 * info["step_s"] + 2.0)
    assert rep.min_separation == pytest.approx(PITCH)


def test_rotation_2x2_one_step_and_verified(monkeypatch):
    # four robots rotate one quarter turn: each enters the cell its neighbour leaves
    ring = [(0, 0), (1, 0), (1, 1), (0, 1)]
    show = lattice_show(ring, ring[1:] + ring[:1], monkeypatch)
    fleet, info = G.plan_grid(show, assign="identity")
    rep = verify(fleet, show["limits"])
    assert rep.ok
    assert info["transitions"][0]["steps"] == 1
    assert info["transitions"][0]["sum_of_costs"] == 4
    assert rep.min_separation >= G.worst_case_separation(PITCH) - 1e-6


def test_corridor_swap_with_side_pocket_known_answer(monkeypatch):
    # corridor x = 0..3, pocket above x = 2. A: 0 -> 3, B: 3 -> 0. The optimum has B step into
    # the pocket: B needs 3 moves plus a 2 move detour (makespan 5), A needs 3 (sum of costs 8).
    blocked = corridor_blocked(0, 3, (2, 1))
    show = lattice_show([(0, 0), (3, 0)], [(3, 0), (0, 0)], monkeypatch)
    fleet, info = G.plan_grid(show, assign="identity", margin=1, blocked_xy=blocked)
    t = info["transitions"][0]
    assert (t["steps"], t["sum_of_costs"], t["discrete_conflicts"]) == (5, 8, 0)
    assert verify(fleet, show["limits"]).ok
    # nobody ever stands on a wall cell, and the pocket is used by exactly one robot
    walls = {(round(x / PITCH), round(y / PITCH)) for x, y in blocked}
    times = np.linspace(0.0, fleet.t_end, 600)
    seen_pocket = set()
    for i, tr in enumerate(fleet.trajectories):
        for p in tr.sample(times):
            cell = (round(p[0] / PITCH), round(p[1] / PITCH))
            assert cell not in walls or math.dist(p[:2], np.array(cell) * PITCH) > 0.9 * PITCH
            if math.dist(p[:2], (2 * PITCH, PITCH)) < 1e-6:
                seen_pocket.add(i)
    assert len(seen_pocket) == 1


def test_known_incompleteness_pocket_between_start_and_goal(monkeypatch):
    # Prioritised planning is incomplete: with the pocket in the middle of a 3 cell corridor,
    # whichever robot is planned first takes the straight path and the other has nowhere to go.
    blocked = corridor_blocked(0, 2, (1, 1))
    show = lattice_show([(0, 0), (2, 0)], [(2, 0), (0, 0)], monkeypatch)
    with pytest.raises(G.GridPlanError) as e:
        G.plan_grid(show, assign="identity", margin=1, blocked_xy=blocked, restarts=8)
    assert "no grid plan" in str(e.value)


def test_impossible_swap_without_pocket_raises_clear_error(monkeypatch):
    blocked = corridor_blocked(0, 3, (99, 99))  # pocket outside the box: plain corridor
    show = lattice_show([(0, 0), (3, 0)], [(3, 0), (0, 0)], monkeypatch)
    with pytest.raises(G.GridPlanError) as e:
        G.plan_grid(show, assign="identity", margin=1, blocked_xy=blocked, restarts=6)
    assert e.value.cause in ("blocked_by_reservations", "enclosed_by_parked_robots")
    assert "no grid plan" in str(e.value)
    assert isinstance(e.value, RuntimeError)


@pytest.mark.parametrize("task,assign", [("shuffle", "random"), ("swap15", "pinned15")])
def test_crossing_tasks_16_verified(task, assign):
    show = grid_show(16, task)
    fleet, info = G.plan_grid(show, assign=assign)
    rep = verify(fleet, show["limits"])
    assert rep.ok
    assert info["transitions"][0]["discrete_conflicts"] == 0
    assert rep.min_separation >= show["limits"]["min_separation_m"]


def test_deterministic_for_fixed_seed():
    show = grid_show(16, "shuffle")
    f1, i1 = G.plan_grid(show, assign="random", seed=5)
    f2, i2 = G.plan_grid(show, assign="random", seed=5)
    assert f1.content_hash() == f2.content_hash()
    assert i1["transitions"] == i2["transitions"]


def test_discrete_conflict_checker_detects_swap_and_collision():
    nc = 20
    assert G.discrete_conflicts([[0, 1], [1, 0]], nc) == 1  # swap
    assert G.discrete_conflicts([[0, 1], [2, 1]], nc) == 1  # same cell at step 1
    assert G.discrete_conflicts([[0, 1], [1, 2]], nc) == 0  # following


def test_rejects_pitch_below_separation_and_off_lattice(monkeypatch):
    show = grid_show(16, "translate")
    with pytest.raises(ValueError, match="too small"):
        G.plan_grid(show, pitch=1.2)  # 1.2 / sqrt(2) < 1.0 m required
    A = np.array([[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
    B = np.array([[0.0, 0.0, 0.0], [2.5, 0.0, 0.0]])
    monkeypatch.setattr(G, "build_targets", lambda show: [A, B])
    with pytest.raises(ValueError, match="lattice"):
        G.plan_grid(new_show("x", 2, grid_show(2, "translate")["stages"], seed=1, limits=LIMITS),
                    assign="identity")
