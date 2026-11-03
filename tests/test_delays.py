from sirb.planner.delays import plan_delay_cbs, precedence_layers
from sirb.spec import build_targets, new_show
from sirb.verify import verify


def _show(n=12):
    g = {"type": "launch_grid", "spacing_m": 3.2, "center": [0, -60, 0]}
    return new_show("delay test", n, [
        {"name": "launch", "shape": g, "hold_s": 1},
        {"name": "orb", "shape": {"type": "sphere", "spacing_m": 3.2, "center": [0, 0, 40]}, "hold_s": 2}],
        seed=1)


def test_conflict_free_assignment_needs_no_delay():
    show = _show()
    fleet, info = plan_delay_cbs(show, assign="hungarian_sq")
    assert info["unresolved_pairs"] == 0
    assert verify(fleet, show["limits"]).ok
    assert info["transitions"][0]["max_delay_steps"] == 0


def test_unresolved_conflicts_are_reported_not_hidden():
    show = _show(30)
    fleet, info = plan_delay_cbs(show, assign="random")
    rep = verify(fleet, show["limits"])
    assert rep.ok or info["unresolved_pairs"] > 0


def test_precedence_layers_order_a_blocking_pair():
    import numpy as np
    src = np.array([[0.0, 0, 0], [0.0, 0, 5]])
    dst = np.array([[0.0, 0, 20], [0.0, 0, 30]])       # drone 0 flies through where drone 1 starts
    layer = precedence_layers(src, dst, 2.05)
    assert layer[1] < layer[0]
    assert build_targets(_show())[0].shape[0] == 12
