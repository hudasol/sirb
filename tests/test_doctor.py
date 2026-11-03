"""Doctor: structure of the diagnosis and its consistency with the stress simulation."""

import dataclasses

import numpy as np

from sirb.disturbance import StressConfig
from sirb.doctor import FORMAT, diagnose, trial_pairs
from sirb.planner.baseline import plan_baseline
from sirb.spec import new_show
from sirb.stress import monte_carlo, run_trial, sample_reference


def _show():
    grid = {"type": "launch_grid", "spacing_m": 2.4, "center": [0.0, -40.0, 0.0]}
    return new_show("doctor test", 12, [
        {"name": "launch", "shape": grid, "hold_s": 1},
        {"name": "orb", "shape": {"type": "sphere", "spacing_m": 2.4, "center": [0, 0, 40]},
         "hold_s": 2},
        {"name": "land", "shape": grid, "hold_s": 1}], seed=3)


CFG = StressConfig(dt=0.2, w20=6.0, gnss_bias_sigma=0.8, gnss_deg_fraction=0.5)


def test_diagnosis_agrees_with_monte_carlo():
    show = _show()
    fleet, _ = plan_baseline(show)
    d = diagnose(show, fleet, CFG, trials=30, seed=11)
    ref = sample_reference(fleet, CFG.dt)
    mc = monte_carlo(ref, CFG, show["limits"]["min_separation_m"], 30, 11)
    assert d["format"] == FORMAT
    assert d["stress"]["p_violation"] == mc["p_violation"]
    assert d["fleet_sha256"] == fleet.content_hash()
    assert d["environment"]["numpy"] and d["environment"]["seeds"]["stress_seed"] == 11


def test_worst_pair_matches_run_trial():
    show = _show()
    fleet, _ = plan_baseline(show)
    ref = sample_reference(fleet, CFG.dt)
    d_min = show["limits"]["min_separation_m"]
    for t in range(5):
        seed = [11, 0, t]
        r = run_trial(ref, CFG, d_min, seed)
        rows = trial_pairs(ref, CFG, d_min, seed)["rows"]
        assert bool(rows) == r.violated
        if rows:
            assert min(x[3] for x in rows) == r.min_separation
            assert len(rows) == r.violating_pairs


def test_events_and_shares_are_consistent():
    show = _show()
    fleet, _ = plan_baseline(show)
    d = diagnose(show, fleet, CFG, trials=30, seed=11)["stress"]
    assert d["violation_events"] >= sum(r["violating_trials"] for r in d["pair_ranking"])
    if d["violation_events"]:
        assert abs(sum(w["events"] for w in d["time_windows"]) / d["violation_events"]) <= 1.0
        assert 0.0 <= d["coverage_by_exact_rank"]["top_10_exact_share_of_events"] <= 1.0


def test_profile_is_recorded_and_plan_unchanged():
    show = _show()
    fleet, _ = plan_baseline(show)
    h = fleet.content_hash()
    d = diagnose(show, fleet, CFG, trials=5, seed=1)
    assert d["stress"]["profile"] == dataclasses.asdict(CFG)
    assert fleet.content_hash() == h
    assert "relative GNSS error" in d["notice"]
    assert np.isfinite(d["exact"]["min_separation_m"])
