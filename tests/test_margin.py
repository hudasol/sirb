"""Margin synthesis (sirb.margin): search logic with a synthetic evaluator, scaling, one real run."""

import math

import pytest

from sirb.disturbance import StressConfig
from sirb.margin import MODEL_DEPENDENCE, scale_show, synthesise, wilson_floor
from sirb.spec import new_show
from sirb.stress import wilson


def _show(n=9, spacing=4.0):
    grid = {"type": "launch_grid", "spacing_m": spacing, "center": [0.0, -85.0, 0.0]}
    return new_show("margin test", n, [
        {"name": "a", "shape": grid, "hold_s": 1},
        {"name": "b", "shape": {"type": "sphere", "spacing_m": spacing, "center": [0, 0, 30]},
         "hold_s": 1},
        {"name": "c", "shape": grid, "hold_s": 1}], seed=3)


def _fake(threshold, nominal1=2.5, n=200, verify_up_to=math.inf, lucky=()):
    """Evaluator whose risk is 0 at or above ``threshold`` and 1 below it (plus lucky factors)."""
    calls = []

    def ev(f):
        calls.append(f)
        k = 0 if (f >= threshold or round(f, 6) in lucky) else n
        return {"factor": f, "verified_ok": f <= verify_up_to,
                "nominal_min_separation_m": nominal1 * f, "p_violation": k / n,
                "p_violation_ci95": list(wilson(k, n)), "trials": n}
    return ev, calls


def test_scale_show_scales_spacing_and_size_only():
    s = _show()
    s["stages"][1]["shape"] = {"type": "torus", "size": 10.0, "center": [1, 2, 30]}
    t = scale_show(s, 1.5)
    assert t["stages"][0]["shape"]["spacing_m"] == 6.0
    assert t["stages"][1]["shape"]["size"] == 15.0
    assert t["stages"][1]["shape"]["center"] == [1, 2, 30]
    assert s["stages"][0]["shape"]["spacing_m"] == 4.0   # input untouched


@pytest.mark.parametrize("thr", [0.9, 1.0, 1.37, 2.2])
def test_search_returns_upper_end_within_tolerance(thr):
    ev, _ = _fake(thr)
    r = synthesise(_show(), StressConfig(), evaluator=ev, tol=0.02)
    assert r["status"] == "ok"
    assert thr <= r["factor"] <= thr * 1.02 + 1e-9 or r["factor"] == 1.0 and thr <= 1.0
    assert r["required_nominal_min_separation_m"] == pytest.approx(2.5 * r["factor"])
    assert r["required_margin_m"] == pytest.approx(2.5 * r["factor"] - 2.0)
    assert r["model_dependence"] == MODEL_DEPENDENCE


def test_lucky_row_is_not_accepted():
    # factor 1.0 passes by luck; 1.02 (the guard) fails, so the search must move above it
    ev, _ = _fake(1.6, lucky=(1.0,))
    r = synthesise(_show(), StressConfig(), evaluator=ev, tol=0.02)
    assert r["status"] == "ok" and r["factor"] >= 1.6


def test_reports_failure_when_plan_stops_verifying():
    ev, _ = _fake(3.0, verify_up_to=2.0)
    r = synthesise(_show(), StressConfig(), evaluator=ev)
    assert r["factor"] is None and "stops verifying" in r["status"]
    assert r["required_nominal_min_separation_m"] is None


def test_reports_failure_beyond_max_factor():
    ev, _ = _fake(10.0)
    r = synthesise(_show(), StressConfig(), evaluator=ev, max_factor=3.0)
    assert r["factor"] is None and "no factor" in r["status"]


def test_target_range_checked():
    with pytest.raises(ValueError):
        synthesise(_show(), StressConfig(), target=0.0, evaluator=_fake(1.0)[0])


def test_wilson_floor_matches_wilson_with_zero_violations():
    for n in (50, 200, 300):
        assert wilson_floor(n) == pytest.approx(wilson(0, n)[1])


def test_real_small_run_meets_target_and_is_deterministic():
    cfg = StressConfig(dt=0.2, w20=3.0, gnss_deg_fraction=0.25, gnss_bias_sigma=0.3)
    a = synthesise(_show(), cfg, trials=30, tol=0.05, target=0.15)
    b = synthesise(_show(), cfg, trials=30, tol=0.05, target=0.15)
    assert a == b
    assert a["status"] == "ok"
    chosen = [r for r in a["evaluations"] if r["factor"] == round(a["factor"], 6)][0]
    assert chosen["verified_ok"] and chosen["p_violation_ci95"][1] <= 0.15
    assert a["required_nominal_min_separation_m"] >= a["d_min"]


def test_cli_margin_writes_report_and_export(tmp_path):
    import json

    from sirb.cli import main
    show = tmp_path / "s.json"
    show.write_text(json.dumps(_show()))
    rep, out = tmp_path / "r.json", tmp_path / "e.json"
    rc = main(["margin", str(show), "--gnss", "shows/gnss_assumed_default.json", "--trials", "10",
               "--target", "0.3", "--report", str(rep), "--out", str(out)])
    assert rc == 0
    r = json.loads(rep.read_text())
    assert r["status"] == "ok" and r["layer"].startswith("simulated")
    assert json.loads(out.read_text())["certificate"]["verdict"] == "PASS"
