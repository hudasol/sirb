"""Regression tests for defects found in the critique rounds (docs/REVIEW_ROUNDS.md).

Each test names the round that found the defect.
"""

import copy
import json

import numpy as np
import pytest

from sirb import cli
from sirb.assign import linear_conflicts
from sirb.planner.baseline import plan_baseline
from sirb.spec import new_show, validate_show


def _show(n=12):
    return new_show("t", n, [
        {"name": "a", "shape": {"type": "launch_grid", "spacing_m": 4.0, "center": [0, -85, 0]},
         "hold_s": 1},
        {"name": "b", "shape": {"type": "sphere", "spacing_m": 4.0, "center": [0, 0, 60]},
         "hold_s": 2}], seed=3)


# ---- Round 1 (EDGE-style engineer) ---------------------------------------------------------

MALFORMED = {
    "geofence missing x": lambda s: s["limits"]["geofence"].pop("x"),
    "crowd missing y": lambda s: s["limits"]["crowd"].pop("y"),
    "geofence not an object": lambda s: s["limits"].__setitem__("geofence", [1, 2]),
    "stage not an object": lambda s: s["stages"].__setitem__(0, 5),
    "hold_s string": lambda s: s["stages"][0].__setitem__("hold_s", "2"),
    "z_max string": lambda s: s["limits"].__setitem__("z_max", "5"),
    "limits is a list": lambda s: s.__setitem__("limits", []),
    "stages is an object": lambda s: s.__setitem__("stages", {}),
    "inf limit": lambda s: s["limits"].__setitem__("amax", float("inf")),
    "nan limit": lambda s: s["limits"].__setitem__("vmax_h", float("nan")),
    "nan geofence": lambda s: s["limits"]["geofence"].__setitem__("x", [float("nan"), 3]),
    "threshold below vehicle diameter":
        lambda s: s["limits"].__setitem__("min_separation_m", 0.3),
    "vehicle not an object": lambda s: s.__setitem__("vehicle", 3),
}


@pytest.mark.parametrize("name", list(MALFORMED))
def test_round1_validator_rejects_malformed_without_raising(name):
    s = copy.deepcopy(_show())
    MALFORMED[name](s)
    errs = validate_show(s, check_targets=False)
    assert errs, name


def test_round1_validator_rejects_non_object():
    assert validate_show([1, 2]) == ["show must be a JSON object"]


def test_round1_valid_show_still_valid():
    assert validate_show(_show()) == []


def test_round1_linear_conflicts_block_equals_all_pairs():
    rng = np.random.default_rng(5)
    n = 150
    src = rng.uniform(0, 25, (n, 3))
    dst = rng.uniform(0, 25, (n, 3))
    perm = rng.permutation(n)
    v = dst[perm] - src
    # reference: explicit double loop with the closed-form minimum
    ref = 0
    for i in range(n):
        for j in range(i + 1, n):
            dp, dv = src[i] - src[j], v[i] - v[j]
            a = dv @ dv
            u = 0.0 if a < 1e-18 else min(max(-(dp @ dv) / a, 0.0), 1.0)
            if np.linalg.norm(dp + u * dv) < 3.0:
                ref += 1
    assert ref > 0
    for block in (7, 64, 256, 1000):
        assert linear_conflicts(src, dst, perm, 3.0, block=block) == ref


def test_round1_monte_carlo_serial_fallback_without_fork(monkeypatch):
    import sirb.stress as st
    from sirb.disturbance import StressConfig

    show = _show(10)
    fleet, _ = plan_baseline(show)
    ref = st.sample_reference(fleet, 0.5)
    cfg = StressConfig(dt=0.5, w20=3.0)
    a = st.monte_carlo(ref, cfg, 2.0, 4, 1, workers=1)
    monkeypatch.setattr(st, "_fork_available", lambda: False)
    b = st.monte_carlo(ref, cfg, 2.0, 4, 1, workers=2)   # would raise ValueError on Windows
    assert a["min_separation_quantiles"] == b["min_separation_quantiles"]


def test_round1_cli_validate_plan_check(tmp_path, capsys):
    from sirb.spec import save_show
    sp, out = tmp_path / "s.json", tmp_path / "e.json"
    save_show(_show(), sp)
    assert cli.main(["validate", str(sp)]) == 0
    assert cli.main(["plan", str(sp), "--out", str(out)]) == 0
    assert cli.main(["check", str(out)]) == 0
    text = capsys.readouterr().out
    assert "not an approval" in text and "PASS" in text
    b = json.loads(out.read_text())
    b["fleet"]["drones"][0][0]["coeffs"][0][0] += 1.0
    out.write_text(json.dumps(b))
    assert cli.main(["check", str(out)]) == 1          # tamper -> non-zero, no traceback
    assert cli.main(["validate", str(tmp_path / "missing.json")]) == 1


# ---- Round 2 (TII-style robotics researcher) -------------------------------------------------

def _seg(coeffs, t0, t1):
    from sirb.trajectory import Segment
    return Segment(t0, t1, np.array(coeffs, float))


def test_round2_acceleration_step_between_segments_is_flagged():
    """Velocity-continuous but acceleration-discontinuous path (infinite jerk) used to pass the
    jerk check, which only looks inside segments."""
    from sirb.spec import DEFAULT_LIMITS
    from sirb.trajectory import Fleet, Trajectory
    from sirb.verify import verify

    # seg 1: x = 0.5 t^2 (a = 1, v(1) = 1); seg 2: x = 0.5 + t (a = 0, v(0) = 1)
    s1 = _seg([[0, 0, 20], [0, 0, 0], [0.5, 0, 0]], 0.0, 1.0)
    s2 = _seg([[0.5, 0, 20], [1.0, 0, 0]], 1.0, 2.0)
    rep = verify(Fleet([Trajectory([s1, s2])]), DEFAULT_LIMITS)
    assert {v.kind for v in rep.violations} == {"accel_discontinuity"}
    assert not rep.ok
    # a smooth chain (a = 1 on both sides) is accepted
    s2b = _seg([[0.5, 0, 20], [1.0, 0, 0], [0.5, 0, 0]], 1.0, 2.0)
    assert verify(Fleet([Trajectory([s1, s2b])]), DEFAULT_LIMITS).ok


def test_round2_planner_output_has_no_acceleration_steps():
    from sirb.spec import DEFAULT_LIMITS
    from sirb.verify import verify

    show = _show(20)
    fleet, _ = plan_baseline(show)
    rep = verify(fleet, show["limits"])
    assert rep.ok and "accel_discontinuity" not in rep.violation_counts
    assert DEFAULT_LIMITS["jmax"] > 0


def test_round2_rejoin_default_is_unrestricted_and_cap_is_opt_in():
    from sirb.contingency import rejoin_reference
    from sirb.disturbance import StressConfig

    dt, K = 0.2, 600
    t = np.arange(K) * dt
    nom = np.stack([np.clip(10 * (t - 10), 0, 100), 0 * t, 0 * t + 50], axis=1)  # 100 m in 10 s
    k0, k1 = int(10 / dt), int(20 / dt)

    def peak(vmax):
        cfg = StressConfig(dt=dt, rejoin_vmax=vmax)
        R = nom.copy()
        rejoin_reference(nom, R, k0, k1, cfg)
        return float((np.linalg.norm(np.diff(R, axis=0), axis=1) / dt).max()), R

    assert StressConfig().rejoin_vmax is None
    p_default, _ = peak(None)
    p_cap, R = peak(10.0)
    assert p_default > 25.0                  # 100 m gap closed in 6 s: far above any airframe limit
    assert p_cap <= 10.0 + 0.1
    assert np.linalg.norm(R[-1] - nom[-1]) < 1e-9   # still rejoins the nominal path


def test_round2_candidate_pair_pruning_never_hides_a_violation():
    """dev_bound is a heuristic (6-sigma style) pad. Check it against all-pairs on stressed runs."""
    import itertools

    from sirb.disturbance import StressConfig
    from sirb.stress import candidate_pairs, dev_bound, exact_pair_min, realize, sample_reference

    show = _show(30)
    fleet, _ = plan_baseline(show)
    dt = 0.2
    ref = sample_reference(fleet, dt)
    n = ref.shape[1]
    allp = np.array(list(itertools.combinations(range(n), 2)))
    cfgs = [dict(gnss_deg_fraction=1.0, gnss_bias_sigma=2.0, w20=6.0),
            dict(gnss_loss_fraction=0.4, behavior={"comms_loss": "continue", "battery_low": "descend",
                                                   "gnss_loss": "rejoin", "motor_failure": "ballistic"})]
    violating = 0
    for kw in cfgs:
        cfg = StressConfig(dt=dt).with_(**kw)
        for seed in range(2):
            X, R, *_ = realize(ref, cfg, [9, seed])
            full, _ = exact_pair_min(X, allp)
            truth = {tuple(p) for p in allp[full < 2.0]}
            violating += len(truth)
            pairs, klo, khi = candidate_pairs(R, 2.0 + dev_bound(cfg), dt=dt)
            pm, _ = exact_pair_min(X, pairs, klo, khi)
            assert truth <= {tuple(p) for p in pairs[pm < 2.0]}
    assert violating > 0     # the test must actually exercise violations


# ---- Round 3 (Al Ain University senior: docs-code consistency, claims vs evidence) -----------

def test_round3_monte_carlo_is_independent_of_worker_count():
    """README/METHODS say results are deterministic given the seed; the runner may use 2 workers."""
    import sirb.stress as st
    from sirb.disturbance import StressConfig

    show = _show(10)
    fleet, _ = plan_baseline(show)
    ref = st.sample_reference(fleet, 0.5)
    cfg = StressConfig(dt=0.5, w20=6.0, gnss_deg_fraction=0.5, gnss_bias_sigma=0.5)
    a = st.monte_carlo(ref, cfg, 2.0, 6, 3, workers=1)
    b = st.monte_carlo(ref, cfg, 2.0, 6, 3, workers=2)
    for k in ("p_violation", "p_violation_ci95", "min_separation_quantiles",
              "mean_max_tracking_error_m", "mean_violating_pairs"):
        assert a[k] == b[k], k


def test_round3_required_value_needs_all_larger_rows_to_qualify():
    from sirb.stress import required_value, wilson

    def row(v, k, n=300):
        return {"formation_spacing_m": v, "p_violation_ci95": list(wilson(k, n))}

    rows = [row(3.0, 120), row(4.0, 30), row(5.0, 0), row(6.0, 4), row(8.0, 0)]
    # 5.0 is clean, but 6.0 (4/300) has an upper bound above 2%, so only 8.0 qualifies at 2%
    assert required_value(rows, 0.02)["value"] == 8.0
    assert required_value(rows, 0.05)["value"] == 5.0
    # below the best attainable bound for N = 300 nothing can qualify
    assert required_value(rows, 0.001)["value"] is None


def test_round3_docs_list_the_shapes_and_letters_the_code_supports():
    import re
    from pathlib import Path

    from sirb.shapes import SUPPORTED_LETTERS
    from sirb.spec import SHAPE_TYPES

    doc = Path("docs/SHOW_SCHEMA.md").read_text(encoding="utf-8")
    line = next(ln for ln in doc.splitlines() if ln.startswith("Shape types:"))
    documented = set(re.findall(r"`([a-z_]+)`", line.split("Common keys")[0]))
    assert SHAPE_TYPES <= documented
    assert set(SUPPORTED_LETTERS) == set(re.findall(r"letters ([A-Z ]+)\)", line)[0].replace(" ", ""))


def test_round3_methods_quotes_current_stress_defaults():
    from pathlib import Path

    from sirb.disturbance import StressConfig

    cfg = StressConfig()
    doc = Path("docs/METHODS.md").read_text(encoding="utf-8")
    for needle in (f"omega_n = {cfg.omega_n:g} rad/s", f"zeta = {cfg.zeta:g}",
                   f"drag_gain = {cfg.drag_gain:g}", f"{cfg.gnss_white_sigma:g} m per axis",
                   f"{cfg.gnss_dead_reckoning_rate:g} m/sqrt(s)",
                   f"decay {cfg.air_drag_per_s:g} /s"):
        assert needle in doc, needle
    assert "2.0 m centre-to-centre separation threshold is an **ASSUMPTION**" in doc


def test_round3_spec_labels_separation_threshold_as_assumption():
    from sirb.spec import DEFAULT_LIMITS
    assert DEFAULT_LIMITS["sources"]["min_separation_m"] == "assumption"
