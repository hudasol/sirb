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
