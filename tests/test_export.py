import json

import pytest

from sirb.export import LABEL, build_export, load_export, save_export
from sirb.planner.baseline import plan_baseline
from sirb.spec import new_show


def _show():
    return new_show("t", 12, [
        {"name": "a", "shape": {"type": "launch_grid", "spacing_m": 4.0, "center": [0, -85, 0]},
         "hold_s": 1},
        {"name": "b", "shape": {"type": "sphere", "spacing_m": 4.0, "center": [0, 0, 60]},
         "hold_s": 2}], seed=3)


def test_roundtrip_and_label(tmp_path):
    show = _show()
    fleet, _ = plan_baseline(show)
    p = tmp_path / "e.json"
    save_export(p, build_export(show, fleet))
    assert json.loads(p.read_text())["label"] == LABEL
    _, f2, cert = load_export(p)
    assert f2.content_hash() == fleet.content_hash() and cert["verdict"] == "PASS"


def test_tamper_detected(tmp_path):
    show = _show()
    fleet, _ = plan_baseline(show)
    p = tmp_path / "e.json"
    save_export(p, build_export(show, fleet))
    b = json.loads(p.read_text())
    b["fleet"]["drones"][0][0]["coeffs"][0][0] += 0.5
    p.write_text(json.dumps(b))
    with pytest.raises(ValueError):
        load_export(p)


def test_label_required(tmp_path):
    show = _show()
    fleet, _ = plan_baseline(show)
    p = tmp_path / "e.json"
    b = build_export(show, fleet)
    b["label"] = "approved"
    save_export(p, b)
    with pytest.raises(ValueError):
        load_export(p)
