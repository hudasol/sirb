import copy

import numpy as np
import pytest

from sirb.planner.baseline import plan_baseline
from sirb.preflight import (
    fleet_plan_hashes,
    nominal_declaration,
    preflight,
    transform_fleet,
)
from sirb.spec import load_show
from sirb.trajectory import Fleet
from sirb.verify import verify


@pytest.fixture(scope="module")
def pack():
    show = load_show("shows/sirb_text_100.json")
    fleet, _ = plan_baseline(show)
    return show, fleet


def test_clean_deployment_passes(pack):
    show, fleet = pack
    declared, loaded = nominal_declaration(show, fleet)
    rep = preflight(show, fleet, declared, loaded, wind_forecast_mps=4, wind_limit_mps=9)
    assert rep.ok, [x.message for x in rep.findings if x.level == "FAIL"]
    assert "NOT_CHECKED" in rep.codes("INFO")


@pytest.mark.parametrize("perturb,code", [
    (lambda d, l: l.update(center_xy=[5.0, 0.0]), "CENTER_MISMATCH"),
    (lambda d, l: l.update(heading_deg=7.0), "HEADING_MISMATCH"),
    (lambda d, l: l.update(geofence_buffer_m=1.0), "GEOFENCE_BUFFER_MISMATCH"),
    (lambda d, l: l.update(geofence={"x": [-150, 150], "y": [-130, 150]}), "GEOFENCE_MISMATCH"),
    (lambda d, l: l["aircraft_plan_sha256"].__setitem__(3, None), "PLAN_NOT_RECEIVED"),
    (lambda d, l: l["aircraft_plan_sha256"].__setitem__(4, "deadbeef"), "PLAN_HASH_MISMATCH"),
    (lambda d, l: l.update(aircraft_plan_sha256=l["aircraft_plan_sha256"][:-2]),
     "AIRCRAFT_COUNT_MISMATCH"),
])
def test_orlando_style_perturbations_are_caught(pack, perturb, code):
    show, fleet = pack
    declared, loaded = nominal_declaration(show, fleet)
    declared, loaded = copy.deepcopy(declared), copy.deepcopy(loaded)
    perturb(declared, loaded)
    rep = preflight(show, fleet, declared, loaded)
    assert not rep.ok
    assert code in rep.codes("FAIL")


def test_two_aircraft_swapped_plans_detected(pack):
    show, fleet = pack
    declared, loaded = nominal_declaration(show, fleet)
    h = loaded["aircraft_plan_sha256"]
    h[10], h[11] = h[11], h[10]
    rep = preflight(show, fleet, declared, loaded)
    assert "PLAN_HASH_MISMATCH" in rep.codes("FAIL")


def test_shift_toward_crowd_violates_after_placement(pack):
    show, fleet = pack
    declared, loaded = nominal_declaration(show, fleet)
    loaded["center_xy"] = [0.0, -60.0]          # shifts the whole show toward the crowd
    rep = preflight(show, fleet, declared, loaded)
    assert "PLACED_PLAN_VIOLATES_LIMITS" in rep.codes("FAIL")


def test_wind_limits(pack):
    show, fleet = pack
    declared, loaded = nominal_declaration(show, fleet)
    assert "WIND_ABOVE_LIMIT" in preflight(show, fleet, declared, loaded, wind_forecast_mps=10,
                                           wind_limit_mps=9).codes("FAIL")
    assert "WIND_NEAR_LIMIT" in preflight(show, fleet, declared, loaded, wind_forecast_mps=8,
                                          wind_limit_mps=9).codes("WARN")


def test_transform_is_exact_rigid_motion(pack):
    show, fleet = pack
    moved = transform_fleet(fleet, dx=3.0, dy=-2.0, rot_deg=30.0, about=(10.0, 5.0))
    t = 40.0
    p, q = fleet.positions(t), moved.positions(t)
    th = np.radians(30.0)
    R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    exp = (p[:, :2] - [10, 5]) @ R.T + [10, 5] + [3.0, -2.0]
    assert np.allclose(q[:, :2], exp, atol=1e-9)
    assert np.allclose(q[:, 2], p[:, 2])
    # rigid motion preserves separation exactly
    assert verify(moved, {**show["limits"], "geofence": {"x": [-1e4, 1e4], "y": [-1e4, 1e4]},
                          "crowd": {"y": -1e5, "min_distance_m": 1.0}}).min_separation == \
        pytest.approx(verify(fleet, show["limits"]).min_separation, abs=1e-6)


def test_hashes_unique_and_stable(pack):
    _, fleet = pack
    h = fleet_plan_hashes(fleet)
    assert len(set(h)) == len(h)
    assert h == fleet_plan_hashes(Fleet.from_dict(fleet.to_dict()))
