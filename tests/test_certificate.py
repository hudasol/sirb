import copy
import json

import numpy as np

from sirb.certificate import make_certificate, verify_certificate
from sirb.spec import DEFAULT_LIMITS
from sirb.trajectory import Fleet, Segment, Trajectory
from sirb.verify import verify

LIM = DEFAULT_LIMITS


def lin(p, v, t0, t1):
    return Segment(t0, t1, np.array([p, v], float))


def fleet(off=5.0):
    return Fleet([Trajectory([lin([-10, 0, 20], [2, 0, 0], 0, 10)]),
                  Trajectory([lin([10, off, 20], [-2, 0, 0], 0, 10)])])


def test_roundtrip_pass_and_fail():
    for off, verdict in [(5.0, "PASS"), (0.5, "FAIL")]:
        f = fleet(off)
        cert = make_certificate(verify(f, LIM), f, LIM, extra={"show": "demo"})
        assert cert["verdict"] == verdict
        assert cert["fleet_sha256"] == f.content_hash()
        back = json.loads(json.dumps(cert, allow_nan=False))
        assert verify_certificate(back, f)
        assert cert["extra"] == {"show": "demo"}
        assert "not flight-certified" in cert["statement"]
        assert cert["not_guaranteed"] and cert["guarantees"]


def test_hash_changes_on_modification_and_cert_rejects():
    f = fleet()
    cert = make_certificate(verify(f, LIM), f, LIM)
    g = fleet()
    g.trajectories[0].segments[0].coeffs[0, 0] += 1e-6
    assert g.content_hash() != f.content_hash()
    assert not verify_certificate(cert, g)


def test_tampered_certificate_rejected():
    f = fleet(0.5)
    cert = make_certificate(verify(f, LIM), f, LIM)
    bad = copy.deepcopy(cert)
    bad["verdict"] = "PASS"
    assert not verify_certificate(bad, f)
    bad = copy.deepcopy(cert)
    bad["results"]["min_separation"] = 3.0
    assert not verify_certificate(bad, f)


def test_json_serialisable_deterministic_no_timestamp():
    f = fleet()
    c1 = make_certificate(verify(f, LIM), f, LIM)
    c2 = make_certificate(verify(f, LIM), f, LIM)
    s1 = json.dumps(c1, sort_keys=True, allow_nan=False)
    assert s1 == json.dumps(c2, sort_keys=True, allow_nan=False)
    def keys(x):
        if isinstance(x, dict):
            for k, v in x.items():
                yield k.lower()
                yield from keys(v)
        elif isinstance(x, list):
            for v in x:
                yield from keys(v)
    for k in keys(c1):
        assert not any(w in k for w in ("time", "date", "created", "runtime")) or k in (
            "min_separation_time", "t", "time_contiguity_s"), k
    assert c1["tool"] == "sirb.verify" and c1["certificate_version"] == 2
    assert c1["n_drones"] == 2 and c1["limits"] == json.loads(json.dumps(LIM))


def test_numpy_limits_serialise():
    lim = copy.deepcopy(LIM)
    lim["vmax_h"] = np.float64(10.0)
    lim["geofence"]["x"] = np.array([-150.0, 150.0])
    f = fleet()
    cert = make_certificate(verify(f, lim), f, lim)
    json.dumps(cert, allow_nan=False)
    assert verify_certificate(json.loads(json.dumps(cert)), f)


def test_loosened_tolerances_cannot_turn_a_fail_into_a_pass():
    f = fleet()
    cert = make_certificate(verify(f, LIM), f, LIM)
    assert verify_certificate(cert, f)
    for key, val in (("tol", 1e-3), ("position_continuity_m", 1.0), ("velocity_continuity_mps", 1.0),
                     ("acceleration_continuity_mps2", 1.0), ("search_radius", 0.5)):
        bad = copy.deepcopy(cert)
        bad["results"]["tolerances"][key] = val
        assert not verify_certificate(bad, f), key


def test_robustness_and_clearance_fields():
    f = fleet()
    rep = verify(f, LIM)
    c = make_certificate(rep, f, LIM, vehicle_radius_m=0.24)
    assert c["separation_robustness_m"] == rep.min_separation - LIM["min_separation_m"]
    assert abs(c["vehicle"]["surface_clearance_m"] - (rep.min_separation - 0.48)) < 1e-12
    assert "not an approval" in c["meaning"]
    assert make_certificate(rep, f, LIM)["vehicle"] is None


def test_changing_any_safety_input_invalidates_the_record(tmp_path):
    import json as _json

    from sirb.export import build_export, load_export, save_export
    from sirb.planner.baseline import plan_baseline
    from sirb.spec import load_show

    show = load_show("shows/sirb_text_100.json")
    fleet_, _ = plan_baseline(show)
    path = tmp_path / "e.json"
    save_export(path, build_export(show, fleet_))
    load_export(path)                                  # unmodified: loads
    base = _json.loads(path.read_text())
    edits = {
        "limit": lambda b: (b["show"]["limits"].__setitem__("min_separation_m", 1.0)),
        "vehicle": lambda b: b["show"]["vehicle"].__setitem__("radius_m", 0.1),
        "coefficient": lambda b: b["fleet"]["drones"][0][0]["coeffs"][0].__setitem__(0, 99.0),
        "verdict": lambda b: b["certificate"].__setitem__("verdict", "FAIL"),
    }
    for name, edit in edits.items():
        b = _json.loads(_json.dumps(base))
        edit(b)
        p = tmp_path / f"{name}.json"
        p.write_text(_json.dumps(b))
        try:
            load_export(p)
        except (ValueError, KeyError, TypeError):
            continue
        raise AssertionError(f"edit {name!r} was not detected")
