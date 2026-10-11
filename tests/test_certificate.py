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
    assert c1["tool"] == "sirb.verify" and c1["certificate_version"] == 1
    assert c1["n_drones"] == 2 and c1["limits"] == json.loads(json.dumps(LIM))


def test_numpy_limits_serialise():
    lim = copy.deepcopy(LIM)
    lim["vmax_h"] = np.float64(10.0)
    lim["geofence"]["x"] = np.array([-150.0, 150.0])
    f = fleet()
    cert = make_certificate(verify(f, lim), f, lim)
    json.dumps(cert, allow_nan=False)
    assert verify_certificate(json.loads(json.dumps(cert)), f)
