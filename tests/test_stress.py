import math

import numpy as np
import pytest

from sirb import contingency as C
from sirb import disturbance as D
from sirb.planner.baseline import plan_baseline
from sirb.spec import new_show
from sirb.stress import (
    _zoh,
    ballistic_track,
    exact_pair_min,
    monte_carlo,
    run_trial,
    sample_reference,
    track,
    wilson,
)
from sirb.verify import verify

DT = 0.2


def small_show(n=30, spacing=3.2):
    grid = {"type": "launch_grid", "spacing_m": spacing, "center": [0.0, -85.0, 0.0]}
    return new_show("small", n, [
        {"name": "launch", "shape": grid, "hold_s": 2},
        {"name": "orb", "shape": {"type": "sphere", "spacing_m": spacing, "center": [0, 0, 60]},
         "hold_s": 3},
        {"name": "land", "shape": grid, "hold_s": 2},
    ], seed=5)


@pytest.fixture(scope="module")
def planned():
    show = small_show()
    fleet, _ = plan_baseline(show)
    return show, fleet, sample_reference(fleet, DT)


def test_noise_free_matches_exact_verifier(planned):
    show, fleet, ref = planned
    rep = verify(fleet, show["limits"])
    cfg = D.StressConfig(dt=DT, gnss_white_sigma=0.0)
    tr = run_trial(ref, cfg, 2.0, 1)
    assert tr.min_separation == pytest.approx(rep.min_separation, abs=1e-6)
    assert tr.max_tracking_error == 0.0


def test_exact_pair_min_matches_dense_sampling():
    rng = np.random.default_rng(0)
    K, n = 12, 6
    X = np.cumsum(rng.standard_normal((K, n, 3)) * 2, axis=0)
    pairs = np.array([(i, j) for i in range(n) for j in range(i + 1, n)])
    pmin, _ = exact_pair_min(X, pairs)
    u = np.linspace(0, 1, 4001)
    for p, (i, j) in enumerate(pairs):
        best = np.inf
        for k in range(K - 1):
            a = X[k, i] + u[:, None] * (X[k + 1, i] - X[k, i])
            b = X[k, j] + u[:, None] * (X[k + 1, j] - X[k, j])
            best = min(best, np.linalg.norm(a - b, axis=1).min())
        assert pmin[p] <= best + 1e-12
        assert pmin[p] == pytest.approx(best, abs=2e-2)


def test_zoh_matches_fine_euler():
    cfg = D.StressConfig(dt=0.2)
    ref = np.zeros((26, 1, 3))
    u = np.zeros((26, 1, 3))
    u[:, 0, 0] = 1.0
    eps = track(ref, u, cfg)[:, 0, 0]
    kp, kd = cfg.omega_n**2, 2 * cfg.zeta * cfg.omega_n
    x = v = 0.0
    h = 1e-4
    for _ in range(int(5.0 / h)):
        a = -kd * v - kp * x + 1.0
        x, v = x + h * v, v + h * a
    assert eps[-1] == pytest.approx(x, abs=2e-3)
    Ad, _ = _zoh(cfg)
    assert abs(np.linalg.eigvals(Ad)).max() < 1.0


def test_determinism_and_seed_sensitivity(planned):
    _, _, ref = planned
    cfg = D.StressConfig(dt=DT, gnss_deg_fraction=0.5, gnss_bias_sigma=0.5, w20=6.0)
    a = run_trial(ref, cfg, 2.0, [1, 2, 3])
    b = run_trial(ref, cfg, 2.0, [1, 2, 3])
    c = run_trial(ref, cfg, 2.0, [1, 2, 4])
    assert a.min_separation == b.min_separation
    assert a.min_separation != c.min_separation


def test_risk_increases_with_gnss_differential_error(planned):
    _, _, ref = planned
    base = D.StressConfig(dt=DT, gnss_deg_fraction=0.25)
    lo = monte_carlo(ref, base.with_(gnss_bias_sigma=0.0), 2.0, 8, 1)
    hi = monte_carlo(ref, base.with_(gnss_bias_sigma=3.0), 2.0, 8, 1)
    assert lo["p_violation"] == 0.0
    assert hi["p_violation"] == 1.0


def test_wind_differential_between_neighbours_is_correlated():
    ref = np.zeros((600, 2, 3))
    ref[:, 0, :] = [0, 0, 60]
    ref[:, 1, :] = [3.0, 0, 60]
    cfg = D.StressConfig(dt=0.1, w20=6.0, wind_dir_deg=0.0)
    w = D.wind_velocity(ref, cfg, np.random.default_rng(3))
    a, b = w[:, 0, 0], w[:, 1, 0]
    assert np.corrcoef(a, b)[0, 1] > 0.5
    assert np.std(a - b) < np.std(a)
    zero = D.wind_velocity(ref, D.StressConfig(dt=0.1, w20=0.0), np.random.default_rng(3))
    assert not zero.any()


def test_dryden_values():
    su, sw = D.dryden_sigmas(10.0, 60.0)
    assert sw == pytest.approx(1.0)
    assert su == pytest.approx(1.0 / (0.177 + 0.000823 * (60 / 0.3048)) ** 0.4)


def test_wilson_interval():
    lo, hi = wilson(0, 300)
    assert lo == 0.0 and 0.01 < hi < 0.015
    lo, hi = wilson(150, 300)
    assert lo < 0.5 < hi


def test_ballistic_fall_hits_ground_and_drone_below():
    cfg = D.StressConfig(dt=0.1)
    K = 200
    x_before = np.array([0.0, 0.0, 30.1])
    x_at = np.array([0.0, 0.0, 30.0])
    fall = ballistic_track(x_before, x_at, K, 10, cfg)
    assert fall[0, 2] == pytest.approx(30.0)
    assert fall[-1, 2] == 0.0
    assert (np.diff(fall[:, 2]) <= 1e-12).all()


def test_ballistic_fall_causes_violation_on_toy_column():
    # drone 0 above drone 1 in a hovering column; failure of 0 must produce a violation
    K = 200
    ref = np.zeros((K, 2, 3))
    ref[:, 0, :] = [0, 0, 40]
    ref[:, 1, :] = [0, 0, 36]
    cfg = D.StressConfig(dt=0.1, motor_failures=1, gnss_white_sigma=0.0)
    hits = [run_trial(ref, cfg, 2.0, [1, 1, t]) for t in range(10)]
    upper = [h for h in hits if h.events[0]["drone"] == 0]
    lower = [h for h in hits if h.events[0]["drone"] == 1]
    assert upper and lower
    assert all(h.violated and h.contact and h.pair == (0, 1) for h in upper)
    assert not any(h.violated for h in lower)   # the lower drone falls away from the upper one


def test_descend_and_hold_references():
    cfg = D.StressConfig(dt=0.1, descent_rate=2.0, hold_s=1.0)
    K = 100
    ref = np.zeros((K, 1, 3))
    ref[:, 0, :] = np.array([5.0, 6.0, 30.0]) + np.arange(K)[:, None] * 0.01
    r = ref.copy()
    C.descend_reference(r[:, 0, :], 20, cfg)
    assert r[19, 0, 2] == pytest.approx(ref[19, 0, 2])
    assert r[30, 0, 2] == pytest.approx(ref[20, 0, 2] - 2.0)
    assert (r[20:, 0, 0] == ref[20, 0, 0]).all()
    assert r[-1, 0, 2] >= 0.0
    h = ref.copy()
    C.hold_reference(h[:, 0, :], 20, cfg)
    assert (h[20:30, 0, :] == ref[20, 0, :]).all()
    assert h[31, 0, 2] < ref[20, 0, 2]


def test_corridor_picks_the_free_direction():
    K, n = 60, 9
    ref = np.zeros((K, n, 3))
    ref[:, 0, :] = [0, 0, 50]
    angles = np.arange(8) * math.pi / 4
    for j in range(1, 8):  # block seven directions with hovering neighbours 6 m away
        a = angles[j]
        ref[:, j, :] = [6 * math.cos(a), 6 * math.sin(a), 50]
    ref[:, 8, :] = [500, 500, 50]
    cfg = D.StressConfig(dt=0.1)
    u = C.corridor_direction(ref, 0, 5, cfg)
    assert np.allclose(u, [math.cos(angles[0]), math.sin(angles[0]), 0.0], atol=1e-9)


def test_rejoin_returns_to_nominal():
    cfg = D.StressConfig(dt=0.1, rejoin_s=2.0)
    K = 100
    nominal = np.zeros((K, 3))
    nominal[:, 0] = np.arange(K) * 0.5
    r = nominal.copy()
    C.rejoin_reference(nominal, r, 10, 30, cfg)
    assert np.allclose(r[10:30], nominal[10])
    assert np.allclose(r[60:], nominal[60:])
    assert np.allclose(r[49], nominal[49])


def test_unknown_behavior_rejected(planned):
    _, _, ref = planned
    cfg = D.StressConfig(dt=DT, comms_loss_fraction=0.5).with_(
        behavior={"comms_loss": "teleport", "battery_low": "descend", "gnss_loss": "continue",
                  "motor_failure": "ballistic"})
    with pytest.raises(ValueError):
        run_trial(ref, cfg, 2.0, 1)
