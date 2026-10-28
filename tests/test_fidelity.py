"""Optional tracker limits (command latency, acceleration cap) in sirb.stress."""

import numpy as np

from sirb.disturbance import StressConfig
from sirb.stress import track, track_limited


def _inputs(K=200, n=3, seed=3):
    rng = np.random.default_rng(seed)
    e = 0.3 * rng.standard_normal((K, n, 3))
    wind = 4.0 * rng.standard_normal((K, n, 3))
    return e, wind


def test_unlimited_matches_linear_loop_closely():
    cfg = StressConfig(dt=0.05)
    e, wind = _inputs()
    kp = cfg.omega_n**2
    exact = track(np.zeros_like(e), -kp * e + cfg.drag_gain * wind, cfg)
    approx = track_limited(e, wind, cfg)
    assert np.abs(exact - approx).max() < 0.05 * max(np.abs(exact).max(), 1e-9)


def test_latency_increases_tracking_error():
    e, wind = _inputs()
    base = np.abs(track_limited(e, wind, StressConfig(dt=0.1, latency_s=0.0))).max()
    late = np.abs(track_limited(e, wind, StressConfig(dt=0.1, latency_s=0.4))).max()
    assert late > base


def test_accel_limit_caps_feedback_acceleration():
    cfg = StressConfig(dt=0.1, accel_limit=0.5)
    K = 60
    e = np.zeros((K, 1, 3))
    e[:, 0, 0] = 5.0                       # large constant position error: feedback would be 20 m/s^2
    wind = np.zeros((K, 1, 3))
    eps = track_limited(e, wind, cfg)
    vel = np.diff(eps[:, 0, :], axis=0) / cfg.dt
    acc = np.diff(vel, axis=0) / cfg.dt
    assert np.abs(acc).max() <= 0.5 + 1e-6


def test_default_config_keeps_limits_off():
    cfg = StressConfig()
    assert cfg.latency_s == 0.0 and cfg.accel_limit is None
