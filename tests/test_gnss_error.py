"""Direct tests of the GNSS error model (sirb.disturbance.gnss_error).

Every stressed verdict depends on the bias term (docs/SENSITIVITY.md), so its statistics are
checked against theory: per axis standard deviation equal to bias_sigma, the bias only on degraded
drones, the share of degraded drones, the first order Gauss-Markov autocorrelation exp(-lag/tau),
white noise at white_sigma, and dropout windows of the stated length. Fixed seeds; the tolerances
are several standard errors of each estimate, derived in the comments.
"""

import math

import numpy as np
import pytest

from sirb.disturbance import StressConfig, gnss_error


def _cfg(**kw):
    base = dict(dt=0.2, gnss_white_sigma=0.0, gnss_deg_fraction=0.0, gnss_bias_sigma=0.0,
                gnss_bias_tau=30.0, gnss_loss_fraction=0.0)
    return StressConfig(**{**base, **kw})


def test_bias_only_on_degraded_drones():
    e, info = gnss_error(20, 400, _cfg(gnss_deg_fraction=0.5, gnss_bias_sigma=1.0),
                         np.random.default_rng(1))
    deg = info["degraded"]
    assert 0 < deg.sum() < len(deg)
    assert np.all(e[:, ~deg, :] == 0.0)          # white noise is off, so exactly zero
    assert np.all(np.abs(e[:, deg, :]).max(axis=(0, 2)) > 0)


def test_bias_standard_deviation_equals_bias_sigma():
    sigma = 0.8
    e, _ = gnss_error(2, 20000, _cfg(gnss_deg_fraction=1.0, gnss_bias_sigma=sigma),
                      np.random.default_rng(2))
    x = e[0].ravel()          # 60000 independent N(0, sigma^2) draws at the first step
    # standard error of a sample std is about sigma / sqrt(2 N) = 0.0023 m; allow 4 of them
    assert x.std() == pytest.approx(sigma, abs=4 * sigma / math.sqrt(2 * x.size))
    assert abs(x.mean()) < 4 * sigma / math.sqrt(x.size)


def test_bias_is_stationary_and_has_gauss_markov_autocorrelation():
    sigma, tau, dt = 1.0, 30.0, 0.2
    e, _ = gnss_error(301, 4000, _cfg(dt=dt, gnss_deg_fraction=1.0, gnss_bias_sigma=sigma,
                                      gnss_bias_tau=tau), np.random.default_rng(3))
    for lag in (50, 150, 300):                   # 10 s, 30 s, 60 s
        a, b = e[0].ravel(), e[lag].ravel()
        rho = np.corrcoef(a, b)[0, 1]
        # theory exp(-lag dt / tau); standard error of a correlation from 12000 pairs < 0.01
        assert rho == pytest.approx(math.exp(-lag * dt / tau), abs=0.03)
        assert b.std() == pytest.approx(sigma, abs=0.03)   # stationary: same spread later


def test_degraded_share_matches_fraction():
    n, frac = 20000, 0.25
    _, info = gnss_error(2, n, _cfg(gnss_deg_fraction=frac, gnss_bias_sigma=0.5),
                         np.random.default_rng(4))
    # binomial standard error sqrt(f (1 - f) / n) = 0.0031; allow 4 of them
    assert info["degraded"].mean() == pytest.approx(frac, abs=4 * math.sqrt(frac * (1 - frac) / n))


def test_white_noise_level_and_independence_between_steps():
    w = 0.015
    e, _ = gnss_error(200, 300, _cfg(gnss_white_sigma=w), np.random.default_rng(5))
    assert e.std() == pytest.approx(w, rel=0.02)              # 180000 draws
    assert abs(np.corrcoef(e[:-1].ravel(), e[1:].ravel())[0, 1]) < 0.02


def test_no_bias_when_sigma_is_zero_even_if_degraded():
    e, info = gnss_error(10, 50, _cfg(gnss_deg_fraction=1.0, gnss_bias_sigma=0.0),
                         np.random.default_rng(6))
    assert info["degraded"].all() and np.all(e == 0.0)


def test_dropout_windows_have_the_stated_length():
    dt, loss_s, K = 0.2, 10.0, 600
    e, info = gnss_error(K, 30, _cfg(dt=dt, gnss_loss_fraction=1.0, gnss_loss_s=loss_s,
                                     gnss_dead_reckoning_rate=0.3), np.random.default_rng(7))
    assert len(info["loss_windows"]) == 30
    for i, (k0, k1) in info["loss_windows"].items():
        assert k1 - k0 == round(loss_s / dt)
        assert 0.15 * K * dt <= k0 * dt + dt                  # starts after 15% of the show
        assert np.all(e[:k0, i, :] == 0.0) and np.any(e[k0:k1, i, :] != 0.0)


def test_common_fraction_zero_is_the_old_stream_and_one_shares_the_bias():
    import numpy as np

    from sirb.disturbance import StressConfig, gnss_error

    base = StressConfig(gnss_deg_fraction=1.0, gnss_bias_sigma=1.0, gnss_white_sigma=0.0, dt=0.2)
    a, _ = gnss_error(50, 6, base, np.random.default_rng(5))
    b, _ = gnss_error(50, 6, base.with_(gnss_common_fraction=0.0), np.random.default_rng(5))
    assert np.array_equal(a, b)
    c, _ = gnss_error(50, 6, base.with_(gnss_common_fraction=1.0), np.random.default_rng(5))
    assert np.allclose(c, c[:, :1, :])
