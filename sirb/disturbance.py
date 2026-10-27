"""Disturbance models for Monte-Carlo stress tests (M5).

Every parameter is either cited (see docs/RESEARCH.md) or labeled ASSUMPTION here and in
docs/METHODS.md. Nothing here uses real flight data.

Wind. Mean wind at 20 ft W20 with a 1/7 power-law profile (ASSUMPTION). Turbulence intensity
from the Dryden low-altitude relations of MIL-HDBK-1797 as implemented in MathWorks
documentation: sigma_w = 0.1 W20, sigma_u = sigma_v = sigma_w / (0.177 + 0.000823 h_ft)^0.4.
Those relations were built for piloted aircraft; applying them to a small multirotor is an
ASSUMPTION. Spatial structure by frozen turbulence: every drone sees the same first-order
Gauss-Markov process delayed by (position along the wind) / V_adv, plus an independent local
component of fraction ``local_fraction`` (ASSUMPTION), so *differential* wind between
neighbours is physical rather than imposed.

GNSS. Per drone position-estimate error e(t): white noise (RTK-fixed level, ASSUMPTION informed
by the ZED-F9P datasheet figure of 0.01 m + 1 ppm) plus, for degraded drones, a first-order
Gauss-Markov bias with sigma ``bias_sigma`` and time constant ``bias_tau`` (ASSUMPTIONS informed
by forum-reported float errors), plus optional dropout windows where the error random-walks
(dead reckoning, ASSUMPTION). Differential (per-drone) errors are what matter for separation.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

G = 9.80665


@dataclass
class StressConfig:
    dt: float = 0.1
    # tracking loop (closed-loop second order), ASSUMPTION
    omega_n: float = 2.0
    zeta: float = 0.9
    # wind
    w20: float = 0.0               # mean wind at 20 ft, m/s
    wind_dir_deg: float = 0.0
    wind_tau: float = 5.0          # turbulence correlation time, s (ASSUMPTION)
    wind_local_fraction: float = 0.3  # independent local share of turbulence (ASSUMPTION)
    drag_gain: float = 0.3         # disturbance accel per m/s of wind, 1/s (ASSUMPTION)
    # gnss
    gnss_white_sigma: float = 0.015   # m, per axis (ASSUMPTION, RTK fixed)
    gnss_deg_fraction: float = 0.0    # share of drones in degraded mode
    gnss_common_fraction: float = 0.0  # share of the bias variance common to all degraded drones
    gnss_bias_sigma: float = 0.0      # m, per axis, degraded drones (ASSUMPTION)
    gnss_bias_tau: float = 30.0       # s (ASSUMPTION)
    gnss_loss_fraction: float = 0.0   # share of drones with a dropout window
    gnss_loss_s: float = 10.0
    gnss_dead_reckoning_rate: float = 0.3  # m / sqrt(s), random walk in dropout (ASSUMPTION)
    # events (comms loss, low battery, motor failure)
    comms_loss_fraction: float = 0.0
    battery_low_fraction: float = 0.0
    motor_failures: int = 0
    behavior: dict = field(default_factory=lambda: {
        "comms_loss": "continue", "battery_low": "descend", "gnss_loss": "continue",
        "motor_failure": "ballistic"})
    descent_rate: float = 2.0      # m/s (ASSUMPTION, below vmax_down)
    hold_s: float = 10.0
    corridor_len: float = 40.0
    corridor_speed: float = 3.0
    rejoin_s: float = 6.0
    rejoin_vmax: float | None = None  # opt-in cap on the rejoin blend closing speed, m/s
    drone_radius: float = 0.24     # m, X7 fact sheet (half of 48 cm)
    air_drag_per_s: float = 0.5    # horizontal velocity decay in ballistic fall (ASSUMPTION)

    def with_(self, **kw) -> StressConfig:
        d = dict(self.__dict__)
        d.update(kw)
        return StressConfig(**d)


def dryden_sigmas(w20: float, h_m: float) -> tuple[float, float]:
    """(sigma_horizontal, sigma_vertical) turbulence intensity in m/s."""
    h_ft = max(h_m / 0.3048, 1.0)
    sigma_w = 0.1 * w20
    sigma_u = sigma_w / (0.177 + 0.000823 * h_ft) ** 0.4
    return sigma_u, sigma_w


def mean_wind_vector(w20: float, wind_dir_deg: float, z: np.ndarray) -> np.ndarray:
    """(n, 3) mean wind at altitudes z (m), 1/7 power law referenced to 6.1 m (20 ft)."""
    speed = w20 * (np.maximum(z, 1.0) / 6.1) ** (1.0 / 7.0)
    th = math.radians(wind_dir_deg)
    return np.stack([speed * math.cos(th), speed * math.sin(th), np.zeros_like(speed)], axis=1)


def _ou(K: int, n: int, dt: float, tau: float, sigma: float, rng: np.random.Generator,
        axes: int = 3) -> np.ndarray:
    """Stationary first-order Gauss-Markov process, shape (K, n, axes)."""
    phi = math.exp(-dt / tau)
    out = np.empty((K, n, axes))
    out[0] = sigma * rng.standard_normal((n, axes))
    drive = sigma * math.sqrt(1 - phi * phi) * rng.standard_normal((K - 1, n, axes))
    for k in range(1, K):
        out[k] = phi * out[k - 1] + drive[k - 1]
    return out


def wind_velocity(ref: np.ndarray, cfg: StressConfig, rng: np.random.Generator) -> np.ndarray:
    """(K, n, 3) wind velocity felt by each drone along its reference."""
    K, n, _ = ref.shape
    mean_z = ref[:, :, 2]
    th = math.radians(cfg.wind_dir_deg)
    w_dir = np.array([math.cos(th), math.sin(th), 0.0])
    if cfg.w20 <= 0:
        return np.zeros((K, n, 3))
    v_adv = max(cfg.w20 * (60.0 / 6.1) ** (1.0 / 7.0), 2.0)
    s = ref[0] @ w_dir / v_adv                       # delay of each drone along the wind
    shift = np.round((s - s.min()) / cfg.dt).astype(int)
    pad = int(shift.max()) + 1
    h_mid = float(np.clip(np.median(mean_z), 5.0, 200.0))
    sig_h, sig_v = dryden_sigmas(cfg.w20, h_mid)
    sig = np.array([sig_h, sig_h, sig_v])
    f = cfg.wind_local_fraction
    common = _ou(K + pad, 1, cfg.dt, cfg.wind_tau, 1.0, rng)[:, 0, :] * sig * math.sqrt(1 - f * f)
    local = _ou(K, n, cfg.dt, cfg.wind_tau, 1.0, rng) * sig * f
    turb = np.empty((K, n, 3))
    for i in range(n):
        turb[:, i, :] = common[pad - shift[i]: pad - shift[i] + K]
    turb += local
    mean = np.empty((K, n, 3))
    th_vec = np.array([math.cos(th), math.sin(th), 0.0])
    mean[:] = (np.maximum(mean_z, 1.0)[:, :, None] / 6.1) ** (1.0 / 7.0) * cfg.w20 * th_vec
    return mean + turb


def gnss_error(K: int, n: int, cfg: StressConfig, rng: np.random.Generator,
               ) -> tuple[np.ndarray, dict]:
    """(K, n, 3) position-estimate error and bookkeeping (degraded mask, loss windows)."""
    e = cfg.gnss_white_sigma * rng.standard_normal((K, n, 3))
    degraded = rng.random(n) < cfg.gnss_deg_fraction
    if cfg.gnss_bias_sigma > 0 and degraded.any():
        b = _ou(K, int(degraded.sum()), cfg.dt, cfg.gnss_bias_tau, cfg.gnss_bias_sigma, rng)
        c = cfg.gnss_common_fraction
        if c > 0:   # drawn after the individual part so c = 0 keeps every earlier stream
            shared = _ou(K, 1, cfg.dt, cfg.gnss_bias_tau, cfg.gnss_bias_sigma, rng)
            b = math.sqrt(1 - c) * b + math.sqrt(c) * shared
        e[:, degraded, :] += b
    loss_mask = rng.random(n) < cfg.gnss_loss_fraction
    windows: dict[int, tuple[int, int]] = {}
    if loss_mask.any():
        T = K * cfg.dt
        steps = max(int(cfg.gnss_loss_s / cfg.dt), 1)
        for i in np.where(loss_mask)[0]:
            t0 = rng.uniform(0.15 * T, max(0.85 * T - cfg.gnss_loss_s, 0.16 * T))
            k0 = int(t0 / cfg.dt)
            k1 = min(k0 + steps, K - 1)
            walk = np.cumsum(cfg.gnss_dead_reckoning_rate * math.sqrt(cfg.dt)
                             * rng.standard_normal((k1 - k0, 3)), axis=0)
            e[k0:k1, i, :] += walk
            e[k1:, i, :] += walk[-1] * np.exp(-np.arange(K - k1) * cfg.dt / 5.0)[:, None]
            windows[int(i)] = (k0, k1)
    return e, {"degraded": degraded, "loss_windows": windows}


def sample_events(K: int, n: int, cfg: StressConfig, rng: np.random.Generator) -> list[dict]:
    """Discrete per-drone events at uniform random times in [0.15 T, 0.85 T]."""
    T = K * cfg.dt
    events: list[dict] = []
    taken: set[int] = set()

    def pick(count: int) -> list[int]:
        pool = [i for i in range(n) if i not in taken]
        if not pool or count <= 0:
            return []
        chosen = rng.choice(pool, size=min(count, len(pool)), replace=False)
        taken.update(int(c) for c in chosen)
        return [int(c) for c in chosen]

    for kind, count in (
        ("motor_failure", cfg.motor_failures),
        ("comms_loss", int(rng.binomial(n, cfg.comms_loss_fraction)) if cfg.comms_loss_fraction else 0),
        ("battery_low", int(rng.binomial(n, cfg.battery_low_fraction)) if cfg.battery_low_fraction else 0),
    ):
        for i in pick(count):
            t = rng.uniform(0.15 * T, 0.85 * T)
            events.append({"drone": i, "kind": kind, "k": int(t / cfg.dt)})
    return events
