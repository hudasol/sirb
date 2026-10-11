"""Failure-aware behaviours (M6), expressed as modified *reference* trajectories.

A behaviour replaces one drone's reference from the event step onward; the tracking loop in
``sirb.stress`` then flies it, with the same wind and GNSS disturbances as every other drone.

Behaviours
- continue: do nothing (pre-programmed path keeps playing)
- hold: hover at the event position for ``hold_s``, then descend vertically
- descend: descend vertically in place at ``descent_rate``
- corridor: leave horizontally along the clearest of 8 directions for ``corridor_len`` metres,
  then descend vertically. Clearance is evaluated against the other drones' nominal paths.
- rejoin (GNSS loss only): hold during the dropout window, then blend back to the nominal path
- ballistic (motor failure only): uncontrolled fall, handled by ``sirb.stress``
"""

from __future__ import annotations

import math

import numpy as np

from .disturbance import StressConfig


def _descend(ref_i: np.ndarray, k_start: int, start_pos: np.ndarray, cfg: StressConfig) -> None:
    K = len(ref_i)
    steps = np.arange(K - k_start)
    z = np.maximum(start_pos[2] - cfg.descent_rate * steps * cfg.dt, 0.0)
    ref_i[k_start:, 0] = start_pos[0]
    ref_i[k_start:, 1] = start_pos[1]
    ref_i[k_start:, 2] = z


def hold_reference(ref_i, k_e, cfg):
    K = len(ref_i)
    p = ref_i[k_e].copy()
    k_hold_end = min(k_e + int(cfg.hold_s / cfg.dt), K)
    ref_i[k_e:k_hold_end] = p
    if k_hold_end < K:
        _descend(ref_i, k_hold_end, p, cfg)


def descend_reference(ref_i, k_e, cfg):
    _descend(ref_i, k_e, ref_i[k_e].copy(), cfg)


def corridor_direction(ref: np.ndarray, i: int, k_e: int, cfg: StressConfig) -> np.ndarray:
    """Pick the horizontal unit vector whose exit path stays farthest from the other drones."""
    K, n, _ = ref.shape
    steps = int(cfg.corridor_len / cfg.corridor_speed / cfg.dt)
    ks = np.arange(k_e, min(k_e + steps, K))
    p0 = ref[k_e, i]
    others = np.delete(np.arange(n), i)
    best_u, best_clear = None, -1.0
    angles = np.arange(8) * math.pi / 4 + math.pi / 2  # try +y (away from the crowd) first
    for a in angles:
        u = np.array([math.cos(a), math.sin(a), 0.0])
        path = p0[None, :] + u[None, :] * cfg.corridor_speed * ((ks - k_e) * cfg.dt)[:, None]
        d = np.linalg.norm(ref[ks][:, others, :] - path[:, None, :], axis=2).min()
        if d > best_clear + 1e-9:
            best_clear, best_u = d, u
    return best_u


def corridor_reference(ref_nominal, ref_i, i, k_e, cfg):
    K = len(ref_i)
    u = corridor_direction(ref_nominal, i, k_e, cfg)
    p0 = ref_nominal[k_e, i].copy()
    steps = int(cfg.corridor_len / cfg.corridor_speed / cfg.dt)
    k_end = min(k_e + steps, K - 1)
    for k in range(k_e, k_end + 1):
        ref_i[k] = p0 + u * cfg.corridor_speed * (k - k_e) * cfg.dt
    if k_end + 1 < K:
        _descend(ref_i, k_end + 1, ref_i[k_end].copy(), cfg)


def rejoin_reference(ref_nominal_i, ref_i, k0, k1, cfg):
    """Hold during [k0, k1), then min-jerk blend to the nominal path over ``rejoin_s``."""
    K = len(ref_i)
    p = ref_nominal_i[k0].copy()
    ref_i[k0:k1] = p
    m = max(int(cfg.rejoin_s / cfg.dt), 1)
    if cfg.rejoin_vmax is not None:
        # Opt-in rate limit: lengthen the blend until the (moving) distance to cover implies a
        # closing speed 1.875 * d / T (minimum-jerk peak) no larger than rejoin_vmax.
        for _ in range(8):
            d = float(np.linalg.norm(ref_nominal_i[k1:min(k1 + m + 1, K)] - p, axis=1).max()) \
                if k1 < K else 0.0
            m_new = max(m, int(math.ceil(1.875 * d / cfg.rejoin_vmax / cfg.dt)))
            if m_new == m:
                break
            m = m_new
    for j in range(m):
        k = k1 + j
        if k >= K:
            break
        u = (j + 1) / m
        s = 10 * u**3 - 15 * u**4 + 6 * u**5
        ref_i[k] = (1 - s) * p + s * ref_nominal_i[k]


def apply_events(ref_nominal: np.ndarray, events: list[dict], gnss_info: dict,
                 cfg: StressConfig) -> tuple[np.ndarray, list[tuple[int, int]]]:
    """Return (modified reference, ballistic list of (drone, event step))."""
    R = ref_nominal.copy()
    ballistic: list[tuple[int, int]] = []
    for ev in sorted(events, key=lambda e: e["k"]):
        i, k_e, kind = ev["drone"], ev["k"], ev["kind"]
        beh = cfg.behavior.get(kind, "continue")
        if kind == "motor_failure" or beh == "ballistic":
            ballistic.append((i, k_e))
        elif beh == "hold":
            hold_reference(R[:, i, :], k_e, cfg)
        elif beh == "descend":
            descend_reference(R[:, i, :], k_e, cfg)
        elif beh == "corridor":
            corridor_reference(ref_nominal, R[:, i, :], i, k_e, cfg)
        elif beh == "continue":
            pass
        else:
            raise ValueError(f"unknown behaviour {beh!r} for {kind}")
    beh = cfg.behavior.get("gnss_loss", "continue")
    for i, (k0, k1) in gnss_info.get("loss_windows", {}).items():
        if beh == "hold" or beh == "rejoin":
            rejoin_reference(ref_nominal[:, i, :], R[:, i, :], k0, k1, cfg)
        elif beh == "descend":
            descend_reference(R[:, i, :], k0, cfg)
        elif beh == "corridor":
            corridor_reference(ref_nominal, R[:, i, :], i, k0, cfg)
    return R, ballistic
