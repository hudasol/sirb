"""Monte-Carlo stress testing (M5): realized paths under disturbance, exact separation on them.

Trial = nominal reference (from a planned Fleet) + disturbances -> realized paths -> minimum
pairwise separation over the whole show. Realized paths are piecewise linear between samples
(step cfg.dt); the minimum distance over each linear piece is computed in closed form, so the
only approximation is the linear interpolation of the sampled realized path (documented in
docs/METHODS.md; cross-checked against ``sirb.verify`` on noise-free runs in tests).

Tracking model. Each drone's tracking error eps = x - r obeys
    eps'' = -kd eps' - kp eps - kp e(t) + a_wind(t),   kp = wn^2, kd = 2 zeta wn
where e(t) is the GNSS position-estimate error (the controller acts on the *estimated* position)
and a_wind = drag_gain * wind velocity. Discretised exactly with a zero-order hold.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

import numpy as np
from scipy.linalg import expm
from scipy.spatial import cKDTree

from .contingency import apply_events
from .disturbance import G, StressConfig, gnss_error, sample_events, wind_velocity
from .trajectory import Fleet


@dataclass
class Trial:
    min_separation: float
    pair: tuple[int, int] | None
    violated: bool           # min separation below the declared threshold
    contact: bool            # below 2 x drone radius
    violating_pairs: int
    max_tracking_error: float
    events: list[dict]
    runtime_s: float
    event_drones: tuple[int, ...] = ()
    pair_involves_event_drone: bool = False


def _zoh(cfg: StressConfig) -> tuple[np.ndarray, np.ndarray]:
    kp, kd = cfg.omega_n**2, 2 * cfg.zeta * cfg.omega_n
    A = np.array([[0.0, 1.0], [-kp, -kd]])
    B = np.array([0.0, 1.0])
    Ad = expm(A * cfg.dt)
    Bd = np.linalg.solve(A, (Ad - np.eye(2)) @ B)
    return Ad, Bd


def track(ref: np.ndarray, u: np.ndarray, cfg: StressConfig) -> np.ndarray:
    """Tracking error eps (K, n, 3) given input u (K, n, 3) = -kp e + disturbance accel."""
    Ad, Bd = _zoh(cfg)
    K = ref.shape[0]
    eps = np.zeros_like(ref)
    vel = np.zeros_like(ref)
    for k in range(K - 1):
        eps[k + 1] = Ad[0, 0] * eps[k] + Ad[0, 1] * vel[k] + Bd[0] * u[k]
        vel[k + 1] = Ad[1, 0] * eps[k] + Ad[1, 1] * vel[k] + Bd[1] * u[k]
    return eps


def ballistic_track(x_before: np.ndarray, x_at: np.ndarray, K: int, k_e: int,
                    cfg: StressConfig) -> np.ndarray:
    """Positions (K - k_e, 3) of an uncontrolled fall starting at step k_e."""
    v0 = (x_at - x_before) / cfg.dt
    t = np.arange(K - k_e) * cfg.dt
    c = cfg.air_drag_per_s
    out = np.empty((K - k_e, 3))
    decay = (1 - np.exp(-c * t)) / c
    out[:, 0] = x_at[0] + v0[0] * decay
    out[:, 1] = x_at[1] + v0[1] * decay
    out[:, 2] = np.maximum(x_at[2] + v0[2] * t - 0.5 * G * t * t, 0.0)
    return out


def dev_bound(cfg: StressConfig) -> float:
    """Conservative bound on |eps_i - eps_j| used only to choose candidate pairs."""
    gn = 6 * cfg.gnss_white_sigma + 6 * cfg.gnss_bias_sigma
    gn += 6 * cfg.gnss_dead_reckoning_rate * math.sqrt(max(cfg.gnss_loss_s, 1.0)) \
        if cfg.gnss_loss_fraction > 0 else 0.0
    kp = cfg.omega_n**2
    wind = cfg.drag_gain * (1.5 * cfg.w20 + 0.0) / kp * 2
    return 2 * (gn + wind) + 1.0


def candidate_pairs(R: np.ndarray, radius: float, stride: int = 4,
                    vmax_rel: float = 20.0, dt: float = 0.1
                    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Pairs ever within ``radius`` (+ motion pad) at sampled steps of R, with step windows.

    Returns (pairs (P, 2), k_lo (P,), k_hi (P,)); the pair can only violate inside
    [k_lo, k_hi] (sampled near-steps widened by one stride).
    """
    pad = vmax_rel * stride * dt / 2
    lo: dict[tuple[int, int], int] = {}
    hi: dict[tuple[int, int], int] = {}
    for k in range(0, R.shape[0], stride):
        for pr in cKDTree(R[k]).query_pairs(radius + pad):
            if pr not in lo:
                lo[pr] = k
            hi[pr] = k
    keys = sorted(lo)
    pairs = np.array(keys, dtype=int).reshape(-1, 2)
    k_lo = np.array([max(lo[q] - stride, 0) for q in keys], dtype=int)
    k_hi = np.array([min(hi[q] + stride, R.shape[0] - 1) for q in keys], dtype=int)
    return pairs, k_lo, k_hi


def exact_pair_min(X: np.ndarray, pairs: np.ndarray, k_lo: np.ndarray | None = None,
                   k_hi: np.ndarray | None = None, block: int = 40
                   ) -> tuple[np.ndarray, np.ndarray]:
    """Per-pair minimum distance of piecewise-linear paths X (K, n, 3), and the step of the min.

    If windows are given, pair p is only evaluated for steps in [k_lo[p], k_hi[p]).
    """
    P = len(pairs)
    K = X.shape[0]
    pmin = np.full(P, np.inf)
    karg = np.zeros(P, dtype=int)
    if P == 0:
        return pmin, karg
    if k_lo is None:
        k_lo = np.zeros(P, dtype=int)
        k_hi = np.full(P, K - 1, dtype=int)
    for b0 in range(0, K - 1, block):
        b1 = min(b0 + block, K - 1)
        sel = np.where((k_lo < b1) & (k_hi > b0))[0]
        if len(sel) == 0:
            continue
        i, j = pairs[sel, 0], pairs[sel, 1]
        cur = pmin[sel].copy()
        arg = karg[sel].copy()
        d0 = X[b0, i] - X[b0, j]
        for k in range(b0, b1):
            d1 = X[k + 1, i] - X[k + 1, j]
            dv = d1 - d0
            a = np.einsum("ij,ij->i", dv, dv)
            bb = np.einsum("ij,ij->i", d0, dv)
            u = np.where(a > 1e-18, np.clip(-bb / np.where(a > 1e-18, a, 1.0), 0.0, 1.0), 0.0)
            m = d0 + u[:, None] * dv
            dist = np.sqrt(np.einsum("ij,ij->i", m, m))
            better = dist < cur
            cur = np.where(better, dist, cur)
            arg = np.where(better, k, arg)
            d0 = d1
        pmin[sel], karg[sel] = cur, arg
    return pmin, karg


def sample_reference(fleet: Fleet, dt: float, t_end: float | None = None) -> np.ndarray:
    T = fleet.t_end if t_end is None else t_end
    times = np.arange(0.0, T + dt / 2, dt)
    return fleet.sample(times)


def realize(ref: np.ndarray, cfg: StressConfig, seed):
    """One disturbed realisation: returns (X realized, R modified reference, eps, events,
    ginfo, ballistic drone list)."""
    rng = np.random.default_rng(seed)
    K, n, _ = ref.shape
    wind = wind_velocity(ref, cfg, rng)
    e, ginfo = gnss_error(K, n, cfg, rng)
    events = sample_events(K, n, cfg, rng)
    R, ballistic = apply_events(ref, events, ginfo, cfg)
    kp = cfg.omega_n**2
    u = -kp * e + cfg.drag_gain * wind
    eps = track(R, u, cfg)
    X = R + eps
    bal_idx = []
    for i, k_e in ballistic:
        if k_e < 1:
            continue
        X[k_e:, i, :] = ballistic_track(X[k_e - 1, i], X[k_e, i], K, k_e, cfg)
        bal_idx.append(i)
    return X, R, eps, events, ginfo, bal_idx


def run_trial(ref: np.ndarray, cfg: StressConfig, d_min: float, seed) -> Trial:
    t_wall = time.perf_counter()
    K, n, _ = ref.shape
    X, R, eps, events, ginfo, bal_idx = realize(ref, cfg, seed)
    radius = d_min + dev_bound(cfg)
    pairs, k_lo, k_hi = candidate_pairs(R, radius, dt=cfg.dt)
    if bal_idx:
        extra = np.array([(min(f, j), max(f, j)) for f in bal_idx for j in range(n) if j != f],
                         dtype=int)
        full_lo = np.zeros(len(extra), dtype=int)
        full_hi = np.full(len(extra), K - 1, dtype=int)
        pairs = np.vstack([pairs, extra])
        k_lo = np.concatenate([k_lo, full_lo])
        k_hi = np.concatenate([k_hi, full_hi])
    pmin, karg = exact_pair_min(X, pairs, k_lo, k_hi)
    if len(pmin):
        m = int(np.argmin(pmin))
        dmin, pair = float(pmin[m]), (int(pairs[m, 0]), int(pairs[m, 1]))
    else:
        dmin, pair = math.inf, None
    ev_drones = tuple(sorted({e["drone"] for e in events} | set(ginfo["loss_windows"])))
    involves = pair is not None and (pair[0] in ev_drones or pair[1] in ev_drones)
    return Trial(dmin, pair, dmin < d_min, dmin < 2 * cfg.drone_radius,
                 int((pmin < d_min).sum()), float(np.abs(eps).max()), events,
                 time.perf_counter() - t_wall, ev_drones, involves)


def wilson(k: int, n: int, z: float = 1.96) -> tuple[float, float]:
    if n == 0:
        return 0.0, 1.0
    p = k / n
    den = 1 + z * z / n
    c = (p + z * z / (2 * n)) / den
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return max(0.0, c - h), min(1.0, c + h)


def required_value(rows: list[dict], target: float, key: str = "formation_spacing_m") -> dict:
    """Smallest value of ``key`` whose Monte-Carlo violation risk is demonstrably <= ``target``.

    Rule (conservative): value v qualifies when the upper 95% Wilson bound of ``p_violation`` is
    at most ``target`` for v *and for every larger value in the table*, so one lucky row does
    not set the answer. Returns ``{"value": v or None, "target": target, "rows_checked": ...}``.
    The answer holds only for the simulated scenario and model, and only resolves targets above
    the best attainable bound for the trial count (with zero violations in N trials the upper
    bound is 3.84 / (N + 3.84)); a smaller target returns ``None`` however good the plan is.
    """
    ordered = sorted(rows, key=lambda r: r[key])
    ok_from = None
    for r in reversed(ordered):
        if r["p_violation_ci95"][1] <= target:
            ok_from = r[key]
        else:
            break
    return {"value": ok_from, "target": target, "key": key,
            "rows_checked": [(r[key], r["p_violation_ci95"][1]) for r in ordered]}


_WORK: dict = {}


def _worker(args):
    cfg, d_min, seed, tag, t = args
    r = run_trial(_WORK["ref"], cfg, d_min, [seed, tag, t])
    return (r.min_separation, r.violated, r.contact, r.violating_pairs, r.max_tracking_error,
            r.pair_involves_event_drone, bool(r.event_drones), r.runtime_s)


def _fork_available() -> bool:
    """Worker pools rely on fork (shared read-only reference). Windows has no fork; there the
    runner falls back to serial execution, which gives identical numbers (each trial has its
    own seed)."""
    import multiprocessing as mp
    return "fork" in mp.get_all_start_methods()


def monte_carlo(ref: np.ndarray, cfg: StressConfig, d_min: float, trials: int, seed: int,
                tag: int = 0, workers: int = 1) -> dict:
    t0 = time.perf_counter()
    jobs = [(cfg, d_min, seed, tag, t) for t in range(trials)]
    _WORK["ref"] = ref
    if workers > 1 and _fork_available():
        import multiprocessing as mp
        with mp.get_context("fork").Pool(workers) as pool:
            res = pool.map(_worker, jobs, chunksize=max(1, trials // (workers * 4)))
    else:
        res = [_worker(j) for j in jobs]
    seps = np.array([r[0] for r in res])
    k = sum(r[1] for r in res)
    kc = sum(r[2] for r in res)
    k_ev = sum(1 for r in res if r[1] and r[5])
    k_bystander = sum(1 for r in res if r[1] and not r[5])
    lo, hi = wilson(k, trials)
    return {
        "trials": trials,
        "p_violation": k / trials, "p_violation_ci95": [lo, hi],
        "p_contact": kc / trials, "p_contact_ci95": list(wilson(kc, trials)),
        "p_violation_involving_event_drone": k_ev / trials,
        "p_violation_between_bystanders": k_bystander / trials,
        "min_separation_quantiles": {str(q): float(np.quantile(seps, q)) for q in (0.0, 0.05, 0.5)},
        "mean_violating_pairs": float(np.mean([r[3] for r in res])),
        "mean_max_tracking_error_m": float(np.mean([r[4] for r in res])),
        "wall_s": time.perf_counter() - t0,
        "mean_trial_cpu_s": float(np.mean([r[7] for r in res])),
    }


def sweep(ref: np.ndarray, base: StressConfig, param: str, values, d_min: float, trials: int,
          seed: int, workers: int = 1) -> list[dict]:
    rows = []
    for idx, v in enumerate(values):
        cfg = base.with_(**{param: v}) if not isinstance(v, dict) else base.with_(**v)
        row = monte_carlo(ref, cfg, d_min, trials, seed, tag=idx, workers=workers)
        row["value"] = v
        rows.append(row)
    return rows
