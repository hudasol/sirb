"""Reciprocal collision avoidance in three dimensions (ORCA), after van den Berg, Guy, Lin and
Manocha, "Reciprocal n-body collision avoidance" (2011).

Each agent sees its nearest neighbours and builds one half space of allowed velocities per
neighbour. The new velocity is the allowed velocity closest to the preferred one, found by Dykstra
projection onto the half spaces and the speed ball. When no allowed velocity exists, a small
optimisation minimises the largest violation (the role of the fourth linear program in the
reference code). This module is an independent reimplementation from the paper; it was compared
numerically with the Apache-2.0 reference implementation RVO2-3D (``scripts/reference_orca.py``).

``orca_velocities`` is one decision step. ``plan_orca`` runs a scenario to completion and returns
a ``Fleet`` of straight segments (velocity held for one time step), so the exact verifier can judge
the result. Velocity changes are instantaneous, as in the original method; the planner does not
respect acceleration limits and the benchmark reports that separately.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np
from scipy.optimize import minimize
from scipy.spatial import cKDTree

from ..trajectory import Fleet, Segment, Trajectory


def _unit_perp(a: np.ndarray) -> np.ndarray:
    """Some unit vector orthogonal to each row of ``a`` (unit rows)."""
    helper = np.where(np.abs(a[..., 0:1]) < 0.9, np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0]))
    e = np.cross(a, helper)
    return e / np.maximum(np.linalg.norm(e, axis=-1, keepdims=True), 1e-12)


def constraint_planes(p, vrel, r, tau, dt):
    """Half space data for relative position ``p`` and relative velocity ``vrel`` (same shape,
    last axis 3), combined radius ``r``. Returns ``(u, n)``: the smallest change of the relative
    velocity that reaches the boundary of the velocity obstacle, and its outward unit normal."""
    d2 = np.einsum("...i,...i", p, p)
    d = np.sqrt(d2)
    out_u = np.zeros_like(p)
    out_n = np.zeros_like(p)

    # collision already: aim to separate within one time step
    coll = d <= r
    # cut off sphere (truncation at the time horizon)
    w = vrel - p / tau
    wl = np.linalg.norm(w, axis=-1)
    dot1 = np.einsum("...i,...i", w, p)
    cap = (~coll) & (dot1 < 0) & (dot1 * dot1 > r * r * wl * wl)
    # cone
    a = p / np.maximum(d, 1e-12)[..., None]
    v_ax = np.einsum("...i,...i", vrel, a)
    vperp = vrel - v_ax[..., None] * a
    h = np.linalg.norm(vperp, axis=-1)
    e = np.where((h > 1e-12)[..., None], vperp / np.maximum(h, 1e-12)[..., None], _unit_perp(a))
    sin_a = np.minimum(r / np.maximum(d, 1e-12), 1.0)
    cos_a = np.sqrt(np.maximum(1.0 - sin_a * sin_a, 0.0))
    s = v_ax * cos_a + h * sin_a
    q = s[..., None] * (cos_a[..., None] * a + sin_a[..., None] * e)
    n_cone = -sin_a[..., None] * a + cos_a[..., None] * e
    u_cone = q - vrel

    # cap
    wd = w / np.maximum(wl, 1e-12)[..., None]
    u_cap = ((r / tau) - wl)[..., None] * wd
    # collision
    wc = vrel - p / dt
    wcl = np.linalg.norm(wc, axis=-1)
    wcd = wc / np.maximum(wcl, 1e-12)[..., None]
    u_coll = ((r / dt) - wcl)[..., None] * wcd

    out_u = np.where(coll[..., None], u_coll, np.where(cap[..., None], u_cap, u_cone))
    out_n = np.where(coll[..., None], wcd, np.where(cap[..., None], wd, n_cone))
    return out_u, out_n


def _dykstra(vpref, pts, nrm, mask, vmax, iters=400):
    """Closest point to ``vpref`` in the intersection of the half spaces
    (v - pts_k) . nrm_k >= 0 (where mask) and the ball |v| <= vmax. Shapes: vpref (n,3),
    pts and nrm (n,K,3), mask (n,K)."""
    n, K, _ = pts.shape
    x = vpref.copy()
    inc = np.zeros((n, K + 1, 3))
    for _ in range(iters):
        for k in range(K):
            y = x + inc[:, k]
            gap = np.einsum("ni,ni->n", y - pts[:, k], nrm[:, k])
            viol = (gap < 0) & mask[:, k]
            xn = np.where(viol[:, None], y - gap[:, None] * nrm[:, k], y)
            inc[:, k] = y - xn
            x = xn
        y = x + inc[:, K]
        sp = np.linalg.norm(y, axis=1)
        xn = np.where((sp > vmax)[:, None], y * (vmax / np.maximum(sp, 1e-12))[:, None], y)
        inc[:, K] = y - xn
        x = xn
    return x


def _project_exact(vpref, pts, nrm, mask, vmax, x0):
    """Closest feasible velocity to ``vpref`` (SLSQP polish). Returns None when infeasible."""
    P = pts[mask]
    N = nrm[mask]
    b = np.einsum("ki,ki->k", P, N)
    cons = [{"type": "ineq", "fun": lambda z: N @ z - b, "jac": lambda z: N},
            {"type": "ineq", "fun": lambda z: vmax * vmax - z @ z, "jac": lambda z: -2.0 * z}]
    res = minimize(lambda z: 0.5 * np.sum((z - vpref) ** 2), x0, jac=lambda z: z - vpref,
                   constraints=cons, method="SLSQP", options={"maxiter": 200, "ftol": 1e-12})
    v = res.x
    if np.any(N @ v - b < -1e-6) or v @ v > vmax * vmax * (1 + 1e-9):
        return None
    return v


def _least_violation(vpref, pts, nrm, mask, vmax, x0):
    """Velocity within the speed ball that minimises the largest violation of the half spaces."""
    P = pts[mask]
    N = nrm[mask]
    b = np.einsum("ki,ki->k", P, N)

    def obj(z):
        return z[3]

    cons = [{"type": "ineq", "fun": lambda z: N @ z[:3] - b + z[3]},      # n.v - b + d >= 0
            {"type": "ineq", "fun": lambda z: vmax * vmax - z[:3] @ z[:3]},
            {"type": "ineq", "fun": lambda z: z[3] + 1e3}]
    d0 = float(np.max(b - N @ x0, initial=0.0))
    res = minimize(obj, np.append(x0, max(d0, 0.0)), constraints=cons, method="SLSQP",
                   options={"maxiter": 100, "ftol": 1e-9})
    v = res.x[:3]
    sp = np.linalg.norm(v)
    if sp > vmax:
        v = v * (vmax / sp)
    return v


def orca_velocities(P, V, Vpref, *, radius_sum: float, tau: float, dt: float, vmax: float,
                    max_neighbors: int = 10, neighbor_dist: float | None = None,
                    dim: int = 3) -> np.ndarray:
    """New velocities for all agents. ``radius_sum`` is the separation to keep (centre to
    centre); each agent takes half of the avoidance (reciprocity). ``dim=2`` keeps the motion in
    the horizontal plane."""
    n = len(P)
    if neighbor_dist is None:
        neighbor_dist = 2.0 * vmax * tau + radius_sum
    K = min(max_neighbors, n - 1)
    if K <= 0:
        return _clip(Vpref, vmax)
    tree = cKDTree(P)
    dist, idx = tree.query(P, k=K + 1, distance_upper_bound=neighbor_dist)
    dist, idx = dist[:, 1:], idx[:, 1:]
    mask = np.isfinite(dist)
    idx_safe = np.where(mask, idx, 0)
    p = P[idx_safe] - P[:, None, :]
    vrel = V[:, None, :] - V[idx_safe]
    if dim == 2:
        p = p.copy(); vrel = vrel.copy()
        p[..., 2] = 0.0
        vrel[..., 2] = 0.0
    u, nrm = constraint_planes(p, vrel, radius_sum, tau, dt)
    pts = V[:, None, :] + 0.5 * u
    if dim == 2:
        nrm = nrm.copy(); pts = pts.copy()
        nrm[..., 2] = 0.0
        nn = np.linalg.norm(nrm, axis=-1, keepdims=True)
        nrm = nrm / np.maximum(nn, 1e-12)
    vp = np.array(Vpref, dtype=float)
    if dim == 2:
        vp = vp.copy(); vp[:, 2] = 0.0
    x = _dykstra(vp, pts, nrm, mask, vmax)
    gap = np.einsum("nki,nki->nk", x[:, None, :] - pts, nrm)
    bad = np.where(mask & (gap < -1e-4), True, False).any(axis=1)
    for i in np.nonzero(bad)[0]:
        y = _project_exact(vp[i], pts[i], nrm[i], mask[i], vmax, x[i])
        if y is not None:
            x[i] = y
            continue
        x[i] = _least_violation(vp[i], pts[i], nrm[i], mask[i], vmax, x[i])
        if dim == 2:
            x[i, 2] = 0.0
    return x


def _clip(v, vmax):
    sp = np.linalg.norm(v, axis=-1, keepdims=True)
    return np.where(sp > vmax, v * (vmax / np.maximum(sp, 1e-12)), v)


@dataclass
class OrcaResult:
    fleet: Fleet
    makespan_s: float
    arrived: bool
    steps: int
    runtime_s: float


def plan_orca(start: np.ndarray, goal: np.ndarray, *, radius_sum: float, vmax: float,
              dt: float = 0.1, tau: float = 3.0, max_time_s: float = 300.0,
              arrive_tol: float = 0.05, max_neighbors: int = 10, dim: int = 3) -> OrcaResult:
    """Run ORCA from ``start`` to ``goal`` (n x 3 each, agent i goes to goal i). Stops when every
    agent is within ``arrive_tol`` of its goal and slower than ``arrive_tol / dt``, or at
    ``max_time_s``. Returns the motion as straight segments, one per time step."""
    t_wall = time.perf_counter()
    P = start.astype(float).copy()
    V = np.zeros_like(P)
    n = len(P)
    pos_hist = [P.copy()]
    arrived = False
    steps = int(round(max_time_s / dt))
    for _ in range(steps):
        to_goal = goal - P
        dist = np.linalg.norm(to_goal, axis=1, keepdims=True)
        speed = np.minimum(vmax, dist / dt)
        vpref = np.where(dist > 1e-9, to_goal / np.maximum(dist, 1e-12) * speed, 0.0)
        V = orca_velocities(P, V, vpref, radius_sum=radius_sum, tau=tau, dt=dt, vmax=vmax,
                            max_neighbors=max_neighbors, dim=dim)
        P = P + V * dt
        pos_hist.append(P.copy())
        if (np.linalg.norm(goal - P, axis=1) < arrive_tol).all() and \
                (np.linalg.norm(V, axis=1) < arrive_tol / dt).all():
            arrived = True
            break
    H = np.stack(pos_hist, axis=0)                      # (T+1, n, 3)
    T = len(H) - 1
    trajs = []
    for i in range(n):
        segs = [Segment(j * dt, (j + 1) * dt,
                        np.stack([H[j, i], (H[j + 1, i] - H[j, i]) / dt]))
                for j in range(T)]
        trajs.append(Trajectory(segs))
    makespan = T * dt
    return OrcaResult(Fleet(trajs, {"planner": "orca", "dt": dt, "tau": tau}), makespan, arrived,
                      T, time.perf_counter() - t_wall)
