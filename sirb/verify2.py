"""Second, independent separation verifier: conservative branch and bound (no root finding).

Shares nothing with ``sirb.verify`` (it does not import it). Method:

* Each drone is tiled by pieces (hold the first position before the first segment, the
  segments, hold the last position after; gaps are bridged by holding).
* For two pieces overlapping on ``[a, b]`` the relative position ``D(s)``, ``s = t - a``, is a
  vector polynomial. On a cell ``[m - w, m + w]`` it is re-expanded around the midpoint by a
  Horner (synthetic division) Taylor shift, ``D(m + r) = sum q_k r^k``, and the squared norm
  ``P(r) = |D|^2 = sum p_k r^k`` is formed. Lower bound on the cell:
  ``p0 + min_{|r|<=w}(p1 r + p2 r^2) - sum_{k>=3} |p_k| w^k`` (the quadratic part is
  minimised exactly, the remainder is bounded by its absolute value), minus a rounding
  allowance. The value at the midpoint, ``|q0|``, is an upper bound on the minimum.
* Cells are bisected until their lower bound reaches the pruning level (the best upper bound
  found minus ``tol``, or the cap); what survives to the depth limit is reported as a gap.

Rigour: the bounds are interval-style bounds in exact arithmetic. Floating point is covered
by an explicit rounding allowance (``gamma * eps`` times the magnitude of the terms involved),
not by directed rounding, so the claim is "rigorous up to that documented allowance".
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

import numpy as np

EPS = float(np.finfo(float).eps)
_MAX_LEVEL = 48            # bisection depth limit; cells reaching it are reported as leaves
_MAX_ITEMS = 2_000_000     # work-list size limit; larger levels are treated as leaves
_CHUNK = 100_000


@dataclass
class Verify2Result:
    verdict: str                       # "PASS", "FAIL" or "UNDECIDED"
    d_min: float
    lower_bound: float                 # rigorous lower bound on the minimum separation (<= cap)
    upper_bound: float                 # smallest distance witnessed (inf if no pair)
    gap_bound: float                   # guaranteed bound on (true min - lower_bound) if decided
    tol: float
    cap: float
    max_rounding_allowance: float
    n_drones: int
    n_piece_pairs: int
    n_cells: int
    n_leaves: int                      # cells stopped by the depth or size limit
    runtime_s: float
    pair_lower: dict = field(default_factory=dict)    # {(i, j): lower bound} (per_pair mode)
    pair_upper: dict = field(default_factory=dict)    # {(i, j): witnessed distance}


# --------------------------------------------------------------------------- polynomials

def _taylor_shift(c: np.ndarray, d: np.ndarray) -> np.ndarray:
    """c (n, K, 3) -> coefficients of c(d + r) in r (Horner synthetic division)."""
    c = np.array(c, dtype=float)
    K = c.shape[1]
    dd = d[:, None]
    for i in range(K - 1):
        for j in range(K - 2, i - 1, -1):
            c[:, j] += dd * c[:, j + 1]
    return c


def _abs_eval(c: np.ndarray, x: np.ndarray) -> np.ndarray:
    """Sum_k |c_k| x^k per coordinate, c (n, K, 3), x (n,) >= 0 -> (n, 3)."""
    out = np.zeros((c.shape[0], 3))
    for k in range(c.shape[1] - 1, -1, -1):
        out = out * x[:, None] + np.abs(c[:, k, :])
    return out


def _hold_pos(c: np.ndarray, tau: float) -> np.ndarray:
    out = np.zeros(3)
    for k in range(c.shape[0] - 1, -1, -1):
        out = out * tau + c[k]
    return out


# --------------------------------------------------------------------------- pieces

def _pieces(fleet):
    drone, lo, hi, t0r, coefs = [], [], [], [], []
    for i, tr in enumerate(fleet.trajectories):
        segs = []
        for sg in tr.segments:
            t0, t1 = float(sg.t0), float(sg.t1)
            c = np.array(sg.coeffs, dtype=float)
            if not (t1 > t0) or c.ndim != 2 or c.shape[1] != 3 or c.shape[0] < 1:
                continue
            if not (math.isfinite(t0) and math.isfinite(t1) and np.all(np.isfinite(c))):
                continue
            segs.append((t0, t1, c))
        if not segs:
            continue
        first = segs[0]
        drone.append(i); lo.append(-math.inf); hi.append(first[0]); t0r.append(0.0)
        coefs.append(first[2][:1].copy())
        cursor, last = first[0], first[2][0].copy()
        for t0, t1, c in segs:
            if t1 <= cursor:
                continue
            if t0 > cursor:
                drone.append(i); lo.append(cursor); hi.append(t0); t0r.append(0.0)
                coefs.append(last[None, :].copy())
                cursor = t0
            drone.append(i); lo.append(max(t0, cursor)); hi.append(t1); t0r.append(t0)
            coefs.append(c)
            cursor = t1
            last = _hold_pos(c, t1 - t0)
        drone.append(i); lo.append(cursor); hi.append(math.inf); t0r.append(0.0)
        coefs.append(last[None, :].copy())
    n = len(drone)
    K = max((c.shape[0] for c in coefs), default=1)
    cf = np.zeros((n, K, 3))
    for m, c in enumerate(coefs):
        cf[m, : c.shape[0]] = c
    return (np.array(drone, dtype=np.int64), np.array(lo), np.array(hi), np.array(t0r), cf)


def _piece_boxes(lo, hi, t0r, cf, gamma):
    """Conservative axis-aligned boxes: centred form about the piece midpoint."""
    n = len(lo)
    fin = np.isfinite(lo) & np.isfinite(hi)
    mid = np.where(fin, 0.5 * (lo + hi), 0.0)
    w = np.where(fin, 0.5 * (hi - lo), 0.0)
    q = _taylor_shift(cf, np.where(fin, mid - t0r, 0.0))
    rad = np.zeros((n, 3))
    for k in range(1, cf.shape[1]):
        rad += np.abs(q[:, k, :]) * (w ** k)[:, None]
    mag = _abs_eval(cf, np.where(fin, np.abs(mid - t0r) + w, 0.0))
    rad += gamma * EPS * mag
    return q[:, 0, :] - rad, q[:, 0, :] + rad


def _candidate_piece_pairs(drone, lo, hi, blo, bhi, cap):
    order = np.argsort(lo, kind="stable")
    slo, shi = lo[order], hi[order]
    sbl, sbh, sdr = blo[order], bhi[order], drone[order]
    # Only pieces with a positive-length time overlap are paired (lo_j < hi_i). A single shared
    # instant would mix the end value of one drone's piece with the start value of another's,
    # which differs only when a trajectory has a position jump (a structural error).
    ends = np.searchsorted(slo, shi, side="left")
    cap2 = cap * cap if math.isfinite(cap) else math.inf
    A, B = [], []
    for i in range(len(order)):
        e = ends[i]
        if e <= i + 1:
            continue
        sl = slice(i + 1, e)
        sel = sdr[sl] != sdr[i]
        if cap2 < math.inf:
            gap = np.maximum(np.maximum(sbl[sl] - sbh[i], sbl[i] - sbh[sl]), 0.0)
            sel &= np.einsum("ij,ij->i", gap, gap) < cap2
        js = np.nonzero(sel)[0]
        if len(js):
            A.append(np.full(len(js), i, dtype=np.int64))
            B.append(js + i + 1)
    if not A:
        return np.zeros(0, np.int64), np.zeros(0, np.int64)
    return order[np.concatenate(A)], order[np.concatenate(B)]


# --------------------------------------------------------------------------- cell bounds

def _cell_bounds(Droot, pid, mid, w, E, gamma):
    """Lower bound (distance), midpoint value and E-inflated midpoint value for cells."""
    q = _taylor_shift(Droot[pid], mid)
    m, K, _ = q.shape
    G = np.einsum("mia,mja->mij", q, q)
    P = np.zeros((m, 2 * K - 1))
    for i in range(K):
        P[:, i:i + K] += G[:, i, :]
    wp = w[:, None] ** np.arange(2 * K - 1)[None, :]
    tail = np.sum(np.abs(P[:, 3:]) * wp[:, 3:], axis=1) if K > 2 else np.zeros(m)
    p0 = P[:, 0]
    p1 = P[:, 1] if K > 1 else np.zeros(m)
    p2 = P[:, 2] if K > 1 else np.zeros(m)
    cand = np.minimum(p1 * w + p2 * w * w, -p1 * w + p2 * w * w)
    with np.errstate(divide="ignore", invalid="ignore"):
        rs = np.where(p2 > 0, np.clip(-p1 / (2.0 * p2), -w, w), 0.0)
    vert = np.where(p2 > 0, p1 * rs + p2 * rs * rs, np.inf)
    qmin = np.minimum(cand, vert)
    nrm = np.linalg.norm(q, axis=2)
    S = np.sum(nrm[:, 1:] * wp[:, 1:K], axis=1) if K > 1 else np.zeros(m)
    sl2 = gamma * EPS * (nrm[:, 0] + S) ** 2
    lb2 = p0 + qmin - tail - sl2
    lbd = np.maximum(np.sqrt(np.maximum(lb2, 0.0)) - E, 0.0)
    ub = nrm[:, 0]
    return lbd, ub, ub + E


# --------------------------------------------------------------------------- driver

def verify2(fleet, d_min: float, *, cap: float | None = None, tol: float = 1e-9,
            per_pair: bool = False) -> Verify2Result:
    """Rigorous lower bound on the minimum pairwise separation of a fleet and a verdict.

    ``cap`` bounds the work: distances above it are not resolved, so ``lower_bound`` is at most
    ``cap`` (default ``2 * d_min + 1``). With ``per_pair=True`` every drone pair is resolved to
    ``tol`` against its own best witness (``pair_lower``/``pair_upper``); otherwise pruning uses
    the global best witness, which is much cheaper and gives the same overall bounds.
    Verdict: PASS if ``lower_bound >= d_min``; FAIL if a witnessed distance (plus the rounding
    allowance) is below ``d_min``; else UNDECIDED.
    """
    t_start = time.perf_counter()
    cap = max(float(cap) if cap is not None else 2.0 * d_min + 1.0, d_min)
    drone, lo, hi, t0r, cf = _pieces(fleet)
    nd = len(fleet.trajectories)
    K = cf.shape[1]
    gamma = 4.0 * (K + 1) ** 2
    empty = Verify2Result("PASS" if cap >= d_min else "UNDECIDED", d_min, cap, math.inf, 0.0,
                          tol, cap, 0.0, nd, 0, 0, 0, 0.0)
    if len(drone) == 0:
        return empty
    blo, bhi = _piece_boxes(lo, hi, t0r, cf, gamma)
    ia, ib = _candidate_piece_pairs(drone, lo, hi, blo, bhi, cap)
    npp = len(ia)
    if npp == 0:
        empty.runtime_s = time.perf_counter() - t_start
        return empty

    # Root polynomials D(s) = A(a + s) - B(a + s), s in [0, h].
    a = np.maximum(lo[ia], lo[ib])
    b = np.minimum(hi[ia], hi[ib])
    fin = np.isfinite(a) & np.isfinite(b)
    a = np.where(fin, a, np.where(np.isfinite(a), a, np.where(np.isfinite(b), b, 0.0)))
    h = np.where(fin, np.maximum(b - a, 0.0), 0.0)
    xa = np.where(fin, a - t0r[ia], 0.0)
    xb = np.where(fin, a - t0r[ib], 0.0)
    SA = _taylor_shift(cf[ia], xa)
    SB = _taylor_shift(cf[ib], xb)
    D = SA - SB
    mag = (np.linalg.norm(_abs_eval(cf[ia], np.abs(xa) + h), axis=1)
           + np.linalg.norm(_abs_eval(cf[ib], np.abs(xb) + h), axis=1)
           + np.linalg.norm(_abs_eval(D, h), axis=1))
    E = gamma * EPS * mag
    lo_d, hi_d = np.minimum(drone[ia], drone[ib]), np.maximum(drone[ia], drone[ib])
    key = lo_d * nd + hi_d
    ukeys, dp_of_pp = np.unique(key, return_inverse=True)
    ndp = len(ukeys)

    u = np.full(ndp, np.inf)            # best witnessed distance per drone pair
    uE = np.full(ndp, np.inf)           # same, inflated by the rounding allowance
    leaf_lb = np.full(ndp, np.inf)      # lowest bound among unresolved leaf cells
    pid = np.arange(npp)
    mid = 0.5 * h
    w = 0.5 * h
    level, n_cells, n_leaves = 0, 0, 0
    while len(pid):
        m_all = len(pid)
        n_cells += m_all
        lbd = np.empty(m_all)
        for s in range(0, m_all, _CHUNK):
            sl = slice(s, s + _CHUNK)
            p = pid[sl]
            l_, ub_, ue_ = _cell_bounds(D, p, mid[sl], w[sl], E[p], gamma)
            lbd[sl] = l_
            dp = dp_of_pp[p]
            np.minimum.at(u, dp, ub_)
            np.minimum.at(uE, dp, ue_)
        dps = dp_of_pp[pid]
        if per_pair:
            thr = np.minimum(u[dps] - tol, cap)
        else:
            thr = np.full(m_all, min(float(u.min()) - tol, cap))
        alive = lbd < thr
        pid, mid, w, lbd, dps = pid[alive], mid[alive], w[alive], lbd[alive], dps[alive]
        if not len(pid):
            break
        leaf = (w <= 0.0) | (level >= _MAX_LEVEL) | (2 * len(pid) > _MAX_ITEMS)
        if leaf.any():
            n_leaves += int(leaf.sum())
            np.minimum.at(leaf_lb, dps[leaf], lbd[leaf])
        pid, mid, w = pid[~leaf], mid[~leaf], w[~leaf]
        hw = 0.5 * w
        pid = np.concatenate([pid, pid])
        mid = np.concatenate([mid - hw, mid + hw])
        w = np.concatenate([hw, hw])
        level += 1

    # Assemble bounds. Discarded cells all had bound >= the final pruning level.
    if per_pair:
        thr_dp = np.minimum(u - tol, cap)
        l_dp = np.maximum(np.minimum(leaf_lb, thr_dp), 0.0)
        l_dp = np.minimum(l_dp, u)
        lower = float(min(cap, l_dp.min()))
    else:
        thr_all = min(float(u.min()) - tol, cap)
        lower = float(max(min(float(leaf_lb.min()), thr_all), 0.0))
        lower = min(lower, float(u.min()), cap)
    upper = float(u.min())
    witness = float(uE.min())
    if lower >= d_min:
        verdict = "PASS"
    elif witness < d_min:
        verdict = "FAIL"
    else:
        verdict = "UNDECIDED"
    res = Verify2Result(verdict, d_min, lower, upper, tol + float(E.max()), tol, cap,
                        float(E.max()), nd, npp, n_cells, n_leaves,
                        time.perf_counter() - t_start)
    if per_pair:
        for q, kq in enumerate(ukeys):
            ij = (int(kq // nd), int(kq % nd))
            res.pair_lower[ij] = float(l_dp[q])
            res.pair_upper[ij] = float(u[q])
    return res
