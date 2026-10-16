"""Independent trajectory verifier (exact polynomial root finding, no sampling).

Shares only the data classes of ``sirb.trajectory`` with the planner. See
``docs/VERIFIER.md`` for method, tolerance policy and guarantees.

Model: a drone sits at its first position before its first segment and at its final
position after its last segment (still occupying space). Gaps between segments (a
structural error, flagged) are bridged by holding the last position.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

import numpy as np

# Structural tolerances (absolute, fixed by the spec).
TIME_TOL = 1e-9          # s, contiguity of segments
POS_TOL = 1e-4           # m, position continuity at breakpoints
VEL_TOL = 1e-3           # m/s, velocity continuity at breakpoints
MAX_DEGREE = 7           # highest supported polynomial degree (docs/ARCHITECTURE.md); bounds memory
ACC_TOL = 1e-3           # m/s^2, acceleration continuity at breakpoints (a step = unbounded jerk)
VIOLATION_CAP = 1000     # max listed violations per kind
_ROOT_IMAG_TOL = 1e-3    # accept roots with |imag| below this (generous: extra candidates are harmless)
_GRID = 8                # safety-net grid candidates k/8 added to every root search
_CHUNK = 20000
_PAIR_BLOCK = 1 << 18      # max index pairs materialised at once by _candidate_pairs


@dataclass
class Violation:
    kind: str
    drones: tuple
    t: float
    value: float
    limit: float
    detail: str


@dataclass
class VerifyReport:
    ok: bool
    min_separation: float
    min_separation_is_lower_bound: bool
    min_separation_pair: tuple | None
    min_separation_time: float | None
    max_speed_h: float
    max_speed_up: float
    max_speed_down: float
    max_accel: float
    max_jerk: float
    min_z: float
    max_z: float
    geofence_margin: float
    crowd_margin: float
    violations: list
    n_drones: int
    n_segments: int
    n_pairs_checked: int
    runtime_s: float
    tolerances: dict
    violation_counts: dict = field(default_factory=dict)
    violations_truncated: bool = False


# ---------------------------------------------------------------------------
# Polynomial helpers (all batched; coefficient arrays are ascending powers)
# ---------------------------------------------------------------------------

def _polyval_rows(c: np.ndarray, s: np.ndarray) -> np.ndarray:
    """c: (m, L) ascending coefficients, s: (m, W) -> (m, W)."""
    out = np.zeros(s.shape)
    for k in range(c.shape[1] - 1, -1, -1):
        out = out * s + c[:, k][:, None]
    return out


def _critical_points(df: np.ndarray) -> np.ndarray:
    """Candidate parameters in [0,1] (nan = invalid) for the stationary points of a
    polynomial whose derivative has ascending coefficients ``df`` (m, L).

    Roots come from batched companion-matrix eigenvalues (exact root finding), are
    polished with Newton steps on ``df`` and are supplemented by the endpoints and a
    coarse grid as a safety net against ill-conditioned root finding. Because every
    candidate is later evaluated directly on the polynomial, extra candidates can never
    produce a wrong (too small) minimum, only help.
    """
    m, L = df.shape
    grid = np.broadcast_to(np.linspace(0.0, 1.0, _GRID + 1), (m, _GRID + 1))
    if L < 2:
        return np.array(grid)
    roots = np.full((m, L - 1), np.nan)
    with np.errstate(all="ignore"):
        scale = np.max(np.abs(df), axis=1)
        finite = np.isfinite(scale)
        big = (np.abs(df) > (1e-13 * scale)[:, None]) & finite[:, None]
        any_big = big.any(axis=1)
        deg = np.where(any_big, L - 1 - np.argmax(big[:, ::-1], axis=1), 0)
        for q in np.unique(deg[any_big]):
            if q < 1:
                continue
            rows = np.nonzero(any_big & (deg == q))[0]
            c = df[rows, : q + 1]
            comp = np.zeros((len(rows), q, q))
            if q > 1:
                comp[:, np.arange(1, q), np.arange(0, q - 1)] = 1.0
            comp[:, :, q - 1] = -c[:, :q] / c[:, q:q + 1]
            try:
                ev = np.linalg.eigvals(comp)
            except np.linalg.LinAlgError:
                ev = np.full((len(rows), q), np.nan + 0j)
            re, im = ev.real, ev.imag
            ok = (np.abs(im) < _ROOT_IMAG_TOL) & (re > -1e-3) & (re < 1 + 1e-3)
            roots[rows, :q] = np.where(ok, np.clip(re, 0.0, 1.0), np.nan)
        # Newton polish on the derivative polynomial.
        if L >= 3:
            d2 = df[:, 1:] * np.arange(1, L)
            s = np.nan_to_num(roots, nan=0.0)
            valid = ~np.isnan(roots)
            for _ in range(2):
                f1 = _polyval_rows(df, s)
                f2 = _polyval_rows(d2, s)
                step = np.where(np.abs(f2) > 1e-300, f1 / f2, 0.0)
                s_new = np.clip(s - step, 0.0, 1.0)
                better = np.abs(_polyval_rows(df, s_new)) <= np.abs(f1)
                s = np.where(better & np.isfinite(s_new), s_new, s)
            roots = np.where(valid, s, np.nan)
    return np.concatenate([roots, grid], axis=1)


def _extremes(c: np.ndarray):
    """Exact extrema of polynomials on [0,1]. c: (m, L). Returns
    (fmin, smin, fmax, smax), each (m,)."""
    m, L = c.shape
    df = c[:, 1:] * np.arange(1, L) if L > 1 else np.zeros((m, 0))
    s = _critical_points(df)
    bad = np.isnan(s)
    s0 = np.where(bad, 0.0, s)
    v = _polyval_rows(c, s0)
    vmin = np.where(bad, np.inf, v)
    vmax = np.where(bad, -np.inf, v)
    imin = np.argmin(vmin, axis=1)
    imax = np.argmax(vmax, axis=1)
    r = np.arange(m)
    return vmin[r, imin], s0[r, imin], vmax[r, imax], s0[r, imax]


_BINOM_CACHE: dict = {}


def _binom(K: int) -> np.ndarray:
    if K not in _BINOM_CACHE:
        B = np.zeros((K, K))
        for j in range(K):
            for k in range(j + 1):
                B[k, j] = math.comb(j, k)
        _BINOM_CACHE[K] = B
    return _BINOM_CACHE[K]


def _shift(coef: np.ndarray, tau0: np.ndarray, h: np.ndarray) -> np.ndarray:
    """Re-express vector polynomials p(tau) (n,K,3) as q(s)=p(tau0+h*s), s in [0,1]."""
    n, K, _ = coef.shape
    if K == 1:
        return coef.copy()
    kk = np.arange(K)
    E = kk[None, :] - kk[:, None]          # E[k, j] = j - k
    mask = E >= 0
    Ez = np.where(mask, E, 0).astype(float)
    with np.errstate(all="ignore"):
        pw = np.where(mask[None], tau0[:, None, None] ** Ez[None], 0.0)
    T = _binom(K)[None] * pw * (h[:, None] ** kk[None, :])[:, :, None]
    return np.einsum("nkj,njc->nkc", T, coef)


def _horner_rows(coef: np.ndarray, s: np.ndarray) -> np.ndarray:
    """coef (n,K,3), s (n,W) -> (n,W,3)."""
    out = np.zeros(s.shape + (3,))
    for k in range(coef.shape[1] - 1, -1, -1):
        out = out * s[..., None] + coef[:, k, :][:, None, :]
    return out


def _deriv(a: np.ndarray) -> np.ndarray:
    K = a.shape[1]
    if K == 1:
        return np.zeros_like(a)
    return a[:, 1:, :] * np.arange(1, K)[None, :, None]


def _sq_poly(a: np.ndarray, cols) -> np.ndarray:
    """Squared-norm polynomial of vector polynomials a (n,K,3) over given columns."""
    n, K, _ = a.shape
    P = np.zeros((n, 2 * K - 1))
    for i in range(K):
        for j in range(K):
            P[:, i + j] += np.sum(a[:, i, cols] * a[:, j, cols], axis=1)
    return P


def _over(value: float, limit: float, tol: float) -> bool:
    return value > limit + tol * (1.0 + abs(limit))


def _under(value: float, limit: float, tol: float) -> bool:
    return value < limit - tol * (1.0 + abs(limit))


# ---------------------------------------------------------------------------
# Structure
# ---------------------------------------------------------------------------

def _eval_vec(c: np.ndarray, tau: float, order: int = 0) -> np.ndarray:
    c = np.array(c, dtype=float)
    for _ in range(order):
        if c.shape[0] == 1:
            return np.zeros(3)
        c = c[1:] * np.arange(1, c.shape[0])[:, None]
    out = np.zeros(3)
    for k in range(c.shape[0] - 1, -1, -1):
        out = out * tau + c[k]
    return out


def _structure(fleet, viol: list):
    """Structural checks. Returns (good_segments, bad_drones) where good_segments is a
    list per drone of (seg_index, t0, t1, coeffs) usable for geometry."""
    per_drone = []
    bad = set()
    for i, tr in enumerate(fleet.trajectories):
        good = []
        segs = list(tr.segments)
        if len(segs) == 0:
            viol.append(Violation("time_gap", (i,), 0.0, 0.0, 0.0, "drone has no segments"))
            bad.add(i)
            per_drone.append(good)
            continue
        prev = None
        for k, sg in enumerate(segs):
            try:
                t0, t1 = float(sg.t0), float(sg.t1)
                c = np.array(sg.coeffs, dtype=float)
            except Exception as e:  # noqa: BLE001
                viol.append(Violation("nonfinite", (i,), 0.0, float("nan"), 0.0,
                                      f"segment {k}: unreadable ({e})"))
                bad.add(i)
                prev = None
                continue
            if c.ndim != 2 or c.shape[1] != 3 or c.shape[0] < 1:
                viol.append(Violation("nonfinite", (i,), t0, float("nan"), 0.0,
                                      f"segment {k}: malformed coefficient shape {c.shape}"))
                bad.add(i)
                prev = None
                continue
            if not (math.isfinite(t0) and math.isfinite(t1) and np.all(np.isfinite(c))):
                viol.append(Violation("nonfinite", (i,), t0 if math.isfinite(t0) else 0.0,
                                      float("nan"), 0.0, f"segment {k}: NaN/inf in times or coefficients"))
                bad.add(i)
                prev = None
                continue
            nz = np.nonzero(np.any(c != 0.0, axis=1))[0]
            c = c[: int(nz[-1]) + 1] if len(nz) else c[:1]      # drop trailing zero rows
            if c.shape[0] - 1 > MAX_DEGREE:
                viol.append(Violation("unsupported_degree", (i,), t0, float(c.shape[0] - 1),
                                      float(MAX_DEGREE),
                                      f"segment {k}: degree {c.shape[0] - 1} exceeds the "
                                      f"supported maximum {MAX_DEGREE}"))
                bad.add(i)
                prev = None
                continue
            if not t1 - t0 > 0.0:
                viol.append(Violation("time_gap", (i,), t0, t1 - t0, 0.0,
                                      f"segment {k}: non-positive duration"))
            else:
                good.append((k, t0, t1, c))
            if prev is not None:
                pk, pt0, pt1, pc = prev
                gap = t0 - pt1
                if abs(gap) > TIME_TOL:
                    viol.append(Violation("time_gap", (i,), pt1, gap, TIME_TOL,
                                          f"between segments {pk} and {k}: "
                                          f"{'gap' if gap > 0 else 'overlap'} of {gap:.3g} s"))
                pe = _eval_vec(pc, pt1 - pt0)
                ps = _eval_vec(c, 0.0)
                dj = float(np.linalg.norm(pe - ps))
                if dj > POS_TOL:
                    viol.append(Violation("position_jump", (i,), pt1, dj, POS_TOL,
                                          f"between segments {pk} and {k}: jump of {dj:.4g} m"))
                dv = float(np.linalg.norm(_eval_vec(pc, pt1 - pt0, 1) - _eval_vec(c, 0.0, 1)))
                if dv > VEL_TOL:
                    viol.append(Violation("velocity_discontinuity", (i,), pt1, dv, VEL_TOL,
                                          f"between segments {pk} and {k}: velocity jump of "
                                          f"{dv:.4g} m/s (unbounded acceleration)"))
                da = float(np.linalg.norm(_eval_vec(pc, pt1 - pt0, 2) - _eval_vec(c, 0.0, 2)))
                if da > ACC_TOL:
                    viol.append(Violation("accel_discontinuity", (i,), pt1, da, ACC_TOL,
                                          f"between segments {pk} and {k}: acceleration jump "
                                          f"of {da:.4g} m/s^2 (unbounded jerk)"))
            prev = (k, t0, t1, c)
        per_drone.append(good)
        if not good:
            bad.add(i)
    return per_drone, bad


# ---------------------------------------------------------------------------
# Pairwise separation
# ---------------------------------------------------------------------------

class _Pieces:
    pass


def _build_pieces(good_per_drone, bad) -> _Pieces:
    drone, lo, hi, t0r, coefs = [], [], [], [], []
    for i, good in enumerate(good_per_drone):
        if i in bad or not good:
            continue
        first = good[0]
        drone.append(i); lo.append(-np.inf); hi.append(first[1]); t0r.append(0.0)
        coefs.append(first[3][:1].copy())
        cursor = first[1]
        last_pos = first[3][0].copy()
        for (_, t0, t1, c) in good:
            if t1 <= cursor:
                continue
            if t0 > cursor:   # gap: hold position
                drone.append(i); lo.append(cursor); hi.append(t0); t0r.append(0.0)
                coefs.append(last_pos[None, :].copy())
                cursor = t0
            a = max(t0, cursor)
            drone.append(i); lo.append(a); hi.append(t1); t0r.append(t0); coefs.append(c)
            cursor = t1
            last_pos = _eval_vec(c, t1 - t0)
        drone.append(i); lo.append(cursor); hi.append(np.inf); t0r.append(0.0)
        coefs.append(last_pos[None, :].copy())
    P = _Pieces()
    P.n = len(drone)
    P.drone = np.array(drone, dtype=np.int64)
    P.lo = np.array(lo, dtype=float)
    P.hi = np.array(hi, dtype=float)
    P.t0r = np.array(t0r, dtype=float)
    P.K = max((c.shape[0] for c in coefs), default=1)
    P.coef = np.zeros((P.n, P.K, 3))
    for m, c in enumerate(coefs):
        P.coef[m, : c.shape[0]] = c
    # Exact axis-aligned boxes.
    P.blo = np.zeros((P.n, 3))
    P.bhi = np.zeros((P.n, 3))
    real = np.nonzero(np.isfinite(P.lo) & np.isfinite(P.hi))[0]
    virt = np.nonzero(~(np.isfinite(P.lo) & np.isfinite(P.hi)))[0]
    P.blo[virt] = P.coef[virt, 0, :]
    P.bhi[virt] = P.coef[virt, 0, :]
    for s in range(0, len(real), _CHUNK):
        idx = real[s:s + _CHUNK]
        sh = _shift(P.coef[idx], P.lo[idx] - P.t0r[idx], P.hi[idx] - P.lo[idx])
        for ax in range(3):
            fmin, _, fmax, _ = _extremes(sh[:, :, ax])
            P.blo[idx, ax] = fmin
            P.bhi[idx, ax] = fmax
    return P


def _candidate_pairs(P: _Pieces, R: float, prefilter: bool):
    """Piece-index pairs (ia, ib) of different drones with overlapping time intervals
    whose boxes are not farther apart than R (all such pairs if prefilter is False).
    Vectorised over blocks of rows; element for element, in the same order, equal to
    ``_candidate_pairs_loop`` (rows ascending in start-time order, then columns)."""
    order = np.argsort(P.lo, kind="stable")
    lo, hi = P.lo[order], P.hi[order]
    blo, bhi, dr = P.blo[order], P.bhi[order], P.drone[order]
    ends = np.searchsorted(lo, hi, side="left")
    R2 = (R * (1 + 1e-9) + 1e-9) ** 2 if prefilter else np.inf
    cnt = np.maximum(ends - np.arange(1, P.n + 1), 0)   # time-overlapping partners j > i of row i
    cum = np.concatenate([[0], np.cumsum(cnt)])
    A, B = [], []
    r0 = 0
    while r0 < P.n:
        # Rows r0..r1-1 hold at most _PAIR_BLOCK index pairs (a single wider row is its own block).
        r1 = int(np.searchsorted(cum, cum[r0] + _PAIR_BLOCK, side="right")) - 1
        r1 = min(max(r1, r0 + 1), P.n)
        c = cnt[r0:r1]
        tot = int(cum[r1] - cum[r0])
        if tot:
            ii = np.repeat(np.arange(r0, r1), c)
            jj = ii + 1 + (np.arange(tot) - np.repeat(cum[r0:r1] - cum[r0], c))
            sel = dr[jj] != dr[ii]
            ii, jj = ii[sel], jj[sel]
            if prefilter:
                gap = np.maximum(np.maximum(blo[jj] - bhi[ii], blo[ii] - bhi[jj]), 0.0)
                sel = np.einsum("ij,ij->i", gap, gap) <= R2
                ii, jj = ii[sel], jj[sel]
            if len(ii):
                A.append(ii)
                B.append(jj)
        r0 = r1
    if not A:
        return np.zeros(0, np.int64), np.zeros(0, np.int64)
    return order[np.concatenate(A)], order[np.concatenate(B)]


def _candidate_pairs_loop(P: _Pieces, R: float, prefilter: bool):
    """Reference implementation of ``_candidate_pairs`` (one Python iteration per piece);
    kept only so tests can assert that the vectorised version returns identical arrays."""
    order = np.argsort(P.lo, kind="stable")
    lo, hi = P.lo[order], P.hi[order]
    blo, bhi, dr = P.blo[order], P.bhi[order], P.drone[order]
    ends = np.searchsorted(lo, hi, side="left")
    R2 = (R * (1 + 1e-9) + 1e-9) ** 2 if prefilter else np.inf
    A, B = [], []
    for i in range(P.n):
        j_end = ends[i]
        if j_end <= i + 1:
            continue
        sl = slice(i + 1, j_end)
        sel = dr[sl] != dr[i]
        if prefilter:
            gap = np.maximum(np.maximum(blo[sl] - bhi[i], blo[i] - bhi[sl]), 0.0)
            sel &= np.einsum("ij,ij->i", gap, gap) <= R2
        js = np.nonzero(sel)[0]
        if len(js):
            A.append(np.full(len(js), i, dtype=np.int64))
            B.append(js + i + 1)
    if not A:
        return np.zeros(0, np.int64), np.zeros(0, np.int64)
    return order[np.concatenate(A)], order[np.concatenate(B)]


_REFINE_HALF_WIDTH = 5e-3   # fraction of the overlap interval searched around the best candidate
_REFINE_ITERS = 60


def _refine_minimum(D: np.ndarray, d_best: np.ndarray, s_best: np.ndarray, below: float):
    """Local golden-section refinement of |D(s)| around the best candidate.

    Companion-matrix roots are only accurate to about eps^(1/k) at a k-fold root, so a
    tangent or colliding pair can be reported slightly too far apart (found by differential
    testing, docs/VERIFIER.md). Every value returned is a direct evaluation of the polynomial
    at a real time, so refinement can only lower the reported minimum towards the true one,
    never below it. Only rows already below ``below`` are refined, to bound the cost.
    """
    rows = np.nonzero(np.isfinite(d_best) & (d_best < below))[0]
    if len(rows) == 0:
        return d_best, s_best
    Dr = D[rows]
    lo = np.clip(s_best[rows] - _REFINE_HALF_WIDTH, 0.0, 1.0)
    hi = np.clip(s_best[rows] + _REFINE_HALF_WIDTH, 0.0, 1.0)
    g = 0.6180339887498949
    x1 = hi - g * (hi - lo)
    x2 = lo + g * (hi - lo)

    def f(x):
        return np.linalg.norm(_horner_rows(Dr, x[:, None])[:, 0, :], axis=1)

    f1, f2 = f(x1), f(x2)
    for _ in range(_REFINE_ITERS):
        left = f1 <= f2
        hi = np.where(left, x2, hi)
        lo = np.where(left, lo, x1)
        x1n = hi - g * (hi - lo)
        x2n = lo + g * (hi - lo)
        x1, x2 = np.where(left, x1n, x2), np.where(left, x1, x2n)
        f1n = np.where(left, f(x1), f2)
        f2n = np.where(left, f1, f(x2))
        f1, f2 = f1n, f2n
    xm = 0.5 * (lo + hi)
    fm = f(xm)
    better = fm < d_best[rows]
    d_new, s_new = d_best.copy(), s_best.copy()
    d_new[rows[better]] = fm[better]
    s_new[rows[better]] = xm[better]
    return d_new, s_new


def _solve_pairs(P: _Pieces, ia: np.ndarray, ib: np.ndarray, refine_below: float = np.inf):
    """Exact minimum centre distance (and its time) for each piece pair over the
    overlap of the two pieces."""
    n = len(ia)
    dist = np.full(n, np.inf)
    tmin = np.zeros(n)
    for s in range(0, n, _CHUNK):
        a_i, b_i = ia[s:s + _CHUNK], ib[s:s + _CHUNK]
        a = np.maximum(P.lo[a_i], P.lo[b_i])
        b = np.minimum(P.hi[a_i], P.hi[b_i])
        with np.errstate(invalid="ignore"):
            fin = np.isfinite(a) & np.isfinite(b) & (b - a > 1e-12)
        # Degenerate / unbounded overlaps: both pieces are then constant or the overlap
        # is a single point; evaluate directly.
        pt = np.nonzero(~fin)[0]
        if len(pt):
            tp = np.where(np.isfinite(a[pt]), a[pt], b[pt])
            pa = _horner_rows(P.coef[a_i[pt]], (tp - P.t0r[a_i[pt]])[:, None])[:, 0, :]
            pb = _horner_rows(P.coef[b_i[pt]], (tp - P.t0r[b_i[pt]])[:, None])[:, 0, :]
            dist[s + pt] = np.linalg.norm(pa - pb, axis=1)
            tmin[s + pt] = tp
        iv = np.nonzero(fin)[0]
        if len(iv):
            ai, bi = a_i[iv], b_i[iv]
            h = b[iv] - a[iv]
            SA = _shift(P.coef[ai], a[iv] - P.t0r[ai], h)
            SB = _shift(P.coef[bi], a[iv] - P.t0r[bi], h)
            D = SA - SB
            Pq = _sq_poly(D, slice(0, 3))
            dP = Pq[:, 1:] * np.arange(1, Pq.shape[1])
            sc = _critical_points(dP)
            bad = np.isnan(sc)
            sc = np.where(bad, 0.0, sc)
            sc = np.concatenate([sc, np.zeros((len(iv), 1)), np.ones((len(iv), 1))], axis=1)
            bad = np.concatenate([bad, np.zeros((len(iv), 2), bool)], axis=1)
            vec = _horner_rows(D, sc)
            nrm = np.where(bad, np.inf, np.linalg.norm(vec, axis=2))
            best = np.argmin(nrm, axis=1)
            r = np.arange(len(iv))
            d_best = nrm[r, best]
            s_best = sc[r, best]
            d_best, s_best = _refine_minimum(D, d_best, s_best, refine_below)
            dist[s + iv] = d_best
            tmin[s + iv] = a[iv] + s_best * h
    return dist, tmin


def pair_minima(fleet, search_radius: float, *, prefilter: bool = True, _pieces=None):
    """Exact per-drone-pair minimum distances for pairs whose minimum is below
    ``search_radius``. Returns (dict {(i,j): (dmin, t)}, n_pairs_checked)."""
    if _pieces is None:
        viol: list = []
        good, bad = _structure(fleet, viol)
        _pieces = _build_pieces(good, bad)
    P = _pieces
    if P.n == 0:
        return {}, 0
    ia, ib = _candidate_pairs(P, search_radius, prefilter)
    if len(ia) == 0:
        return {}, 0
    dist, t = _solve_pairs(P, ia, ib, refine_below=search_radius)
    da, db = P.drone[ia], P.drone[ib]
    lo_d, hi_d = np.minimum(da, db), np.maximum(da, db)
    key = lo_d * (int(P.drone.max()) + 1) + hi_d
    o = np.lexsort((dist, key))
    key_s = key[o]
    first = np.ones(len(o), bool)
    first[1:] = key_s[1:] != key_s[:-1]
    sel = o[first]
    out = {}
    for q in sel:
        if dist[q] < search_radius:
            out[(int(lo_d[q]), int(hi_d[q]))] = (float(dist[q]), float(t[q]))
    return out, int(first.sum())


def pairwise_min_separation(fleet, search_radius: float):
    """Returns (dmin, i, j, t, is_lower_bound, n_pairs_checked). If no pair is below
    ``search_radius`` the result is (search_radius, None, None, None, True, n)."""
    res, n = pair_minima(fleet, search_radius)
    if not res:
        return float(search_radius), None, None, None, True, n
    (i, j), (d, t) = min(res.items(), key=lambda kv: kv[1][0])
    return d, i, j, t, False, n


# ---------------------------------------------------------------------------
# Dynamics
# ---------------------------------------------------------------------------

def _dynamics(good_per_drone, bad, limits, tol, viol: list):
    segs = [(i, k, t0, t1 - t0, c) for i, good in enumerate(good_per_drone)
            if i not in bad for (k, t0, t1, c) in good]
    stats = dict(max_speed_h=0.0, max_speed_up=0.0, max_speed_down=0.0, max_accel=0.0,
                 max_jerk=0.0, min_z=math.inf, max_z=-math.inf,
                 geofence_margin=math.inf, crowd_margin=math.inf)
    if not segs:
        return stats
    n = len(segs)
    K = max(s[4].shape[0] for s in segs)
    cs = np.zeros((n, K, 3))
    hh = np.array([s[3] for s in segs])
    t0s = np.array([s[2] for s in segs])
    for m, (_, _, _, h, c) in enumerate(segs):
        cs[m, : c.shape[0]] = c * (h ** np.arange(c.shape[0]))[:, None]
    V = _deriv(cs); A = _deriv(V); J = _deriv(A)

    def add(kind, m, s, value, limit, what):
        i, k = segs[m][0], segs[m][1]
        viol.append(Violation(kind, (i,), float(t0s[m] + s * hh[m]), float(value), float(limit),
                              f"segment {k}: {what}"))

    vmax_h = limits.get("vmax_h")
    _, _, fmax, smax = _extremes(_sq_poly(V, [0, 1]))
    sp = np.sqrt(np.maximum(fmax, 0)) / hh
    stats["max_speed_h"] = float(sp.max())
    if vmax_h is not None:
        for m in np.nonzero(sp > vmax_h + tol * (1 + abs(vmax_h)))[0]:
            add("speed_h", m, smax[m], sp[m], vmax_h, "horizontal speed")

    zmin_v, zs_min, zmax_v, zs_max = _extremes(V[:, :, 2])
    vz_max, vz_min = zmax_v / hh, zmin_v / hh
    stats["max_speed_up"] = float(max(0.0, vz_max.max()))
    stats["max_speed_down"] = float(max(0.0, (-vz_min).max()))
    if limits.get("vmax_up") is not None:
        L = limits["vmax_up"]
        for m in np.nonzero(vz_max > L + tol * (1 + abs(L)))[0]:
            add("speed_up", m, zs_max[m], vz_max[m], L, "climb rate")
    if limits.get("vmax_down") is not None:
        L = limits["vmax_down"]
        for m in np.nonzero(-vz_min > L + tol * (1 + abs(L)))[0]:
            add("speed_down", m, zs_min[m], -vz_min[m], L, "descent rate")

    _, _, fmax, smax = _extremes(_sq_poly(A, [0, 1, 2]))
    ac = np.sqrt(np.maximum(fmax, 0)) / hh ** 2
    stats["max_accel"] = float(ac.max())
    if limits.get("amax") is not None:
        L = limits["amax"]
        for m in np.nonzero(ac > L + tol * (1 + abs(L)))[0]:
            add("accel", m, smax[m], ac[m], L, "acceleration")

    _, _, fmax, smax = _extremes(_sq_poly(J, [0, 1, 2]))
    jk = np.sqrt(np.maximum(fmax, 0)) / hh ** 3
    stats["max_jerk"] = float(jk.max())
    if limits.get("jmax") is not None:
        L = limits["jmax"]
        for m in np.nonzero(jk > L + tol * (1 + abs(L)))[0]:
            add("jerk", m, smax[m], jk[m], L, "jerk")

    xmin, xsmin, xmax, xsmax = _extremes(cs[:, :, 0])
    ymin, ysmin, ymax, ysmax = _extremes(cs[:, :, 1])
    zmn, zsmn, zmx, zsmx = _extremes(cs[:, :, 2])
    stats["min_z"] = float(zmn.min())
    stats["max_z"] = float(zmx.max())
    zlo, zhi = limits.get("z_min"), limits.get("z_max")
    if zhi is not None:
        for m in np.nonzero(zmx > zhi + tol * (1 + abs(zhi)))[0]:
            add("altitude", m, zsmx[m], zmx[m], zhi, "altitude above z_max")
    if zlo is not None:
        for m in np.nonzero(zmn < zlo - tol * (1 + abs(zlo)))[0]:
            add("altitude", m, zsmn[m], zmn[m], zlo, "altitude below z_min")
    gf = limits.get("geofence")
    if gf:
        for ax, (lo_a, hi_a, name) in {0: (xmin, xmax, "x"), 1: (ymin, ymax, "y")}.items():
            a, b = gf[name]
            smn = xsmin if ax == 0 else ysmin
            smx = xsmax if ax == 0 else ysmax
            stats["geofence_margin"] = min(stats["geofence_margin"],
                                           float((lo_a - a).min()), float((b - hi_a).min()))
            for m in np.nonzero(hi_a > b + tol * (1 + abs(b)))[0]:
                add("geofence", m, smx[m], hi_a[m], b, f"{name} above geofence max")
            for m in np.nonzero(lo_a < a - tol * (1 + abs(a)))[0]:
                add("geofence", m, smn[m], lo_a[m], a, f"{name} below geofence min")
    cr = limits.get("crowd")
    if cr:
        need = cr["y"] + cr["min_distance_m"]
        stats["crowd_margin"] = float((ymin - need).min())
        for m in np.nonzero(ymin < need - tol * (1 + abs(need)))[0]:
            add("crowd", m, ysmin[m], ymin[m], need, "too close to crowd region")
    return stats


# ---------------------------------------------------------------------------
# Top level
# ---------------------------------------------------------------------------

def verify(fleet, limits: dict, *, search_radius: float | None = None,
           tol: float = 1e-6) -> VerifyReport:
    t_wall = time.perf_counter()
    min_sep = float(limits["min_separation_m"])
    R = float(search_radius) if search_radius is not None else 4.0 * min_sep
    R = max(R, 1.01 * min_sep + 1e-3)
    viol: list = []
    good, bad = _structure(fleet, viol)
    stats = _dynamics(good, bad, limits, tol, viol)

    pieces = _build_pieces(good, bad)
    pm, n_checked = pair_minima(fleet, R, _pieces=pieces)
    sep_thr = min_sep - tol * (1 + abs(min_sep))
    sep_viol = []
    for (i, j), (d, t) in pm.items():
        if d < sep_thr:
            sep_viol.append(Violation("separation", (i, j), t, d, min_sep,
                                      f"centre distance {d:.6g} m < {min_sep} m"))
    if pm:
        (bi, bj), (bd, bt) = min(pm.items(), key=lambda kv: kv[1][0])
        dmin, pair, tt, lb = bd, (bi, bj), bt, False
    else:
        dmin, pair, tt, lb = R, None, None, True

    counts: dict = {}
    out: list = []
    truncated = False
    for kind in ("separation", "speed_h", "speed_up", "speed_down", "accel", "jerk", "altitude",
                 "geofence", "crowd", "time_gap", "position_jump", "velocity_discontinuity",
                 "accel_discontinuity", "unsupported_degree",
                 "nonfinite"):
        items = sep_viol if kind == "separation" else [v for v in viol if v.kind == kind]
        if not items:
            continue
        counts[kind] = len(items)
        if kind == "separation":
            items = sorted(items, key=lambda v: v.value)
        else:
            items = sorted(items, key=lambda v: -abs(v.value - v.limit) if math.isfinite(v.value) else 0)
        if len(items) > VIOLATION_CAP:
            truncated = True
            items = items[:VIOLATION_CAP]
        out.extend(items)

    n_seg = sum(len(t.segments) for t in fleet.trajectories)
    tolerances = {
        "tol": tol,
        "policy": "limit violated only if exceeded by more than tol*(1+|limit|)",
        "search_radius": R,
        "time_contiguity_s": TIME_TOL,
        "position_continuity_m": POS_TOL,
        "velocity_continuity_mps": VEL_TOL,
        "acceleration_continuity_mps2": ACC_TOL,
        "violation_cap_per_kind": VIOLATION_CAP,
    }
    return VerifyReport(
        ok=(not counts), min_separation=float(dmin), min_separation_is_lower_bound=lb,
        min_separation_pair=pair, min_separation_time=tt,
        max_speed_h=stats["max_speed_h"], max_speed_up=stats["max_speed_up"],
        max_speed_down=stats["max_speed_down"], max_accel=stats["max_accel"],
        max_jerk=stats["max_jerk"], min_z=stats["min_z"], max_z=stats["max_z"],
        geofence_margin=stats["geofence_margin"], crowd_margin=stats["crowd_margin"],
        violations=out, n_drones=fleet.n, n_segments=n_seg, n_pairs_checked=n_checked,
        runtime_s=time.perf_counter() - t_wall, tolerances=tolerances,
        violation_counts=counts, violations_truncated=truncated)
