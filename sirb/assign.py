"""Drone-to-target assignment for one formation transition (M2).

``cost`` is squared Euclidean distance for the Hungarian method (minimising the sum of squared
distances is the standard choice for synchronous straight-line motion, because it tends to
avoid crossings); a total-length variant is also provided. Baselines: identity (index order),
seeded random, and greedy nearest. Metrics describe *synchronous constant-velocity straight
lines*, the simplest transition model: total path length and number of conflicting pairs
(pairs whose centre distance drops below ``d_sep`` at some time in [0, 1], exact for linear
relative motion).
"""

from __future__ import annotations

import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.spatial.distance import cdist


def assign_hungarian_sq(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    """perm such that drone i goes to dst[perm[i]], minimising sum of squared distances."""
    cost = cdist(src, dst, "sqeuclidean")
    r, c = linear_sum_assignment(cost)
    perm = np.empty(len(src), dtype=int)
    perm[r] = c
    return perm


def assign_hungarian_len(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    cost = cdist(src, dst, "euclidean")
    r, c = linear_sum_assignment(cost)
    perm = np.empty(len(src), dtype=int)
    perm[r] = c
    return perm


def assign_identity(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    return np.arange(len(src))


def assign_random(src: np.ndarray, dst: np.ndarray, seed: int = 0) -> np.ndarray:
    return np.random.default_rng(seed).permutation(len(src))


def assign_greedy(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    """Repeatedly take the globally closest unassigned (drone, target) pair."""
    d = cdist(src, dst, "euclidean")
    n = len(src)
    perm = -np.ones(n, dtype=int)
    order = np.argsort(d, axis=None)
    used_t = np.zeros(n, bool)
    done = 0
    for flat in order:
        i, j = divmod(int(flat), n)
        if perm[i] >= 0 or used_t[j]:
            continue
        perm[i] = j
        used_t[j] = True
        done += 1
        if done == n:
            break
    return perm


def assign_pinned(src: np.ndarray, dst: np.ndarray, seed: int = 0, frac: float = 0.15) -> np.ndarray:
    """Hungarian, then ``frac`` of drones pairwise swap targets: mimics designer-pinned roles."""
    perm = assign_hungarian_sq(src, dst)
    rng = np.random.default_rng(seed)
    k = max(2, int(frac * len(src)) // 2 * 2)
    pick = rng.choice(len(src), size=k, replace=False)
    a, b = pick[: k // 2], pick[k // 2:]
    perm[a], perm[b] = perm[b].copy(), perm[a].copy()
    return perm


METHODS = {
    "identity": assign_identity,
    "random": assign_random,
    "greedy": assign_greedy,
    "hungarian_len": assign_hungarian_len,
    "hungarian_sq": assign_hungarian_sq,
    "pinned15": assign_pinned,
}


def total_path_length(src: np.ndarray, dst: np.ndarray, perm: np.ndarray) -> float:
    return float(np.linalg.norm(dst[perm] - src, axis=1).sum())


def linear_conflicts(src: np.ndarray, dst: np.ndarray, perm: np.ndarray, d_sep: float,
                     block: int = 256) -> int:
    """Pairs whose synchronous straight-line paths come closer than d_sep (exact).

    Processed in row blocks so memory is O(block * n) rather than O(n^2); the result is
    identical to the all-pairs formulation (checked in tests).
    """
    p0 = src
    v = dst[perm] - src  # displacement over u in [0, 1]
    n = len(src)
    total = 0
    for i0 in range(0, n, block):
        i1 = min(i0 + block, n)
        dp = p0[i0:i1, None, :] - p0[None, :, :]
        dv = v[i0:i1, None, :] - v[None, :, :]
        a = (dv * dv).sum(-1)
        b = (dp * dv).sum(-1)
        c = (dp * dp).sum(-1)
        with np.errstate(divide="ignore", invalid="ignore"):
            u = np.where(a > 1e-18, np.clip(-b / np.where(a > 1e-18, a, 1.0), 0.0, 1.0), 0.0)
        dmin2 = c + 2 * b * u + a * u * u
        # count each unordered pair once: columns j > i
        upper = np.arange(n)[None, :] > np.arange(i0, i1)[:, None]
        total += int(((dmin2 < d_sep * d_sep) & upper).sum())
    return total


def evaluate(src: np.ndarray, dst: np.ndarray, d_sep: float, seed: int = 0) -> dict[str, dict]:
    out = {}
    for name, fn in METHODS.items():
        perm = fn(src, dst, seed) if name in ("random", "pinned15") else fn(src, dst)
        assert sorted(perm.tolist()) == list(range(len(src)))
        out[name] = {
            "total_path_length_m": total_path_length(src, dst, perm),
            "conflicting_pairs": linear_conflicts(src, dst, perm, d_sep),
        }
    return out
