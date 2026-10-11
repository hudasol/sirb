"""Conflict-driven wave scheduling.

Each transition is split into sequential *waves*. Inside a wave every drone flies a straight leg
with one shared time profile (duration = slowest drone in the wave), so relative motion inside a
wave is linear in the profile variable and the minimum pairwise distance has a closed form.
Drones outside the wave sit still at their start (not yet moved) or target (already arrived).

Pairwise constraints, all in closed form, no sampling:
- same wave: min over s in [0,1] of |dp + s dv| >= d_plan
- i in an earlier wave than j: segment_i stays d_plan from point src_j, and segment_j stays
  d_plan from point dst_i
Waves are assigned greedily (largest conflict degree first) at the earliest position, existing or
newly inserted, that satisfies every constraint with already-assigned drones.

Not allowed to import sirb.verify.
"""

from __future__ import annotations

import time

import numpy as np

from ..assign import METHODS, linear_conflicts, total_path_length
from ..spec import build_targets
from ..trajectory import Fleet
from .common import hold_segment, leg_segment, min_leg_time, stitch


def _assign(name, src, dst, seed):
    fn = METHODS[name]
    return fn(src, dst, seed) if name in ("random", "pinned15") else fn(src, dst)


def _point_seg_dist(points: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """dist[i, j] from point j to segment i (a_i -> b_i)."""
    ab = b - a
    L2 = (ab * ab).sum(1)
    ap = points[None, :, :] - a[:, None, :]
    with np.errstate(divide="ignore", invalid="ignore"):
        t = np.where(L2[:, None] > 1e-18, (ap * ab[:, None, :]).sum(2) / np.where(
            L2[:, None] > 1e-18, L2[:, None], 1.0), 0.0)
    t = np.clip(t, 0.0, 1.0)
    proj = a[:, None, :] + t[:, :, None] * ab[:, None, :]
    return np.linalg.norm(points[None, :, :] - proj, axis=2)


def _sync_min_dist(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    dp = src[:, None, :] - src[None, :, :]
    v = dst - src
    dv = v[:, None, :] - v[None, :, :]
    a = (dv * dv).sum(-1)
    b = (dp * dv).sum(-1)
    c = (dp * dp).sum(-1)
    with np.errstate(divide="ignore", invalid="ignore"):
        u = np.where(a > 1e-18, np.clip(-b / np.where(a > 1e-18, a, 1.0), 0.0, 1.0), 0.0)
    return np.sqrt(np.maximum(c + 2 * b * u + a * u * u, 0.0))


def schedule_waves(src: np.ndarray, dst: np.ndarray, d_plan: float) -> np.ndarray:
    """Return an integer wave index per drone (0 = first). Raises RuntimeError if stuck."""
    n = len(src)
    sync_ok = _sync_min_dist(src, dst) >= d_plan
    seg_src = _point_seg_dist(src, src, dst) >= d_plan   # [i, j]: seg_i vs src_j
    seg_dst = _point_seg_dist(dst, src, dst) >= d_plan   # [i, j]: seg_i vs dst_j
    np.fill_diagonal(sync_ok, True)
    np.fill_diagonal(seg_src, True)
    np.fill_diagonal(seg_dst, True)
    degree = (~sync_ok).sum(1)
    order = np.argsort(-degree, kind="stable")
    pos = np.full(n, np.nan)  # float wave position
    done = np.zeros(n, bool)

    def feasible(j: int, w: float) -> bool:
        idx = np.where(done)[0]
        if len(idx) == 0:
            return True
        wa = pos[idx]
        same, early, late = idx[wa == w], idx[wa < w], idx[wa > w]
        return bool(sync_ok[j, same].all()
                    and seg_src[early, j].all() and seg_dst[j, early].all()
                    and seg_src[j, late].all() and seg_dst[late, j].all())

    for j in order:
        levels = np.unique(pos[done])
        cands: list[float] = list(levels)
        if len(levels) == 0:
            cands = [0.0]
        else:
            mids = (levels[:-1] + levels[1:]) / 2
            cands += list(mids) + [levels[0] - 1.0, levels[-1] + 1.0]
        # prefer joining existing waves, then the newest-wave options in time order
        cands = list(dict.fromkeys(cands))
        for w in cands:
            if feasible(int(j), float(w)):
                pos[j] = w
                done[j] = True
                break
        else:
            raise RuntimeError(f"wave scheduling stuck at drone {int(j)}")
    ranks = {v: r for r, v in enumerate(np.unique(pos))}
    return np.array([ranks[v] for v in pos])


def plan_waves(show: dict, *, assign: str = "hungarian_sq", margin: float = 0.05
               ) -> tuple[Fleet, dict]:
    t_wall = time.perf_counter()
    lim = show["limits"]
    d_plan = lim["min_separation_m"] + margin
    targets = build_targets(show)
    n = show["n_drones"]
    pos = targets[0].copy()
    pieces: list[list] = [[] for _ in range(n)]
    stages = show["stages"]
    t = stages[0].get("hold_s", 0.0)
    transitions = []
    for k in range(len(stages) - 1):
        dst_all = targets[k + 1]
        perm = _assign(assign, pos, dst_all, show["seed"] + k)
        dst = dst_all[perm]
        wave = schedule_waves(pos, dst, d_plan)
        n_waves = int(wave.max()) + 1
        t_w = t
        wave_times = []
        for w in range(n_waves):
            members = np.where(wave == w)[0]
            own = [min_leg_time(dst[i] - pos[i], lim) for i in members]
            T = max(own)
            wave_times.append(T)
            for i in members:
                if np.linalg.norm(dst[i] - pos[i]) < 1e-9:
                    continue
                pieces[i].append(leg_segment(t_w, T, pos[i], dst[i]))
            t_w += T
        for i in range(n):
            if not pieces[i] or pieces[i][-1].t1 <= t:
                pieces[i].append(hold_segment(t, max(t_w - t, 1e-6), pos[i]))
        transitions.append({
            "t_start": t, "window_s": t_w - t, "waves": n_waves, "wave_s": wave_times,
            "total_path_length_m": total_path_length(pos, dst_all, perm),
            "synchronous_conflicting_pairs": linear_conflicts(
                pos, dst_all, perm, lim["min_separation_m"]),
        })
        t = t_w + stages[k + 1].get("hold_s", 0.0)
        pos = dst
    trajs = stitch(pieces, t)
    fleet = Fleet(trajs, {"planner": f"waves(assign={assign})", "show": show["name"],
                          "seed": show["seed"]})
    return fleet, {"transitions": transitions, "t_end": t,
                   "plan_runtime_s": time.perf_counter() - t_wall}
