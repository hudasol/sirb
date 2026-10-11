"""Baseline planner: straight lines, synchronous or time-staggered in waves.

Every drone flies a straight rest-to-rest minimum-jerk leg to its assigned target. All legs of a
transition share one duration T (the slowest drone's minimum), so the motion is a scaled copy of
the straight-line model used by ``sirb.assign``. With ``waves=K`` drone i starts at
``(i mod K) * stagger_s`` after the window opens; K=1 is fully synchronous.
"""

from __future__ import annotations

import time

import numpy as np

from ..assign import METHODS, linear_conflicts, total_path_length
from ..spec import build_targets
from ..trajectory import Fleet
from .common import hold_segment, leg_segment, min_leg_time, stitch


def _assign(name: str, src, dst, seed: int):
    fn = METHODS[name]
    return fn(src, dst, seed) if name in ("random", "pinned15") else fn(src, dst)


def plan_baseline(show: dict, *, assign: str = "hungarian_sq", waves: int = 1,
                  stagger_s: float = 0.0, sync: bool = True) -> tuple[Fleet, dict]:
    t_wall = time.perf_counter()
    lim = show["limits"]
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
        own = [min_leg_time(d, lim) for d in dst - pos]
        T = max(own)
        W = T + (waves - 1) * stagger_s
        for i in range(n):
            if np.linalg.norm(dst[i] - pos[i]) < 1e-9:
                pieces[i].append(hold_segment(t, W, pos[i]))
                continue
            delay = (i % waves) * stagger_s
            pieces[i].append(leg_segment(t + delay, T if sync else own[i], pos[i], dst[i]))
        transitions.append({
            "t_start": t, "window_s": W, "leg_s": T,
            "total_path_length_m": total_path_length(pos, dst_all, perm),
            "synchronous_conflicting_pairs": linear_conflicts(
                pos, dst_all, perm, lim["min_separation_m"]),
        })
        t += W + stages[k + 1].get("hold_s", 0.0)
        pos = dst
    trajs = stitch(pieces, t)
    fleet = Fleet(trajs, {"planner": f"baseline(assign={assign},waves={waves},stagger_s={stagger_s},sync={sync})",
                          "show": show["name"], "seed": show["seed"]})
    return fleet, {"transitions": transitions, "t_end": t,
                   "plan_runtime_s": time.perf_counter() - t_wall}
