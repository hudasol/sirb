"""Delay planner: resolve conflicting straight legs by start delays, round by round.

Opt in (``--planner delays``); the baseline, waves and prioritized planners are unchanged. For
each transition the given assignment is flown on synchronous rest to rest straight legs. Pairs
that come closer than the declared threshold plus a margin are found with a sampled clearance
test padded by the Lipschitz bound (as in ``prioritized``: sampled distance must be at least
``d_min + margin + dt / 2 * (v_i + v_j)``). In each round the drones with the most conflicts form
a set of mutually non conflicting drones, and each is delayed by one more step; the process is a
greedy path through the tree of delay constraints that a conflict based search would explore.
It ends when no pair conflicts or a cap is hit; an unresolved transition is reported, not hidden.

The independent verifier (``sirb.verify``) is still the judge; this module must not import it.
"""

from __future__ import annotations

import time

import numpy as np
from scipy.spatial import cKDTree

from ..assign import METHODS
from ..spec import build_targets
from ..trajectory import Fleet
from .common import K_V, hold_segment, leg_segment, min_leg_time, stitch


def _assign(name, src, dst, seed):
    fn = METHODS[name]
    return fn(src, dst, seed) if name in ("random", "pinned15") else fn(src, dst)


def _positions(src, delta, delays, T, times):
    u = np.clip((times[:, None] - delays[None, :]) / T, 0.0, 1.0)
    s = 10 * u**3 - 15 * u**4 + 6 * u**5
    return src[None, :, :] + s[:, :, None] * delta[None, :, :]


def conflicts(src, delta, delays, T, d_req, dt):
    """Conflicting pairs {(i, j)} for synchronous legs of duration T with the given start delays."""
    n = len(src)
    W = T + float(delays.max())
    times = np.arange(0.0, W + dt / 2, dt)
    P = _positions(src, delta, delays, T, times)
    vp = K_V * np.linalg.norm(delta, axis=1) / T
    moving = np.linalg.norm(delta, axis=1) > 1e-9
    vmax = float(vp.max()) if n else 0.0
    R = d_req + dt * vmax
    out: set = set()
    for k in range(len(times)):
        tree = cKDTree(P[k])
        pr = tree.query_pairs(R, output_type="ndarray")
        if len(pr) == 0:
            continue
        i, j = pr[:, 0], pr[:, 1]
        d = np.linalg.norm(P[k, i] - P[k, j], axis=1)
        t = times[k]
        act_i = moving[i] & (t > delays[i]) & (t < delays[i] + T)
        act_j = moving[j] & (t > delays[j]) & (t < delays[j] + T)
        both_hold = ~(act_i | act_j)
        pad = dt / 2 * (np.where(moving[i], vp[i], 0.0) + np.where(moving[j], vp[j], 0.0))
        bad = (d < d_req + pad) & ~both_hold
        for a, b in zip(i[bad], j[bad]):
            out.add((int(a), int(b)))
    return out


def precedence_layers(src, dst, d_req):
    """Layer index per drone from geometry alone.

    If the straight path of A passes within ``d_req`` of the destination of B, A should pass
    before B arrives (A before B). If it passes within ``d_req`` of the start of B, B should have
    left first (B before A). Layers are longest paths in that graph; where it has cycles the
    drone with the fewest unmet predecessors is placed and its incoming edges are dropped.
    """
    n = len(src)
    d = dst - src
    L2 = np.maximum((d * d).sum(axis=1), 1e-12)

    def dist_to_paths(points):          # (n_paths, n_points) distance from each path to each point
        w = points[None, :, :] - src[:, None, :]
        u = np.clip((w * d[:, None, :]).sum(axis=2) / L2[:, None], 0.0, 1.0)
        c = src[:, None, :] + u[:, :, None] * d[:, None, :]
        return np.linalg.norm(points[None, :, :] - c, axis=2)

    near_dst = dist_to_paths(dst) < d_req
    near_src = dist_to_paths(src) < d_req
    np.fill_diagonal(near_dst, False)
    np.fill_diagonal(near_src, False)
    before = near_dst | near_src.T          # before[a, b]: a should come before b
    indeg = before.sum(axis=0).astype(int)
    layer = np.zeros(n, dtype=int)
    placed = np.zeros(n, dtype=bool)
    for _ in range(n):
        free = np.where(~placed & (indeg == 0))[0]
        if len(free) == 0:
            free = np.array([int(np.argmin(np.where(placed, 10**9, indeg)))])
        i = int(free[0])
        placed[i] = True
        for j in np.where(before[i] & ~placed)[0]:
            layer[j] = max(layer[j], layer[i] + 1)
            indeg[j] -= 1
    return layer


def plan_delay_cbs(show: dict, *, assign: str = "hungarian_sq", dt: float = 0.1,
                   margin: float = 0.05, step_s: float | None = None, max_rounds: int = 8,
                   max_delay_steps: int = 2000, layered: bool = True, time_budget_s: float = 240.0) -> tuple[Fleet, dict]:
    t_wall = time.perf_counter()
    lim = show["limits"]
    d_req = lim["min_separation_m"] + margin
    targets = build_targets(show)
    n = show["n_drones"]
    pos = targets[0].copy()
    pieces: list[list] = [[] for _ in range(n)]
    stages = show["stages"]
    t = stages[0].get("hold_s", 0.0)
    transitions = []
    for k in range(len(stages) - 1):
        perm = _assign(assign, pos, targets[k + 1], show["seed"] + k)
        dst = targets[k + 1][perm]
        delta = dst - pos
        T = max(min_leg_time(d, lim) for d in delta)
        step = step_s if step_s is not None else max(0.5, 0.15 * T)
        steps = np.zeros(n, dtype=int)
        rounds, unresolved = 0, 0
        if T > 0 and not conflicts(pos, delta, np.zeros(n), T, d_req, dt):
            pass        # synchronous legs are already clear: same plan as the baseline planner
        elif T > 0:
            layer = precedence_layers(pos, dst, d_req) if layered else np.zeros(n, dtype=int)
            gap = max(1, int(np.ceil(T / step)))
            color = np.zeros(n, dtype=int)
            ncol = 1
            for rounds in range(max_rounds + 1):
                steps = (layer * ncol + color) * gap
                conf = conflicts(pos, delta, steps * step, T, d_req, dt)
                unresolved = len(conf)
                if not conf or rounds == max_rounds or time.perf_counter() - t_wall > time_budget_s:
                    break
                # colour the conflict graph inside each layer; each colour flies in its own slot
                nb: dict = {}
                for a_, b_ in conf:
                    nb.setdefault(a_, set()).add(b_)
                    nb.setdefault(b_, set()).add(a_)
                new_color = np.zeros(n, dtype=int)
                colored = np.zeros(n, dtype=bool)
                for i in sorted(nb, key=lambda q: -len(nb[q])):
                    used = {int(new_color[j]) for j in nb[i] if layer[j] == layer[i] and colored[j]}
                    c = 0
                    while c in used:
                        c += 1
                    new_color[i] = c
                    colored[i] = True
                color = new_color
                ncol = int(color.max()) + 1
        W = T + (float(steps.max()) * step if T > 0 else 0.0)
        for i in range(n):
            if np.linalg.norm(delta[i]) > 1e-9:
                pieces[i].append(leg_segment(t + steps[i] * step, T, pos[i], dst[i]))
        transitions.append({"t_start": t, "window_s": W, "leg_s": T, "rounds": rounds,
                            "max_delay_steps": int(steps.max()), "unresolved_pairs": unresolved,
                            "step_s": step})
        t += W + stages[k + 1].get("hold_s", 0.0)
        pos = dst
    for i in range(n):
        if not pieces[i]:
            pieces[i].append(hold_segment(0.0, t, pos[i]))
    fleet = Fleet(stitch(pieces, t), {"planner": f"delays(assign={assign})", "show": show["name"],
                                      "seed": show["seed"]})
    return fleet, {"transitions": transitions, "t_end": t,
                   "plan_runtime_s": time.perf_counter() - t_wall,
                   "unresolved_pairs": int(sum(x["unresolved_pairs"] for x in transitions))}
