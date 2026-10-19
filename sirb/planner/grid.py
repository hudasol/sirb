"""Grid planner for ground robots: prioritised space time A* with waiting (exploratory).

Robots live on a square lattice of cells at a fixed pitch (default 2.0 m). Time is discrete: every
robot either waits or moves to a 4 connected neighbour in each step, and every step has the same
duration T, chosen from the vehicle limits so that one rest to rest minimum jerk move over one
pitch respects the speed, acceleration and jerk limits (``min_leg_time``). A step is a straight
quintic leg (or a hold), so all robots move in lockstep and every cell centre is a rest point.

Planning is prioritised: robots are planned one at a time in a priority order against a
reservation table of the robots already planned. A reservation covers

* vertices (cell, step): two robots may not occupy the same cell at the same step;
* edge swaps: a robot may not traverse an edge in the direction opposite to a reserved traversal
  of the same edge in the same step;
* parked robots: a robot that has reached its goal reserves that cell from its arrival step on
  for ever, and no robot may use a goal cell at or before the last step another robot uses it
  (so a robot never parks where somebody still has to pass).

Following (entering a cell another robot leaves in the same step) is allowed. Prioritised
planning is incomplete: a robot planned late can be boxed in by robots already parked, and an
early robot never steps aside for a later one. The planner therefore tries a bounded number of
priority orders (longest distance first; after a failure the failed robot is bumped to the front
of the order, and every fourth retry is a seeded random order) and raises ``GridPlanError`` if
none succeeds within the restart and time budget.

Continuous separation. Because every robot uses the same profile s(u) (monotone, 0 to 1), the
relative position of two robots during a step is r0 + s(u) * d with d the difference of their
step vectors, i.e. it traces the straight segment from r0 to r0 + d. If the discrete plan has no
vertex conflict and no edge swap, r0 and r0 + d are distinct non zero lattice vectors and the
segment does not pass through the origin; ``worst_case_separation`` enumerates every such pair of
step choices and returns the smallest distance to the origin. It is pitch / sqrt(2) (two
perpendicular moves into or past diagonal neighbour cells), so no extra conflict rule is needed
when pitch / sqrt(2) exceeds the required separation, which the planner checks. Head on swaps
(distance 0) are excluded by the edge rule. The exact verifier remains the judge of every plan.

Simulation and planning only.
"""

from __future__ import annotations

import heapq
import itertools
import math
import time
from collections import deque

import numpy as np

from ..spec import build_targets
from ..trajectory import Fleet
from .baseline import _assign
from .common import hold_segment, leg_segment, min_leg_time, stitch

_MOVES = ((0, 0), (1, 0), (-1, 0), (0, 1), (0, -1))  # wait, E, W, N, S


class GridPlanError(RuntimeError):
    """No plan found. ``cause`` is one of: enclosed_by_parked_robots, blocked_by_reservations,
    time_budget."""

    def __init__(self, message: str, cause: str):
        super().__init__(message)
        self.cause = cause


def worst_case_separation(pitch: float) -> float:
    """Smallest continuous distance between two robots over one step, over every pair of step
    choices (wait or one of four moves) whose discrete plan has no vertex conflict and no edge
    swap, for all relative cell offsets within three cells. In metres."""
    best = math.inf
    for rx, ry in itertools.product(range(-3, 4), repeat=2):
        if (rx, ry) == (0, 0):
            continue
        for (ax, ay), (bx, by) in itertools.product(_MOVES, repeat=2):
            dx, dy = ax - bx, ay - by
            r1 = (rx + dx, ry + dy)
            if r1 == (0, 0):
                continue  # vertex conflict
            if (rx, ry) == (-ax, -ay) and (ax, ay) == (-bx, -by) and (ax, ay) != (0, 0):
                continue  # edge swap
            best = min(best, _seg_dist((rx, ry), r1))
    return best * pitch


def _seg_dist(p, q) -> float:
    px, py = p
    dx, dy = q[0] - px, q[1] - py
    L2 = dx * dx + dy * dy
    if L2 == 0:
        return math.hypot(px, py)
    u = max(0.0, min(1.0, -(px * dx + py * dy) / L2))
    return math.hypot(px + u * dx, py + u * dy)


class _Grid:
    def __init__(self, w: int, h: int, blocked: set[tuple[int, int]]):
        self.w, self.h, self.nc = w, h, w * h
        self.cx = [c // h for c in range(self.nc)]
        self.cy = [c % h for c in range(self.nc)]
        self.free = [(self.cx[c], self.cy[c]) not in blocked for c in range(self.nc)]
        self.nbrs: list[list[int]] = []
        for c in range(self.nc):
            out = []
            if self.free[c]:
                for dx, dy in _MOVES[1:]:
                    x, y = self.cx[c] + dx, self.cy[c] + dy
                    if 0 <= x < w and 0 <= y < h and self.free[x * h + y]:
                        out.append(x * h + y)
            self.nbrs.append(out)

    def cell(self, x: int, y: int) -> int:
        return x * self.h + y


def _astar(grid, start, goal, vert, edges, park, last, max_steps, deadline):
    """Shortest arrival step path (list of cells, index = step) or None, or the string
    'time' if the deadline passed."""
    nc = grid.nc
    gx, gy = grid.cx[goal], grid.cy[goal]
    t_goal = last.get(goal, -1) + 1
    inf = 1 << 60
    h0 = abs(grid.cx[start] - gx) + abs(grid.cy[start] - gy)
    heap = [(h0, h0, start, 0)]
    parent = {start: -1}  # state key t * nc + c -> parent key
    count = 0
    while heap:
        _, _, c, t = heapq.heappop(heap)
        if c == goal and t >= t_goal:
            path, k = [], t * nc + c
            while k != -1:
                path.append(k % nc)
                k = parent[k]
            return path[::-1]
        if t >= max_steps:
            continue
        count += 1
        if count % 2000 == 0 and time.perf_counter() > deadline:
            return "time"
        t2, key = t + 1, t * nc + c
        for c2 in (c, *grid.nbrs[c]):
            k2 = t2 * nc + c2
            if k2 in parent or k2 in vert or park.get(c2, inf) <= t2:
                continue
            if c2 != c and ((t * nc + c2) * nc + c) in edges:
                continue
            parent[k2] = key
            hh = abs(grid.cx[c2] - gx) + abs(grid.cy[c2] - gy)
            heapq.heappush(heap, (t2 + hh, hh, c2, t2))
    return None


def _static_reachable(grid, start, goal, park) -> bool:
    """Goal reachable from start ignoring time, treating parked goal cells as walls."""
    if goal in park or not grid.free[goal]:
        return False
    seen, dq = {start}, deque([start])
    while dq:
        c = dq.popleft()
        if c == goal:
            return True
        for c2 in grid.nbrs[c]:
            if c2 not in seen and c2 not in park:
                seen.add(c2)
                dq.append(c2)
    return False


def _plan_once(grid, order, starts, goals, max_steps, deadline):
    nc = grid.nc
    vert: set[int] = set()
    edges: set[int] = set()
    park: dict[int, int] = {}
    last: dict[int, int] = {}
    paths: dict[int, list[int]] = {}
    for j in order:
        p = _astar(grid, starts[j], goals[j], vert, edges, park, last, max_steps, deadline)
        if p == "time":
            return paths, j, "time_budget"
        if p is None:
            cause = ("blocked_by_reservations" if _static_reachable(grid, starts[j], goals[j], park)
                     else "enclosed_by_parked_robots")
            return paths, j, cause
        while len(p) > 1 and p[-1] == p[-2]:
            p.pop()  # trailing waits at the goal are the parked state
        L = len(p) - 1
        for t in range(L):
            vert.add(t * nc + p[t])
            last[p[t]] = max(last.get(p[t], -1), t)
            if p[t] != p[t + 1]:
                edges.add((t * nc + p[t]) * nc + p[t + 1])
        park[p[-1]] = L
        paths[j] = p
    return paths, None, ""


def discrete_conflicts(paths: list[list[int]], nc: int) -> int:
    """Independent check of a set of step paths (cells indexed as in the grid, padded with the
    final cell): number of vertex conflicts plus edge swaps."""
    L = max(len(p) for p in paths) - 1
    pad = [p + [p[-1]] * (L + 1 - len(p)) for p in paths]
    bad = 0
    for t in range(L + 1):
        cells = [p[t] for p in pad]
        bad += len(cells) - len(set(cells))
        if t < L:
            moves = {(p[t], p[t + 1]) for p in pad if p[t] != p[t + 1]}
            bad += sum(1 for a, b in moves if (b, a) in moves and a < b)
    return bad


def plan_cells(starts_xy, goals_xy, *, margin: int = 2, blocked=(), restarts: int = 30,
               max_steps: int | None = None, time_budget_s: float = 60.0, seed: int = 0):
    """Plan on integer lattice coordinates. ``starts_xy`` and ``goals_xy`` are lists of (ix, iy);
    ``blocked`` is a set of (ix, iy) cells nobody may enter. Returns (paths, cells_to_xy, info):
    ``paths[i]`` is the list of lattice (ix, iy) positions per step, all padded to equal
    length."""
    t_wall = time.perf_counter()
    deadline = t_wall + time_budget_s
    n = len(starts_xy)
    xs = [p[0] for p in (*starts_xy, *goals_xy)]
    ys = [p[1] for p in (*starts_xy, *goals_xy)]
    x0, y0 = min(xs) - margin, min(ys) - margin
    w, h = max(xs) - x0 + margin + 1, max(ys) - y0 + margin + 1
    blk = {(x - x0, y - y0) for x, y in blocked}
    grid = _Grid(w, h, blk)
    starts = [grid.cell(x - x0, y - y0) for x, y in starts_xy]
    goals = [grid.cell(x - x0, y - y0) for x, y in goals_xy]
    if len(set(starts)) < n or len(set(goals)) < n:
        raise ValueError("starts and goals must each be distinct cells")
    for c in (*starts, *goals):
        if not grid.free[c]:
            raise ValueError("a start or goal lies on a blocked cell")
    if max_steps is None:
        max_steps = 6 * (w + h) + 2 * n
    dist = [abs(grid.cx[s] - grid.cx[g]) + abs(grid.cy[s] - grid.cy[g])
            for s, g in zip(starts, goals)]
    order = sorted(range(n), key=lambda i: (-dist[i], i))
    rng = np.random.default_rng(seed)
    tried, fail = 0, None
    for attempt in range(max(1, restarts)):
        paths, failed, cause = _plan_once(grid, order, starts, goals, max_steps, deadline)
        tried = attempt + 1
        if failed is None:
            break
        fail = (failed, cause, len(paths))
        if cause == "time_budget":
            break
        if attempt % 4 != 3:  # bump the failed robot to the front (accumulates over attempts)
            order = [failed] + [i for i in order if i != failed]
        else:  # seeded random order
            order = [int(i) for i in rng.permutation(n)]
    else:
        failed = fail[0]
    if failed is not None:
        j, cause, planned = fail
        raise GridPlanError(
            f"no grid plan: robot {j} could not be routed after {planned} of {n} robots "
            f"were planned (cause {cause}, {tried} priority orders tried, horizon "
            f"{max_steps} steps)", cause)
    L = max(len(p) for p in paths.values()) - 1
    out = []
    for i in range(n):
        p = paths[i] + [paths[i][-1]] * (L + 1 - len(paths[i]))
        out.append([(grid.cx[c] + x0, grid.cy[c] + y0) for c in p])
    flat = [[grid.cell(x - x0, y - y0) for x, y in p] for p in out]
    info = {"steps": L, "orders_tried": tried, "grid_cells": [w, h],
            "sum_of_costs": sum(len(paths[i]) - 1 for i in range(n)),
            "max_steps": max_steps, "discrete_conflicts": discrete_conflicts(flat, grid.nc),
            "plan_runtime_s": time.perf_counter() - t_wall}
    return out, info


def plan_grid(show: dict, *, assign: str = "hungarian_sq", pitch: float = 2.0, margin: int = 2,
              blocked_xy=(), restarts: int = 30, max_steps: int | None = None,
              time_budget_s: float = 60.0, seed: int | None = None) -> tuple[Fleet, dict]:
    """Grid plan for every transition of ``show``; positions must lie on one lattice of the
    given pitch (anchored at the first robot's start) and share one height per robot."""
    t_wall = time.perf_counter()
    lim = show["limits"]
    sep = lim["min_separation_m"]
    worst = worst_case_separation(pitch)
    if worst < sep * 1.001 or pitch < sep * 1.001:
        raise ValueError(f"pitch {pitch} m is too small: worst case continuous separation "
                         f"{worst:.3f} m against a required {sep} m")
    step_s = min_leg_time(np.array([pitch, 0.0, 0.0]), lim)
    targets = build_targets(show)
    n = show["n_drones"]
    pos = targets[0].copy()
    ox, oy = pos[0, 0], pos[0, 1]
    base_seed = show["seed"] if seed is None else seed

    def lattice(p: np.ndarray) -> list[tuple[int, int]]:
        ix = np.rint((p[:, 0] - ox) / pitch)
        iy = np.rint((p[:, 1] - oy) / pitch)
        if (np.abs(ix * pitch + ox - p[:, 0]).max() > 1e-6
                or np.abs(iy * pitch + oy - p[:, 1]).max() > 1e-6):
            raise ValueError(f"positions are not on a {pitch} m lattice")
        return [(int(a), int(b)) for a, b in zip(ix, iy)]

    blocked = lattice(np.array([[x, y, 0.0] for x, y in blocked_xy])) if len(blocked_xy) else []
    pieces: list[list] = [[] for _ in range(n)]
    stages = show["stages"]
    t = stages[0].get("hold_s", 0.0)
    transitions = []
    for k in range(len(stages) - 1):
        dst_all = targets[k + 1]
        perm = _assign(assign, pos, dst_all, show["seed"] + k)
        dst = dst_all[perm]
        if np.abs(dst[:, 2] - pos[:, 2]).max() > 1e-9:
            raise ValueError("the grid planner is planar: start and goal heights must match")
        cell_paths, ci = plan_cells(lattice(pos), lattice(dst), margin=margin, blocked=blocked,
                                    restarts=restarts, max_steps=max_steps,
                                    time_budget_s=time_budget_s, seed=base_seed + k)
        L = ci["steps"]
        for i in range(n):
            z = pos[i, 2]
            pts = [np.array([ox + a * pitch, oy + b * pitch, z]) for a, b in cell_paths[i]]
            pts[0], pts[-1] = pos[i].copy(), dst[i].copy()
            if L == 0:
                pieces[i].append(hold_segment(t, step_s, pos[i]))
                continue
            s = 0
            while s < L:
                if cell_paths[i][s] == cell_paths[i][s + 1]:
                    e = s
                    while e < L and cell_paths[i][e] == cell_paths[i][e + 1]:
                        e += 1
                    pieces[i].append(hold_segment(t + s * step_s, (e - s) * step_s, pts[s]))
                    s = e
                else:
                    pieces[i].append(leg_segment(t + s * step_s, step_s, pts[s], pts[s + 1]))
                    s += 1
        W = max(L, 1) * step_s
        transitions.append({"t_start": t, "window_s": W, "steps": L, "step_s": step_s,
                            "orders_tried": ci["orders_tried"], "sum_of_costs": ci["sum_of_costs"],
                            "grid_cells": ci["grid_cells"],
                            "discrete_conflicts": ci["discrete_conflicts"]})
        t += W + stages[k + 1].get("hold_s", 0.0)
        pos = dst
    trajs = stitch(pieces, t)
    fleet = Fleet(trajs, {"planner": f"grid(assign={assign},pitch={pitch},margin={margin})",
                          "show": show["name"], "seed": show["seed"]})
    return fleet, {"transitions": transitions, "t_end": t, "step_s": step_s,
                   "worst_case_separation_m": worst,
                   "plan_runtime_s": time.perf_counter() - t_wall}
