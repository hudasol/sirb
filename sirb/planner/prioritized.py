"""Prioritized planner with detour lanes and start delays.

For each transition, drones are planned one at a time in priority order (longest straight path
first). A drone's candidates are routes (direct, or via a point offset vertically or sideways
from the midpoint) crossed with start delays; candidates are tried in order of finish time and
the first one clear of every already-planned drone is accepted. If a drone has no feasible
candidate it is promoted to the front and the window re-planned; after several failures the
window is widened.

Clearance test (the planner's own, *sampled*): positions are compared at a fixed step dt. Since
the true centre distance is Lipschitz with constant (v_i + v_j) and the nearest sample is within
dt/2, requiring sampled distance >= d_min + dt/2 * (vpeak_i + vpeak_j) guarantees the continuous
distance is >= d_min. Peak speeds are exact (1.875 L / T per leg). The independent verifier
(`sirb.verify`) is still the judge; this module must not import it.
"""

from __future__ import annotations

import math
import time

import numpy as np

from ..assign import METHODS, linear_conflicts, total_path_length
from ..spec import build_targets
from ..trajectory import Fleet
from .common import K_V, hold_segment, leg_segment, min_leg_time, stitch


class _Fail(Exception):
    def __init__(self, drone: int):
        self.drone = drone


def _assign(name, src, dst, seed):
    fn = METHODS[name]
    return fn(src, dst, seed) if name in ("random", "pinned15") else fn(src, dst)


def _sample_path(wps, durs, delay, times):
    out = np.empty((len(times), 3))
    out[:] = wps[-1]
    out[times < delay] = wps[0]
    t = delay
    for a, b, T in zip(wps[:-1], wps[1:], durs):
        if T <= 0:
            continue
        m = (times >= t) & (times < t + T)
        u = (times[m] - t) / T
        s = 10 * u**3 - 15 * u**4 + 6 * u**5
        out[m] = a + (b - a) * s[:, None]
        t += T
    return out


def _routes(src, dst, lim, lane_dz, lane_dh):
    mid = (src + dst) / 2
    d = dst - src
    hv = np.array([-d[1], d[0], 0.0])
    hv = hv / np.linalg.norm(hv) if np.linalg.norm(hv) > 0.5 else np.array([1.0, 0.0, 0.0])
    gx, gy = lim["geofence"]["x"], lim["geofence"]["y"]
    routes = [[src, dst]]
    cands = [mid + np.array([0, 0, s * h]) for h in lane_dz for s in (1, -1)]
    cands += [mid + hv * (s * h) for h in lane_dh for s in (1, -1)]
    for v in cands:
        if lim["z_min"] + 0.5 <= v[2] <= lim["z_max"] - 0.5 and gx[0] <= v[0] <= gx[1] \
                and gy[0] <= v[1] <= gy[1]:
            routes.append([src, v, dst])
    return routes


def _plan_window(src, dst, lim, W, dt, step_s, delay_max, lane_dz, lane_dh, order, t_sync,
                 deadline=math.inf):
    n = len(src)
    d_min = lim["min_separation_m"]
    times = np.arange(0.0, W + dt / 2, dt)
    S = len(times)
    samples = np.zeros((S, n, 3))
    bb_lo = np.zeros((n, 3))
    bb_hi = np.zeros((n, 3))
    vpk = np.zeros(n)
    done: list[int] = []
    chosen: dict[int, tuple] = {}
    delays = np.arange(0.0, delay_max + 1e-9, step_s)
    for i in order:
        if time.perf_counter() > deadline:
            raise RuntimeError("planning time budget exceeded")
        routes = _routes(src[i], dst[i], lim, lane_dz, lane_dh)
        cands = []
        # Preferred candidate: synchronous straight leg over the common duration t_sync. With a
        # sum-of-squares assignment this is the cleanest motion and usually already clear.
        if np.linalg.norm(dst[i] - src[i]) > 1e-9:
            r = routes[0]
            vp = K_V * float(np.linalg.norm(dst[i] - src[i])) / t_sync
            cands.append((-1.0, 0.0, r, [t_sync], vp, np.minimum.reduce(r), np.maximum.reduce(r)))
        for r in routes:
            durs = [min_leg_time(b - a, lim) for a, b in zip(r[:-1], r[1:])]
            dur = sum(durs)
            vp = max([K_V * np.linalg.norm(b - a) / T for (a, b), T in
                      zip(zip(r[:-1], r[1:]), durs) if T > 0] or [0.0])
            lo = np.minimum.reduce(r)
            hi = np.maximum.reduce(r)
            for dl in delays:
                if dl + dur <= W + 1e-9:
                    cands.append((dl + dur, dl, r, durs, vp, lo, hi))
        cands.sort(key=lambda c: (c[0], c[1]))
        ok = None
        for _fin, dl, r, durs, vp, lo, hi in cands:
            if done:
                idx = np.array(done)
                req = d_min + 0.5 * dt * (vp + vpk[idx])
                gap = np.maximum(np.maximum(bb_lo[idx] - hi, lo - bb_hi[idx]), 0.0)
                near = np.linalg.norm(gap, axis=1) < req
                if near.any():
                    sel = idx[near]
                    cs = _sample_path(r, durs, dl, times)
                    dd = np.linalg.norm(samples[:, sel, :] - cs[:, None, :], axis=2)
                    reqn = req[near]
                    bad = False
                    for col, j in enumerate(sel):
                        viol = np.where(dd[:, col] < reqn[col])[0]
                        if len(viol) == 0:
                            continue
                        if dd[:, col].min() < d_min:
                            bad = True
                            break
                        # Uncertain: re-check the neighbourhood of the near samples finely.
                        lo_t = max(times[viol[0]] - dt, 0.0)
                        hi_t = min(times[viol[-1]] + dt, times[-1])
                        dtf = dt / 16
                        tf = np.arange(lo_t, hi_t + dtf / 2, dtf)
                        dlj, rj, dursj = chosen[int(j)]
                        pj = _sample_path(rj, dursj, dlj, tf)
                        pc = _sample_path(r, durs, dl, tf)
                        dmin_f = np.linalg.norm(pj - pc, axis=1).min()
                        if dmin_f < d_min + 0.5 * dtf * (vp + vpk[j]):
                            bad = True
                            break
                    if bad:
                        continue
                    ok = (dl, r, durs, vp, lo, hi, cs)
                    break
            ok = (dl, r, durs, vp, lo, hi, None)
            break
        if ok is None:
            raise _Fail(i)
        dl, r, durs, vp, lo, hi, cs = ok
        if cs is None:
            cs = _sample_path(r, durs, dl, times)
        samples[:, i, :] = cs
        bb_lo[i], bb_hi[i], vpk[i] = lo, hi, vp
        done.append(i)
        chosen[i] = (dl, r, durs)
    return chosen


def plan_prioritized(show: dict, *, assign: str = "hungarian_sq", dt: float = 0.05,
                     step_s: float = 0.5, delay_frac: float = 0.6,
                     lane_dz=(6.0, 12.0, 18.0), lane_dh=(6.0, 12.0),
                     promote_retries: int = 3, widen: float = 1.25,
                     max_widen: int = 5, time_budget_s: float = 60.0) -> tuple[Fleet, dict]:
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
        direct = np.array([min_leg_time(d, lim) for d in dst - pos])
        T0 = float(direct.max())
        order = list(np.argsort(-direct, kind="stable"))
        promotions = widenings = 0
        W = 1.5 * T0 + 2.0
        while True:
            dmax = delay_frac * T0 * (1.25 ** widenings)
            try:
                chosen = _plan_window(pos, dst, lim, W, dt, step_s, dmax, lane_dz, lane_dh, order,
                                      T0, t_wall + time_budget_s)
                break
            except _Fail as f:
                if promotions < promote_retries * (widenings + 1):
                    order.remove(f.drone)
                    order.insert(0, f.drone)
                    promotions += 1
                    continue
                widenings += 1
                if widenings > max_widen:
                    raise RuntimeError(f"transition {k}: no feasible plan") from None
                W *= widen
        W_used = max(chosen[i][0] + sum(chosen[i][2]) for i in range(n))
        W_plan, W = W, max(W_used, 1e-6)
        detours = 0
        for i in range(n):
            dl, r, durs = chosen[i]
            tt = t + dl
            added = False
            for a, b, T in zip(r[:-1], r[1:], durs):
                if T > 0:
                    pieces[i].append(leg_segment(tt, T, a, b))
                    tt += T
                    added = True
            if not added:
                pieces[i].append(hold_segment(t, W, pos[i]))
            detours += len(r) > 2
        transitions.append({
            "t_start": t, "window_s": W, "planned_window_s": W_plan, "direct_min_s": T0, "promotions": promotions,
            "widenings": widenings, "detoured_drones": detours,
            "total_path_length_m": total_path_length(pos, dst_all, perm),
            "synchronous_conflicting_pairs": linear_conflicts(
                pos, dst_all, perm, lim["min_separation_m"]),
        })
        t += W + stages[k + 1].get("hold_s", 0.0)
        pos = dst
    trajs = stitch(pieces, t)
    fleet = Fleet(trajs, {"planner": f"prioritized(assign={assign})", "show": show["name"],
                          "seed": show["seed"]})
    return fleet, {"transitions": transitions, "t_end": t,
                   "plan_runtime_s": time.perf_counter() - t_wall}
