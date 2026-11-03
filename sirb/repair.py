"""Repair of a planned show: the smallest set of explicit moves that brings it under a target.

Moves (all keep every drone's waypoints; none changes a limit or the declared minimum
separation):

* ``stage_spacing``  enlarge the formation of one stage about its centre by a factor, then replan;
* ``dilate_leg``     stretch the time of one drone's transition leg by a factor (later segments of
                     that drone shift by the extra time);
* ``start_delay``    hold one drone at its start for a number of seconds before it begins.

The search is greedy on the number of simulated violation events (``sirb.doctor.trial_pairs``)
for a fixed search seed; a move is accepted only if the exact verifier still passes every declared
limit. This module orchestrates the baseline planner and the verifiers; the verifier modules do
not import it. Results are SIMULATED under assumed disturbance models and are not flight evidence.
"""

from __future__ import annotations

import copy
import time

import numpy as np

from .disturbance import StressConfig
from .doctor import trial_pairs
from .planner.baseline import plan_baseline
from .spec import validate_show
from .stress import sample_reference, wilson
from .trajectory import Fleet, Segment, Trajectory
from .verify import verify
from .verify2 import verify2

FORMAT = "sirb-repair-1"
LABEL = "simulation only; not flight evidence"
SPACING_FACTORS = (1.05, 1.10, 1.20)
DELAYS_S = (1.0, 2.0)
DILATIONS = (1.15, 1.30)


# ----------------------------------------------------------------------------- moves

def scale_stage(show: dict, k: int, factor: float) -> dict:
    out = copy.deepcopy(show)
    sh = dict(out["stages"][k]["shape"])
    if sh.get("type") == "points":
        pts = np.array(sh["points"], float)
        c = pts.mean(axis=0)
        sh["points"] = (c + factor * (pts - c)).tolist()
    else:
        for key in ("spacing_m", "size"):
            if sh.get(key) is not None:
                sh[key] = sh[key] * factor
    out["stages"][k]["shape"] = sh
    return out


def _dilate(tr: Trajectory, t_ref: float, f: float) -> Trajectory:
    segs, shift = [], 0.0
    for s in tr.segments:
        if shift == 0.0 and s.t0 <= t_ref < s.t1 and len(s.coeffs) > 1:
            T = s.t1 - s.t0
            c = s.coeffs / (f ** np.arange(len(s.coeffs)))[:, None]
            segs.append(Segment(s.t0, s.t0 + f * T, c))
            shift = (f - 1.0) * T
        else:
            segs.append(Segment(s.t0 + shift, s.t1 + shift, s.coeffs))
    return Trajectory(segs)


def _delay(tr: Trajectory, d: float) -> Trajectory:
    first = tr.segments[0]
    hold = Segment(first.t0, first.t0 + d, np.array([first.coeffs[0]]))
    return Trajectory([hold] + [Segment(s.t0 + d, s.t1 + d, s.coeffs) for s in tr.segments])


def apply_fleet_moves(fleet: Fleet, moves: list[dict]) -> Fleet:
    """Dilations first (latest leg first, so earlier times stay valid), then delays."""
    trs = list(fleet.trajectories)
    for m in sorted((m for m in moves if m["type"] == "dilate_leg"), key=lambda m: -m["t_ref"]):
        trs[m["drone"]] = _dilate(trs[m["drone"]], m["t_ref"], m["factor"])
    for m in moves:
        if m["type"] == "start_delay":
            trs[m["drone"]] = _delay(trs[m["drone"]], m["delay_s"])
    meta = dict(fleet.meta)
    meta["repair_moves"] = len(moves)
    return Fleet(trs, meta)


def build(show: dict, stage_factors: dict, fleet_moves: list[dict]):
    s = show
    for k, f in sorted(stage_factors.items()):
        s = scale_stage(s, k, f)
    fleet, info = plan_baseline(s)
    return s, apply_fleet_moves(fleet, fleet_moves), info


def stage_windows(show: dict, info: dict) -> list[tuple]:
    """(kind, index, t0, t1): ``hold`` of stage k, or ``transition`` k between k and k+1."""
    out = []
    tr = info["transitions"]
    out.append(("hold", 0, 0.0, float(tr[0]["t_start"]) if tr else float(info["t_end"])))
    for k, x in enumerate(tr):
        a, b = float(x["t_start"]), float(x["t_start"]) + float(x["window_s"])
        out.append(("transition", k, a, b))
        nxt = float(tr[k + 1]["t_start"]) if k + 1 < len(tr) else float(info["t_end"])
        out.append(("hold", k + 1, b, nxt))
    return out


def locate(windows: list[tuple], t: float) -> tuple:
    for w in windows:
        if w[2] <= t <= w[3]:
            return w
    return windows[-1]


# ----------------------------------------------------------------------------- costs

def path_length(fleet: Fleet, dt: float = 0.05) -> float:
    times = np.arange(0.0, fleet.t_end + dt / 2, dt)
    X = fleet.sample(times)
    return float(np.linalg.norm(np.diff(X, axis=0), axis=2).sum())


def cost(base: Fleet, fleet: Fleet) -> dict:
    return {"extra_duration_s": fleet.t_end - base.t_end,
            "extra_path_length_m": path_length(fleet) - path_length(base)}


# ----------------------------------------------------------------------------- evaluation

def quick(fleet: Fleet, cfg: StressConfig, d_min: float, trials: int, seed: int, tag: int = 0):
    ref = sample_reference(fleet, cfg.dt)
    rows, kv = [], 0
    for t in range(trials):
        r = trial_pairs(ref, cfg, d_min, [seed, tag, t])["rows"]
        kv += bool(r)
        rows += r
    return {"k": kv, "trials": trials, "events": len(rows), "rows": rows}


def final_check(show: dict, fleet: Fleet, cfg: StressConfig, trials: int, seed: int) -> dict:
    d_min = show["limits"]["min_separation_m"]
    q = quick(fleet, cfg, d_min, trials, seed, tag=2)
    lo, hi = wilson(q["k"], trials)
    rep = verify(fleet, show["limits"])
    v2 = verify2(fleet, d_min)
    return {"exact_verdict": "PASS" if rep.ok else "FAIL", "exact_min_separation_m": rep.min_separation,
            "interval_verdict": v2.verdict, "interval_lower_bound_m": v2.lower_bound,
            "p_violation": q["k"] / trials, "p_violation_ci95": [lo, hi], "trials": trials,
            "seed": seed, "meets_target": None}


def repair(show: dict, cfg: StressConfig, *, target: float = 0.05, max_moves: int = 15,
           search_trials: int = 100, search_seed: int = 7001, stop_seed: int = 7002,
           final_seed: int = 7003, final_trials: int = 200,
           time_budget_s: float | None = None) -> dict:
    t0 = time.perf_counter()
    d_min = show["limits"]["min_separation_m"]
    base_fleet, base_info = plan_baseline(show)
    factors: dict = {}
    fmoves: list = []
    log: list = []
    cur_show, cur_fleet, cur_info = show, base_fleet, base_info
    cur = quick(cur_fleet, cfg, d_min, search_trials, search_seed)
    status = "max_moves"
    for it in range(max_moves + 1):
        if cur["k"] <= 3:
            chk = quick(cur_fleet, cfg, d_min, final_trials, stop_seed, tag=1)
            if wilson(chk["k"], final_trials)[1] <= target:
                status = "target_reached"
                break
        if it == max_moves:
            break
        if time_budget_s is not None and time.perf_counter() - t0 > time_budget_s:
            status = "time_budget"
            break
        windows = stage_windows(cur_show, cur_info)
        cnt: dict = {}
        for (i, j, k, _d, _g, _e) in cur["rows"]:
            cnt.setdefault((i, j), []).append(k)
        order = sorted(cnt, key=lambda p: -len(cnt[p]))[:2]
        cands: list = []
        for (i, j) in order:
            w = locate(windows, float(np.median(cnt[(i, j)])) * cfg.dt)
            if w[0] == "hold":
                cands += [("stage_spacing", w[1], f) for f in SPACING_FACTORS]
            else:
                tref = (w[2] + w[3]) / 2
                for st in (w[1], w[1] + 1):
                    cands += [("stage_spacing", st, f) for f in SPACING_FACTORS]
                for dr in (i, j):
                    cands += [("start_delay", dr, d) for d in DELAYS_S]
                    cands += [("dilate_leg", (dr, tref), f) for f in DILATIONS]
        best = None
        for kind, a, b in cands:
            nf, nm = dict(factors), list(fmoves)
            if kind == "stage_spacing":
                nf[a] = nf.get(a, 1.0) * b
                mv = {"type": kind, "stage": a, "factor": b}
            elif kind == "start_delay":
                nm.append({"type": kind, "drone": a, "delay_s": b})
                mv = nm[-1]
            else:
                nm.append({"type": kind, "drone": a[0], "t_ref": a[1], "factor": b})
                mv = nm[-1]
            s2, f2, i2 = build(show, nf, nm)
            if validate_show(s2, check_targets=False) or not verify(f2, s2["limits"]).ok:
                continue
            q = quick(f2, cfg, d_min, search_trials, search_seed)
            c = cost(base_fleet, f2)
            key = (q["events"], c["extra_path_length_m"] + c["extra_duration_s"])
            if best is None or key < best[0]:
                best = (key, nf, nm, mv, s2, f2, i2, q, c)
        if best is None or best[7]["events"] >= cur["events"]:
            status = "no_improving_move"
            break
        _, factors, fmoves, mv, cur_show, cur_fleet, cur_info, cur, c = best
        log.append({"move": mv, "search_events": cur["events"], "search_violating_trials": cur["k"],
                    "cumulative_cost": c})
    final = final_check(cur_show, cur_fleet, cfg, final_trials, final_seed)
    final["meets_target"] = (final["exact_verdict"] == "PASS" and final["interval_verdict"] != "FAIL"
                             and final["p_violation_ci95"][1] <= target)
    if final["meets_target"]:
        status = "target_reached"
    before = final_check(show, base_fleet, cfg, final_trials, final_seed)
    return {"format": FORMAT, "label": LABEL, "target_upper_bound": target, "status": status,
            "moves": log, "n_moves": len(log), "before": before, "after": final,
            "cost": cost(base_fleet, cur_fleet), "stage_factors": {str(k): v for k, v in factors.items()},
            "fleet_moves": fmoves, "search": {"trials": search_trials, "seed": search_seed,
                                              "stop_seed": stop_seed, "final_seed": final_seed},
            "wall_s": time.perf_counter() - t0}, cur_show, cur_fleet
