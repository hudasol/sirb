"""Diagnosis of a planned show: which drone pairs, time windows and limits drive the risk.

Combines the exact verifier (nominal plan, exact per pair minima, declared limits) with the
Monte Carlo stress model of ``sirb.stress``, but attributes every simulated violation to the pair
that produced it, not only the worst pair of each trial. Reads a plan; it never changes one and
contains no planning code. Everything under ``stress`` is SIMULATED under assumed disturbance
models; it is not flight evidence.
"""

from __future__ import annotations

import dataclasses
import math
import time

import numpy as np

from .disturbance import StressConfig
from .envinfo import environment
from .stress import (
    _fork_available,
    candidate_pairs,
    dev_bound,
    exact_pair_min,
    realize,
    sample_reference,
    wilson,
)
from .verify import pair_minima, verify

FORMAT = "sirb-diagnosis-1"
LABEL = "simulation only; not flight evidence"
GNSS_NOTICE = ("The stress part of this diagnosis depends on the relative GNSS error between "
               "nearby receivers. No free source gives a measured value for that parameter; the "
               "value used is stated under stress.profile and every verdict that depends on it "
               "should be read as conditional on it (docs/GNSS_SOURCED.md).")

_WORK: dict = {}


def trial_pairs(ref: np.ndarray, cfg: StressConfig, d_min: float, seed) -> dict:
    """One disturbed trial, returning every pair below ``d_min`` (not only the worst pair)."""
    K, n, _ = ref.shape
    X, R, eps, events, ginfo, bal_idx = realize(ref, cfg, seed)
    pairs, k_lo, k_hi = candidate_pairs(R, d_min + dev_bound(cfg), dt=cfg.dt)
    if bal_idx:
        extra = np.array([(min(f, j), max(f, j)) for f in bal_idx for j in range(n) if j != f],
                         dtype=int)
        pairs = np.vstack([pairs, extra])
        k_lo = np.concatenate([k_lo, np.zeros(len(extra), dtype=int)])
        k_hi = np.concatenate([k_hi, np.full(len(extra), K - 1, dtype=int)])
    pmin, karg = exact_pair_min(X, pairs, k_lo, k_hi)
    bad = np.where(pmin < d_min)[0] if len(pmin) else np.array([], dtype=int)
    deg = ginfo["degraded"]
    ev = {e["drone"] for e in events} | set(ginfo["loss_windows"])
    rows = [(int(pairs[q, 0]), int(pairs[q, 1]), int(karg[q]), float(pmin[q]),
             int(deg[pairs[q, 0]]) + int(deg[pairs[q, 1]]),
             int(pairs[q, 0] in ev or pairs[q, 1] in ev)) for q in bad]
    return {"rows": rows, "worst": float(pmin.min()) if len(pmin) else math.inf}


def _worker(args):
    cfg, d_min, seed, tag, t = args
    return trial_pairs(_WORK["ref"], cfg, d_min, [seed, tag, t])


def _exact_block(show: dict, fleet, top_k: int, radius: float) -> tuple[dict, dict]:
    lim = show["limits"]
    rep = verify(fleet, lim)
    mins, _ = pair_minima(fleet, radius)
    ranked = sorted(mins.items(), key=lambda kv: kv[1][0])
    d_min = lim["min_separation_m"]
    near = [{"pair": [i, j], "min_separation_m": d, "time_s": t, "margin_m": d - d_min}
            for (i, j), (d, t) in ranked[:top_k]]
    util = {
        "speed_h": [rep.max_speed_h, lim.get("vmax_h")],
        "speed_up": [rep.max_speed_up, lim.get("vmax_up")],
        "speed_down": [rep.max_speed_down, lim.get("vmax_down")],
        "accel": [rep.max_accel, lim.get("amax")],
        "jerk": [rep.max_jerk, lim.get("jmax")],
    }
    limits = {k: {"value": v, "limit": lm,
                  "utilisation": (v / lm if lm else None)} for k, (v, lm) in util.items()}
    block = {
        "verdict": "PASS" if rep.ok else "FAIL",
        "min_separation_m": rep.min_separation,
        "min_separation_is_lower_bound": rep.min_separation_is_lower_bound,
        "threshold_m": d_min,
        "nominal_margin_m": rep.min_separation - d_min,
        "violation_counts": rep.violation_counts,
        "pairs_within_search_radius": len(ranked),
        "nearest_pairs": near,
        "limits": limits,
        "geofence_margin_m": rep.geofence_margin,
        "crowd_margin": rep.crowd_margin,
    }
    return block, {p: (rank, d) for rank, (p, (d, _)) in enumerate(ranked, 1)}


def diagnose(show: dict, fleet, cfg: StressConfig, trials: int = 200, seed: int = 20261016,
             top_k: int = 10, window_s: float = 2.0, target: float = 0.05,
             workers: int = 1) -> dict:
    """Structured diagnosis of ``fleet`` against the limits of ``show`` (see module docstring)."""
    t0 = time.perf_counter()
    d_min = show["limits"]["min_separation_m"]
    radius = d_min + dev_bound(cfg)
    exact, rank_of = _exact_block(show, fleet, top_k, radius)
    ref = sample_reference(fleet, cfg.dt)
    jobs = [(cfg, d_min, seed, 0, t) for t in range(trials)]
    _WORK["ref"] = ref
    try:
        if workers > 1 and _fork_available():
            import multiprocessing as mp
            with mp.get_context("fork").Pool(workers) as pool:
                res = pool.map(_worker, jobs, chunksize=max(1, trials // (workers * 4)))
        else:
            res = [_worker(j) for j in jobs]
    finally:
        _WORK.pop("ref", None)

    k_viol = sum(1 for r in res if r["rows"])
    per_pair: dict = {}
    win: dict = {}
    by_deg = {0: 0, 1: 0, 2: 0}
    ev_events = 0
    total = 0
    for r in res:
        for (i, j, k, d, ndeg, evd) in r["rows"]:
            e = per_pair.setdefault((i, j), {"trials": 0, "min": math.inf})
            e["trials"] += 1
            e["min"] = min(e["min"], d)
            b = int(k * cfg.dt // window_s)
            win[b] = win.get(b, 0) + 1
            by_deg[ndeg] += 1
            ev_events += evd
            total += 1
    ranking = []
    for (i, j), e in sorted(per_pair.items(), key=lambda kv: -kv[1]["trials"]):
        rk = rank_of.get((i, j))
        ranking.append({"pair": [i, j], "violating_trials": e["trials"],
                        "p_violation_pair": e["trials"] / trials,
                        "share_of_events": e["trials"] / total,
                        "worst_simulated_separation_m": e["min"],
                        "exact_rank": rk[0] if rk else None,
                        "exact_min_separation_m": rk[1] if rk else None})
    windows = [{"t0_s": b * window_s, "t1_s": (b + 1) * window_s, "events": c,
                "share_of_events": c / total}
               for b, c in sorted(win.items(), key=lambda kv: -kv[1])[:top_k]]
    cover = {}
    for k in (1, 5, 10):
        sel = sum(e["violating_trials"] for e in ranking if e["exact_rank"] is not None
                  and e["exact_rank"] <= k)
        cover[f"top_{k}_exact_share_of_events"] = (sel / total) if total else None
    lo, hi = wilson(k_viol, trials)
    drv = {"events_with_0_degraded_drones": by_deg[0], "events_with_1_degraded_drone": by_deg[1],
           "events_with_2_degraded_drones": by_deg[2], "events_involving_event_drone": ev_events}
    stress = {
        "label": LABEL,
        "profile": dataclasses.asdict(cfg),
        "trials": trials, "seed": seed, "d_min_m": d_min,
        "p_violation": k_viol / trials, "p_violation_ci95": [lo, hi],
        "target_upper_bound": target, "exceeds_target": hi > target,
        "violation_events": total, "pairs_with_violations": len(ranking),
        "pair_ranking": ranking[:max(top_k, 20)],
        "time_windows": windows, "coverage_by_exact_rank": cover, "drivers": drv,
    }
    return {"format": FORMAT, "label": LABEL, "show": show["name"],
            "fleet_sha256": fleet.content_hash(), "exact": exact, "stress": stress,
            "notice": GNSS_NOTICE, "wall_s": time.perf_counter() - t0,
            "environment": environment({"stress_seed": seed})}
