"""Speed of the vectorised candidate search against the loop reference, and its effect on the
total ``verify()`` time. Fixed seeds; the fleets are the planned baseline fleets of
``scripts/profile_scale.py`` (same ``scale_show``, relaxed limits, computational scaling only).

For each n: time ``_candidate_pairs`` and ``_candidate_pairs_loop`` on the same pieces (best of
``--repeats``), check that both return identical arrays, then time ``verify()`` with each
implementation swapped in by monkeypatching the module attribute (old and new interleaved, best
of ``--verify-repeats``) and check that the two reports agree. Wall times are machine dependent;
the machine is recorded in the output.

    PYTHONPATH=. python scripts/bench_verify_speed.py [--sizes 100 250 ...] [--out results/verify_speed.json]
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
import platform
import sys
import time
import types
from pathlib import Path

import numpy as np
import scipy

import sirb.verify as V

ROOT = Path(__file__).resolve().parents[1]
SIZES = (100, 250, 500, 1000, 2000)


def _profile_scale():
    """Import scripts/profile_scale.py for its show builder (it imports the POSIX-only
    ``resource`` module at top level, which is replaced by an unused stub on Windows)."""
    try:
        import resource  # noqa: F401
    except ImportError:
        sys.modules["resource"] = types.ModuleType("resource")
    spec = importlib.util.spec_from_file_location("profile_scale", ROOT / "scripts" / "profile_scale.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _best(fn, repeats):
    best, out = float("inf"), None
    for _ in range(repeats):
        t = time.perf_counter()
        out = fn()
        best = min(best, time.perf_counter() - t)
    return best, out


def _same_report(a, b):
    keys = ("ok", "min_separation", "min_separation_pair", "min_separation_time",
            "n_pairs_checked", "n_segments", "violation_counts")
    return all(getattr(a, k) == getattr(b, k) for k in keys)


def bench(n, repeats, verify_repeats, ps):
    from sirb.planner.baseline import plan_baseline
    show = ps.scale_show(n)
    fleet, _ = plan_baseline(show)
    limits = show["limits"]
    R = max(4.0 * float(limits["min_separation_m"]), 1.01 * float(limits["min_separation_m"]) + 1e-3)
    good, bad = V._structure(fleet, [])
    P = V._build_pieces(good, bad)
    row = {"n": n, "n_pieces": int(P.n), "search_radius_m": R}
    for prefilter in (True, False):
        if not prefilter and n > 500:
            continue                      # all time-overlapping pairs: memory heavy, not the verify path
        t_old, (oa, ob) = _best(lambda: V._candidate_pairs_loop(P, R, prefilter), repeats)
        t_new, (na, nb) = _best(lambda: V._candidate_pairs(P, R, prefilter), repeats)
        tag = "prefilter" if prefilter else "no_prefilter"
        row[tag] = {"n_pairs": int(len(na)), "old_s": t_old, "new_s": t_new,
                    "speedup": t_old / t_new,
                    "identical": bool(np.array_equal(oa, na) and np.array_equal(ob, nb)
                                      and oa.dtype == na.dtype and ob.dtype == nb.dtype)}
    new_impl, old_impl = V._candidate_pairs, V._candidate_pairs_loop
    t_new_v, t_old_v, rep_new, rep_old = float("inf"), float("inf"), None, None
    try:
        for _ in range(verify_repeats):
            V._candidate_pairs = old_impl
            t = time.perf_counter()
            rep_old = V.verify(fleet, limits)
            t_old_v = min(t_old_v, time.perf_counter() - t)
            V._candidate_pairs = new_impl
            t = time.perf_counter()
            rep_new = V.verify(fleet, limits)
            t_new_v = min(t_new_v, time.perf_counter() - t)
    finally:
        V._candidate_pairs = new_impl
    row["verify"] = {"old_s": t_old_v, "new_s": t_new_v, "speedup": t_old_v / t_new_v,
                     "repeats": verify_repeats, "reports_equal": _same_report(rep_old, rep_new),
                     "ok": bool(rep_new.ok), "n_pairs_checked": int(rep_new.n_pairs_checked),
                     "candidate_old_share_of_verify_old": row["prefilter"]["old_s"] / t_old_v}
    return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sizes", type=int, nargs="*", default=list(SIZES))
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--verify-repeats", type=int, default=3)
    ap.add_argument("--out", default=str(ROOT / "results" / "verify_speed.json"))
    a = ap.parse_args()
    ps = _profile_scale()
    rows = []
    for n in a.sizes:
        # The largest size runs verify() fewer times: one call takes about a minute.
        vr = min(a.verify_repeats, 2) if n >= 2000 else a.verify_repeats
        row = bench(n, a.repeats, vr, ps)
        rows.append(row)
        print(json.dumps({"n": n, "cand_old_s": row["prefilter"]["old_s"],
                          "cand_new_s": row["prefilter"]["new_s"],
                          "verify_old_s": row["verify"]["old_s"],
                          "verify_new_s": row["verify"]["new_s"],
                          "identical": row["prefilter"]["identical"],
                          "reports_equal": row["verify"]["reports_equal"]}), flush=True)
    out = {
        "generated_by": "scripts/bench_verify_speed.py",
        "note": ("wall times are machine dependent (another job may have been running); compare "
                 "old and new within one row only. Fleets: planned baseline fleets of "
                 "scripts/profile_scale.py (relaxed limits, computational scaling only). "
                 "Best of `repeats` for the candidate search, best of verify.repeats for verify()."),
        "machine": {"platform": platform.platform(), "processor": platform.processor(),
                    "cpu_count": os.cpu_count(), "python": platform.python_version(),
                    "numpy": np.__version__, "scipy": scipy.__version__},
        "pair_block": V._PAIR_BLOCK,
        "repeats": a.repeats,
        "rows": rows,
    }
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).write_text(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
