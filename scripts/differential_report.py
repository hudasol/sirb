"""Large differential tier: sirb.verify (exact roots) against sirb.verify2 (interval branch and
bound), plus a brute-force sampling check. Fixed seeds only. Writes results/differential.json.

Usage: PYTHONPATH=. python scripts/differential_report.py [--random N] [--adv N] [--quick]
"""

from __future__ import annotations

import argparse
import json
import platform
import time
from pathlib import Path

import numpy as np
import scipy

from sirb.diffcases import (
    ADVERSARIAL,
    adversarial_case,
    compare,
    exact_pairs,
    hold_sample,
    rand_poly,
    random_fleet,
    random_large_fleet,
    reference_min,
)
from sirb.trajectory import Fleet, Segment, Trajectory
from sirb.verify import pairwise_min_separation
from sirb.verify2 import verify2

OUT = Path(__file__).resolve().parents[1] / "results" / "differential.json"

# Families whose construction has a multiple root of d|D|^2/dt (used only to label a cause that
# the reference computation has already confirmed; never to hide a disagreement).
MULTIPLE_ROOT = {"degenerate_collision", "degenerate_roots", "parallel_double"}


def diagnose(fleet, rec, family):
    """Cause analysis for one disagreement: extended precision reference near the exact
    verifier's argmin, then a labelled cause."""
    d = {}
    pair = rec.get("pair")
    if pair is None and fleet.n == 2:
        pair = [0, 1]
    if pair is None or "t_e" not in rec:
        d["cause"] = "UNDIAGNOSED: no pair/time in record"
        return d
    ref, t_ref = reference_min(fleet, tuple(pair), rec["t_e"])
    e, l_, u_ = rec.get("e"), rec.get("l"), rec.get("u")
    d["reference_min_longdouble"] = ref
    d["reference_argmin_t"] = t_ref
    if e is not None:
        d["e_minus_reference"] = e - ref
    if l_ is not None:
        d["l_minus_reference"] = l_ - ref
    if u_ is not None:
        d["u_minus_reference"] = u_ - ref
    verify_off = e is not None and abs(e - ref) > 1e-9 and e - ref > 0
    verify2_off = (l_ is not None and l_ > ref + 1e-9) or (u_ is not None and u_ < ref - 1e-9)
    if verify_off and not verify2_off:
        if family in MULTIPLE_ROOT:
            d["cause"] = ("float conditioning in sirb.verify: the minimum is a root of high "
                          "multiplicity of d|D|^2/dt, eigenvalue/Newton root error ~ eps^(1/m) "
                          "leaves the best candidate off the flat minimum; the reported minimum "
                          "is too large by e_minus_reference, below the 1e-6 policy tolerance "
                          "when that value is under 1e-6")
        else:
            d["cause"] = "UNDIAGNOSED: sirb.verify above reference, family not multiple-root"
    elif verify2_off and not verify_off:
        d["cause"] = "UNDIAGNOSED: sirb.verify2 bound inconsistent with reference"
    elif verify_off and verify2_off:
        d["cause"] = "UNDIAGNOSED: both deviate from reference"
    else:
        d["cause"] = "UNDIAGNOSED: both agree with reference within 1e-9; tolerance policy?"
    return d


def sampled_min(fleet, i, j, t_exact):
    """Minimum of |p_i - p_j| from dt = 1e-4 s samples: windows around the exact argmin and
    around the coarse (1e-2 s) global argmin; the whole span at 1e-4 when it is short."""
    a, b = fleet.trajectories[i], fleet.trajectories[j]
    t_lo = min(a.t_start, b.t_start)
    t_hi = max(a.t_end, b.t_end)

    def dist(ts):
        return np.linalg.norm(hold_sample(a, ts) - hold_sample(b, ts), axis=1)

    best = np.inf
    span = t_hi - t_lo
    coarse = np.arange(t_lo, t_hi + 1e-2, 1e-2)
    dc = dist(coarse)
    best = min(best, float(dc.min()))
    centres = [t_exact, float(coarse[int(np.argmin(dc))])]
    for c in centres:
        w = np.arange(max(t_lo, c - 0.05), min(t_hi, c + 0.05) + 1e-4, 1e-4)
        if len(w):
            best = min(best, float(dist(w).min()))
    if span <= 1.0:
        fine = np.arange(t_lo, t_hi + 1e-4, 1e-4)
        best = min(best, float(dist(fine).min()))
    breaks = np.array([x for tr in (a, b) for sg in tr.segments for x in (sg.t0, sg.t1)])
    with_breaks = min(best, float(dist(breaks).min()))
    return best, with_breaks


def brute_force(n_single: int, n_fleets: int):
    rows = []
    rng = np.random.default_rng(424242)
    for k in range(n_single):
        T = float(rng.uniform(0.2, 3.0)) if k % 2 == 0 else float(rng.uniform(3.0, 20.0))
        trs = []
        for _ in range(2):
            c = rand_poly(rng, int(rng.integers(1, 6)))
            if k % 2:
                c = c / (T ** np.arange(c.shape[0]))[:, None]
            trs.append(Trajectory([Segment(0.0, T, c)]))
        fleet = Fleet(trs)
        res = exact_pairs(fleet)
        if not res:
            continue
        (e, t), = res.values()
        r = verify2(fleet, 1.0, cap=1e9, per_pair=True)
        sm, smb = sampled_min(fleet, 0, 1, t)
        rows.append(("single", k, e, sm, r.pair_lower[(0, 1)], smb, t, fleet))
    for s in range(n_fleets):
        fleet, _, _ = random_fleet(100000 + s, clean=True)
        res = exact_pairs(fleet)
        r = verify2(fleet, 1.0, cap=1e9, per_pair=True)
        for (i, j), (e, t) in res.items():
            sm, smb = sampled_min(fleet, i, j, t)
            rows.append(("fleet", s, e, sm, r.pair_lower[(i, j)], smb, t, fleet))
    diff = np.array([r[3] - r[2] for r in rows])        # sampled - exact
    lgap = np.array([r[3] - r[4] for r in rows])        # sampled - lower bound
    exact = np.array([r[2] for r in rows])
    kink = exact < 1e-2
    diff_b = np.array([r[5] - r[2] for r in rows])      # grid plus breakpoints - exact
    at_break = np.array([
        any(abs(r[6] - x) < 1e-9 for tr in r[7].trajectories for sg in tr.segments
            for x in (sg.t0, sg.t1)) for r in rows])
    return {
        "n_exact_argmin_at_a_segment_breakpoint": int(at_break.sum()),
        "max_sampled_minus_exact_argmin_not_at_breakpoint": float(diff[~at_break].max()),
        "n_above_1e-6_argmin_at_breakpoint": int(((diff > 1e-6) & at_break).sum()),
        "n_above_1e-6_argmin_not_at_breakpoint": int(((diff > 1e-6) & ~at_break).sum()),
        "grid_plus_breakpoints_max_sampled_minus_exact": float(diff_b.max()),
        "grid_plus_breakpoints_max_negative_gap": float(max(0.0, (-diff_b).max())),
        "n_comparisons": len(rows),
        "n_single_segment_pairs": sum(1 for r in rows if r[0] == "single"),
        "n_fleet_pairs": sum(1 for r in rows if r[0] == "fleet"),
        "dt_s": 1e-4,
        "max_sampled_minus_exact": float(diff.max()),
        "max_negative_gap_exact_minus_sampled": float(max(0.0, (-diff).max())),
        "n_sampled_below_exact_by_more_than_1e-9": int((diff < -1e-9).sum()),
        "n_sampled_above_exact_by_more_than_1e-6": int((diff > 1e-6).sum()),
        "n_sampled_above_exact_by_more_than_1e-6_with_exact_below_1e-2": int(
            ((diff > 1e-6) & kink).sum()),
        "n_sampled_above_exact_by_more_than_1e-6_with_exact_above_1e-2": int(
            ((diff > 1e-6) & ~kink).sum()),
        "max_sampled_minus_exact_where_exact_above_1e-2": float(
            diff[~kink].max()) if (~kink).any() else None,
        "verify2_lower_bound_above_sampled_by_more_than_1e-9": int((lgap < -1e-9).sum()),
        "max_verify2_lower_bound_minus_sampled": float(max(0.0, (-lgap).max())),
    }


def brute_force_near_collision(n: int):
    """Straight passes with a tiny offset d: |D(t)| has a V shape of slope v at the minimum, so
    even an exact argmin window on a 1e-4 grid can sit up to v * dt / 2 above the minimum."""
    rng = np.random.default_rng(777)
    rows = []
    for _ in range(n):
        v = float(rng.uniform(0.5, 10.0))
        d = float(rng.choice([0.0, 1e-5, 1e-3]))
        ts = float(rng.uniform(0.5, 1.5))
        a = Trajectory([Segment(0.0, 2.0, np.zeros((1, 3)))])
        b = Trajectory([Segment(0.0, 2.0, np.array([[-v * ts, d, 0.0], [v, 0.0, 0.0]]))])
        fleet = Fleet([a, b])
        (e, t), = exact_pairs(fleet).values()
        rows.append((v, d, e, sampled_min(fleet, 0, 1, t)[0]))
    diff = np.array([r[3] - r[2] for r in rows])
    bound = np.array([r[0] * 1e-4 / 2 for r in rows])
    return {"n": len(rows), "dt_s": 1e-4,
            "max_sampled_minus_exact": float(diff.max()),
            "max_negative_gap_exact_minus_sampled": float(max(0.0, (-diff).max())),
            "n_sampled_above_exact_by_more_than_1e-6": int((diff > 1e-6).sum()),
            "max_ratio_to_v_dt_over_2": float((diff / bound).max()),
            "note": "V-shaped minimum: sampling error is bounded by v*dt/2, not by 1e-6"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--random", type=int, default=8000)
    ap.add_argument("--adv", type=int, default=250, help="seeds per adversarial family")
    ap.add_argument("--big", type=int, default=100, help="12-drone fleets")
    ap.add_argument("--brute-single", type=int, default=1500)
    ap.add_argument("--brute-fleets", type=int, default=300)
    ap.add_argument("--out", default=str(OUT))
    args = ap.parse_args()

    t_all = time.perf_counter()
    disagreements: list = []
    agg = {"max_e_minus_l": 0.0, "max_l_minus_e": 0.0, "max_abs_e_minus_construction": 0.0,
           "max_e_minus_u_witness": 0.0, "n_undecided_leaves": 0, "n_pairs_compared": 0,
           "max_gap_bound": 0.0}
    counts = {"random_fleets": 0, "adversarial_fleets": 0, "adversarial_by_family": {},
              "larger_fleets_12_drones": 0, "verdict_checks": 0}
    runtimes = {}
    # every counter is written even when zero, so a reader can tell zero from a missing key
    vc: dict = {"PASS": 0, "FAIL": 0, "UNDECIDED": 0, "pass_with_exact_below_threshold": 0,
                "pass_with_exact_below_threshold_max_margin": 0.0,
                "fail_with_exact_at_or_above_threshold": 0,
                "fail_with_exact_above_threshold_max_margin": 0.0,
                "undecided_max_abs_e_minus_threshold": 0.0}

    def absorb(stats, dis, family, seed, idx, tag):
        counts["verdict_checks"] += len(stats.get("verdicts", []))
        agg["n_pairs_compared"] += stats.get("n_pairs", 0)
        if "e" not in stats:
            return
        agg["max_e_minus_l"] = max(agg["max_e_minus_l"], stats["max_e_minus_l"])
        agg["max_l_minus_e"] = max(agg["max_l_minus_e"], stats["max_l_minus_e"])
        agg["n_undecided_leaves"] += stats["undecided_leaves"]
        agg["max_gap_bound"] = max(agg["max_gap_bound"], stats["gap_bound"])
        if "e_minus_true" in stats:
            agg["max_abs_e_minus_construction"] = max(
                agg["max_abs_e_minus_construction"], abs(stats["e_minus_true"]))
        agg["max_e_minus_u_witness"] = max(agg["max_e_minus_u_witness"],
                                           stats["e"] - stats["u"])
        for v, off in stats.get("verdicts", []):
            vc[v] = vc.get(v, 0) + 1
            if v == "UNDECIDED":
                vc["undecided_max_abs_e_minus_threshold"] = max(
                    vc.get("undecided_max_abs_e_minus_threshold", 0.0), abs(off))
            if v == "FAIL" and off >= 0:
                vc["fail_with_exact_at_or_above_threshold"] = vc.get(
                    "fail_with_exact_at_or_above_threshold", 0) + 1
                vc["fail_with_exact_above_threshold_max_margin"] = max(
                    vc.get("fail_with_exact_above_threshold_max_margin", 0.0), off)
            if v == "PASS" and off < 0:
                vc["pass_with_exact_below_threshold"] = vc.get(
                    "pass_with_exact_below_threshold", 0) + 1
                vc["pass_with_exact_below_threshold_max_margin"] = max(
                    vc.get("pass_with_exact_below_threshold_max_margin", 0.0), -off)

    def record(dis, fleet, family, seed, idx, meta):
        for d in dis:
            rec = dict(d)
            rec.update(family=family, seed=seed, index=idx,
                       meta={k: v for k, v in meta.items() if k != "d_true"} if meta else {})
            rec["diagnosis"] = diagnose(fleet, rec, family)
            disagreements.append(rec)

    # --- random fleets
    t0 = time.perf_counter()
    for s in range(args.random):
        fleet, d_min, meta = random_fleet(s)
        stats, dis = compare(fleet, d_min, None, np.random.default_rng(s))
        absorb(stats, dis, "random", s, s, "random")
        record(dis, fleet, "random", s, s, meta)
        counts["random_fleets"] += 1
    runtimes["random_fleets_s"] = time.perf_counter() - t0

    # --- adversarial families
    t0 = time.perf_counter()
    for fam in sorted(ADVERSARIAL):
        n_fam = 0
        for s in range(args.adv):
            fleet, d_min, meta = adversarial_case(fam, s)
            stats, dis = compare(fleet, d_min, meta.get("d_true"), np.random.default_rng(s))
            absorb(stats, dis, fam, s, s, "adv")
            record(dis, fleet, fam, s, s, meta)
            n_fam += 1
        counts["adversarial_by_family"][fam] = n_fam
        counts["adversarial_fleets"] += n_fam
    runtimes["adversarial_fleets_s"] = time.perf_counter() - t0

    # --- larger fleets, including 100 drones
    t0 = time.perf_counter()
    for s in range(args.big):
        fleet, _, meta = random_large_fleet(s, n=12)
        stats, dis = compare(fleet, None, None, np.random.default_rng(s))
        absorb(stats, dis, "larger_12", s, s, "random")
        record(dis, fleet, "larger_12", s, s, meta)
        counts["larger_fleets_12_drones"] += 1
    runtimes["larger_fleets_s"] = time.perf_counter() - t0

    hundred = []
    for s in range(6):
        spread = 6.0 if s < 3 else 60.0
        fleet, _, _ = random_large_fleet(1000 + s, n=100, spread=spread)
        t1 = time.perf_counter()
        r = verify2(fleet, 1.0)
        t_v2 = time.perf_counter() - t1
        t1 = time.perf_counter()
        pairwise_min_separation(fleet, 4.0)
        t_v1 = time.perf_counter() - t1
        e_full = min(v[0] for v in exact_pairs(fleet).values())
        r_lo = verify2(fleet, 0.9 * e_full, cap=1.8 * e_full + 1)     # threshold below e
        r_hi = verify2(fleet, 1.1 * e_full, cap=2.2 * e_full + 1)     # threshold above e
        row = dict(seed=1000 + s, spread_m=spread, n_drones=100, verify2_s=t_v2,
                   verify_s=t_v1, verify2_cells=r.n_cells,
                   verify2_piece_pairs=r.n_piece_pairs, exact_min=e_full,
                   verify2_lower=r.lower_bound, verify2_upper=r.upper_bound,
                   verdict_at_0p9_exact=r_lo.verdict, verdict_at_1p1_exact=r_hi.verdict,
                   ok=bool(r.lower_bound <= e_full + 1e-9
                           and (e_full - r.lower_bound <= r.gap_bound
                                or r.lower_bound >= r.cap - 1e-12)
                           and r_lo.verdict == "PASS" and r_hi.verdict == "FAIL"))
        hundred.append(row)
        if not row["ok"]:
            disagreements.append(dict(kind="hundred_drone_mismatch", family="larger_100",
                                      seed=1000 + s, index=s, **row,
                                      diagnosis={"cause": "UNDIAGNOSED"}))
    runtimes["hundred_drone_fleets"] = hundred
    counts["hundred_drone_fleets"] = len(hundred)

    # --- brute force
    t0 = time.perf_counter()
    bf = brute_force(args.brute_single, args.brute_fleets)
    bf["near_collision_passes"] = brute_force_near_collision(300)
    runtimes["brute_force_s"] = time.perf_counter() - t0

    kinds = {}
    for d in disagreements:
        kinds[d["kind"]] = kinds.get(d["kind"], 0) + 1
    total_fleets = (counts["random_fleets"] + counts["adversarial_fleets"]
                    + counts["larger_fleets_12_drones"] + counts["hundred_drone_fleets"])
    out = {
        "generated_by": "scripts/differential_report.py",
        "environment": {"python": platform.python_version(),
                        "numpy": np.__version__, "scipy": scipy.__version__},
        "criteria": {"C1": "l <= e + 1e-9", "C2": "if no undecided cell: e - l <= gap_bound + 1e-9 "
                     "(verify2 tol 1e-9 plus rounding allowance, plus the 1e-9 accuracy "
                     "claimed for the exact verifier)",
                     "C3": "u >= e - 1e-9 (witnessed distance cannot beat the exact minimum)",
                     "C4": "no PASS/FAIL contradiction outside +/-1e-6 m of the threshold"},
        "counts": dict(counts, total_fleets=total_fleets,
                       n_disagreement_records=len(disagreements),
                       disagreements_by_kind=kinds),
        "verdicts": vc,
        "max_discrepancies": agg,
        "brute_force": bf,
        "disagreements": disagreements,
        "runtimes": dict(runtimes, total_s=time.perf_counter() - t_all),
    }
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(out, indent=1))
    print(json.dumps({k: out[k] for k in ("counts", "verdicts", "max_discrepancies",
                                          "brute_force")},
                     indent=1))
    print("runtime", out["runtimes"]["total_s"])


if __name__ == "__main__":
    main()
