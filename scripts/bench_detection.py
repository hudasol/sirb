"""Detection benchmark: exact verifier, interval verifier and sampled checkers on cases with a
truth that does not come from the verifier under test. Fixed seeds. Writes
results/benchmark_v2.json (docs/BENCHMARK_V2.md is generated from it).

Strata (threshold 2.0 m):
  gross     two drone straight crossings, analytic miss distance 0 to 0.5 x threshold
  shallow   miss distance 0.5 to 0.99 x threshold
  safe      miss distance 1.01 to 1.5 x threshold (control: any alarm is a false positive)
  razor     analytic adversarial families (depth or margin of 1e-6 and 1e-4 m)
Also: fleet size sweep 4 to 64 drones with repeated timing.
"""

from __future__ import annotations

import json
import statistics
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from sirb.baselines.sampled import sampled_min_separation_dense  # noqa: E402
from sirb.diffcases import adversarial_case  # noqa: E402
from sirb.stress import wilson  # noqa: E402
from sirb.trajectory import Fleet, Segment, Trajectory  # noqa: E402
from sirb.verify import pairwise_min_separation  # noqa: E402
from sirb.verify2 import verify2  # noqa: E402

D_MIN = 2.0
T = 10.0
N_PER = 200
FIXED_DT = (0.5, 0.2, 0.1, 0.05, 0.01)
RAZOR_FAMILIES = ("graze_linear", "graze_curved", "boundary_min", "split_smooth", "tiny_segments")
SEED = 20261028


def line(p0, v, T_=T):
    return Trajectory([Segment(0.0, T_, np.stack([np.asarray(p0, float), np.asarray(v, float)]))])


def crossing(rng, lo: float, hi: float):
    """Two straight flights whose closest approach is exactly s in [lo, hi] x D_MIN."""
    while True:
        va = rng.normal(size=3)
        vb = rng.normal(size=3)
        va = rng.uniform(1.0, 4.0) * va / np.linalg.norm(va)
        vb = rng.uniform(1.0, 4.0) * vb / np.linalg.norm(vb)
        vrel = va - vb
        if np.linalg.norm(vrel) >= 1.0:
            break
    s = rng.uniform(lo, hi) * D_MIN
    w = rng.normal(size=3)
    w -= w @ vrel / (vrel @ vrel) * vrel
    w = s * w / np.linalg.norm(w)
    tc = rng.uniform(2.0, 8.0)
    pa = -va * tc
    pb = w - vb * tc
    return Fleet([line(pa, va), line(pb, vb)]), s


def timed(fn, reps=3):
    best = []
    out = None
    for _ in range(reps):
        t0 = time.perf_counter()
        out = fn()
        best.append(time.perf_counter() - t0)
    return out, statistics.median(best)


def matched_dt(fleet, budget_s: float) -> float:
    """Frame step whose measured sampler cost is about ``budget_s`` (never above T)."""
    probe = 0.5
    _, c = timed(lambda: sampled_min_separation_dense(fleet, probe, D_MIN), reps=3)
    per_frame = c / max(int(fleet.t_end / probe) + 1, 1)
    frames = max(int(budget_s / per_frame), 2)
    return float(min(fleet.t_end / frames, fleet.t_end))


def run_case(fleet, truth_violates: bool):
    row = {"truth_violates": bool(truth_violates)}
    (d, *_), t_exact = timed(lambda: pairwise_min_separation(fleet, 1000.0))
    row["exact_pass"] = bool(d >= D_MIN)
    row["exact_min"] = float(d)
    row["exact_s"] = t_exact
    r2, t2 = timed(lambda: verify2(fleet, D_MIN), reps=1)
    row["interval_verdict"] = r2.verdict
    row["interval_s"] = t2
    row["fixed"] = {}
    for dt in FIXED_DT:
        rep, ts = timed(lambda dt=dt: sampled_min_separation_dense(fleet, dt, D_MIN), reps=1)
        row["fixed"][str(dt)] = {"pass": bool(rep.ok), "s": ts}
    dtm = matched_dt(fleet, t_exact)
    rep = sampled_min_separation_dense(fleet, dtm, D_MIN)
    row["matched"] = {"dt": dtm, "pass": bool(rep.ok)}
    return row


def stratum_summary(rows, violating: bool):
    """Counts of correct and incorrect verdicts; truth_violates is the label."""
    n = len(rows)

    def rate(k):
        lo, hi = wilson(k, n)
        return {"count": k, "n": n, "rate": k / n, "ci95": [lo, hi]}

    out = {"n": n,
           "exact_wrong": rate(sum(r["exact_pass"] == r["truth_violates"] for r in rows)),
           "interval_wrong": rate(sum(
               (r["interval_verdict"] == "PASS") == r["truth_violates"]
               and r["interval_verdict"] != "UNDECIDED" for r in rows)),
           "interval_undecided": rate(sum(r["interval_verdict"] == "UNDECIDED" for r in rows)),
           "median_exact_s": statistics.median(r["exact_s"] for r in rows),
           "median_interval_s": statistics.median(r["interval_s"] for r in rows),
           "fixed_wrong": {}, "median_fixed_s": {}}
    for dt in FIXED_DT:
        k = str(dt)
        out["fixed_wrong"][k] = rate(sum(r["fixed"][k]["pass"] == r["truth_violates"] for r in rows))
        out["median_fixed_s"][k] = statistics.median(r["fixed"][k]["s"] for r in rows)
    out["matched_wrong"] = rate(sum(r["matched"]["pass"] == r["truth_violates"] for r in rows))
    out["matched_dt_median"] = statistics.median(r["matched"]["dt"] for r in rows)
    out["label"] = "violating" if violating else "safe"
    return out


def size_sweep(rng_seed=0):
    sizes = (4, 8, 16, 32, 64)
    res = {}
    for n in sizes:
        rng = np.random.default_rng(rng_seed + n)
        fl = Fleet([line(rng.uniform(-30, 30, 3), rng.normal(size=3) * 2.0) for _ in range(n)])
        times = []
        for _ in range(7):
            t0 = time.perf_counter()
            d, *_rest, npairs = pairwise_min_separation(fl, 1000.0)
            times.append(time.perf_counter() - t0)
        dts = {}
        for dt in (0.1, 0.05, 0.01):
            tt = []
            for _ in range(7):
                t0 = time.perf_counter()
                sampled_min_separation_dense(fl, dt, D_MIN)
                tt.append(time.perf_counter() - t0)
            dts[str(dt)] = {"median_s": statistics.median(tt),
                            "cv": statistics.pstdev(tt) / statistics.mean(tt)}
        res[str(n)] = {"exact_median_s": statistics.median(times),
                       "exact_cv": statistics.pstdev(times) / statistics.mean(times),
                       "pairs_examined": int(npairs), "pairs_total": n * (n - 1) // 2,
                       "sampled": dts}
    return res


def main():
    rng = np.random.default_rng(SEED)
    strata = {}
    specs = {"gross": (0.0, 0.5, True), "shallow": (0.5, 0.99, True), "safe": (1.01, 1.5, False)}
    for name, (lo, hi, viol) in specs.items():
        rows = []
        for _ in range(N_PER):
            fl, s = crossing(rng, lo, hi)
            rows.append(run_case(fl, s < D_MIN))
        strata[name] = stratum_summary(rows, viol)
        print(name, {k: strata[name][k]["rate"] for k in ("exact_wrong", "matched_wrong")},
              flush=True)
    rows_v, rows_s = [], []
    seed = 0
    for fam in RAZOR_FAMILIES:
        for _ in range(N_PER // len(RAZOR_FAMILIES)):
            seed += 1
            fl, d_min, meta = adversarial_case(fam, seed)
            if meta["d_true"] is None or abs(meta["delta"]) == 0.0:
                continue
            # these cases carry their own d_min; rescale is not needed, only the threshold
            row = run_case_threshold(fl, d_min, meta["d_true"] < d_min)
            (rows_v if meta["d_true"] < d_min else rows_s).append(row)
    strata["razor_violating"] = stratum_summary(rows_v, True)
    strata["razor_safe"] = stratum_summary(rows_s, False)
    print("razor", len(rows_v), len(rows_s), flush=True)
    out = {"seed": SEED, "d_min_m": D_MIN, "n_per_stratum": N_PER, "strata": strata,
           "size_sweep": size_sweep()}
    (ROOT / "results" / "benchmark_v2.json").write_text(json.dumps(out, indent=1))


def run_case_threshold(fleet, d_min, truth_violates):
    global D_MIN
    old, D_MIN = D_MIN, d_min
    try:
        return run_case(fleet, truth_violates)
    finally:
        D_MIN = old


if __name__ == "__main__":
    main()
