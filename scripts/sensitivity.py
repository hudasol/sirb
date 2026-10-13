"""G2: which assumptions move the probability of a separation violation most.

Method (fixed seeds, common random numbers: every configuration reuses the same trial seeds, so
differences between configurations are paired and less noisy than independent runs):
 1. One at a time (OAT): each continuous parameter at a low and high value (about half and double
    the default, range stated in PARAMS) around two baselines, a stressed one (formation spacing 4.0 m)
    and a benign one (5.0 m). Add on hazards (dropout, low battery, comms loss, one motor failure)
    are switched on from zero.
 2. Global: a Latin hypercube over all continuous parameters (log uniform), Spearman rank correlation
    of each parameter with the median minimum separation per sample.
Everything is a property of the simulated model with ASSUMED parameters. Results are cached per unit
in results/.cache so an interrupted run resumes.

    python scripts/sensitivity.py            # writes results/sensitivity.json
"""

from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from run_stress import DT, SEED, WORKERS, cached, family_show, plan_and_ref  # noqa: E402

from sirb.disturbance import StressConfig  # noqa: E402
from sirb.stress import monte_carlo  # noqa: E402

TRIALS_OAT = 300
TRIALS_LHS = 40
N_LHS = 120
SCENARIO = {"w20": 6.0, "gnss_deg_fraction": 0.25, "gnss_bias_sigma": 0.5}
BASELINES = {"stressed_spacing_4.0": 4.0, "benign_spacing_5.0": 5.0}

# name: (low, default, high). Defaults are the values in the scenario or StressConfig.
PARAMS = {
    "w20": (3.0, 6.0, 12.0),
    "wind_local_fraction": (0.15, 0.3, 0.6),
    "wind_tau": (2.5, 5.0, 10.0),
    "drag_gain": (0.15, 0.3, 0.6),
    "omega_n": (1.0, 2.0, 4.0),
    "zeta": (0.7, 0.9, 1.2),
    "gnss_white_sigma": (0.0075, 0.015, 0.03),
    "gnss_bias_sigma": (0.25, 0.5, 1.0),
    "gnss_bias_tau": (15.0, 30.0, 60.0),
    "gnss_deg_fraction": (0.125, 0.25, 0.5),
}
ADDONS = {
    "gnss_dropout_5pct_10s": {"gnss_loss_fraction": 0.05},
    "battery_low_5pct_descend": {"battery_low_fraction": 0.05},
    "comms_loss_5pct_continue": {"comms_loss_fraction": 0.05},
    "one_motor_failure_ballistic": {"motor_failures": 1},
}


def run(ref, d_min, over, trials, tag=0):
    cfg = StressConfig(dt=DT, **{**SCENARIO, **over})
    return monte_carlo(ref, cfg, d_min, trials, SEED, tag=tag, workers=WORKERS)


def slim(r):
    return {"p": r["p_violation"], "ci": r["p_violation_ci95"],
            "median_min_sep": r["min_separation_quantiles"]["0.5"],
            "q05_min_sep": r["min_separation_quantiles"]["0.05"],
            "mean_max_tracking_error_m": r["mean_max_tracking_error_m"]}


def oat(name, spacing):
    show = family_show(spacing)
    _fleet, rep, ref = plan_and_ref(show)
    d_min = show["limits"]["min_separation_m"]
    out = {"nominal_min_separation_m": rep.min_separation, "d_min": d_min, "trials": TRIALS_OAT,
           "baseline": cached(f"sens_{name}_base", lambda: slim(run(ref, d_min, {}, TRIALS_OAT))),
           "params": {}, "addons": {}}
    for p, (lo, _de, hi) in PARAMS.items():
        out["params"][p] = {
            "low": {"value": lo, **cached(f"sens_{name}_{p}_lo",
                                          lambda: slim(run(ref, d_min, {p: lo}, TRIALS_OAT)))},
            "high": {"value": hi, **cached(f"sens_{name}_{p}_hi",
                                           lambda: slim(run(ref, d_min, {p: hi}, TRIALS_OAT)))}}
        print(name, p, flush=True)
    for a, over in ADDONS.items():
        out["addons"][a] = cached(f"sens_{name}_add_{a}",
                                  lambda: slim(run(ref, d_min, over, TRIALS_OAT)))
    return out


def lhs():
    show = family_show(4.0)
    _fleet, _rep, ref = plan_and_ref(show)
    d_min = show["limits"]["min_separation_m"]
    rng = np.random.default_rng(SEED)
    names = list(PARAMS)
    u = np.empty((N_LHS, len(names)))
    for j in range(len(names)):
        u[:, j] = (rng.permutation(N_LHS) + rng.random(N_LHS)) / N_LHS
    rows = []
    for i in range(N_LHS):
        over = {}
        for j, p in enumerate(names):
            lo, _de, hi = PARAMS[p]
            over[p] = float(math.exp(math.log(lo) + u[i, j] * (math.log(hi) - math.log(lo))))
        r = cached(f"sens_lhs_{i}", lambda: slim(run(ref, d_min, over, TRIALS_LHS, tag=i + 1)))
        rows.append({"params": over, **r})
    from scipy.stats import spearmanr
    y_med = [r["median_min_sep"] for r in rows]
    y_p = [r["p"] for r in rows]
    corr = {}
    for p in names:
        x = [r["params"][p] for r in rows]
        a = spearmanr(x, y_med)
        b = spearmanr(x, y_p)
        corr[p] = {"rho_median_min_sep": float(a.statistic), "p_value_median": float(a.pvalue),
                   "rho_p_violation": float(b.statistic), "p_value_pviol": float(b.pvalue)}
    return {"n_samples": N_LHS, "trials_per_sample": TRIALS_LHS, "spearman": corr,
            "note": "log uniform over the stated low..high ranges; spacing 4.0 m show"}


def rank_table(res):
    """Rank parameters by |delta median min separation| (low noise) for each baseline."""
    out = {}
    for name, d in res["oat"].items():
        b = d["baseline"]["median_min_sep"]
        rows = []
        for p, v in d["params"].items():
            dl = v["low"]["median_min_sep"] - b
            dh = v["high"]["median_min_sep"] - b
            rows.append({"param": p, "delta_median_min_sep_low": dl, "delta_median_min_sep_high": dh,
                         "swing_m": abs(dh - dl),
                         "delta_p_low": v["low"]["p"] - d["baseline"]["p"],
                         "delta_p_high": v["high"]["p"] - d["baseline"]["p"]})
        rows.sort(key=lambda r: -r["swing_m"])
        out[name] = rows
    return out


def main():
    t = time.time()
    res = {"scenario": SCENARIO, "trials_oat": TRIALS_OAT, "ranges": PARAMS, "oat": {}}
    for name, sp in BASELINES.items():
        res["oat"][name] = oat(name, sp)
    res["ranking_by_swing"] = rank_table(res)
    res["global"] = lhs()
    res["wall_s"] = time.time() - t
    Path("results/sensitivity.json").write_text(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
