"""Stress credibility studies. Seeds are declared and separated:

  SEED_TUNE  used by the worst case searches while they look for bad parameters
  SEED_EVAL  used only to evaluate the configurations the searches return

    python scripts/stress_credibility.py dt          # frame step convergence   -> results/stress_dt.json
    python scripts/stress_credibility.py search      # worst case search        -> results/stress_search.json
    python scripts/stress_credibility.py faults      # fault matrix with delay  -> results/stress_faults.json
"""

from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
os.chdir(ROOT)

from run_stress import cached, family_show  # noqa: E402

from sirb.disturbance import StressConfig  # noqa: E402
from sirb.planner.baseline import plan_baseline  # noqa: E402
from sirb.spec import load_show  # noqa: E402
from sirb.stress import monte_carlo, sample_reference  # noqa: E402
from sirb.verify import verify  # noqa: E402

SEED_TUNE = 20261101
SEED_EVAL = 20261202
WORKERS = int(os.environ.get("SIRB_WORKERS", "2"))
TRIALS = 100


def prep(show):
    fleet, _ = plan_baseline(show)
    return fleet, verify(fleet, show["limits"])


# ------------------------------------------------------------------ dt convergence

def exp_dt() -> dict:
    show = load_show("shows/sirb_text_100.json")
    fleet, rep = prep(show)
    d_min = show["limits"]["min_separation_m"]
    configs = {
        "gnss_bias0.5_deg25": dict(gnss_deg_fraction=0.25, gnss_bias_sigma=0.5),
        "wind9": dict(w20=9.0),
        "comms_loss5_corridor": dict(comms_loss_fraction=0.05,
                                     behavior={"comms_loss": "corridor", "battery_low": "descend",
                                               "gnss_loss": "continue",
                                               "motor_failure": "ballistic"}),
    }
    out = {"show": show["name"], "trials": TRIALS, "seed": SEED_EVAL, "rows": []}
    for name, kw in configs.items():
        for dt in (0.2, 0.1, 0.05):
            def one(name=name, kw=kw, dt=dt):
                ref = sample_reference(fleet, dt)
                return monte_carlo(ref, StressConfig(dt=dt, rejoin_vmax=10.0, **kw), d_min,
                                   TRIALS, SEED_EVAL, workers=WORKERS)
            t = time.time()
            mc = cached(f"cred_dt_{name}_{dt}", one)
            out["rows"].append({"config": name, "dt": dt, **mc})
            print(name, dt, round(mc["p_violation"], 3), round(time.time() - t), "s", flush=True)
    return out


# ------------------------------------------------------------------ worst case search

BOX = {  # name: (low, high)
    "gnss_bias_sigma": (0.0, 1.5),
    "gnss_deg_fraction": (0.0, 1.0),
    "w20": (0.0, 15.0),
    "gnss_loss_fraction": (0.0, 0.25),
    "comms_loss_fraction": (0.0, 0.10),
}
KEYS = list(BOX)
LO = np.array([BOX[k][0] for k in KEYS])
HI = np.array([BOX[k][1] for k in KEYS])
EVALS, M = 120, 8


def make_cfg(x: np.ndarray) -> StressConfig:
    return StressConfig(dt=0.2, rejoin_vmax=10.0, **dict(zip(KEYS, map(float, x))))


def exp_search() -> dict:
    show = family_show(5.0)
    fleet, rep = prep(show)
    d_min = show["limits"]["min_separation_m"]
    ref = sample_reference(fleet, 0.2)
    counter = {"n": 0}

    def objective(x: np.ndarray) -> float:
        counter["n"] += 1
        mc = monte_carlo(ref, make_cfg(x), d_min, M, SEED_TUNE + counter["n"], workers=1)
        return mc["min_separation_quantiles"]["0.5"]       # lower is worse for the plan

    def evaluate(x: np.ndarray) -> dict:
        return monte_carlo(ref, make_cfg(x), d_min, TRIALS, SEED_EVAL, workers=WORKERS)

    def random_search(rng):
        best_x, best_f, hist = None, np.inf, []
        for _ in range(EVALS):
            x = rng.uniform(LO, HI)
            f = objective(x)
            if f < best_f:
                best_x, best_f = x, f
            hist.append(best_f)
        return best_x, best_f, hist

    def evolution(rng):
        x = (LO + HI) / 2
        f = objective(x)
        best_x, best_f, hist = x.copy(), f, [f]
        sigma = 0.25 * (HI - LO)
        used = 1
        while used + 4 <= EVALS:
            cands = [np.clip(best_x + rng.normal(size=len(KEYS)) * sigma, LO, HI) for _ in range(4)]
            fs = [objective(c) for c in cands]
            used += 4
            k = int(np.argmin(fs))
            if fs[k] < best_f:
                best_x, best_f = cands[k], fs[k]
            hist.extend([best_f] * 4)
        return best_x, best_f, hist

    def run(name, fn, seed):
        def go():
            counter["n"] = 0
            bx, bf, hist = fn(np.random.default_rng(seed))
            ev = evaluate(bx)
            return {"method": name, "best_params": dict(zip(KEYS, map(float, bx))),
                    "tuning_objective_median_min_sep_m": float(bf), "history": [float(h) for h in hist],
                    "evaluation": ev}
        return cached(f"cred_search_{name}_{EVALS}_{M}", go)

    rs = run("random", random_search, 1)
    es = run("evolution_1plus4", evolution, 2)
    default = cached("cred_search_default", lambda: evaluate((LO + HI) * 0.0))
    for r in (rs, es):
        print(r["method"], r["tuning_objective_median_min_sep_m"], r["evaluation"]["p_violation"],
              r["best_params"], flush=True)
    return {"show": show["name"], "nominal_min_separation_m": rep.min_separation, "d_min_m": d_min,
            "box": BOX, "evaluations_per_method": EVALS, "trials_per_evaluation": M,
            "seed_tune": SEED_TUNE, "seed_eval": SEED_EVAL, "eval_trials": TRIALS,
            "benign_default": default, "random": rs, "evolution": es}


# ------------------------------------------------------------------ fault matrix

BEH = {"comms_loss": "continue", "battery_low": "descend", "gnss_loss": "continue",
       "motor_failure": "ballistic"}


def classify(mc: dict) -> str:
    """Outcome class from the measured violation rates (see docs/FAULT_MATRIX.md)."""
    hi = mc["p_violation_ci95"][1]
    if hi <= 0.05:
        return "no violation observed"
    if mc["p_violation_between_bystanders"] <= 0.05:
        return "violations involve an event drone"
    return "violations among uninvolved drones too"


def exp_faults() -> dict:
    show = load_show("shows/sirb_text_100.json")
    fleet, rep = prep(show)
    d_min = show["limits"]["min_separation_m"]
    ref = sample_reference(fleet, 0.2)
    rows = []

    def add(label, cfg, key):
        mc = cached(f"cred_fault_{key}", lambda: monte_carlo(ref, cfg, d_min, TRIALS, SEED_EVAL,
                                                            workers=WORKERS))
        rows.append({"case": label, "outcome": classify(mc), **mc})
        print(label, round(mc["p_violation"], 3), rows[-1]["outcome"], flush=True)

    for kind, param in (("comms_loss", "comms_loss_fraction"), ("battery_low", "battery_low_fraction")):
        behaviours = ["continue", "hold", "descend", "corridor"]
        for beh in behaviours:
            delays = (0.0,) if beh == "continue" else (0.0, 1.0, 3.0)
            for dl in delays:
                cfg = StressConfig(dt=0.2, rejoin_vmax=10.0, detection_delay_s=dl, **{param: 0.05},
                                   behavior={**BEH, kind: beh})
                add(f"{kind} 5%, {beh}, delay {dl:g} s", cfg, f"{kind}_{beh}_{dl}")
    add("one motor failure (ballistic)", StressConfig(dt=0.2, rejoin_vmax=10.0, motor_failures=1,
                                                      behavior=BEH), "motor1")
    for dl in (0.0, 3.0):
        cfg = StressConfig(dt=0.2, rejoin_vmax=10.0, detection_delay_s=dl, comms_loss_fraction=0.05,
                           battery_low_fraction=0.05, motor_failures=1, gnss_loss_fraction=0.05,
                           gnss_deg_fraction=0.25, gnss_bias_sigma=0.5,
                           behavior={**BEH, "comms_loss": "corridor", "battery_low": "descend",
                                     "gnss_loss": "hold"})
        add(f"combined faults, delay {dl:g} s", cfg, f"combined_{dl}")
    return {"show": show["name"], "trials": TRIALS, "seed": SEED_EVAL, "rows": rows,
            "nominal_min_separation_m": rep.min_separation}


def main():
    which = sys.argv[1]
    fn = {"dt": exp_dt, "search": exp_search, "faults": exp_faults}[which]
    res = fn()
    (ROOT / "results" / f"stress_{which}.json").write_text(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
