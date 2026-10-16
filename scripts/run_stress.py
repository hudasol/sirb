"""Run the M5/M6 experiments. Seeds are fixed; trial counts are arguments.

    python scripts/run_stress.py sweeps      --trials 200
    python scripts/run_stress.py spacing     --trials 300
    python scripts/run_stress.py scaling
    python scripts/run_stress.py contingency --trials 200

Writes results/stress_<experiment>.json. Disturbance parameters: see sirb/disturbance.py and
docs/METHODS.md (assumptions are labeled there).
"""

from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path

from sirb.disturbance import StressConfig
from sirb.planner.baseline import plan_baseline
from sirb.spec import load_show, new_show
from sirb.stress import monte_carlo, sample_reference, sweep
from sirb.verify import verify

SEED = 20261011
DT = 0.2
WORKERS = int(os.environ.get("SIRB_WORKERS", "2"))   # numbers do not depend on it (per trial seeds)


def family_show(spacing: float, n: int = 100, big: bool = False) -> dict:
    grid = {"type": "launch_grid", "spacing_m": spacing, "center": [0.0, -85.0, 0.0]}
    over = {"limits": {"geofence": {"x": [-200.0, 200.0], "y": [-200.0, 150.0]}}} if big else {}
    return new_show(f"family s={spacing} n={n}", n, [
        {"name": "launch", "shape": grid, "hold_s": 2},
        {"name": "orb", "shape": {"type": "sphere", "spacing_m": spacing, "center": [0, 0, 60]},
         "hold_s": 4},
        {"name": "wall", "shape": {"type": "wall", "aspect": 1.6, "spacing_m": spacing,
                                    "center": [0, 0, 60]}, "hold_s": 4},
        {"name": "land", "shape": grid, "hold_s": 2},
    ], seed=7, **over)


CACHE = Path("results/.cache")


def cached(key: str, fn):
    """Checkpoint each unit of work so a killed run resumes (results are deterministic, so a
    cached unit is identical to a recomputed one). Cache lives in results/.cache (gitignored)."""
    CACHE.mkdir(parents=True, exist_ok=True)
    f = CACHE / (key.replace("/", "_").replace(" ", "_") + ".json")
    if f.exists():
        return json.loads(f.read_text())
    val = fn()
    f.write_text(json.dumps(val))
    return json.loads(json.dumps(val))


def plan_and_ref(show: dict):
    fleet, _ = plan_baseline(show)
    rep = verify(fleet, show["limits"])
    return fleet, rep, sample_reference(fleet, DT)


def exp_sweeps(trials: int) -> dict:
    show = load_show("shows/sirb_text_100.json")
    fleet, rep, ref = plan_and_ref(show)
    d_min = show["limits"]["min_separation_m"]
    base = StressConfig(dt=DT)
    out = {"show": show["name"], "nominal_min_separation_m": rep.min_separation,
           "verified_ok": rep.ok, "trials": trials, "d_min_m": d_min, "dt": DT, "sweeps": {}}
    plan = {
        "wind_w20_mps": ("w20", [0, 3, 6, 9, 12, 15, 20], {}),
        "gnss_bias_sigma_m_25pct_degraded": ("gnss_bias_sigma",
                                              [0.0, 0.1, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0],
                                              {"gnss_deg_fraction": 0.25}),
        "gnss_degraded_fraction_bias0.5m": ("gnss_deg_fraction", [0.0, 0.05, 0.1, 0.25, 0.5, 1.0],
                                             {"gnss_bias_sigma": 0.5}),
        "gnss_dropout_fraction_10s": ("gnss_loss_fraction", [0.0, 0.02, 0.05, 0.1, 0.25], {}),
        "battery_low_fraction_descend": ("battery_low_fraction", [0.0, 0.01, 0.02, 0.05, 0.1], {}),
        "motor_failures_ballistic": ("motor_failures", [0, 1, 2, 4, 8], {}),
    }
    for name, (param, values, extra) in plan.items():
        t = time.time()
        out["sweeps"][name] = cached(f"sweeps_{trials}_{name}", lambda: sweep(
            ref, base.with_(**extra), param, values, d_min, trials, SEED, WORKERS))
        print(f"{name}: {time.time() - t:.0f}s", flush=True)
    return out


def exp_spacing(trials: int) -> dict:
    scenario = {"w20": 6.0, "gnss_deg_fraction": 0.25, "gnss_bias_sigma": 0.5}
    out = {"scenario": scenario, "trials": trials, "dt": DT, "rows": []}
    for s in (3.0, 3.5, 4.0, 5.0, 6.0, 8.0):
        def one(s=s):
            show = family_show(s)
            fleet, rep, ref = plan_and_ref(show)
            d_min = show["limits"]["min_separation_m"]
            mc = monte_carlo(ref, StressConfig(dt=DT, **scenario), d_min, trials, SEED,
                             workers=WORKERS)
            return {"formation_spacing_m": s, "nominal_min_separation_m": rep.min_separation,
                    "verified_ok": rep.ok, **mc}
        row = cached(f"spacing_{trials}_{s}", one)
        out["rows"].append(row)
        print(f"spacing {s}: nominal {row['nominal_min_separation_m']:.2f} m, "
              f"P={row['p_violation']:.3f} hi={row['p_violation_ci95'][1]:.3f}", flush=True)
    return out


def exp_scaling() -> dict:
    out = {"rows": [], "scenario": {"gnss_deg_fraction": 0.25, "gnss_bias_sigma": 0.25}}
    for n in (20, 50, 100, 200, 500, 1000, 2000):
        def one(n=n):
            show = family_show(3.2, n, big=True)
            t = time.time()
            fleet, _ = plan_baseline(show)
            plan_s = time.time() - t
            rep = verify(fleet, show["limits"])
            ref = sample_reference(fleet, DT)
            trials = 20 if n <= 500 else 6
            mc = monte_carlo(ref, StressConfig(dt=DT, **out["scenario"]), 2.0, trials, SEED,
                             workers=WORKERS)
            return {"n": n, "plan_s": plan_s, "verify_s": rep.runtime_s,
                    "verified_ok": rep.ok, "nominal_min_separation_m": rep.min_separation,
                    "pairs_checked": rep.n_pairs_checked, "stress_trials": trials,
                    "stress_mean_trial_cpu_s": mc["mean_trial_cpu_s"],
                    "p_violation": mc["p_violation"]}
        out["rows"].append(cached(f"scaling_{n}", one))
        print(out["rows"][-1], flush=True)
    return out


def exp_contingency(trials: int) -> dict:
    show = load_show("shows/sirb_text_100.json")
    fleet, rep, ref = plan_and_ref(show)
    d_min = show["limits"]["min_separation_m"]
    out = {"show": show["name"], "trials": trials, "rejoin_vmax_mps": 10.0, "rows": []}
    cases = [
        ("comms_loss 5%", "comms_loss_fraction", 0.05, "comms_loss",
         ["continue", "hold", "descend", "corridor"], {}),
        ("battery_low 5%", "battery_low_fraction", 0.05, "battery_low",
         ["hold", "descend", "corridor"], {}),
        ("gnss_dropout 10% x 10 s", "gnss_loss_fraction", 0.10, "gnss_loss",
         ["continue", "hold", "descend", "corridor"], {}),
        ("gnss_dropout 10% x 10 s, wind 9 m/s", "gnss_loss_fraction", 0.10, "gnss_loss",
         ["continue", "hold", "descend", "corridor"], {"w20": 9.0}),
    ]
    for label, param, val, kind, behaviours, extra in cases:
        for beh in behaviours:
            cfg = StressConfig(dt=DT, rejoin_vmax=10.0, **{param: val}, **extra).with_(
                behavior={"comms_loss": "continue", "battery_low": "descend",
                          "gnss_loss": "continue", "motor_failure": "ballistic", kind: beh})
            mc = cached(f"contingency_{trials}_{label}_{beh}", lambda: monte_carlo(
                ref, cfg, d_min, trials, SEED, workers=WORKERS))
            out["rows"].append({"case": label, "behavior": beh, **mc})
            print(f"{label:38s} {beh:9s} P={mc['p_violation']:.3f} "
                  f"event={mc['p_violation_involving_event_drone']:.3f} "
                  f"bystander={mc['p_violation_between_bystanders']:.3f}", flush=True)
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("exp", choices=["sweeps", "spacing", "scaling", "contingency"])
    ap.add_argument("--trials", type=int, default=100)
    a = ap.parse_args()
    res = {"sweeps": lambda: exp_sweeps(a.trials), "spacing": lambda: exp_spacing(a.trials),
           "scaling": exp_scaling, "contingency": lambda: exp_contingency(a.trials)}[a.exp]()
    Path("results").mkdir(exist_ok=True)
    Path(f"results/stress_{a.exp}.json").write_text(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
