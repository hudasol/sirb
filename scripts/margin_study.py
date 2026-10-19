"""Stream C (phase 6): how the separation needed for a 5% Wilson upper bound scales with n and
GNSS bias sigma. Planning aid study; everything is a property of the simulated model.

Show family: ``run_stress.family_show`` (launch grid, sphere, wall, land) with a widened geofence,
formation spacing 4.0 m before scaling. Scenario: mean wind 6 m/s at 20 ft, 25% of drones
degraded, bias sigma as listed; every other value is the StressConfig default (ASSUMED). The last
rows use the parameter files ``shows/gnss_faa_sps_standalone.json`` and
``shows/gnss_relative_literature.json`` (docs/GNSS_SOURCED.md).
Fixed seeds; each evaluated factor is cached in results/.cache so an interrupted run resumes.

    SIRB_WORKERS=8 python scripts/margin_study.py      # writes results/margin.json
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from run_stress import DT, WORKERS, cached, family_show  # noqa: E402

from sirb.disturbance import StressConfig  # noqa: E402
from sirb.gnss_params import load, to_stress_config  # noqa: E402
from sirb.margin import MODEL_DEPENDENCE, evaluate, synthesise, wilson_floor  # noqa: E402

TARGET = 0.05
TRIALS = 200
SEED = 20261016
TOL = 0.02
BASE = {"w20": 6.0, "gnss_deg_fraction": 0.25}
CASES = ([{"n": 100, "bias_sigma": s} for s in (0.0, 0.25, 0.5, 1.0)]
         + [{"n": n, "bias_sigma": 0.5} for n in (50, 200, 400)]
         + [{"n": 100, "gnss_file": "shows/gnss_faa_sps_standalone.json"},
            {"n": 100, "gnss_file": "shows/gnss_relative_literature.json"}])


def config(case: dict) -> StressConfig:
    if "gnss_file" in case:
        return to_stress_config(load(case["gnss_file"]), base=StressConfig(dt=DT, w20=BASE["w20"]))
    return StressConfig(dt=DT, **BASE, gnss_bias_sigma=case["bias_sigma"])


def run_case(case: dict) -> dict:
    show = family_show(4.0, n=case["n"], big=True)
    cfg = config(case)
    key = "_".join(f"{k}{v}" for k, v in case.items()).replace("/", "_")

    def ev(f):
        return cached(f"margin_{key}_f{f:.6f}",
                      lambda: evaluate(show, f, cfg, TRIALS, SEED, workers=WORKERS))

    t = time.time()
    r = synthesise(show, cfg, target=TARGET, trials=TRIALS, seed=SEED, tol=TOL, evaluator=ev)
    r["case"] = case
    r["base_spacing_m"] = 4.0
    r["required_spacing_m"] = None if r["factor"] is None else 4.0 * r["factor"]
    r["wall_s"] = time.time() - t
    print(case, r["status"], r["required_nominal_min_separation_m"], flush=True)
    return r


def main():
    rows = [run_case(c) for c in CASES]
    Path("results/margin.json").write_text(json.dumps({
        "target": TARGET, "trials": TRIALS, "seed": SEED, "tol": TOL, "dt": DT, "scenario": BASE,
        "wilson_floor": wilson_floor(TRIALS), "model_dependence": MODEL_DEPENDENCE,
        "note": "family_show(4.0, n, big=True) scaled by factor; baseline planner; exact verifier",
        "rows": rows}, indent=1))


if __name__ == "__main__":
    main()
