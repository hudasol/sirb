"""WP2 study (predictions P36 to P40 in docs/PREDICTIONS.md, committed before this ran).

Greedy repair against a uniform spacing increase and against random repairs of equal budget.
SYNTHETIC fleets, ASSUMED disturbance profile, SIMULATED; nothing here is flight evidence.

    python scripts/repair_study.py        # writes results/repair_study.json, resumable
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from doctor_study import PROFILE, RANDOM_SEEDS, random_show  # noqa: E402

from sirb.disturbance import StressConfig  # noqa: E402
from sirb.envinfo import environment  # noqa: E402
from sirb.margin import scale_show, synthesise  # noqa: E402
from sirb.planner.baseline import plan_baseline  # noqa: E402
from sirb.repair import (  # noqa: E402
    DELAYS_S,
    DILATIONS,
    SPACING_FACTORS,
    build,
    cost,
    final_check,
    repair,
    stage_windows,
)
from sirb.spec import load_show, validate_show  # noqa: E402
from sirb.verify import verify  # noqa: E402

TARGET = 0.05
FINAL_SEED, SYNTH_SEED, RANDOM_SEED = 7003, 7004, 7005
N_RANDOM = 10
BUDGET_S = 300.0
SEARCH_TRIALS = 40
MAX_MOVES = 8
CACHE = Path("results/.cache")
CFG = StressConfig(**PROFILE)


def meets(f: dict) -> bool:
    return (f["exact_verdict"] == "PASS" and f["interval_verdict"] != "FAIL"
            and f["p_violation_ci95"][1] <= TARGET)


def random_repair(show, base_fleet, info, types, rng):
    sw = [w for w in stage_windows(show, info) if w[0] == "transition"]
    n, S = show["n_drones"], len(show["stages"])
    for _ in range(20):
        fac, mv = {}, []
        for ty in types:
            if ty == "stage_spacing":
                k = int(rng.integers(0, S))
                fac[k] = fac.get(k, 1.0) * float(rng.choice(SPACING_FACTORS))
            elif ty == "start_delay":
                mv.append({"type": ty, "drone": int(rng.integers(0, n)),
                           "delay_s": float(rng.choice(DELAYS_S))})
            else:
                w = sw[int(rng.integers(0, len(sw)))]
                mv.append({"type": ty, "drone": int(rng.integers(0, n)),
                           "t_ref": (w[2] + w[3]) / 2, "factor": float(rng.choice(DILATIONS))})
        s2, f2, _ = build(show, fac, mv)
        if not validate_show(s2, check_targets=False) and verify(f2, s2["limits"]).ok:
            return s2, f2
    return None, None


def run_case(name: str, show: dict) -> dict:
    t = time.time()
    base, info = plan_baseline(show)
    before = final_check(show, base, CFG, 200, FINAL_SEED)
    out = {"name": name, "n_drones": show["n_drones"], "before": before}
    if before["p_violation_ci95"][1] <= TARGET:
        out["included"] = False
        return out
    out["included"] = True
    r, s2, f2 = repair(show, CFG, target=TARGET, final_seed=FINAL_SEED, time_budget_s=BUDGET_S,
                       search_trials=SEARCH_TRIALS, max_moves=MAX_MOVES)
    out["greedy"] = {k: r[k] for k in ("status", "n_moves", "moves", "after", "cost", "wall_s")}
    out["greedy"]["meets_target"] = meets(r["after"])
    # uniform spacing increase
    u = synthesise(show, CFG, target=TARGET, trials=200, seed=SYNTH_SEED, tol=0.02)
    if u["factor"] is None:
        out["uniform"] = {"status": u["status"], "meets_target": False}
    else:
        su = scale_show(show, u["factor"])
        fu, _ = plan_baseline(su)
        fin = final_check(su, fu, CFG, 200, FINAL_SEED)
        out["uniform"] = {"factor": u["factor"], "after": fin, "cost": cost(base, fu),
                          "meets_target": meets(fin)}
    # random repairs of equal budget
    types = [m["move"]["type"] for m in r["moves"]]
    rr = []
    if types:
        rng = np.random.default_rng(RANDOM_SEED)
        for _ in range(N_RANDOM):
            s3, f3 = random_repair(show, base, info, types, rng)
            if f3 is None:
                rr.append({"valid": False, "meets_target": False})
                continue
            fin = final_check(s3, f3, CFG, 200, FINAL_SEED)
            rr.append({"valid": True, "after": {k: fin[k] for k in ("p_violation", "p_violation_ci95")},
                       "cost": cost(base, f3), "meets_target": meets(fin)})
    out["random"] = rr
    out["wall_s"] = time.time() - t
    return out


def main() -> None:
    CACHE.mkdir(parents=True, exist_ok=True)
    cases = [(f"random_{s}", random_show(s)) for s in RANDOM_SEEDS]
    cases += [("sirb_text_100", load_show("shows/sirb_text_100.json")),
              ("gallery_120", load_show("shows/gallery_120.json"))]
    done = []
    for name, show in cases:
        f = CACHE / f"repair_{name}.json"
        if f.exists():
            done.append(json.loads(f.read_text()))
            continue
        res = run_case(name, show)
        f.write_text(json.dumps(res, default=float))
        done.append(json.loads(f.read_text()))
        print(name, res.get("greedy", {}).get("status"), flush=True)
    inc = [c for c in done if c["included"]]
    g = [c for c in inc if c["greedy"]["meets_target"]]
    both = [c for c in g if c["uniform"]["meets_target"]]
    moves = [m["move"]["type"] for c in inc for m in c["greedy"]["moves"]]
    rr = [x["meets_target"] for c in g for x in c["random"]]
    v2 = [c["greedy"]["after"]["interval_verdict"] for c in inc]
    res = {
        "label": "SIMULATED; synthetic fleets; assumed disturbance profile; not flight evidence",
        "profile": PROFILE, "target_upper_bound": TARGET, "seeds": {"final": FINAL_SEED,
        "uniform_synthesis": SYNTH_SEED, "random_repair": RANDOM_SEED}, "cases": done,
        "n_cases": len(done), "n_included": len(inc), "n_greedy_success": len(g),
        "P36": {"fraction": len(g) / len(inc) if inc else None, "threshold": 0.8},
        "P37": {"n_both": len(both),
                "fraction_greedy_less_path": (sum(c["greedy"]["cost"]["extra_path_length_m"]
                                                  < c["uniform"]["cost"]["extra_path_length_m"]
                                                  for c in both) / len(both)) if both else None,
                "threshold": 0.7},
        "P38": {"random_draws": len(rr), "fraction_success": (sum(rr) / len(rr)) if rr else None,
                "threshold": 0.2},
        "P39": {"interval_verdicts": {k: v2.count(k) for k in set(v2)},
                "fraction_pass": (v2.count("PASS") / len(v2)) if v2 else None, "threshold": 0.9},
        "P40": {"moves": {k: moves.count(k) for k in set(moves)},
                "spacing_fraction": (moves.count("stage_spacing") / len(moves)) if moves else None,
                "threshold": 0.8},
        "environment": environment({"final": FINAL_SEED, "random_fleet_seeds": RANDOM_SEEDS}),
    }
    Path("results/repair_study.json").write_text(json.dumps(res, indent=1, default=float))
    print(json.dumps({k: res[k] for k in ("n_cases", "n_included", "n_greedy_success", "P36",
                                          "P37", "P38", "P39", "P40")}, indent=1))


if __name__ == "__main__":
    main()
