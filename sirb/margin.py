"""Margin synthesis: the nominal separation a show needs for a stated simulated risk target.

PLANNING AID ONLY. The answer is a property of the simulated model (tracking loop, wind, GNSS
error model and the parameter file supplied), not of real aircraft. It changes when any assumption
changes; docs/MARGIN.md shows by how much for GNSS bias sigma and fleet size.

Method. The show is scaled by a factor f applied to every shape's ``spacing_m`` (or ``size``) and
to the launch grid pitch; centres stay where they are. For each f the baseline planner plans, the
exact verifier checks the plan, and the Monte Carlo stress simulator estimates P(separation
violation) with a 95% Wilson interval. Every f uses the same trial seeds (common random numbers),
so neighbouring factors are paired. The search brackets the smallest f whose Wilson upper bound
is at most ``target`` (growing f by ``grow`` until one qualifies), bisects the bracket to a
relative width ``tol``, and then checks that the next larger factor also qualifies (a guard
against one lucky row, as in ``sirb.stress.required_value``); if it does not, the search
continues upwards. The result is the upper end of the final bracket, so it is conservative to
within ``tol``. ``required_nominal_min_separation_m`` is the exact verified minimum separation of
the plan at that factor.
"""

from __future__ import annotations

import copy
import math

from .disturbance import StressConfig
from .planner.baseline import plan_baseline
from .stress import monte_carlo, sample_reference
from .verify import verify

MODEL_DEPENDENCE = (
    "Simulated, model dependent planning aid. The required separation holds only for the "
    "simulated tracking loop, wind model, GNSS error model and parameter values used; GNSS errors "
    "are independent between drones in this model. Not a safety case and not flight evidence.")


def scale_show(show: dict, factor: float) -> dict:
    """Copy of ``show`` with every formation enlarged by ``factor`` about its centre."""
    out = copy.deepcopy(show)
    for st in out["stages"]:
        # a fresh dict per stage: stages may share one shape object, which must be scaled once
        st["shape"] = {k: (v * factor if k in ("spacing_m", "size") and v is not None else v)
                       for k, v in st["shape"].items()}
    out["name"] = f"{show['name']} x{factor:.4f}"
    return out


def evaluate(show: dict, factor: float, cfg: StressConfig, trials: int, seed: int,
             workers: int = 1) -> dict:
    s = scale_show(show, factor)
    fleet, _ = plan_baseline(s)
    rep = verify(fleet, s["limits"])
    row = {"factor": factor, "verified_ok": rep.ok, "nominal_min_separation_m": rep.min_separation,
           "violation_counts": rep.violation_counts}
    if not rep.ok:
        return row
    mc = monte_carlo(sample_reference(fleet, cfg.dt), cfg, s["limits"]["min_separation_m"],
                     trials, seed, workers=workers)
    row.update({k: mc[k] for k in ("p_violation", "p_violation_ci95", "trials")})
    row["median_min_separation_m"] = mc["min_separation_quantiles"]["0.5"]
    return row


def synthesise(show: dict, cfg: StressConfig, *, target: float = 0.05, trials: int = 200,
               seed: int = 20261016, workers: int = 1, tol: float = 0.02, grow: float = 1.25,
               max_factor: float = 4.0, evaluator=None) -> dict:
    """Smallest formation scale whose simulated violation risk has Wilson upper bound <= target.

    ``evaluator(factor) -> row`` can replace ``evaluate`` (used for caching and in tests).
    Returns a dict with the chosen factor, its verified nominal minimum separation, every
    evaluated row, and the model dependence statement. ``factor`` is None if no factor up to
    ``max_factor`` qualified or the plan stopped verifying first (the reason is in ``status``).
    """
    if not 0 < target < 1:
        raise ValueError("target must be in (0, 1)")
    ev = evaluator or (lambda f: evaluate(show, f, cfg, trials, seed, workers))
    rows: dict = {}

    def get(f):
        f = round(f, 6)
        if f not in rows:
            rows[f] = ev(f)
        return rows[f]

    def ok(f):
        r = get(f)
        return r["verified_ok"] and r["p_violation_ci95"][1] <= target

    d_min = show["limits"]["min_separation_m"]
    r1 = get(1.0)
    if not r1["verified_ok"]:
        return _result(None, rows, target, d_min, "the unscaled plan does not verify")
    # Lower end: a factor whose nominal separation is at d_min cannot meet any useful target.
    lo = max(d_min / r1["nominal_min_separation_m"], 1e-3)
    hi = 1.0
    while not ok(hi):
        if not get(hi)["verified_ok"]:
            return _result(None, rows, target, d_min, f"plan stops verifying at factor {hi:.4f}")
        lo, hi = hi, hi * grow
        if hi > max_factor:
            return _result(None, rows, target, d_min, f"no factor up to {max_factor} qualified")
    while True:
        while hi - lo > tol * hi:
            mid = 0.5 * (lo + hi)
            if ok(mid):
                hi = mid
            else:
                lo = mid
        nxt = hi * (1 + tol)
        if ok(nxt):
            break
        if not get(nxt)["verified_ok"]:
            return _result(None, rows, target, d_min, f"plan stops verifying at factor {nxt:.4f}")
        lo, hi = nxt, nxt * grow   # the guard failed: one lucky row; search above it
        while not ok(hi):
            lo, hi = hi, hi * grow
            if hi > max_factor:
                return _result(None, rows, target, d_min, f"no factor up to {max_factor} qualified")
    return _result(hi, rows, target, d_min, "ok")


def _result(f, rows, target, d_min, status):
    chosen = rows.get(round(f, 6)) if f is not None else None
    req = chosen["nominal_min_separation_m"] if chosen else None
    return {"status": status, "target": target, "factor": f,
            "required_nominal_min_separation_m": req,
            "required_margin_m": None if req is None else req - d_min,
            "d_min": d_min, "evaluations": [rows[k] for k in sorted(rows)],
            "model_dependence": MODEL_DEPENDENCE,
            "layer": "simulated (planning aid); not guaranteed"}


def wilson_floor(trials: int) -> float:
    """Best attainable Wilson upper bound (zero violations) for ``trials``: 3.84 / (N + 3.84)."""
    z2 = 1.96 ** 2
    return z2 / (trials + z2) if trials > 0 else math.inf
