"""Sensitivity band over the unsourced relative GNSS error.

No free source gives a measured error between the position estimates of two neighbouring show
drones (docs/GNSS_SOURCED.md). Instead of choosing one value, this module evaluates a grid of
ASSUMED values and reports for which of them the stress target is met. A case whose grid mixes
met and not met cells depends on the parameter and is flagged so in every report.
"""

from __future__ import annotations

from .disturbance import StressConfig
from .stress import monte_carlo, sample_reference

SIGMAS_M = (0.1, 0.25, 0.5, 1.0)
COMMON_FRACTIONS = (0.0, 0.5)
NOTICE = ("The decision depends on this parameter: the relative GNSS error between nearby receivers "
          "is not measured by this project and no free source supplies it. The band below lists "
          "ASSUMED values; read every stressed verdict as conditional on the value used.")


def band(fleet, d_min: float, base: StressConfig, *, sigmas=SIGMAS_M, commons=COMMON_FRACTIONS,
         trials: int = 100, seed: int = 20261016, target: float = 0.05, workers: int = 1) -> dict:
    ref = sample_reference(fleet, base.dt)
    cells = []
    for s in sigmas:
        for c in commons:
            cfg = base.with_(gnss_bias_sigma=s, gnss_common_fraction=c)
            mc = monte_carlo(ref, cfg, d_min, trials, seed, workers=workers)
            hi = mc["p_violation_ci95"][1]
            cells.append({"bias_sigma_m": s, "common_fraction": c, "p_violation": mc["p_violation"],
                          "p_violation_ci95": mc["p_violation_ci95"], "meets_target": hi <= target})
    met = [c["meets_target"] for c in cells]
    depends = any(met) and not all(met)
    return {"label": "SIMULATED; assumed parameter grid; not flight evidence", "trials": trials,
            "seed": seed, "target_upper_bound": target, "cells": cells,
            "depends_on_parameter": depends,
            "notice": NOTICE if depends else NOTICE.replace(
                "The decision depends on this parameter", "Within the grid the decision does not "
                "change with this parameter, which is still unmeasured")}
