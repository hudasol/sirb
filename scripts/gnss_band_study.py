"""WP5 study (predictions P43 and P44 in docs/PREDICTIONS.md, committed before this ran).

    python scripts/gnss_band_study.py     # writes results/gnss_band_study.json
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from doctor_study import PROFILE, RANDOM_SEEDS, random_show  # noqa: E402

from sirb.disturbance import StressConfig  # noqa: E402
from sirb.envinfo import environment  # noqa: E402
from sirb.gnss_band import band  # noqa: E402
from sirb.planner.baseline import plan_baseline  # noqa: E402
from sirb.spec import load_show  # noqa: E402

SEED = 20261016
BASE = StressConfig(**{**PROFILE, "gnss_bias_sigma": 0.5})


def main() -> None:
    cases = [(n, load_show(f"shows/{n}.json")) for n in ("sirb_text_100", "gallery_120", "torus_300")]
    cases += [(f"random_{s}", random_show(s)) for s in RANDOM_SEEDS]
    out = []
    for name, show in cases:
        fleet, _ = plan_baseline(show)
        b = band(fleet, show["limits"]["min_separation_m"], BASE, seed=SEED)
        cell = {(c["bias_sigma_m"], c["common_fraction"]): c["meets_target"] for c in b["cells"]}
        out.append({"name": name, "n_drones": show["n_drones"], "band": b,
                    "common_flips_at_0.5": cell[(0.5, 0.0)] != cell[(0.5, 0.5)]})
        print(name, b["depends_on_parameter"], flush=True)
    res = {"label": "SIMULATED; synthetic fleets; assumed parameter grid; not flight evidence",
           "profile": PROFILE, "cases": out,
           "P43": {"fraction_depends": sum(c["band"]["depends_on_parameter"] for c in out) / len(out),
                   "threshold": 0.8},
           "P44": {"fraction_flips": sum(c["common_flips_at_0.5"] for c in out) / len(out),
                   "threshold": 0.3},
           "environment": environment({"stress_seed": SEED, "random_fleet_seeds": RANDOM_SEEDS})}
    Path("results/gnss_band_study.json").write_text(json.dumps(res, indent=1))
    print(res["P43"], res["P44"])


if __name__ == "__main__":
    main()
