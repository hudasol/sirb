"""WP4 study (predictions P41 and P42 in docs/PREDICTIONS.md, committed before this ran).

Counts drone pairs of the four demo shows that would violate an ASSUMED ellipsoidal downwash rule
(horizontal radius = the declared 2.0 m, several vertical radii). Planning only; not flight
evidence and not a statement about any real aircraft.

    python scripts/downwash_study.py     # writes results/downwash_study.json
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from sirb.downwash import load_rule, report  # noqa: E402
from sirb.envinfo import environment  # noqa: E402
from sirb.planner.baseline import plan_baseline  # noqa: E402
from sirb.spec import load_show  # noqa: E402

DEMOS = {"gallery_120": "shows/gallery_120.json", "oryx_500": "shows/oryx_500.json",
         "torus_300": "shows/torus_300.json", "sirb_text_100": "shows/sirb_text_100.json"}
RULE = "shows/downwash_assumed.json"
D_V = [2.0, 3.0, 4.56, 5.0]


def main() -> None:
    rows = {}
    for name, path in DEMOS.items():
        show = load_show(path)
        fleet, _ = plan_baseline(show)
        d_h = show["limits"]["min_separation_m"]
        rows[name] = {"n_drones": show["n_drones"], "fleet_sha256": fleet.content_hash(),
                      "violating_pairs": {}}
        for dv in D_V:
            r = report(fleet, load_rule(RULE, d_h, dv))
            rows[name]["violating_pairs"][str(dv)] = r["violating_pairs"]
        print(name, rows[name]["violating_pairs"], flush=True)
    c = {k: v["violating_pairs"] for k, v in rows.items()}
    res = {
        "label": "ASSUMED rule scaled from a smaller vehicle; planning only; not flight evidence",
        "rule_file": RULE, "d_h_m": 2.0, "vertical_radii_m": D_V, "shows": rows,
        "P41": {"all_have_violation_at_4.56": all(v["4.56"] > 0 for v in c.values()),
                "none_at_2.0": all(v["2.0"] == 0 for v in c.values())},
        "P42": {"shows_with_ratio_ge_5": sum(
                    v["4.56"] >= 5 * v["3.0"] and v["4.56"] > 0 for v in c.values()),
                "of": len(c)},
        "environment": environment({}),
    }
    Path("results/downwash_study.json").write_text(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
