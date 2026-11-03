"""Optional downwash rule: more vertical than horizontal clearance between drones.

Default off. A rule is an ellipsoid with horizontal radius ``d_h`` (the declared minimum
separation) and vertical radius ``d_v``; the check is exact (``sirb.clearance`` scales the z
coefficients so the ordinary exact verifier applies). The vertical radius is a parameter with
provenance, read from a file such as ``shows/downwash_assumed.json``; the shipped value is
ASSUMED, scaled from measurements on a much smaller vehicle, and is not a statement about any
real aircraft.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

from .clearance import scale_z
from .fileio import read_text_limited
from .verify import pair_minima


@dataclass(frozen=True)
class DownwashRule:
    d_h: float
    d_v: float
    assumed: bool = True
    source: str = ""

    def __post_init__(self):
        if not (self.d_h > 0 and self.d_v >= self.d_h):
            raise ValueError("need 0 < d_h <= d_v")


def load_rule(path: str | Path, d_h: float, d_v: float | None = None) -> DownwashRule:
    d = json.loads(read_text_limited(path))
    p = d["parameters"]["vertical_radius_m"]
    return DownwashRule(d_h, float(p["value"] if d_v is None else d_v), bool(p.get("assumed", True)),
                        d["provenance"]["source_name"])


def violating_pairs(fleet, rule: DownwashRule) -> list[dict]:
    """Pairs whose ellipsoidal minimum is below ``d_h`` at some time, worst first."""
    res, _ = pair_minima(scale_z(fleet, rule.d_h / rule.d_v), rule.d_h)
    rows = [{"pair": [i, j], "ellipsoidal_min_m": d, "time_s": t}
            for (i, j), (d, t) in res.items()]
    return sorted(rows, key=lambda r: r["ellipsoidal_min_m"])


def report(fleet, rule: DownwashRule) -> dict:
    v = violating_pairs(fleet, rule)
    return {"rule": {"d_h_m": rule.d_h, "d_v_m": rule.d_v, "assumed": rule.assumed,
                     "source": rule.source},
            "verdict": "FAIL" if v else "PASS", "violating_pairs": len(v),
            "worst": v[:10]}
