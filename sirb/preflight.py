"""Pre-flight consistency validator (M6b).

Motivated by the NTSB Orlando findings (wrong show centre, wrong heading, outdated geofence on the
ground-station laptop, plan file never sent to some aircraft, individual errors going undetected;
see docs/RESEARCH.md section 2.1). It compares what the *safety case declares* with what the
ground station *actually loaded* and re-verifies the plan after any placement transform.

This is a software consistency check on declared inputs. It does not observe the real world: if
the declaration itself is wrong (surveyed centre, heading), nothing here can know.
"""

from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass, field

import numpy as np

from .trajectory import Fleet, Segment, Trajectory
from .verify import verify


def drone_plan_hash(traj: Trajectory) -> str:
    h = hashlib.sha256()
    for s in traj.segments:
        h.update(np.array([s.t0, s.t1], dtype="<f8").tobytes())
        h.update(np.ascontiguousarray(s.coeffs, dtype="<f8").tobytes())
    return h.hexdigest()


def fleet_plan_hashes(fleet: Fleet) -> list[str]:
    return [drone_plan_hash(t) for t in fleet.trajectories]


def transform_fleet(fleet: Fleet, dx: float = 0.0, dy: float = 0.0, rot_deg: float = 0.0,
                    about: tuple[float, float] = (0.0, 0.0)) -> Fleet:
    """Rigid placement transform (rotate about ``about``, then translate), exact on polynomials."""
    th = math.radians(rot_deg)
    Rz = np.array([[math.cos(th), -math.sin(th), 0.0], [math.sin(th), math.cos(th), 0.0],
                   [0.0, 0.0, 1.0]])
    c = np.array([about[0], about[1], 0.0])
    t = np.array([dx, dy, 0.0])
    trs = []
    for tr in fleet.trajectories:
        segs = []
        for s in tr.segments:
            co = s.coeffs @ Rz.T
            co[0] = co[0] + (c - Rz @ c) + t
            segs.append(Segment(s.t0, s.t1, co))
        trs.append(Trajectory(segs))
    return Fleet(trs, dict(fleet.meta))


@dataclass
class Finding:
    level: str      # "FAIL" | "WARN" | "INFO"
    code: str
    message: str
    value: object = None


@dataclass
class PreflightReport:
    ok: bool
    findings: list[Finding] = field(default_factory=list)

    def codes(self, level: str | None = None) -> set[str]:
        return {f.code for f in self.findings if level is None or f.level == level}


def _finite_vec(x, n):
    try:
        a = np.asarray(x, dtype=float)
    except (TypeError, ValueError):
        return False
    return a.shape == (n,) and bool(np.isfinite(a).all())


def _finite_num(x) -> bool:
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def _check_inputs(declared, loaded, wind_forecast_mps, wind_limit_mps) -> list[str]:
    errs: list[str] = []
    for name, d in (("declared", declared), ("loaded", loaded)):
        if not isinstance(d, dict):
            errs.append(f"{name} must be a dict")
            continue
        if not _finite_vec(d.get("center_xy"), 2):
            errs.append(f"{name}.center_xy must be two finite numbers")
        if not _finite_num(d.get("heading_deg")):
            errs.append(f"{name}.heading_deg must be a finite number")
    if wind_forecast_mps is not None and not _finite_num(wind_forecast_mps):
        errs.append("wind_forecast_mps must be a finite number")
    if wind_limit_mps is not None and not (_finite_num(wind_limit_mps) and wind_limit_mps > 0):
        errs.append("wind_limit_mps must be a finite number > 0")
    return errs


def preflight(show: dict, fleet: Fleet, declared: dict, loaded: dict, *,
              center_tol_m: float = 1.0, heading_tol_deg: float = 1.0,
              wind_forecast_mps: float | None = None, wind_limit_mps: float | None = None
              ) -> PreflightReport:
    """Compare ``declared`` (safety case) with ``loaded`` (ground station state).

    declared / loaded keys: center_xy [x, y], heading_deg, geofence_buffer_m,
    geofence {"x": [..], "y": [..]} (optional), n_aircraft (loaded only),
    aircraft_plan_sha256 (loaded only: one hash per aircraft, None = nothing received).
    """
    f: list[Finding] = []
    lim = show["limits"]

    bad_inputs = _check_inputs(declared, loaded, wind_forecast_mps, wind_limit_mps)
    if bad_inputs:
        # A consistency check that cannot read its inputs must fail, not raise and not pass
        # (comparisons with NaN are False, which would silently read as "no mismatch").
        f.extend(Finding("FAIL", "INPUT_INVALID", m) for m in bad_inputs)
        return PreflightReport(ok=False, findings=f)

    dc = np.array(declared["center_xy"], float)
    lc = np.array(loaded["center_xy"], float)
    off = float(np.linalg.norm(dc - lc))
    if off > center_tol_m:
        f.append(Finding("FAIL", "CENTER_MISMATCH",
                         f"loaded show centre is {off:.2f} m from the declared centre", off))
    dh = (loaded["heading_deg"] - declared["heading_deg"] + 180.0) % 360.0 - 180.0
    if abs(dh) > heading_tol_deg:
        f.append(Finding("FAIL", "HEADING_MISMATCH",
                         f"loaded heading differs from declared by {dh:.2f} deg", dh))
    if declared.get("geofence_buffer_m") != loaded.get("geofence_buffer_m"):
        f.append(Finding("FAIL", "GEOFENCE_BUFFER_MISMATCH",
                         f"geofence buffer declared {declared.get('geofence_buffer_m')} m but "
                         f"loaded {loaded.get('geofence_buffer_m')} m"))
    if declared.get("geofence") and loaded.get("geofence") and declared["geofence"] != loaded["geofence"]:
        f.append(Finding("FAIL", "GEOFENCE_MISMATCH", "loaded geofence differs from declared"))

    # Re-verify the plan as it will actually be placed.
    if off > 1e-9 or abs(dh) > 1e-9:
        placed = transform_fleet(fleet, dx=lc[0] - dc[0], dy=lc[1] - dc[1], rot_deg=dh,
                                 about=(dc[0], dc[1]))
        rep = verify(placed, lim)
        bad = {v.kind for v in rep.violations if v.kind in ("geofence", "crowd", "altitude")}
        if bad:
            f.append(Finding("FAIL", "PLACED_PLAN_VIOLATES_LIMITS",
                             "after the loaded placement transform the plan violates: "
                             + ", ".join(sorted(bad)), rep.crowd_margin))
        f.append(Finding("INFO", "CROWD_MARGIN_AFTER_PLACEMENT",
                         f"crowd margin after placement: {rep.crowd_margin:.2f} m "
                         f"(required >= 0)", rep.crowd_margin))

    n_exp = show["n_drones"]
    hashes = loaded.get("aircraft_plan_sha256", [])
    if loaded.get("n_aircraft", len(hashes)) != n_exp or len(hashes) != n_exp:
        f.append(Finding("FAIL", "AIRCRAFT_COUNT_MISMATCH",
                         f"expected {n_exp} aircraft, loaded record covers "
                         f"{loaded.get('n_aircraft', len(hashes))}/{len(hashes)}"))
    want = fleet_plan_hashes(fleet)
    missing = [i for i, h in enumerate(hashes[:n_exp]) if h is None]
    wrong = [i for i, h in enumerate(hashes[:n_exp]) if h is not None and h != want[i]]
    if missing:
        f.append(Finding("FAIL", "PLAN_NOT_RECEIVED",
                         f"{len(missing)} aircraft never confirmed the plan: {missing[:10]}",
                         missing))
    if wrong:
        f.append(Finding("FAIL", "PLAN_HASH_MISMATCH",
                         f"{len(wrong)} aircraft hold a different plan: {wrong[:10]}", wrong))

    if wind_forecast_mps is not None and wind_limit_mps is not None:
        if wind_forecast_mps > wind_limit_mps:
            f.append(Finding("FAIL", "WIND_ABOVE_LIMIT",
                             f"forecast {wind_forecast_mps} m/s exceeds airframe limit "
                             f"{wind_limit_mps} m/s", wind_forecast_mps))
        elif wind_forecast_mps > 0.8 * wind_limit_mps:
            f.append(Finding("WARN", "WIND_NEAR_LIMIT",
                             f"forecast {wind_forecast_mps} m/s is above 80% of the limit",
                             wind_forecast_mps))
    f.append(Finding("INFO", "NOT_CHECKED",
                     "surveyed centre/heading truth, GNSS/RTK status, battery state, comms "
                     "health and spectator positions are outside this check"))
    return PreflightReport(ok=not any(x.level == "FAIL" for x in f), findings=f)


def nominal_declaration(show: dict, fleet: Fleet, center_xy=(0.0, 0.0), heading_deg=0.0,
                        buffer_m: float = 5.0) -> tuple[dict, dict]:
    """Matching declared / loaded dicts for a clean deployment (used by tests and demos)."""
    declared = {"center_xy": list(center_xy), "heading_deg": heading_deg,
                "geofence_buffer_m": buffer_m, "geofence": show["limits"]["geofence"]}
    loaded = dict(declared)
    loaded["aircraft_plan_sha256"] = fleet_plan_hashes(fleet)
    loaded["n_aircraft"] = show["n_drones"]
    return declared, loaded
