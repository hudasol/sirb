"""Reproducible verification certificate (no timestamps, JSON-serialisable)."""

from __future__ import annotations

import copy
import math
from dataclasses import asdict

import numpy as np

from sirb import SAFETY_STATEMENT, __version__
from sirb.verify import VerifyReport, verify

CERT_VERSION = 1
STATEMENT = "simulation-validated only; not flight-certified; no regulatory compliance claim"

METHOD = (
    "Exact verification of piecewise-polynomial trajectories. Pairwise separation: for each "
    "pair of drones, time is split at all segment breakpoints (a drone is held at its start "
    "position before its first segment and at its final position after its last); on each "
    "sub-interval the squared centre distance is a scalar polynomial whose minimum is at an "
    "endpoint or a real root of its derivative, found by companion-matrix eigenvalues, "
    "Newton-polished and evaluated directly. Exact axis-aligned segment bounding boxes (from "
    "derivative roots) discard pairs that provably cannot come within the search radius. "
    "Speed, acceleration, jerk, altitude, geofence and crowd limits are checked via the "
    "extrema of the corresponding scalar polynomials (derivative roots) per segment. "
    "Structural checks cover time contiguity, position, velocity and acceleration continuity "
    "and finite coefficients. No time sampling is used."
)

GUARANTEES = [
    "Under the declared model (polynomial paths exactly as written, drones as points whose "
    "required separation is min_separation_m centre to centre) the reported minimum "
    "separation over all time is exact up to floating-point and root-finding tolerance, for "
    "every pair whose minimum is below the search radius.",
    "Dynamic limits (speed, climb/descent rate, acceleration, jerk, altitude, geofence, "
    "crowd distance) hold for the polynomials as written, within the stated tolerance policy.",
    "Segments are contiguous in time with position, velocity and acceleration continuity "
    "within the stated tolerances (an acceleration step would mean unbounded jerk).",
]

NOT_GUARANTEED = [
    "Real aircraft tracking of the commanded polynomials (controller error, latency)",
    "Wind and other weather disturbances",
    "GNSS position error, jamming or spoofing",
    "Aerodynamic interaction such as downwash",
    "Vehicle size, shape or battery behaviour beyond the declared limits",
    "Failures, comms loss and contingency behaviour",
    "Regulatory, airworthiness or operational approval properties of any kind",
]


def _jsonable(x):
    if isinstance(x, dict):
        return {str(k): _jsonable(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_jsonable(v) for v in x]
    if isinstance(x, np.ndarray):
        return _jsonable(x.tolist())
    if isinstance(x, (np.bool_, bool)):
        return bool(x)
    if isinstance(x, (np.integer, int)):
        return int(x)
    if isinstance(x, (np.floating, float)):
        f = float(x)
        return f if math.isfinite(f) else None
    return x


# Limit families the verifier checks when present. A limit that is absent is silently skipped by
# ``verify`` (only min_separation_m is mandatory), so a certificate must say so rather than PASS.
CHECKED_LIMITS = ("vmax_h", "vmax_up", "vmax_down", "amax", "jmax", "z_min", "z_max",
                  "geofence", "crowd")


def unchecked_limits(limits: dict) -> list[str]:
    return [k for k in CHECKED_LIMITS if limits.get(k) is None]


def verdict_for(ok: bool, limits: dict) -> str:
    """PASS only if nothing failed and every limit family was actually supplied."""
    if not ok:
        return "FAIL"
    return "PASS" if not unchecked_limits(limits) else "INCOMPLETE"


def make_certificate(report: VerifyReport, fleet, limits: dict, *, extra: dict | None = None) -> dict:
    results = asdict(report)
    results.pop("runtime_s", None)   # not reproducible; deliberately excluded
    results["violations"] = results["violations"][:200]
    cert = {
        "certificate_version": CERT_VERSION,
        "tool": "sirb.verify",
        "sirb_version": __version__,
        "fleet_sha256": fleet.content_hash(),
        "n_drones": int(fleet.n),
        "limits": copy.deepcopy(limits),
        "results": results,
        "verdict": verdict_for(report.ok, limits),
        "unchecked_limits": unchecked_limits(limits),
        "method": METHOD,
        "guarantees": list(GUARANTEES),
        "not_guaranteed": list(NOT_GUARANTEED),
        "statement": STATEMENT,
        "safety_statement": SAFETY_STATEMENT,
    }
    if extra:
        cert["extra"] = copy.deepcopy(extra)
    return _jsonable(cert)


_NUMERIC = ("min_separation", "max_speed_h", "max_speed_up", "max_speed_down", "max_accel",
            "max_jerk", "min_z", "max_z", "geofence_margin", "crowd_margin")


def verify_certificate(cert: dict, fleet) -> bool:
    """True iff the fleet hash matches and an independent recomputation agrees."""
    try:
        if cert.get("tool") != "sirb.verify" or cert.get("certificate_version") != CERT_VERSION:
            return False
        if cert["fleet_sha256"] != fleet.content_hash() or cert["n_drones"] != fleet.n:
            return False
        res = cert["results"]
        tol = res["tolerances"]
        rep = verify(fleet, cert["limits"], search_radius=tol["search_radius"], tol=tol["tol"])
        if verdict_for(rep.ok, cert["limits"]) != cert["verdict"]:
            return False
        if unchecked_limits(cert["limits"]) != cert.get("unchecked_limits"):
            return False
        new = _jsonable(asdict(rep))
        for k in _NUMERIC:
            a, b = res.get(k), new.get(k)
            if (a is None) != (b is None):
                return False
            if a is not None and not math.isclose(a, b, rel_tol=1e-9, abs_tol=1e-9):
                return False
        if res.get("violation_counts") != new.get("violation_counts"):
            return False
        return res.get("min_separation_is_lower_bound") == new.get("min_separation_is_lower_bound")
    except Exception:  # noqa: BLE001
        return False
