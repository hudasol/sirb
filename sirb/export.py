"""M8: file export of a verified plan with its safety report.

Files only. There is no uplink, MAVLink, radio or flight-controller code in Sirb. The bundle is
labeled "simulation-validated only" and must not be read as an approval of any kind.
"""

from __future__ import annotations

import json
from pathlib import Path

from .certificate import make_certificate, verify_certificate
from .trajectory import Fleet
from .verify import verify

LABEL = "simulation-validated only"
FORMAT = "sirb-export-1"


def build_export(show: dict, fleet: Fleet, stress_summary: dict | None = None) -> dict:
    from . import SAFETY_STATEMENT
    rep = verify(fleet, show["limits"])
    cert = make_certificate(rep, fleet, show["limits"])
    return {
        "format": FORMAT,
        "label": LABEL,
        "safety_statement": SAFETY_STATEMENT,
        "show": show,
        "fleet": fleet.to_dict(),
        "certificate": cert,
        "stress_summary": stress_summary,
    }


def save_export(path: str | Path, bundle: dict) -> None:
    Path(path).write_text(json.dumps(bundle, indent=1), encoding="utf-8")


def _canon(x):
    """JSON round trip, so tuples/ints/floats compare the way they do on disk."""
    return json.loads(json.dumps(x))


def load_export(path: str | Path) -> tuple[dict, Fleet, dict]:
    """Load and re-check an export bundle.

    Checks: format and label; the show validates; the fleet parses; the fleet hash matches; the
    certificate's limits are the show's limits (otherwise a certificate computed against looser
    limits could sit next to a stricter show); the drone count matches; the verification is
    recomputed and agrees. This detects accidental change or stale bundles. It is not
    authentication: whoever edits a bundle can also recompute every hash.
    Raises ValueError for any failure, including malformed structure.
    """
    from .spec import validate_show

    b = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(b, dict) or b.get("format") != FORMAT or b.get("label") != LABEL:
        raise ValueError("not a sirb export or label missing")
    try:
        show, cert = b["show"], b["certificate"]
        errs = validate_show(show, check_targets=False)
        if errs:
            raise ValueError("embedded show is invalid: " + "; ".join(errs))
        fleet = Fleet.from_dict(b["fleet"])
        if fleet.n != show["n_drones"]:
            raise ValueError(f"fleet has {fleet.n} drones, show declares {show['n_drones']}")
        if _canon(cert["limits"]) != _canon(show["limits"]):
            raise ValueError("certificate was computed against different limits than the show")
        ok = verify_certificate(cert, fleet)
    except (KeyError, TypeError, AttributeError, IndexError) as exc:
        raise ValueError(f"malformed export: {type(exc).__name__}: {exc}") from None
    if not ok:
        raise ValueError("certificate does not match fleet (tampered or stale)")
    return show, fleet, cert
