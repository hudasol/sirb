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
    Path(path).write_text(json.dumps(bundle, indent=1))


def load_export(path: str | Path) -> tuple[dict, Fleet, dict]:
    """Load and re-check: label present, fleet hash matches, certificate recomputes."""
    b = json.loads(Path(path).read_text())
    if b.get("format") != FORMAT or b.get("label") != LABEL:
        raise ValueError("not a sirb export or label missing")
    fleet = Fleet.from_dict(b["fleet"])
    if not verify_certificate(b["certificate"], fleet):
        raise ValueError("certificate does not match fleet (tampered or stale)")
    return b["show"], fleet, b["certificate"]
