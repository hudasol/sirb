"""Show specification: schema, validation, target generation.

See docs/SHOW_SCHEMA.md. Frame: x east, y north, z up, metres, local origin.
"""

from __future__ import annotations

import copy
import json
from pathlib import Path

import numpy as np

from .shapes import generate_points, min_nn_distance

SCHEMA_VERSION = 1
SHAPE_TYPES = {"launch_grid", "sphere", "supershape", "ring", "helix", "torus", "galaxy",
               "text", "wall", "skyline"}

# Vehicle numbers: X7 published values where marked, everything else ASSUMPTION.
DEFAULT_VEHICLE = {
    "name": "reference-multirotor-0.83kg",
    "mass_kg": 0.83,            # Verge Aero X7 fact sheet (2026.03)
    "radius_m": 0.24,           # half of 48 cm prop-to-prop (X7 fact sheet)
    "sources": {"mass_kg": "datasheet", "radius_m": "datasheet"},
}
DEFAULT_LIMITS = {
    "vmax_h": 10.0,             # datasheet (X7 max horizontal speed)
    "vmax_up": 6.0,             # datasheet (X7 max vertical speed)
    "vmax_down": 3.0,           # ASSUMPTION (conservative descent)
    "amax": 4.0,                # ASSUMPTION
    "jmax": 8.0,                # ASSUMPTION
    "z_min": 0.0,
    "z_max": 120.0,             # ASSUMPTION, about 400 ft
    "geofence": {"x": [-150.0, 150.0], "y": [-130.0, 100.0]},  # ASSUMPTION, box
    "min_separation_m": 2.0,    # ASSUMPTION, declared verification threshold (centre to centre)
    "crowd": {"y": -250.0, "min_distance_m": 50.0},  # 50 m per CAP 722E / AC-010 (see RESEARCH)
    "sources": {"vmax_h": "datasheet", "vmax_up": "datasheet", "vmax_down": "assumption",
                "amax": "assumption", "jmax": "assumption", "z_max": "assumption",
                "geofence": "assumption", "min_separation_m": "assumption",
                "crowd": "cited guidance, see docs/RESEARCH.md"},
}


def new_show(name: str, n: int, stages: list[dict], seed: int = 0, **overrides) -> dict:
    show = {
        "schema_version": SCHEMA_VERSION,
        "name": name,
        "n_drones": n,
        "seed": seed,
        "vehicle": copy.deepcopy(DEFAULT_VEHICLE),
        "limits": copy.deepcopy(DEFAULT_LIMITS),
        "stages": stages,
        "disclaimer": "simulation-validated only; not flight-certified",
    }
    for k, v in overrides.items():
        if isinstance(v, dict) and isinstance(show.get(k), dict):
            show[k].update(v)
        else:
            show[k] = v
    return show


def build_targets(show: dict) -> list[np.ndarray]:
    """Target points per stage, each (n, 3). Deterministic given show['seed']."""
    n = show["n_drones"]
    return [generate_points(st["shape"], n, seed=show["seed"] + 1000 * i)
            for i, st in enumerate(show["stages"])]


def validate_show(show: dict, check_targets: bool = True) -> list[str]:
    """Return a list of error strings (empty if valid)."""
    errs: list[str] = []
    for key in ("schema_version", "name", "n_drones", "seed", "vehicle", "limits", "stages"):
        if key not in show:
            errs.append(f"missing key {key!r}")
    if errs:
        return errs
    if show["schema_version"] != SCHEMA_VERSION:
        errs.append(f"unsupported schema_version {show['schema_version']}")
    n = show["n_drones"]
    if not isinstance(n, int) or n < 2:
        errs.append("n_drones must be an integer >= 2")
        return errs
    lim = show["limits"]
    for key in ("vmax_h", "vmax_up", "vmax_down", "amax", "jmax", "z_min", "z_max",
                "geofence", "min_separation_m", "crowd"):
        if key not in lim:
            errs.append(f"limits missing {key!r}")
    if errs:
        return errs
    for key in ("vmax_h", "vmax_up", "vmax_down", "amax", "jmax", "min_separation_m"):
        if not (isinstance(lim[key], (int, float)) and lim[key] > 0):
            errs.append(f"limits.{key} must be > 0")
    if lim["z_max"] <= lim["z_min"]:
        errs.append("limits.z_max must exceed z_min")
    g = lim["geofence"]
    if not (g["x"][0] < g["x"][1] and g["y"][0] < g["y"][1]):
        errs.append("geofence ranges must be increasing")
    crowd = lim["crowd"]
    if g["y"][0] < crowd["y"] + crowd["min_distance_m"] - 1e-9:
        errs.append("geofence reaches inside the crowd buffer")
    stages = show["stages"]
    if len(stages) < 2:
        errs.append("need at least 2 stages")
    for i, st in enumerate(stages):
        if st.get("shape", {}).get("type") not in SHAPE_TYPES:
            errs.append(f"stage {i}: unknown shape type")
        if st.get("hold_s", 0) < 0:
            errs.append(f"stage {i}: hold_s must be >= 0")
    if errs or not check_targets:
        return errs
    try:
        targets = build_targets(show)
    except Exception as exc:  # noqa: BLE001 - report any generator failure
        return [f"shape generation failed: {exc}"]
    for i, pts in enumerate(targets):
        if len(pts) != n:
            errs.append(f"stage {i}: produced {len(pts)} points, expected {n}")
            continue
        nn = min_nn_distance(pts)
        if nn < lim["min_separation_m"] * 1.5 - 1e-9:
            errs.append(f"stage {i}: target spacing {nn:.2f} m is below 1.5 x min separation")
        if (pts[:, 2] < lim["z_min"] - 1e-9).any() or (pts[:, 2] > lim["z_max"] + 1e-9).any():
            errs.append(f"stage {i}: targets outside altitude band")
        if ((pts[:, 0] < g["x"][0]) | (pts[:, 0] > g["x"][1])
                | (pts[:, 1] < g["y"][0]) | (pts[:, 1] > g["y"][1])).any():
            errs.append(f"stage {i}: targets outside geofence")
    return errs


def save_show(show: dict, path: str | Path) -> None:
    Path(path).write_text(json.dumps(show, indent=2) + "\n")


def load_show(path: str | Path) -> dict:
    show = json.loads(Path(path).read_text())
    errs = validate_show(show, check_targets=False)
    if errs:
        raise ValueError("invalid show: " + "; ".join(errs))
    return show
