"""Generate the original demo shows into shows/. Deterministic (fixed seeds)."""

from __future__ import annotations

import sys
from pathlib import Path

from sirb.spec import build_targets, new_show, save_show, validate_show
from sirb.shapes import min_nn_distance

LAUNCH = {"type": "launch_grid", "spacing_m": 3.0, "center": [0.0, -85.0, 0.0]}


def demos() -> dict[str, dict]:
    out = {}
    out["sirb_text_100"] = new_show("Sirb text (100 drones)", 100, [
        {"name": "launch", "shape": LAUNCH, "hold_s": 2},
        {"name": "orb", "shape": {"type": "sphere", "spacing_m": 3.5, "center": [0, 0, 55]}, "hold_s": 4},
        {"name": "word", "shape": {"type": "text", "text": "SIRB", "spacing_m": 3.2,
                                    "center": [0, 0, 60]}, "hold_s": 5},
        {"name": "ring", "shape": {"type": "ring", "plane": "xz", "spacing_m": 3.2,
                                    "center": [0, 0, 60]}, "hold_s": 4},
        {"name": "land", "shape": LAUNCH, "hold_s": 2},
    ], seed=11)
    out["skyline_200"] = new_show("Original skyline (200 drones)", 200, [
        {"name": "launch", "shape": LAUNCH, "hold_s": 2},
        {"name": "skyline", "shape": {"type": "skyline", "buildings": 9, "spacing_m": 3.2,
                                       "center": [0, 0, 55]}, "hold_s": 5},
        {"name": "galaxy", "shape": {"type": "galaxy", "arms": 3, "spacing_m": 3.2,
                                      "center": [0, 0, 70], "rotate_z_deg": 0}, "hold_s": 5},
        {"name": "helix", "shape": {"type": "helix", "turns": 3, "strands": 2, "spacing_m": 3.2,
                                     "center": [0, 0, 20]}, "hold_s": 4},
        {"name": "land", "shape": LAUNCH, "hold_s": 2},
    ], seed=22)
    out["torus_300"] = new_show("Torus orbit (300 drones)", 300, [
        {"name": "launch", "shape": LAUNCH, "hold_s": 2},
        {"name": "torus", "shape": {"type": "torus", "minor_ratio": 0.35, "spacing_m": 3.2,
                                     "center": [0, 0, 60]}, "hold_s": 5},
        {"name": "super", "shape": {"type": "supershape", "m1": 6, "m2": 2, "amplitude": 0.3,
                                      "spacing_m": 3.3, "center": [0, 0, 60]}, "hold_s": 5},
        {"name": "land", "shape": LAUNCH, "hold_s": 2},
    ], seed=33)
    out["wall_500"] = new_show("Skyline and orb (500 drones)", 500, [
        {"name": "launch", "shape": LAUNCH, "hold_s": 2},
        {"name": "orb", "shape": {"type": "sphere", "spacing_m": 3.4, "center": [0, 0, 60]},
         "hold_s": 4},
        {"name": "skyline", "shape": {"type": "skyline", "buildings": 11, "spacing_m": 3.2,
                                       "center": [0, 0, 55]}, "hold_s": 5},
        {"name": "super", "shape": {"type": "supershape", "m1": 5, "m2": 3, "amplitude": 0.3,
                                      "spacing_m": 3.3, "center": [0, 0, 60]}, "hold_s": 5},
        {"name": "land", "shape": LAUNCH, "hold_s": 2},
    ], seed=44)
    return out


def main(outdir: str = "shows") -> int:
    Path(outdir).mkdir(exist_ok=True)
    bad = 0
    for key, show in demos().items():
        errs = validate_show(show)
        tg = build_targets(show) if not errs else []
        for i, t in enumerate(tg):
            print(f"{key} stage {i} {show['stages'][i]['name']}: min_nn={min_nn_distance(t):.2f} "
                  f"z=[{t[:,2].min():.1f},{t[:,2].max():.1f}] x=[{t[:,0].min():.0f},{t[:,0].max():.0f}] "
                  f"y=[{t[:,1].min():.0f},{t[:,1].max():.0f}]")
        if errs:
            bad += 1
            print(key, "INVALID:", errs)
        else:
            save_show(show, Path(outdir) / f"{key}.json")
    return bad


if __name__ == "__main__":
    sys.exit(main(*sys.argv[1:]))
