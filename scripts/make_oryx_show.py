"""Generate shows/oryx_500.json: the orb, an Arabian oryx silhouette (original drawing) and a
supershape, 500 drones. Same launch, orb, supershape and landing stages as wall_500, with the oryx
in place of the skyline. Deterministic. Run:  python scripts/make_oryx_show.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from matplotlib.path import Path as MPath

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sirb.shapes import _lattice_fit, min_nn_distance  # noqa: E402
from sirb.spec import build_targets, new_show, save_show, validate_show  # noqa: E402

N = 500
SPACING = 3.3
LAUNCH = {"type": "launch_grid", "spacing_m": 3.0, "center": [0.0, -85.0, 0.0]}


def oryx():
    """Side view facing left: head with two long, almost straight horns swept back, body, four legs, tail."""
    body = [(0.4, 3.4), (1.0, 6.3), (3.0, 7.0), (6.0, 7.0), (8.2, 6.4), (9.0, 5.0), (8.4, 3.2),
            (6.0, 2.9), (2.0, 2.9)]
    neck = [(0.3, 5.0), (2.0, 6.8), (0.4, 8.6), (-1.2, 8.0)]
    head = [(-1.2, 8.0), (0.4, 8.6), (0.8, 9.0), (-0.3, 9.1), (-2.1, 8.0), (-2.5, 7.4), (-1.7, 7.3)]
    horn_a = [(0.1, 8.9), (0.8, 9.0), (3.3, 10.0), (3.0, 10.2)]
    horn_b = [(-0.2, 8.7), (0.5, 8.7), (2.7, 9.5), (2.4, 9.7)]
    leg_f1 = [(1.0, 3.4), (1.6, 3.4), (1.6, 0.0), (1.0, 0.0)]
    leg_f2 = [(2.4, 3.2), (3.0, 3.2), (3.0, 0.0), (2.4, 0.0)]
    leg_b1 = [(6.2, 3.2), (6.8, 3.2), (6.8, 0.0), (6.2, 0.0)]
    leg_b2 = [(7.6, 3.6), (8.2, 3.6), (8.2, 0.0), (7.6, 0.0)]
    tail = [(8.8, 5.6), (9.6, 4.9), (9.9, 3.2), (9.3, 3.3), (9.0, 4.6)]
    return [body, neck, head, horn_a, horn_b, leg_f1, leg_f2, leg_b1, leg_b2, tail]


Z_SQUASH = 0.85
BOTTOM_Z = 6.0


def silhouette(polys, n=N, spacing=SPACING):
    """n points filling the union of the polygons, nearest neighbour distance scaled to spacing."""
    polys = [[(x, z * Z_SQUASH) for x, z in poly] for poly in polys]
    allp = np.array([p for poly in polys for p in poly], float)
    lo = allp.min(axis=0)
    paths = [MPath(np.array(poly, float) - lo) for poly in polys]
    w, h = allp.max(axis=0) - lo

    def mask(x, z):
        pts = np.stack([x, z], axis=1)
        m = np.zeros(len(pts), bool)
        for p in paths:
            m |= p.contains_points(pts)
        return m

    pts2 = _lattice_fit(mask, (float(w), float(h)), n)
    pts = np.stack([pts2[:, 0], np.zeros(n), pts2[:, 1]], axis=1)
    pts *= spacing / min_nn_distance(pts)
    pts[:, 0] -= (pts[:, 0].max() + pts[:, 0].min()) / 2
    pts[:, 2] += BOTTOM_Z - pts[:, 2].min()
    return np.round(pts, 3)


def show() -> dict:
    pts = silhouette(oryx())
    return new_show("Oryx and orb (500 drones)", N, [
        {"name": "launch", "shape": LAUNCH, "hold_s": 2},
        {"name": "orb", "shape": {"type": "sphere", "spacing_m": 3.4, "center": [0, 0, 60]},
         "hold_s": 4},
        {"name": "oryx", "shape": {"type": "points", "points": pts.tolist()}, "hold_s": 5},
        {"name": "super", "shape": {"type": "supershape", "m1": 5, "m2": 3, "amplitude": 0.3,
                                      "spacing_m": 3.3, "center": [0, 0, 60]}, "hold_s": 5},
        {"name": "land", "shape": LAUNCH, "hold_s": 2},
    ], seed=44)


def main(outdir: str = "shows") -> int:
    s = show()
    errs = validate_show(s)
    for st, t in zip(s["stages"], build_targets(s), strict=True):
        print(f"{st['name']:7s} min_nn={min_nn_distance(t):.2f} x=[{t[:,0].min():.0f},{t[:,0].max():.0f}] "
              f"z=[{t[:,2].min():.0f},{t[:,2].max():.0f}]")
    if errs:
        print("INVALID:", errs)
        return 1
    save_show(s, Path(outdir) / "oryx_500.json")
    return 0


if __name__ == "__main__":
    sys.exit(main(*sys.argv[1:]))
