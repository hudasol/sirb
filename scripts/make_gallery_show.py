"""Generate shows/gallery_120.json: a multi stage show of original silhouettes.

Every silhouette is drawn here from polygons designed for this repository. Deterministic (no random
numbers except the planner's own seeded steps). Run:  python scripts/make_gallery_show.py
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
from matplotlib.path import Path as MPath

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sirb.shapes import min_nn_distance  # noqa: E402
from sirb.spec import build_targets, new_show, save_show, validate_show  # noqa: E402

N = 120
SPACING = 3.1          # target nearest neighbour distance inside each silhouette, metres
CENTER_Z = 60.0
LAUNCH = {"type": "launch_grid", "spacing_m": 3.0, "center": [0.0, -85.0, 0.0]}


def mirror(poly):
    return [(-x, z) for x, z in reversed(poly)]


def circle(cx, cz, r, k=20):
    return [(cx + r * math.cos(2 * math.pi * i / k), cz + r * math.sin(2 * math.pi * i / k))
            for i in range(k)]


def leaf(base, tip, width):
    b, t = np.array(base, float), np.array(tip, float)
    d = t - b
    perp = np.array([-d[1], d[0]]) / np.linalg.norm(d)
    mid = b + 0.45 * d
    return [tuple(b), tuple(mid + perp * width), tuple(t), tuple(mid - perp * width)]


def falcon():
    wing = [(0.6, 8.0), (2.5, 8.6), (5, 8.0), (7.5, 6.2), (8.2, 4.6), (7.0, 5.0), (7.2, 3.8),
            (6.0, 4.4), (6.0, 3.2), (4.8, 4.2), (4.0, 3.4), (3.4, 4.6), (0.7, 5.0)]
    body = [(0, 9), (0.6, 8.2), (0.7, 6), (0.5, 3.5), (0, 1), (-0.5, 3.5), (-0.7, 6), (-0.6, 8.2)]
    tail = [(-0.6, 3.5), (0.6, 3.5), (1.1, 0.3), (0, 0.8), (-1.1, 0.3)]
    beak = [(-0.1, 9.3), (0.4, 9.2), (0.05, 8.5)]
    return [wing, mirror(wing), body, tail, circle(0, 9.6, 0.6), beak]


def dhow():
    hull = [(-6, 2.2), (6.5, 3.0), (5, 0.8), (-4.5, 0.8)]
    mast = [(-0.3, 2.2), (0.3, 2.2), (0.3, 9), (-0.3, 9)]
    sail = [(0.3, 8.8), (6, 3.3), (0.3, 3.3)]
    jib = [(-0.3, 8.2), (-4.8, 3.3), (-0.3, 3.3)]
    wave = [(-7, 0.8), (7, 0.8), (7, 0.0), (-7, 0.0)]
    return [hull, mast, sail, jib, wave]


def palm():
    trunk = [(-0.5, 0), (0.5, 0), (0.8, 3), (0.6, 5), (0.0, 5.6), (-0.5, 5), (-0.3, 3)]
    crown = (0.1, 5.4)
    tips = [(-6.0, 3.4), (-5.0, 6.9), (-2.4, 8.1), (2.0, 8.1), (5.2, 6.9), (6.2, 3.4), (0.1, 8.7)]
    return [trunk] + [leaf(crown, tip, 0.7) for tip in tips] + [circle(0.1, 5.4, 0.55)]


def hero():
    head = circle(0, 9.0, 0.75)
    torso = [(-1.6, 8.2), (1.6, 8.2), (1.2, 5.2), (0.9, 4.6), (-0.9, 4.6), (-1.2, 5.2)]
    arm_up = [(-1.6, 8.2), (-4.2, 9.6), (-4.5, 8.8), (-1.5, 7.2)]
    arm_out = [(1.6, 8.2), (4.6, 8.4), (4.6, 7.6), (1.4, 7.0)]
    leg_l = [(-0.9, 4.6), (-0.1, 4.6), (-0.3, 0.6), (-1.5, 0.0), (-1.3, 0.8)]
    leg_r = [(0.1, 4.6), (0.9, 4.6), (1.3, 0.8), (1.5, 0.0), (0.3, 0.6)]
    cape = [(-1.6, 8.0), (-3.8, 2.0), (-2.4, 2.8), (-1.2, 5.0)]
    return [head, torso, arm_up, arm_out, leg_l, leg_r, cape]


def heart():
    t = np.linspace(0, 2 * math.pi, 60, endpoint=False)
    x = 16 * np.sin(t) ** 3
    z = 13 * np.cos(t) - 5 * np.cos(2 * t) - 2 * np.cos(3 * t) - np.cos(4 * t)
    return [list(zip((x / 2.2).tolist(), (z / 3.4).tolist(), strict=True))]


SHAPES = {"falcon": falcon, "dhow": dhow, "palm": palm, "hero": hero, "heart": heart}


def silhouette(polys, n=N, spacing=SPACING, res=260):
    """n points evenly spaced along the outline of the union of the polygons."""
    from scipy.ndimage import binary_erosion

    from sirb.shapes import _farthest_point_subsample
    allp = np.array([p for poly in polys for p in poly], float)
    lo, hi = allp.min(axis=0), allp.max(axis=0)
    pitch = float(max(hi - lo)) / res
    xs = np.arange(lo[0] - 2 * pitch, hi[0] + 2 * pitch, pitch)
    zs = np.arange(lo[1] - 2 * pitch, hi[1] + 2 * pitch, pitch)
    gx, gz = np.meshgrid(xs, zs)
    flat = np.stack([gx.ravel(), gz.ravel()], axis=1)
    inside = np.zeros(len(flat), bool)
    for poly in polys:
        inside |= MPath(np.array(poly, float)).contains_points(flat)
    img = inside.reshape(gx.shape)
    edge = img & ~binary_erosion(img, iterations=2)
    cand = np.stack([gx[edge], gz[edge]], axis=1)
    pts2 = _farthest_point_subsample(cand, n)
    pts = np.stack([pts2[:, 0], np.zeros(n), pts2[:, 1]], axis=1)
    pts *= spacing / min_nn_distance(pts)
    pts[:, 0] -= (pts[:, 0].max() + pts[:, 0].min()) / 2
    pts[:, 2] += CENTER_Z - (pts[:, 2].max() + pts[:, 2].min()) / 2
    return np.round(pts, 3)


def pts_stage(name, key, hold):
    pts = silhouette(SHAPES[key]())
    return {"name": name, "shape": {"type": "points", "points": pts.tolist()}, "hold_s": hold}


def gallery() -> dict:
    stages = [
        {"name": "launch", "shape": LAUNCH, "hold_s": 2},
        pts_stage("falcon", "falcon", 5),
        pts_stage("dhow", "dhow", 5),
        pts_stage("palm", "palm", 5),
        pts_stage("hero", "hero", 5),
        pts_stage("heart", "heart", 4),
        {"name": "word", "shape": {"type": "text", "text": "SIRB", "spacing_m": SPACING,
                                    "center": [0, 0, CENTER_Z]}, "hold_s": 5},
        {"name": "skyline", "shape": {"type": "skyline", "buildings": 11, "spacing_m": SPACING,
                                       "center": [0, 0, 55]}, "hold_s": 5},
        {"name": "land", "shape": LAUNCH, "hold_s": 2},
    ]
    return new_show("Gallery of silhouettes (120 drones)", N, stages, seed=55)


def main(outdir: str = "shows") -> int:
    show = gallery()
    errs = validate_show(show)
    for st, t in zip(show["stages"], build_targets(show), strict=True):
        print(f"{st['name']:8s} min_nn={min_nn_distance(t):.2f} x=[{t[:,0].min():.0f},{t[:,0].max():.0f}] "
              f"z=[{t[:,2].min():.0f},{t[:,2].max():.0f}]")
    if errs:
        print("INVALID:", errs)
        return 1
    save_show(show, Path(outdir) / "gallery_120.json")
    return 0


if __name__ == "__main__":
    sys.exit(main(*sys.argv[1:]))
