"""Original, procedurally generated shape sources.

Every shape returns exactly ``n`` points (n, 3) in metres. Size is set either
explicitly (``size``) or by ``spacing_m``: the shape is scaled so the *minimum*
nearest-neighbour distance equals ``spacing_m``. Then it is rotated about z by
``rotate_z_deg`` and translated to ``center``. No logos, fonts or external data.
"""

from __future__ import annotations

import math

import numpy as np
from scipy.spatial import cKDTree

# Minimal original stroke font on a unit box (x 0..1, y 0..1). Letters only as needed.
_FONT: dict[str, list[list[tuple[float, float]]]] = {
    "A": [[(0, 0), (0.5, 1), (1, 0)], [(0.2, 0.4), (0.8, 0.4)]],
    "B": [[(0, 0), (0, 1), (0.75, 1), (0.95, 0.85), (0.95, 0.65), (0.75, 0.5), (0, 0.5)],
          [(0.75, 0.5), (1, 0.35), (1, 0.15), (0.8, 0), (0, 0)]],
    "D": [[(0, 0), (0, 1), (0.7, 1), (1, 0.75), (1, 0.25), (0.7, 0), (0, 0)]],
    "E": [[(1, 1), (0, 1), (0, 0), (1, 0)], [(0, 0.5), (0.8, 0.5)]],
    "H": [[(0, 0), (0, 1)], [(1, 0), (1, 1)], [(0, 0.5), (1, 0.5)]],
    "I": [[(0.5, 0), (0.5, 1)]],
    "L": [[(0, 1), (0, 0), (1, 0)]],
    "N": [[(0, 0), (0, 1), (1, 0), (1, 1)]],
    "O": [[(0.2, 0), (0.8, 0), (1, 0.2), (1, 0.8), (0.8, 1), (0.2, 1), (0, 0.8),
           (0, 0.2), (0.2, 0)]],
    "R": [[(0, 0), (0, 1), (0.8, 1), (1, 0.85), (1, 0.65), (0.8, 0.5), (0, 0.5)],
          [(0.5, 0.5), (1, 0)]],
    "S": [[(1, 0.85), (0.8, 1), (0.2, 1), (0, 0.85), (0, 0.65), (0.2, 0.5), (0.8, 0.5),
           (1, 0.35), (1, 0.15), (0.8, 0), (0.2, 0), (0, 0.15)]],
    "T": [[(0, 1), (1, 1)], [(0.5, 1), (0.5, 0)]],
    "U": [[(0, 1), (0, 0.15), (0.2, 0), (0.8, 0), (1, 0.15), (1, 1)]],
}
SUPPORTED_LETTERS = "".join(sorted(_FONT))


def _along_polylines(polys: list[np.ndarray], n: int) -> np.ndarray:
    """n points equally spaced by arc length along the concatenated polylines."""
    pts, cum, total = [], [], 0.0
    for p in polys:
        d = np.linalg.norm(np.diff(p, axis=0), axis=1)
        for a, b, L in zip(p[:-1], p[1:], d):
            pts.append((a, b, total, L))
            total += L
    starts = np.array([s for _, _, s, _ in pts])
    targets = (np.arange(n) + 0.5) * total / n
    idx = np.clip(np.searchsorted(starts, targets, side="right") - 1, 0, len(pts) - 1)
    out = np.empty((n, p.shape[1]))
    for i, (t, j) in enumerate(zip(targets, idx)):
        a, b, s, L = pts[j]
        out[i] = a + (b - a) * ((t - s) / L if L > 0 else 0.0)
    return out


def _fibonacci_sphere(n: int) -> np.ndarray:
    i = np.arange(n) + 0.5
    phi = math.pi * (3.0 - math.sqrt(5.0))
    z = 1 - 2 * i / n
    r = np.sqrt(1 - z * z)
    th = phi * i
    return np.stack([r * np.cos(th), r * np.sin(th), z], axis=1)


def _farthest_point_subsample(pts: np.ndarray, n: int) -> np.ndarray:
    if len(pts) < n:
        raise ValueError(f"lattice produced {len(pts)} points, need {n}")
    chosen = [0]
    dist = np.linalg.norm(pts - pts[0], axis=1)
    for _ in range(n - 1):
        k = int(np.argmax(dist))
        chosen.append(k)
        dist = np.minimum(dist, np.linalg.norm(pts - pts[k], axis=1))
    return pts[np.array(chosen)]


def _lattice_fit(mask_fn, extent: tuple[float, float], n: int) -> np.ndarray:
    """2D unit lattice points inside mask, refined until at least 1.3 n exist, then FPS."""
    w, h = extent
    pitch = math.sqrt(w * h / n)
    while True:
        xs = np.arange(0, w + 1e-9, pitch)
        ys = np.arange(0, h + 1e-9, pitch)
        gx, gy = np.meshgrid(xs, ys)
        cand = np.stack([gx.ravel(), gy.ravel()], axis=1)
        cand = cand[mask_fn(cand[:, 0], cand[:, 1])]
        if len(cand) >= int(1.3 * n):
            return _farthest_point_subsample(cand, n)
        pitch *= 0.85


def _unit_shape(spec: dict, n: int, rng: np.random.Generator) -> np.ndarray:
    kind = spec["type"]
    if kind == "sphere":
        return _fibonacci_sphere(n)
    if kind == "supershape":
        # Original radial modulation of a sphere: r(theta, phi) = 1 + a cos(m1 th) cos(m2 ph)
        dirs = _fibonacci_sphere(n)
        th = np.arctan2(dirs[:, 1], dirs[:, 0])
        ph = np.arcsin(np.clip(dirs[:, 2], -1, 1))
        m1, m2 = spec.get("m1", 5), spec.get("m2", 3)
        a = spec.get("amplitude", 0.35)
        r = 1 + a * np.cos(m1 * th) * np.cos(m2 * ph)
        return dirs * r[:, None]
    if kind == "ring":
        t = (np.arange(n) + 0.5) * 2 * math.pi / n
        if spec.get("plane", "xz") == "xz":
            return np.stack([np.cos(t), np.zeros(n), np.sin(t)], axis=1)
        return np.stack([np.cos(t), np.sin(t), np.zeros(n)], axis=1)
    if kind == "helix":
        turns = spec.get("turns", 3)
        strands = spec.get("strands", 2)
        per = n // strands
        parts = []
        for s in range(strands):
            m = per + (1 if s < n - per * strands else 0)
            t = np.linspace(0, 1, m)
            ang = 2 * math.pi * turns * t + 2 * math.pi * s / strands
            parts.append(np.stack([np.cos(ang), np.sin(ang), 2.5 * t], axis=1))
        return np.concatenate(parts)
    if kind == "torus":
        ratio = spec.get("minor_ratio", 0.35)
        i = np.arange(n) + 0.5
        u = i * 2 * math.pi / n
        v = (i * 2 * math.pi * 0.618033988749895 * 11) % (2 * math.pi)
        x = (1 + ratio * np.cos(v)) * np.cos(u)
        y = (1 + ratio * np.cos(v)) * np.sin(u)
        z = ratio * np.sin(v)
        return np.stack([x, y, z], axis=1)
    if kind == "galaxy":
        arms = spec.get("arms", 3)
        m = 30 * n
        r = np.sqrt(rng.random(m))
        th = 2 * math.pi * rng.random(m)
        keep = np.cos(arms * (th - 3.2 * r)) > 0.15
        r, th = r[keep], th[keep]
        cand = np.stack([r * np.cos(th), np.zeros(len(r)), r * np.sin(th)], axis=1)
        return _farthest_point_subsample(cand, n)
    if kind == "text":
        text = spec["text"].upper()
        polys, x0 = [], 0.0
        for ch in text:
            if ch == " ":
                x0 += 0.8
                continue
            if ch not in _FONT:
                raise ValueError(f"letter {ch!r} not in stroke font; supported: {SUPPORTED_LETTERS}")
            for stroke in _FONT[ch]:
                polys.append(np.array([(x0 + a, b) for a, b in stroke], dtype=float))
            x0 += 1.4
        dense = _along_polylines(polys, 24 * n)
        pts2 = _farthest_point_subsample(dense, n)
        pts2[:, 0] -= pts2[:, 0].mean()
        pts2[:, 1] -= pts2[:, 1].mean()
        return np.stack([pts2[:, 0], np.zeros(n), pts2[:, 1]], axis=1)
    if kind == "wall":
        ratio = spec.get("aspect", 1.6)
        pts2 = _lattice_fit(lambda x, y: np.ones_like(x, bool), (ratio, 1.0), n)
        return np.stack([pts2[:, 0] - ratio / 2, np.zeros(n), pts2[:, 1] - 0.5], axis=1)
    if kind == "skyline":
        k = spec.get("buildings", 9)
        heights = 0.35 + 0.65 * rng.random(k)
        heights[k // 2] = 1.0
        ratio = spec.get("aspect", 2.0)

        def mask(x, y):
            idx = np.minimum((x / ratio * k).astype(int), k - 1)
            return y <= heights[idx]

        pts2 = _lattice_fit(mask, (ratio, 1.0), n)
        return np.stack([pts2[:, 0] - ratio / 2, np.zeros(n), pts2[:, 1] - 0.5], axis=1)
    raise ValueError(f"unknown shape type {kind!r}")


def min_nn_distance(pts: np.ndarray) -> float:
    if len(pts) < 2:
        return math.inf
    d, _ = cKDTree(pts).query(pts, k=2)
    return float(d[:, 1].min())


def launch_grid(n: int, spacing_m: float, center=(0.0, 0.0)) -> np.ndarray:
    cols = math.ceil(math.sqrt(n))
    idx = np.arange(n)
    gx = (idx % cols) * spacing_m
    gy = (idx // cols) * spacing_m
    pts = np.stack([gx, gy, np.zeros(n)], axis=1)
    pts[:, 0] += center[0] - pts[:, 0].mean()
    pts[:, 1] += center[1] - pts[:, 1].mean()
    return pts


def generate_points(spec: dict, n: int, seed: int = 0) -> np.ndarray:
    """Generate (n, 3) points for a shape spec."""
    if spec["type"] == "launch_grid":
        c = spec.get("center", [0.0, 0.0, 0.0])
        return launch_grid(n, spec["spacing_m"], (c[0], c[1]))
    rng = np.random.default_rng(seed)
    pts = _unit_shape(spec, n, rng)
    if spec.get("size") is not None and spec.get("spacing_m") is not None:
        raise ValueError("give either size or spacing_m, not both")
    if spec.get("spacing_m") is not None:
        nn = min_nn_distance(pts)
        scale = spec["spacing_m"] / nn
    else:
        scale = float(spec.get("size", 1.0))
    pts = pts * scale
    th = math.radians(spec.get("rotate_z_deg", 0.0))
    rot = np.array([[math.cos(th), -math.sin(th), 0], [math.sin(th), math.cos(th), 0], [0, 0, 1]])
    pts = pts @ rot.T
    return pts + np.array(spec.get("center", [0.0, 0.0, 0.0]), dtype=float)
