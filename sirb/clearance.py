"""Anisotropic (ellipsoidal) separation, for vehicles that must keep more vertical than
horizontal distance (downwash). Exact: scaling the z coefficients of every path by
``d_h / d_v`` maps the ellipsoid ``(dx^2 + dy^2) / d_h^2 + dz^2 / d_v^2 >= 1`` to the ordinary
sphere of radius ``d_h``, and the scaled paths are still polynomials, so the ordinary exact
verifier applies unchanged. The ellipsoid model follows the form used by Preiss et al. (2017,
arXiv 1704.04852); the numbers a given vehicle needs are not supplied here.
"""

from __future__ import annotations

import numpy as np

from .trajectory import Fleet, Segment, Trajectory
from .verify import pairwise_min_separation


def scale_z(fleet: Fleet, factor: float) -> Fleet:
    if not (np.isfinite(factor) and factor > 0):
        raise ValueError("factor must be a finite number > 0")
    trajs = []
    for tr in fleet.trajectories:
        segs = []
        for s in tr.segments:
            c = np.array(s.coeffs, dtype=float)
            c[:, 2] *= factor
            segs.append(Segment(s.t0, s.t1, c))
        trajs.append(Trajectory(segs))
    return Fleet(trajs, dict(fleet.meta))


def ellipsoidal_min_separation(fleet: Fleet, d_h: float, d_v: float,
                               search_radius: float | None = None):
    """Minimum over pairs and time of the ellipsoidal distance, in units of ``d_h`` metres
    (a value below ``d_h`` means the ellipsoids overlap). Returns the same tuple as
    ``pairwise_min_separation``."""
    if not (d_h > 0 and d_v >= d_h):
        raise ValueError("need 0 < d_h <= d_v")
    R = search_radius if search_radius is not None else 4.0 * d_h
    return pairwise_min_separation(scale_z(fleet, d_h / d_v), R)
