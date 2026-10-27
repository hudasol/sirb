"""Separation check on a fixed time grid.

This is the usual way to check a drone show for collisions: evaluate all positions every ``dt``
seconds and take the smallest pairwise distance. It can miss a close approach that falls between
two samples. ``sirb.verify`` finds the exact minimum instead; the benchmark counts how often the
two disagree.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree

from ..trajectory import Fleet


@dataclass
class SampledReport:
    dt: float
    min_separation: float
    ok: bool
    n_samples: int


def sampled_min_separation(fleet: Fleet, dt: float, d_min: float,
                           chunk: int = 256) -> SampledReport:
    """Smallest pairwise distance over samples spaced ``dt`` apart (end points included)."""
    t_end = fleet.t_end
    times = np.unique(np.append(np.arange(0.0, t_end, dt), t_end))
    best = np.inf
    for lo in range(0, len(times), chunk):
        block = fleet.sample(times[lo:lo + chunk])           # (T, n, 3)
        for frame in block:
            if frame.shape[0] < 2:
                continue
            d, _ = cKDTree(frame).query(frame, k=2)
            best = min(best, float(d[:, 1].min()))
    return SampledReport(dt=dt, min_separation=best, ok=best >= d_min, n_samples=len(times))
