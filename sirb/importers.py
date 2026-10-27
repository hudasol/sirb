"""Load externally produced trajectories so they can be checked with the exact verifier.

CSV format: header ``drone,t,x,y,z`` (metres, seconds; x east, y north, z up), one row per sample,
times strictly increasing per drone, every drone sampled at the same first and last time. The
samples are joined by straight lines, so the check is exact for that piecewise linear path, which
is only as faithful to the true flown path as the sampling is fine. Separation only: no speed,
acceleration, geofence or altitude limits are checked.
"""

from __future__ import annotations

import csv
import io
import math
from pathlib import Path

import numpy as np

from .fileio import read_text_limited
from .trajectory import Fleet, Segment, Trajectory

MAX_DRONES = 5000
MAX_ROWS = 2_000_000


def fleet_from_csv(path: str | Path) -> Fleet:
    text = read_text_limited(path)
    reader = csv.reader(io.StringIO(text))
    try:
        header = [h.strip().lower() for h in next(reader)]
    except StopIteration:
        raise ValueError("empty file") from None
    if header != ["drone", "t", "x", "y", "z"]:
        raise ValueError(f"header must be drone,t,x,y,z, got {','.join(header)}")
    rows: dict[str, list[tuple[float, float, float, float]]] = {}
    for k, row in enumerate(reader, start=2):
        if not row:
            continue
        if k > MAX_ROWS:
            raise ValueError(f"more than {MAX_ROWS} rows")
        if len(row) != 5:
            raise ValueError(f"line {k}: expected 5 fields, got {len(row)}")
        try:
            vals = tuple(float(v) for v in row[1:])
        except ValueError:
            raise ValueError(f"line {k}: non numeric field") from None
        if not all(math.isfinite(v) for v in vals):
            raise ValueError(f"line {k}: non finite value")
        rows.setdefault(row[0].strip(), []).append(vals)  # type: ignore[arg-type]
        if len(rows) > MAX_DRONES:
            raise ValueError(f"more than {MAX_DRONES} drones")
    if len(rows) < 2:
        raise ValueError("need samples for at least 2 drones")
    trajs = []
    span = None
    for name in sorted(rows, key=lambda s: (len(s), s)):
        a = np.array(rows[name], dtype=float)
        if len(a) < 2:
            raise ValueError(f"drone {name}: need at least 2 samples")
        if (np.diff(a[:, 0]) <= 0).any():
            raise ValueError(f"drone {name}: times must be strictly increasing")
        if span is None:
            span = (a[0, 0], a[-1, 0])
        elif abs(a[0, 0] - span[0]) > 1e-9 or abs(a[-1, 0] - span[1]) > 1e-9:
            raise ValueError(f"drone {name}: sample times do not span the same interval")
        segs = [Segment(float(a[i, 0]), float(a[i + 1, 0]),
                        np.stack([a[i, 1:], (a[i + 1, 1:] - a[i, 1:]) / (a[i + 1, 0] - a[i, 0])]))
                for i in range(len(a) - 1)]
        trajs.append(Trajectory(segs))
    return Fleet(trajs, {"source": "csv", "interpolation": "piecewise linear"})
