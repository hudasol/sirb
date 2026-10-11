"""Trajectory data format shared by planners, stress tests, export and certificates.

Data classes plus small helpers (evaluation, sampling, content hash, dict round trip), no planning
or verification logic. ``sirb.verify`` does not import this module (it reads ``.segments``,
``.t0``, ``.t1`` and ``.coeffs`` by attribute), which ``tests/test_review_regressions.py`` checks
from the import graph. A trajectory is a list of
contiguous polynomial segments; within a segment ``pos(tau) = sum_k coeffs[k] * tau**k``
with ``tau = t - t0`` (metres, seconds; x east, y north, z up).
"""

from __future__ import annotations

import hashlib
from dataclasses import dataclass, field

import numpy as np


def horner(coeffs: np.ndarray, tau) -> np.ndarray:
    """Evaluate vector polynomial (deg+1, 3) at tau (any shape) -> tau.shape + (3,)."""
    tau = np.asarray(tau, dtype=float)
    out = np.zeros(tau.shape + (3,))
    for k in range(len(coeffs) - 1, -1, -1):
        out = out * tau[..., None] + coeffs[k]
    return out


@dataclass(frozen=True)
class Segment:
    t0: float
    t1: float
    coeffs: np.ndarray  # shape (deg+1, 3)

    def pos(self, t) -> np.ndarray:
        return horner(self.coeffs, np.asarray(t, dtype=float) - self.t0)

    def start(self) -> np.ndarray:
        return self.coeffs[0].copy()

    def end(self) -> np.ndarray:
        return self.pos(self.t1)


@dataclass
class Trajectory:
    segments: list[Segment]

    @property
    def t_start(self) -> float:
        return self.segments[0].t0

    @property
    def t_end(self) -> float:
        return self.segments[-1].t1

    def pos(self, t: float) -> np.ndarray:
        t = float(np.clip(t, self.t_start, self.t_end))
        for seg in self.segments:
            if t <= seg.t1:
                return seg.pos(t)
        return self.segments[-1].end()

    def sample(self, times: np.ndarray) -> np.ndarray:
        """Positions at an array of times, shape (len(times), 3)."""
        times = np.clip(np.asarray(times, dtype=float), self.t_start, self.t_end)
        out = np.empty((len(times), 3))
        edges = np.array([s.t1 for s in self.segments])
        idx = np.minimum(np.searchsorted(edges, times, side="left"), len(self.segments) - 1)
        for i in np.unique(idx):
            m = idx == i
            out[m] = self.segments[i].pos(times[m])
        return out


@dataclass
class Fleet:
    trajectories: list[Trajectory]
    meta: dict = field(default_factory=dict)

    @property
    def n(self) -> int:
        return len(self.trajectories)

    @property
    def t_end(self) -> float:
        return max(t.t_end for t in self.trajectories)

    def positions(self, t: float) -> np.ndarray:
        return np.array([tr.pos(t) for tr in self.trajectories])

    def sample(self, times: np.ndarray) -> np.ndarray:
        """Shape (len(times), n, 3)."""
        return np.stack([tr.sample(times) for tr in self.trajectories], axis=1)

    def content_hash(self) -> str:
        h = hashlib.sha256()
        for tr in self.trajectories:
            for s in tr.segments:
                h.update(np.array([s.t0, s.t1], dtype="<f8").tobytes())
                h.update(np.ascontiguousarray(s.coeffs, dtype="<f8").tobytes())
            h.update(b"|")
        return h.hexdigest()

    def to_dict(self) -> dict:
        return {
            "meta": self.meta,
            "drones": [
                [
                    {"t0": s.t0, "t1": s.t1, "coeffs": s.coeffs.tolist()}
                    for s in tr.segments
                ]
                for tr in self.trajectories
            ],
        }

    @classmethod
    def from_dict(cls, d: dict) -> Fleet:
        trs = [
            Trajectory([Segment(s["t0"], s["t1"], np.array(s["coeffs"], dtype=float))
                        for s in segs])
            for segs in d["drones"]
        ]
        return cls(trs, dict(d.get("meta", {})))
