"""Shared planner helpers: min-jerk legs, timing, limits, polynomial segment construction.

A leg is a rest-to-rest straight line with the quintic minimum-jerk profile
s(u) = 10u^3 - 15u^4 + 6u^5, u = tau / T. For displacement d (vector) and duration T:
peak speed component = 1.875 |d_k| / T, peak accel = 5.7735 |d| / T^2, peak jerk = 60 |d| / T^3.
"""

from __future__ import annotations

import math

import numpy as np

from ..trajectory import Segment, Trajectory

K_V, K_A, K_J = 1.875, 5.773502691896258, 60.0


def min_leg_time(d: np.ndarray, lim: dict, margin: float = 1.02) -> float:
    """Smallest T meeting speed, accel and jerk limits for a straight rest-to-rest leg."""
    dh = math.hypot(d[0], d[1])
    dz = d[2]
    L = float(np.linalg.norm(d))
    if L < 1e-12:
        return 0.0
    vz = lim["vmax_up"] if dz >= 0 else lim["vmax_down"]
    t = max(
        K_V * dh / lim["vmax_h"],
        K_V * abs(dz) / vz,
        math.sqrt(K_A * L / lim["amax"]),
        (K_J * L / lim["jmax"]) ** (1.0 / 3.0),
    )
    return t * margin


def leg_segment(t0: float, T: float, p0: np.ndarray, p1: np.ndarray) -> Segment:
    """Quintic min-jerk straight leg from p0 to p1 over duration T starting at t0."""
    d = np.asarray(p1, float) - np.asarray(p0, float)
    c = np.zeros((6, 3))
    c[0] = p0
    c[3] = 10 * d / T**3
    c[4] = -15 * d / T**4
    c[5] = 6 * d / T**5
    return Segment(t0, t0 + T, c)


def hold_segment(t0: float, T: float, p: np.ndarray) -> Segment:
    return Segment(t0, t0 + T, np.array([np.asarray(p, float)]))


def path_segments(t0: float, waypoints: list[np.ndarray], lim: dict) -> list[Segment]:
    """Chain of rest-to-rest legs through waypoints, each at its minimum feasible time."""
    segs, t = [], t0
    for a, b in zip(waypoints[:-1], waypoints[1:]):
        T = min_leg_time(b - a, lim)
        if T <= 0:
            continue
        segs.append(leg_segment(t, T, a, b))
        t += T
    return segs


def path_duration(waypoints: list[np.ndarray], lim: dict) -> float:
    return sum(min_leg_time(b - a, lim) for a, b in zip(waypoints[:-1], waypoints[1:]))


def stitch(per_drone_pieces: list[list[Segment]], t_end: float) -> list[Trajectory]:
    """Pad each drone's pieces with holds so all are contiguous from t=0 to t_end."""
    out = []
    for pieces in per_drone_pieces:
        segs: list[Segment] = []
        t = 0.0
        pos = pieces[0].start()
        for s in pieces:
            if s.t0 > t + 1e-12:
                segs.append(hold_segment(t, s.t0 - t, pos))
            segs.append(s)
            t = s.t1
            pos = s.end()
        if t < t_end - 1e-12:
            segs.append(hold_segment(t, t_end - t, pos))
        out.append(Trajectory(segs))
    return out
