"""Tests added after triaging the survivors of the mutation run on ``sirb/assign.py`` (phase 6).

The survivors that are not equivalent were: the default seeds of ``assign_random``,
``assign_pinned`` and ``evaluate``; the handling of slow relative motion in ``linear_conflicts``
(the divisor guard and its threshold); the value used for (almost) parallel motion; and the
floating-point error filter around the division. Expected numbers are derived by hand.
"""

from __future__ import annotations

import warnings

import numpy as np

from sirb import assign as A


def _cloud(n, seed):
    rng = np.random.default_rng(seed)
    return rng.uniform(0, 20, size=(n, 3)), rng.uniform(0, 20, size=(n, 3))


def test_assign_random_default_seed_is_zero():
    src, dst = _cloud(40, 1)
    assert A.assign_random(src, dst).tolist() == np.random.default_rng(0).permutation(40).tolist()
    assert A.assign_random(src, dst).tolist() != np.random.default_rng(1).permutation(40).tolist()


def test_assign_pinned_default_seed_is_zero():
    src, dst = _cloud(100, 2)
    p0 = A.assign_pinned(src, dst, 0)
    assert A.assign_pinned(src, dst).tolist() == p0.tolist()
    assert A.assign_pinned(src, dst).tolist() != A.assign_pinned(src, dst, 1).tolist()


def test_evaluate_default_seed_is_zero():
    src, dst = _cloud(30, 3)
    default = A.evaluate(src, dst, 1.0)
    assert default == A.evaluate(src, dst, 1.0, 0)
    # the random baseline of seed 0: its path length computed independently
    perm0 = np.random.default_rng(0).permutation(30)
    expect = float(np.linalg.norm(dst[perm0] - src, axis=1).sum())
    assert abs(default["random"]["total_path_length_m"] - expect) < 1e-9
    perm1 = np.random.default_rng(1).permutation(30)
    other = float(np.linalg.norm(dst[perm1] - src, axis=1).sum())
    assert abs(default["random"]["total_path_length_m"] - other) > 1e-3


def test_linear_conflicts_slow_relative_motion():
    # A moves (0,0,0) -> (0.5,0,0); B stays at (0.25,-0.1,0).  Relative position
    # dp + u dv = (-0.25 + 0.5 u, 0.1, 0): closest at u = 1/2, distance 0.1.  |dv|^2 = 0.25 <= 1.
    src = np.array([[0.0, 0.0, 0.0], [0.25, -0.1, 0.0]])
    dst = np.array([[0.5, 0.0, 0.0], [0.25, -0.1, 0.0]])
    perm = np.array([0, 1])
    assert A.linear_conflicts(src, dst, perm, 0.2) == 1       # 0.1 < 0.2
    assert A.linear_conflicts(src, dst, perm, 0.05) == 0      # 0.1 > 0.05
    # the start-to-end distances are 0.269 (start) and 0.269 (end): only the interior minimum counts
    assert A.linear_conflicts(src, dst, perm, 0.15) == 1


def test_linear_conflicts_almost_parallel_motion_is_evaluated_at_the_start():
    # |dv|^2 = 2.5e-19 <= 1e-18: treated as no relative motion, the distance is the start distance
    # exactly 1.  (Evaluating the end of the path instead would give 1 - 1e-9 < 1.)
    src = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    dst = np.array([[5e-10, 0.0, 0.0], [1.0, 0.0, 0.0]])
    perm = np.array([0, 1])
    assert A.linear_conflicts(src, dst, perm, 1.0) == 0
    assert A.linear_conflicts(src, dst, perm, 1.0 + 1e-6) == 1


def test_linear_conflicts_numpy_error_filter_hides_invalid_division():
    # Overflowing displacements make the closest-approach parameter inf / inf inside the guarded
    # division; the 'invalid' flag must be silenced there (overflow itself is silenced by the caller).
    src = np.array([[1e200, 0.0, 0.0], [0.0, 0.0, 0.0]])
    dst = np.array([[2e200, 0.0, 0.0], [0.0, 0.0, 0.0]])
    perm = np.array([0, 1])
    with np.errstate(over="ignore"), warnings.catch_warnings():
        warnings.simplefilter("error")
        n = A.linear_conflicts(src, dst, perm, 1.0)
    assert n == 0
