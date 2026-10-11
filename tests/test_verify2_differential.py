"""Differential tests: the exact verifier (sirb.verify) against the independent interval
branch-and-bound verifier (sirb.verify2). Fixed seeds. The default tier runs in well under a
minute; set SIRB_BIG=1 for the larger tier (the full tier is scripts/differential_report.py).

Agreement criteria C1 to C4 are defined in sirb/diffcases.py and docs/VERIFIER.md.
"""

import os

import numpy as np
import pytest

import sirb.verify2 as v2
from sirb.diffcases import (
    ADVERSARIAL,
    adversarial_case,
    compare,
    exact_pairs,
    random_fleet,
    random_large_fleet,
)
from sirb.trajectory import Fleet, Segment, Trajectory
from sirb.verify2 import verify2

BIG = bool(os.environ.get("SIRB_BIG"))
N_RANDOM = 4000 if BIG else 300
N_ADV = 150 if BIG else 12          # seeds per adversarial family

# Families where the exact verifier is known to lose accuracy: the minimum is a root of
# multiplicity up to 2m-1 of d|D|^2/dt, so eigenvalue error ~ eps^(1/(2m-1)) moves the
# candidate away from the flat minimum. The deviation stays below the verifier's own 1e-6
# tolerance; it must never become a verdict contradiction or a bound violation.
CONDITIONING_FAMILIES = {"degenerate_collision"}
CONDITIONING_KINDS = {"C2_gap_too_wide", "C3_witness_below_exact", "exact_vs_construction"}


def _seg(t0, t1, *rows):
    return Segment(float(t0), float(t1), np.array(rows, float))


def _check(fleet, d_min=None, d_true=None, seed=0):
    return compare(fleet, d_min, d_true, np.random.default_rng(seed))


@pytest.mark.parametrize("seed", range(N_RANDOM))
def test_random_fleets_degree_1_to_5(seed):
    fleet, d_min, meta = random_fleet(seed)
    stats, dis = _check(fleet, d_min, None, seed)
    assert not dis, dis


@pytest.mark.parametrize("family", sorted(ADVERSARIAL))
def test_adversarial_families(family):
    bad = []
    for seed in range(N_ADV):
        fleet, d_min, meta = adversarial_case(family, seed)
        stats, dis = _check(fleet, d_min, meta.get("d_true"), seed)
        bad += [(seed, d) for d in dis]
    if family in CONDITIONING_FAMILIES:
        for seed, d in bad:
            assert d["kind"] in CONDITIONING_KINDS, (seed, d)
            assert abs(d["diff"]) < 1e-6, (seed, d)
    else:
        assert not bad, bad[:3]


def test_verdict_both_sides_of_threshold():
    # Two static drones exactly 3 m apart on an axis.
    f = Fleet([Trajectory([_seg(0, 1, [0, 0, 0])]), Trajectory([_seg(0, 1, [3, 0, 0])])])
    assert verify2(f, 3.0 - 1e-6).verdict == "PASS"
    assert verify2(f, 3.0 + 1e-6).verdict == "FAIL"
    r = verify2(f, 3.0)
    assert r.verdict in ("PASS", "UNDECIDED")      # never FAIL at equality
    assert r.lower_bound <= 3.0 + 1e-9
    assert r.upper_bound >= 3.0 - 1e-9


def test_hold_semantics_match():
    # B exists only later; A is parked: the minimum is between A's parked point and B's path.
    a = Trajectory([_seg(0, 1, [0, 0, 0])])
    b = Trajectory([_seg(5, 6, [4, 3, 0], [-1, 0, 0])])
    f = Fleet([a, b])
    e = exact_pairs(f)[(0, 1)][0]
    r = verify2(f, 1.0, cap=1e9, per_pair=True)
    assert r.pair_lower[(0, 1)] <= e + 1e-9
    assert e - r.pair_lower[(0, 1)] <= r.gap_bound
    assert abs(e - 18 ** 0.5) < 1e-9     # B ends at (3, 3, 0); A stays at the origin


def test_no_shared_code_with_verify():
    src = open(v2.__file__).read()
    assert "sirb.verify " not in src and "from sirb.verify import" not in src
    assert "import sirb.verify\n" not in src


def test_harness_detects_a_broken_second_verifier(monkeypatch):
    """Mutation check of the harness itself: an over-optimistic lower bound must be caught."""
    real = v2._cell_bounds

    def broken(D, pid, mid, w, E, gamma):
        lbd, ub, ue = real(D, pid, mid, w, E, gamma)
        return lbd + 0.05, ub, ue

    monkeypatch.setattr(v2, "_cell_bounds", broken)
    found = 0
    for seed in range(20):
        fleet, d_min, meta = adversarial_case("graze_linear", seed)
        _, dis = _check(fleet, d_min, meta.get("d_true"), seed)
        found += bool(dis)
    assert found >= 15


def test_hundred_drone_fleet_is_fast():
    fleet, _, _ = random_large_fleet(0, n=100)
    r = verify2(fleet, 1.0)
    assert r.runtime_s < 20.0
    e = min(v[0] for v in exact_pairs(fleet).values())
    assert r.lower_bound <= e + 1e-9
    assert e - r.lower_bound <= r.gap_bound or r.lower_bound >= r.cap - 1e-12


@pytest.mark.skipif(not BIG, reason="set SIRB_BIG=1")
@pytest.mark.parametrize("seed", range(50))
def test_larger_fleets_big_tier(seed):
    fleet, _, _ = random_large_fleet(seed, n=12)
    _, dis = _check(fleet, None, None, seed)
    assert not dis, dis
