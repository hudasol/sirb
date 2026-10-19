import time

import numpy as np
import pytest
from hypothesis import HealthCheck, given, settings
from hypothesis import strategies as st

from sirb.spec import DEFAULT_LIMITS
from sirb.trajectory import Fleet, Segment, Trajectory
from sirb.verify import pair_minima, pairwise_min_separation, verify

LIM = DEFAULT_LIMITS
BIG = 1e9


def lin(p, v, t0, t1):
    return Segment(t0, t1, np.array([p, v], float))


def traj(*segs):
    return Trajectory(list(segs))


def quintic(p0, p1, t0, T):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = p1 - p0
    c = np.zeros((6, 3))
    c[0] = p0
    c[3] = 10 * d / T**3
    c[4] = -15 * d / T**4
    c[5] = 6 * d / T**5
    return Segment(t0, t0 + T, c)


def kinds(rep):
    return {v.kind for v in rep.violations}


def rnd_seg(rng, deg, t0, T, scale=5.0):
    c = rng.uniform(-scale, scale, (deg + 1, 3)) / (1 + np.arange(deg + 1))[:, None] ** 1.5
    return Segment(t0, t0 + T, c)


# ---------------------------------------------------------------- (1) vs sampling
@settings(max_examples=120, deadline=None, suppress_health_check=list(HealthCheck))
@given(seed=st.integers(0, 2**32 - 1), da=st.integers(1, 7), db=st.integers(1, 7),
       T=st.floats(0.3, 4.0))
def test_exact_min_vs_bruteforce(seed, da, db, T):
    rng = np.random.default_rng(seed)
    a, b = rnd_seg(rng, da, 0.0, T), rnd_seg(rng, db, 0.0, T)
    fleet = Fleet([traj(a), traj(b)])
    d, i, j, t, lb, n = pairwise_min_separation(fleet, BIG)
    assert (i, j) == (0, 1) and not lb and n == 1
    N = 200_000
    ts = np.linspace(0, T, N)
    samp = np.linalg.norm(a.pos(ts) - b.pos(ts), axis=1).min()
    D = a.coeffs.copy()
    D = np.zeros((max(da, db) + 1, 3)); D[:da + 1] += a.coeffs; D[:db + 1] -= b.coeffs
    Lip = sum(k * np.linalg.norm(D[k]) * T ** (k - 1) for k in range(1, len(D)))
    step_bound = Lip * (T / (N - 1)) / 2
    assert d <= samp + 1e-9
    assert samp <= d + step_bound + 1e-9
    assert 0 <= t <= T
    assert abs(np.linalg.norm(a.pos(t) - b.pos(t)) - d) < 1e-9


# ---------------------------------------------------------------- (2) analytic
def test_head_on_linear():
    f = Fleet([traj(lin([-10, 0, 10], [5, 0, 0], 0, 4)), traj(lin([10, 0.5, 10], [-5, 0, 0], 0, 4))])
    d, i, j, t, lb, _ = pairwise_min_separation(f, 100)
    assert d == pytest.approx(0.5, abs=1e-12) and t == pytest.approx(2.0, abs=1e-9) and not lb


def test_exact_collision_zero_distance():
    f = Fleet([traj(lin([-10, 0, 5], [5, 0, 0], 0, 4)), traj(lin([10, 0, 5], [-5, 0, 0], 0, 4))])
    d, _, _, t, _, _ = pairwise_min_separation(f, 100)
    assert d < 1e-9 and t == pytest.approx(2.0, abs=1e-6)


def test_crossing_known_time():
    # A east along y=0, B north along x=3; closest at t where both near (3,0)
    a = traj(lin([-2, 0, 5], [1, 0, 0], 0, 10))     # x = -2 + t
    b = traj(lin([3, -4, 5], [0, 1, 0], 0, 10))     # y = -4 + t
    d, _, _, t, _, _ = pairwise_min_separation(Fleet([a, b]), 100)
    # relative: (-5+t, 4-t): min at t=4.5, distance sqrt(0.5)
    assert t == pytest.approx(4.5, abs=1e-9) and d == pytest.approx(np.sqrt(0.5), abs=1e-12)


def test_grazing_between_samples():
    # Relative path passes at exactly h at t=0.5; coarse sampling cannot see it.
    h = 2.0000
    f = Fleet([traj(lin([0, 0, 10], [0, 0, 0], 0, 1)), traj(lin([-10, h, 10], [20, 0, 0], 0, 1))])
    d, _, _, t, _, _ = pairwise_min_separation(f, 100)
    assert d == pytest.approx(h, abs=1e-12) and t == pytest.approx(0.5, abs=1e-9)
    ts = np.arange(0, 1.0001, 0.37)
    coarse = np.linalg.norm(f.sample(ts)[:, 0] - f.sample(ts)[:, 1], axis=1).min()
    assert coarse > h + 1.0   # sampling badly overestimates; exact roots do not


def test_grazing_cubic_vs_scipy():
    from scipy.optimize import minimize_scalar
    c1 = np.array([[0, 0, 5], [1.3, -0.4, 0], [-0.9, 0.6, 0], [0.25, 0.1, 0]], float)
    c2 = np.array([[2, 2.2, 5], [-0.2, 0.3, 0], [0.5, -0.8, 0], [0.1, 0.2, 0]], float)
    a, b = Segment(0, 3, c1), Segment(0, 3, c2)
    d, _, _, t, _, _ = pairwise_min_separation(Fleet([traj(a), traj(b)]), 100)
    ts = np.linspace(0, 3, 3001)
    t0 = ts[np.argmin(np.linalg.norm(a.pos(ts) - b.pos(ts), axis=1))]
    r = minimize_scalar(lambda x: np.linalg.norm(a.pos(x) - b.pos(x)),
                        bounds=(max(0, t0 - 0.01), min(3, t0 + 0.01)), method="bounded",
                        options={"xatol": 1e-12})
    assert d <= r.fun + 1e-9 and d == pytest.approx(r.fun, abs=1e-6)


def test_circle_approximations():
    from math import factorial
    def circ(r, phase, w, deg=7):
        c = np.zeros((deg + 1, 3))
        for k in range(deg + 1):
            c[k, 0] = r * w**k * np.cos(phase + k * np.pi / 2) / factorial(k)
            c[k, 1] = r * w**k * np.sin(phase + k * np.pi / 2) / factorial(k)
            c[0, 2] = 10
        return c
    T = 1.0
    a = Segment(0, T, circ(10.0, 0.0, 0.8))
    b = Segment(0, T, circ(13.0, 0.0, 0.8))
    d, _, _, t, _, _ = pairwise_min_separation(Fleet([traj(a), traj(b)]), 100)
    assert d == pytest.approx(3.0, abs=1e-5)       # concentric, same phase: radial gap
    # opposite phase, crossing the centre line: gap 2r at every time
    c2 = Segment(0, T, circ(10.0, np.pi, 0.8))
    d2, *_ = pairwise_min_separation(Fleet([traj(a), traj(c2)]), 100)
    assert d2 == pytest.approx(20.0, abs=1e-4)


# ---------------------------------------------------------------- (3) different spans
def test_landed_drone_and_passer():
    landed = traj(quintic([0, 0, 5], [20, 0, 0.0], 0, 8))   # lands at (20,0,0) at t=8
    passer = traj(lin([20, 0.8, 0], [0, 0, 0], 0, 1), lin([20, 0.8, 0], [0, 0, 0], 1, 30))
    # passer sits near the landing point the whole time -> conflict after t=8
    f = Fleet([landed, passer])
    r = verify(f, LIM)
    assert not r.ok and "separation" in kinds(r)
    v = [x for x in r.violations if x.kind == "separation"][0]
    assert v.value == pytest.approx(0.8, abs=1e-9)
    # The distance reaches 0.8 m at t = 8 and stays there, and the minimum jerk landing makes it
    # flat before that: at t = 8 - 0.04 the excess over 0.8 m is below 1e-9 m (offset about
    # 10 L (0.04/8)^3 with L = 20.6 m). Any time on that plateau is a correct minimiser, and which
    # one the root finder returns differs between platforms in the last bits, so check that the
    # reported time is a minimiser of the true distance and lies at the landing.
    gap = np.linalg.norm(landed.pos(v.t) - passer.pos(v.t))
    assert gap == pytest.approx(0.8, abs=1e-9) and v.t >= 8 - 0.04


def test_mover_passes_parked_drone_after_all_end():
    parked = traj(lin([0, 0, 5], [0, 0, 0], 0, 2))
    mover = traj(lin([-50, 1.0, 5], [0, 0, 0], 5, 6),
                 lin([-50, 1.0, 5], [10, 0, 0], 6, 20))
    r = verify(Fleet([parked, mover]), {**LIM, "vmax_h": 20})
    d = [v for v in r.violations if v.kind == "separation"]
    assert d and d[0].value == pytest.approx(1.0, abs=1e-9) and d[0].t == pytest.approx(11.0, abs=1e-6)


def test_before_first_segment_start_position_occupied():
    late = traj(lin([0, 0, 5], [0, 0, 0], 10, 12))     # sits at (0,0,5) from -inf
    early = traj(lin([-5, 0.5, 5], [5, 0, 0], 0, 2))   # passes x=0 at t=1
    r = verify(Fleet([late, early]), LIM)
    d = [v for v in r.violations if v.kind == "separation"]
    assert d and d[0].value == pytest.approx(0.5, abs=1e-9) and d[0].t == pytest.approx(1.0, abs=1e-6)


# ---------------------------------------------------------------- (4) prefilter
def rnd_traj(rng, nseg, tstart, spread):
    t = tstart
    segs = []
    for _ in range(nseg):
        T = rng.uniform(0.5, 3.0)
        deg = int(rng.integers(1, 8))
        s = rnd_seg(rng, deg, t, T, scale=spread)
        s.coeffs[0] += rng.uniform(-spread, spread, 3)
        segs.append(s)
        t += T
    return Trajectory(segs)


@settings(max_examples=60, deadline=None, suppress_health_check=list(HealthCheck))
@given(seed=st.integers(0, 2**32 - 1), n=st.integers(2, 7), R=st.floats(0.5, 8.0))
def test_prefilter_equals_bruteforce(seed, n, R):
    rng = np.random.default_rng(seed)
    f = Fleet([rnd_traj(rng, int(rng.integers(1, 4)), rng.uniform(0, 3), 6.0) for _ in range(n)])
    pre, npre = pair_minima(f, R, prefilter=True)
    full, nfull = pair_minima(f, R, prefilter=False)
    assert set(pre) == set(full)
    for k in pre:
        assert pre[k][0] == pytest.approx(full[k][0], abs=1e-9)
    assert npre <= nfull
    # and against dense sampling of the whole fleet (different time spans, clipped = held)
    ts = np.linspace(-1, max(t.t_end for t in f.trajectories) + 1, 60001)
    X = f.sample(ts)
    for (i, j), (d, _) in full.items():
        samp = np.linalg.norm(X[:, i] - X[:, j], axis=1).min()
        assert d <= samp + 1e-9
    for i in range(n):
        for j in range(i + 1, n):
            samp = np.linalg.norm(X[:, i] - X[:, j], axis=1).min()
            if samp < R - 0.5 and (i, j) not in full:
                raise AssertionError("pair missed")


# ---------------------------------------------------------------- (5) corrupted
def clean_pair():
    a = traj(quintic([0, 0, 20], [20, 0, 20], 0, 10), quintic([20, 0, 20], [20, 20, 20], 10, 10))
    b = traj(quintic([0, 10, 25], [20, 10, 25], 0, 10), quintic([20, 10, 25], [20, 30, 25], 10, 10))
    return a, b


def test_clean_passes():
    r = verify(Fleet(list(clean_pair())), LIM)
    assert r.ok, r.violations
    assert r.min_separation == pytest.approx(min(5.0, 10.0), abs=1e-9) or r.min_separation_is_lower_bound or r.min_separation >= 2


def test_teleport_jump():
    a = traj(quintic([0, 0, 20], [20, 0, 20], 0, 10), quintic([21, 0, 20], [21, 20, 20], 10, 10))
    r = verify(Fleet([a]), LIM)
    assert "position_jump" in kinds(r) and not r.ok


def test_velocity_discontinuity():
    a = traj(lin([0, 0, 20], [2, 0, 0], 0, 5), lin([10, 0, 20], [-2, 0, 0], 5, 10))
    r = verify(Fleet([a]), LIM)
    assert kinds(r) == {"velocity_discontinuity"}


def test_speed_limits():
    r = verify(Fleet([traj(quintic([0, 0, 20], [100, 0, 20], 0, 5))]), LIM)
    assert "speed_h" in kinds(r)
    r = verify(Fleet([traj(quintic([0, 0, 10], [0, 0, 70], 0, 5))]), LIM)
    assert "speed_up" in kinds(r)
    r = verify(Fleet([traj(quintic([0, 0, 70], [0, 0, 10], 0, 5))]), LIM)
    assert "speed_down" in kinds(r)


def test_accel_only_and_jerk_only():
    r = verify(Fleet([traj(quintic([0, 0, 20], [5, 0, 20], 0, 1.0))]), LIM)
    assert "accel" in kinds(r) and "speed_h" not in kinds(r)
    r = verify(Fleet([traj(quintic([0, 0, 20], [6, 0, 20], 0, 3.0))]), LIM)
    assert kinds(r) == {"jerk"}, r.violations


def test_altitude_geofence_crowd():
    r = verify(Fleet([traj(quintic([0, 0, 100], [0, 0, 130], 0, 20))]), LIM)
    assert "altitude" in kinds(r)
    r = verify(Fleet([traj(quintic([0, 0, -5], [0, 0, 5], 0, 20))]), LIM)
    assert "altitude" in kinds(r)
    r = verify(Fleet([traj(quintic([100, 0, 20], [180, 0, 20], 0, 20))]), LIM)
    assert "geofence" in kinds(r)
    r = verify(Fleet([traj(quintic([0, -150, 20], [0, -210, 20], 0, 30))]), {**LIM, "geofence": {"x": [-150, 150], "y": [-300, 100]}})
    assert "crowd" in kinds(r)
    ok = verify(Fleet([traj(quintic([0, -150, 20], [0, -190, 20], 0, 30))]), {**LIM, "geofence": {"x": [-150, 150], "y": [-300, 100]}})
    assert "crowd" not in kinds(ok)


def test_overshoot_inside_segment_found():
    # endpoints inside the geofence but the polynomial bulges out in the middle
    c = np.zeros((3, 3)); c[:, 2] = [20, 0, 0]
    c[:, 0] = [0, 90, -60]     # x = 60 tau (1.5 - tau): 0 at both ends, 33.75 at tau=0.75
    seg = Segment(0, 1.5, c)
    x = lambda t: np.polyval(c[::-1, 0], t)
    assert max(x(np.linspace(0, 1.5, 10001))) > 30
    lim = {**LIM, "geofence": {"x": [-150, 29.5], "y": [-130, 100]}, "vmax_h": 1e3, "amax": 1e6, "jmax": 1e9}
    r = verify(Fleet([traj(seg)]), lim)
    assert "geofence" in kinds(r)


def test_time_gap_and_overlap():
    a = traj(lin([0, 0, 20], [1, 0, 0], 0, 5), lin([5, 0, 20], [1, 0, 0], 5.5, 10))
    assert "time_gap" in kinds(verify(Fleet([a]), LIM))
    b = traj(lin([0, 0, 20], [1, 0, 0], 0, 5), lin([4.5, 0, 20], [1, 0, 0], 4.5, 10))
    assert "time_gap" in kinds(verify(Fleet([b]), LIM))


def test_nan_coefficients_never_crash():
    c = np.array([[0, 0, 20], [np.nan, 0, 0]], float)
    other = traj(lin([5, 0, 20], [0, 0, 0], 0, 5))
    r = verify(Fleet([traj(Segment(0, 5, c)), other]), LIM)
    assert "nonfinite" in kinds(r) and not r.ok
    r = verify(Fleet([traj(Segment(0, np.inf, np.zeros((2, 3)))), other]), LIM)
    assert "nonfinite" in kinds(r)
    r = verify(Fleet([Trajectory([]), other]), LIM)
    assert not r.ok


def test_collision_course():
    a = traj(lin([-10, 0, 20], [2, 0, 0], 0, 10))
    b = traj(lin([10, 0, 20], [-2, 0, 0], 0, 10))
    r = verify(Fleet([a, b]), LIM)
    v = [x for x in r.violations if x.kind == "separation"]
    assert len(v) == 1 and v[0].drones == (0, 1) and v[0].t == pytest.approx(5.0, abs=1e-6)
    assert v[0].value < 1e-6


@pytest.mark.parametrize("off,expect_flag", [(1.99, True), (2.01, False), (1.999999, True), (2.000001, False)])
def test_grazing_threshold(off, expect_flag):
    a = traj(lin([-20, 0, 20], [5, 0, 0], 0, 8))
    b = traj(lin([20, off, 20], [-5, 0, 0], 0, 8))
    r = verify(Fleet([a, b]), LIM, tol=1e-9)
    assert ("separation" in kinds(r)) == expect_flag
    assert r.min_separation == pytest.approx(off, abs=1e-9)


def test_every_violating_pair_listed():
    ds = [traj(lin([0, 3 * i, 20], [0, 0, 0], 0, 5)) for i in range(5)]   # spacing 3 -> ok
    assert verify(Fleet(ds), LIM).ok
    ds = [traj(lin([0, 1.5 * i, 20], [0, 0, 0], 0, 5)) for i in range(5)]  # neighbours 1.5, next 3.0
    r = verify(Fleet(ds), LIM)
    sv = [v for v in r.violations if v.kind == "separation"]
    assert sorted(v.drones for v in sv) == [(0, 1), (1, 2), (2, 3), (3, 4)]


def test_no_pair_below_radius_is_lower_bound():
    a = traj(lin([0, 0, 20], [0, 0, 0], 0, 5))
    b = traj(lin([100, 0, 20], [0, 0, 0], 0, 5))
    d, i, j, t, lb, n = pairwise_min_separation(Fleet([a, b]), 8.0)
    assert d == 8.0 and lb and i is None and t is None
    r = verify(Fleet([a, b]), LIM)
    assert r.ok and r.min_separation_is_lower_bound


# ---------------------------------------------------------------- (6),(7) fleets
def grid_fleet(nx, ny, spacing=3.0, T=15.0, grow=1.3):
    trs = []
    for ix in range(nx):
        for iy in range(ny):
            p0 = np.array([(ix - nx / 2) * spacing, iy * spacing, 20.0])
            p1 = np.array([p0[0] * grow, p0[1] * grow, 20.0 + (iy % 3) * 0.0])
            trs.append(Trajectory([quintic(p0, p1, 0.0, T)]))
    return Fleet(trs)


def test_clean_synthetic_fleet_passes():
    r = verify(grid_fleet(6, 5), LIM)
    assert r.ok, r.violations[:3]
    assert r.min_separation == pytest.approx(3.0, abs=1e-6)


def test_1000_drone_fleet():
    f = grid_fleet(40, 25)
    assert f.n == 1000
    t = time.perf_counter()
    r = verify(f, LIM)
    wall = time.perf_counter() - t
    print(f"\n1000-drone verify: {wall:.2f}s, pairs checked {r.n_pairs_checked}, sep {r.min_separation:.4f}")
    assert wall < 60
    assert r.ok, r.violations[:3]
    assert r.min_separation == pytest.approx(3.0, abs=1e-6)
    assert r.n_pairs_checked > 0


def test_1000_drone_fleet_with_one_collision_found():
    f = grid_fleet(40, 25)
    # move drone 500 onto drone 501's path
    f.trajectories[500] = Trajectory([quintic(f.trajectories[501].segments[0].coeffs[0] + [0.5, 0, 0],
                                              f.trajectories[501].segments[0].end() + [0.5, 0, 0], 0.0, 15.0)])
    r = verify(f, LIM)
    sv = [v for v in r.violations if v.kind == "separation"]
    assert any(v.drones == (500, 501) and v.value == pytest.approx(0.5, abs=1e-6) for v in sv)
