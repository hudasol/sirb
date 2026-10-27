"""Case generators and comparison logic for the verify / verify2 differential tests.

Used by ``tests/test_verify2_differential.py`` and ``scripts/differential_report.py``. Every
generator takes a seed (``numpy.random.default_rng``), so every case is reproducible from
``(family, seed)``. Adversarial families build a pair of drones whose true minimum
separation is known analytically (``d_true``), so the exact verifier can be checked against
construction as well as against the second verifier.

Agreement (see ``docs/VERIFIER.md``): with ``e`` the exact minimum from ``sirb.verify``,
``l`` / ``u`` the lower bound and the witnessed distance from ``sirb.verify2``,

* C1  ``l <= e + 1e-9``;
* C2  if no cell was left undecided, ``e - l <= gap_bound + 1e-9`` (verify2's own ``tol`` plus
  rounding allowance, plus the 1e-9 accuracy the exact verifier claims for itself, the same band
  as in C1 and C3: ``e - l = (e - true) + (true - l)``);
* C3  ``u >= e - 1e-9`` (a witnessed distance is a real distance, so it cannot beat the minimum);
* C4  no verdict contradiction outside +/-1e-6 m of the threshold: verify2 PASS with
  ``e < d_min - 1e-6``, or verify2 FAIL with ``e > d_min + 1e-6``.
"""

from __future__ import annotations

import math

import numpy as np

from sirb.trajectory import Fleet, Segment, Trajectory
from sirb.verify import pair_minima
from sirb.verify2 import verify2

BIG = 1e9
AGREE = 1e-9          # allowed slack for C1 and C3
VERDICT_BAND = 1e-6   # C4 band
DELTAS = (-1e-4, -1e-6, 0.0, 1e-6, 1e-4)


# ------------------------------------------------------------------ small helpers

def _unit(rng):
    v = rng.normal(size=3)
    return v / np.linalg.norm(v)


def _perp(rng, n):
    v = rng.normal(size=3)
    v -= n * (v @ n)
    return v / np.linalg.norm(v)


def _seg(t0, t1, coeffs):
    return Segment(float(t0), float(t1), np.array(coeffs, dtype=float))


def _line(p_at_t, vel, t_ref, t0, t1):
    """Linear segment through p_at_t at t_ref with velocity vel, on [t0, t1]."""
    c0 = np.asarray(p_at_t, float) + np.asarray(vel, float) * (t0 - t_ref)
    return _seg(t0, t1, [c0, vel])


def _shift_coeffs(c, d):
    """Coefficients of c(d + r) in r, ascending, c (K, 3) (own Horner shift)."""
    c = np.array(c, dtype=float)
    K = c.shape[0]
    for i in range(K - 1):
        for j in range(K - 2, i - 1, -1):
            c[j] += d * c[j + 1]
    return c


def _static(p, t0, t1):
    return _seg(t0, t1, [p])


def translate(fleet, off):
    off = np.asarray(off, float)
    out = []
    for tr in fleet.trajectories:
        segs = []
        for s in tr.segments:
            c = s.coeffs.copy()
            c[0] += off
            segs.append(Segment(s.t0, s.t1, c))
        out.append(Trajectory(segs))
    return Fleet(out)


def _fleet(a_segs, b_segs):
    return Fleet([Trajectory(a_segs), Trajectory(b_segs)])


# ------------------------------------------------------------------ random fleets

def rand_poly(rng, deg, scale=5.0):
    return rng.uniform(-scale, scale, (deg + 1, 3)) / (1 + np.arange(deg + 1))[:, None] ** 1.5


def random_fleet(seed: int, clean: bool = False):
    """Random polynomial fleet: 2-5 drones, 1-4 segments, degree 1-5; about half are C0
    continuous, some have gaps or a deliberate overlap, some start late. ``clean=True`` forces
    C0 continuity and no overlaps (gaps are kept: the drone holds), same random draws."""
    rng = np.random.default_rng(seed)
    n = int(rng.integers(2, 6))
    cont = rng.random() < 0.5 or clean
    trs = []
    for _ in range(n):
        t = float(rng.uniform(0.0, 1.0)) if rng.random() < 0.3 else 0.0
        segs = []
        prev_end = None
        for _k in range(int(rng.integers(1, 5))):
            T = float(rng.uniform(0.3, 2.0))
            c = rand_poly(rng, int(rng.integers(1, 6)))
            if cont and prev_end is not None:
                c[0] = prev_end
            r = rng.random()
            if segs and r < 0.06:
                t += float(rng.uniform(0.05, 0.5))          # gap (held)
            elif segs and r < 0.10 and not clean:
                t -= min(0.1, 0.5 * (segs[-1].t1 - segs[-1].t0))   # overlap
            s = Segment(t, t + T, c)
            segs.append(s)
            prev_end = s.end()
            t += T
        trs.append(Trajectory(segs))
    return Fleet(trs), None, {"n": n, "continuous": cont}


def random_large_fleet(seed: int, n: int = 12, spread: float = 6.0):
    """Larger random fleet, continuous, for the bigger tier."""
    rng = np.random.default_rng(seed)
    trs = []
    for _ in range(n):
        t = 0.0
        segs = []
        pos = rng.uniform(-spread, spread, 3)
        for _k in range(int(rng.integers(2, 6))):
            T = float(rng.uniform(0.5, 2.0))
            c = rand_poly(rng, int(rng.integers(1, 6)), 4.0)
            c[0] = pos
            s = Segment(t, t + T, c)
            segs.append(s)
            pos = s.end()
            t += T
        trs.append(Trajectory(segs))
    return Fleet(trs), None, {"n": n}


# ------------------------------------------------------------------ adversarial families
# Each returns (fleet, d_min, meta) with meta["d_true"] the analytic minimum separation of the
# constructed pair (drones 0 and 1) when it is known.

def _base(rng, delta, lo=1.0, hi=5.0):
    d_min = float(rng.choice([1.0, 1.5, 2.0, 3.0, 5.0, rng.uniform(lo, hi)]))
    return d_min, d_min + delta


def graze_linear(seed, delta):
    rng = np.random.default_rng(seed)
    d_min, d = _base(rng, delta)
    n = _unit(rng)
    u = _perp(rng, n)
    p0 = rng.uniform(-10, 10, 3)
    v = float(rng.uniform(0.5, 10.0))
    T = float(rng.uniform(1.0, 6.0))
    ts = float(rng.uniform(0.2, 0.8) * T)
    a = [_static(p0, 0.0, T)]
    b = [_line(p0 + n * d, u * v, ts, 0.0, T)]
    return _fleet(a, b), d_min, {"d_true": d}


def graze_curved(seed, delta):
    """Relative motion D = v s u + (d + c s^2 [+ c4 s^4]) n, s = t - t*; min d at t* (c >= 0)."""
    rng = np.random.default_rng(seed)
    d_min, d = _base(rng, delta)
    n = _unit(rng)
    u = _perp(rng, n)
    p0 = rng.uniform(-10, 10, 3)
    v = float(rng.uniform(0.0, 6.0))
    c2 = float(rng.uniform(0.0, 3.0))
    c4 = float(rng.uniform(0.0, 1.0)) if rng.random() < 0.5 else 0.0
    T = float(rng.uniform(1.0, 4.0))
    ts = float(rng.uniform(0.3, 0.7) * T)
    # D(s) in s-polynomial form, then re-expand to tau = t - 0 (shift by -ts).
    cs = np.zeros((5, 3))
    cs[0] = d * n
    cs[1] = v * u
    cs[2] = c2 * n
    cs[4] = c4 * n
    c_t = _shift_coeffs(cs, -ts)           # polynomial in tau where s = tau - ts
    c_t[0] += p0
    a = [_static(p0, 0.0, T)]
    b = [_seg(0.0, T, c_t)]
    return _fleet(a, b), d_min, {"d_true": d}


def boundary_min(seed, delta):
    """Distance minimum exactly at a segment boundary (approach along the line, then turn)."""
    rng = np.random.default_rng(seed)
    d_min, d = _base(rng, delta)
    n = _unit(rng)
    u = _perp(rng, n)
    p0 = rng.uniform(-10, 10, 3)
    tb = float(rng.uniform(0.5, 3.0))
    T2 = float(rng.uniform(0.5, 3.0))
    v1 = float(rng.uniform(0.5, 8.0))
    phi = float(rng.uniform(0.0, math.pi))
    w = (np.cos(phi) * u + np.sin(phi) * n) * float(rng.uniform(0.5, 8.0))
    pb = p0 + n * d
    s1 = _line(pb, u * v1, tb, 0.0, tb)
    s2 = _seg(tb, tb + T2, [pb, w])
    return _fleet([_static(p0, 0.0, tb + T2)], [s1, s2]), d_min, {"d_true": d}


def split_smooth(seed, delta):
    """A smooth curved graze split at the minimum time into 2-3 segments (min at breakpoint)."""
    rng = np.random.default_rng(seed)
    d_min, d = _base(rng, delta)
    n = _unit(rng)
    u = _perp(rng, n)
    p0 = rng.uniform(-10, 10, 3)
    v = float(rng.uniform(0.0, 6.0))
    c2 = float(rng.uniform(0.2, 3.0))
    cs = np.zeros((4, 3))
    cs[0] = d * n
    cs[1] = v * u
    cs[2] = c2 * n
    cs[3] = float(rng.uniform(-0.5, 0.5)) * u
    tb = float(rng.uniform(0.5, 2.0))
    T2 = float(rng.uniform(0.5, 2.0))
    left = _shift_coeffs(cs, -tb)          # valid for tau in [0, tb]: s = tau - tb
    left[0] += p0
    right = cs.copy()
    right[0] += p0
    segs = [_seg(0.0, tb, left), _seg(tb, tb + T2, right)]
    if rng.random() < 0.5:                  # a third piece, split again later
        t3 = tb + T2
        c3 = _shift_coeffs(right, T2)
        segs.append(_seg(t3, t3 + 0.7, c3))
        tail = t3 + 0.7
    else:
        tail = tb + T2
    return _fleet([_static(p0, 0.0, tail)], segs), d_min, {"d_true": d}


def tiny_segments(seed, delta):
    """A straight graze passed through a chain of 1e-6 s segments around the minimum."""
    rng = np.random.default_rng(seed)
    d_min, d = _base(rng, delta)
    n = _unit(rng)
    u = _perp(rng, n)
    p0 = rng.uniform(-10, 10, 3)
    v = float(rng.uniform(0.5, 10.0))
    ts = float(rng.uniform(1.0, 3.0))
    k = int(rng.integers(4, 12))
    h = 1e-6
    edges = [0.0, ts - 0.5 * k * h]
    for j in range(k):
        edges.append(ts - 0.5 * k * h + (j + 1) * h)
    edges.append(edges[-1] + float(rng.uniform(0.5, 2.0)))
    b = [_line(p0 + n * d, u * v, ts, edges[j], edges[j + 1]) for j in range(len(edges) - 1)]
    a = [_static(p0, 0.0, edges[-1])]
    return _fleet(a, b), d_min, {"d_true": d}


def tiny_random(seed, delta):
    """Random C0 chain of 1e-6 s segments next to a normal drone (threshold set from e)."""
    rng = np.random.default_rng(seed)
    segs = []
    pos = rng.uniform(-1, 1, 3)
    t = 0.0
    for _ in range(int(rng.integers(5, 30))):
        c = rand_poly(rng, int(rng.integers(1, 6)), 1e-3)
        c[0] = pos
        s = Segment(t, t + 1e-6, c)
        segs.append(s)
        pos = s.end()
        t += 1e-6
    c = rand_poly(rng, 3)
    c[0] = rng.uniform(-1, 1, 3) * 0.5
    other = [Segment(0.0, 1.0, c)]
    return Fleet([Trajectory(segs), Trajectory(other)]), None, {"d_true": None}


def stationary_threshold(seed, delta):
    rng = np.random.default_rng(seed)
    d_min, d = _base(rng, delta)
    mode = int(rng.integers(0, 3))
    p0 = rng.uniform(-10, 10, 3)
    if mode == 0:
        n = np.zeros(3)
        n[int(rng.integers(0, 3))] = 1.0       # axis aligned: exactly representable offsets
    else:
        n = _unit(rng)
    p1 = p0 + n * d
    if mode == 2:   # different time spans: B parked, A passes the hold semantics only
        a = [_static(p0, 0.0, 1.0)]
        b = [_static(p1, 3.0, 4.0)]
    else:
        a = [_static(p0, 0.0, 1.0)]
        b = [_static(p1, 0.0, 1.0)]
    return _fleet(a, b), d_min, {"d_true": float(np.linalg.norm(p1 - p0))}


def degenerate_roots(seed, delta):
    """Relative x-motion with a (near) multiple root of degree 3-5, constant offset d in y.

    |D|^2 = Dx(t)^2 + d^2 and Dx has all its roots clustered, so the derivative of |D|^2 has a
    root of high multiplicity; the minimum is exactly d at a root inside the segment.
    """
    rng = np.random.default_rng(seed)
    d_min, d = _base(rng, delta)
    deg = int(rng.integers(3, 6))
    centre = float(rng.uniform(0.8, 1.5))
    spread = float(rng.choice([0.0, 1e-4, 1e-3, 1e-2]))
    roots = centre + spread * rng.uniform(-1, 1, deg)
    px = np.polynomial.polynomial.polyfromroots(roots)       # ascending, monic
    px = px * float(rng.uniform(0.5, 3.0))
    c = np.zeros((deg + 1, 3))
    c[:, 0] = px
    c[0, 1] = d
    p0 = rng.uniform(-5, 5, 3)
    T = centre + 1.0
    a = [_static(p0, 0.0, T)]
    cb = c.copy()
    cb[0] += p0
    return _fleet(a, [_seg(0.0, T, cb)]), d_min, {"d_true": d, "min_is_exact": True}


def parallel_double(seed, delta):
    """Nearly parallel relative motion (grazing angle theta) and a sextic-flat double root."""
    rng = np.random.default_rng(seed)
    d_min, d = _base(rng, delta)
    p0 = rng.uniform(-5, 5, 3)
    if rng.random() < 0.5:
        theta = float(rng.uniform(0.02, 0.2))
        ts = float(rng.uniform(2.0, 4.0))
        T = 2 * ts
        n = _unit(rng)
        u = _perp(rng, n)
        r = d / math.sin(theta)                 # |D0|
        D0 = r * (math.cos(theta) * u + math.sin(theta) * n)
        # D(t) = D0 - (D0 . u / ts) u t ... closest approach along the line direction.
        vel = -u * (r * math.cos(theta) / ts)
        cb = np.array([D0, vel])
        return (_fleet([_static(p0, 0.0, T)], [_seg(0.0, T, np.array([p0 + cb[0], cb[1]]))]),
                d_min, {"d_true": d})
    n = _unit(rng)
    u = _perp(rng, n)
    ts = float(rng.uniform(1.0, 2.0))
    s = float(rng.uniform(0.5, 4.0))
    cs = np.zeros((7, 3))
    cs[0] = d * n
    cs[3] = s * u                                   # D = d n + s u (t - ts)^3 : P' ~ (t-ts)^5
    cs[6] = 0.0
    c_t = _shift_coeffs(cs[:4], -ts)
    c_t[0] += p0
    T = 2 * ts
    return _fleet([_static(p0, 0.0, T)], [_seg(0.0, T, c_t)]), d_min, {"d_true": d}


def degenerate_collision(seed, delta):
    """Same as degenerate_roots but with a tiny or zero offset: the drones (nearly) collide at
    a high-multiplicity root; d_min is a normal separation so the exact verdict is FAIL."""
    rng = np.random.default_rng(seed)
    d_min = float(rng.choice([1.0, 2.0, 5.0]))
    d = float(rng.choice([0.0, 1e-6, 1e-4, 1e-3]))
    fleet, _, meta = degenerate_roots(seed, 0.0)
    # rebuild with the new offset: degenerate_roots stores d in coefficient (0, 1) of drone 1
    segs = []
    for s in fleet.trajectories[1].segments:
        c = s.coeffs.copy()
        c[0, 1] += d - meta["d_true"]
        segs.append(Segment(s.t0, s.t1, c))
    return Fleet([fleet.trajectories[0], Trajectory(segs)]), d_min, {"d_true": d}


def long_high_degree(seed, delta):
    """Long segments (up to 25 s) of degree up to 7 for both drones (numerical stress)."""
    rng = np.random.default_rng(seed)
    trs = []
    for _ in range(2):
        T = float(rng.uniform(5.0, 25.0))
        c = rand_poly(rng, int(rng.integers(4, 8)), 30.0)
        c = c / (T ** np.arange(c.shape[0]))[:, None]      # keep excursions to tens of metres
        trs.append(Trajectory([Segment(0.0, T, c)]))
    return Fleet(trs), None, {"d_true": None}


ADVERSARIAL = {
    "graze_linear": graze_linear,
    "graze_curved": graze_curved,
    "boundary_min": boundary_min,
    "split_smooth": split_smooth,
    "tiny_segments": tiny_segments,
    "tiny_random": tiny_random,
    "stationary_threshold": stationary_threshold,
    "degenerate_roots": degenerate_roots,
    "parallel_double": parallel_double,
    "degenerate_collision": degenerate_collision,
    "long_high_degree": long_high_degree,
}


def adversarial_case(family: str, seed: int):
    """(fleet, d_min, meta) for family and seed; delta and optional 1e3 translation are
    derived from the seed, so the case is fully reproducible."""
    rng = np.random.default_rng(seed + 7919)
    delta = float(rng.choice(DELTAS))
    fleet, d_min, meta = ADVERSARIAL[family](seed, delta)
    meta = dict(meta)
    meta["delta"] = delta
    if rng.random() < 0.3:
        off = rng.uniform(-1e3, 1e3, 3)
        fleet = translate(fleet, off)
        meta["offset"] = off.tolist()
    # Occasionally add a distant bystander drone to exercise indexing.
    if rng.random() < 0.3:
        base = fleet.trajectories[0].segments[0].coeffs[0]
        far = _static(base + np.array([500.0, 500.0, 0.0]), 0.0, 1.0)
        fleet = Fleet(fleet.trajectories + [Trajectory([far])])
        meta["bystander"] = True
    return fleet, d_min, meta


# ------------------------------------------------------------------ comparison

def exact_pairs(fleet):
    res, _ = pair_minima(fleet, BIG)
    return res


def compare(fleet, d_min=None, d_true=None, rng=None):
    """Run verify and verify2 on one fleet and return (stats, disagreements).

    ``stats`` has the exact minimum, the bounds and the largest gaps. ``disagreements`` is a
    list of dicts (without seed/family; the caller adds them) for every violated criterion.
    """
    exact = exact_pairs(fleet)
    dis: list = []
    stats: dict = {"n_pairs": len(exact)}
    if not exact:
        return stats, dis
    e_all = min(v[0] for v in exact.values())
    r = verify2(fleet, 1.0, cap=BIG, per_pair=True)
    stats["e"] = e_all
    stats["l"] = r.lower_bound
    stats["u"] = r.upper_bound
    stats["undecided_leaves"] = r.n_leaves
    stats["gap_bound"] = r.gap_bound
    stats["cells"] = r.n_cells
    max_below, max_above = 0.0, 0.0       # e - l (>=0 good), l - e (should be <= 1e-9)
    for ij, (e, t_e) in exact.items():
        l_, u_ = r.pair_lower.get(ij, math.nan), r.pair_upper.get(ij, math.nan)
        if not (l_ == l_):
            dis.append(dict(kind="pair_missing_in_verify2", pair=list(ij), e=e, t_e=t_e))
            continue
        max_below = max(max_below, e - l_)
        max_above = max(max_above, l_ - e)
        if l_ > e + AGREE:
            dis.append(dict(kind="C1_lower_bound_above_exact", pair=list(ij), e=e, l=l_, u=u_,
                            t_e=t_e, diff=l_ - e))
        if r.n_leaves == 0 and e - l_ > r.gap_bound + AGREE:
            dis.append(dict(kind="C2_gap_too_wide", pair=list(ij), e=e, l=l_, u=u_, t_e=t_e,
                            diff=e - l_, gap_bound=r.gap_bound + AGREE))
        if u_ < e - AGREE:
            dis.append(dict(kind="C3_witness_below_exact", pair=list(ij), e=e, l=l_, u=u_,
                            t_e=t_e, diff=e - u_))
    stats["max_e_minus_l"] = max_below
    stats["max_l_minus_e"] = max_above
    # Verdicts: at the construction threshold (if any) and at one threshold placed relative to
    # the exact minimum (offsets of +/-1e-4, +/-1e-6, 0), chosen by the caller's rng.
    stats["verdicts"] = []
    if d_min is not None:
        dis += _verdict_checks(fleet, e_all, d_min, "construction", stats["verdicts"])
    k = 0 if rng is None else int(rng.integers(0, len(DELTAS)))
    dis += _verdict_checks(fleet, e_all, e_all + DELTAS[k], f"e{DELTAS[k]:+.0e}",
                           stats["verdicts"])
    stats["verdict_delta"] = DELTAS[k]
    if d_true is not None:
        stats["d_true"] = d_true
        stats["e_minus_true"] = e_all - d_true
        if abs(e_all - d_true) > 1e-8:
            ij0 = min(exact, key=lambda k: exact[k][0])
            dis.append(dict(kind="exact_vs_construction", pair=list(ij0), e=e_all,
                            t_e=exact[ij0][1], d_true=d_true, diff=e_all - d_true,
                            l=r.lower_bound, u=r.upper_bound))
        if r.lower_bound > d_true + AGREE:
            dis.append(dict(kind="lower_bound_above_construction", l=r.lower_bound,
                            d_true=d_true, diff=r.lower_bound - d_true))
    return stats, dis


def _verdict_checks(fleet, e_all, th, name, log=None):
    out = []
    v = verify2(fleet, th, cap=max(2 * th + 1.0, e_all + 1.0))
    if log is not None:
        log.append((v.verdict, e_all - th))
    if v.verdict == "PASS" and e_all < th - VERDICT_BAND:
        out.append(dict(kind="C4_pass_but_exact_fails", threshold=th, which=name, e=e_all,
                        l=v.lower_bound, u=v.upper_bound))
    if v.verdict == "FAIL" and e_all > th + VERDICT_BAND:
        out.append(dict(kind="C4_fail_but_exact_passes", threshold=th, which=name, e=e_all,
                        l=v.lower_bound, u=v.upper_bound))
    if v.lower_bound > e_all + AGREE:
        out.append(dict(kind="C1_lower_bound_above_exact_default_mode", threshold=th, e=e_all,
                        l=v.lower_bound))
    return out


def hold_sample(tr, ts, dtype=float):
    """Positions of one trajectory at times ``ts`` under the hold model (first position
    before, last position after, gaps held). Independent of ``Trajectory.sample`` (which
    extrapolates across gaps) and of both verifiers; used for sampling and diagnosis only."""
    ts = np.asarray(ts, dtype=dtype)
    segs = [q for q in tr.segments if q.t1 > q.t0]
    out = np.empty((len(ts), 3), dtype=dtype)
    first = segs[0]
    cur = first.t0
    out[:] = first.coeffs[0].astype(dtype)
    last = first.coeffs[0].astype(dtype)

    def horner(c, tau):
        v = np.zeros(np.shape(tau) + (3,), dtype=dtype)
        for k in range(c.shape[0] - 1, -1, -1):
            v = v * np.asarray(tau, dtype=dtype)[..., None] + c[k].astype(dtype)
        return v

    for q in segs:
        if q.t1 <= cur:
            continue
        if q.t0 > cur:
            out[(ts > cur) & (ts <= q.t0)] = last
            cur = q.t0
        m = (ts > cur) & (ts <= q.t1) if cur > q.t0 else (ts >= q.t0) & (ts <= q.t1)
        out[m] = horner(q.coeffs, ts[m] - dtype(q.t0))
        last = horner(q.coeffs, np.asarray(dtype(q.t1) - dtype(q.t0)))
        cur = q.t1
    out[ts > cur] = last
    return out


def reference_min(fleet, pair, t_hint=None, n_grid=40001):
    """Independent extended-precision reference for the minimum distance of one drone pair:
    a global grid scan in long double, then zoom searches around the eight lowest grid
    minima. Diagnostic only (not a proof): returns (value, time)."""
    i, j = pair
    ld = np.longdouble
    a, b = fleet.trajectories[i], fleet.trajectories[j]
    t_lo = min(a.t_start, b.t_start)
    t_hi = max(a.t_end, b.t_end)

    def dist(ts):
        d = hold_sample(a, ts, ld) - hold_sample(b, ts, ld)
        return np.sqrt(np.sum(d * d, axis=1)).astype(float)

    ts = np.linspace(t_lo, t_hi, n_grid)
    dv = dist(ts)
    step0 = (t_hi - t_lo) / (n_grid - 1)
    loc = [k for k in range(n_grid)
           if dv[k] <= (dv[k - 1] if k else np.inf) and dv[k] <= (dv[k + 1] if k < n_grid - 1
                                                                  else np.inf)]
    loc = sorted(loc, key=lambda k: dv[k])[:8]
    if t_hint is not None:
        loc.append(int(np.clip(round((t_hint - t_lo) / step0), 0, n_grid - 1)))
    best, best_t = np.inf, t_lo
    for k in loc:
        c, step = float(ts[k]), step0
        for _ in range(14):
            g = np.linspace(c - 2 * step, c + 2 * step, 41)
            g = g[(g >= t_lo) & (g <= t_hi)]
            vals = dist(g)
            q = int(np.argmin(vals))
            if vals[q] < best:
                best, best_t = float(vals[q]), float(g[q])
            c, step = float(g[q]), step / 10.0
    return best, best_t
