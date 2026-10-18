# Reproducibility of plans across machines

## Problem

Fleet hashes of the demo shows differed between machines. Three machines were compared on the
same commit (`phase5-hardening`, before this change), building `viewer/data` with
`scripts/build_viewer_data.py`:

| Show | CI copy committed on the branch | Linux (WSL, this machine) | Windows (this machine) |
|---|---|---|---|
| `sirb_text_100` | `4c4766d0...` | same as CI | `cb4b5dd1...` |
| `torus_300` | `59b00af3...` | `ab8ba9eb...` | `d7509bf9...` |
| `wall_500` | `a53142d7...` | `a3d785be...` | `0c3e12f9...` |

An earlier CI run on the same branch (run 38412392116) produced `ab8ba9eb...` and `a3d785be...` for
the torus and the wall, the Linux values above, while a later CI run agreed with the committed
copy. So even CI runners disagreed with each other, which points to the CPU, not only the operating
system.

Two separate causes were found by comparing the certificates:

1. **Last bit noise in the targets.** For `torus_300` the closest pair, its time and every report
   field agreed to about 1e-14, only the last bits differed. The targets are computed with
   trigonometry and a rotation matrix product. numpy dispatches `sin`, `cos` and matrix products to
   different kernels depending on the CPU features (for example AVX512 builds of SVML on some Linux
   runners) and on the platform's maths library, so the last bit of a target coordinate depends on
   the machine. Every polynomial coefficient inherits it, so the hash changes.
2. **Tied assignments.** For `sirb_text_100` on Windows the stressed scenario minimum distances
   changed by metres (for example the comms loss scenario 2.1 m versus 0.56 m), so the plan itself
   was different. Text and grid shapes have many assignments of exactly equal cost. The Hungarian
   solver (`scipy.optimize.linear_sum_assignment`) is deterministic, but when the last bits of the
   costs differ it sees a different problem and can return another member of the tie.

An earlier attempt (reverted on `phase5-hardening`, commits `1d5a473`, `86f1aec`) quantised the
cost matrix. It cannot work in general: costs computed from noisy coordinates can fall on either
side of a quantisation edge.

## Fix

Remove the noise at its source, then make the assignment input exact.

1. `sirb.shapes.generate_points` snaps every point to a binary grid of 2^-20 m (about 0.95
   micrometre): `round(x * 2^20) / 2^20`. Multiplying by a power of two and rounding are exact in
   IEEE 754 arithmetic, so the snapped value is identical on every machine as long as the
   unsnapped values are within the same half cell, which fails only if a coordinate lies within a
   few units in the last place of a cell edge.
2. `sirb.assign` computes the Hungarian cost from the snapped coordinates in 64 bit integers
   (exact squared distances in units of 2^-40 m^2), then converts to floating point. Equal
   geometric costs are therefore exactly equal everywhere, and the solver's fixed tie rule (its
   index order) picks the same member of a tie on every machine. Coordinates beyond 512 m raise an
   error instead of overflowing.
3. `sirb.planner.common` no longer calls BLAS (`np.linalg.norm` on a 3 vector) or `pow` (`T**3`)
   when building a leg; it uses products and `math.sqrt`, which are correctly rounded. The cube
   root in `min_leg_time` still uses `pow`; it matters only when the jerk limit sets the leg time.

A lexicographic secondary cost was considered as the tie break and is not needed: the primary
cost is exact, and among exact ties the solver's index order decides. A separate secondary cost was
not added, because once the primary cost is exact no further rule is needed for agreement.

## Evidence

After the fix the three hashes built on Windows and on Linux under WSL are identical, and every
viewer data file is byte identical apart from line endings (Windows writes CRLF text):

| Show | Windows | Linux (WSL) |
|---|---|---|
| `sirb_text_100` | `dc4aba75...` | `dc4aba75...` |
| `torus_300` | `d17f2c1a...` | `d17f2c1a...` |
| `wall_500` | `7283bef4...` | `7283bef4...` |

The CI runner (GitHub Actions run 38857921036 on commit `60565d2`, ubuntu-latest) regenerated the
same three hashes, `dc4aba75...`, `d17f2c1a...` and `7283bef4...`, so three machines agree. The
check against the committed copy in `scripts/check_viewer_data.py` is therefore now a hard check:
a future change that reintroduces machine dependence fails CI.

## What changed in published numbers

Snapping moves targets by at most 2^-21 m (about 0.5 micrometre) per coordinate, and the tied
assignments of grid and text shapes may resolve to a different member of the tie. Every result
derived from planned fleets was regenerated; see `docs/CHANGES.md`.

## Limits

This is agreement by construction for the operations listed, tested on two platforms and the CI
runner, not a proof for every CPU. A coordinate that lands within a few units in the last place of a
grid cell edge would still snap differently; with about 10^4 coordinates per show and cells of 2^-20
m this is very unlikely but not impossible. The stress simulator draws random numbers with numpy's
`default_rng`, whose streams are specified to be platform independent, but its arithmetic on top
uses ordinary floating point, so Monte Carlo minimum distances can still differ in the last bits.
