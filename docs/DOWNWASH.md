# Optional downwash rule

Author: Huda Mueen. Status: optional, default off, ASSUMED parameter. Simulation and planning only; not flight certified, no regulatory compliance claim.

## What it is

A multirotor flying directly above another loses thrust in the air that the upper vehicle pushes down. A separation rule that demands more vertical than horizontal clearance models this with an ellipsoid: two drones conflict when `(dx^2 + dy^2) / d_h^2 + dz^2 / d_v^2 < 1`. The exact check already existed as `sirb.clearance` (used by `verify-external --d-min-vertical`); the new module `sirb/downwash.py` adds a rule object with provenance, a pair report, and the command

```
python -m sirb downwash export.json [--rule shows/downwash_assumed.json] [--d-v 4.56] [--report out.json]
```

The default verification of a show does not use it. The planners and the exact verifier are unchanged.

## Where the number comes from

| Source | Used for |
|---|---|
| Kiran, Ayanian and Breuer, arXiv 2507.09463 (2025), Crazyflie measurements | Thrust of the lower vehicle falls to 65 percent of hover at a vertical offset of 4 arm lengths and is within 95 to 98 percent of hover beyond 19 arm lengths. Arm length 0.0325 m. |
| Preiss, Hönig, Ayanian and Sukhatme, arXiv 1704.04852 | Ellipsoid form, horizontal radius 0.12 m and vertical radius 0.3 m (ratio 2.5). |
| Jain et al., Berkeley HiPeRLab | A downwash model valid beyond seven vehicle sizes of vertical separation; used only as a second scale reference. |

The reference multirotor of the demo shows has a radius of 0.24 m. Scaling 19 arm lengths by the ratio of 0.24 m to 0.0325 m gives a vertical radius of 4.56 m. That scaling from a 27 g vehicle to a 0.83 kg vehicle is an assumption, so the value is flagged `assumed: true` in `shows/downwash_assumed.json`, and the study reports three vertical radii (3.0, 4.56 and 5.0 m).

## What it shows for the demo shows

`results/downwash_study.json`, `scripts/downwash_study.py`, predictions P41 and P42 in `docs/PREDICTIONS.md`. Number of drone pairs that would violate the rule (horizontal radius 2.0 m):

| Show | d_v = 2.0 m (existing check) | 3.0 m | 4.56 m | 5.0 m |
|---|---|---|---|---|
| gallery_120 | 0 | 48 | 183 | 217 |
| oryx_500 | 0 | 3 | 317 | 322 |
| torus_300 | 0 | 0 | 5 | 9 |
| sirb_text_100 | 0 | 11 | 66 | 75 |

Reading: the existing verdicts say nothing about vertical stacking. If a vehicle needs a vertical radius near the scaled value, none of the four plans would pass without re planning with that rule. The planners do not take the rule into account; this package only counts the conflicts.
