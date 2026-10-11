# Show file format (schema_version 1)

Frame: x east, y north, z up, metres, local origin. All shapes are original or procedural.

```
{
  "schema_version": 1, "name": str, "n_drones": int >= 2, "seed": int,
  "vehicle": {"mass_kg", "radius_m", "sources": {...}},
  "limits": {
    "vmax_h", "vmax_up", "vmax_down", "amax", "jmax",      # m/s, m/s^2, m/s^3
    "z_min", "z_max",                                      # altitude band, m
    "geofence": {"x": [min, max], "y": [min, max]},        # box
    "min_separation_m",                                    # declared centre-to-centre threshold
    "crowd": {"y": float, "min_distance_m": float},        # crowd region is y <= crowd.y
    "sources": {...}                                       # "datasheet" | "assumption" | citation
  },
  "stages": [ {"name": str, "shape": {...}, "hold_s": float >= 0}, ... ],  # >= 2 stages
  "disclaimer": "simulation-validated only; not flight-certified"
}
```

Shape types: `launch_grid` (`spacing_m`, `center`), `sphere`, `supershape` (`m1`, `m2`, `amplitude`), `ring` (`plane` xz|xy), `helix` (`turns`, `strands`), `torus` (`minor_ratio`), `galaxy` (`arms`), `text` (`text`, letters A B D E H I L N O R S T U), `wall` (`aspect`), `skyline` (`buildings`, `aspect`). Common keys: `spacing_m` (scales so the minimum nearest-neighbour distance equals it) or `size`, `center`, `rotate_z_deg`.

Validation (`sirb.spec.validate_show`) rejects: missing keys, non-positive limits, geofence inside the crowd buffer, unknown shapes, target spacing below 1.5 x minimum separation, targets outside the altitude band or geofence.

Numbers marked `assumption` in `sources` are not from datasheets; see `docs/RESEARCH.md`.
