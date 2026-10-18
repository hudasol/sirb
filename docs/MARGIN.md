# Margin synthesis (planning aid)

Generated from `results/margin.json` by `scripts/make_margin_doc.py`. **Simulated, model dependent planning aid, not a guarantee.** Simulated, model dependent planning aid. The required separation holds only for the simulated tracking loop, wind model, GNSS error model and parameter values used; GNSS errors are independent between drones in this model. Not a safety case and not flight evidence.

## Tool

`python -m sirb margin SHOW --gnss PARAMS.json [--w20 V] [--target 0.05] [--trials N] [--out export.json]` scales every formation of the show about its centre, plans each scale with the baseline planner, checks the plan with the exact verifier (the **guaranteed** layer: the nominal minimum separation of the plan as written), and estimates P(separation violation) in the stress simulator (the **simulated** layer). It returns the smallest scale, to a relative tolerance, whose 95% Wilson upper bound is at most the target, with a guard against one lucky row, and writes the plan at that scale (method in `sirb/margin.py`). Applying it to a real show would be the **planned** layer and is not done here.

## Study

Show family: family_show(4.0, n, big=True) scaled by factor; baseline planner; exact verifier. Scenario: {'w20': 6.0, 'gnss_deg_fraction': 0.25} plus the bias sigma of each row; other values are the assumed defaults. 200 trials per evaluated scale, seed 20261016, relative tolerance 0.02, target 0.05. With zero violations the smallest attainable upper bound is 0.0188.

| n | Bias sigma (m) | GNSS file | Status | Required spacing (m) | Required nominal min separation (m) | Required margin over d_min (m) | Scales evaluated |
|---|---|---|---|---|---|---|---|
| 100 | 0.0 |  | ok | 2.97 | 2.104 | 0.104 | 7 |
| 100 | 0.25 |  | ok | 4.06 | 2.873 | 0.873 | 7 |
| 100 | 0.5 |  | ok | 5.08 | 3.591 | 1.591 | 8 |
| 100 | 1.0 |  | ok | 7.42 | 5.248 | 3.248 | 9 |
| 50 | 0.5 |  | ok | 4.81 | 3.403 | 1.403 | 7 |
| 200 | 0.5 |  | ok | 5.31 | 3.757 | 1.757 | 8 |
| 400 | 0.5 |  | ok | 5.70 | 4.033 | 2.033 | 8 |
| 100 | from file | shows/gnss_faa_sps_standalone.json | ok | 7.62 | 5.386 | 3.386 | 9 |

## Scaling against the pre-registered prediction (P11)

* Bias sigma 1.0 m versus 0.5 m at n = 100: margin ratio 2.04 (predicted between 1.6 and 2.4).
* Bias sigma 0 at n = 100: required margin 0.104 m (predicted below 0.3 m).
* n = 400 versus n = 50 at bias sigma 0.5 m: margin ratio 1.45 (predicted below 1.25).

## Limits

The answer is conditional on every assumption of the stress model (`docs/METHODS.md` sections 5 and 9) and on the show family; scaling a formation also changes its size, so the geofence and altitude band can stop a scale from verifying. The tolerance makes the answer conservative by at most that fraction of the spacing. The Monte Carlo bound is a statistical statement about the simulator, not a probability for real flights.
