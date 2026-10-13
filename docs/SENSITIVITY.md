# Sensitivity of the stress results

Generated from `results/sensitivity.json` by `scripts/make_sensitivity_doc.py`. Everything here is a property of the simulated model with assumed parameters (see `docs/METHODS.md`). It says which assumptions the stress results depend on, not what a real fleet would do.

Scenario base: {'w20': 6.0, 'gnss_deg_fraction': 0.25, 'gnss_bias_sigma': 0.5}. One at a time (OAT) runs use 300 trials per configuration with common random numbers (same trial seeds in every configuration, so differences are paired). Ranges are about half to double the default.

## One at a time, stressed_spacing_4.0

Nominal planned minimum separation 2.828 m, required 2.00 m. Baseline P(violation) 0.597 (95% CI 0.540 to 0.651), median minimum separation 1.936 m.

| Parameter | Low | High | Delta median sep, low (m) | Delta median sep, high (m) | Swing (m) | P(viol) low | P(viol) high |
|---|---|---|---|---|---|---|---|
| gnss_bias_sigma | 0.25 | 1.0 | +0.477 | -1.089 | 1.566 | 0.013 | 1.000 |
| gnss_deg_fraction | 0.125 | 0.5 | +0.217 | -0.233 | 0.451 | 0.297 | 0.863 |
| gnss_bias_tau | 15.0 | 60.0 | -0.057 | +0.044 | 0.101 | 0.650 | 0.520 |
| omega_n | 1.0 | 4.0 | +0.023 | -0.026 | 0.050 | 0.577 | 0.620 |
| w20 | 3.0 | 12.0 | +0.045 | +0.003 | 0.042 | 0.527 | 0.570 |
| zeta | 0.7 | 1.2 | -0.019 | +0.022 | 0.041 | 0.617 | 0.557 |
| wind_tau | 2.5 | 10.0 | -0.014 | +0.011 | 0.025 | 0.590 | 0.593 |
| gnss_white_sigma | 0.0075 | 0.03 | +0.004 | -0.009 | 0.013 | 0.597 | 0.597 |
| drag_gain | 0.15 | 0.6 | +0.009 | -0.001 | 0.010 | 0.590 | 0.603 |
| wind_local_fraction | 0.15 | 0.6 | +0.003 | -0.005 | 0.008 | 0.587 | 0.593 |

Add on hazards switched on from zero:

| Add on | P(viol) | 95% CI | Median min sep (m) |
|---|---|---|---|
| gnss_dropout_5pct_10s | 0.697 | 0.642 to 0.746 | 1.861 |
| battery_low_5pct_descend | 0.943 | 0.911 to 0.964 | 0.760 |
| comms_loss_5pct_continue | 0.597 | 0.540 to 0.651 | 1.936 |
| one_motor_failure_ballistic | 0.827 | 0.780 to 0.865 | 1.548 |

## One at a time, benign_spacing_5.0

Nominal planned minimum separation 3.536 m, required 2.00 m. Baseline P(violation) 0.020 (95% CI 0.009 to 0.043), median minimum separation 2.694 m.

| Parameter | Low | High | Delta median sep, low (m) | Delta median sep, high (m) | Swing (m) | P(viol) low | P(viol) high |
|---|---|---|---|---|---|---|---|
| gnss_bias_sigma | 0.25 | 1.0 | +0.446 | -1.136 | 1.582 | 0.000 | 0.840 |
| gnss_deg_fraction | 0.125 | 0.5 | +0.208 | -0.210 | 0.418 | 0.010 | 0.097 |
| gnss_bias_tau | 15.0 | 60.0 | -0.038 | +0.057 | 0.095 | 0.037 | 0.030 |
| omega_n | 1.0 | 4.0 | +0.017 | -0.040 | 0.057 | 0.020 | 0.027 |
| zeta | 0.7 | 1.2 | -0.022 | +0.024 | 0.046 | 0.020 | 0.017 |
| w20 | 3.0 | 12.0 | +0.034 | +0.000 | 0.034 | 0.027 | 0.037 |
| gnss_white_sigma | 0.0075 | 0.03 | +0.003 | -0.008 | 0.012 | 0.020 | 0.020 |
| drag_gain | 0.15 | 0.6 | -0.000 | -0.011 | 0.011 | 0.020 | 0.020 |
| wind_local_fraction | 0.15 | 0.6 | +0.001 | -0.005 | 0.006 | 0.020 | 0.020 |
| wind_tau | 2.5 | 10.0 | +0.004 | +0.001 | 0.004 | 0.020 | 0.020 |

Add on hazards switched on from zero:

| Add on | P(viol) | 95% CI | Median min sep (m) |
|---|---|---|---|
| gnss_dropout_5pct_10s | 0.050 | 0.031 to 0.081 | 2.646 |
| battery_low_5pct_descend | 0.847 | 0.802 to 0.883 | 0.776 |
| comms_loss_5pct_continue | 0.020 | 0.009 to 0.043 | 2.694 |
| one_motor_failure_ballistic | 0.537 | 0.480 to 0.592 | 1.800 |

## Global (Latin hypercube)

120 samples, 40 trials each, log uniform over the ranges. Spearman rank correlation with the median minimum separation (negative means that raising the parameter reduces separation). p values are not corrected for multiple comparisons.

| Parameter | rho (median sep) | p | rho (P viol) | p |
|---|---|---|---|---|
| gnss_bias_sigma | -0.95 | 1.92e-60 | +0.94 | 5.23e-58 |
| zeta | +0.21 | 0.0194 | -0.22 | 0.0147 |
| gnss_deg_fraction | -0.19 | 0.0388 | +0.15 | 0.105 |
| w20 | -0.19 | 0.0402 | +0.17 | 0.0571 |
| wind_tau | +0.09 | 0.31 | -0.11 | 0.233 |
| gnss_bias_tau | -0.07 | 0.42 | +0.08 | 0.411 |
| gnss_white_sigma | -0.06 | 0.528 | +0.06 | 0.487 |
| drag_gain | +0.06 | 0.53 | -0.07 | 0.456 |
| wind_local_fraction | -0.06 | 0.543 | +0.07 | 0.464 |
| omega_n | -0.03 | 0.786 | -0.01 | 0.898 |

## What survives and what does not

Top three by swing in median minimum separation, stressed baseline: gnss_bias_sigma, gnss_deg_fraction, gnss_bias_tau.
Top three, benign baseline: gnss_bias_sigma, gnss_deg_fraction, gnss_bias_tau.
Largest change in P(violation) across a parameter's range: gnss_bias_sigma (0.013 to 1.000).

Parameters whose range moves P(violation) across 0.05 (so any statement that the configuration is safe or unsafe depends on them): gnss_bias_sigma.

Conclusions that do not survive the tested ranges are listed above and must not be quoted without the parameter range. Any single wind speed threshold is a function of the tracking loop, GNSS and spacing assumptions, and has no meaning on its own.
