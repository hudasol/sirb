# Sensitivity of the stress results

Generated from `results/sensitivity.json` by `scripts/make_sensitivity_doc.py`. Everything here is a property of the simulated model with assumed parameters (see `docs/METHODS.md`). It says which assumptions the stress results depend on, not what a real fleet would do.

Scenario base: {'w20': 6.0, 'gnss_deg_fraction': 0.25, 'gnss_bias_sigma': 0.5}. One at a time (OAT) runs use 300 trials per configuration with common random numbers (same trial seeds in every configuration, so differences are paired). Ranges are about half to double the default.

## One at a time, stressed_spacing_4.0

Nominal planned minimum separation 2.828 m, required 2.00 m. Baseline P(violation) 0.583 (95% CI 0.527 to 0.638), median minimum separation 1.937 m.

| Parameter | Low | High | Delta median sep, low (m) | Delta median sep, high (m) | Swing (m) | P(viol) low | P(viol) high |
|---|---|---|---|---|---|---|---|
| gnss_bias_sigma | 0.25 | 1.0 | +0.490 | -1.082 | 1.572 | 0.007 | 1.000 |
| gnss_deg_fraction | 0.125 | 0.5 | +0.209 | -0.223 | 0.432 | 0.297 | 0.857 |
| gnss_bias_tau | 15.0 | 60.0 | -0.035 | +0.059 | 0.094 | 0.653 | 0.510 |
| omega_n | 1.0 | 4.0 | +0.046 | -0.035 | 0.080 | 0.533 | 0.623 |
| zeta | 0.7 | 1.2 | -0.019 | +0.025 | 0.044 | 0.597 | 0.570 |
| gnss_white_sigma | 0.0075 | 0.03 | +0.005 | -0.006 | 0.011 | 0.580 | 0.587 |
| drag_gain | 0.15 | 0.6 | -0.002 | -0.005 | 0.003 | 0.583 | 0.590 |
| wind_local_fraction | 0.15 | 0.6 | -0.000 | -0.003 | 0.003 | 0.590 | 0.580 |
| wind_tau | 2.5 | 10.0 | +0.002 | -0.000 | 0.002 | 0.597 | 0.580 |
| w20 | 3.0 | 12.0 | -0.019 | -0.019 | 0.000 | 0.617 | 0.607 |

Add on hazards switched on from zero:

| Add on | P(viol) | 95% CI | Median min sep (m) |
|---|---|---|---|
| gnss_dropout_5pct_10s | 0.660 | 0.605 to 0.711 | 1.873 |
| battery_low_5pct_descend | 0.937 | 0.903 to 0.959 | 0.787 |
| comms_loss_5pct_continue | 0.583 | 0.527 to 0.638 | 1.937 |
| one_motor_failure_ballistic | 0.820 | 0.773 to 0.859 | 1.515 |

## One at a time, benign_spacing_5.0

Nominal planned minimum separation 3.536 m, required 2.00 m. Baseline P(violation) 0.020 (95% CI 0.009 to 0.043), median minimum separation 2.707 m.

| Parameter | Low | High | Delta median sep, low (m) | Delta median sep, high (m) | Swing (m) | P(viol) low | P(viol) high |
|---|---|---|---|---|---|---|---|
| gnss_bias_sigma | 0.25 | 1.0 | +0.439 | -1.153 | 1.592 | 0.000 | 0.867 |
| gnss_deg_fraction | 0.125 | 0.5 | +0.191 | -0.221 | 0.412 | 0.013 | 0.080 |
| gnss_bias_tau | 15.0 | 60.0 | -0.083 | +0.082 | 0.165 | 0.017 | 0.013 |
| omega_n | 1.0 | 4.0 | +0.020 | -0.038 | 0.058 | 0.017 | 0.023 |
| w20 | 3.0 | 12.0 | +0.010 | -0.035 | 0.045 | 0.057 | 0.023 |
| zeta | 0.7 | 1.2 | -0.013 | +0.022 | 0.035 | 0.020 | 0.017 |
| wind_local_fraction | 0.15 | 0.6 | -0.004 | +0.005 | 0.008 | 0.020 | 0.017 |
| drag_gain | 0.15 | 0.6 | -0.003 | -0.008 | 0.005 | 0.020 | 0.020 |
| wind_tau | 2.5 | 10.0 | +0.001 | -0.001 | 0.002 | 0.020 | 0.020 |
| gnss_white_sigma | 0.0075 | 0.03 | -0.000 | +0.001 | 0.002 | 0.020 | 0.020 |

Add on hazards switched on from zero:

| Add on | P(viol) | 95% CI | Median min sep (m) |
|---|---|---|---|
| gnss_dropout_5pct_10s | 0.037 | 0.021 to 0.064 | 2.647 |
| battery_low_5pct_descend | 0.847 | 0.802 to 0.883 | 0.775 |
| comms_loss_5pct_continue | 0.020 | 0.009 to 0.043 | 2.707 |
| one_motor_failure_ballistic | 0.557 | 0.500 to 0.612 | 1.719 |

## Global (Latin hypercube)

120 samples, 40 trials each, log uniform over the ranges. Spearman rank correlation with the median minimum separation (negative means that raising the parameter reduces separation). p values are not corrected for multiple comparisons.

| Parameter | rho (median sep) | p | rho (P viol) | p |
|---|---|---|---|---|
| gnss_bias_sigma | -0.95 | 1.16e-59 | +0.95 | 4.07e-59 |
| zeta | +0.20 | 0.0314 | -0.21 | 0.0231 |
| gnss_deg_fraction | -0.19 | 0.0338 | +0.16 | 0.0879 |
| w20 | -0.17 | 0.0681 | +0.15 | 0.112 |
| wind_tau | +0.10 | 0.269 | -0.12 | 0.193 |
| gnss_white_sigma | -0.09 | 0.343 | +0.08 | 0.408 |
| gnss_bias_tau | -0.07 | 0.443 | +0.06 | 0.546 |
| drag_gain | +0.07 | 0.447 | -0.06 | 0.549 |
| wind_local_fraction | -0.05 | 0.623 | +0.05 | 0.552 |
| omega_n | -0.02 | 0.826 | +0.03 | 0.773 |

## What survives and what does not

Top three by swing in median minimum separation, stressed baseline: gnss_bias_sigma, gnss_deg_fraction, gnss_bias_tau.
Top three, benign baseline: gnss_bias_sigma, gnss_deg_fraction, gnss_bias_tau.
Largest change in P(violation) across a parameter's range: gnss_bias_sigma (0.007 to 1.000).

Parameters whose range moves P(violation) across 0.05 (so any statement that the configuration is safe or unsafe depends on them): gnss_bias_sigma.

## With the sourced GNSS bias (phase 6)

Rerun with the bias sigma derived from FAA accuracy statistics (1.1071 m; range 0.670 to 3.240 m over the reference sites), `results/sensitivity_faa.json`, `docs/GNSS_SOURCED.md`. That value is an absolute error of one receiver used as if it were the error between drones, so it is still an assumption.

| Baseline | Rank | Assumed scenario | Sourced scenario |
|---|---|---|---|
| stressed_spacing_4.0 | 1 | gnss_bias_sigma | gnss_bias_sigma |
| stressed_spacing_4.0 | 2 | gnss_deg_fraction | gnss_deg_fraction |
| stressed_spacing_4.0 | 3 | gnss_bias_tau | omega_n |
| benign_spacing_5.0 | 1 | gnss_bias_sigma | gnss_bias_sigma |
| benign_spacing_5.0 | 2 | gnss_deg_fraction | gnss_deg_fraction |
| benign_spacing_5.0 | 3 | gnss_bias_tau | gnss_bias_tau |

Which assumptions matter most: the top three change for stressed_spacing_4.0 (top three gnss_bias_sigma, gnss_deg_fraction, gnss_bias_tau become gnss_bias_sigma, gnss_deg_fraction, omega_n). Baseline P(violation) with the sourced value: stressed_spacing_4.0 1.000, benign_spacing_5.0 0.947.

Conclusions that do not survive the tested ranges are listed above and must not be quoted without the parameter range. Any single wind speed threshold is a function of the tracking loop, GNSS and spacing assumptions, and has no meaning on its own.
