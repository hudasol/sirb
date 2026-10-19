#!/usr/bin/env bash
# Resumable chain for phase 6: regenerate everything derived from planned fleets after the
# deterministic target change (docs/CHANGES.md), then the sourced GNSS sensitivity (stream B) and
# the margin study (stream C), the computational scaling profile and the differential report. Monte Carlo units are cached in results/.cache, so re-running the
# same command after an interruption continues where it stopped. Usage:
#   SIRB_WORKERS=4 nice -n 10 bash scripts/run_phase6_chain.sh ~/p6logs
# scripts/watchdog_phase6.sh relaunches it if it died (for example after a WSL restart).
set -e
export PYTHONPATH=.
LOG=${1:-/tmp}
mkdir -p "$LOG"
step() { echo "$(date -Is) start $1" >> "$LOG/p6chain.log"; shift; "$@"; echo "$(date -Is) done" >> "$LOG/p6chain.log"; }
step planners python3 scripts/bench_planners.py >> "$LOG/p6_planners.log" 2>&1
step viewer python3 scripts/build_viewer_data.py >> "$LOG/p6_viewer.log" 2>&1
step derive python3 scripts/derive_gnss_faa.py >> "$LOG/p6_derive.log" 2>&1
step sens_faa python3 scripts/sensitivity.py --gnss shows/gnss_faa_sps_standalone.json \
    --bias-range-json results/gnss_faa_derivation.json --tag faa \
    --out results/sensitivity_faa.json >> "$LOG/p6_sens_faa.log" 2>&1
step margin python3 scripts/margin_study.py >> "$LOG/p6_margin.log" 2>&1
step stress bash scripts/run_stress_all.sh "$LOG"
step sens python3 scripts/sensitivity.py >> "$LOG/p6_sens.log" 2>&1
step profile python3 scripts/profile_scale.py >> "$LOG/p6_profile.log" 2>&1
step differential python3 scripts/differential_report.py >> "$LOG/p6_differential.log" 2>&1
echo ALLDONE > "$LOG/p6done.flag"
