#!/usr/bin/env bash
# Resumable chain for phase 6: regenerate everything derived from planned fleets after the
# deterministic target change (docs/CHANGES.md), then the sourced GNSS sensitivity (stream B) and
# the margin study (stream C). Monte Carlo units are cached in results/.cache, so re-running the
# same command after an interruption continues where it stopped. Usage:
#   SIRB_WORKERS=8 nice -n 5 bash scripts/run_phase6_chain.sh /tmp/p6logs
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
echo ALLDONE > "$LOG/p6done.flag"
