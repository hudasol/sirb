#!/usr/bin/env bash
# Resumable chain: finished units are cached in results/.cache, so re-running after an interruption
# continues where it stopped. Writes results/stress_*.json.
set -e
LOG=${1:-/tmp}
python3 scripts/run_stress.py sweeps --trials 200 >> $LOG/sweeps.log 2>&1
python3 scripts/run_stress.py spacing --trials 300 >> $LOG/spacing.log 2>&1
python3 scripts/run_stress.py contingency --trials 200 >> $LOG/contingency.log 2>&1
python3 scripts/run_stress.py scaling >> $LOG/scaling.log 2>&1
echo ALLDONE > $LOG/done.flag
