#!/usr/bin/env bash
# Resumable chain for the phase 5 experiments (cached per unit in results/.cache).
set -e
export PYTHONPATH=.
LOG=${1:-/tmp}
python3 scripts/sensitivity.py >> $LOG/sensitivity.log 2>&1
python3 scripts/ground_robots.py >> $LOG/ground.log 2>&1
echo ALLDONE > $LOG/p5done.flag
