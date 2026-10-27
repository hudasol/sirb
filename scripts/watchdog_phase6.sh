#!/usr/bin/env bash
# Relaunch the phase 6 chain if it is not running and has not finished. Finished Monte Carlo units
# are cached in results/.cache, so a relaunch resumes. Run from the repository root:
#   bash scripts/watchdog_phase6.sh            # once, or from a loop or cron entry
LOG=${P6LOG:-$HOME/p6logs}
if pgrep -f run_phase6_chain.sh > /dev/null; then echo "chain running"; exit 0; fi
if [ -f "$LOG/p6done.flag" ]; then echo "chain finished"; exit 0; fi
mkdir -p "$LOG"
SIRB_WORKERS=${SIRB_WORKERS:-4} nohup setsid nice -n 10 bash scripts/run_phase6_chain.sh "$LOG"     >> "$LOG/watchdog.out" 2>&1 < /dev/null &
echo "chain relaunched $(date -Is)" | tee -a "$LOG/watchdog.out"
