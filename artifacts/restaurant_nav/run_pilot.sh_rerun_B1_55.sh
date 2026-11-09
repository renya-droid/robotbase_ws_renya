#!/usr/bin/env bash
# Rerun B1 seed 55 (the original run was interrupted by a laptop hybrid sleep).
cd /home/renya/robotbase_ws_renya
while pgrep -f "queue_more_seeds.sh" >/dev/null || pgrep -f "queue_p_seated.sh" >/dev/null || pgrep -f "run_condition.sh" >/dev/null; do sleep 5; done
bash src/restaurant_nav_experiment/scripts/run_pilot.sh artifacts/restaurant_nav/pilot_high "B1" "55" high start_sim:=40.0 stop_sim:=330.0
