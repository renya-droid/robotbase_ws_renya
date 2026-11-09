#!/usr/bin/env bash
# Wait for the first pilot batch to finish, then add seeds 33 44 55 (skips finished runs).
cd /home/renya/robotbase_ws_renya
while pgrep -f "run_pilot.sh artifacts/restaurant_nav/pilot_high \"B0 B1 P\" \"42 11 22\"" >/dev/null || pgrep -f 'run_pilot.sh artifacts/restaurant_nav/pilot_high B0 B1 P 42 11 22' >/dev/null || pgrep -f "run_condition.sh" >/dev/null; do sleep 10; done
bash src/restaurant_nav_experiment/scripts/run_pilot.sh artifacts/restaurant_nav/pilot_high "B0 B1 P" "33 44 55" high start_sim:=40.0 stop_sim:=330.0
