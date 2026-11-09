#!/usr/bin/env bash
# After the extra seeds: condition P with seated people as a small circle (0.4 m, cost 120).
cd /home/renya/robotbase_ws_renya
while pgrep -f "queue_more_seeds.sh" >/dev/null || pgrep -f "run_condition.sh" >/dev/null; do sleep 10; done
bash src/restaurant_nav_experiment/scripts/run_pilot.sh artifacts/restaurant_nav/pilot_high_Pseated "P" "42 11 22" high \
    start_sim:=40.0 stop_sim:=330.0 seated_radius:=0.4 seated_cost:=120
