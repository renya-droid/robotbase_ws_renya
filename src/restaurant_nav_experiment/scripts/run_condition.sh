#!/usr/bin/env bash
# Run one condition headless and clean up only the processes this script started.
#   run_condition.sh <B0|B1|P> <scenario> <seed> <out_dir> [extra launch args...]
# Uses an isolated ROS domain / Gazebo partition; never sources robot.env.
set -uo pipefail
condition="$1"; scenario="$2"; seed="$3"; out="$4"; shift 4
ws="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../../.." && pwd)"
[[ -f "$ws/install/setup.bash" ]] || ws="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../../../../.." && pwd)"
mkdir -p "$out"; out="$(cd "$out" && pwd)"
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH ROS_LOCALHOST_ONLY
export ROS_DOMAIN_ID="${EXPERIMENT_ROS_DOMAIN_ID:-94}" ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION="${EXPERIMENT_GZ_PARTITION:-restaurant_nav_exp}" GZ_IP=127.0.0.1
export ROBOTBASE_PARAMS_DIR="$ws/params" ROS_LOG_DIR="$out/ros_log"
set +u  # ROS setup scripts read unset variables
source /opt/ros/jazzy/setup.bash
source "$ws/install/setup.bash"
set -u
limit="${EXPERIMENT_WALL_LIMIT:-900}"
echo "condition=$condition scenario=$scenario seed=$seed out=$out domain=$ROS_DOMAIN_ID ws=$ws"
setsid ros2 launch restaurant_nav_experiment experiment.launch.py condition:="$condition" \
    scenario:="$scenario" seed:="$seed" run_trips:=true out_dir:="$out" "$@" > "$out/launch.log" 2>&1 &
sid=$!
echo "$sid" > "$out/launch.sid"
start=$(date +%s); status=timeout
while (( $(date +%s) - start < limit )); do
    if [[ -f "$out/summary.json" ]]; then status=done; break; fi
    if ! kill -0 "$sid" 2>/dev/null; then status=launch_exited; break; fi
    sleep 5
done
echo "status=$status after $(( $(date +%s) - start )) s"
# SIGINT the launch process group we created, then make sure nothing is left.
kill -INT -- "-$sid" 2>/dev/null
for _ in $(seq 1 30); do pgrep -s "$sid" >/dev/null || break; sleep 1; done
if pgrep -s "$sid" >/dev/null; then kill -TERM -- "-$sid" 2>/dev/null; sleep 3; fi
if pgrep -s "$sid" >/dev/null; then kill -KILL -- "-$sid" 2>/dev/null; fi
echo "$status" > "$out/status.txt"
pgrep -s "$sid" >/dev/null && echo "WARNING: processes left in session $sid" || echo "cleanup ok"
