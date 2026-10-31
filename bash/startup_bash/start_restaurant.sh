#!/usr/bin/env bash
# Standalone restaurant viewer; never source the original workspace or robot.env.
set -eo pipefail
workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
scenario="${1:-medium}"
case "$scenario" in base|low|medium|high) ;; *) echo 'scenario: base|low|medium|high' >&2; exit 2;; esac
# Discard inherited overlays before loading only the ROS underlay and this build.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
unset GZ_SIM_RESOURCE_PATH IGN_GAZEBO_RESOURCE_PATH ROS_DISCOVERY_SERVER
export PATH=/usr/bin:/bin:/usr/sbin:/sbin
export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1
export GZ_PARTITION=koko_restaurant_launcher2
export ROS_LOG_DIR="$workspace/log/restaurant"
mkdir -p "$ROS_LOG_DIR" "$workspace/.runtime"
exec 9>"$workspace/.runtime/restaurant-viewer.lock"
flock -n 9 || { echo 'レストランは既に起動しています。起動したボタンの停止後に切り替えてください。' >&2; exit 1; }
source /opt/ros/jazzy/setup.bash
if [[ ! -f "$workspace/install/restaurant_sim/share/ament_index/resource_index/packages/restaurant_sim" ]]; then
    echo '独立worktreeのrestaurant_simを先にビルドしてください。' >&2
    exit 1
fi
source "$workspace/install/local_setup.bash"
cd "$workspace"
echo "レストラン: $scenario | ROS_DOMAIN_ID=$ROS_DOMAIN_ID | GZ_PARTITION=$GZ_PARTITION"
# Viewer default: record stalls/timeouts and keep people moving. Arguments
# given after the scenario come later on the command line and override this.
exec ros2 launch restaurant_sim restaurant.launch.py scenario:="$scenario" failure_policy:=log "${@:2}"
