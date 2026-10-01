#!/usr/bin/env bash
set -euo pipefail

backup_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd "${backup_dir}/../.." && pwd)"

cp -p "${backup_dir}/params/real/nav2.yaml" \
  "${workspace_dir}/params/real/nav2.yaml"
cp -p "${backup_dir}/params/real/roboteq.yaml" \
  "${workspace_dir}/params/real/roboteq.yaml"
cp -p "${backup_dir}/src/roboteq_ros2_jazzy_driver/roboteq_ros2_driver/src/driver_dev.cpp" \
  "${workspace_dir}/src/roboteq_ros2_jazzy_driver/roboteq_ros2_driver/src/driver_dev.cpp"
cp -p "${backup_dir}/src/roboteq_ros2_jazzy_driver/roboteq_ros2_driver/include/roboteq_ros2_driver/roboteq_ros2_driver.hpp" \
  "${workspace_dir}/src/roboteq_ros2_jazzy_driver/roboteq_ros2_driver/include/roboteq_ros2_driver/roboteq_ros2_driver.hpp"

echo "Restored the pre-rotation-smoothing files. Rebuild the driver before use."
