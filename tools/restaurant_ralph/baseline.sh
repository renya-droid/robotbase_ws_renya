#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/../.."
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/src/restaurant_sim:${PYTHONPATH:-}"
python3 -m pytest -q src/restaurant_sim/test
for world in src/restaurant_sim/worlds/*.sdf; do gz sdf -k "$world"; done
printf '%s\n' 'Baseline only: research gates G0-G7 are NOT certified.'
