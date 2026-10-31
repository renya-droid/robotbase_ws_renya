#!/usr/bin/env bash
# Run in a child shell; never change the caller's aliases or ROS environment.
set -e
workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${workspace}/bash/activate_koko_env.sh"
cd "${workspace}"
launcher_python=/usr/bin/python3
if [[ -x "${workspace}/.venv/bin/python" ]]; then
  launcher_python="${workspace}/.venv/bin/python"
fi
exec "${launcher_python}" "${workspace}/other_programs/sirius_launcher/robot_launcher.py" "$@"
