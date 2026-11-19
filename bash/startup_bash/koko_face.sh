#!/usr/bin/env bash
# Standalone face UI: use the same real-robot DDS settings as the launcher.
set -e
KOKO_FACE_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source "${KOKO_FACE_WS}/bash/activate_koko_env.sh"
source /opt/ros/jazzy/setup.bash
exec /usr/bin/python3 "${KOKO_FACE_WS}/other_programs/koko-face/robot_face_display.py" "$@"
