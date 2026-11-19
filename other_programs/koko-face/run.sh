#!/usr/bin/env bash
set -e
KOKO_FACE_ROOT="$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")"
KOKO_FACE_ENV_FILE="${KOKO_FACE_ENV_FILE:-${HOME}/.config/koko-face/env}"
if [ -f "$KOKO_FACE_ENV_FILE" ]; then
  source "$KOKO_FACE_ENV_FILE"
fi
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
if [ ! -f "$ROS_SETUP" ]; then
  echo "ROS setupがありません: $ROS_SETUP" >&2
  exit 1
fi
source "$ROS_SETUP"
if [ -n "${KOKO_FACE_WORKSPACE_SETUP:-}" ]; then
  if [ ! -f "$KOKO_FACE_WORKSPACE_SETUP" ]; then
    echo "ワークスペースsetupがありません: $KOKO_FACE_WORKSPACE_SETUP" >&2
    exit 1
  fi
  source "$KOKO_FACE_WORKSPACE_SETUP"
fi
exec /usr/bin/python3 "$KOKO_FACE_ROOT/robot_face_display.py" "$@"
