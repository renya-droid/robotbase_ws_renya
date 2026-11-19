#!/usr/bin/env bash
set -e
PACKAGE_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
INSTALL_DIR="${KOKO_FACE_INSTALL_DIR:-${HOME}/.local/share/koko-face}"
BIN_DIR="${KOKO_FACE_BIN_DIR:-${HOME}/.local/bin}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
if [ ! -f "$ROS_SETUP" ]; then
  echo "ROS 2 setupがありません: $ROS_SETUP" >&2
  exit 1
fi
if [ "$(realpath -m -- "$INSTALL_DIR")" = "$PACKAGE_DIR" ]; then
  echo "展開フォルダーとは別のインストール先を指定してください" >&2
  exit 1
fi
(cd "$PACKAGE_DIR" && sha256sum --check --quiet SHA256SUMS)
source "$ROS_SETUP"
if ! /usr/bin/python3 -c 'import rclpy; from geometry_msgs.msg import Twist; from std_msgs.msg import Bool, String; from PyQt5.QtWidgets import QApplication' 2>/dev/null || ! command -v aplay >/dev/null; then
  echo "必要な依存関係が不足しています。Ubuntu + ROS 2 Jazzyでは次を実行してください:" >&2
  echo "sudo apt install python3-pyqt5 alsa-utils ros-jazzy-rclpy ros-jazzy-geometry-msgs ros-jazzy-std-msgs" >&2
  exit 1
fi
STAMP="$(date +%Y%m%d_%H%M%S)_$$"
if [ -e "$INSTALL_DIR" ]; then
  cp -a -- "$INSTALL_DIR" "${INSTALL_DIR}.backup.${STAMP}"
  echo "既存版をバックアップ: ${INSTALL_DIR}.backup.${STAMP}"
fi
mkdir -p "$INSTALL_DIR" "$BIN_DIR"
for name in install.sh robot_face_display.py face_motion.py mouth_art.py expression_art.py butterfly.py split_play.py companion_play.py hand_gesture.py touch_voice.py check_environment.py test_face_motion.py test_voice_routing.py test_butterfly.py test_split_play.py test_transparent.py test_companion_play.py run.sh README.md SHA256SUMS MANIFEST.txt env.example; do
  cp -- "$PACKAGE_DIR/$name" "$INSTALL_DIR/$name"
done
cp -R -- "$PACKAGE_DIR/assets" "$INSTALL_DIR/"
chmod +x "$INSTALL_DIR/run.sh"
for name in koko-face koko_face koko_face2; do
  if [ -e "$BIN_DIR/$name" ] || [ -L "$BIN_DIR/$name" ]; then
    cp -a -- "$BIN_DIR/$name" "$BIN_DIR/$name.backup.$STAMP"
  fi
  # Do not follow an existing launcher symlink when replacing it.
  TEMP_LAUNCHER="$(mktemp "$BIN_DIR/.koko-face.XXXXXX")"
  if [ "$name" = koko_face2 ]; then
    printf '#!/usr/bin/env bash\nif [ -z "${ROS_SETUP:-}" ]; then\n  export ROS_SETUP=%q\nfi\nexec %q --transparent "$@"\n' "$ROS_SETUP" "$INSTALL_DIR/run.sh" > "$TEMP_LAUNCHER"
  else
    printf '#!/usr/bin/env bash\nif [ -z "${ROS_SETUP:-}" ]; then\n  export ROS_SETUP=%q\nfi\nexec %q "$@"\n' "$ROS_SETUP" "$INSTALL_DIR/run.sh" > "$TEMP_LAUNCHER"
  fi
  chmod +x "$TEMP_LAUNCHER"
  mv -f -- "$TEMP_LAUNCHER" "$BIN_DIR/$name"
done
echo "インストール完了: $INSTALL_DIR"
echo "黒背景版: $BIN_DIR/koko-face"
echo "背景透明版: $BIN_DIR/koko_face2"
echo "環境チェック: /usr/bin/python3 $INSTALL_DIR/check_environment.py --headless"
echo "既存のkoko_faceエイリアスがある場合も、上記の絶対パスで新版を起動できます。"
