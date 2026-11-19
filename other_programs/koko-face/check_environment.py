#!/usr/bin/env python3
"""Verify the installed assets without publishing any robot command or playing sound."""
import argparse
import hashlib
import os
from pathlib import Path
import shutil
import sys
import wave


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless', action='store_true', help='Use offscreen Qt')
    args = parser.parse_args()
    if args.headless:
        os.environ['QT_QPA_PLATFORM'] = 'offscreen'
    try:
        import rclpy
        from geometry_msgs.msg import Twist
        from std_msgs.msg import Bool, String
        from PyQt5.QtWidgets import QApplication
        from PyQt5.QtGui import QMovie, QPixmap
        from expression_art import NATIVE_EXPRESSIONS
        from robot_face_display import DEFAULT_STATES, DEFAULT_WINKS, ASSET_DIRECTORY, FaceWindow, DEFAULT_FACES
    except ImportError as error:
        print(f'FAIL: ROS setupをsourceして依存関係を確認してください: {error}', file=sys.stderr)
        return 1
    root = Path(__file__).resolve().parent
    for line in (root / 'SHA256SUMS').read_text().splitlines():
        expected, name = line.split('  ', 1)
        path = root / name
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            print(f'FAIL: ファイル欠落または破損: {name}', file=sys.stderr)
            return 1
    if not shutil.which('aplay'):
        print('FAIL: aplayがありません。sudo apt install alsa-utils', file=sys.stderr)
        return 1
    app = QApplication.instance() or QApplication([])
    for path in [*DEFAULT_STATES.values(), *DEFAULT_WINKS.values()]:
        movie = QMovie(str(path))
        movie.jumpToFrame(0)
        if not movie.isValid():
            raise RuntimeError(f'GIFを読み込めません: {path}')
    for name in ('hand_open.png', 'hand_grip_sheet.png', 'gap_tooth_smile.png', 'human_tongue.png'):
        if QPixmap(str(ASSET_DIRECTORY/name)).isNull():
            raise RuntimeError(f'手の画像を読み込めません: {name}')
    clips = list((ASSET_DIRECTORY / 'voice').glob('*.wav'))
    if not clips:
        raise RuntimeError('音声素材がありません')
    for path in clips:
        with wave.open(str(path)) as stream:
            assert stream.getsampwidth() == 2 and stream.getnframes() > 0
    window = FaceWindow(DEFAULT_FACES, speech_probability=0)
    window.pointer_timer.stop()
    for state in DEFAULT_STATES:
        assert window.show_state(state)
        assert window.face.rect().contains(window.face.face_rect())
        assert window.face.hands.active() == (state == 'smile'), state
        if state in NATIVE_EXPRESSIONS:
            assert window.face.motion_sample()['expression'] == state
    window.show_state('normal')
    window.set_speaking(True)
    for level in (0, .5, 1, 0):
        window.set_mouth_level(level)
    window.set_speaking(False)
    assert window.current_state == 'normal'
    assert window.start_split_play()
    window.face.split_play.touch(.5, .5)
    window.face.advance_motion(window.face.split_play.last_touch+33.)
    assert not window.face.split_play.active
    window.close()
    app.processEvents()
    print(f'PASS: {len(DEFAULT_STATES)} states, hand sprites, {len(clips)} WAVs, checksums and mouth control')
    print('実機の音量・出力先、タッチ位置、ROS通信はGUI起動後に確認してください。')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
