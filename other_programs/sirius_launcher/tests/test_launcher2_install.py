"""Verify coexistence and relocation without installing into the user's HOME."""

import json
import os
from pathlib import Path
import shutil
import subprocess

from robot_config import LAUNCHER_ID, LAUNCHER_TITLE, WORKSPACE
from process_manager import ProcessManager


def test_launcher2_has_separate_process_files():
    manager = ProcessManager('koko_sim', 'true')
    assert LAUNCHER_TITLE == 'ココちゃんランチャー2'
    assert LAUNCHER_ID == 'koko_launcher2'
    assert manager.pid_file == '/tmp/koko_launcher2_koko_sim.pid'
    assert manager.log_file == '/tmp/koko_launcher2_koko_sim.log'


def test_install_and_launch_from_relocated_workspace_preserves_original(tmp_path):
    workspace = tmp_path / 'another user' / 'robotbase_ws_renya'
    shutil.copytree(WORKSPACE / 'bash', workspace / 'bash')
    shutil.copy2(WORKSPACE / 'robot.env', workspace / 'robot.env')
    launcher_dir = workspace / 'other_programs' / 'sirius_launcher'
    launcher_dir.mkdir(parents=True)
    # Replace the GUI with a probe so this test never starts ROS or Terminator.
    (launcher_dir / 'robot_launcher.py').write_text(
        'import json, os\n'
        'print(json.dumps({"cwd": os.getcwd(), '
        '"params": os.environ["ROBOTBASE_PARAMS_DIR"], '
        '"domain": os.environ["ROS_DOMAIN_ID"], '
        '"partition": os.environ["GZ_PARTITION"]}))\n'
    )
    home = tmp_path / 'home'
    original_paths = [
        home / 'Desktop' / 'koko_launcher.desktop',
        home / '.local/share/applications/koko_launcher.desktop',
        home / '.local/bin/koko_launcher',
        home / '.bashrc',
        home / '.koko_nav2_current_map.yaml',
    ]
    for path in original_paths:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('original environment\n')
    fake_bin = tmp_path / 'bin'
    fake_bin.mkdir()
    # Ensure the test cannot resolve the real desktop directory via XDG config.
    for name, script in {
        'xdg-user-dir': '#!/bin/sh\nprintf "%s/Desktop\\n" "$HOME"\n',
        'gio': '#!/bin/sh\nexit 0\n',
    }.items():
        path = fake_bin / name
        path.write_text(script)
        path.chmod(0o755)
    env = dict(os.environ, HOME=str(home), PATH=f'{fake_bin}:/usr/bin:/bin')
    for _ in range(2):
        subprocess.run(['bash', str(workspace / 'bash/install_launcher_shortcut.sh')],
                       env=env, check=True, capture_output=True, text=True)
    desktop = home / 'Desktop/koko_launcher2.desktop'
    assert 'Name=ココちゃんランチャー2\n' in desktop.read_text()
    assert f'Exec=/bin/bash "{workspace}/bash/start_launcher2.sh"' in desktop.read_text()
    command = home / '.local/bin/koko_launcher2'
    result = subprocess.run([str(command)], env=env, check=True, capture_output=True, text=True)
    probe = json.loads(result.stdout)
    assert probe == {
        'cwd': str(workspace),
        'params': str(workspace / 'params'),
        'domain': '54',
        'partition': 'koko_renya',
    }
    for path in original_paths:
        assert path.read_text() == 'original environment\n'
