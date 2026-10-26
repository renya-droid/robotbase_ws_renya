"""Exercise the real startup script with a harmless executable in a temporary workspace."""

from pathlib import Path
import shutil
import subprocess

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / 'bash/startup_bash/waypoint_follow.sh'


@pytest.fixture
def workspace(tmp_path):
    script = tmp_path / 'bash/startup_bash/waypoint_follow.sh'
    script.parent.mkdir(parents=True)
    shutil.copyfile(SCRIPT, script)
    routes = tmp_path / 'maps_waypoints/waypoints'
    routes.mkdir(parents=True)
    (routes / 'route.yaml').write_text('waypoints: []\n')
    executable = tmp_path / 'install/robotbase_waypoint/lib/robotbase_waypoint/waypoint_follow'
    executable.parent.mkdir(parents=True)
    executable.write_text('#!/bin/bash\nprintf "ARG:%s\\n" "$@"\n')
    executable.chmod(0o755)
    return script


@pytest.mark.parametrize('mode', ['real', 'sim'])
@pytest.mark.parametrize('choice', ['', '1', '2'])
def test_selection_and_threshold_arguments(workspace, mode, choice):
    answers = ['1', '2', choice]
    answers.append('2')
    answers.append('y')
    result = subprocess.run(['bash', str(workspace), mode, '--navigation-only'],
                            input='\n'.join(answers) + '\n', text=True, capture_output=True)
    assert result.returncode == 0, result.stderr
    output = result.stdout
    navigation = 'through_poses' if choice == '2' else 'to_pose'
    assert f'ARG:navigation_mode:={navigation}' in output
    assert f'ARG:use_sim_time:={str(mode == "sim").lower()}' in output
    assert 'ARG:start_index:=2' in output
    assert 'ARG:loop:=true' in output
    assert 'ARG:navigation_only:=true' in output
    assert ('ARG:default_threshold:=2.0' in output) == (choice != '2')
    assert ('ARG:section_switch_distance:=2.0' in output) == (choice == '2')
    assert ('中間点の切替距離' in output) == (choice != '2')
    assert '走行方式:' in output


@pytest.mark.parametrize('choice', ['0', '3', '-1', 'abc', '1.0'])
def test_invalid_mode_never_starts_follower(workspace, choice):
    result = subprocess.run(['bash', str(workspace)], input=f'1\n1\n{choice}\n',
                            text=True, capture_output=True)
    assert result.returncode == 1
    assert '無効な走行方式' in result.stderr
    assert 'ARG:' not in result.stdout


@pytest.mark.parametrize('threshold', ['0', '-1', 'nan', 'abc'])
def test_invalid_to_pose_threshold_is_rejected(workspace, threshold):
    result = subprocess.run(['bash', str(workspace)], input=f'1\n1\n1\n{threshold}\n',
                            text=True, capture_output=True)
    assert result.returncode == 1
    assert '切替距離には0より大きい数値' in result.stderr
    assert 'ARG:' not in result.stdout


@pytest.mark.parametrize('mode', ['real', 'sim'])
def test_through_default_switch_distance(workspace, mode):
    result = subprocess.run(['bash', str(workspace), mode], input='1\n1\n2\n\nn\n',
                            text=True, capture_output=True)
    assert result.returncode == 0, result.stderr
    assert 'ARG:section_switch_distance:=6.0' in result.stdout
    assert 'ARG:default_threshold:=' not in result.stdout


@pytest.mark.parametrize('threshold', ['0', '-1', 'nan', 'inf', 'abc'])
def test_invalid_through_switch_distance_is_rejected(workspace, threshold):
    result = subprocess.run(['bash', str(workspace)], input=f'1\n1\n2\n{threshold}\n',
                            text=True, capture_output=True)
    assert result.returncode == 1
    assert 'ARG:' not in result.stdout
