"""World selection must preserve explicit overrides and match Gazebo to Nav2."""

import importlib.util
import json
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

from PIL import Image
import pytest
import yaml

from simulation_worlds import WORKSPACE, launch_arguments, last_selection, load_worlds, selection_path


def test_cli_hands_selected_world_and_overridden_pose_to_nav2(tmp_path, monkeypatch):
    import simulation_worlds as module
    state = selection_path(tmp_path, 203)
    calls = []
    monkeypatch.setattr(module, 'selection_path', lambda: state)
    monkeypatch.setattr(module.os, 'execvp', lambda program, args: calls.append((program, args)))
    monkeypatch.setattr(sys, 'argv', ['simulation_worlds.py', '--world-id',
        'indoor_facility', 'spawn_x:=1.0', 'gui:=false'])
    module.main()
    assert json.loads(state.read_text()) == {'world_id': 'indoor_facility', 'spawn': [1., 0., 0.]}
    program, args = calls[0]
    assert program == 'ros2'
    assert any(arg.endswith('/worlds/indoor_facility.sdf') for arg in args)
    assert args.count('spawn_x:=1.0') == 1
    assert 'gui:=false' in args
    # An external world must clear the previous built-in map association.
    monkeypatch.setattr(sys, 'argv', ['simulation_worlds.py', 'world:=/tmp/external.sdf'])
    module.main()
    assert json.loads(state.read_text())['world_id'] is None


def test_world_catalogue_and_facility_are_navigable():
    worlds = {world['id']: world for world in load_worlds()}
    facility = worlds['indoor_facility']
    root = ET.parse(facility['world']).getroot()
    assert len(root.findall('.//model')) > 20
    assert root.findall('.//include') == []  # No network models are required.
    meta = yaml.safe_load(Path(facility['map']).read_text())
    image = Image.open(Path(facility['map']).parent / meta['image'])
    assert image.size == (820, 620)

    def pixel(x, y):
        col = int((x-meta['origin'][0])/meta['resolution'])
        row = image.height-1-int((y-meta['origin'][1])/meta['resolution'])
        return image.getpixel((col, row))

    assert pixel(0, 0) > 250  # Spawn is in the corridor intersection.
    for x in range(-19, 20):
        assert pixel(x, 0) > 250  # Long continuous east-west corridor.
    for y in range(-14, 15):
        assert pixel(0, y) > 250
    for x in (-15.5, -6.5, 6.5, 15.5):
        for y in (-2.1, 2.1):
            assert pixel(x, y) > 250  # Every room has an open doorway.
    assert pixel(20.1, 0) == 0


def test_explicit_spawn_overrides_do_not_duplicate_launch_arguments():
    world = next(world for world in load_worlds() if world['id'] == 'room_360x270')
    args = launch_arguments(world, ['spawn_x:=1.0', 'gui:=false'])
    assert args.count('spawn_x:=1.0') == 1
    assert not any(arg == 'spawn_x:=1.8' for arg in args)
    assert 'spawn_y:=1.35' in args
    assert 'gui:=false' in args


def test_last_selection_is_scoped_by_workspace_and_ros_domain(tmp_path):
    shutil.copytree(WORKSPACE / 'src/robotbase_sim', tmp_path / 'src/robotbase_sim')
    path = selection_path(tmp_path, 203)
    path.parent.mkdir()
    path.write_text(json.dumps({'world_id': 'indoor_facility', 'spawn': [1., 0., 0.]}))
    world = last_selection(tmp_path, 203)
    assert world['map'].startswith(str(tmp_path))
    assert world['spawn'] == [1., 0., 0.]
    assert last_selection(tmp_path, 204) is None
    path.write_text('{"world_id": null}')
    assert last_selection(tmp_path, 203) is None
    with pytest.raises(ValueError):
        selection_path(tmp_path, '../unsafe')


def test_selected_spawn_is_written_to_amcl_without_mutating_source():
    from launch import LaunchContext
    path = WORKSPACE / 'src/robotbase_sim/launch/navigation.launch.py'
    spec = importlib.util.spec_from_file_location('navigation_world_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    original = WORKSPACE / 'params/sim/nav2.yaml'
    before = original.read_bytes()
    context = LaunchContext()
    context.launch_configurations.update(params_file=str(original), initial_pose_x='1.8',
        initial_pose_y='1.35', initial_pose_yaw='0.5')
    action = module.configure_navigation(context)[0]
    args = dict(action.launch_arguments)
    generated = Path(args['params_file'].perform(context))
    try:
        amcl = yaml.safe_load(generated.read_text())['amcl']['ros__parameters']
        assert amcl['initial_pose']['x'] == 1.8
        assert amcl['initial_pose']['y'] == 1.35
        assert amcl['initial_pose']['yaw'] == .5
        assert original.read_bytes() == before
    finally:
        generated.unlink(missing_ok=True)


def test_world_dialog_cancels_preset_or_builds_selected_command(monkeypatch):
    from PySide6.QtWidgets import QApplication, QInputDialog
    from robot_launcher import RobotLauncher
    app = QApplication.instance() or QApplication([])
    window = RobotLauncher()
    button = window.button_map['koko_sim']
    label = next(world['label'] for world in load_worlds() if world['id'] == 'indoor_facility')
    monkeypatch.setattr(QInputDialog, 'getItem', lambda *args: (label, True))
    assert window.prepare_simulation_launch(button)
    assert button.process_manager.command.endswith('--world-id indoor_facility')
    # Re-selecting must not append a second world ID.
    assert window.prepare_simulation_launch(button)
    assert button.process_manager.command.count('--world-id') == 1
    window.preset_queue = ['koko_nav2_sim_map']
    monkeypatch.setattr(QInputDialog, 'getItem', lambda *args: ('', False))
    assert not window.prepare_simulation_launch(button)
    assert window.preset_queue == []
    # Cancelling the actual preset must not start the Gazebo readiness probe.
    monkeypatch.setattr(button.process_manager, 'is_running', lambda: False)
    monkeypatch.setattr(window, 'wait_for_simulation', lambda: pytest.fail('Cancelled launch started a probe'))
    window.launch_preset('test', ['koko_sim', 'koko_nav2_sim_map'])
    assert window.preset_probe is None
    assert window.preset_queue == []
    window.close()
