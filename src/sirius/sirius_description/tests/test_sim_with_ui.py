import importlib.util
import os
from pathlib import Path


def import_launch_module():
    launch_file = Path(__file__).resolve().parents[1] / 'launch' / 'sim_with_ui.launch.py'
    spec = importlib.util.spec_from_file_location('sim_with_ui', str(launch_file))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_build_gz_command_headless_true():
    module = import_launch_module()
    cmd = module.build_gz_command('world.sdf', {'headless': True})
    assert 'gz' in cmd[0]
    assert '-s' in cmd


def test_build_gz_command_headless_false():
    module = import_launch_module()
    cmd = module.build_gz_command('world.sdf', {'headless': False})
    assert 'gz' in cmd[0]
    assert '-s' not in cmd


def test_build_gz_command_run_on_start_true():
    module = import_launch_module()
    cmd = module.build_gz_command('world.sdf', {'run_on_start': True})
    assert '-r' in cmd


def test_build_gz_command_run_on_start_false():
    module = import_launch_module()
    cmd = module.build_gz_command('world.sdf', {'run_on_start': False})
    assert '-r' not in cmd
