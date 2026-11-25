"""Disk-only elevator map profiles used by the coordinate editor."""

import math
import os
import re
import shutil
import tempfile
from pathlib import Path

import psutil
import yaml


DEFAULT_ROOT = Path.home() / 'kuwa_ws2'
FILES = ('site.yaml', 'entry.local.yaml', 'arrival.local.yaml', 'imu.local.yaml')
DOOR = ('door_left_x', 'door_left_y', 'door_right_x', 'door_right_y')
LOCAL = ('elevator_local_x_min', 'elevator_local_x_max',
         'elevator_local_y_min', 'elevator_local_y_max')
FRONT = ('door_front_local_x_min', 'door_front_local_x_max',
         'door_front_local_y_min', 'door_front_local_y_max')
NUMBER_FIELDS = (*DOOR, 'elevator_inside_sign', *LOCAL, *FRONT)
ENTRY_EXTRA = ('manual_exit_local_x', 'manual_exit_local_y')


def profile_dir(root, name):
    if name != 'current' and not re.fullmatch(r'[A-Za-z0-9_-]+', name):
        raise ValueError('環境名は英数字・_・-で指定してください')
    directory = root / 'config' if name == 'current' else root / 'config/profiles' / name
    if not directory.is_dir():
        raise ValueError(f'環境設定がありません: {directory}')
    return directory


def profiles(root=DEFAULT_ROOT):
    folder = root / 'config/profiles'
    return ['current', *sorted(p.name for p in folder.iterdir() if p.is_dir())] if folder.is_dir() else ['current']


def read_profile(root, name):
    folder = profile_dir(root, name)
    return {filename: yaml.safe_load((folder / filename).read_text()) for filename in FILES}


def effective_arrival_path(root, name, entry):
    reference = (entry.get('exit_map_geometry_file') if entry.get('switch_map_after_exit')
                 else entry.get('arrival_geometry_file'))
    if not reference:
        return profile_dir(root, name) / 'arrival.local.yaml'
    path = Path(reference).expanduser()
    return path if path.is_absolute() else profile_dir(root, name) / path


def create_profile(root, name, source='current'):
    if name == 'current' or not re.fullmatch(r'[A-Za-z0-9_-]+', name):
        raise ValueError('新しい環境名を英数字・_・-で指定してください')
    source_dir = profile_dir(root, source)
    contents = {filename: (source_dir / filename).read_bytes() for filename in FILES}
    site = yaml.safe_load(contents['site.yaml'])
    for key in ('ros_domain_id', 'robot_workspace', 'mux_config'):
        if site.get(key) is None:
            raise ValueError(f'複製元の {key} が未設定です。設定済みの環境から複製してください')
    target = root / 'config/profiles' / name
    target.mkdir(parents=True, exist_ok=False)
    try:
        for filename, data in contents.items():
            (target / filename).write_bytes(data)
        config = read_profile(root, name)
        entry = config['entry.local.yaml']['elevator_entry_map_diff']['ros__parameters']
        source_geometry = effective_arrival_path(root, source, entry)
        if source_geometry.is_file():
            config['arrival.local.yaml'] = yaml.safe_load(source_geometry.read_text())
        entry['arrival_geometry_file'] = 'arrival.local.yaml'
        if entry.get('switch_map_after_exit'):
            entry['exit_map_geometry_file'] = 'arrival.local.yaml'
        entry['lift_auto_exit_after_stop_open'] = False
        config['site.yaml']['measurements_reviewed'] = False
        if config['site.yaml'].get('stationary_mock_elevator'):
            config['site.yaml']['mock_geometry_reviewed'] = False
        config['site.yaml']['require_human_destination'] = True
        for key in ('departure_waypoint_file', 'departure_wait_waypoint_number',
                    'arrival_waypoint_file'):
            if key in config['site.yaml']:
                config['site.yaml'][key] = None
        _write_files(target, config)
    except Exception:
        shutil.rmtree(target)
        raise
    return target


def _map_file(value):
    path = Path(value).expanduser()
    if not path.is_absolute() or not path.is_file():
        raise ValueError(f'地図YAMLがありません: {value}')
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict) or not data.get('image'):
        raise ValueError(f'地図YAMLに image がありません: {path}')
    image = Path(data['image'])
    if not image.is_absolute():
        image = path.parent / image
    if not image.is_file():
        raise ValueError(f'地図画像がありません: {image}')
    return str(path.resolve())


def _number(value, label):
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise ValueError(f'{label}: 数値を入力してください') from None
    if not math.isfinite(number):
        raise ValueError(f'{label}: 有限の数値を入力してください')
    return number


def _polygon(value, label):
    parts = [part for part in re.split(r'[\s,]+', value.strip()) if part]
    result = [_number(part, label) for part in parts]
    if len(result) < 3:
        raise ValueError(f'{label}: 3点以上を入力してください')
    return result


def _geometry(fields, label):
    result = {key: _number(fields.get(key), f'{label} {key}') for key in NUMBER_FIELDS}
    result['elevator_polygon_x'] = _polygon(fields.get('elevator_polygon_x', ''), f'{label} polygon X')
    result['elevator_polygon_y'] = _polygon(fields.get('elevator_polygon_y', ''), f'{label} polygon Y')
    if len(result['elevator_polygon_x']) != len(result['elevator_polygon_y']):
        raise ValueError(f'{label}: かご頂点の X/Y の数が違います')
    if result['elevator_inside_sign'] not in (-1.0, 1.0):
        raise ValueError(f'{label}: 内側方向は +1 または -1 です')
    if math.hypot(result['door_left_x'] - result['door_right_x'],
                  result['door_left_y'] - result['door_right_y']) < 0.1:
        raise ValueError(f'{label}: ドア両端が近すぎます')
    for prefix in ('elevator_local_x', 'elevator_local_y', 'door_front_local_x', 'door_front_local_y'):
        if result[prefix + '_min'] >= result[prefix + '_max']:
            raise ValueError(f'{label}: {prefix} の最小値は最大値より小さくしてください')
    xs, ys = result['elevator_polygon_x'], result['elevator_polygon_y']
    area = sum(xs[i] * ys[(i + 1) % len(xs)] - xs[(i + 1) % len(xs)] * ys[i]
               for i in range(len(xs)))
    if abs(area) < 0.01:
        raise ValueError(f'{label}: かご多角形の面積が小さすぎます')
    result['use_door_based_elevator'] = True
    result['use_elevator_polygon'] = True
    return result


def _write_files(folder, config):
    original = {name: (folder / name).read_bytes() for name in config}
    staged = {}
    try:
        for name, data in config.items():
            with tempfile.NamedTemporaryFile('w', dir=folder, prefix='.' + name + '.',
                                             suffix='.tmp', delete=False) as stream:
                yaml.safe_dump(data, stream, allow_unicode=True, sort_keys=False)
                staged[name] = Path(stream.name)
        replaced = []
        try:
            for name, path in staged.items():
                os.replace(path, folder / name)
                replaced.append(name)
        except OSError:
            for name in replaced:
                (folder / name).write_bytes(original[name])
            raise
    finally:
        for path in staged.values():
            path.unlink(missing_ok=True)


def control_active():
    """Avoid changing preview geometry while an elevator controller is running."""
    for process in psutil.process_iter(['pid', 'cmdline']):
        try:
            if process.info['pid'] == os.getpid():
                continue
            command = process.info['cmdline'] or []
            executable = Path(command[0]).name if command else ''
            script = Path(command[1]).name if len(command) > 1 else ''
            ros_run = len(command) > 3 and Path(command[0]).name == 'ros2' and command[1:3] == ['run', 'elevator_space_navigation']
            if (executable in ('elevator_entry_map_diff', 'elevator_entry_map_diff.py') or
                    script in ('elevator_entry_map_diff', 'elevator_entry_map_diff.py') or
                    (ros_run and command[3] == 'elevator_entry_map_diff')):
                return True
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    return False


def save_profile(root, name, departure_map, arrival_map, departure_fields,
                 arrival_fields, reviewed=False):
    if name == 'current':
        raise ValueError('current は保護されています。新しい環境設定を作成してください')
    if control_active():
        raise ValueError('エレベーター制御ノードが起動中です。停止後に保存してください')
    folder = profile_dir(root, name)
    config = read_profile(root, name)
    departure = _geometry(departure_fields, '出発側')
    for key in ENTRY_EXTRA:
        departure[key] = _number(departure_fields.get(key), f'出発側 {key}')
    arrival = _geometry(arrival_fields, '到着側')
    departure_map = _map_file(departure_map)
    arrival_map = _map_file(arrival_map)
    entry = config['entry.local.yaml']['elevator_entry_map_diff']['ros__parameters']
    door = config['entry.local.yaml']['elevator_door_node']['ros__parameters']
    for key in (*DOOR, 'elevator_inside_sign'):
        door[key] = departure[key]
    entry.update(departure)
    entry['map_yaml_path'] = departure_map
    entry['arrival_geometry_file'] = 'arrival.local.yaml'
    if entry.get('switch_map_after_exit'):
        entry['exit_map_yaml_path'] = arrival_map
        entry['exit_map_geometry_file'] = 'arrival.local.yaml'
    geometry = config['arrival.local.yaml']['arrival_geometry']
    geometry.update(arrival)
    geometry['elevator_x_min'] = min(arrival['elevator_polygon_x'])
    geometry['elevator_x_max'] = max(arrival['elevator_polygon_x'])
    geometry['elevator_y_min'] = min(arrival['elevator_polygon_y'])
    geometry['elevator_y_max'] = max(arrival['elevator_polygon_y'])
    geometry['map_yaml_path'] = arrival_map
    config['site.yaml']['measurements_reviewed'] = bool(reviewed)
    if config['site.yaml'].get('stationary_mock_elevator'):
        config['site.yaml']['mock_geometry_reviewed'] = bool(reviewed)
    _write_files(folder, config)
    return folder
