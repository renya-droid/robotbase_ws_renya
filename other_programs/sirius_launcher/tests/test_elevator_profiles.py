import pytest
import yaml
from types import SimpleNamespace

from elevator_profiles import (ENTRY_EXTRA, FRONT, LOCAL, NUMBER_FIELDS, control_active, create_profile,
                               effective_arrival_path, read_profile, save_profile)


def sample_fields(offset=0.0):
    fields = dict(door_left_x=0.0 + offset, door_left_y=0.0,
                  door_right_x=1.0 + offset, door_right_y=0.0,
                  elevator_inside_sign=1.0,
                  elevator_polygon_x=[offset, offset + 1, offset + 1, offset],
                  elevator_polygon_y=[0.0, 0.0, 2.0, 2.0])
    fields.update({key: value for key, value in zip(LOCAL, (-1.0, 1.0, 0.0, 2.0))})
    fields.update({key: value for key, value in zip(FRONT, (-0.5, 0.5, 0.0, 0.6))})
    fields.update(manual_exit_local_x=0.0, manual_exit_local_y=-1.2)
    return fields


def text_fields(values):
    return {key: ', '.join(map(str, values[key])) if isinstance(values[key], list)
            else str(values[key]) for key in (*NUMBER_FIELDS, *ENTRY_EXTRA,
                                               'elevator_polygon_x', 'elevator_polygon_y')}


@pytest.fixture
def workspace(tmp_path):
    config = tmp_path / 'config'
    config.mkdir()
    entry = sample_fields()
    arrival = sample_fields(10.0)
    docs = {
        'site.yaml': dict(ros_domain_id=54, robot_workspace='/robot', mux_config='/mux.yaml',
                          measurements_reviewed=True, require_human_destination=False,
                          departure_waypoint_file='/old/route.yaml'),
        'entry.local.yaml': {
            'elevator_door_node': {'ros__parameters': {key: entry[key] for key in
                                                       (*('door_left_x', 'door_left_y', 'door_right_x',
                                                          'door_right_y'), 'elevator_inside_sign')}},
            'elevator_entry_map_diff': {'ros__parameters': {
                **entry, 'map_yaml_path': '/old/map.yaml',
                'switch_map_after_exit': True, 'switch_map_before_exit': False,
                'exit_map_yaml_path': '/old/arrival.yaml',
                'exit_map_geometry_file': str(config / 'arrival_special.local.yaml'),
                'arrival_geometry_file': '/old/arrival.local.yaml',
                'lift_auto_exit_after_stop_open': True}}},
        'arrival.local.yaml': {'arrival_geometry': arrival},
        'imu.local.yaml': {'elevator_imu_monitor': {'ros__parameters': {}}},
    }
    for filename, data in docs.items():
        (config / filename).write_text(yaml.safe_dump(data))
    (config / 'arrival_special.local.yaml').write_text(
        yaml.safe_dump({'arrival_geometry': sample_fields(50.0)}))
    for name in ('departure', 'arrival'):
        (tmp_path / f'{name}.yaml').write_text(f'image: {name}.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n')
        (tmp_path / f'{name}.pgm').write_bytes(b'P2\n1 1\n255\n0\n')
    return tmp_path


def test_new_map_profile_preserves_current_and_updates_both_sides(workspace):
    before = read_profile(workspace, 'current')
    create_profile(workspace, 'new_map')
    staged = read_profile(workspace, 'new_map')
    assert staged['arrival.local.yaml']['arrival_geometry']['door_left_x'] == 50.0
    staged_entry = staged['entry.local.yaml']['elevator_entry_map_diff']['ros__parameters']
    assert effective_arrival_path(workspace, 'new_map', staged_entry) == (
        workspace / 'config/profiles/new_map/arrival.local.yaml')
    assert staged['site.yaml']['measurements_reviewed'] is False
    assert staged['site.yaml']['require_human_destination'] is True
    assert staged['site.yaml']['departure_waypoint_file'] is None
    assert staged['entry.local.yaml']['elevator_entry_map_diff']['ros__parameters']['lift_auto_exit_after_stop_open'] is False
    save_profile(workspace, 'new_map', str(workspace / 'departure.yaml'),
                 str(workspace / 'arrival.yaml'), text_fields(sample_fields(2.0)),
                 text_fields(sample_fields(12.0)), reviewed=True)
    result = read_profile(workspace, 'new_map')
    entry = result['entry.local.yaml']['elevator_entry_map_diff']['ros__parameters']
    door = result['entry.local.yaml']['elevator_door_node']['ros__parameters']
    arrival = result['arrival.local.yaml']['arrival_geometry']
    assert read_profile(workspace, 'current') == before
    assert entry['map_yaml_path'] == str(workspace / 'departure.yaml')
    assert entry['exit_map_yaml_path'] == str(workspace / 'arrival.yaml')
    assert entry['exit_map_geometry_file'] == 'arrival.local.yaml'
    assert door['door_left_x'] == entry['door_left_x'] == 2.0
    assert arrival['door_left_x'] == 12.0
    assert arrival['elevator_x_min'] == 12.0
    assert arrival['map_yaml_path'] == str(workspace / 'arrival.yaml')
    assert result['site.yaml']['measurements_reviewed'] is True


def test_invalid_geometry_does_not_change_profile(workspace, monkeypatch):
    create_profile(workspace, 'test_map')
    before = read_profile(workspace, 'test_map')
    invalid = text_fields(sample_fields())
    invalid['elevator_polygon_y'] = '0, 0'
    with pytest.raises(ValueError, match='3点以上'):
        save_profile(workspace, 'test_map', str(workspace / 'departure.yaml'),
                     str(workspace / 'arrival.yaml'), invalid,
                     text_fields(sample_fields(10.0)))
    assert read_profile(workspace, 'test_map') == before
    with pytest.raises(ValueError, match='current'):
        save_profile(workspace, 'current', str(workspace / 'departure.yaml'),
                     str(workspace / 'arrival.yaml'), text_fields(sample_fields()),
                     text_fields(sample_fields(10.0)))
    monkeypatch.setattr('elevator_profiles.control_active', lambda: True)
    with pytest.raises(ValueError, match='起動中'):
        save_profile(workspace, 'test_map', str(workspace / 'departure.yaml'),
                     str(workspace / 'arrival.yaml'), text_fields(sample_fields()),
                     text_fields(sample_fields(10.0)))
    assert read_profile(workspace, 'test_map') == before


def test_incomplete_source_is_rejected_before_creating_folder(workspace):
    site = workspace / 'config/site.yaml'
    data = yaml.safe_load(site.read_text())
    data['ros_domain_id'] = None
    site.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='ros_domain_id'):
        create_profile(workspace, 'new_map')
    assert not (workspace / 'config/profiles/new_map').exists()


def test_running_installed_controller_is_detected(monkeypatch):
    process = SimpleNamespace(info={'pid': 999999,
        'cmdline': ['/usr/bin/python3', '/opt/ros/lib/elevator_entry_map_diff']})
    monkeypatch.setattr('elevator_profiles.psutil.process_iter', lambda _attrs: [process])
    assert control_active()


def test_editor_shows_active_arrival_and_clears_review_after_edit(workspace):
    from PySide6.QtTest import QTest
    from PySide6.QtWidgets import QApplication
    from elevator_profile_editor import ElevatorProfileEditor

    app = QApplication.instance() or QApplication([])
    editor = ElevatorProfileEditor(workspace)
    assert editor.fields['arrival']['door_left_x'].text() == '50.0'

    create_profile(workspace, 'new_map')
    editor.selector.addItem('new_map')
    editor.selector.setCurrentText('new_map')
    assert editor.fields['arrival']['door_left_x'].text() == '50.0'
    assert not editor.reviewed.isChecked()

    editor.reviewed.setChecked(True)
    field = editor.fields['departure']['door_left_x']
    field.setFocus()
    QTest.keyClicks(field, '1')
    assert not editor.reviewed.isChecked()
    assert editor.status.property('level') == 'warning'
    editor.close()
