import importlib.util
import time
from pathlib import Path
from types import SimpleNamespace as NS
import yaml
import pytest

spec = importlib.util.spec_from_file_location('preview', Path(__file__).parents[1]/'scripts/elevator_geometry_preview.py')
preview = importlib.util.module_from_spec(spec)
spec.loader.exec_module(preview)


def config(path):
    return {'elevator_entry_map_diff': {'ros__parameters': dict(
        map_yaml_path=str(path), door_left_x=0., door_left_y=0.,
        door_right_x=1., door_right_y=0., use_elevator_polygon=True,
        elevator_polygon_x=[0., 1., 1., 0.], elevator_polygon_y=[0., 0., 2., 2.])}}


def test_polygon_is_closed():
    lines = preview.geometry(config('/map.yaml'))
    assert lines[0] == ((0., 0.), (1., 0.))
    assert lines[1][0] == lines[1][-1] and len(lines[1]) == 5


def test_invalid_geometry_rejected():
    c = config('/map.yaml')
    c['elevator_entry_map_diff']['ros__parameters']['door_left_x'] = float('nan')
    with pytest.raises(ValueError):
        preview.geometry(c)


def test_preview_selects_unique_profile_for_active_map(tmp_path):
    root = tmp_path / 'config'
    root.mkdir()
    current = root / 'entry.local.yaml'
    current.write_text(yaml.safe_dump(config(tmp_path / 'original.yaml')))
    profile = root / 'profiles' / 'new_map' / 'entry.local.yaml'
    profile.parent.mkdir(parents=True)
    chosen = config(tmp_path / 'new.yaml')
    chosen['elevator_entry_map_diff']['ros__parameters']['door_left_x'] = 9.0
    profile.write_text(yaml.safe_dump(chosen))
    (profile.parent / 'site.yaml').write_text('measurements_reviewed: true\n')
    selected = preview.config_for_map(str(tmp_path / 'new.yaml'), current)
    assert selected['elevator_entry_map_diff']['ros__parameters']['door_left_x'] == 9.0
    duplicate = root / 'profiles' / 'duplicate' / 'entry.local.yaml'
    duplicate.parent.mkdir(parents=True)
    duplicate.write_text(yaml.safe_dump(chosen))
    (duplicate.parent / 'site.yaml').write_text('measurements_reviewed: true\n')
    assert preview.config_for_map(str(tmp_path / 'new.yaml'), current) is None
    (duplicate.parent / 'site.yaml').write_text('measurements_reviewed: false\n')
    assert preview.config_for_map(str(tmp_path / 'new.yaml'), current) is not None


def test_explicit_bank_profile_can_share_current_map(tmp_path):
    root = tmp_path / 'config'
    root.mkdir()
    current = root / 'entry.local.yaml'
    current.write_text(yaml.safe_dump(config(tmp_path / '1018.yaml')))
    bank = root / 'profiles' / 'bank' / 'entry.local.yaml'
    bank.parent.mkdir(parents=True)
    selected = config(tmp_path / '1018.yaml')
    selected['elevator_entry_map_diff']['ros__parameters']['elevator_bank_config'] = 'bank.yaml'
    bank.write_text(yaml.safe_dump(selected))
    state = tmp_path / 'records' / 'rviz_profile.txt'
    state.parent.mkdir()
    state.write_text('bank\n')
    chosen, path = preview.selected_config_for_map(str(tmp_path / '1018.yaml'), current)
    assert path == bank
    assert chosen['elevator_entry_map_diff']['ros__parameters']['elevator_bank_config'] == 'bank.yaml'
    state.write_text('current\n')
    _, path = preview.selected_config_for_map(str(tmp_path / '1018.yaml'), current)
    assert path == current
    assert preview.selected_config_for_map(str(tmp_path / 'other.yaml'), current) == (None, None)


@pytest.mark.parametrize('matching', [True, False])
def test_preview_only_on_configured_map(tmp_path, matching):
    cfg = tmp_path/'entry.yaml'
    cfg.write_text(yaml.safe_dump(config(tmp_path/'1018.yaml')))
    messages = []
    active = tmp_path/('1018.yaml' if matching else 'other.yaml')
    node = NS(path=cfg, pending=NS(done=lambda: True,
        result=lambda: NS(values=[NS(string_value=str(active))])),
        pub=NS(publish=messages.append),
        get_clock=lambda: NS(now=lambda: NS(to_msg=lambda: preview.Marker().header.stamp)))
    node.clear=lambda: preview.Preview.clear(node)
    preview.Preview.tick(node)
    assert len(messages) == 1
    assert messages[0].markers[0].action == preview.Marker.DELETEALL
    if matching:
        assert len(messages[0].markers) == 3
        assert all(m.action == preview.Marker.ADD for m in messages[0].markers[1:])
        assert all(m.ns == 'configured_elevator_preview' for m in messages[0].markers[1:])
    else:
        assert len(messages[0].markers) == 1


def test_live_controller_hides_static_preview():
    messages = []
    node = NS(last_control_state_at=time.monotonic(), pub=NS(publish=messages.append))
    node.clear = lambda: preview.Preview.clear(node)
    preview.Preview.tick(node)
    assert len(messages) == 1
    assert messages[0].markers[0].action == preview.Marker.DELETEALL


@pytest.mark.parametrize('before_exit', [True, False])
def test_selected_profile_displays_arrival_map_geometry(tmp_path, before_exit):
    root = tmp_path / 'config'
    root.mkdir()
    current = root / 'entry.local.yaml'
    current.write_text(yaml.safe_dump(config(tmp_path / 'old.yaml')))
    path = root / 'profiles' / 'nakanoshima' / 'entry.local.yaml'
    path.parent.mkdir(parents=True)
    departure = config(tmp_path / 'atc4.yaml')
    entry = departure['elevator_entry_map_diff']['ros__parameters']
    entry.update(switch_map_before_exit=before_exit, switch_map_after_exit=not before_exit,
                 elevator_bank_config='departure_bank.yaml')
    entry['arrival_geometry_file' if before_exit else 'exit_map_geometry_file'] = 'arrival.local.yaml'
    path.write_text(yaml.safe_dump(departure))
    arrival = config(tmp_path / 'atc6.yaml')['elevator_entry_map_diff']['ros__parameters']
    arrival.update(door_left_x=3., door_right_x=4., elevator_polygon_x=[3., 4., 4., 3.])
    (path.parent / 'arrival.local.yaml').write_text(yaml.safe_dump({'arrival_geometry': arrival}))
    state = tmp_path / 'records' / 'rviz_profile.txt'
    state.parent.mkdir()
    state.write_text('nakanoshima\n')
    chosen, chosen_path = preview.selected_config_for_map(str(tmp_path / 'atc6.yaml'), current)
    assert chosen_path == path
    assert preview.geometry(chosen)[0] == ((3., 0.), (4., 0.))
    assert not chosen['elevator_entry_map_diff']['ros__parameters'].get('elevator_bank_config')
    # Matching the departure map still returns the unmodified departure geometry.
    chosen, _ = preview.selected_config_for_map(str(tmp_path / 'atc4.yaml'), current)
    assert chosen == departure
    assert yaml.safe_load(path.read_text()) == departure
    assert preview.selected_config_for_map(str(tmp_path / 'unrelated.yaml'), current) == (None, None)


def test_auto_preview_finds_reviewed_arrival_map(tmp_path):
    root = tmp_path / 'config'
    root.mkdir()
    current = root / 'entry.local.yaml'
    current.write_text(yaml.safe_dump(config(tmp_path / 'old.yaml')))
    path = root / 'profiles' / 'new_map' / 'entry.local.yaml'
    path.parent.mkdir(parents=True)
    departure = config(tmp_path / 'departure.yaml')
    departure['elevator_entry_map_diff']['ros__parameters'].update(
        switch_map_before_exit=True, arrival_geometry_file='arrival.local.yaml')
    path.write_text(yaml.safe_dump(departure))
    arrival = config(tmp_path / 'arrival.yaml')['elevator_entry_map_diff']['ros__parameters']
    (path.parent / 'arrival.local.yaml').write_text(yaml.safe_dump({'arrival_geometry': arrival}))
    site = path.parent / 'site.yaml'
    site.write_text('measurements_reviewed: true\n')
    assert preview.config_for_map(str(tmp_path / 'arrival.yaml'), current) is not None
    site.write_text('measurements_reviewed: false\n')
    assert preview.config_for_map(str(tmp_path / 'arrival.yaml'), current) is None
