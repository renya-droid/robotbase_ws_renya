import importlib.util
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
    assert all(m.action == (preview.Marker.ADD if matching else preview.Marker.DELETE) for m in messages[0].markers)
    assert all(m.ns == 'configured_elevator_preview' for m in messages[0].markers)
