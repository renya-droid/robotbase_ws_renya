from pathlib import Path
import sys

import pytest
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from configure import profile_dir,profile_paths,validate,snapshot,arrival_door_parameters
from new_profile import create


def test_current_selection_preserves_existing_files(monkeypatch):
    monkeypatch.delenv('ELEVATOR_PROFILE',raising=False)
    assert profile_dir()==ROOT/'config'
    assert profile_paths()['entry.local.yaml']==ROOT/'config/entry.local.yaml'


def test_nakanoshima_coordinates_are_separate_and_require_measurement_review(monkeypatch):
    before=(ROOT/'config/entry.local.yaml').read_bytes()
    monkeypatch.setenv('ELEVATOR_PROFILE','nakanoshima')
    with pytest.raises(ValueError) as exc:validate()
    assert 'measurements_reviewed' in str(exc.value)
    site,cfg=validate(allow_unreviewed=True)
    assert site['measurements_reviewed'] is False
    assert site['ros_domain_id']==54
    entry=yaml.safe_load(profile_paths()['entry.local.yaml'].read_text())['elevator_entry_map_diff']['ros__parameters']
    assert Path(entry['map_yaml_path']).name=='atc4.yaml'
    arrival=yaml.safe_load(Path(cfg['elevator_entry_map_diff']['ros__parameters']['arrival_geometry_file']).read_text())['arrival_geometry']
    assert Path(arrival['map_yaml_path']).name=='atc6.yaml'
    assert entry['switch_map_before_exit'] is True and entry['switch_map_after_exit'] is False
    assert entry['lift_auto_exit_after_stop_open'] is False and entry['arrival_exit_mode']=='map_only'
    assert (ROOT/'config/entry.local.yaml').read_bytes()==before


def test_1018_bank_trial_requires_explicit_flag(monkeypatch, tmp_path):
    import shutil
    monkeypatch.setenv('ELEVATOR_PROFILE','1018_bank_template')
    site,cfg=validate()
    assert site['measurements_reviewed'] is False
    assert site['allow_unreviewed_trial'] is True
    assert site['elevator_mode']=='bank'
    assert cfg['elevator_entry_map_diff']['ros__parameters']['elevator_bank_config'].endswith('/bank.yaml')
    target=tmp_path/'config/profiles/1018_bank_template'
    shutil.copytree(ROOT/'config/profiles/1018_bank_template',target)
    target_site=target/'site.yaml'
    data=yaml.safe_load(target_site.read_text())
    data['allow_unreviewed_trial']=False
    target_site.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError,match='measurements_reviewed'):
        validate(tmp_path)


def test_stationary_mock_uses_one_map_and_door_cycle_without_imu(monkeypatch):
    monkeypatch.setenv('ELEVATOR_PROFILE','mock_elevator')
    site,cfg=validate()
    entry=cfg['elevator_entry_map_diff']['ros__parameters']
    door=cfg['elevator_door_node']['ros__parameters']
    assert site['stationary_mock_elevator'] is True
    assert entry['lift_cycle_enabled'] is False
    assert entry['use_full_door_cycle'] and entry['use_door_open_check']
    assert not entry['switch_map_before_exit'] and not entry['switch_map_after_exit']
    assert entry['verify_active_map'] is True
    assert Path(entry['map_yaml_path']).name=='1119ele.yaml'
    assert (entry['door_left_x'],entry['door_left_y'],entry['door_right_x'],entry['door_right_y']) == (
        door['door_left_x'],door['door_left_y'],door['door_right_x'],door['door_right_y'])


@pytest.mark.parametrize('name',['../current','unknown','', '/tmp/other'])
def test_bad_profile_selection_cannot_fall_back(monkeypatch,name):
    monkeypatch.setenv('ELEVATOR_PROFILE',name)
    with pytest.raises(ValueError):profile_dir()


def test_manifest_keeps_version_and_selected_source_hashes(monkeypatch):
    monkeypatch.setenv('ELEVATOR_PROFILE','current')
    data=snapshot()
    assert data['profile']=='current' and len(data['commit'])==40
    assert 'config/entry.local.yaml' in data['files']
    assert 'src/elevator_space_navigation/elevator_space_navigation/arrival_map.py' in data['source_sha256']


def test_copy_profile_is_stationary_and_does_not_overwrite(tmp_path,monkeypatch):
    import shutil
    shutil.copytree(ROOT/'config',tmp_path/'config')
    before=(tmp_path/'config/entry.local.yaml').read_bytes()
    folder=create('bench',root=tmp_path)
    entry=yaml.safe_load((folder/'entry.local.yaml').read_text())['elevator_entry_map_diff']['ros__parameters']
    assert entry['arrival_geometry_file']=='arrival.local.yaml'
    assert entry['arrival_exit_mode']=='map_only' and entry['lift_auto_exit_after_stop_open'] is False
    assert yaml.safe_load((folder/'site.yaml').read_text())['require_human_destination'] is True
    assert (tmp_path/'config/entry.local.yaml').read_bytes()==before
    with pytest.raises(FileExistsError):create('bench',root=tmp_path)
    monkeypatch.setenv('ELEVATOR_PROFILE','bench')
    _,cfg=validate(tmp_path)
    assert cfg['elevator_entry_map_diff']['ros__parameters']['arrival_geometry_file']==str(folder/'arrival.local.yaml')
    arrival=arrival_door_parameters(cfg)
    source=cfg['elevator_door_node']['ros__parameters']
    for key in ('scan_topic','pose_topic','odom_topic','use_odom_pose','use_sim_time'):
        assert source[key]==arrival[key]
    assert arrival['door_left_x']!=source['door_left_x']
    assert arrival['door_state_topic']==cfg['elevator_entry_map_diff']['ros__parameters']['arrival_door_state_topic']
    cfg['elevator_door_arrival']={'ros__parameters':{'pose_topic':'/wrong_pose'}}
    with pytest.raises(ValueError,match='観測設定のみ'):arrival_door_parameters(cfg)
