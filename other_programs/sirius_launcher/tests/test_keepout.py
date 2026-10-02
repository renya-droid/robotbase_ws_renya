import importlib.util
import json
from pathlib import Path
from unittest.mock import patch

import pytest
import yaml
from PySide6.QtCore import Qt
from PySide6.QtGui import QImage
from PySide6.QtWidgets import QApplication

from keepout_editor import KeepoutEditor, settings_path


@pytest.fixture
def app():
    return QApplication.instance() or QApplication([])


@pytest.fixture
def map_file(tmp_path):
    image = QImage(100, 80, QImage.Format_RGB32)
    image.fill(Qt.white)
    image.save(str(tmp_path / 'map.png'))
    p = tmp_path / 'map.yaml'
    p.write_text(yaml.safe_dump(dict(image='map.png', resolution=0.05, origin=[-2.,-3.,0.])))
    return p


def test_save_reopen_and_mask_geometry(app, map_file):
    with patch('keepout_editor.QFileDialog.getOpenFileName', return_value=(str(map_file), '')), \
         patch('keepout_editor.QMessageBox.information'), \
         patch('keepout_editor.QMessageBox.critical') as error:
        editor = KeepoutEditor(workspace=map_file.parent)
        editor.open_map()
        editor.canvas.shapes = [[[10,10],[30,10],[30,30],[10,30]]]
        editor.enabled.setChecked(True)
        editor.save()
        assert not error.called
        state = json.loads(settings_path(map_file).read_text())
        assert state['enabled']
        mask_path = settings_path(map_file).parent
        mask = QImage(str(mask_path / 'mask.png'))
        assert mask.pixelColor(20,20).value() == 0
        assert mask.pixelColor(50,50).value() == 255
        assert mask.size() == editor.canvas.image.size()
        meta = yaml.safe_load((mask_path / 'mask.yaml').read_text())
        assert meta['origin'] == [-2.,-3.,0.]
        other = KeepoutEditor(workspace=map_file.parent)
        other.open_map()
        assert other.canvas.shapes == state['shapes']
        assert other.enabled.isChecked()
        editor.close()
        other.close()


def test_launch_keepout_disabled_enabled_and_wrong_map(app, map_file):
    from launch import LaunchContext
    from launch.actions import SetLaunchConfiguration
    path = Path('src/robotbase_bringup/launch/nav2.launch.py')
    spec = importlib.util.spec_from_file_location('keepout_launch_test', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    context = LaunchContext()
    context.launch_configurations.update(map=str(map_file), localization='amcl',
        params_file=str(Path('params/sim/nav2.yaml').resolve()), use_sim_time='true')
    assert mod.configure_keepout(context) == []
    state_path = settings_path(map_file)
    state_path.parent.mkdir(parents=True)
    state = dict(enabled=False, source_map=str(map_file),
                 map_metadata=dict(resolution=0.05,origin=[-2.,-3.,0.],size=[100,80]))
    state_path.write_text(json.dumps(state))
    assert mod.configure_keepout(context) == []
    state['enabled'] = True
    state_path.write_text(json.dumps(state))
    (state_path.parent / 'mask.yaml').write_text('image: mask.png\n')
    actions = mod.configure_keepout(context)
    assert len(actions) == 4
    assert isinstance(actions[0], SetLaunchConfiguration)
    actions[0].execute(context)
    params = yaml.safe_load(Path(context.launch_configurations['params_file']).read_text())
    for name in ['global_costmap','local_costmap']:
        assert params[name][name]['ros__parameters']['filters'] == ['keepout_filter']
    state['source_map'] = '/different/map.yaml'
    state_path.write_text(json.dumps(state))
    with pytest.raises(RuntimeError, match='different map'):
        mod.configure_keepout(context)


def test_editor_button_survives_launcher_reload(app):
    from sirius_launcher import RobotLauncher
    window = RobotLauncher()
    assert any(window.preset_layout.itemAt(i).widget().text() == '進入禁止領域を編集'
               for i in range(window.preset_layout.count()))
    window.reload_launcher()
    assert any(window.preset_layout.itemAt(i).widget().text() == '進入禁止領域を編集'
               for i in range(window.preset_layout.count()))
    window.close()
