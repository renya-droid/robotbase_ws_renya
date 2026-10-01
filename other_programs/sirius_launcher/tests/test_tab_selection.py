import os
import sys
from pathlib import Path
import tempfile
import unittest

# Ensure the project root is on sys.path so imports work
sys.path.insert(0, str(Path.cwd()))
sys.path.insert(0, str(Path.cwd().joinpath('other_programs', 'sirius_launcher')))

from PySide6.QtWidgets import QApplication
from PySide6.QtGui import QMouseEvent
from PySide6.QtCore import QEvent, QPointF, Qt

from other_programs.sirius_launcher.robot_launcher import RobotLauncher
from other_programs.sirius_launcher.alias_parser import parse_bash_aliases
from other_programs.sirius_launcher.robot_config import (
    ALIAS_FILE,
    SIM_ROS_DOMAIN_ID,
    _read_env_file,
    save_sim_ros_domain_id,
)


class TestTabSelection(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ["QT_QPA_PLATFORM"] = "offscreen"
        cls.app = QApplication([])

    def test_select_tab_on_click_running(self):
        window = RobotLauncher()
        layout, group_widget = window.add_group('TestGroup', tab_name='センサー・ハードウェア')
        window.add_button(layout, 'Test', 'echo "Test"', 'desc', group_widget)
        btn = window.buttons[-1]
        # Force the process_manager to appear running
        btn.process_manager.is_running = lambda: True

        # Simulate mouse press on the launch button (child widget event filter should catch it)
        point = QPointF(1, 1)
        event = QMouseEvent(
            QEvent.MouseButtonPress,
            point,
            point,
            point,
            Qt.LeftButton,
            Qt.LeftButton,
            Qt.NoModifier,
        )
        btn.launch_btn.event(event)

        self.assertEqual(window.tab_widget.currentIndex(), btn.tab_index)

    def test_stop_all_simulation_button_is_visible(self):
        window = RobotLauncher()

        self.assertTrue(window.stop_simulation_btn.isVisibleTo(window))
        self.assertIn('シミュレーション一式を終了', window.stop_simulation_btn.text())
        self.assertEqual(window.sim_domain_spin.value(), int(SIM_ROS_DOMAIN_ID))
        self.assertIn('Domainを保存', window.save_sim_domain_btn.text())

    def test_simulation_domain_is_persisted_without_losing_other_settings(self):
        with tempfile.TemporaryDirectory() as directory:
            config_file = Path(directory) / 'robot.env'
            config_file.write_text(
                '# keep this comment\n'
                'ROBOTBASE_DISPLAY_NAME="ココちゃん"\n'
                'ROBOTBASE_ROS_DOMAIN_ID="57"\n'
                'ROBOTBASE_SIM_ROS_DOMAIN_ID="58"\n'
                'ROBOTBASE_GZ_PARTITION="koko"\n',
                encoding='utf-8',
            )

            saved = save_sim_ros_domain_id('59', config_file)
            values = _read_env_file(config_file)

            self.assertEqual(saved, '59')
            self.assertEqual(values['ROBOTBASE_SIM_ROS_DOMAIN_ID'], '59')
            self.assertEqual(values['ROBOTBASE_DISPLAY_NAME'], 'ココちゃん')
            self.assertIn('# keep this comment', config_file.read_text(encoding='utf-8'))

            with self.assertRaises(ValueError):
                save_sim_ros_domain_id('57', config_file)

    def test_simulation_commands_are_grouped_and_split_by_map_mode(self):
        groups, presets = parse_bash_aliases(ALIAS_FILE)
        simulation_names = [item[0] for item in groups['シミュレーション']]
        real_names = [item[0] for item in groups['リアル実験']]
        utility_names = [item[0] for item in groups['ユーティリティ']]

        self.assertIn('koko_slamtoolbox_sim', simulation_names)
        self.assertIn('koko_nav2_sim_map', simulation_names)
        self.assertIn('koko_nav2_sim_slam', simulation_names)
        self.assertIn('koko_keyop2_sim', simulation_names)
        self.assertIn('koko_map_save_sim', simulation_names)
        self.assertNotIn('koko_twist_mux', simulation_names)
        self.assertIn('koko_twist_mux', real_names)
        self.assertIn('koko_nav2_real_slam', real_names)
        self.assertIn('koko_foxglove', utility_names)
        self.assertIn('koko_foxglove_sim', utility_names)

        simulation_commands = {
            name: command for name, command, _description in groups['シミュレーション']
        }
        for command in simulation_commands.values():
            self.assertIn('activate_koko_sim_env.sh', command)
            self.assertNotIn('koko_sim_src', command)
            self.assertNotIn('koko_sim_env &&', command)

        preset_names = [name for name, _ in presets]
        self.assertIn('自律移動（シミュレーション）', preset_names)
        self.assertIn('SLAMしながら自律移動（シミュレーション）', preset_names)
        self.assertIn('SLAMしながら自律移動（実機）', preset_names)


if __name__ == '__main__':
    unittest.main()
