#!/usr/bin/env python3
"""Edit saved elevator geometry for each map without starting ROS control."""

import os
import sys
from pathlib import Path

import yaml
from PySide6.QtCore import QProcess, QProcessEnvironment, Qt
from PySide6.QtWidgets import (QApplication, QCheckBox, QComboBox, QFileDialog,
    QFrame, QGridLayout, QGroupBox, QHBoxLayout, QInputDialog, QLabel,
    QLineEdit, QMainWindow, QPushButton, QScrollArea, QTabWidget, QToolButton,
    QVBoxLayout, QWidget)

from elevator_profiles import (DEFAULT_ROOT, DOOR, ENTRY_EXTRA, FRONT, LOCAL,
                               create_profile, effective_arrival_path, profiles,
                               read_profile, save_profile)


STYLE = '''
QMainWindow, QWidget#root, QWidget#tabPage { background: #f3f6fb; color: #193047; }
QWidget { font-size: 14px; }
QFrame#hero { background: #183b5c; border-radius: 14px; }
QLabel#eyebrow { color: #a9d8f7; font-size: 11px; font-weight: 700; letter-spacing: 1px; }
QLabel#heroTitle { color: white; font-size: 24px; font-weight: 700; }
QLabel#heroText { color: #d9e9f5; }
QFrame#profileCard, QFrame#footer { background: white; border: 1px solid #dce5ef; border-radius: 12px; }
QLabel#sectionTitle { color: #1e405e; font-size: 16px; font-weight: 700; }
QLabel#hint { color: #667b8e; font-size: 12px; }
QLabel#step { background: #eaf2f8; color: #315873; border-radius: 7px; padding: 6px 10px; }
QLabel#sideTag { color: white; border-radius: 7px; padding: 6px 10px; font-weight: 700; }
QLabel#sideTag[side="departure"] { background: #2766a2; }
QLabel#sideTag[side="arrival"] { background: #138478; }
QTabWidget::pane { border: none; }
QTabBar::tab { background: #e5edf5; color: #4a6279; padding: 10px 25px;
    margin-right: 5px; border-top-left-radius: 8px; border-top-right-radius: 8px; font-weight: 600; }
QTabBar::tab:selected { background: white; color: #1d587f; }
QScrollArea { border: none; }
QGroupBox { background: white; border: 1px solid #dce5ef; border-radius: 10px;
    margin-top: 13px; padding: 18px 16px 14px; font-weight: 700; color: #254866; }
QGroupBox::title { subcontrol-origin: margin; left: 14px; padding: 0 6px; }
QLineEdit, QComboBox { background: #fbfdff; color: #18344d; border: 1px solid #c7d5e3;
    border-radius: 7px; padding: 8px 10px; min-height: 20px; selection-background-color: #2672ae; }
QLineEdit:focus, QComboBox:focus { border-color: #2672ae; background: white; }
QLineEdit:read-only { background: #f1f4f8; color: #536a7e; }
QPushButton { background: white; color: #254866; border: 1px solid #c6d5e4;
    border-radius: 7px; padding: 8px 14px; min-height: 22px; font-weight: 600; }
QPushButton:hover { background: #eaf4fc; border-color: #7ba9cb; }
QPushButton#primary { background: #2469a2; border-color: #2469a2; color: white; }
QPushButton#primary:hover { background: #1c5788; }
QPushButton:disabled { background: #edf1f5; color: #91a0ad; border-color: #e1e7ed; }
QToolButton { color: #28638e; background: transparent; border: none; padding: 7px; font-weight: 600; }
QToolButton:hover { color: #174466; }
QCheckBox { color: #254866; spacing: 9px; font-weight: 600; }
QCheckBox::indicator { width: 18px; height: 18px; }
QLabel#status { border-radius: 8px; padding: 10px 12px; font-weight: 600; }
QLabel#status[level="info"] { background: #edf4fa; color: #2c5777; }
QLabel#status[level="warning"] { background: #fff4dc; color: #805c16; }
QLabel#status[level="error"] { background: #fcebec; color: #9c3039; }
QLabel#status[level="success"] { background: #e7f5ed; color: #226b46; }
'''


LABELS = {
    'door_left_x': '左 X [m]', 'door_left_y': '左 Y [m]',
    'door_right_x': '右 X [m]', 'door_right_y': '右 Y [m]',
    'elevator_inside_sign': 'かご内側方向',
    'elevator_local_x_min': 'かご X 最小 [m]',
    'elevator_local_x_max': 'かご X 最大 [m]',
    'elevator_local_y_min': 'かご Y 最小 [m]',
    'elevator_local_y_max': 'かご Y 最大 [m]',
    'door_front_local_x_min': 'ドア前 X 最小 [m]',
    'door_front_local_x_max': 'ドア前 X 最大 [m]',
    'door_front_local_y_min': 'ドア前 Y 最小 [m]',
    'door_front_local_y_max': 'ドア前 Y 最大 [m]',
    'elevator_polygon_x': '頂点 X [m]',
    'elevator_polygon_y': '頂点 Y [m]',
    'manual_exit_local_x': '退出停止 X [m]',
    'manual_exit_local_y': '退出停止 Y [m]',
}


def display(value):
    if value is None:
        return ''
    if isinstance(value, list):
        return ', '.join(str(item) for item in value)
    return str(value)


class ElevatorProfileEditor(QMainWindow):
    def __init__(self, root=DEFAULT_ROOT):
        super().__init__()
        self.root = Path(root)
        self.setWindowTitle('エレベーター地図・座標設定')
        self.setMinimumSize(760, 620)
        self.resize(1000, 850)
        self.setStyleSheet(STYLE)
        self.map_edits = {}
        self.map_buttons = {}
        self.fields = {}
        self.detail_toggles = {}
        self.check_process = None
        self.check_sequence = 0

        root_widget = QWidget(objectName='root')
        self.setCentralWidget(root_widget)
        main = QVBoxLayout(root_widget)
        main.setContentsMargins(18, 16, 18, 16)
        main.setSpacing(12)

        hero = QFrame(objectName='hero')
        hero_layout = QVBoxLayout(hero)
        hero_layout.setContentsMargins(22, 15, 22, 16)
        hero_layout.setSpacing(3)
        hero_layout.addWidget(QLabel('ROBOTBASE  /  ELEVATOR', objectName='eyebrow'))
        hero_layout.addWidget(QLabel('地図ごとのエレベーター座標', objectName='heroTitle'))
        hero_layout.addWidget(QLabel('地図別に保存し、kuwa_ws2 の操作 GUI で使う設定を選びます。', objectName='heroText'))
        main.addWidget(hero)

        profile_card = QFrame(objectName='profileCard')
        profile_layout = QVBoxLayout(profile_card)
        profile_layout.setContentsMargins(16, 12, 16, 12)
        profile_layout.setSpacing(9)
        profile_row = QHBoxLayout()
        profile_row.addWidget(QLabel('環境設定', objectName='sectionTitle'))
        self.selector = QComboBox()
        names = profiles(self.root)
        self.selector.addItems(names)
        selected = os.environ.get('ELEVATOR_PROFILE', 'current')
        if selected in names:
            self.selector.setCurrentText(selected)
        self.selector.currentTextChanged.connect(self.load)
        profile_row.addWidget(self.selector, 1)
        create = QPushButton('＋ 新しい地図用に複製')
        create.clicked.connect(self.create)
        profile_row.addWidget(create)
        profile_layout.addLayout(profile_row)
        steps = QHBoxLayout()
        for text in ('1  設定を複製', '2  地図と座標を入力', '3  確認して保存'):
            steps.addWidget(QLabel(text, objectName='step'))
        steps.addStretch()
        profile_layout.addLayout(steps)
        main.addWidget(profile_card)

        self.tabs = QTabWidget()
        self.tabs.addTab(self._side_tab('departure', '出発側',
                                    '乗る階の地図とドア・かごを入力'), '出発側  ·  乗る階')
        self.tabs.addTab(self._side_tab('arrival', '到着側',
                                    '降りる階の地図とドア・かごを入力'), '到着側  ·  降りる階')
        main.addWidget(self.tabs, 1)

        footer = QFrame(objectName='footer')
        footer_layout = QVBoxLayout(footer)
        footer_layout.setContentsMargins(16, 11, 16, 12)
        footer_layout.setSpacing(8)
        self.reviewed = QCheckBox('地図と座標を実測値・現地表示と照合した')
        footer_layout.addWidget(self.reviewed)
        actions = QHBoxLayout()
        actions.addWidget(QLabel('制御ノードを停止してから保存してください。', objectName='hint'))
        actions.addStretch()
        check = QPushButton('設定をチェック')
        check.clicked.connect(self.check)
        actions.addWidget(check)
        self.save_button = QPushButton('この設定を保存')
        self.save_button.setObjectName('primary')
        self.save_button.clicked.connect(self.save)
        actions.addWidget(self.save_button)
        footer_layout.addLayout(actions)
        self.status = QLabel(objectName='status')
        self.status.setWordWrap(True)
        footer_layout.addWidget(self.status)
        main.addWidget(footer)

        self.load(self.selector.currentText())

    def _side_tab(self, side, title, subtitle):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        page = QWidget(objectName='tabPage')
        scroll.setWidget(page)
        layout = QVBoxLayout(page)
        layout.setContentsMargins(4, 12, 4, 12)
        layout.setSpacing(10)
        tag = QLabel(title, objectName='sideTag')
        tag.setProperty('side', side)
        tag.setFixedWidth(88)
        layout.addWidget(tag)
        layout.addWidget(QLabel(subtitle, objectName='hint'))

        map_group = QGroupBox('地図')
        map_layout = QVBoxLayout(map_group)
        map_layout.addWidget(QLabel('Nav2 で使う地図 YAML を選択', objectName='hint'))
        map_row = QHBoxLayout()
        map_edit = QLineEdit()
        map_edit.setPlaceholderText('地図 YAML の絶対パス')
        map_edit.textEdited.connect(self.mark_dirty)
        map_row.addWidget(map_edit, 1)
        choose = QPushButton('参照…')
        choose.clicked.connect(lambda _=False, s=side: self.choose_map(s))
        map_row.addWidget(choose)
        map_layout.addLayout(map_row)
        self.map_edits[side] = map_edit
        self.map_buttons[side] = choose
        layout.addWidget(map_group)

        self.fields[side] = {}
        door = QGroupBox('ドアの両端  ·  map 座標')
        door_layout = QVBoxLayout(door)
        door_layout.addWidget(QLabel('ドアを外側から見た左端・右端を入力します。', objectName='hint'))
        door_grid = QGridLayout()
        self._add_pair(door_grid, side, 0, DOOR[0], DOOR[1])
        self._add_pair(door_grid, side, 1, DOOR[2], DOOR[3])
        door_layout.addLayout(door_grid)
        sign_row = QHBoxLayout()
        sign_row.addWidget(QLabel('かご内側方向'))
        sign_row.addWidget(self._edit(side, 'elevator_inside_sign'))
        sign_row.addWidget(QLabel('+1 または -1。既存設定を参考にしてください。', objectName='hint'))
        sign_row.addStretch()
        door_layout.addLayout(sign_row)
        layout.addWidget(door)

        cabin = QGroupBox('かごの輪郭  ·  map 座標')
        cabin_layout = QVBoxLayout(cabin)
        cabin_layout.addWidget(QLabel('頂点を周回順に、X と Y を同じ個数だけ入力します。', objectName='hint'))
        for key in ('elevator_polygon_x', 'elevator_polygon_y'):
            row = QHBoxLayout()
            row.addWidget(QLabel(LABELS[key]))
            edit = self._edit(side, key)
            edit.setPlaceholderText('例: 1.0, 2.0, 2.0, 1.0')
            row.addWidget(edit, 1)
            cabin_layout.addLayout(row)
        layout.addWidget(cabin)

        toggle = QToolButton()
        toggle.setCheckable(True)
        toggle.setChecked(False)
        toggle.setText('▶  詳細設定を表示  ·  local 範囲と退出位置')
        layout.addWidget(toggle)
        detail = QGroupBox('詳細設定  ·  ドア中心を原点とする local 座標')
        detail_layout = QVBoxLayout(detail)
        detail_layout.addWidget(QLabel('かごとドア前の判定範囲です。新しい地図では値を確認してください。', objectName='hint'))
        detail_grid = QGridLayout()
        detail_keys = (*LOCAL, *FRONT, *(ENTRY_EXTRA if side == 'departure' else ()))
        for row, index in enumerate(range(0, len(detail_keys), 2)):
            left = detail_keys[index]
            right = detail_keys[index + 1] if index + 1 < len(detail_keys) else None
            self._add_pair(detail_grid, side, row, left, right)
        detail_layout.addLayout(detail_grid)
        detail.setVisible(False)
        toggle.toggled.connect(lambda checked, group=detail, button=toggle:
            self._toggle_detail(group, button, checked))
        self.detail_toggles[side] = toggle
        layout.addWidget(detail)
        layout.addStretch()
        return scroll

    @staticmethod
    def _toggle_detail(group, button, checked):
        group.setVisible(checked)
        button.setText(('▼  詳細設定を隠す' if checked else
                        '▶  詳細設定を表示  ·  local 範囲と退出位置'))

    def _edit(self, side, key):
        edit = QLineEdit()
        edit.setObjectName('coordinate')
        if not key.startswith('elevator_polygon_'):
            edit.setAlignment(Qt.AlignRight)
        edit.textEdited.connect(self.mark_dirty)
        self.fields[side][key] = edit
        return edit

    def _add_pair(self, grid, side, row, left, right):
        grid.addWidget(QLabel(LABELS[left]), row, 0)
        grid.addWidget(self._edit(side, left), row, 1)
        if right:
            grid.addWidget(QLabel(LABELS[right]), row, 2)
            grid.addWidget(self._edit(side, right), row, 3)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(3, 1)

    def set_status(self, message, level='info'):
        self.status.setText(message)
        self.status.setProperty('level', level)
        self.status.style().unpolish(self.status)
        self.status.style().polish(self.status)

    def mark_dirty(self, _text=''):
        if self.selector.currentText() == 'current' or not self.save_button.isEnabled():
            return
        self.reviewed.setChecked(False)
        self.set_status('未保存の変更があります。両方の地図と座標を確認してから保存してください。', 'warning')

    def load(self, name):
        self.check_sequence += 1
        try:
            config = read_profile(self.root, name)
            entry = config['entry.local.yaml']['elevator_entry_map_diff']['ros__parameters']
            arrival_file = effective_arrival_path(self.root, name, entry)
            arrival_data = (yaml.safe_load(arrival_file.read_text()) if arrival_file.is_file()
                            else config['arrival.local.yaml'])
            arrival = arrival_data['arrival_geometry']
            target = (entry.get('exit_map_yaml_path') if entry.get('switch_map_after_exit')
                      else arrival.get('map_yaml_path'))
            self.map_edits['departure'].setText(display(entry.get('map_yaml_path')))
            self.map_edits['arrival'].setText(display(target))
            for side, values in (('departure', entry), ('arrival', arrival)):
                for key, widget in self.fields[side].items():
                    widget.setText(display(values.get(key)))
                detail_keys = (*LOCAL, *FRONT, *(ENTRY_EXTRA if side == 'departure' else ()))
                self.detail_toggles[side].setChecked(any(values.get(key) is None for key in detail_keys))
            self.reviewed.setChecked(config['site.yaml'].get('measurements_reviewed') is True)
            bank_mode = config['site.yaml'].get('elevator_mode') == 'bank'
            readonly = name == 'current' or bank_mode
            self.save_button.setEnabled(not readonly)
            self.reviewed.setEnabled(not readonly)
            for widget in (*self.map_edits.values(),
                           *(item for group in self.fields.values() for item in group.values())):
                widget.setReadOnly(readonly)
            for button in self.map_buttons.values():
                button.setEnabled(not readonly)
            if bank_mode:
                self.set_status('複数基設定はこの1基用画面では保存できません。各号機の座標は kuwa_ws2/config/profiles/'
                                + name + '/bank.yaml と arrival_*.yaml にあります。実測未確認の間、制御起動はできません。',
                                'warning')
            else:
                self.set_status('current は参照専用です。新しい地図には上の「複製」を押してください。'
                                if readonly else f'編集中: {name}。保存後、kuwa_ws2 の操作 GUI で同じ名前を選択します。',
                                'info')
        except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
            self.set_status(str(exc), 'error')

    def create(self):
        source = self.selector.currentText()
        name, ok = QInputDialog.getText(self, '新しい環境設定',
                                        '名前（英数字・_・-）:', text='new_map')
        if not ok:
            return
        try:
            create_profile(self.root, name.strip(), source)
        except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
            self.set_status(f'作成できません: {exc}', 'error')
            return
        self.selector.addItem(name.strip())
        self.selector.setCurrentText(name.strip())
        self.reviewed.setChecked(False)
        self.set_status('新しい設定を作成しました。出発側・到着側の地図と座標を入力してください。', 'warning')

    def choose_map(self, side):
        if self.selector.currentText() == 'current':
            return
        start = self.map_edits[side].text().strip()
        if not start or not Path(start).exists():
            start = str(self.root.parent / 'robotbase_ws_renya/maps_waypoints/maps')
        path, _ = QFileDialog.getOpenFileName(self, '地図YAMLを選択', start,
                                               '地図YAML (*.yaml *.yml)')
        if path:
            self.map_edits[side].setText(path)
            self.mark_dirty()

    def save(self):
        name = self.selector.currentText()
        try:
            save_profile(self.root, name,
                         self.map_edits['departure'].text().strip(),
                         self.map_edits['arrival'].text().strip(),
                         {key: edit.text() for key, edit in self.fields['departure'].items()},
                         {key: edit.text() for key, edit in self.fields['arrival'].items()},
                         self.reviewed.isChecked())
        except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
            self.set_status(f'保存できません: {exc}', 'error')
            return
        self.set_status('保存しました。設定チェックを実行しています…', 'info')
        self.check()

    def check(self):
        name = self.selector.currentText()
        bank_mode = read_profile(self.root, name)['site.yaml'].get('elevator_mode') == 'bank'
        script = self.root / 'tools/configure.py'
        if not script.is_file():
            self.set_status(f'設定チェックがありません: {script}', 'warning')
            return
        self.check_sequence += 1
        sequence = self.check_sequence
        previous = self.check_process
        if previous is not None and previous.state() != QProcess.NotRunning:
            previous.kill()
        process = QProcess(self)
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.setWorkingDirectory(str(self.root))
        environment = QProcessEnvironment.systemEnvironment()
        environment.insert('ELEVATOR_PROFILE', name)
        process.setProcessEnvironment(environment)
        self.check_process = process
        self.set_status(f'{name} の設定をチェック中…', 'info')

        def finished(code, _status):
            output = bytes(process.readAllStandardOutput()).decode(errors='replace').strip()
            if sequence == self.check_sequence and self.selector.currentText() == name:
                if code == 0:
                    if bank_mode:
                        self.set_status('5基の座標案チェックOK。実測未確認のため制御起動はできません。', 'warning')
                    else:
                        self.set_status(f'設定チェックOK。kuwa_ws2 の操作 GUI で「{name}」を選択してください。', 'success')
                else:
                    self.set_status('設定チェック未完了: ' + (output or f'終了コード {code}'), 'warning')
            if self.check_process is process:
                self.check_process = None
            process.deleteLater()

        process.finished.connect(finished)
        process.errorOccurred.connect(
            lambda _error: self.set_status('設定チェックを起動できません: ' +
                                           process.errorString(), 'error')
            if sequence == self.check_sequence and self.selector.currentText() == name else None)
        process.start('/usr/bin/python3', [str(script), 'draft-check' if bank_mode else 'check'])


def main():
    app = QApplication(sys.argv)
    window = ElevatorProfileEditor()
    window.show()
    return app.exec()


if __name__ == '__main__':
    sys.exit(main())
