#!/usr/bin/env python3
"""
Sirius シミュレーション起動設定UI
PySide6を使用したGUIで起動パラメータを設定
"""

import os
import sys
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                               QHBoxLayout, QCheckBox, QPushButton, QComboBox, 
                               QLabel, QGroupBox, QTextEdit, QDoubleSpinBox, QGridLayout,
                               QScrollArea)
from PySide6.QtCore import Qt, QEvent
from ament_index_python.packages import get_package_share_directory


class LaunchConfigUI(QMainWindow):
    """シミュレーション起動設定用のGUIウィンドウ"""
    
    def __init__(self):
        super().__init__()
        self.config = {
            'world_file': 'sirius_world.sdf',
            'spawn_robot': True,
            'spawn_x': 0.0,
            'spawn_y': 0.0,
            'spawn_z': 0.0825,
            'spawn_yaw': 1.5708,  # 90度 (π/2)
            'robot_state_publisher': True,
            'tf_bridge': False,  # EKF使用時はFalse（IMU融合TFを使用）
            'odom_bridge': True,
            'twist_bridge': True,
            'joint_state_bridge': True,
            'lidar_bridge': True,
            'lidar2_bridge': True,
            'imu_bridge': True,
            'velodyne_bridge': True,
            'clock_bridge': True,
            'teleop': True,
            'rviz': True,
            'headless': False,
            'run_on_start': True
        }
        self.launched = False
        self.init_ui()
        
    def init_ui(self):
        """UIの初期化"""
        self.setWindowTitle('Sirius シミュレーション起動設定')
        self.setGeometry(100, 100, 550, 700)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(5, 5, 5, 5)
        
        # スクロールエリアを作成
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        
        # スクロール内のコンテンツウィジェット
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)
        
        # ワールド選択セクション
        scroll_layout.addWidget(self._create_world_selection_group())
        
        # スポーン座標設定セクション
        scroll_layout.addWidget(self._create_spawn_position_group())
        
        # コンポーネント選択セクション
        scroll_layout.addWidget(self._create_components_group())
        
        # 説明表示エリア
        scroll_layout.addWidget(self._create_description_area())
        
        # スクロールエリアにコンテンツを設定
        scroll_area.setWidget(scroll_content)
        main_layout.addWidget(scroll_area)
        
        # 起動ボタン（スクロール外、常に下部に固定）
        main_layout.addLayout(self._create_launch_button())
        
    def _create_world_selection_group(self):
        """ワールド選択グループの作成"""
        world_group = QGroupBox("ワールド選択")
        world_layout = QVBoxLayout()
        
        world_label = QLabel("起動するワールドファイル:")
        world_layout.addWidget(world_label)
        
        self.world_combo = QComboBox()
        self.world_combo.description = "シミュレーション環境（ワールド）のSDFファイルを選択します"
        self.world_combo.installEventFilter(self)
        
        # worldsディレクトリからSDFファイルを検索（srcディレクトリを直接参照）
        # このファイルのパスから推測: launch_config_ui.py -> launch/ -> sirius_description/ -> worlds/
        launch_file_dir = os.path.dirname(os.path.abspath(__file__))
        src_pkg_path = os.path.dirname(launch_file_dir)  # sirius_descriptionディレクトリ
        worlds_path = os.path.join(src_pkg_path, 'worlds')
        
        if os.path.exists(worlds_path):
            sdf_files = sorted([f for f in os.listdir(worlds_path) if f.endswith('.sdf')])
            if sdf_files:
                self.world_combo.addItems(sdf_files)
            else:
                self.world_combo.addItem('sirius_world.sdf')
        else:
            self.world_combo.addItem('sirius_world.sdf')
        
        world_layout.addWidget(self.world_combo)
        world_group.setLayout(world_layout)
        
        return world_group
    
    def _create_spawn_position_group(self):
        """スポーン座標設定グループの作成"""
        spawn_group = QGroupBox("ロボットスポーン座標")
        spawn_layout = QGridLayout()
        
        # X座標
        x_label = QLabel("X [m]:")
        self.spawn_x_spin = QDoubleSpinBox()
        self.spawn_x_spin.setRange(-5000.0, 5000.0)
        self.spawn_x_spin.setDecimals(2)
        self.spawn_x_spin.setSingleStep(0.5)
        self.spawn_x_spin.setValue(self.config['spawn_x'])
        self.spawn_x_spin.description = "ロボットのX座標（前後方向）[m]"
        self.spawn_x_spin.installEventFilter(self)
        spawn_layout.addWidget(x_label, 0, 0)
        spawn_layout.addWidget(self.spawn_x_spin, 0, 1)
        
        # Y座標
        y_label = QLabel("Y [m]:")
        self.spawn_y_spin = QDoubleSpinBox()
        self.spawn_y_spin.setRange(-5000.0, 5000.0)
        self.spawn_y_spin.setDecimals(2)
        self.spawn_y_spin.setSingleStep(0.5)
        self.spawn_y_spin.setValue(self.config['spawn_y'])
        self.spawn_y_spin.description = "ロボットのY座標（左右方向）[m]"
        self.spawn_y_spin.installEventFilter(self)
        spawn_layout.addWidget(y_label, 0, 2)
        spawn_layout.addWidget(self.spawn_y_spin, 0, 3)
        
        # Z座標
        z_label = QLabel("Z [m]:")
        self.spawn_z_spin = QDoubleSpinBox()
        self.spawn_z_spin.setRange(-10.0, 10.0)
        self.spawn_z_spin.setDecimals(4)
        self.spawn_z_spin.setSingleStep(0.01)
        self.spawn_z_spin.setValue(self.config['spawn_z'])
        self.spawn_z_spin.description = "ロボットのZ座標（高さ）[m]。地面からの高さを設定"
        self.spawn_z_spin.installEventFilter(self)
        spawn_layout.addWidget(z_label, 1, 0)
        spawn_layout.addWidget(self.spawn_z_spin, 1, 1)
        
        # Yaw角度
        yaw_label = QLabel("Yaw [rad]:")
        self.spawn_yaw_spin = QDoubleSpinBox()
        self.spawn_yaw_spin.setRange(-3.1416, 3.1416)
        self.spawn_yaw_spin.setDecimals(4)
        self.spawn_yaw_spin.setSingleStep(0.1)
        self.spawn_yaw_spin.setValue(self.config['spawn_yaw'])
        self.spawn_yaw_spin.description = "ロボットの向き（Yaw角）[rad]。0=+X方向、π/2≈1.57=+Y方向"
        self.spawn_yaw_spin.installEventFilter(self)
        spawn_layout.addWidget(yaw_label, 1, 2)
        spawn_layout.addWidget(self.spawn_yaw_spin, 1, 3)
        
        spawn_group.setLayout(spawn_layout)
        return spawn_group
    
    def _create_components_group(self):
        """コンポーネント選択グループの作成"""
        components_group = QGroupBox("起動するコンポーネント")
        components_layout = QVBoxLayout()
        
        self.checkboxes = {}
        
        # ロボット関連
        components_layout.addWidget(self._create_section_label("ロボット:"))
        self._add_checkbox(
            components_layout, 
            'spawn_robot', 
            "ロボットをスポーン", 
            True,
            "Gazeboシミュレーション環境内にSirius3ロボットを配置します"
        )
        self._add_checkbox(
            components_layout, 
            'robot_state_publisher', 
            "Robot State Publisher", 
            True,
            "ロボットの各リンク間の座標変換（TF）を配信します。RVizでの可視化に必要です"
        )
        
        # ブリッジ関連
        components_layout.addWidget(self._create_section_label("\nROS-Gazebo ブリッジ:"))
        self._add_checkbox(
            components_layout, 
            'tf_bridge', 
            "TF Bridge（⚠️ EKF使用時は無効化推奨）", 
            False,  # デフォルトはFalse
            "GazeboからROS 2へ座標変換（TF: odom→base_footprint）をブリッジします。\n"
            "⚠️ EKFでセンサフュージョンを使用する場合は無効化してください（TF競合を防ぐため）。\n"
            "EKFがIMU融合後のTFを配信します。"
        )
        self._add_checkbox(
            components_layout, 
            'odom_bridge', 
            "Odometry Bridge", 
            True,
            "ロボットの位置・速度情報（オドメトリ）をGazeboからROS 2へブリッジします。\n"
            "EKFの入力データとして使用されます。"
        )
        self._add_checkbox(
            components_layout, 
            'twist_bridge', 
            "Twist Bridge (cmd_vel)", 
            True,
            "ROS 2からGazeboへ速度指令（cmd_vel）をブリッジします。ロボット制御に必要です"
        )
        self._add_checkbox(
            components_layout, 
            'joint_state_bridge', 
            "Joint State Bridge", 
            True,
            "ロボットの関節（車輪など）の状態をGazeboからROS 2へブリッジします"
        )
        self._add_checkbox(
            components_layout, 
            'clock_bridge', 
            "Clock Bridge", 
            True,
            "シミュレーション時刻をGazeboからROS 2へブリッジします。時刻同期に必須です"
        )
        
        # センサー関連
        components_layout.addWidget(self._create_section_label("\nセンサー:"))
        self._add_checkbox(
            components_layout, 
            'lidar_bridge', 
            "LiDAR Bridge (/scan3)", 
            True,
            "2D LiDARセンサーのデータ（/scan3）をGazeboからROS 2へブリッジします。"
        )
        self._add_checkbox(
            components_layout, 
            'lidar2_bridge', 
            "Hokuyo Bridge (/hokuyo_scan)", 
            True,
            "2台目の2D LiDARセンサーのデータ（/hokuyo_scan）をブリッジします"
        )
        self._add_checkbox(
            components_layout, 
            'imu_bridge', 
            "IMU Bridge", 
            True,
            "IMU（慣性計測装置）のデータをブリッジします。姿勢・加速度・角速度情報を取得。\n"
            "EKFと組み合わせてオドメトリと融合することで、より正確な自己位置推定が可能です。"
        )
        self._add_checkbox(
            components_layout, 
            'velodyne_bridge', 
            "Velodyne Bridge (/velodyne_points)", 
            True,
            "3D LiDAR（Velodyne）の点群データをブリッジします。"
        )
        self._add_checkbox(
            components_layout, 
            'realsense_bridge', 
            "RealSense Camera Bridge", 
            True,
            "RGBDカメラ（RealSense）のRGB画像、深度画像、点群、カメラ情報をブリッジします"
        )
        
        # ツール関連
        components_layout.addWidget(self._create_section_label("\nツール:"))
        self._add_checkbox(
            components_layout, 
            'teleop', 
            "Teleopキーボードコントロール", 
            True,
            "キーボードでロボットを手動操作できるターミナルを起動します"
        )
        self._add_checkbox(
            components_layout, 
            'rviz', 
            "RViz2", 
            True,
            "ROS 2の可視化ツールを起動します。センサーデータやロボットの状態を確認できます"
        )
        # Gazeboをヘッドレス（サーバのみ）で起動するオプション
        self._add_checkbox(
            components_layout,
            'headless',
            "Gazebo: Headless (サーバのみ起動)",
            False,
            "Gazeboをサーバ（GUIなし）で起動します。起動オプション: gz sim -s"
        )
        self._add_checkbox(
            components_layout,
            'run_on_start',
            "Gazebo: 起動時にシミュレーションを開始 (gz sim -r)",
            True,
            "gz sim -r オプションで、起動と同時にシミュレーションを実行します"
        )
        
        components_group.setLayout(components_layout)
        return components_group
    
    def _create_section_label(self, text):
        """セクションラベルの作成"""
        label = QLabel(text)
        label.setStyleSheet("font-weight: bold;")
        return label
    
    def _create_description_area(self):
        """説明表示エリアの作成"""
        description_group = QGroupBox("説明")
        description_layout = QVBoxLayout()
        
        self.description_text = QTextEdit()
        self.description_text.setReadOnly(True)
        self.description_text.setMaximumHeight(80)
        self.description_text.setStyleSheet("""
            QTextEdit {
                background-color: #2b2b2b;
                color: #e0e0e0;
                border: 1px solid #555;
                border-radius: 3px;
                padding: 5px;
                font-size: 12px;
            }
        """)
        self.description_text.setText("項目の上にマウスを置くと、ここに説明が表示されます。")
        
        description_layout.addWidget(self.description_text)
        description_group.setLayout(description_layout)
        
        return description_group
    
    def _add_checkbox(self, layout, key, text, checked, description=None):
        """チェックボックスを追加"""
        checkbox = QCheckBox(text)
        checkbox.setChecked(checked)
        if description:
            checkbox.description = description
            checkbox.installEventFilter(self)
        self.checkboxes[key] = checkbox
        layout.addWidget(checkbox)
    
    def eventFilter(self, obj, event):
        """イベントフィルター：ホバー時に説明を表示"""
        if event.type() == QEvent.Type.Enter:
            if hasattr(obj, 'description'):
                self.description_text.setText(obj.description)
        elif event.type() == QEvent.Type.Leave:
            self.description_text.setText("項目の上にマウスを置くと、ここに説明が表示されます。")
        return super().eventFilter(obj, event)
    
    def _create_launch_button(self):
        """起動ボタンレイアウトの作成"""
        button_layout = QHBoxLayout()
        
        self.launch_button = QPushButton("起動")
        self.launch_button.description = "選択した設定でシミュレーションを開始します"
        self.launch_button.installEventFilter(self)
        self.launch_button.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                font-size: 16px;
                font-weight: bold;
                padding: 10px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
        """)
        self.launch_button.clicked.connect(self.on_launch)
        button_layout.addWidget(self.launch_button)
        
        return button_layout
    
    def on_launch(self):
        """起動ボタンがクリックされた時の処理"""
        # 設定を保存
        self.config['world_file'] = self.world_combo.currentText()
        
        # スポーン座標を保存
        self.config['spawn_x'] = self.spawn_x_spin.value()
        self.config['spawn_y'] = self.spawn_y_spin.value()
        self.config['spawn_z'] = self.spawn_z_spin.value()
        self.config['spawn_yaw'] = self.spawn_yaw_spin.value()
        
        for key, checkbox in self.checkboxes.items():
            self.config[key] = checkbox.isChecked()
        
        self.launched = True
        self.close()
    
    def get_config(self):
        """設定を取得（起動された場合のみ）"""
        return self.config if self.launched else None


def show_launch_config_ui():
    """
    UIを表示して設定を取得する関数
    
    Returns:
        dict or None: ユーザーが選択した設定、またはキャンセルされた場合はNone
    """
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)
    
    ui = LaunchConfigUI()
    ui.show()
    app.exec()
    
    return ui.get_config()
