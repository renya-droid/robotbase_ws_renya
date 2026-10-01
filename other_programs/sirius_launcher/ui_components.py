"""Robot ROS 2 Launch Manager UI components."""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QGroupBox, QSpinBox
)
from PySide6.QtCore import Qt
from PySide6.QtGui import QFont

from robot_config import GZ_PARTITION, ROS_DOMAIN_ID, SIM_ROS_DOMAIN_ID


class LaunchButtonUI(QWidget):
    """起動ボタンウィジェットのUI"""
    
    def __init__(self, name, description=""):
        super().__init__()
        self.name = name
        self.description = description
        self.setup_ui()
    
    def setup_ui(self):
        """UIのセットアップ"""
        layout = QHBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)

        # 起動ボタン
        self.launch_btn = QPushButton(f"▶ {self.name}")
        self.launch_btn.setMinimumWidth(200)
        layout.addWidget(self.launch_btn)

        # ステータス表示
        self.status_label = QLabel("●")
        self.status_label.setStyleSheet("color: gray; font-size: 20px;")
        self.status_label.setFixedWidth(30)
        self.status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.status_label)

        # 説明ラベル
        self.desc_label = QLabel(self.description)
        self.desc_label.setStyleSheet("color: gray;")
        layout.addWidget(self.desc_label, 1)

        # 停止ボタン
        self.stop_btn = QPushButton("■ 停止")
        self.stop_btn.setMaximumWidth(100)
        self.stop_btn.setEnabled(False)
        layout.addWidget(self.stop_btn)
    
    def update_status(self, is_running, has_error=False):
        """ステータス表示を更新"""
        self.launch_btn.setEnabled(True)  # 起動中も有効にし、フォーカスボタンとして機能させる
        self.stop_btn.setEnabled(is_running)
        
        if is_running:
            self.launch_btn.setText(f"👁 {self.name}")
            self.launch_btn.setStyleSheet("background-color: #17a2b8; color: white; font-weight: bold;")
        else:
            self.launch_btn.setText(f"▶ {self.name}")
            self.launch_btn.setStyleSheet("")
            
        if has_error:
            self.status_label.setStyleSheet("color: #dc3545; font-size: 16px; font-weight: bold;") # 赤
            self.status_label.setText("!")
        elif is_running:
            self.status_label.setStyleSheet("color: #28a745; font-size: 16px;")  # 緑
            self.status_label.setText("●")
        else:
            self.status_label.setStyleSheet("color: gray; font-size: 16px;")
            self.status_label.setText("●")


class MainWindowUI:
    """メインウィンドウのUIセットアップ"""
    
    @staticmethod
    def setup_ui(window, display_name, tab_names=None):
        """UIのセットアップ（タブ切り替え対応）"""
        from PySide6.QtWidgets import QWidget, QVBoxLayout, QScrollArea, QFrame, QTabWidget

        launcher_title = f"{display_name} ROS 2 Launch Manager"
        window.setWindowTitle(launcher_title)
        window.setMinimumSize(800, 600)

        central_widget = QWidget()
        window.setCentralWidget(central_widget)

        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(10, 10, 10, 10)

        # ヘッダーレイアウト（タイトルとリロードボタン）
        header_layout = QHBoxLayout()
        header_layout.addStretch(1)

        # タイトル
        title = QLabel(launcher_title)
        title_font = QFont()
        title_font.setPointSize(16)
        title_font.setBold(True)
        title.setFont(title_font)
        title.setAlignment(Qt.AlignCenter)
        header_layout.addWidget(title)
        
        header_layout.addStretch(1)

        # リロードボタン
        reload_btn = QPushButton("🔄 リロード")
        reload_btn.setFixedWidth(100)
        reload_btn.setStyleSheet("background-color: #28a745; color: white; font-weight: bold; border-radius: 4px; padding: 5px;")
        header_layout.addWidget(reload_btn)

        # Domain内に残ったGazebo/Nav2/RVizをまとめて終了する非常停止ボタン。
        stop_simulation_btn = QPushButton("🛑 シミュレーション一式を終了")
        stop_simulation_btn.setMinimumWidth(220)
        stop_simulation_btn.setStyleSheet(
            "background-color: #dc3545; color: white; font-weight: bold; "
            "border-radius: 4px; padding: 5px;")
        stop_simulation_btn.setToolTip(
            "ココちゃんのシミュレーションDomainに属するGazebo、RViz、"
            "Nav2、SLAM等を終了します")
        header_layout.addWidget(stop_simulation_btn)
        window.stop_simulation_btn = stop_simulation_btn

        main_layout.addLayout(header_layout)

        # Simulation ROS Domain can be changed and persisted without editing files.
        domain_layout = QHBoxLayout()
        domain_layout.addStretch(1)
        domain_label = QLabel("シミュレーション ROS_DOMAIN_ID:")
        domain_layout.addWidget(domain_label)
        sim_domain_spin = QSpinBox()
        sim_domain_spin.setRange(0, 232)
        sim_domain_spin.setValue(int(SIM_ROS_DOMAIN_ID))
        sim_domain_spin.setFixedWidth(80)
        sim_domain_spin.setToolTip(
            "Gazebo、シミュレーションRViz、Nav2、SLAMで共通使用します。既定値は58です")
        domain_layout.addWidget(sim_domain_spin)
        save_sim_domain_btn = QPushButton("💾 Domainを保存")
        save_sim_domain_btn.setStyleSheet(
            "background-color: #6f42c1; color: white; font-weight: bold; "
            "border-radius: 4px; padding: 5px;")
        domain_layout.addWidget(save_sim_domain_btn)
        default_domain_label = QLabel("既定値: 58")
        default_domain_label.setStyleSheet("color: gray;")
        domain_layout.addWidget(default_domain_label)
        domain_layout.addStretch(1)
        main_layout.addLayout(domain_layout)
        window.sim_domain_spin = sim_domain_spin
        window.save_sim_domain_btn = save_sim_domain_btn

        # 情報ラベル
        info_label = QLabel(
            "ボタンを押すとTerminatorのタブで起動します | "
            f"実機Domain={ROS_DOMAIN_ID} | シミュレーションDomain={SIM_ROS_DOMAIN_ID} | "
            f"GZ_PARTITION={GZ_PARTITION} | 緑●=起動中")
        info_label.setStyleSheet("color: gray; font-style: italic;")
        info_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(info_label)
        window.runtime_info_label = info_label

        # プリセットセクション
        preset_group = QGroupBox("プリセット")
        preset_group.setStyleSheet("QGroupBox { font-weight: bold; }")
        preset_layout = QHBoxLayout()
        preset_group.setLayout(preset_layout)
        main_layout.addWidget(preset_group)

        # タブウィジェット追加
        tab_widget = QTabWidget()
        tab_layouts = {}
        if tab_names is None:
            tab_names = [
                "センサー・ハードウェア",
                "シミュレーション",
                "ユーティリティ",
                "ナビゲーション",
                "リアル実験",
            ]
        for tab_name in tab_names:
            scroll = QScrollArea()
            scroll.setWidgetResizable(True)
            scroll.setFrameShape(QFrame.NoFrame)
            
            tab_content = QWidget()
            tab_layout = QVBoxLayout(tab_content)
            tab_layout.setContentsMargins(10, 10, 10, 10)
            tab_layout.setSpacing(5)
            
            scroll.setWidget(tab_content)
            tab_widget.addTab(scroll, tab_name)
            tab_layouts[tab_name] = tab_layout

        main_layout.addWidget(tab_widget)

        return preset_layout, tab_layouts, tab_widget, reload_btn
    
    @staticmethod
    def create_preset_button(preset_name):
        """プリセットボタンを作成"""
        preset_btn = QPushButton(f"▶ {preset_name}")
        preset_btn.setMinimumHeight(40)
        preset_btn.setStyleSheet("background-color: #007bff; color: white; font-weight: bold;")
        return preset_btn
    
    @staticmethod
    def create_group(title):
        """グループボックスを作成"""
        group = QGroupBox(title)
        group_layout = QVBoxLayout()
        group.setLayout(group_layout)
        return group, group_layout
