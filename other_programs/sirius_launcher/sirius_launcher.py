#!/usr/bin/env python3
"""Configurable robot ROS 2 launch manager."""

import os
import signal
import subprocess
import sys
import psutil
from PySide6.QtWidgets import QApplication, QMainWindow, QMessageBox
from PySide6.QtCore import QTimer, QEvent

from alias_parser import parse_bash_aliases
from ui_components import LaunchButtonUI, MainWindowUI
from process_manager import ProcessManager
from robot_config import (
    ALIAS_FILE,
    DISPLAY_NAME,
    GZ_PARTITION,
    ROBOT_ID,
    ROS_DOMAIN_ID,
    SIM_ROS_DOMAIN_ID,
    WORKSPACE,
    save_sim_ros_domain_id,
)


TAB_NAMES = [
    "センサー・ハードウェア",
    "シミュレーション",
    "ユーティリティ",
    "ナビゲーション",
    "リアル実験",
]


class LaunchButton(LaunchButtonUI):
    """起動ボタンを含むウィジェット（ロジック統合版）"""
    
    def __init__(self, name, command, description="", tab_widget=None, tab_index=None):
        super().__init__(name, description)
        # Tab control info
        self.tab_widget = tab_widget
        self.tab_index = tab_index
        self.command = command
        self.process_manager = ProcessManager(name, command)
        
        # ボタンのイベント接続
        self.launch_btn.clicked.connect(self.launch)
        self.stop_btn.clicked.connect(self.stop)
        # クリックでタブ選択（実行中のみ）
        self.launch_btn.clicked.connect(self.select_tab_if_running)
        self.stop_btn.clicked.connect(self.select_tab_if_running)
        
        # 起動時に古いPIDファイルをチェック
        self.load_pid()
        
        # 定期的にプロセス状態をチェック
        self.timer = QTimer()
        self.timer.timeout.connect(self.check_process_status)
        self.timer.start(1000)
        
        self.has_error = False
        # イベントフィルタをインストールして、ウィジェット全体のクリックを捕まえる
        self.installEventFilter(self)
        # 子ウィジェットにもフィルタをインストール
        try:
            self.launch_btn.installEventFilter(self)
            self.stop_btn.installEventFilter(self)
            self.status_label.installEventFilter(self)
            # desc_labelはui_componentsでself.desc_labelとして公開済み
            if hasattr(self, 'desc_label'):
                self.desc_label.installEventFilter(self)
        except Exception:
            pass
    
    def launch(self):
        """コマンドを新しいターミナルタブで起動。既に起動している場合はそのタブを表示"""
        if self.process_manager.is_running():
            print(f"既に起動しています。タブをフォーカスします: {self.name}")
            import shutil
            tools_installed = shutil.which('wmctrl') is not None and shutil.which('xdotool') is not None
            
            if not self.process_manager.focus_terminator_tab():
                if not tools_installed:
                    QMessageBox.information(
                        self, 
                        "情報", 
                        f"「{self.name}」は既に起動しています。\n\n"
                        "※開いているターミナルタブに自動で切り替えるには、wmctrl と xdotool のインストールが必要です。\n"
                        "以下のコマンドを実行してください：\n"
                        "sudo apt install wmctrl xdotool -y"
                    )
                else:
                    QMessageBox.warning(
                        self,
                        "警告",
                        f"「{self.name}」のターミナルタブをフォーカスできませんでした。\n"
                        "タブが手動で閉じられたか、名前が変更された可能性があります。"
                    )
            return
            
        if self.process_manager.launch():
            QTimer.singleShot(1500, self.load_pid)
            print(f"起動: {self.name}")
        else:
            QMessageBox.critical(self, "エラー", f"起動に失敗しました: {self.name}")
    
    def load_pid(self, retry_count=0):
        """PIDファイルからプロセスIDを読み込む"""
        result = self.process_manager.load_pid(retry_count)
        if result == "retry":
            QTimer.singleShot(500, lambda: self.load_pid(retry_count + 1))
        elif result:
            self.update_status(True)
    
    def stop(self):
        """プロセスを停止"""
        result = self.process_manager.stop()
        if result:
            pids = result['pids']
            terminator_pid = result['terminator_pid']
            
            # 2秒待っても終了しない場合は強制終了
            QTimer.singleShot(2000, lambda: self.process_manager.force_kill_tree(pids, terminator_pid))
            
            # Terminatorのタブを閉じる
            QTimer.singleShot(2500, lambda: self.process_manager.close_terminator_tab(terminator_pid))
            
            self.update_status(False)
        else:
            QMessageBox.critical(self, "エラー", "停止に失敗しました")
    
    def check_process_status(self):
        """プロセスの状態を定期的にチェック"""
        is_running = self.process_manager.is_running()
        self.has_error = self.process_manager.check_for_errors() if is_running else False
        
        self.update_status(is_running, self.has_error)
        
        # メインウィンドウに通知してタブの状態を更新させる
        main_win = self.window()
        if hasattr(main_win, 'update_tab_error_status'):
            main_win.update_tab_error_status()

    def select_tab(self):
        """この項目があるタブを選択する"""
        if self.tab_widget is not None and self.tab_index is not None and self.tab_index >= 0:
            try:
                self.tab_widget.setCurrentIndex(self.tab_index)
            except Exception:
                pass

    def select_tab_if_running(self):
        """実行中のときだけタブを選択するユーティリティ"""
        if self.process_manager.is_running():
            self.select_tab()

    def eventFilter(self, watched, event):
        # マウスクリックイベントを捕まえてタブ選択を行う
        if event.type() == QEvent.MouseButtonPress:
            # クリックされたとき、実行中ならタブ選択
            if self.process_manager.is_running():
                self.select_tab()
        return super().eventFilter(watched, event)


class RobotLauncher(QMainWindow):
    """Robot ROS 2 Launch Manager main window."""
    
    def __init__(self):
        super().__init__()
        self.buttons = []
        self.button_map = {}
        self.presets = []
        self.sim_ros_domain_id = SIM_ROS_DOMAIN_ID
        self.original_tab_names = {} # index -> name
        self.setup_ui()
        self.load_aliases()
    
    def setup_ui(self):
        """UIのセットアップ"""
        self.preset_layout, self.tab_layouts, self.tab_widget, self.reload_btn = MainWindowUI.setup_ui(
            self, DISPLAY_NAME, TAB_NAMES)
        self.reload_btn.clicked.connect(self.reload_launcher)
        self.stop_simulation_btn.clicked.connect(self.stop_all_simulation)
        self.save_sim_domain_btn.clicked.connect(self.save_simulation_domain)

    def simulation_processes_running(self):
        """Return True if the currently saved simulation graph still exists."""
        partition_prefix = f'{GZ_PARTITION}_sim_'
        for process in psutil.process_iter(['pid']):
            if process.pid == os.getpid():
                continue
            try:
                environment = process.environ()
            except (psutil.AccessDenied, psutil.NoSuchProcess):
                continue
            if environment.get('ROS_DOMAIN_ID') == self.sim_ros_domain_id:
                return True
            if environment.get('GZ_PARTITION', '').startswith(partition_prefix):
                return True
        return False

    def save_simulation_domain(self):
        """Validate and persist the selected simulation ROS Domain ID."""
        selected = str(self.sim_domain_spin.value())
        if selected == ROS_DOMAIN_ID:
            QMessageBox.warning(
                self,
                "保存できません",
                f"実機Domainも{ROS_DOMAIN_ID}です。実機とシミュレーションには"
                "異なる値を指定してください。",
            )
            self.sim_domain_spin.setValue(int(self.sim_ros_domain_id))
            return
        if selected == self.sim_ros_domain_id:
            QMessageBox.information(
                self, "保存済み", f"シミュレーションDomainは既に{selected}です。")
            return
        if self.simulation_processes_running():
            QMessageBox.warning(
                self,
                "シミュレーションを先に終了してください",
                f"現在のDomain {self.sim_ros_domain_id}にシミュレーション系プロセスが"
                "残っています。\n\n赤い「シミュレーション一式を終了」を押してから"
                "Domainを保存してください。",
            )
            self.sim_domain_spin.setValue(int(self.sim_ros_domain_id))
            return

        try:
            saved = save_sim_ros_domain_id(selected)
        except (OSError, ValueError) as error:
            QMessageBox.critical(
                self, "保存失敗", f"robot.envへDomainを保存できませんでした。\n{error}")
            self.sim_domain_spin.setValue(int(self.sim_ros_domain_id))
            return

        self.sim_ros_domain_id = saved
        os.environ['ROBOTBASE_SIM_ROS_DOMAIN_ID'] = saved
        self.runtime_info_label.setText(
            "ボタンを押すとTerminatorのタブで起動します | "
            f"実機Domain={ROS_DOMAIN_ID} | シミュレーションDomain={saved} | "
            f"GZ_PARTITION={GZ_PARTITION} | 緑●=起動中")
        QMessageBox.information(
            self,
            "保存完了",
            f"シミュレーションROS_DOMAIN_IDを{saved}へ保存しました。\n"
            "次に起動するGazebo、RViz、Nav2、SLAMから反映されます。",
        )

    def stop_all_simulation(self):
        """Stop the isolated simulation graph, including orphaned processes."""
        answer = QMessageBox.question(
            self,
            "シミュレーション一式を終了",
            f"ROS_DOMAIN_ID={self.sim_ros_domain_id}のGazebo、RViz、Nav2、SLAM等を"
            "すべて終了します。\n\n実行しますか？",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer != QMessageBox.Yes:
            return

        script = WORKSPACE / 'bash' / 'startup_bash' / 'stop_simulation.sh'
        cleanup_environment = os.environ.copy()
        cleanup_environment['ROBOTBASE_STOP_SIM_DOMAIN'] = self.sim_ros_domain_id
        try:
            result = subprocess.run(
                ['bash', str(script)],
                capture_output=True,
                text=True,
                timeout=15,
                check=False,
                env=cleanup_environment,
            )
        except (OSError, subprocess.TimeoutExpired) as error:
            QMessageBox.critical(
                self, "終了失敗", f"シミュレーションを終了できませんでした。\n{error}")
            return

        output = '\n'.join(
            part.strip() for part in (result.stdout, result.stderr) if part.strip())
        if result.returncode == 0:
            for button in self.buttons:
                if button.name == f'{ROBOT_ID}_sim' or '_sim' in button.name:
                    button.process_manager.pid_file_content = None
                    button.update_status(False)
            QMessageBox.information(
                self,
                "終了完了",
                output or "シミュレーション一式を終了しました。",
            )
        else:
            QMessageBox.warning(
                self,
                "一部終了失敗",
                output or f"終了処理がコード{result.returncode}で失敗しました。",
            )

    def add_group(self, title, tab_name=None):
        """グループボックスを追加（タブ対応）"""
        group, group_layout = MainWindowUI.create_group(title)
        # タブ名指定がなければ最初のタブに追加
        if tab_name is None:
            tab_name = list(self.tab_layouts.keys())[0]
        self.tab_layouts[tab_name].addWidget(group)
        return group_layout, group
    
    # NOTE: previous single-tab add_group kept for reference is removed
    
    def load_aliases(self):
        """エイリアスファイルを読み込んでボタンを作成"""
        # Never fall back to the Sirius workspace: this launcher belongs to
        # robotbase_ws and must remain safe when both workspaces coexist.
        alias_file = ALIAS_FILE
        if not alias_file.exists():
            QMessageBox.warning(
                self, "警告", f"エイリアスファイルが見つかりません。\n{alias_file}")
            return

        groups, presets = parse_bash_aliases(str(alias_file))
        self.presets = presets

        if not groups:
            QMessageBox.warning(self, "警告", "エイリアスが見つかりませんでした。")
            return

        # プリセットボタンを作成
        for preset_name, items in presets:
            self.add_preset_button(preset_name, items)

        # タブ名リスト（ui_components.pyのデフォルトと合わせる）
        tab_names_list = TAB_NAMES
        for i, name in enumerate(tab_names_list):
            self.original_tab_names[i] = name

        # 通常のボタンを作成（タブごとにグループ追加）
        for i, (group_name, aliases) in enumerate(groups.items()):
            if aliases:
                tab_name = tab_names_list[i] if i < len(tab_names_list) else tab_names_list[0]
                group_layout, group_widget = self.add_group(group_name, tab_name)
                for alias_name, command, description in aliases:
                    self.add_button(group_layout, alias_name, command, description, group_widget)

        # 各タブのレイアウトにストレッチ追加
        for layout in self.tab_layouts.values():
            layout.addStretch()
            
    def clear_layout(self, layout):
        """レイアウト内のウィジェットを再帰的に削除"""
        while layout.count():
            child = layout.takeAt(0)
            if child.widget():
                child.widget().deleteLater()
            elif child.layout():
                self.clear_layout(child.layout())
                
    def reload_launcher(self):
        """エイリアスファイルを再パースしてGUIを再構築（実行中のプロセスはそのまま維持）"""
        print("🔄 エイリアス設定を再読み込み中...")
        
        # 1. 内部のボタンマップ等をクリア
        self.buttons = []
        self.button_map = {}
        self.presets = []
        
        # 2. プリセットボタンをUIから削除
        self.clear_layout(self.preset_layout)
        
        # 3. 各タブのグループボックスなどのUI要素を削除
        for layout in self.tab_layouts.values():
            self.clear_layout(layout)
            
        # 4. 再ロードと再配置
        self.load_aliases()
        print("✅ 再読み込み完了。")
    
    def add_preset_button(self, preset_name, items):
        """プリセットボタンを追加"""
        preset_btn = MainWindowUI.create_preset_button(preset_name)
        preset_btn.clicked.connect(lambda: self.launch_preset(preset_name, items))
        self.preset_layout.addWidget(preset_btn)
    
    def launch_preset(self, preset_name, items):
        """プリセットの複数コマンドを同時起動"""
        print(f"プリセット起動: {preset_name}")
        import time
        for item in items:
            if item in self.button_map:
                button = self.button_map[item]
                # 起動中でない場合のみ起動
                if not button.process_manager.is_running():
                    button.launch()
                    time.sleep(0.5)
                else:
                    print(f"  スキップ: {item} (既に起動中)")
            else:
                print(f"  エラー: {item} が見つかりません")
    
    def add_button(self, layout, name, command, description, group_widget=None):
        """ボタンを追加"""
        tab_index = None
        if group_widget is not None and self.tab_widget is not None:
            # 親を辿ってTabのインデックスを探す（ScrollArea導入に対応）
            parent = group_widget.parentWidget()
            while parent:
                tab_index = self.tab_widget.indexOf(parent)
                if tab_index >= 0:
                    break
                parent = parent.parentWidget()
        button = LaunchButton(name, command, description, tab_widget=self.tab_widget, tab_index=tab_index)
        layout.addWidget(button)
        self.buttons.append(button)
        self.button_map[name] = button

    def update_tab_error_status(self):
        """全てのタブのエラー状況をスキャンして表示を更新"""
        tab_errors = {} # index -> bool
        
        # 各タブにエラーがあるかチェック
        for button in self.buttons:
            if button.tab_index is not None:
                if button.tab_index not in tab_errors:
                    tab_errors[button.tab_index] = False
                if button.has_error:
                    tab_errors[button.tab_index] = True
        
        # タブの表示（テキスト）を更新
        for idx, original_name in self.original_tab_names.items():
            if idx < self.tab_widget.count():
                has_error = tab_errors.get(idx, False)
                current_text = self.tab_widget.tabText(idx)
                
                if has_error:
                    new_text = "⚠️ " + original_name
                    if current_text != new_text:
                        self.tab_widget.setTabText(idx, new_text)
                else:
                    if current_text != original_name:
                        self.tab_widget.setTabText(idx, original_name)


def main():
    # Apply robot-specific isolation only to the launcher and its children.
    # The parent Sirius shell keeps its original environment.
    os.environ['ROS_DOMAIN_ID'] = ROS_DOMAIN_ID
    os.environ['GZ_PARTITION'] = GZ_PARTITION
    os.environ['ROBOTBASE_DISPLAY_NAME'] = DISPLAY_NAME
    os.environ['ROBOTBASE_ID'] = ROBOT_ID

    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    
    window = RobotLauncher()
    window.show()
    
    # Ctrl+Cでの終了を処理
    import signal
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
