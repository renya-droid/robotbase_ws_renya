import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import weakref
from rclpy.action import ActionClient
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Pose2D, PoseWithCovarianceStamped, PoseStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from nav2_msgs.srv import ClearEntireCostmap
from std_msgs.msg import Bool, String
from action_msgs.msg import GoalStatus

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy
)

import math
import time
import signal
import json
import yaml
import cv2
import os
import numpy as np

from .arrival_map import ArrivalMapHandoff
from .exit_map_switch import ExitMapSwitch
from .input_health import InputHealthMixin
from .motion_watchdog import MotionWatchdog, motion_locked
from .observation_ros import ObservationGuard
from .lift_cycle_ros import LiftCycleLink
from .executor_cleanup import shutdown_executor
from .voice_announcements import VoiceAnnouncements, door_progress_event
from .waypoint_cycle import WaypointCycleLink
from .elevator_bank_ros import ElevatorBankLink, bank_geometry_markers
from .research_ros import ResearchLink
from .comparison import evaluate_cell, comparison_summary
from .shadow_geometry import directed_shadow, prepare_shadows
from .boarding_admission import BoardingAdmission, AREA_DEFAULTS, area_gate_reasons
from .observation import ScanEvidence
from .controller_weights import ControllerWeightProfile


class ElevatorEntryMapDiff(ControllerWeightProfile, BoardingAdmission, ArrivalMapHandoff, ElevatorBankLink, ResearchLink, WaypointCycleLink, LiftCycleLink, ObservationGuard, MotionWatchdog, InputHealthMixin, ExitMapSwitch, Node):

    def __init__(self):
        super().__init__('elevator_entry_map_diff')
        # Clock updates must not wait behind candidate geometry callbacks.
        # Otherwise fresh sensor stamps look falsely future-dated to safety callbacks.
        self.sim_clock_group = MutuallyExclusiveCallbackGroup()
        for subscription in self.subscriptions:
            if subscription.topic_name == '/clock':
                subscription.callback_group.entities.discard(weakref.ref(subscription))
                subscription.callback_group = self.sim_clock_group
                self.sim_clock_group.add_entity(subscription)
        self.setup_input_health()
        self.setup_motion_watchdog()
        self.setup_observation_guard()
        self.declare_parameter('voice_enabled', False)
        self.declare_parameter('voice_device', 'default')
        self.declare_parameter('voice_directory', '')
        voice_directory = self.get_parameter('voice_directory').value
        voice_enabled = self.get_parameter('voice_enabled').value
        if voice_enabled and not voice_directory:
            from ament_index_python.packages import get_package_share_directory
            voice_directory = os.path.join(
                get_package_share_directory('elevator_space_navigation'), 'audio')
        self.voice = VoiceAnnouncements(
            voice_directory, self.get_logger(), enabled=voice_enabled,
            device=self.get_parameter('voice_device').value)

        # ==========================
        # パラメータ設定
        # ==========================
        self.declare_parameter('map_yaml_path', '/home/koko-chan-nuc/robotbase_ws/maps_waypoints/maps/1013ele.yaml.yaml')
        self.declare_parameter('scan_topic', '/scan')
        # Optional second scan for entry candidate obstacles. Its measured
        # base_footprint extrinsic is used when the observation guard is off.
        self.declare_parameter('hokuyo_scan_topic', '')
        self.declare_parameter('hokuyo_origin_x', 0.196)
        self.declare_parameter('hokuyo_origin_y', 0.0)
        self.declare_parameter('hokuyo_origin_yaw', 0.0)
        self.declare_parameter('hokuyo_scan_max_age_sec', 0.5)
        self.declare_parameter('pose_topic', '/amcl_pose')
        # 通常コマンドは /elevator_entry_start の true を待つ。
        # 即時開始したい場合は elevator_entry_map_diff_auto を使用する。
        self.declare_parameter('start_immediately', False)
        self.declare_parameter('auto_start_on_closed', False)
        self.declare_parameter('main_loop_period_sec', 1.0)
        self.declare_parameter('entry_start_topic', '/elevator_entry_start')
        # Gazebo版は走行中も連続更新される/odomを使える。
        # DiffDriveの/odom原点がmap上のspawn位置と異なる場合は、
        # 下記の剛体変換でmap座標へ変換する。
        self.declare_parameter('use_odom_pose', True)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_to_map_x', 0.0)
        self.declare_parameter('odom_to_map_y', 0.0)
        self.declare_parameter('odom_to_map_yaw', 0.0)

        # エレベータ内として調べる範囲
        self.declare_parameter('elevator_x_min', 43.98731231689453)
        self.declare_parameter('elevator_x_max', 45.940914154052734)
        self.declare_parameter('elevator_y_min', 7.162367820739746)
        self.declare_parameter('elevator_y_max', 8.86912727355957)

        # 調べる間隔
        self.declare_parameter('grid_step', 0.10)

        # 障害物からこれ以上離れていたら安全とする距離
        self.declare_parameter('safe_distance', 0.3)

        # ゴール姿勢での非対称長方形footprint。
        # base_footprint（駆動輪中心付近）から各車体端までの距離と、
        # 車体面を検査するサンプリング間隔を指定する。
        self.declare_parameter('robot_front_length', 0.30)
        self.declare_parameter('robot_rear_length', 0.30)
        self.declare_parameter('robot_left_width', 0.34)
        self.declare_parameter('robot_right_width', 0.34)
        self.declare_parameter('footprint_sample_step', 0.08)

        # エレベータ内部のドア前を禁止領域にするか
        self.declare_parameter('use_door_front_forbidden', True)

        # ドア前は通過可能だが、最終ゴールには選ばない。
        self.declare_parameter('door_front_x_min', -0.60)
        self.declare_parameter('door_front_x_max', 0.45)
        self.declare_parameter('door_front_y_min', 2.40)
        self.declare_parameter('door_front_y_max', 3.00)

        # LiDARで障害物に遮られて見えない奥側を危険領域にするか
        self.declare_parameter('use_unknown_shadow', True)

        # 障害物の何m奥から未観測領域として扱うか
        self.declare_parameter('unknown_start_margin', 0.05)

        # ゴール到達のタイムアウト秒数
        self.declare_parameter('goal_timeout_sec', 60.0)

        # Nav2のinflation radius（障害物膨張半径）を小さくするための値
        self.declare_parameter('inflation_radius', 0.40)
        self.declare_parameter('dynamic_costmap_profile', True)
        self.declare_parameter('entry_cost_scaling_factor', 8.0)
        # Zero preserves the existing common global/local entry profile.
        self.declare_parameter('entry_global_inflation_radius', 0.0)
        self.declare_parameter('entry_global_cost_scaling_factor', 0.0)
        self.declare_parameter('normal_inflation_radius', 0.70)
        self.declare_parameter('normal_global_cost_scaling_factor', 3.0)
        self.declare_parameter('normal_local_cost_scaling_factor', 5.0)

        # 障害物の奥側を何mまで未観測領域として扱うか
        self.declare_parameter('unknown_shadow_depth', 0.60)

        # 未観測領域の横幅。大きくすると人の影を広めに危険扱いする
        self.declare_parameter('unknown_shadow_width', 0.12)

        # 地図上で壁と判定するしきい値
        self.declare_parameter('map_wall_threshold', 100)

        # 探索範囲はかごの床全体まで取る一方、Burgerの中心が静的な壁へ
        # 近づきすぎるセルは停止候補から外す。
        self.declare_parameter('static_wall_clearance', 0.20)

        # 入室後にドア方向へ戻れる経路を残すか
        self.declare_parameter('use_exit_path_check', True)
        self.declare_parameter('exit_path_clearance', 0.25)
        self.declare_parameter('exit_path_sample_step', 0.05)

        # ドア左右端からエレベータ座標系を作るか
        self.declare_parameter('use_door_based_elevator', True)
        self.declare_parameter('door_left_x', 44.298343658447266)
        self.declare_parameter('door_left_y', 7.2010178565979)

        self.declare_parameter('door_right_x', 45.20166778564453)
        self.declare_parameter('door_right_y', 7.103000164031982)
        self.declare_parameter('elevator_inside_sign', 1.0)

        # ドア基準ローカル座標での探索範囲
        # local_x: ドア左右方向, local_y: エレベータ奥方向
        self.declare_parameter('elevator_local_x_min', -1.05)
        self.declare_parameter('elevator_local_x_max', 1.05)
        self.declare_parameter('elevator_local_y_min', 0.00)
        # かご内寸（幅約2.15 m、奥行約1.60 m）に合わせて床全体を
        # 調べる。壁際の停止候補はstatic_wall_clearanceで除外する。
        self.declare_parameter('elevator_local_y_max', 1.55)

        # map座標で計測したかご床の四隅。斜め・台形のかご形状でも
        # 探索候補と車体footprintが実測ポリゴン外へ出ないようにする。
        self.declare_parameter('use_elevator_polygon', True)
        self.declare_parameter('elevator_polygon_x', [
            43.98731231689453, 44.15085983276367,
            45.940914154052734, 45.79412078857422,
        ])
        self.declare_parameter('elevator_polygon_y', [
            7.327237129211426, 8.86912727355957,
            8.470598220825195, 7.162367820739746,
        ])

        # ドア基準座標でのドア前禁止領域。
        # セル境界は判定時に半セル分だけ拡張して隙間を防ぐ。
        self.declare_parameter('door_front_local_x_min', -0.55)
        self.declare_parameter('door_front_local_x_max', 0.55)
        self.declare_parameter('door_front_local_y_min', 0.00)
        self.declare_parameter('door_front_local_y_max', 0.60)

        # 降車者がドア付近にいる場合は入室せず待機する
        self.declare_parameter('use_exiting_person_wait', True)
        self.declare_parameter('exiting_person_local_x_min', -0.70)
        self.declare_parameter('exiting_person_local_x_max', 0.70)
        self.declare_parameter('exiting_person_local_y_min', -0.80)
        self.declare_parameter('exiting_person_local_y_max', 0.80)
        self.declare_parameter('exiting_person_min_points', 6)
        self.declare_parameter('exiting_person_clear_time_sec', 2.0)
        self.declare_parameter('exiting_person_wait_timeout_sec', 20.0)
        self.declare_parameter('exiting_person_motion_threshold', 0.10)
        self.declare_parameter('exiting_person_stationary_time_sec', 3.0)

        # ドア開口を横切るLiDAR光線が、ドア面で止まるか、
        # かご奥まで抜けるかの判定結果は独立DoorNodeから受信する。
        self.declare_parameter('use_door_open_check', True)
        self.declare_parameter('door_state_topic', '/elevator_door_state')
        # DoorNodeのheartbeatが途切れた場合はUNKNOWNとして安全側に倒す。
        self.declare_parameter('door_state_stale_sec', 1.0)
        self.declare_parameter('door_initial_open_settle_sec', 3.0)
        # 0以下なら、ドア状態が変わるまで時間制限なしで待機する。
        self.declare_parameter('door_wait_timeout_sec', 0.0)
        self.declare_parameter('use_full_door_cycle', True)
        self.declare_parameter('door_close_confirm_sec', 1.0)
        self.declare_parameter('door_close_wait_timeout_sec', 0.0)
        self.declare_parameter('door_reopen_wait_timeout_sec', 0.0)
        # 開扉直後はNav2のコストマップに閉じた扉が残ることがあるため、
        # 更新を待ってから退出経路を問い合わせる。
        self.declare_parameter('door_reopen_settle_sec', 3.0)
        self.declare_parameter('hold_door_open_during_exit', False)
        self.declare_parameter('door_command_topic', '/door_cmd')
        # Gazeboの階移動中は2D AMCLが高さ方向のスキャンを誤って取り込み、
        # 到着後のmap->odomがずれることがある。退室前にワールド座標と
        # 一致する/odomからAMCLを再初期化し、残った障害物も消去する。
        self.declare_parameter('reset_amcl_from_odom_before_exit', False)
        self.declare_parameter('exit_localization_settle_sec', 1.0)
        self.declare_parameter('clear_costmaps_before_exit', False)
        self.declare_parameter('exit_costmap_settle_sec', 1.0)
        # NavigateToPose自身の再計画・復旧を使うため、退出前の単発経路確認は
        # デフォルトで行わない。必要な環境では有効化できる。
        self.declare_parameter('use_exit_path_precheck', False)
        self.declare_parameter('exit_path_retry_interval_sec', 2.0)
        self.declare_parameter('exit_path_max_retries', 5)
        self.declare_parameter('entry_path_retry_interval_sec', 2.0)
        self.declare_parameter('entry_path_max_retries', 5)

        # GUIからの手動操作
        # twist_muxの最優先停止入力
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_stop')
        self.declare_parameter('manual_exit_local_x', 0.0)
        self.declare_parameter('manual_exit_local_y', -0.80)
        self.declare_parameter('exit_keep_inward_heading', False)

        self.map_yaml_path = self.get_parameter('map_yaml_path').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.hokuyo_scan_topic = self.get_parameter('hokuyo_scan_topic').value
        self.hokuyo_origin_x = self.get_parameter('hokuyo_origin_x').value
        self.hokuyo_origin_y = self.get_parameter('hokuyo_origin_y').value
        self.hokuyo_origin_yaw = self.get_parameter('hokuyo_origin_yaw').value
        self.hokuyo_scan_max_age_sec = self.get_parameter('hokuyo_scan_max_age_sec').value
        if (not math.isfinite(self.hokuyo_scan_max_age_sec)
                or self.hokuyo_scan_max_age_sec <= 0):
            raise ValueError('hokuyo_scan_max_age_sec must be positive')
        self.pose_topic = self.get_parameter('pose_topic').value
        self.start_immediately = self.get_parameter('start_immediately').value
        self.auto_start_on_closed = self.get_parameter('auto_start_on_closed').value
        if type(self.auto_start_on_closed) is not bool:
            raise ValueError('auto_start_on_closed must be boolean')
        self.auto_start_consumed = False
        self.entry_start_topic = self.get_parameter('entry_start_topic').value
        self.use_odom_pose = self.get_parameter('use_odom_pose').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_to_map_x = self.get_parameter('odom_to_map_x').value
        self.odom_to_map_y = self.get_parameter('odom_to_map_y').value
        self.odom_to_map_yaw = self.get_parameter('odom_to_map_yaw').value

        self.elevator_x_min = self.get_parameter('elevator_x_min').value
        self.elevator_x_max = self.get_parameter('elevator_x_max').value
        self.elevator_y_min = self.get_parameter('elevator_y_min').value
        self.elevator_y_max = self.get_parameter('elevator_y_max').value

        self.grid_step = self.get_parameter('grid_step').value
        self.safe_distance = self.get_parameter('safe_distance').value
        self.robot_front_length = self.get_parameter(
            'robot_front_length').value
        self.robot_rear_length = self.get_parameter(
            'robot_rear_length').value
        self.robot_left_width = self.get_parameter(
            'robot_left_width').value
        self.robot_right_width = self.get_parameter(
            'robot_right_width').value
        self.footprint_sample_step = self.get_parameter(
            'footprint_sample_step').value
        self.use_door_front_forbidden = self.get_parameter('use_door_front_forbidden').value
        self.door_front_x_min = self.get_parameter('door_front_x_min').value
        self.door_front_x_max = self.get_parameter('door_front_x_max').value
        self.door_front_y_min = self.get_parameter('door_front_y_min').value
        self.door_front_y_max = self.get_parameter('door_front_y_max').value
        self.use_unknown_shadow = self.get_parameter('use_unknown_shadow').value
        self.unknown_start_margin = self.get_parameter('unknown_start_margin').value
        self.unknown_shadow_depth = self.get_parameter('unknown_shadow_depth').value
        self.unknown_shadow_width = self.get_parameter('unknown_shadow_width').value
        self.map_wall_threshold = self.get_parameter('map_wall_threshold').value
        self.static_wall_clearance = self.get_parameter(
            'static_wall_clearance'
        ).value
        self.use_exit_path_check = self.get_parameter('use_exit_path_check').value
        self.exit_path_clearance = self.get_parameter('exit_path_clearance').value
        self.exit_path_sample_step = self.get_parameter('exit_path_sample_step').value
        self.use_door_based_elevator = self.get_parameter('use_door_based_elevator').value
        self.door_left_x = self.get_parameter('door_left_x').value
        self.door_left_y = self.get_parameter('door_left_y').value
        self.door_right_x = self.get_parameter('door_right_x').value
        self.door_right_y = self.get_parameter('door_right_y').value
        self.elevator_inside_sign = self.get_parameter('elevator_inside_sign').value
        self.elevator_local_x_min = self.get_parameter('elevator_local_x_min').value
        self.elevator_local_x_max = self.get_parameter('elevator_local_x_max').value
        self.elevator_local_y_min = self.get_parameter('elevator_local_y_min').value
        self.elevator_local_y_max = self.get_parameter('elevator_local_y_max').value
        self.use_elevator_polygon = self.get_parameter(
            'use_elevator_polygon').value
        polygon_x = list(self.get_parameter('elevator_polygon_x').value)
        polygon_y = list(self.get_parameter('elevator_polygon_y').value)
        self.elevator_polygon = list(zip(polygon_x, polygon_y))
        self.door_front_local_x_min = self.get_parameter('door_front_local_x_min').value
        self.door_front_local_x_max = self.get_parameter('door_front_local_x_max').value
        self.door_front_local_y_min = self.get_parameter('door_front_local_y_min').value
        self.door_front_local_y_max = self.get_parameter('door_front_local_y_max').value
        self.use_exiting_person_wait = self.get_parameter('use_exiting_person_wait').value
        self.exiting_person_local_x_min = self.get_parameter('exiting_person_local_x_min').value
        self.exiting_person_local_x_max = self.get_parameter('exiting_person_local_x_max').value
        self.exiting_person_local_y_min = self.get_parameter('exiting_person_local_y_min').value
        self.exiting_person_local_y_max = self.get_parameter('exiting_person_local_y_max').value
        self.exiting_person_min_points = self.get_parameter('exiting_person_min_points').value
        self.exiting_person_clear_time_sec = self.get_parameter('exiting_person_clear_time_sec').value
        self.exiting_person_wait_timeout_sec = self.get_parameter('exiting_person_wait_timeout_sec').value
        self.exiting_person_motion_threshold = self.get_parameter('exiting_person_motion_threshold').value
        self.exiting_person_stationary_time_sec = self.get_parameter('exiting_person_stationary_time_sec').value
        self.use_door_open_check = self.get_parameter('use_door_open_check').value
        self.door_state_topic = self.get_parameter('door_state_topic').value
        self.door_state_stale_sec = self.get_parameter(
            'door_state_stale_sec').value
        self.door_initial_open_settle_sec = self.get_parameter('door_initial_open_settle_sec').value
        self.door_wait_timeout_sec = self.get_parameter('door_wait_timeout_sec').value
        self.use_full_door_cycle = self.get_parameter('use_full_door_cycle').value
        self.door_close_confirm_sec = self.get_parameter('door_close_confirm_sec').value
        self.door_close_wait_timeout_sec = self.get_parameter('door_close_wait_timeout_sec').value
        self.door_reopen_wait_timeout_sec = self.get_parameter('door_reopen_wait_timeout_sec').value
        self.door_reopen_settle_sec = self.get_parameter('door_reopen_settle_sec').value
        self.hold_door_open_during_exit = self.get_parameter(
            'hold_door_open_during_exit'
        ).value
        self.door_command_topic = self.get_parameter(
            'door_command_topic'
        ).value
        self.reset_amcl_from_odom_before_exit = self.get_parameter(
            'reset_amcl_from_odom_before_exit'
        ).value
        self.exit_localization_settle_sec = self.get_parameter(
            'exit_localization_settle_sec'
        ).value
        self.clear_costmaps_before_exit = self.get_parameter(
            'clear_costmaps_before_exit'
        ).value
        self.exit_costmap_settle_sec = self.get_parameter(
            'exit_costmap_settle_sec'
        ).value
        self.use_exit_path_precheck = self.get_parameter('use_exit_path_precheck').value
        self.exit_path_retry_interval_sec = self.get_parameter('exit_path_retry_interval_sec').value
        self.exit_path_max_retries = self.get_parameter('exit_path_max_retries').value
        self.entry_path_retry_interval_sec = self.get_parameter('entry_path_retry_interval_sec').value
        self.entry_path_max_retries = self.get_parameter('entry_path_max_retries').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.manual_exit_local_x = self.get_parameter('manual_exit_local_x').value
        self.manual_exit_local_y = self.get_parameter('manual_exit_local_y').value
        self.exit_keep_inward_heading = self.get_parameter('exit_keep_inward_heading').value
        self.goal_timeout_sec = self.get_parameter('goal_timeout_sec').value
        self.inflation_radius = self.get_parameter('inflation_radius').value
        self.dynamic_costmap_profile = self.get_parameter(
            'dynamic_costmap_profile').value
        self.entry_cost_scaling_factor = self.get_parameter(
            'entry_cost_scaling_factor').value
        self.entry_global_inflation_radius = self.get_parameter('entry_global_inflation_radius').value
        self.entry_global_cost_scaling_factor = self.get_parameter('entry_global_cost_scaling_factor').value
        for value in (self.entry_global_inflation_radius,self.entry_global_cost_scaling_factor):
            if not math.isfinite(value) or value<0:
                raise ValueError('entry global costmap overrides must be finite and nonnegative')
        self.normal_inflation_radius = self.get_parameter(
            'normal_inflation_radius').value
        self.normal_global_cost_scaling_factor = self.get_parameter(
            'normal_global_cost_scaling_factor').value
        self.normal_local_cost_scaling_factor = self.get_parameter(
            'normal_local_cost_scaling_factor').value

        self.elevator_geometry_valid = True
        self.configure_door_based_frame()

        # ==========================
        # ロボットの現在位置
        # ==========================
        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None

        # 最新のLiDARデータ
        self.latest_scan = None
        self.latest_hokuyo_scan = None
        self.latest_hokuyo_receipt = None
        self.last_hokuyo_obstacle_points = []

        # 一度ゴールを送ったかどうか
        self.goal_sent = False
        self.current_goal_x = None
        self.current_goal_y = None

        self.latest_turn_angle_deg = 0.0
        self.manual_goal_yaw = None
        self.exiting_person_wait_start_time = None
        self.exiting_person_clear_start_time = None
        self.exiting_person_stationary_start_time = None
        self.last_exiting_person_center = None
        self.door_raw_state = 'UNKNOWN'
        self.door_source_valid = False
        self.door_confirmed_open = False
        self.door_confirmed_closed = False
        self.door_state_summary = {
            'observed': 0, 'open': 0, 'closed': 0, 'blocked': 0
        }
        self.last_door_state_time = None
        self.door_wait_start_time = None
        self.door_initial_open_confirmed_at = None
        self.door_close_wait_start_time = None
        self.door_reopen_wait_start_time = None
        self.door_reopen_confirmed_at = None
        self.exit_localization_reset_at = None
        self.exit_costmap_clear_requested_at = None
        self.exit_door_hold_sent = False
        self.exit_path_retry_due = None
        self.exit_path_retry_count = 0
        self.entry_path_retry_due = None
        self.entry_path_retry_count = 0
        self.operation_phase = 'WAIT_INITIAL_OPEN'
        self.stop_active = False

        # ==========================
        # GUI状態表示
        # ==========================
        self.status_pub = self.create_publisher(
            String,
            '/elevator_status',
            10
        )
        self.last_status_text = ''
        self.state_pub = self.create_publisher(
            String, '/elevator_entry_state', 10
        )
        self.result_pub = self.create_publisher(
            String,
            '/elevator_experiment_result',
            10
        )
        score_marker_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, '/elevator_score_markers', score_marker_qos
        )
        self.evidence_marker_pub = self.create_publisher(
            MarkerArray, '/elevator_evidence_markers', score_marker_qos
        )
        self.candidate_diagnostics_pub = self.create_publisher(
            String, '/elevator_candidate_diagnostics', score_marker_qos
        )
        geometry_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.geometry_marker_pub = self.create_publisher(
            MarkerArray, '/elevator_geometry_markers', geometry_qos
        )
        self.navigation_goal_pub = self.create_publisher(
            PoseStamped,
            '/elevator_navigation_goal',
            10
        )
        self.door_state_sub = self.create_subscription(
            String,
            self.door_state_topic,
            lambda msg, epoch=0: self.door_state_for_epoch(msg, epoch),
            10, callback_group=self.input_group
        )
        self.reset_sub = self.create_subscription(
            String,
            '/elevator_experiment_reset',
            self.reset_callback,
            10
)
        self.manual_command_sub = self.create_subscription(
            String,
            '/elevator_manual_command',
            self.manual_command_callback,
            10, callback_group=self.safety_group
        )
        self.entry_start_sub = self.create_subscription(
            Bool,
            self.entry_start_topic,
            self.entry_start_callback,
            10
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10
        )
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        self.door_command_pub = self.create_publisher(
            String,
            self.door_command_topic,
            10
        )
        self.odom_offset_sub = self.create_subscription(
            Pose2D,
            '/elevator_odom_to_map',
            self.odom_offset_callback,
            10
        )

        # ==========================
        # 地図の読み込み
        # ==========================
        self.load_map()

        # ==========================
        # AMCL用QoS
        # ==========================
        amcl_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ==========================
        # 現在位置の購読
        # ==========================
        self.pose_sub = None
        self.odom_sub = None

        if self.use_odom_pose:
            self.odom_sub = self.create_subscription(
                Odometry,
                self.odom_topic,
                self.odom_callback,
                qos_profile_sensor_data, callback_group=self.input_group
            )
        else:
            self.pose_sub = self.create_subscription(
                PoseWithCovarianceStamped,
                self.pose_topic,
                self.pose_callback,
                amcl_qos, callback_group=self.input_group
            )

        # ==========================
        # LiDARの購読
        # ==========================
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            qos_profile_sensor_data, callback_group=self.input_group
        )
        self.hokuyo_scan_sub = (self.create_subscription(
            LaserScan, self.hokuyo_scan_topic, self.hokuyo_scan_callback,
            qos_profile_sensor_data, callback_group=self.input_group)
            if self.hokuyo_scan_topic else None)

        # ==========================
        # Nav2アクションクライアント
        # ==========================
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose', callback_group=self.safety_group
        )

# ==========================
# 経路生成用アクションクライアント
# ==========================
        self.path_client = ActionClient(
            self,
            ComputePathToPose,
            'compute_path_to_pose'
        )


        self.local_costmap_param_client = self.create_client(
            SetParameters,
            '/local_costmap/local_costmap/set_parameters'
        )
        self.global_costmap_param_client = self.create_client(
            SetParameters,
            '/global_costmap/global_costmap/set_parameters'
        )
        self.costmap_profile_state = (
            'normal' if self.dynamic_costmap_profile else 'entry')
        self.costmap_profile_request = None
        self.costmap_profile_pending = 0
        self.costmap_profile_success = True
        self.costmap_restore_required = False
        self.costmap_profile_started = None
        self.setup_controller_weights()
        self.global_costmap_clear_client = self.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap'
        )
        self.local_costmap_clear_client = self.create_client(
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap'
        )

        # 経路確認中かどうか
        self.path_checking = False
        self.active_goal_handle = None
        self.goal_start_time = None
        self.operation_finished = False
        self.entry_enabled = bool(self.start_immediately)
        self.area_stats = None
        self.setup_exit_map_switch()

        # 経路の最大許容曲がり角
        self.max_turn_angle = math.radians(120.0)        

        # ドア確定後の処理待ちを抑える。確定時間と開扉後の安全待機は別設定。
        self.main_loop_period_sec = self.get_parameter('main_loop_period_sec').value
        if (type(self.main_loop_period_sec) not in (int, float)
                or not math.isfinite(self.main_loop_period_sec)
                or self.main_loop_period_sec < 0.2):
            raise ValueError('main_loop_period_sec must be at least 0.2 seconds')
        self.timer = self.create_timer(
            self.main_loop_period_sec,
            self.main_loop
        )
        self.geometry_marker_timer = self.create_timer(
            1.0, self.publish_geometry_markers
        )
        self.state_timer = self.create_timer(
            1.0, self.publish_entry_state, callback_group=self.safety_group,
            clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.start_motion_watchdog()
        self.setup_lift_cycle()
        self.setup_arrival_map()
        self.setup_waypoint_cycle()
        self.setup_elevator_bank()
        self.setup_research()
        self.setup_boarding_admission()
        if self.entry_enabled:
            self.research_begin()
        self.publish_geometry_markers()

        self.get_logger().info(
            'エレベータ空き空間探索ノードを開始しました'
        )
        self.publish_entry_state()

    # =====================================================
    # waypoint到着／操作盤から進入開始を受信
    # =====================================================
    @motion_locked
    def maybe_start_on_closed_door(self):
        if (not (getattr(self, 'auto_start_on_closed', False)
                 or getattr(self, 'waypoint_cycle_armed', False))
                or getattr(self, 'auto_start_consumed', False)
                or self.entry_enabled or self.shutdown_requested or self.stop_active
                or self.navigation_unresolved()):
            return
        if getattr(self, 'bank_enabled', False) and self.bank_stage != 'READY_TO_BOARD':
            return
        _, confirmed_open, confirmed_closed, _ = self.get_door_state()
        if (getattr(self, 'waypoint_cycle_armed', False)
                and getattr(self, 'lift_cycle_enabled', False)):
            if confirmed_closed:
                self.waypoint_cycle_closed_seen = True
            if not getattr(self, 'waypoint_cycle_closed_seen', False):
                self.publish_status('waypoint到着：閉扉の確認を待っています')
                return
            if not confirmed_open:
                self.publish_status('waypoint到着・閉扉確認済み：開扉を待っています')
                return
            if not self.lift_start_ready():
                # Keep the waypoint request armed while the independent
                # startup checks finish their stability interval. The next
                # main-loop tick will retry against current inputs.
                self.publish_status('waypoint到着・開扉確認済み：起動条件の確認・安定待ち')
                return
            self.entry_start_callback(Bool(data=True))
            return
        if not confirmed_closed:
            self.publish_status('自動開始待ち：最初のドア閉を確認しています（②の操作は不要です）')
            return
        self.entry_start_callback(Bool(data=True))
        if self.entry_enabled:
            self.publish_status('最初のドア閉を確認：自動開始しました。ドアが開くまで待機します')

    def announce(self, event):
        voice = getattr(self, 'voice', None)
        if voice is not None:
            voice.say(event)

    def repeat_observing_while_open(self):
        """Audible progress for the open-door observation and path check."""
        voice = getattr(self, 'voice', None)
        if voice is not None:
            voice.repeat('observing')

    def stop_progress_repeat(self):
        voice = getattr(self, 'voice', None)
        if voice is not None:
            voice.stop_repeat()

    @motion_locked
    def request_motion_stop(self, reason='MANUAL_STOP'):
        self.preboard_data_waiting = False
        self.preboard_data_wait_reasons = []
        super().request_motion_stop(reason)
        voice = getattr(self, 'voice', None)
        if voice is not None:
            voice.cancel()

    def destroy_node(self):
        recorder = getattr(self, 'research_record', None)
        if recorder:
            recorder.finish(self.research_ros_time(), self.navigation_unresolved())
            recorder.close()
            if recorder.error:
                self.get_logger().error('Research recording incomplete: ' + recorder.error)
        voice = getattr(self, 'voice', None)
        if voice is not None:
            voice.close()
        listener = getattr(self, 'exit_tf_listener', None)
        if listener is not None:
            listener.unregister()
            listener.executor.shutdown()
            listener.dedicated_listener_thread.join()
            listener.node.destroy_node()
            self.exit_tf_listener = None
        return super().destroy_node()

    @motion_locked
    def entry_start_callback(self, msg):

        if not msg.data:
            return
        if getattr(self, 'bank_enabled', False) and self.bank_stage != 'READY_TO_BOARD':
            self.publish_status('開始不可：到着EVの選択とドア前への接近を待ってください')
            return
        if self.shutdown_requested or self.stop_active or self.navigation_unresolved():
            self.publish_status('開始不可：終了処理・Nav2のキャンセル完了待ちです'
                                if not self.stop_latching_enabled else
                                '開始不可：安全停止保持中、またはNav2終端未確認です')
            return

        if getattr(self, 'lift_cycle_enabled', False) and not self.lift_start_ready():
            startup = self.lift_startup_snapshot()
            self.safety_event('start_refused', startup=startup)
            if all(startup['checks'].values()) and not startup['stable']:
                self.publish_status(
                    f'開始待ち：起動条件の{self.lift_startup_settle_sec:g}秒安定確認中です')
            else:
                self.publish_status('開始不可：IMU・評価・Nav2・停止経路などの起動条件を確認してください')
            return

        if self.exit_map_requires_restart or self.exit_map_stage is not None:
            self.publish_status('開始不可：地図切替後は新しい地図のドア・かご座標を設定して再起動してください')
            return

        if self.entry_enabled and not self.operation_finished:
            self.get_logger().info('進入処理中のため重複した開始要求を無視します')
            return

        self.stop_route.poll()
        if not self.stop_route.verified:
            self.publish_status('開始不可：速度制御の停止入力を確認できません（' + self.stop_route.reason + '）')
            return
        health = self.input_health()
        invalid = [name + '=' + item['state'] for name, item in health.items() if not item['valid']]
        if invalid:
            self.publish_status('開始不可：入力の更新を待ってください（' + ' / '.join(invalid) + '）')
            return
        if self.doorway_region() != 'OUTSIDE':
            self.publish_status('開始不可：車体全体がドア外にあることを確認してください（' + self.doorway_region() + '）')
            return

        self.reset_callback(String())
        self.stop_reason = None
        self.cancel_state = 'NONE'
        self.cancel_requested_at = None
        # Only an accepted operator/waypoint request can release an old mux lock.
        self.safety_lock_pub.publish(Bool(data=False))
        self.auto_start_consumed = True
        self.waypoint_cycle_armed = False
        self.entry_enabled = True
        if hasattr(self, 'research_begin'):
            self.research_begin()
        self.publish_status('進入開始要求を受信：エレベータ進入処理を開始します')
        self.get_logger().info(
            f'{self.entry_start_topic} でBool trueを受信しました'
        )

    # =====================================================
    # GUIへ状態送信
    # =====================================================
    @motion_locked
    def publish_status(self, text):

        msg = String()

        msg.data = text

        self.last_status_text = text
        self.status_pub.publish(msg)
        if self.operation_phase != self.safety_last_phase:
            previous = self.safety_last_phase
            self.safety_last_phase = self.operation_phase
            self.safety_event('phase_transition', previous_phase=previous)

    @motion_locked
    def publish_entry_state(self):
        """Expose the current step even when no event message is being sent."""
        door_age = (None if self.last_door_state_time is None else
                    time.monotonic() - self.last_door_state_time)
        door_open_fresh = (self.door_source_valid and self.door_confirmed_open
                           and door_age is not None
                           and (self.door_state_stale_sec <= 0.0
                                or door_age <= self.door_state_stale_sec))
        voice = getattr(self, 'voice', None)
        if voice is not None:
            event = door_progress_event(
                self.operation_phase,
                active=not (self.operation_finished or self.stop_active or
                            self.shutdown_requested),
                entry_enabled=self.entry_enabled,
                auto_start_consumed=self.auto_start_consumed,
                auto_start_on_closed=self.auto_start_on_closed,
                waypoint_cycle_armed=self.waypoint_cycle_armed,
                door_open_fresh=door_open_fresh,
                inputs_valid=self.inputs_valid())
            if event == 'observing':
                self.repeat_observing_while_open()
            elif event == 'door_detecting':
                voice.repeat(event, interval=12.0)
            else:
                self.stop_progress_repeat()
        self.entry_state_sequence = getattr(self, 'entry_state_sequence', 0) + 1
        state = {
            'state_sequence': self.entry_state_sequence,
            'stamp_ros': self.get_clock().now().nanoseconds / 1e9,
            'nav_unresolved': self.navigation_unresolved(),
            'normal_profile_ready': (not self.dynamic_costmap_profile or (
                self.costmap_profile_state == 'normal' and self.costmap_profile_success
                and self.costmap_profile_request is None)),
            'enabled': bool(self.entry_enabled),
            'auto_start_on_closed': bool(getattr(self, 'auto_start_on_closed', False)),
            'auto_start_consumed': bool(getattr(self, 'auto_start_consumed', False)),
            'waypoint_cycle_armed': bool(getattr(self, 'waypoint_cycle_armed', False)),
            'waypoint_cycle_id': getattr(self, 'waypoint_cycle_id', None),
            'waypoint_cycle_result': getattr(self, 'waypoint_cycle_result', ''),
            'session_id': self.sweep_owner_id,
            'shutdown_requested': self.shutdown_requested,
            'startup': self.lift_startup_snapshot() if getattr(self, 'lift_cycle_enabled', False) else None,
            'phase': self.operation_phase,
            'bank': self.bank_snapshot(),
            'path_checking': bool(self.path_checking),
            'goal_sent': bool(self.goal_sent),
            'finished': bool(self.operation_finished),
            'door': self.door_raw_state,
            'door_confirmed_open': bool(self.door_confirmed_open),
            'goal_x': self.current_goal_x,
            'goal_y': self.current_goal_y,
            'status': self.last_status_text,
            'area_stats': self.area_stats,
            'boarding': self.boarding_snapshot(),
            'controller_weights': self.controller_weights_snapshot(),
            'preboard_data_waiting': getattr(self, 'preboard_data_waiting', False),
            'preboard_data_wait_reasons': getattr(self, 'preboard_data_wait_reasons', []),
            'research': {
                'method': getattr(self, 'comparison_method', 'legacy'),
                'decision': {k: v for k, v in (getattr(getattr(self, 'research_record', None), 'latest', None) or {}).items() if k != 'cells'},
                'logging_error': getattr(getattr(self, 'research_record', None), 'error', None),
            },
            'map_switch_enabled': bool(self.switch_map_after_exit or getattr(self, 'switch_map_before_exit', False)),
            'map_switch_stage': self.exit_map_stage,
            'exit_map_wait_reasons': getattr(self, 'exit_map_wait_reasons', []),
            'exit_map_tf_error': getattr(self, 'exit_map_tf_error', ''),
            'arrival_map_stage': getattr(self, 'arrival_map_stage', None),
            'arrival_map_ready': getattr(self, 'arrival_map_ready', False),
            'arrival_wait_reasons': getattr(self, 'arrival_wait_reasons', []),
            'arrival_exit_mode': getattr(self, 'arrival_exit_mode', 'auto'),
            'arrival_exit_preview': getattr(self, 'arrival_preview', None),
            'run_manifest_path': getattr(self, 'run_manifest_path', ''),
            'map_switched': self.exit_map_switched,
            'exit_map_yaml_path': self.exit_map_yaml_path if self.switch_map_after_exit else '',
            'safety': self.safety_snapshot(),
            'lift_cycle': self.lift_cycle.snapshot() if getattr(self, 'lift_cycle_enabled', False) else None,
        }
        self.state_pub.publish(String(data=json.dumps(state, ensure_ascii=False)))

    @motion_locked
    def publish_geometry_markers(self):
        """Keep the measured door and cabin geometry visible before entry starts."""
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        post_exit = (getattr(self, 'switch_map_after_exit', False)
                     and getattr(self, 'exit_map_requires_restart', False))
        display_geometry = getattr(self, 'exit_map_geometry', None) if post_exit else None
        if post_exit:
            # The source geometry is no longer in the currently loaded map.
            markers.markers.append(Marker(action=Marker.DELETEALL))
            if not self.exit_map_switched or display_geometry is None:
                self.geometry_marker_pub.publish(markers)
                return

        def add_marker(marker_id, marker_type, color):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = stamp
            marker.ns = 'elevator_geometry'
            marker.id = marker_id
            marker.type = marker_type
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.color.a = 1.0
            marker.color.r, marker.color.g, marker.color.b = color
            markers.markers.append(marker)
            return marker

        door_points = [
            (self.door_left_x, self.door_left_y),
            (self.door_right_x, self.door_right_y),
        ]
        if display_geometry is not None:
            door_points = [(display_geometry['door_left_x'], display_geometry['door_left_y']),
                           (display_geometry['door_right_x'], display_geometry['door_right_y'])]
            cabin_points = list(zip(display_geometry['elevator_polygon_x'], display_geometry['elevator_polygon_y']))
        elif self.use_elevator_polygon:
            cabin_points = self.elevator_polygon
        else:
            cabin_points = [
                self.elevator_local_to_world(local_x, local_y)
                for local_x, local_y in (
                    (self.elevator_local_x_min, self.elevator_local_y_min),
                    (self.elevator_local_x_min, self.elevator_local_y_max),
                    (self.elevator_local_x_max, self.elevator_local_y_max),
                    (self.elevator_local_x_max, self.elevator_local_y_min),
                )
            ]

        for marker_id, points, color in (
            (0, door_points, (1.0, 0.45, 0.0)),
            (1, cabin_points + [cabin_points[0]], (0.0, 0.9, 1.0)),
        ):
            line = add_marker(marker_id, Marker.LINE_STRIP, color)
            line.scale.x = 0.04
            for x, y in points:
                point = Point()
                point.x, point.y, point.z = x, y, 0.08
                line.points.append(point)

        for index, (label, x, y, color) in enumerate(
            [(f'D{i + 1}', x, y, (1.0, 0.45, 0.0))
             for i, (x, y) in enumerate(door_points)]
            + [(f'C{i + 1}', x, y, (0.0, 0.9, 1.0))
               for i, (x, y) in enumerate(cabin_points)]
        ):
            point_marker = add_marker(2 + index * 2, Marker.SPHERE, color)
            point_marker.pose.position.x = x
            point_marker.pose.position.y = y
            point_marker.pose.position.z = 0.1
            point_marker.scale.x = 0.12
            point_marker.scale.y = 0.12
            point_marker.scale.z = 0.12

            text_marker = add_marker(
                3 + index * 2, Marker.TEXT_VIEW_FACING, color
            )
            text_marker.pose.position.x = x
            text_marker.pose.position.y = y
            text_marker.pose.position.z = 0.32
            text_marker.scale.z = 0.14
            text_marker.text = f'{label} ({x:.2f}, {y:.2f})'

        if self.use_door_front_forbidden and not post_exit:
            forbidden = add_marker(100, Marker.LINE_STRIP, (1.0, 1.0, 0.0))
            forbidden.ns = 'elevator_stop_forbidden'
            forbidden.scale.x = 0.025
            half = self.grid_step * .5
            corners = [(self.door_front_local_x_min-half, self.door_front_local_y_min-half),
                       (self.door_front_local_x_max+half, self.door_front_local_y_min-half),
                       (self.door_front_local_x_max+half, self.door_front_local_y_max+half),
                       (self.door_front_local_x_min-half, self.door_front_local_y_max+half)]
            for local_x, local_y in corners + corners[:1]:
                x, y = self.elevator_local_to_world(local_x, local_y)
                forbidden.points.append(Point(x=x, y=y, z=.09))

        if (not post_exit and getattr(self, 'bank_enabled', False)
                and hasattr(self, 'bank_selector')):
            markers.markers.extend(bank_geometry_markers(
                self.bank_selector.elevators, self.bank_selected, stamp))

        self.geometry_marker_pub.publish(markers)
    
    # =====================================================
    # 実験結果送信
    # =====================================================
    def publish_experiment_result(self, text):

        code = text.split(',', 1)[0]
        # The waypoint owner consumes GOAL_REACHED plus map_switched/finished.
        # Keep MAP_SWITCH_DONE on the experiment topic without stranding it.
        self.waypoint_cycle_result = 'GOAL_REACHED' if code == 'MAP_SWITCH_DONE' else code
        msg = String()

        msg.data = text

        self.result_pub.publish(msg)
        if hasattr(self, 'research_result'):
            self.research_result(text)
    

    # =====================================================
    # 実験用リセットを受信
    # =====================================================
    @motion_locked
    def reset_callback(self, msg):

        if self.stop_active or self.navigation_unresolved() or self.path_checking:
            self.publish_status('リセット不可：停止保持の解除、または処理の終端確認が必要です')
            return
        if hasattr(self, 'research_reset'):
            self.research_reset()
        self.motion_epoch += 1
        self.lift_pending_entry = None

        if self.exit_map_requires_restart or self.exit_map_stage is not None:
            self.publish_status('リセット不可：マップ切替状態を確認し、設定を更新して再起動してください')
            return

        if getattr(self, 'lift_cycle_enabled', False):
            self.reset_lift_cycle()

        voice = getattr(self, 'voice', None)
        if voice is not None:
            voice.cancel(reset=True)
        self.clear_candidate_evaluation('RESET')
        self.reset_boarding_admission()
        self.goal_sent = False
        self.current_goal_x = None
        self.current_goal_y = None
        self.path_checking = False
        self.operation_finished = False
        self.active_goal_handle = None
        self.goal_start_time = None
        self.exiting_person_wait_start_time = None
        self.exiting_person_clear_start_time = None
        self.exiting_person_stationary_start_time = None
        self.last_exiting_person_center = None
        self.door_raw_state = 'UNKNOWN'
        self.door_confirmed_open = False
        self.door_confirmed_closed = False
        self.door_state_summary = {
            'observed': 0, 'open': 0, 'closed': 0, 'blocked': 0
        }
        self.last_door_state_time = None
        self.door_wait_start_time = None
        self.door_initial_open_confirmed_at = None
        self.door_close_wait_start_time = None
        self.door_reopen_wait_start_time = None
        self.door_reopen_confirmed_at = None
        self.exit_localization_reset_at = None
        self.exit_costmap_clear_requested_at = None
        self.exit_door_hold_sent = False
        self.exit_path_retry_due = None
        self.exit_path_retry_count = 0
        self.entry_path_retry_due = None
        self.entry_path_retry_count = 0
        self.operation_phase = 'WAIT_INITIAL_OPEN'
        self.stop_active = False

        self.get_logger().info(
            '実験用リセットを受信しました'
        )

        self.publish_status(
            '実験用リセット：再判定します'
        )

    # =====================================================
    # GUIからの手動操作
    # =====================================================
    @motion_locked
    def manual_command_callback(self, msg):

        command = msg.data.strip()
        if command == 'operator_takeover':
            self.handoff_to_operator()
            return
        if command == 'reset_stop':
            self.reset_safety_stop()
            return

        if command == 'stop':
            self.emergency_stop()
            return

        if command == 'force_exit':
            if self.exit_map_requires_restart or self.exit_map_stage is not None:
                self.publish_status('退室要求不可：マップ切替状態を確認して再起動してください')
                return
            self.exit_path_retry_due = None
            self.exit_path_retry_count = 0
            self.force_exit_elevator()
            return

        self.get_logger().warn(
            f'unknown manual command: {command}'
        )

    def emergency_stop(self):
        self.request_motion_stop('MANUAL_STOP')

    def publish_stop_command(self):

        if self.stop_active:
            self.emit_stop()

    @motion_locked
    def force_exit_elevator(self):

        if getattr(self, 'shutdown_requested', False):
            return
        if getattr(self,'arrival_map_ready',False) and self.operation_phase not in ('WAIT_REOPEN','ARRIVAL_HOLD'):
            return
        if getattr(self,'arrival_map_ready',False) and not self.arrival_pose_ready():
            return
        if getattr(self, 'lift_cycle_enabled', False) and not self.lift_exit_allowed():
            self.publish_status('退室不可：今回の昇降停止・目的階確認が必要です')
            return

        if (self.stop_active or self.navigation_unresolved() or self.path_checking
                or not self.inputs_valid() or not self.get_door_state()[1]):
            self.publish_status('退室不可：停止保持・Nav2状態・入力鮮度・開扉を確認してください')
            return

        if self.use_door_based_elevator:
            exit_x, exit_y = self.elevator_local_to_world(
                self.manual_exit_local_x,
                self.manual_exit_local_y
            )
        else:
            exit_x = (
                self.door_front_x_min
                +
                self.door_front_x_max
            ) / 2.0
            exit_y = self.door_front_y_min - 0.80

        self.goal_sent = False
        self.path_checking = False
        self.active_goal_handle = None
        self.goal_start_time = None
        self.current_goal_x = exit_x
        self.current_goal_y = exit_y
        self.manual_goal_yaw = math.atan2(
            -self.inside_axis_y,
            -self.inside_axis_x
        )
        if getattr(self, 'exit_keep_inward_heading', False):
            # Keep the boarding heading at the outside goal so MPPI can back
            # out without being required to finish with a 180-degree turn.
            self.manual_goal_yaw = math.atan2(self.inside_axis_y, self.inside_axis_x)
        if getattr(self,'arrival_map_ready',False) and getattr(self,'arrival_exit_local_pose',[]):
            local_x,local_y,yaw=self.arrival_exit_local_pose
            exit_x,exit_y=self.elevator_local_to_world(local_x,local_y)
            self.current_goal_x,self.current_goal_y=exit_x,exit_y
            self.manual_goal_yaw=math.atan2(self.door_axis_y,self.door_axis_x)+yaw
        self.operation_finished = False
        self.operation_phase = 'EXITING'

        self.publish_status(
            f'強制退室：出口目標へ移動 x={exit_x:.2f}, y={exit_y:.2f}'
        )

        if self.use_exit_path_precheck:
            self.check_path_then_send_goal(
                exit_x,
                exit_y,
                mode='force_exit'
            )
            return

        # ComputePathToPoseの単発判定は、開扉直後の古いコストマップで
        # false negativeになりやすい。通常のNavigateToPoseへ直接渡し、
        # Nav2の再計画・復旧動作に任せる。
        self.set_inflation_radius()

        if self.send_nav_goal(exit_x, exit_y, mode='force_exit'):
            self.goal_sent = True
            return
        if self.stop_active:
            return

        self.publish_experiment_result(
            f'NG_NAV_SERVICE,{exit_x:.2f},{exit_y:.2f},0.0'
        )
        self.operation_finished = True
        self.operation_phase = 'FINISHED'

    def set_inflation_radius(self, radius=None):

        # 従来の呼び出しは残し、global/local両方のプロファイル
        # 変更へ集約する。
        self.request_costmap_profile('entry')

    def make_costmap_request(self, radius, scaling_factor):
        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                name='inflation_layer.inflation_radius',
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE,
                    double_value=float(radius))),
            Parameter(
                name='inflation_layer.cost_scaling_factor',
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE,
                    double_value=float(scaling_factor))),
        ]
        return request

    @motion_locked
    def request_costmap_profile(self, profile, force=False):
        if not self.dynamic_costmap_profile:
            return True
        if self.costmap_profile_request is not None:
            started = getattr(self, 'costmap_profile_started', None)
            if started is not None and time.monotonic() - started > getattr(self, 'nav_profile_timeout_sec', 5.):
                if not self.stop_active:
                    self.request_motion_stop('NAV_PROFILE_UPDATE_TIMEOUT')
            return False
        use_controller = getattr(self, 'dynamic_controller_profile', False)
        if use_controller and profile == 'normal' and not self.costmap_restore_required:
            # No controller write was sent. Discard an abandoned pre-entry read,
            # including its late callback, and recapture at the next entry.
            self.controller_snapshot_future = None
            self.controller_original_weights = None
        if (self.costmap_profile_state == profile and not force
                and (profile != 'normal' or not getattr(self, 'costmap_restore_required', False))):
            return True
        if use_controller:
            if self.navigation_unresolved() or getattr(self, 'lift_foreign_active_ids', set()):
                return False
            if profile == 'entry' and not self.capture_controller_weights():
                return False

        clients = (
            self.global_costmap_param_client,
            self.local_costmap_param_client,
        )
        if use_controller and self.controller_original_weights is not None:
            clients += (self.controller_param_client,)
        if not all(client.service_is_ready() for client in clients):
            return False

        if profile == 'entry':
            radius = self.inflation_radius
            global_scaling = self.entry_cost_scaling_factor
            local_scaling = self.entry_cost_scaling_factor
        else:
            radius = self.normal_inflation_radius
            global_scaling = self.normal_global_cost_scaling_factor
            local_scaling = self.normal_local_cost_scaling_factor

        global_radius=radius
        if profile=='entry':
            if self.entry_global_inflation_radius>0:global_radius=self.entry_global_inflation_radius
            if self.entry_global_cost_scaling_factor>0:global_scaling=self.entry_global_cost_scaling_factor
        self.get_logger().info(f'Nav2 profile request {profile}: global radius={global_radius:.3f}, scaling={global_scaling:.3f}; local radius={radius:.3f}, scaling={local_scaling:.3f}')
        self.costmap_profile_request = profile
        self.costmap_profile_pending = len(clients)
        self.costmap_profile_success = True
        self.costmap_profile_started = time.monotonic()
        if profile == 'entry':
            self.costmap_restore_required = True
        requests = [self.make_costmap_request(global_radius, global_scaling),
                    self.make_costmap_request(radius, local_scaling)]
        if len(clients) == 3:
            requests.append(self.make_controller_request(profile))
            self.safety_event('controller_weights_requested', profile=profile,
                              weights={p.name: p.value.double_value for p in requests[-1].parameters})
        for client, request in zip(clients, requests):
            try:
                future = client.call_async(request)
                future.add_done_callback(self.costmap_profile_callback)
            except Exception as exc:
                self.get_logger().error('Nav2 profile request failed: ' + str(exc))
                self.costmap_profile_callback(None)
        return False

    @motion_locked
    def costmap_profile_callback(self, future):
        try:
            response = future.result()
            if len(response.results) != 2 or not all(result.successful for result in response.results):
                self.costmap_profile_success = False
        except Exception as exc:
            self.get_logger().error(f'costmap profile update failed: {exc}')
            self.costmap_profile_success = False

        self.costmap_profile_pending -= 1
        if self.costmap_profile_pending > 0:
            return

        requested = self.costmap_profile_request
        self.costmap_profile_request = None
        if self.costmap_profile_success:
            self.costmap_profile_state = requested
            if requested == 'normal':
                self.costmap_restore_required = False
                if getattr(self, 'dynamic_controller_profile', False):
                    self.controller_original_weights = None
            if requested == 'entry':
                self.get_logger().info(
                    'Nav2 costmap -> elevator profile '
                    f'(inflation={self.inflation_radius:.2f}, '
                    f'scaling={self.entry_cost_scaling_factor:.1f})')
            else:
                self.get_logger().info(
                    'Nav2 costmap -> normal profile '
                    f'(inflation={self.normal_inflation_radius:.2f})')
        else:
            self.costmap_profile_state = 'unknown'
            self.get_logger().warn(
                f'Nav2 costmap profile "{requested}" could not be applied')
            self.request_motion_stop('NAV_PROFILE_UPDATE_FAILED')
    # =====================================================
    # 地図を読み込む
    # =====================================================
    def load_map(self):

        if self.map_yaml_path == '':
            raise RuntimeError('map_yaml_path が空です')

        with open(self.map_yaml_path, 'r') as f:
            map_info = yaml.safe_load(f)

        image_path = map_info['image']

        if not image_path.startswith('/'):
            image_path = os.path.join(
                os.path.dirname(self.map_yaml_path),
                image_path
            )

        self.map_img = cv2.imread(
            image_path,
            cv2.IMREAD_GRAYSCALE
        )

        if self.map_img is None:
            raise RuntimeError(
                '地図画像を読み込めません: ' + image_path
            )

        self.resolution = map_info['resolution']
        self.origin_x = map_info['origin'][0]
        self.origin_y = map_info['origin'][1]
        self.origin_yaw = map_info['origin'][2]
        if map_info.get('negate',0):self.map_img=255-self.map_img

        self.height, self.width = self.map_img.shape

        self.get_logger().info(
            f'地図を読み込みました: {image_path}'
        )

        self.get_logger().info(
            f'解像度={self.resolution}, '
            f'原点=({self.origin_x}, {self.origin_y})'
        )

    # =====================================================
    # AMCLからロボットの現在位置を取得
    # =====================================================
    @motion_locked
    def pose_callback(self, msg):
        self.observe_pose(msg)
        if (self.robot_x is not None and self.robot_y is not None
                and math.hypot(msg.pose.pose.position.x-self.robot_x,msg.pose.pose.position.y-self.robot_y)>.5):
            self.safety_event('amcl_position_step',previous=[self.robot_x,self.robot_y],
                              current=[msg.pose.pose.position.x,msg.pose.pose.position.y],
                              sample_stamp_ros=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                              frame_id=msg.header.frame_id)
        self.last_pose_frame=msg.header.frame_id
        self.last_pose_covariance=(msg.pose.covariance[0],msg.pose.covariance[7],msg.pose.covariance[35])

        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        siny_cosp = 2.0 * (
            q.w * q.z + q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y + q.z * q.z
        )

        self.robot_yaw = math.atan2(
            siny_cosp,
            cosy_cosp
        )
        self.record_input('pose', msg)

    # =====================================================
    # Gazeboオドメトリからロボットの現在位置を取得
    # =====================================================
    @motion_locked
    def odom_callback(self, msg):

        odom_x = msg.pose.pose.position.x
        odom_y = msg.pose.pose.position.y
        cos_yaw = math.cos(self.odom_to_map_yaw)
        sin_yaw = math.sin(self.odom_to_map_yaw)

        self.robot_x = (
            self.odom_to_map_x
            + cos_yaw * odom_x
            - sin_yaw * odom_y
        )
        self.robot_y = (
            self.odom_to_map_y
            + sin_yaw * odom_x
            + cos_yaw * odom_y
        )

        q = msg.pose.pose.orientation

        siny_cosp = 2.0 * (
            q.w * q.z + q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y + q.z * q.z
        )

        odom_yaw = math.atan2(
            siny_cosp,
            cosy_cosp
        )
        self.robot_yaw = math.atan2(
            math.sin(odom_yaw + self.odom_to_map_yaw),
            math.cos(odom_yaw + self.odom_to_map_yaw)
        )
        self.record_input('pose', msg)

    @motion_locked
    def odom_offset_callback(self, msg):
        """Follow the resettable map->odom transform used by the simulator."""
        self.odom_to_map_x = msg.x
        self.odom_to_map_y = msg.y
        self.odom_to_map_yaw = msg.theta

    # =====================================================
    # LiDARデータを保存
    # =====================================================
    @motion_locked
    def scan_callback(self, msg):
        self.observe_scan(msg)
        self.latest_scan = msg
        self.record_input('scan', msg)

    @motion_locked
    def hokuyo_scan_callback(self, msg):
        self.latest_hokuyo_scan = msg
        self.latest_hokuyo_receipt = time.monotonic()

    def get_hokuyo_obstacles(self):
        """Add fresh foot-level returns as obstacles using the sensor origin."""
        self.last_hokuyo_obstacle_points = []
        if not getattr(self, 'hokuyo_scan_topic', ''):
            return []
        msg = self.latest_hokuyo_scan
        received = self.latest_hokuyo_receipt
        if msg is None or received is None:
            raise ValueError('HOKUYO_MISSING')
        at = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        now = self.get_clock().now().nanoseconds / 1e9
        max_age = self.hokuyo_scan_max_age_sec
        if (at <= 0 or at - now > .1 or now - at > max_age
                or time.monotonic() - received > max_age):
            raise ValueError('HOKUYO_STALE')
        upper = getattr(self, 'latest_scan', None)
        if upper is not None:
            upper_at = upper.header.stamp.sec + upper.header.stamp.nanosec / 1e9
            if abs(at - upper_at) > .2:
                raise ValueError('HOKUYO_SCAN_TIME_MISMATCH')
        if getattr(self, 'observation_guard_enabled', False):
            snapshot = self.observation_at(msg, received)
            origin = snapshot['evidence'].origin
        else:
            if msg.header.frame_id != 'robot/hokuyo_link':
                raise ValueError('HOKUYO_FRAME_MISMATCH')
            c, s = math.cos(self.robot_yaw), math.sin(self.robot_yaw)
            origin = (self.robot_x + c*self.hokuyo_origin_x - s*self.hokuyo_origin_y,
                      self.robot_y + s*self.hokuyo_origin_x + c*self.hokuyo_origin_y,
                      self.robot_yaw + self.hokuyo_origin_yaw)
        points = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            angle = origin[2] + msg.angle_min + i*msg.angle_increment
            x = origin[0] + r*math.cos(angle)
            y = origin[1] + r*math.sin(angle)
            if (self.is_inside_elevator(x, y)
                    and not self.is_near_wall_on_static_map(x, y, .15)):
                points.append((float(x), float(y)))
        self.last_hokuyo_obstacle_points = points
        return points

    # =====================================================
    # 実世界座標を地図画像のピクセル座標に変換
    # =====================================================
    def world_to_map_pixel(self, x, y):
        angle=getattr(self,'origin_yaw',0.)
        dx,dy=x-self.origin_x,y-self.origin_y
        mx=math.floor((math.cos(angle)*dx+math.sin(angle)*dy)/self.resolution)
        my=math.floor((-math.sin(angle)*dx+math.cos(angle)*dy)/self.resolution)

        px = mx
        py = self.height - 1 - my

        return px, py

    # =====================================================
    # 静的地図上で壁らしいかどうか判定
    # 位置ズレがあっても横の壁を見落としにくいように、
    # 周辺の複数点を見て壁らしさを集計する
    # =====================================================
    def is_wall_on_static_map(self, x, y):

        return self.is_wall_like_on_static_map(x, y, radius=0.05)

    def is_too_close_to_static_wall(self, x, y):

        clearance = max(0.0, float(self.static_wall_clearance))
        return self.is_wall_like_on_static_map(x, y, radius=clearance)

    def is_wall_like_on_static_map(self, x, y, radius=0.0):

        offsets = []

        if radius <= 0.0:
            offsets = [(0.0, 0.0)]
        else:
            # 円内を地図解像度ごとに調べる。外周の数点だけを調べると、
            # 細い壁をまたいで見落とすことがある。
            step = max(float(self.resolution), 0.01)
            sample_count = max(1, int(math.ceil(radius / step)))

            for ix in range(-sample_count, sample_count + 1):
                for iy in range(-sample_count, sample_count + 1):
                    dx = ix * step
                    dy = iy * step

                    if math.hypot(dx, dy) <= radius + (step * 0.25):
                        offsets.append((dx, dy))

        wall_hits = 0
        total_checks = 0

        for dx, dy in offsets:
            px, py = self.world_to_map_pixel(x + dx, y + dy)

            total_checks += 1

            if (
                px < 0 or
                px >= self.width or
                py < 0 or
                py >= self.height
            ):
                wall_hits += 1
                continue

            value = self.map_img[py, px]

            if value < self.map_wall_threshold:
                wall_hits += 1

        return wall_hits >= 1

    # =====================================================
    # ドア左右端からエレベータ用ローカル座標系を作る
    # =====================================================
    def configure_door_based_frame(self):

        self.door_center_x = (
            self.door_left_x
            +
            self.door_right_x
        ) / 2.0

        self.door_center_y = (
            self.door_left_y
            +
            self.door_right_y
        ) / 2.0

        door_dx = self.door_right_x - self.door_left_x
        door_dy = self.door_right_y - self.door_left_y
        door_width = math.hypot(door_dx, door_dy)

        if door_width <= 0.0:
            self.get_logger().warn(
                'ドア左右端が同じ点です。map座標の矩形判定へ戻します'
            )
            self.use_door_based_elevator = False
            self.door_axis_x = 1.0
            self.door_axis_y = 0.0
            self.inside_axis_x = 0.0
            self.inside_axis_y = 1.0
            self.door_width = 0.0
            return

        self.door_width = door_width
        self.door_axis_x = door_dx / door_width
        self.door_axis_y = door_dy / door_width

        sign = 1.0

        if self.elevator_inside_sign < 0.0:
            sign = -1.0

        self.inside_axis_x = -self.door_axis_y * sign
        self.inside_axis_y = self.door_axis_x * sign

        if self.use_elevator_polygon:
            if len(self.elevator_polygon) < 3:
                self.get_logger().warn(
                    'かごポリゴンが3点未満のため矩形探索へ戻します'
                )
                self.use_elevator_polygon = False
            else:
                local_corners = [
                    self.world_to_elevator_local(x, y)
                    for x, y in self.elevator_polygon
                ]
                if max(point[1] for point in local_corners) <= 0.0:
                    self.elevator_geometry_valid = False
                    self.get_logger().error(
                        'かご外周がドアの外側にあります。ドア座標と'
                        'かご外周を同じ地図で計測してください'
                    )
                self.elevator_local_x_min = min(
                    point[0] for point in local_corners)
                self.elevator_local_x_max = max(
                    point[0] for point in local_corners)
                self.elevator_local_y_min = min(
                    point[1] for point in local_corners)
                self.elevator_local_y_max = max(
                    point[1] for point in local_corners)

        self.get_logger().info(
            'ドア基準エレベータ座標系: '
            f'door_center=({self.door_center_x:.2f}, {self.door_center_y:.2f}), '
            f'door_width={door_width:.2f}, '
            f'inside_axis=({self.inside_axis_x:.2f}, {self.inside_axis_y:.2f})'
        )

    def world_to_elevator_local(self, x, y):

        dx = x - self.door_center_x
        dy = y - self.door_center_y

        local_x = (
            dx * self.door_axis_x
            +
            dy * self.door_axis_y
        )

        local_y = (
            dx * self.inside_axis_x
            +
            dy * self.inside_axis_y
        )

        return local_x, local_y

    def elevator_local_to_world(self, local_x, local_y):

        x = (
            self.door_center_x
            +
            local_x * self.door_axis_x
            +
            local_y * self.inside_axis_x
        )

        y = (
            self.door_center_y
            +
            local_x * self.door_axis_y
            +
            local_y * self.inside_axis_y
        )

        return x, y

    # =====================================================
    # 独立DoorNodeから最新の認識結果を受け取る
    # =====================================================
    @motion_locked
    def door_state_callback(self, msg):

        fields = [field.strip() for field in msg.data.split(',')]
        raw_state = fields[0].upper() if fields else 'UNKNOWN'
        if raw_state not in {'OPEN', 'CLOSED', 'BLOCKED', 'UNKNOWN'}:
            self.get_logger().warn(f'不正なドア状態を受信しました: {msg.data}')
            return

        values = {}
        for field in fields[1:]:
            if '=' not in field:
                continue
            key, value = field.split('=', 1)
            values[key.strip()] = value.strip()

        self.door_source_valid = values.get('inputs_valid') == '1'
        for key in ('scan','pose'):
            try:
                stamp=float(values.get(key+'_stamp_ros',''))
                if not math.isfinite(stamp) or stamp<=0:stamp=None
            except (ValueError,TypeError):stamp=None
            setattr(self,'last_door_'+key+'_stamp',stamp)
        self.door_raw_state = raw_state
        self.door_confirmed_open = (
            self.door_source_valid and raw_state == 'OPEN' and values.get('confirmed_open') == '1'
        )
        self.door_confirmed_closed = (
            self.door_source_valid and raw_state == 'CLOSED' and values.get('confirmed_closed') == '1'
        )
        for key in self.door_state_summary:
            try:
                self.door_state_summary[key] = int(values.get(key, 0))
            except ValueError:
                self.door_state_summary[key] = 0
        self.last_door_state_time = time.monotonic()
        self.observe_boarding_skip_door()

    @motion_locked
    def get_door_state(self):

        now = time.monotonic()
        stale = self.last_door_state_time is None
        if (
            not stale
            and self.door_state_stale_sec > 0.0
            and now - self.last_door_state_time > self.door_state_stale_sec
        ):
            stale = True

        if stale or not self.door_source_valid or not self.inputs_valid():
            self.door_raw_state = 'UNKNOWN'
            self.door_confirmed_open = False
            self.door_confirmed_closed = False

        return (
            self.door_raw_state,
            self.door_confirmed_open,
            self.door_confirmed_closed,
            self.door_state_summary,
        )

    def should_wait_for_open_door(self):

        if not self.use_door_open_check:
            return False

        raw_state, confirmed_open, _, summary = self.get_door_state()
        now = self.get_clock().now().nanoseconds / 1e9

        if confirmed_open:
            self.repeat_observing_while_open()
            if self.door_initial_open_confirmed_at is None:
                self.door_initial_open_confirmed_at = now

            settle_elapsed = now - self.door_initial_open_confirmed_at

            if settle_elapsed < self.door_initial_open_settle_sec:
                remaining = self.door_initial_open_settle_sec - settle_elapsed
                self.publish_status(
                    f'初回開扉後のコストマップ更新待ち（残り約{remaining:.1f}秒）'
                )
                return True

            self.door_wait_start_time = None
            return False

        self.door_initial_open_confirmed_at = None

        if self.door_wait_start_time is None:
            self.door_wait_start_time = now

        elapsed = now - self.door_wait_start_time

        if raw_state == 'OPEN':
            self.publish_status(
                'ドア開を安全確認中'
            )
        elif raw_state == 'CLOSED':
            self.publish_status(
                f'ドアが閉じています：開くまで待機 ({summary["closed"]}点)'
            )
        elif raw_state == 'BLOCKED':
            self.publish_status(
                'ドア開口が遮られています：待機'
            )
        else:
            self.publish_status(
                'ドア状態を確認できません：待機'
            )

        if (
            self.door_wait_timeout_sec > 0.0
            and
            elapsed >= self.door_wait_timeout_sec
        ):
            self.publish_status(
                '進入不可：ドア待機タイムアウト'
            )
            self.publish_experiment_result(
                f'NG_DOOR_TIMEOUT,0.00,0.00,{elapsed:.1f}'
            )
            self.operation_finished = True
            self.operation_phase = 'FINISHED'

        return True

    def finish_door_cycle_with_error(self, result_code, status_text, elapsed):

        self.publish_status(status_text)
        self.publish_experiment_result(
            f'{result_code},0.00,0.00,{elapsed:.1f}'
        )
        self.operation_finished = True
        self.operation_phase = 'FINISHED'

    def wait_for_door_close_after_entry(self):

        raw_state, _, confirmed_closed, summary = self.get_door_state()
        now = self.get_clock().now().nanoseconds / 1e9

        if self.door_close_wait_start_time is None:
            self.door_close_wait_start_time = now

        elapsed = now - self.door_close_wait_start_time

        if confirmed_closed:
            self.operation_phase = 'WAIT_REOPEN'
            self.door_close_wait_start_time = None
            self.door_reopen_wait_start_time = now
            self.door_reopen_confirmed_at = None
            self.exit_localization_reset_at = None
            self.exit_costmap_clear_requested_at = None
            self.exit_door_hold_sent = False
            self.exit_path_retry_due = None
            self.exit_path_retry_count = 0
            self.publish_status(
                'ドア閉を確認：次にドアが開くまで待機します'
            )
            return

        if raw_state == 'CLOSED':
            self.publish_status(
                f'ドア閉を安全確認中 ({summary["closed"]}点)'
            )
        elif raw_state == 'OPEN':
            self.publish_status(
                'エレベータ内で待機中：ドアが閉まるのを待っています'
            )
        else:
            self.publish_status(
                'エレベータ内で待機中：ドア閉を確認しています'
            )

        if (
            self.door_close_wait_timeout_sec > 0.0
            and
            elapsed >= self.door_close_wait_timeout_sec
        ):
            self.finish_door_cycle_with_error(
                'NG_DOOR_CLOSE_TIMEOUT',
                '動作中止：ドア閉待機タイムアウト',
                elapsed
            )

    def prepare_navigation_after_elevator_move(self, now):

        if (
            self.reset_amcl_from_odom_before_exit
            and
            self.use_odom_pose
        ):
            if self.exit_localization_reset_at is None:
                initial_pose = PoseWithCovarianceStamped()
                initial_pose.header.stamp = self.get_clock().now().to_msg()
                initial_pose.header.frame_id = 'map'
                initial_pose.pose.pose.position.x = self.robot_x
                initial_pose.pose.pose.position.y = self.robot_y
                initial_pose.pose.pose.orientation.z = math.sin(
                    self.robot_yaw / 2.0
                )
                initial_pose.pose.pose.orientation.w = math.cos(
                    self.robot_yaw / 2.0
                )
                initial_pose.pose.covariance[0] = 0.04
                initial_pose.pose.covariance[7] = 0.04
                initial_pose.pose.covariance[35] = 0.0685
                self.initial_pose_pub.publish(initial_pose)
                self.exit_localization_reset_at = now
                self.publish_status(
                    '階移動後のAMCL位置をオドメトリから再設定しました'
                )
                self.get_logger().info(
                    '階移動後のAMCL再初期化: '
                    f'x={self.robot_x:.2f}, y={self.robot_y:.2f}, '
                    f'yaw={self.robot_yaw:.2f}'
                )
                return False

            localization_elapsed = now - self.exit_localization_reset_at
            if localization_elapsed < self.exit_localization_settle_sec:
                remaining = (
                    self.exit_localization_settle_sec
                    - localization_elapsed
                )
                self.publish_status(
                    f'階移動後の自己位置安定待ち（残り約{remaining:.1f}秒）'
                )
                return False

        if self.clear_costmaps_before_exit:
            if self.exit_costmap_clear_requested_at is None:
                global_ready = self.global_costmap_clear_client.wait_for_service(
                    timeout_sec=0.1
                )
                local_ready = self.local_costmap_clear_client.wait_for_service(
                    timeout_sec=0.1
                )

                if not global_ready or not local_ready:
                    self.publish_status(
                        '退室前のコストマップ消去サービスを待っています'
                    )
                    return False

                self.global_costmap_clear_client.call_async(
                    ClearEntireCostmap.Request()
                )
                self.local_costmap_clear_client.call_async(
                    ClearEntireCostmap.Request()
                )
                self.exit_costmap_clear_requested_at = now
                self.publish_status(
                    '階移動中の障害物履歴を消去しました'
                )
                self.get_logger().info(
                    '退室前にglobal/local costmapを消去しました'
                )
                return False

            costmap_elapsed = now - self.exit_costmap_clear_requested_at
            if costmap_elapsed < self.exit_costmap_settle_sec:
                remaining = self.exit_costmap_settle_sec - costmap_elapsed
                self.publish_status(
                    f'退室用コストマップ更新待ち（残り約{remaining:.1f}秒）'
                )
                return False

        return True

    def wait_for_door_reopen_and_exit(self):

        raw_state, confirmed_open, _, summary = self.get_door_state()
        now = self.get_clock().now().nanoseconds / 1e9

        if self.door_reopen_wait_start_time is None:
            self.door_reopen_wait_start_time = now

        elapsed = now - self.door_reopen_wait_start_time

        if confirmed_open:
            if self.door_reopen_confirmed_at is None:
                self.door_reopen_confirmed_at = now
                self.publish_status(
                    'ドア再開を確認：コストマップの更新を待ちます'
                )

            if (
                self.hold_door_open_during_exit
                and
                not self.exit_door_hold_sent
            ):
                command = String()
                command.data = 'open'
                self.door_command_pub.publish(command)
                self.exit_door_hold_sent = True
                self.publish_status(
                    '退室完了までドアを開いたまま保持します'
                )
                self.get_logger().info(
                    '退室用の開扉保持コマンドを送信しました'
                )

            if not self.prepare_navigation_after_elevator_move(now):
                return

            settle_elapsed = now - self.door_reopen_confirmed_at

            if settle_elapsed < self.door_reopen_settle_sec:
                remaining = self.door_reopen_settle_sec - settle_elapsed
                self.publish_status(
                    f'開扉後の安全待機中（残り約{remaining:.1f}秒）'
                )
                return

            if (
                self.exit_path_retry_due is not None
                and
                now < self.exit_path_retry_due
            ):
                remaining = self.exit_path_retry_due - now
                self.publish_status(
                    f'退出経路を再確認するまで待機中（残り約{remaining:.1f}秒）'
                )
                return

            self.exit_path_retry_due = None
            self.publish_status(
                'ドア再開を確認：エレベータから退室します'
            )
            self.force_exit_elevator()
            return

        self.door_reopen_confirmed_at = None

        if raw_state == 'OPEN':
            self.publish_status(
                f'再開扉を安全確認中 ({summary["open"]}点)'
            )
        elif raw_state == 'CLOSED':
            self.publish_status(
                'エレベータ内で待機中：ドアが開くのを待っています'
            )
        else:
            self.publish_status(
                'エレベータ内で待機中：ドア状態を確認しています'
            )

        if (
            self.door_reopen_wait_timeout_sec > 0.0
            and
            elapsed >= self.door_reopen_wait_timeout_sec
        ):
            self.finish_door_cycle_with_error(
                'NG_DOOR_REOPEN_TIMEOUT',
                '動作中止：再開扉待機タイムアウト',
                elapsed
            )

    # =====================================================
    # エレベータ範囲内かどうか判定
    # =====================================================
    @staticmethod
    def is_point_in_polygon(x, y, polygon):

        inside = False
        previous_x, previous_y = polygon[-1]

        for current_x, current_y in polygon:
            # 境界上の点も内部として扱う。
            edge_x = current_x - previous_x
            edge_y = current_y - previous_y
            point_x = x - previous_x
            point_y = y - previous_y
            cross = edge_x * point_y - edge_y * point_x
            if abs(cross) <= 1e-9:
                dot = point_x * edge_x + point_y * edge_y
                edge_length_sq = edge_x * edge_x + edge_y * edge_y
                if 0.0 <= dot <= edge_length_sq:
                    return True

            if (current_y > y) != (previous_y > y):
                intersection_x = (
                    previous_x
                    + (y - previous_y)
                    * (current_x - previous_x)
                    / (current_y - previous_y)
                )
                if x < intersection_x:
                    inside = not inside

            previous_x, previous_y = current_x, current_y

        return inside

    def is_inside_elevator(self, x, y):

        if self.use_elevator_polygon:
            return self.is_point_in_polygon(x, y, self.elevator_polygon)

        if self.use_door_based_elevator:
            local_x, local_y = self.world_to_elevator_local(x, y)

            return (
                self.elevator_local_x_min <= local_x <= self.elevator_local_x_max
                and
                self.elevator_local_y_min <= local_y <= self.elevator_local_y_max
            )

        return (
            self.elevator_x_min <= x <= self.elevator_x_max
            and
            self.elevator_y_min <= y <= self.elevator_y_max
        )

    # =====================================================
    # エレベータ内部のドア前禁止領域かどうか判定する
    # =====================================================
    def is_in_door_front_area(self, x, y):

        if not self.use_door_front_forbidden:
            return False

        if self.use_door_based_elevator:
            local_x, local_y = self.world_to_elevator_local(x, y)
            cell_half = self.grid_step * 0.5

            return (
                self.door_front_local_x_min - cell_half
                <= local_x <=
                self.door_front_local_x_max + cell_half
                and
                self.door_front_local_y_min - cell_half
                <= local_y <=
                self.door_front_local_y_max + cell_half
            )

        cell_half = self.grid_step * 0.5

        return (
            self.door_front_x_min - cell_half
            <= x <=
            self.door_front_x_max + cell_half
            and
            self.door_front_y_min - cell_half
            <= y <=
            self.door_front_y_max + cell_half
        )

    # =====================================================
    # LiDARから動的障害物と未観測領域の元になる影を取り出す
    # =====================================================
    def get_dynamic_obstacles(self):
        if getattr(self,'observation_guard_enabled',False):
            snapshot=self.get_observation()
            if 'evidence' not in snapshot or snapshot['evidence'].reason!='OK':
                raise ValueError('OBSERVATION_UNAVAILABLE '+snapshot['detail']['reason'])
            evidence=snapshot['evidence']
            self.candidate_scan_msg = evidence.scan
            self.scan_origin_x,self.scan_origin_y,self.scan_origin_yaw=evidence.origin
            obstacles=[];shadows=[]
            for i in np.flatnonzero(evidence.finite):
                r=float(evidence.ranges[i]);a=float(evidence.angles[i])
                x=evidence.origin[0]+r*math.cos(evidence.origin[2]+a)
                y=evidence.origin[1]+r*math.sin(evidence.origin[2]+a)
                if not self.is_inside_elevator(x,y) or self.is_near_wall_on_static_map(x,y,.15):continue
                obstacles.append((float(x),float(y)))
                if self.use_unknown_shadow:
                    shadows.append(dict(hit_x=float(x),hit_y=float(y),hit_range=r,scan_angle=a,range_max=evidence.scan.range_max))
            self.publish_observation(snapshot['detail'])
            obstacles.extend(self.get_hokuyo_obstacles())
            return obstacles,shadows

        obstacles = []
        unknown_shadows = []

        msg = self.latest_scan
        self.candidate_scan_msg = msg

        for i, r in enumerate(msg.ranges):

            if math.isinf(r) or math.isnan(r):
                continue

            if r < msg.range_min:
                continue

            if r > msg.range_max:
                continue

            angle = (
                msg.angle_min
                +
                i * msg.angle_increment
            )

            global_angle = (
                self.robot_yaw + angle
            )

            hit_x = (
                self.robot_x
                +
                r * math.cos(global_angle)
            )

            hit_y = (
                self.robot_y
                +
                r * math.sin(global_angle)
            )

            # エレベータ範囲外は無視
            if not self.is_inside_elevator(hit_x, hit_y):
                continue

            # 静的地図上の壁は無視
            if self.is_near_wall_on_static_map(hit_x, hit_y,0.15):
                continue

            # 地図にはないがLiDARに映ったものを動的障害物とする
            obstacles.append((hit_x, hit_y))

            # その障害物の奥側はLiDARで見えていない可能性があるため、
            # 未観測危険領域の候補として保存する
            if self.use_unknown_shadow:
                unknown_shadows.append({
                    'hit_x': hit_x,
                    'hit_y': hit_y,
                    'hit_range': r,
                    'scan_angle': angle,
                    'range_max': msg.range_max
                })

        obstacles.extend(self.get_hokuyo_obstacles())
        return obstacles, unknown_shadows

    # =====================================================
    # ドア付近に降車者らしい動的障害物がいるか調べる
    # =====================================================
    def count_doorway_dynamic_points(self):

        count, _, _ = self.get_doorway_dynamic_summary()

        return count

    def get_doorway_dynamic_summary(self):

        if not self.use_exiting_person_wait:
            return 0, None, None

        if not self.use_door_based_elevator:
            return 0, None, None

        if self.latest_scan is None:
            return 0, None, None

        if self.robot_x is None or self.robot_y is None or self.robot_yaw is None:
            return 0, None, None

        count = 0
        sum_x = 0.0
        sum_y = 0.0
        msg = self.latest_scan

        for i, r in enumerate(msg.ranges):

            if math.isinf(r) or math.isnan(r):
                continue

            if r < msg.range_min:
                continue

            if r > msg.range_max:
                continue

            angle = (
                msg.angle_min
                +
                i * msg.angle_increment
            )

            global_angle = self.robot_yaw + angle

            hit_x = (
                self.robot_x
                +
                r * math.cos(global_angle)
            )

            hit_y = (
                self.robot_y
                +
                r * math.sin(global_angle)
            )

            local_x, local_y = self.world_to_elevator_local(hit_x, hit_y)

            if not (
                self.exiting_person_local_x_min <= local_x <= self.exiting_person_local_x_max
                and
                self.exiting_person_local_y_min <= local_y <= self.exiting_person_local_y_max
            ):
                continue

            if self.is_near_wall_on_static_map(hit_x, hit_y, 0.15):
                continue

            count += 1
            sum_x += hit_x
            sum_y += hit_y

        if count == 0:
            return 0, None, None

        return count, sum_x / count, sum_y / count

    def should_wait_for_exiting_person(self):

        if not self.use_exiting_person_wait:
            return False

        count, center_x, center_y = self.get_doorway_dynamic_summary()
        now = self.get_clock().now().nanoseconds / 1e9

        if count >= self.exiting_person_min_points:

            if self.exiting_person_wait_start_time is None:
                self.exiting_person_wait_start_time = now

            self.exiting_person_clear_start_time = None

            if self.last_exiting_person_center is None:
                self.last_exiting_person_center = (center_x, center_y)
                self.exiting_person_stationary_start_time = now
            else:
                last_x, last_y = self.last_exiting_person_center
                movement = math.hypot(
                    center_x - last_x,
                    center_y - last_y
                )

                if movement >= self.exiting_person_motion_threshold:
                    self.last_exiting_person_center = (center_x, center_y)
                    self.exiting_person_stationary_start_time = now

            stationary_elapsed = 0.0

            if self.exiting_person_stationary_start_time is not None:
                stationary_elapsed = now - self.exiting_person_stationary_start_time

            if stationary_elapsed >= self.exiting_person_stationary_time_sec:
                self.publish_status(
                    f'ドア付近の障害物は静止中：入室判定を継続 ({count}点)'
                )

                self.get_logger().info(
                    f'doorway obstacle stationary: points={count}, '
                    f'stationary={stationary_elapsed:.1f}s'
                )

                return False

            elapsed = now - self.exiting_person_wait_start_time

            self.publish_status(
                f'降車者待機中：ドア付近に{count}点検出'
            )

            self.get_logger().info(
                f'waiting for exiting person: points={count}, elapsed={elapsed:.1f}s'
            )

            if elapsed >= self.exiting_person_wait_timeout_sec:
                if self.skip_boarding('DOORWAY_OCCUPIED_TIMEOUT'):
                    return True
                self.publish_status(
                    '進入不可：降車者待機タイムアウト'
                )

                self.publish_experiment_result(
                    f'NG_EXITING_PERSON_TIMEOUT,0.00,0.00,{elapsed:.1f}'
                )

                self.operation_finished = True
                self.operation_phase = 'FINISHED'

            return True

        if self.exiting_person_wait_start_time is None:
            return False

        if self.exiting_person_clear_start_time is None:
            self.exiting_person_clear_start_time = now

        clear_elapsed = now - self.exiting_person_clear_start_time

        if clear_elapsed < self.exiting_person_clear_time_sec:
            self.publish_status(
                f'降車者通過後の安全確認中：{clear_elapsed:.1f}s'
            )
            return True

        self.exiting_person_wait_start_time = None
        self.exiting_person_clear_start_time = None
        self.exiting_person_stationary_start_time = None
        self.last_exiting_person_center = None

        self.publish_status(
            '降車者なし：入室判定を再開'
        )

        return False
    
    def is_near_wall_on_static_map(self, x, y, radius=0.15):

        offsets = [
            (0.0, 0.0),
            (radius, 0.0),
            (-radius, 0.0),
            (0.0, radius),
            (0.0, -radius),
            (radius, radius),
            (radius, -radius),
            (-radius, radius),
            (-radius, -radius),
        ]

        wall_hits = 0
        total_checks = 0

        for dx, dy in offsets:
            total_checks += 1
            if self.is_wall_like_on_static_map(x + dx, y + dy, radius=0.05):
                wall_hits += 1

        return wall_hits >= 1

    # =====================================================
    # 角度差を -pi 〜 pi に正規化する
    # =====================================================
    def normalize_angle(self, angle):

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    # =====================================================
    # 候補点がLiDARで本当に見えていないか判定する
    #
    # 以前の方式：
    #   障害物に当たったレーザーの奥側をまとめて未観測扱い
    #
    # 改善後の方式：
    #   候補点方向のLiDAR実測距離を確認し、
    #   候補点が実測距離より奥にある場合だけ未観測扱いする
    # =====================================================
    def is_in_unknown_shadow(self, x, y, unknown_shadows):

        if not self.use_unknown_shadow:
            return False

        msg = self.latest_scan

        if msg is None:
            return False

        if self.robot_x is None or self.robot_y is None or self.robot_yaw is None:
            return False

        if (getattr(self, 'comparison_method', 'legacy') != 'legacy'
                and getattr(self, 'comparison_shadow_geometry', 'directed_axial_v1') == 'directed_axial_v1'):
            origin=(getattr(self, 'scan_origin_x', self.robot_x),
                    getattr(self, 'scan_origin_y', self.robot_y),
                    getattr(self, 'scan_origin_yaw', self.robot_yaw))
            frame=getattr(self, '_comparison_shadow_frame', None)
            prepared=frame[2] if frame and frame[0] is unknown_shadows and frame[1]==origin else None
            return directed_shadow(x, y, origin, unknown_shadows, self.unknown_start_margin,
                self.unknown_shadow_depth, self.unknown_shadow_width, prepared)

        dx = x - getattr(self,'scan_origin_x',self.robot_x)
        dy = y - getattr(self,'scan_origin_y',self.robot_y)

        point_range = math.hypot(dx, dy)

        if point_range <= 0.0:
            return False

        if not unknown_shadows:
            return False

        # 候補点のmap座標での角度
        global_angle = math.atan2(dy, dx)

        # LiDAR座標系での角度に変換
        scan_angle = self.normalize_angle(
            global_angle - getattr(self,'scan_origin_yaw',self.robot_yaw)
        )

        for shadow in unknown_shadows:
            hit_range = shadow.get('hit_range')
            hit_angle = shadow.get('scan_angle')

            if hit_range is None or hit_angle is None:
                continue

            angle_error = abs(
                self.normalize_angle(
                    scan_angle - hit_angle
                )
            )

            lateral_error = point_range * math.sin(angle_error)

            if abs(lateral_error) > self.unknown_shadow_width:
                continue

            if point_range <= hit_range + self.unknown_start_margin:
                continue

            if point_range > hit_range + self.unknown_shadow_depth:
                continue

            return True

        return False

    # =====================================================
    # ある点から一番近い障害物までの距離
    # =====================================================
    def distance_to_nearest_obstacle(self, x, y, obstacles):

        if len(obstacles) == 0:
            return 999.0

        min_dist = 999.0

        for ox, oy in obstacles:

            d = math.hypot(
                x - ox,
                y - oy
            )

            min_dist = min(min_dist, d)

        return min_dist

    # =====================================================
    # ゴール姿勢での非対称長方形footprintを扱う
    # =====================================================
    def goal_footprint_yaw(self):
        fixed=getattr(self,'entry_fixed_local_pose',[])
        if fixed and self.operation_phase not in ('EXITING','WAIT_REOPEN','ARRIVAL_HOLD'):
            return math.atan2(self.door_axis_y,self.door_axis_x)+fixed[2]

        if self.use_door_based_elevator:
            return math.atan2(self.inside_axis_y, self.inside_axis_x)

        if self.robot_yaw is not None:
            return self.robot_yaw

        return 0.0

    def robot_footprint_corners(self, x, y, yaw):

        local_corners = [
            (self.robot_front_length, self.robot_left_width),
            (self.robot_front_length, -self.robot_right_width),
            (-self.robot_rear_length, -self.robot_right_width),
            (-self.robot_rear_length, self.robot_left_width),
        ]
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        corners = []

        for forward, lateral in local_corners:
            corners.append((
                x + cos_yaw * forward - sin_yaw * lateral,
                y + sin_yaw * forward + cos_yaw * lateral,
            ))

        return corners

    @staticmethod
    def sampled_axis_values(min_value, max_value, step):

        span = max_value - min_value
        count = max(1, int(math.ceil(span / step)))
        return [
            min_value + span * index / count
            for index in range(count + 1)
        ]

    def sample_robot_footprint(self, x, y, yaw):

        step = max(0.01, float(self.footprint_sample_step))
        forward_values = self.sampled_axis_values(
            -self.robot_rear_length,
            self.robot_front_length,
            step,
        )
        lateral_values = self.sampled_axis_values(
            -self.robot_right_width,
            self.robot_left_width,
            step,
        )
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        for forward in forward_values:
            for lateral in lateral_values:
                yield (
                    x + cos_yaw * forward - sin_yaw * lateral,
                    y + sin_yaw * forward + cos_yaw * lateral,
                )

    def distance_from_obstacle_to_footprint(self, x, y, yaw, ox, oy):

        dx = ox - x
        dy = oy - y
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        obstacle_forward = cos_yaw * dx + sin_yaw * dy
        obstacle_lateral = -sin_yaw * dx + cos_yaw * dy

        forward_gap = max(
            -self.robot_rear_length - obstacle_forward,
            0.0,
            obstacle_forward - self.robot_front_length,
        )
        lateral_gap = max(
            -self.robot_right_width - obstacle_lateral,
            0.0,
            obstacle_lateral - self.robot_left_width,
        )

        return math.hypot(forward_gap, lateral_gap)

    def is_robot_footprint_safe(
        self,
        x,
        y,
        yaw,
        obstacles,
        unknown_shadows,
    ):

        self.last_footprint_rejection = None
        # 車体の頂点・辺・内部を調べるため、中心だけが安全でも車体が
        # 壁またはunknownへ入る候補を除外できる。
        for sample_x, sample_y in self.sample_robot_footprint(x, y, yaw):
            if (
                self.use_elevator_polygon
                and not self.is_inside_elevator(sample_x, sample_y)
            ):
                self.last_footprint_rejection = dict(reason='footprint_outside_cabin', point=[sample_x, sample_y])
                return False

            if self.is_wall_on_static_map(sample_x, sample_y):
                self.last_footprint_rejection = dict(reason='footprint_static_wall_margin', point=[sample_x, sample_y])
                return False

            if self.is_in_unknown_shadow(
                sample_x,
                sample_y,
                unknown_shadows,
            ):
                self.last_footprint_rejection = dict(reason='footprint_unknown_shadow', point=[sample_x, sample_y])
                return False

        # 動的障害物点から回転長方形までの最短距離を使用する。
        for obstacle_x, obstacle_y in obstacles:
            distance = self.distance_from_obstacle_to_footprint(
                x,
                y,
                yaw,
                obstacle_x,
                obstacle_y,
            )
            if distance < self.safe_distance:
                self.last_footprint_rejection = dict(reason='footprint_obstacle_clearance', point=[obstacle_x, obstacle_y], distance=distance)
                return False

        return True

    # =====================================================
    # 入室後に退室しやすい経路が残っているか判定する
    # 候補点からドア中央までの直線上に壁・人・未観測領域があれば避ける
    # =====================================================
    def is_exit_path_clear(self, x, y, obstacles, unknown_shadows):

        self.last_exit_rejection = None
        if not self.use_exit_path_check:
            return True

        if self.use_door_based_elevator:
            door_x = self.door_center_x
            door_y = self.door_center_y
        else:
            door_x = (
                self.door_front_x_min
                +
                self.door_front_x_max
            ) / 2.0

            door_y = self.door_front_y_min

        dx = door_x - x
        dy = door_y - y
        distance = math.hypot(dx, dy)

        if distance <= 0.0:
            return True

        steps = max(
            1,
            int(distance / self.exit_path_sample_step)
        )

        for i in range(1, steps + 1):

            ratio = i / steps

            check_x = x + dx * ratio
            check_y = y + dy * ratio

            if self.is_wall_on_static_map(check_x, check_y):
                self.last_exit_rejection = dict(reason='exit_center_static_wall', point=[check_x, check_y])
                return False

            if self.is_in_unknown_shadow(check_x, check_y, unknown_shadows):
                self.last_exit_rejection = dict(reason='exit_center_unknown_shadow', point=[check_x, check_y])
                return False

            dist_obs = self.distance_to_nearest_obstacle(
                check_x,
                check_y,
                obstacles
            )

            if dist_obs < self.exit_path_clearance:
                self.last_exit_rejection = dict(reason='exit_center_obstacle_clearance', point=[check_x, check_y])
                return False

        return True

    # =====================================================
    # test
    # =====================================================
    def is_narrow_between_obstacles(self, x, y, obstacles):

        near_obstacles = []

        check_radius = 0.55

        for ox, oy in obstacles:

            dx = ox - x
            dy = oy - y
            dist = math.hypot(dx, dy)

            if dist <= check_radius:
                angle = math.atan2(dy, dx)
                near_obstacles.append(angle)

        if len(near_obstacles) < 2:
            return False

        for i in range(len(near_obstacles)):
            for j in range(i + 1, len(near_obstacles)):

                diff = abs(
                    self.normalize_angle(
                        near_obstacles[i] - near_obstacles[j]
                    )
                )

                # 反対方向に障害物があるなら挟まれている
                if math.radians(120) <= diff <= math.radians(180):
                    return True

        return False
# =====================================================
# 評価値マップを表示する
# =====================================================
    def print_score_grid(self, score_cells, best_goal):

        if len(score_cells) == 0:
            return

        cell_dict = {}
        x_values = []
        y_values = []

        for cell in score_cells:

            x = round(cell['x'], 2)
            y = round(cell['y'], 2)

            cell_dict[(x, y)] = cell

            if x not in x_values:
                x_values.append(x)

            if y not in y_values:
                y_values.append(y)

        x_values.sort()
        y_values.sort(reverse=True)

        best_x = None
        best_y = None

        if best_goal is not None:
            best_x = round(best_goal[0], 2)
            best_y = round(best_goal[1], 2)

        print()
        print('===== 評価値マップ =====')
        print('凡例: 数値=評価値, 壁=静的地図上の壁, ド=ドア前禁止領域, 危=障害物に近い, 未=未観測危険領域, ★=選択ゴール')

    @motion_locked
    def clear_candidate_evaluation(self, reason):
        """Replace transient-local results with an explicit empty observation."""
        now = self.get_clock().now()
        self.area_stats = None
        self.candidate_diagnostics = dict(
            schema=1, valid=False, invalidation_reason=reason,
            cleared_at=now.nanoseconds / 1e9, evaluated_at=None,
            session_id=self.sweep_owner_id, motion_epoch=self.motion_epoch,
            cells=[], best_goal=None, area_stats=None)
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = now.to_msg()
        marker.action = Marker.DELETEALL
        self.marker_pub.publish(MarkerArray(markers=[marker]))
        if hasattr(self, 'evidence_marker_pub'):
            self.evidence_marker_pub.publish(MarkerArray(markers=[marker]))
        self.candidate_diagnostics_pub.publish(String(data=json.dumps(self.candidate_diagnostics)))

    def publish_candidate_evidence(self, cells, obstacles, shadows, evaluated_at):
        """Display ray evidence separately from the legacy selection predicates."""
        scan = getattr(self, 'candidate_scan_msg', self.latest_scan)
        origin = [getattr(self, 'scan_origin_x', self.robot_x),
                  getattr(self, 'scan_origin_y', self.robot_y),
                  getattr(self, 'scan_origin_yaw', self.robot_yaw)]
        stamp = scan.header.stamp.sec + scan.header.stamp.nanosec / 1e9
        status = None
        quality = None
        try:
            evidence = ScanEvidence(scan, origin, inf_is_free=False, min_valid=0,
                                    min_fraction=0.)
            status = evidence.coverage([(c['x'], c['y']) for c in cells], margin=.05)
            quality = evidence.quality
        except ValueError as exc:
            quality = {'error': str(exc)}
        names = {1: 'observed_free_diagnostic_only', 0: 'unconfirmed_diagnostic_only',
                 -1: 'return_before_cell_diagnostic_only'}
        shadow_name = 'shadow_conflict_diagnostic_only'
        counts = {name: 0 for name in names.values()}
        counts[shadow_name] = 0
        array = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        groups = {name: [] for name in names.values()}
        groups[shadow_name] = []
        if status is not None:
            for cell, value in zip(cells, status):
                name = names[int(value)]
                if value == 1 and self.is_in_unknown_shadow(cell['x'], cell['y'], shadows):
                    # Nearest-beam evidence can disagree with the wider legacy
                    # shadow envelope. Never draw that envelope as known free.
                    name = shadow_name
                cell['ray_evidence'] = name
                groups[name].append((cell['x'], cell['y']))
                counts[name] += 1
        def points_marker(name, positions, color, size, marker_id):
            if not positions:
                return
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = name
            marker.id = marker_id
            marker.type = Marker.POINTS
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = size
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
            for x, y in positions:
                point = Point()
                point.x, point.y, point.z = float(x), float(y), .09
                marker.points.append(point)
            array.markers.append(marker)
        points_marker('used_obstacle_returns', obstacles, (1., .1, .75, 1.), .035, 1)
        points_marker('observed_free_diagnostic_only', groups[names[1]],
                      (0., .8, .5, .8), .018, 2)
        points_marker('unconfirmed_diagnostic_only', groups[names[0]],
                      (.2, .3, 1., .9), .018, 3)
        points_marker('return_before_cell_diagnostic_only', groups[names[-1]],
                      (1., .5, .1, .9), .018, 4)
        points_marker(shadow_name, groups[shadow_name],
                      (.8, .4, 1., .9), .018, 5)
        self.evidence_marker_pub.publish(array)
        return dict(scan_stamp=stamp, scan_frame=scan.header.frame_id,
                    decision_stamp=evaluated_at, sensor_origin=origin,
                    classification='diagnostic_only_not_used_by_legacy_selector',
                    ray_margin_m=.05, ray_quality=quality, ray_counts=counts,
                    used_obstacle_points=[list(p) for p in obstacles],
                    used_hokuyo_obstacle_points=[list(p) for p in
                                                 getattr(self, 'last_hokuyo_obstacle_points', [])],
                    hokuyo_scan_stamp=(self.latest_hokuyo_scan.header.stamp.sec +
                        self.latest_hokuyo_scan.header.stamp.nanosec / 1e9
                        if getattr(self, 'latest_hokuyo_scan', None) else None),
                    used_shadow_rays=shadows,
                    first_rejection_only=True)

    def publish_score_markers(self, score_cells, best_goal):

        marker_array = MarkerArray()

        marker_yaw = 0.0

        if self.use_door_based_elevator:
            marker_yaw = math.atan2(
                self.door_axis_y,
                self.door_axis_x
            )

        marker_orientation_z = math.sin(marker_yaw / 2.0)
        marker_orientation_w = math.cos(marker_yaw / 2.0)

        delete_marker = Marker()
        delete_marker.header.frame_id = 'map'
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.ns = 'elevator_score'
        delete_marker.id = 0
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        marker_id = 1

        for cell in score_cells:

            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'elevator_score'
            marker.id = marker_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            marker.pose.position.x = cell['x']
            marker.pose.position.y = cell['y']
            marker.pose.position.z = 0.03

            marker.pose.orientation.z = marker_orientation_z
            marker.pose.orientation.w = marker_orientation_w

            marker.scale.x = self.grid_step
            marker.scale.y = self.grid_step
            marker.scale.z = 0.03

            marker.color.a = 0.6

            if cell['type'] == 'wall':
                marker.color.r = 0.4
                marker.color.g = 0.4
                marker.color.b = 0.4

            elif cell['type'] == 'door_front':
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0

            elif cell['type'] == 'danger':
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0

            elif cell['type'] == 'exit_blocked':
                marker.color.r = 1.0
                marker.color.g = 0.45
                marker.color.b = 0.0

            elif cell['type'] == 'unknown':
                marker.color.r = 0.6
                marker.color.g = 0.0
                marker.color.b = 1.0

            elif cell['type'] == 'free':
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0

            else:
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0

            marker_array.markers.append(marker)
            marker_id += 1

        # 緑色のセルは「安全に停止できる候補」であり、かご全体ではない。
        # 認識しているかごの実寸範囲は、別の水色枠で常時表示する。
        if self.use_door_based_elevator:
            cabin_outline = Marker()
            cabin_outline.header.frame_id = 'map'
            cabin_outline.header.stamp = self.get_clock().now().to_msg()
            cabin_outline.ns = 'elevator_recognition_area'
            cabin_outline.id = marker_id
            cabin_outline.type = Marker.LINE_STRIP
            cabin_outline.action = Marker.ADD
            cabin_outline.pose.orientation.w = 1.0
            cabin_outline.scale.x = 0.035
            cabin_outline.color.a = 1.0
            cabin_outline.color.r = 0.0
            cabin_outline.color.g = 0.9
            cabin_outline.color.b = 1.0
            if self.use_elevator_polygon:
                corners = self.elevator_polygon + [self.elevator_polygon[0]]
            else:
                local_corners = [
                    (self.elevator_local_x_min, self.elevator_local_y_min),
                    (self.elevator_local_x_max, self.elevator_local_y_min),
                    (self.elevator_local_x_max, self.elevator_local_y_max),
                    (self.elevator_local_x_min, self.elevator_local_y_max),
                    (self.elevator_local_x_min, self.elevator_local_y_min),
                ]
                corners = [
                    self.elevator_local_to_world(local_x, local_y)
                    for local_x, local_y in local_corners
                ]
            for world_x, world_y in corners:
                point = Point()
                point.x = world_x
                point.y = world_y
                point.z = 0.08
                cabin_outline.points.append(point)
            marker_array.markers.append(cabin_outline)
            marker_id += 1

        if best_goal is not None:

            goal_marker = Marker()
            goal_marker.header.frame_id = 'map'
            goal_marker.header.stamp = self.get_clock().now().to_msg()
            goal_marker.ns = 'elevator_score'
            goal_marker.id = marker_id
            goal_marker.type = Marker.SPHERE
            goal_marker.action = Marker.ADD

            goal_marker.pose.position.x = best_goal[0]
            goal_marker.pose.position.y = best_goal[1]
            goal_marker.pose.position.z = 0.20
            goal_marker.pose.orientation.w = 1.0

            goal_marker.scale.x = 0.20
            goal_marker.scale.y = 0.20
            goal_marker.scale.z = 0.20

            goal_marker.color.a = 1.0
            goal_marker.color.r = 0.0
            goal_marker.color.g = 0.0
            goal_marker.color.b = 1.0

            marker_array.markers.append(goal_marker)
            marker_id += 1

            # 選択ゴールへ奥向きに配置したときの車体外形を表示する。
            footprint_marker = Marker()
            footprint_marker.header.frame_id = 'map'
            footprint_marker.header.stamp = self.get_clock().now().to_msg()
            footprint_marker.ns = 'elevator_goal_footprint'
            footprint_marker.id = marker_id
            footprint_marker.type = Marker.LINE_STRIP
            footprint_marker.action = Marker.ADD
            footprint_marker.pose.orientation.w = 1.0
            footprint_marker.scale.x = 0.05
            footprint_marker.color.a = 1.0
            footprint_marker.color.r = 0.1
            footprint_marker.color.g = 0.4
            footprint_marker.color.b = 1.0

            footprint_corners = self.robot_footprint_corners(
                best_goal[0],
                best_goal[1],
                self.goal_footprint_yaw(),
            )
            for corner_x, corner_y in footprint_corners + footprint_corners[:1]:
                point = Point()
                point.x = corner_x
                point.y = corner_y
                point.z = 0.12
                footprint_marker.points.append(point)

            marker_array.markers.append(footprint_marker)

        self.marker_pub.publish(marker_array)

        self.get_logger().info(
            f'MarkerArray publish: {len(marker_array.markers)} markers'
        )

        
    # =====================================================
    # 一番よい空き場所を探す
    # =====================================================
    def find_best_goal(self, obstacles, unknown_shadows):

        if getattr(self, 'comparison_method', 'legacy') != 'legacy':
            origin=(getattr(self, 'scan_origin_x', self.robot_x),
                    getattr(self, 'scan_origin_y', self.robot_y),
                    getattr(self, 'scan_origin_yaw', self.robot_yaw))
            self._comparison_shadow_frame=(unknown_shadows,origin,prepare_shadows(origin,unknown_shadows))

        best_goal = None
        best_score = -999999.0

        score_cells = []

        # 空き空間計算用
        total_count = 0
        free_count = 0
        wall_count = 0
        door_front_count = 0
        danger_count = 0
        unknown_count = 0

        if self.use_door_based_elevator:
            center_x = 0.0
            scan_x_min = self.elevator_local_x_min
            scan_x_max = self.elevator_local_x_max
            scan_y_min = self.elevator_local_y_min
            scan_y_max = self.elevator_local_y_max
        else:
            center_x = (
                self.elevator_x_min
                +
                self.elevator_x_max
            ) / 2.0
            scan_x_min = self.elevator_x_min
            scan_x_max = self.elevator_x_max
            scan_y_min = self.elevator_y_min
            scan_y_max = self.elevator_y_max

        footprint_yaw = self.goal_footprint_yaw()

        fixed=getattr(self,'entry_fixed_local_pose',[])
        if fixed:
            scan_x_min=scan_x_max=fixed[0]
            scan_y_min=scan_y_max=fixed[1]

        scan_x = scan_x_min

        while scan_x <= scan_x_max:

            scan_y = scan_y_min

            while scan_y <= scan_y_max:

                if self.use_door_based_elevator:
                    x, y = self.elevator_local_to_world(scan_x, scan_y)
                    local_x = scan_x
                    local_y = scan_y
                else:
                    x = scan_x
                    y = scan_y
                    local_x = x
                    local_y = y

                if not self.is_inside_elevator(x, y):
                    scan_y += self.grid_step
                    continue

                total_count += 1

                if getattr(self, 'comparison_method', 'legacy') != 'legacy':
                    cell = evaluate_cell(self, x, y, obstacles, unknown_shadows)
                    score_cells.append(cell)
                    if cell['type'] == 'free':
                        free_count += 1
                        dist_obs = self.distance_to_nearest_obstacle(x, y, obstacles)
                        score = min(dist_obs, 1.0) * 10.0 - abs(local_x - center_x) * 3.0 + local_y * 2.0
                        cell['score'] = score
                        if score > best_score:
                            best_score, best_goal = score, (x, y, score)
                    elif cell['type'] == 'wall':
                        wall_count += 1
                    elif cell['type'] == 'door_front':
                        door_front_count += 1
                    elif cell['type'] == 'unknown':
                        unknown_count += 1
                    else:
                        danger_count += 1
                    scan_y += self.grid_step
                    continue

                # かごの床全体を表示しつつ、壁とBurgerの安全余白内は
                # 停止候補から外す。
                if self.is_too_close_to_static_wall(x, y):
                    wall_count += 1

                    score_cells.append({
                        'x': x,
                        'y': y,
                        'type': 'wall',
                        'reason': 'center_static_wall_clearance',
                        'score': None
                    })

                    scan_y += self.grid_step
                    continue

                # エレベータ内部のドア前は、停車すると危険なので候補から外す
                if self.is_in_door_front_area(x, y):
                    door_front_count += 1

                    score_cells.append({
                        'x': x,
                        'y': y,
                        'type': 'door_front',
                        'reason': 'door_front_stop_prohibited',
                        'score': None
                    })

                    scan_y += self.grid_step
                    continue

                # LiDARで障害物の奥に隠れて見えていない場所は候補から外す
                if self.is_in_unknown_shadow(x, y, unknown_shadows):
                    unknown_count += 1

                    score_cells.append({
                        'x': x,
                        'y': y,
                        'type': 'unknown',
                        'reason': 'center_unknown_shadow',
                        'score': None
                    })

                    scan_y += self.grid_step
                    continue

                # 障害物との距離を計算
                dist_obs = self.distance_to_nearest_obstacle(
                    x,
                    y,
                    obstacles
                )

                # 障害物に近すぎるなら候補から外す
                if dist_obs < self.safe_distance:    

                    danger_count += 1

                    score_cells.append({
                        'x': x,
                        'y': y,
                        'type': 'danger',
                        'reason': 'center_obstacle_clearance',
                        'score': None
                    })

                    scan_y += self.grid_step
                    continue

                # 候補点を基準に車体全体を奥向きに配置し、中心点では
                # 見落とす前後端・側面の壁、人、unknown干渉を除外する。
                if not self.is_robot_footprint_safe(
                    x,
                    y,
                    footprint_yaw,
                    obstacles,
                    unknown_shadows,
                ):
                    danger_count += 1
                    score_cells.append({
                        'x': x,
                        'y': y,
                        'type': 'danger',
                        'reason': self.last_footprint_rejection['reason'],
                        'score': None
                    })
                    scan_y += self.grid_step
                    continue
                
                if self.is_narrow_between_obstacles(x, y, obstacles):

                    danger_count += 1

                    score_cells.append({
                        'x': x,
                        'y': y,
                        'type': 'danger',
                        'reason': 'narrow_between_obstacles',
                        'score': None
                    })

                    scan_y += self.grid_step
                    continue

                if not self.is_exit_path_clear(x, y, obstacles, unknown_shadows):

                    danger_count += 1

                    score_cells.append({
                        'x': x,
                        'y': y,
                        'type': 'exit_blocked',
                        'reason': self.last_exit_rejection['reason'],
                        'score': None
                    })

                    scan_y += self.grid_step
                    continue

                # ここまで来た点は安全な空き場所
                free_count += 1

                # ==========================
                # ゴール候補の点数計算
                # ==========================

                # 障害物から遠いほど高得点。ただし遠すぎる値で他の評価を潰さない。
                obstacle_score = min(dist_obs, 1.0) * 10.0

                # エレベータ中央に近いほど高得点
                center_score = -abs(local_x - center_x) * 3.0

                # 奥側にあるほど高得点
                depth_score = local_y * 2.0

                score = (
                    obstacle_score
                    +
                    center_score
                    +
                    depth_score
                )

                score_cells.append({
                    'x': x,
                    'y': y,
                    'type': 'free',
                    'reason': 'candidate',
                    'score': score
                })

                if score > best_score:
                    best_score = score
                    best_goal = (
                        x,
                        y,
                        score
                    )

                scan_y += self.grid_step

            scan_x += self.grid_step

        # ==========================
        # 空き空間の面積を計算
        # ==========================
        one_cell_area = self.grid_step * self.grid_step
        free_area = free_count * one_cell_area
        total_area = total_count * one_cell_area
        wall_area = wall_count * one_cell_area
        door_front_area = door_front_count * one_cell_area
        danger_area = danger_count * one_cell_area
        unknown_area = unknown_count * one_cell_area

        if total_area > 0.0:
            free_rate = free_area / total_area * 100.0
            danger_rate = danger_area / total_area * 100.0
            unknown_rate = unknown_area / total_area * 100.0
        else:
            free_rate = 0.0
            danger_rate = 0.0
            unknown_rate = 0.0
        # Show why candidate cells were rejected even when the search cannot
        # continue. A single transient-local sample also reaches RViz opened
        # after the search.
        self.print_score_grid(score_cells, best_goal)

        # Keep the latest completed evaluation available to monitors, including
        # searches rejected by the area thresholds below.
        self.area_stats = {
            'evaluated_at': self.get_clock().now().nanoseconds / 1e9,
            'total_area': total_area,
            'free_area': free_area,
            'danger_area': danger_area,
            'unknown_area': unknown_area,
            'wall_area': wall_area,
            'door_front_area': door_front_area,
            'free_rate': free_rate,
            'danger_rate': danger_rate,
            'unknown_rate': unknown_rate,
            'wall_rate': wall_area / total_area * 100.0 if total_area > 0.0 else 0.0,
            'door_front_rate': door_front_area / total_area * 100.0 if total_area > 0.0 else 0.0,
            'free_count': free_count,
            'total_count': total_count,
        }
        self.entry_area_rejection = area_gate_reasons(
            self.area_stats, {k: getattr(self, k, v) for k, v in AREA_DEFAULTS.items()})
        self.publish_score_markers(score_cells,
                                   None if self.entry_area_rejection else best_goal)
        evidence_detail = (self.publish_candidate_evidence(
            score_cells, obstacles, unknown_shadows, self.area_stats['evaluated_at'])
            if hasattr(self, 'evidence_marker_pub') else None)
        for cell in score_cells:
            cell['yaw'] = footprint_yaw
        # Diagnostic only. Nav2 inflation is not an input to this selector.
        # Retain the original marker types and area accounting for consumers.
        self.candidate_diagnostics = dict(
            schema=1, valid=True, evaluated_at=self.area_stats['evaluated_at'],
            area_stats=dict(self.area_stats),
            session_id=getattr(self, 'sweep_owner_id', None),
            motion_epoch=getattr(self, 'motion_epoch', None),
            cells=score_cells, best_goal=best_goal,
            selected_goal=None if self.entry_area_rejection else best_goal,
            selection_stage='before_nav2_path_check',
            evidence=evidence_detail,
            effective=dict((name, getattr(self, name)) for name in (
                'static_wall_clearance', 'safe_distance', 'exit_path_clearance',
                'grid_step', 'footprint_sample_step', 'map_wall_threshold',
                'robot_front_length', 'robot_rear_length', 'robot_left_width',
                'robot_right_width', 'elevator_local_x_min', 'elevator_local_x_max',
                'elevator_local_y_min', 'elevator_local_y_max')),
            static_footprint_probe_radius=0.05,
            nav2_inflation_used_for_candidate_rejection=False,
            exit_check_scope='sampled_center_line_only',
            entry_area_gate_reasons=self.entry_area_rejection,
            entry_area_limits={k: getattr(self, k, v) for k, v in AREA_DEFAULTS.items()},
        )
        if hasattr(self, 'candidate_diagnostics_pub'):
            self.candidate_diagnostics_pub.publish(String(data=json.dumps(self.candidate_diagnostics)))
        self.publish_entry_state()

        if getattr(self, 'comparison_method', 'legacy') != 'legacy':
            summary = comparison_summary(score_cells, self.comparison_method,
                                         {k: getattr(self, k, v) for k, v in AREA_DEFAULTS.items()})
            self.candidate_diagnostics['comparison'] = summary
            return None if summary['gate_reasons'] or self.entry_area_rejection else best_goal

        if self.entry_area_rejection:

            self.get_logger().warn(
                f'進入不可：危険領域 {danger_rate:.1f}% / 未観測領域 {unknown_rate:.1f}%'
            )

            self.publish_status(
                f'進入不可：危険領域 {danger_rate:.1f}% / 未観測領域 {unknown_rate:.1f}%'
            )

            return None

        self.get_logger().info(
            '===== エレベータ内の空き空間情報 ====='
        )

        self.get_logger().info(
            f'全体の調査面積: {total_area:.2f} m^2'
        )

        self.get_logger().info(
            f'空いている面積: {free_area:.2f} m^2 '
            f'({free_rate:.1f} %)'
        )

        self.get_logger().info(
            f'壁として除外した面積: {wall_area:.2f} m^2'
        )

        self.get_logger().info(
            f'ドア前として除外した面積: {door_front_area:.2f} m^2'
        )

        self.get_logger().info(
            f'障害物に近くて危険な面積: {danger_area:.2f} m^2'
        )

        self.get_logger().info(
            f'未観測として除外した面積: {unknown_area:.2f} m^2'
        )

        self.get_logger().info(
            f'安全な候補点: {free_count} / {total_count}'
        )

        self.get_logger().info(
            '===================================='
        )

        self.publish_status(
            f'空き空間を探索完了：空き面積 {free_area:.2f} m2 / ドア前 {door_front_area:.2f} m2 / 未観測 {unknown_area:.2f} m2'
        )
        # ==========================
        # 最終ゴールをエレベータ範囲内に補正
        # ==========================
        if fixed:
            # This exact pose has just passed all predicates. Do not clamp it
            # into a different, unevaluated position.
            return best_goal
        if best_goal is not None:

            x, y, score = best_goal

            if self.use_door_based_elevator:
                local_x, local_y = self.world_to_elevator_local(x, y)

                local_x = max(
                    self.elevator_local_x_min,
                    min(local_x, self.elevator_local_x_max)
                )

                local_y = max(
                    self.elevator_local_y_min,
                    min(local_y, self.elevator_local_y_max)
                )

                x, y = self.elevator_local_to_world(local_x, local_y)
            else:
                x = max(
                    self.elevator_x_min,
                    min(x, self.elevator_x_max)
                )

                y = max(
                    self.elevator_y_min,
                    min(y, self.elevator_y_max)
                )

            best_goal = (
                x,
                y,
                score
            )

            self.get_logger().info(
                f'最終ゴール: ({x:.2f}, {y:.2f})'
            )

        
        return best_goal



    # =====================================================
    # ゴール姿勢を作成する
    # =====================================================
    def create_goal_pose(self, x, y):

        pose = PoseStamped()

        pose.header.frame_id = 'map'

        pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        # エレベータ奥向きにする
        if self.manual_goal_yaw is not None:
            yaw = self.manual_goal_yaw
        elif self.use_door_based_elevator:
            yaw = math.atan2(
                self.inside_axis_y,
                self.inside_axis_x
            )
        else:
            yaw = math.pi / 2.0

        pose.pose.orientation.z = math.sin(
            yaw / 2.0
        )

        pose.pose.orientation.w = math.cos(
            yaw / 2.0
        )

        return pose


    # =====================================================
    # 経路の曲がりが大きすぎるか判定する

    # 始点・中間点・終点の3点で見る
    # =====================================================
    def is_path_too_curved(self, path):

        poses = path.poses

        if len(poses) < 3:
            return False

        # 経路全体から代表3点を取る
        start_pose = poses[0]
        mid_pose = poses[len(poses) // 2]
        end_pose = poses[-1]

        x1 = start_pose.pose.position.x
        y1 = start_pose.pose.position.y

        x2 = mid_pose.pose.position.x
        y2 = mid_pose.pose.position.y

        x3 = end_pose.pose.position.x
        y3 = end_pose.pose.position.y

        # 始点→中間点の角度
        angle1 = math.atan2(
            y2 - y1,
            x2 - x1
        )

        # 中間点→終点の角度
        angle2 = math.atan2(
            y3 - y2,
            x3 - x2
        )

        # 角度差を計算
        angle_diff = abs(angle2 - angle1)

        if angle_diff > math.pi:
            angle_diff = 2.0 * math.pi - angle_diff
        
        self.latest_turn_angle_deg = math.degrees(angle_diff)

        self.get_logger().info(
            f'経路全体の曲がり角: {math.degrees(angle_diff):.1f}度'
        )

        return angle_diff > self.max_turn_angle

    # =====================================================
    # 経路を確認してからゴール送信
    # =====================================================
    @motion_locked
    def check_path_then_send_goal(self, x, y, mode='entry'):

        if self.stop_active or self.operation_finished or not self.inputs_valid():
            return
        if not self.path_client.server_is_ready():
            # Nav2のLifecycle初期化はGazebo起動中に数十秒かかることがある。
            # ここで実験終了にせず、同じ入口判定を再試行する。
            now = self.get_clock().now().nanoseconds / 1e9
            self.entry_path_retry_due = (
                now + self.entry_path_retry_interval_sec
            )
            self.get_logger().warn(
                'Nav2 plannerの起動を待っています'
            )
            self.publish_status(
                'Nav2準備中：planner起動後に再試行します'
            )
            self.path_checking = False
            self.goal_sent = False
            self.operation_finished = False
            self.operation_phase = 'WAIT_INITIAL_OPEN'
            return

        self.path_checking = True

        goal_msg = ComputePathToPose.Goal()

        goal_msg.goal = self.create_goal_pose(x, y)
        goal_msg.planner_id = ''
        goal_msg.use_start = False

        epoch = self.motion_epoch
        self.announce('observing')
        try:
            future = self.path_client.send_goal_async(goal_msg)
        except Exception as exc:
            self.request_motion_stop('PLANNER_SEND_EXCEPTION ' + str(exc))
            return

        future.add_done_callback(
            lambda future: self.path_goal_response_callback(
                future,
                x,
                y,
                mode, epoch
            )
        )

        self.publish_status(
            'Nav2で経路を生成中'
        )


    # =====================================================
    # 経路生成ゴールの受付結果
    # =====================================================
    @motion_locked
    def path_goal_response_callback(self, future, x, y, mode='entry', epoch=None):

        if self.stop_active or epoch != self.motion_epoch:
            return
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.request_motion_stop('PLANNER_RESPONSE_EXCEPTION ' + str(exc))
            return

        if not goal_handle.accepted:
            self.path_checking = False
            if mode == 'entry' and self.skip_boarding('ENTRY_PATH_REJECTED'):
                return
            if mode == 'entry':
                self.announce('no_entry')
            self.get_logger().warn(
                '経路生成ゴールが拒否されました'
            )

            self.publish_status(
                '進入不可：経路生成が拒否されました'
            )

            self.publish_experiment_result(
                f'NG_PATH_REJECTED,{x:.2f},{y:.2f},0.0'
            )

            self.path_checking = False
            self.operation_finished = True
            self.operation_phase = 'FINISHED'
            return

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            lambda future: self.path_result_callback(
                future,
                x,
                y,
                mode, epoch
            )
        )


    # =====================================================
    # 経路生成結果を確認する
    # =====================================================
    @motion_locked
    def path_result_callback(self, future, x, y, mode='entry', epoch=None):

        if self.stop_active or epoch != self.motion_epoch:
            return
        try:
            response = future.result()
            if response.status != GoalStatus.STATUS_SUCCEEDED:
                # A blocked path is a boarding decision; TF/configuration and
                # transport failures still use the existing safety-stop route.
                if (mode == 'entry' and getattr(self, 'boarding_skip_enabled', False)
                        and response.status == GoalStatus.STATUS_ABORTED
                        and getattr(response.result, 'error_code', None) in (
                            ComputePathToPose.Result.START_OCCUPIED,
                            ComputePathToPose.Result.GOAL_OCCUPIED,
                            ComputePathToPose.Result.NO_VALID_PATH)):
                    self.path_checking = False
                    self.skip_boarding('ENTRY_PATH_UNAVAILABLE')
                    return
                raise ValueError('planner did not succeed')
            path = response.result.path
            if len(path.poses) > 10000:
                raise ValueError('planner path exceeds bounded callback limit')
        except Exception as exc:
            self.request_motion_stop('PLANNER_RESULT_EXCEPTION ' + str(exc))
            return

        if len(path.poses) == 0:
            self.path_checking = False
            if mode == 'entry' and self.skip_boarding('ENTRY_PATH_UNAVAILABLE'):
                return
            if hasattr(self, 'research_decision'):
                self.research_decision('OBSERVATION_WAIT',
                    ['ENTRY_PATH_UNAVAILABLE' if mode == 'entry' else 'EXIT_NAV2_PATH_UNAVAILABLE'])
            if mode == 'entry':
                self.announce('no_entry')
            self.get_logger().warn(
                '経路が生成されませんでした'
            )

            if mode == 'force_exit':
                self.exit_path_retry_count += 1
                retry_allowed = (
                    self.exit_path_max_retries <= 0
                    or
                    self.exit_path_retry_count <= self.exit_path_max_retries
                )

                if retry_allowed:
                    now = self.get_clock().now().nanoseconds / 1e9
                    self.exit_path_retry_due = (
                        now + self.exit_path_retry_interval_sec
                    )
                    self.publish_status(
                        '退出経路なし：コストマップ更新後に再試行 '
                        f'({self.exit_path_retry_count}/'
                        f'{self.exit_path_max_retries})'
                    )
                    self.get_logger().warn(
                        '退出経路を'
                        f'{self.exit_path_retry_interval_sec:.1f}秒後に再試行します '
                        f'({self.exit_path_retry_count}/'
                        f'{self.exit_path_max_retries})'
                    )
                    self.path_checking = False
                    self.goal_sent = False
                    self.operation_finished = False
                    self.operation_phase = 'WAIT_REOPEN'
                    return

                self.publish_status(
                    '退室できません：道を開けてください'
                )
            else:
                self.entry_path_retry_count += 1
                retry_allowed = (
                    self.entry_path_max_retries <= 0
                    or
                    self.entry_path_retry_count <= self.entry_path_max_retries
                )

                if retry_allowed:
                    now = self.get_clock().now().nanoseconds / 1e9
                    self.entry_path_retry_due = (
                        now + self.entry_path_retry_interval_sec
                    )
                    self.publish_status(
                        '進入経路なし：コストマップ更新後に再試行 '
                        f'({self.entry_path_retry_count}/'
                        f'{self.entry_path_max_retries})'
                    )
                    self.get_logger().warn(
                        '進入経路を'
                        f'{self.entry_path_retry_interval_sec:.1f}秒後に再試行します '
                        f'({self.entry_path_retry_count}/'
                        f'{self.entry_path_max_retries})'
                    )
                    self.path_checking = False
                    self.goal_sent = False
                    self.operation_finished = False
                    self.operation_phase = 'WAIT_INITIAL_OPEN'
                    return

                self.publish_status(
                    '進入不可：経路が生成されませんでした'
                )

            self.publish_experiment_result(
                f'NG_NO_PATH,{x:.2f},{y:.2f},0.0'
            )

            self.path_checking = False
            self.goal_sent = False
            self.operation_finished = True
            self.operation_phase = 'FINISHED'
            return

        if self.is_path_too_curved(path):
            self.path_checking = False
            if mode == 'entry' and self.skip_boarding('ENTRY_PATH_CURVED'):
                return
            if mode == 'entry':
                self.announce('no_entry')
            self.get_logger().warn(
                '経路の曲がりが大きいため進入を中止します'
            )

            if mode == 'force_exit':
                self.publish_status(
                    '退室不可：経路の曲がりが大きすぎます'
                )
            else:
                self.publish_status(
                    '進入不可：経路の曲がりが大きすぎます'
                )

            self.publish_experiment_result(
                f'NG_PATH_CURVED,{x:.2f},{y:.2f},{self.latest_turn_angle_deg:.1f}'
            )

            self.path_checking = False
            self.goal_sent = False
            self.operation_finished = True
            self.operation_phase = 'FINISHED'
            return

        if mode == 'entry' and self.use_door_open_check:
            _, confirmed_open, _, _ = self.get_door_state()

            if not confirmed_open:
                self.get_logger().warn(
                    '経路生成中にドア開確認が失われたため進入を保留します'
                )
                self.publish_status(
                    '進入保留：ドアが開いていることを再確認します'
                )
                self.path_checking = False
                self.goal_sent = False
                return

        self.get_logger().info(
            '経路の曲がりが小さいため進入します'
        )

        if mode == 'entry':
            self.entry_path_retry_due = None
            self.entry_path_retry_count = 0

        if mode == 'force_exit':
            self.publish_status(
                '退室経路を確認：エレベータから退室します'
            )
        else:
            self.publish_status(
                '経路確認OK：エレベータへ進入します'
            )

        self.set_inflation_radius()
        self.current_goal_x = x
        self.current_goal_y = y

        if self.send_nav_goal(x, y, mode):
            self.goal_sent = True
            if mode == 'force_exit':
                self.operation_phase = 'EXITING'
            else:
                self.operation_phase = 'ENTERING'
        elif (not self.stop_active and mode == 'entry'
              and getattr(self, 'lift_cycle_enabled', False)):
            if getattr(self, 'lift_pending_entry', None) is None:
                self.lift_pending_entry = (x, y, time.monotonic() + 10.)
            self.operation_phase = 'WAIT_INITIAL_OPEN'
            self.publish_status('入室条件の再確認待ち：静止・新鮮なIMUを確認後に経路を再計画します')
        elif not self.stop_active:
            self.current_goal_x = None
            self.current_goal_y = None
            self.publish_experiment_result(
                f'NG_NAV_SERVICE,{x:.2f},{y:.2f},0.0'
            )
            self.operation_finished = True
            self.operation_phase = 'FINISHED'

        self.path_checking = False

    # =====================================================
    # Nav2にゴールを送る
    # =====================================================
    def send_nav_goal(self, x, y, mode='entry'):
        submitted = self.submit_navigation(x, y, mode)
        if submitted:
            self.lift_pending_entry = None
            recorder = getattr(self, 'research_record', None)
            if recorder:
                recorder.phase('exit' if mode == 'force_exit' else 'entry', True)
            if mode == 'entry' and hasattr(self, 'research_decision'):
                self.research_decision('ENTRY_ALLOWED', ['ENTRY_PATH_CONFIRMED'])
            self.announce({
                'force_exit': 'exit',
                'entry_retreat': 'entry_retreat',
            }.get(mode, 'enter'))
        return submitted

    def on_entry_navigation_failure(self, status):
        """Retreat outside once after an aborted entry, if live gates permit it."""
        if (self.stop_active or self.shutdown_requested or not self.entry_enabled
                or self.operation_finished or self.navigation_unresolved()
                or not self.inputs_valid() or not self.get_door_state()[1]
                or not self.elevator_geometry_valid
                or getattr(self, 'exit_map_stage', None) is not None):
            return False
        self.stop_route.poll()
        if not self.stop_route.verified:
            return False
        region = self.doorway_region()
        if region == 'UNKNOWN':
            return False
        if region == 'OUTSIDE':
            self.current_goal_x = self.current_goal_y = None
            self.manual_goal_yaw = None
            self.operation_finished = True
            self.entry_enabled = False
            self.operation_phase = 'FINISHED'
            self.publish_experiment_result('NG_NAV_FAILED_OUTSIDE,0.00,0.00,0.0')
            self.publish_status('入室失敗：車体はドア外にいます')
            self.safety_event('entry_retreat_unnecessary', result_status=status)
            return True
        if (not self.use_door_based_elevator
                or self.manual_exit_local_y >= -self.robot_front_length - .05):
            return False
        # The foot sensor must still be fresh when a reverse move is planned.
        try:
            self.get_hokuyo_obstacles()
        except ValueError:
            return False
        exit_x, exit_y = self.elevator_local_to_world(
            self.manual_exit_local_x, self.manual_exit_local_y)
        if not all(math.isfinite(v) for v in (exit_x, exit_y)):
            return False
        self.manual_goal_yaw = math.atan2(
            self.inside_axis_y, self.inside_axis_x)
        if not self.exit_keep_inward_heading:
            self.manual_goal_yaw = math.atan2(
                -self.inside_axis_y, -self.inside_axis_x)
        self.current_goal_x, self.current_goal_y = exit_x, exit_y
        self.publish_status('入室失敗：ドア外へ退避します')
        self.safety_event('entry_retreat_requested', result_status=status,
                          x=exit_x, y=exit_y, doorway_region=region)
        if self.send_nav_goal(exit_x, exit_y, mode='entry_retreat'):
            return True
        self.current_goal_x = self.current_goal_y = None
        self.manual_goal_yaw = None
        return False

    @motion_locked
    def nav_result_callback(self, future, x, y, mode='entry', token=None):

        goal_result = self.consume_navigation_result(future, token)
        if goal_result is None or self.operation_finished:
            return
        status = goal_result.status
        if mode == 'bank_approach':
            if self.bank_stage=='YIELD_CANCEL':
                self.bank_approach_cancelled()
                return
            self.bank_approach_result(x, y)
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            recorder = getattr(self, 'research_record', None)
            if recorder:
                recorder.phase('exit' if mode == 'force_exit' else 'entry', False)
            self.get_logger().info(
                'ナビゲーションゴールに到達しました'
            )

            if mode == 'entry_retreat':
                if self.doorway_region() != 'OUTSIDE':
                    self.request_motion_stop('ENTRY_RETREAT_NOT_OUTSIDE')
                    return
                self.entry_enabled = False
                self.operation_finished = True
                self.operation_phase = 'FINISHED'
                self.publish_experiment_result('NG_ENTRY_RETREATED,0.00,0.00,0.0')
                self.publish_status('入室失敗：ドア外への退避が完了しました')
                self.safety_event('entry_retreat_completed')
            elif mode == 'force_exit':
                if getattr(self, 'lift_cycle_enabled', False):
                    self.operation_phase = 'WAIT_EXIT_STOP'
                    self.lift_exit_wait_at = time.monotonic()
                    self.reset_lift_body_quiet()
                    self.publish_status('降車目標到達：車体全体の降車と停止を確認中')
                    self.goal_sent = False
                    self.active_goal_handle = None
                    self.goal_start_time = None
                    self.current_goal_x = self.current_goal_y = None
                    self.manual_goal_yaw = None
                    return
                self.publish_experiment_result(
                    f'GOAL_REACHED,{x:.2f},{y:.2f},{self.latest_turn_angle_deg:.1f}'
                )
                if self.switch_map_after_exit:
                    self.begin_exit_map_switch()
                else:
                    self.publish_status('一連動作完了：エレベータ外へ退室しました')
                    self.operation_finished = True
                    self.operation_phase = 'FINISHED'
            else:
                self.publish_status(
                    'ゴール到達：エレベータ内部に到着しました'
                )

                if self.use_full_door_cycle and self.use_door_open_check:
                    now = self.get_clock().now().nanoseconds / 1e9
                    self.operation_phase = 'WAIT_CLOSE'
                    if getattr(self, 'lift_cycle_enabled', False):
                        self.operation_phase = 'WAIT_BOARD_STOP'
                        self.lift_board_wait_at = time.monotonic()
                        self.reset_lift_body_quiet()
                    self.door_close_wait_start_time = now
                    self.door_reopen_wait_start_time = None
                    self.door_reopen_confirmed_at = None
                    self.exit_path_retry_due = None
                    self.exit_path_retry_count = 0
                    self.publish_status(
                        '入室完了：停止してドアが閉まるのを待ちます'
                    )
                else:
                    self.publish_experiment_result(
                        f'GOAL_REACHED,{x:.2f},{y:.2f},{self.latest_turn_angle_deg:.1f}'
                    )
                    self.operation_finished = True
                    self.operation_phase = 'FINISHED'

        self.goal_sent = False
        self.active_goal_handle = None
        self.goal_start_time = None
        self.current_goal_x = None
        self.current_goal_y = None
        self.manual_goal_yaw = None

    # =====================================================
    # メイン処理
    # =====================================================
    def main_loop(self):

        self.bank_tick()
        self.maybe_start_on_closed_door()
        if self.stop_active or self.navigation_unresolved():
            if (self.stop_active and not self.navigation_unresolved()
                    and getattr(self, 'costmap_restore_required', False)):
                self.request_costmap_profile('normal')
            return
        if getattr(self, 'preboard_data_wait_enabled', False) and self.preboard_wait_tick():
            return
        if self.boarding_wait_tick():
            return
        if getattr(self, 'arrival_map_stage', None) is not None:
            self.advance_arrival_map()
            return
        if not self.entry_enabled or self.operation_finished:
            self.request_costmap_profile('normal')
            return
        if self.exit_map_stage is not None:
            self.advance_exit_map_switch()
            return
        if self.entry_enabled and not self.inputs_valid():
            self.publish_status('入力待ち：' + json.dumps(self.input_health(), ensure_ascii=False))
            return

        if not self.elevator_geometry_valid:
            self.publish_status(
                '進入停止：ドア座標とかご外周が一致しません'
            )
            return

        if not self.request_costmap_profile('entry'):
            self.publish_status(
                'Nav2をエレベータ用に切り替え中（コストマップ・走行設定）'
            )
            return

        if self.goal_sent:
            return

        if self.path_checking:
            return

        if (
            self.robot_x is None
            or
            self.robot_y is None
            or
            self.robot_yaw is None
        ):
            pose_source = 'オドメトリ' if self.use_odom_pose else 'AMCL'
            self.get_logger().warn(
                f'{pose_source}の現在位置を待っています...'
            )
            self.publish_status(
                f'{pose_source}の現在位置を待機中'
            )
            return

        if self.latest_scan is None:
            self.get_logger().warn(
                'LiDARデータを待っています...'
            )
            self.publish_status(
                'LiDARデータを待機中'
            )
            return

        if self.observation_guard_enabled:
            snapshot=self.get_observation()
            self.publish_observation(snapshot['detail'])
            if 'evidence' not in snapshot or snapshot['evidence'].reason!='OK':
                if hasattr(self, 'research_decision'):
                    self.research_decision('OBSERVATION_WAIT', ['OBSERVATION_INSUFFICIENT', snapshot['detail']['reason']])
                self.publish_status('観測待ち：'+snapshot['detail']['reason'])
                return

        if getattr(self, 'lift_cycle_enabled', False) and self.operation_phase in (
                'WAIT_BOARD_STOP', 'WAIT_CLOSE', 'WAIT_LIFT_START', 'WAIT_LIFT_STOP',
                'WAIT_DESTINATION', 'WAIT_EXIT_STOP'):
            self.publish_status('昇降連携待機：' + self.operation_phase + ' / ' + self.lift_cycle.reason)
            return

        if self.operation_phase == 'WAIT_CLOSE':
            self.wait_for_door_close_after_entry()
            return

        if self.operation_phase == 'WAIT_REOPEN':
            self.wait_for_door_reopen_and_exit()
            return
        if self.operation_phase == 'ARRIVAL_HOLD':
            self.publish_status('到着確認完了：'+getattr(self,'arrival_exit_mode','auto')+'（退出指令は未送信）')
            return

        if self.operation_phase != 'WAIT_INITIAL_OPEN':
            self.get_logger().warn(
                f'処理待機中：現在の状態={self.operation_phase}'
            )
            return

        now = self.get_clock().now().nanoseconds / 1e9

        if (
            self.entry_path_retry_due is not None
            and
            now < self.entry_path_retry_due
        ):
            remaining = self.entry_path_retry_due - now
            self.publish_status(
                f'進入経路を再確認するまで待機中（残り約{remaining:.1f}秒）'
            )
            return

        self.entry_path_retry_due = None

        pending = getattr(self, 'lift_pending_entry', None)
        if pending is not None:
            x, y, deadline = pending
            if time.monotonic() > deadline:
                self.lift_pending_entry = None
                self.request_motion_stop('ENTRY_ADMISSION_TIMEOUT')
            elif self.lift_start_ready():
                # Re-plan against current costmaps; do not repeat the expensive
                # candidate scan while waiting for fresh stationary evidence.
                self.check_path_then_send_goal(x, y)
            return

        self.publish_status(
            'エレベータ内部を確認中'
        )

        if self.should_wait_for_open_door():
            return

        if self.should_wait_for_exiting_person():
            return

        self.announce('observing')
        try:
            obstacles, unknown_shadows = self.get_dynamic_obstacles()
        except ValueError as exc:
            if self.observation_guard_enabled or str(exc).startswith('HOKUYO_'):
                if hasattr(self, 'research_decision'):
                    self.research_decision('OBSERVATION_WAIT', ['OBSERVATION_INSUFFICIENT', str(exc)])
                self.publish_status('観測待ち：'+str(exc))
                return
            raise

        self.get_logger().info(
            f'検出した動的障害物の点数: {len(obstacles)}'
        )

        self.publish_status(
            f'動的障害物を検出：{len(obstacles)}点 / 未観測影：{len(unknown_shadows)}本'
        )

        self.publish_status(
            '空き空間を探索中'
        )

        evaluation_started = time.monotonic()
        best_goal = self.find_best_goal(obstacles, unknown_shadows)
        if hasattr(self, 'research_evaluation'):
            self.research_evaluation(obstacles, best_goal, time.monotonic() - evaluation_started)

        if best_goal is None:
            if getattr(self, 'selection_stability', None):
                self.selection_stability.reset()
            reason = (' / '.join(self.entry_area_rejection)
                      if self.entry_area_rejection else 'NO_VALID_CANDIDATE')
            if self.skip_boarding(reason):
                return
            self.announce('no_entry')
            self.get_logger().warn(
                '安全な空き場所が見つかりませんでした'
            )
            self.publish_status(
                '進入不可：空き場所なし'
            )
            return

        if getattr(self, 'selection_stability', None):
            stamp = self.latest_scan.header.stamp
            ready = self.selection_stability.update(
                best_goal, (stamp.sec, stamp.nanosec), time.monotonic())
            wait = time.monotonic() - self.selection_stability.since
            if not ready:
                self.research_decision('OBSERVATION_WAIT', ['SELECTION_STABILIZING'],
                                       stability_wait_sec=wait)
                self.publish_status('観測待ち：候補位置の連続確認中')
                return
            self.research_decision('CANDIDATE_AVAILABLE', ['SELECTION_STABLE'], stability_wait_sec=wait)

        goal_x, goal_y, score = best_goal
        if getattr(self,'entry_fixed_local_pose',[]):
            self.manual_goal_yaw=self.goal_footprint_yaw()

        self.publish_status(
            f'進入可能：目標点({goal_x:.2f},{goal_y:.2f})'
        )

        self.get_logger().info(
            f'選ばれたゴール: x={goal_x:.2f}, '
            f'y={goal_y:.2f}, '
            f'評価値={score:.2f}'
        )

        self.check_path_then_send_goal(
            goal_x,
            goal_y
        )


def restore_costmap_profile_on_shutdown(node, executor, timeout_sec=8.0):
    """Keep cancellation and observations alive until a bounded shutdown verdict."""
    with node.motion_lock:
        node.shutdown_requested = True
        passive = (getattr(node, 'operator_handoff', False) or
                   (not node.stop_active and not node.navigation_unresolved()
                   and not node.goal_sent and not node.path_checking
                   and not (node.entry_enabled and not node.operation_finished)
                   and not (getattr(node, 'lift_cycle_enabled', False)
                            and node.lift_cycle.active)))
        if getattr(node, 'lift_cycle_enabled', False) and node.lift_cycle.active:
            if node.lift_cycle.phase != 'FAULT':
                node.lift_cycle.fail('SHUTDOWN')
    node.timer.cancel()
    # A passive node does not own the robot's motion. Publishing the persistent
    # mux lock here would disable keyop after the elevator process has exited.
    if not passive:
        node.request_motion_stop('SHUTDOWN')
    node.safety_event('shutdown_requested', passive=passive)
    deadline = time.monotonic() + timeout_sec
    restore_requested = False
    profile_restored = False
    body_stopped = None
    while rclpy.ok() and time.monotonic() < deadline:
        with node.motion_lock:
            if not node.navigation_unresolved():
                if not node.dynamic_costmap_profile:
                    profile_restored = True
                elif node.costmap_profile_request is None:
                    if (restore_requested and node.costmap_profile_state == 'normal'
                            and node.costmap_profile_success):
                        profile_restored = True
                    elif not profile_restored:
                        node.request_costmap_profile('normal', force=True)
                        restore_requested = node.costmap_profile_request == 'normal'
            if getattr(node, 'lift_cycle_enabled', False):
                body_stopped = node.lift_body_stopped(time.monotonic(), node.get_clock().now().nanoseconds/1e9)
            if not node.navigation_unresolved() and profile_restored and body_stopped is not False:
                break
        executor.spin_once(timeout_sec=0.05)
    result = dict(nav_unresolved=node.navigation_unresolved(), profile_restored=profile_restored,
                  odom_stop_observed=body_stopped, physical_stop_verified=False)
    confirmed = not result['nav_unresolved'] and profile_restored and body_stopped is not False
    node.safety_event('shutdown_complete' if confirmed else 'shutdown_unconfirmed', **result)
    if not confirmed:
        node.get_logger().warn('終端・設定復帰・観測停止の一部を未確認。'
                               + ('待機中のため新しい停止ロックは発行していません' if passive else
                                  '専用停止ロックを保持して終了します' if node.stop_latching_enabled
                                  else '停止保持は無効です。終了時の状態を確認してください'))
    return result


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = executor = None
    stop_requested = False

    def request_shutdown(_signum, _frame):
        nonlocal stop_requested
        stop_requested = True

    old_sigint = signal.signal(signal.SIGINT, request_shutdown)
    old_sigterm = signal.signal(signal.SIGTERM, request_shutdown)
    try:
        node = ElevatorEntryMapDiff()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        while rclpy.ok() and not stop_requested:
            executor.spin_once(timeout_sec=0.1)
    finally:
        if node is not None and executor is not None:
            restore_costmap_profile_on_shutdown(node, executor)
            shutdown_executor(executor)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGINT, old_sigint)
        signal.signal(signal.SIGTERM, old_sigterm)


if __name__ == '__main__':
    main()
