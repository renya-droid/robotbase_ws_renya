# システム構成

## 目的

`robotbase_ws` はSIRIUSを参考にしたROS 2 Jazzyワークスペースだが、現在の起動経路は新機体専用の `robotbase_bringup` と `robotbase_sim` を正本とする。センサーはVelodyne VLP-16とIMUのみ、駆動は差動二輪。

## パッケージの責務

| パッケージ | 責務 |
|---|---|
| `robotbase_bringup` | URDF、RViz、Roboteq、VLP-16、IMU、EKF、twist_mux、SLAM、Nav2 |
| `robotbase_sim` | Gazeboワールド、差動二輪物理、VLP-16/IMU模擬データ |
| `roboteq_ros2_driver` | `/cmd_vel`からモータ指令、encoderから`/odom` |
| `velodyne_*` | VLP-16 packet、PointCloud2、LaserScan変換 |
| `witmotion_ros` | IMU入力 |
| `navigation2` | Nav2本体 |
| `slam_toolbox` | 2D SLAM |
| `robotbase_keyop` | ココちゃん専用の手動操作ノード。`/cmd_vel_teleop` と `/stop` を出力 |
| `robotbase_waypoint` | SIRIUS互換YAMLの地点記録と`NavigateToPose`連続goal送信 |

`src/sirius/sirius_description` と `sirius_navigation` 内の旧launchは、ココちゃんのRViz/Nav2/SLAMから呼ばない。

## 実機データフロー

### センサーと自己位置

```text
Roboteq encoder -> /odom ----+
                              +-> robot_localization EKF -> /odom/filtered
IMU ------------> /imu ------+

VLP-16 packets -> /velodyne_points -> velodyne_laserscan -> /scan3

/scan3 + map + odom TF -> AMCL -> map -> robot/odom
```

TF責務:

| TF | 配信元 |
|---|---|
| `map -> robot/odom` | 既存地図ではAMCL、SLAM時はslam_toolbox、Nav2 simの試験用static指定時だけ固定TF |
| `robot/odom -> robot/base_footprint` | 実機はEKF、simはGazebo DiffDrive |
| `robot/base_footprint -> robot/base_link/lidar_link/imu_link` | robot_state_publisher |

`koko_roboteq` は `pub_odom_tf:=false` で、EKFとの二重TFを防ぐ。EKFを使わないdriver単体試験だけ `koko_roboteq_no_sf` を使う。

### Nav2速度指令

```text
RViz / NavigateToPose
  -> bt_navigator
  -> planner_server (NavFn)
  -> controller_server (MPPI, DiffDrive) /cmd_vel_nav
  -> velocity_smoother                /cmd_vel_smoothed
  -> twist_mux                         /cmd_vel
  -> roboteq_ros2_driver
```

YAML追従は`robotbase_waypoint`が各地点を`NavigateToPose`へ送る。Nav2標準`waypoint_follower`も起動しており、RViz等からの`/follow_waypoints`を受けられる。両方式のgoalを同時に送らない。

この接続の正本:

| 境界 | ファイル |
|---|---|
| Nav2 node起動とremap | `robotbase_bringup/launch/nav2.launch.py` |
| Nav2 pluginとcostmap | `params/real/nav2.yaml` / `params/sim/nav2.yaml` |
| mux入力と優先度 | `params/real/twist_mux.yaml` / `params/sim/twist_mux.yaml` |
| Roboteq入力と車輪値 | `params/real/roboteq.yaml` |

### 手動操作

```text
keyop2 ------> /cmd_vel_teleop -- priority 100 --+
direct ------> /cmd_vel_direct -- priority  90 --+-> twist_mux -> /cmd_vel
Nav2 --------> /cmd_vel_smoothed priority  10 --+
idle --------> /cmd_vel_idle ---- priority   1 --+
/stop ------------------------------------- 255 lock
```

## シミュレーション

シミュレーション系alias/UIは`ROS_DOMAIN_ID=58`、実機系は57を使用する。これによりGazeboの`/clock`が実機ノードや別PCのシミュレータと混在しない。

```text
koko_sim
  Gazebo + robot_state_publisher + VLP-16 + IMU + twist_mux

koko_rviz_sim
  RVizのみ

koko_slamtoolbox_sim
  slam_toolboxのみ

koko_nav2_sim_map
  map server + AMCL + 最小Nav2のみ

koko_nav2_sim_slam
  slam_toolbox + 最小Nav2（地図なし）

koko_waypoint_follow_sim
  YAML選択 + NavigateToPose連続送信
```

GazeboとRVizは独立して起動する。地図作成だけなら `koko_slamtoolbox_sim`、既存地図なら `koko_nav2_sim_map`、地図なし自律移動なら `koko_nav2_sim_slam` を選ぶ。シミュレーションの `twist_mux` は `koko_sim` 内で起動する。

## 実機

```text
koko_roboteq
koko_velodyne
koko_imu
koko_sf_real
koko_twist_mux

# 用途に応じてどちらか
koko_slamtoolbox_real
koko_nav2_real
koko_nav2_real_slam  # SLAM Toolbox + Nav2（地図を生成・更新）
koko_waypoint_follow # YAML選択 + NavigateToPose連続送信

# 表示
koko_rviz_real
```

`koko_nav2_real_slam` はmap serverとAMCLを起動しない。SLAM Toolboxが `/map` と `map -> robot/odom` を配信し、Nav2は更新中の地図を使う。永続化は別ターミナルの `koko_map_save` で行う。

## SIRIUSとの差分

| 項目 | SIRIUS | ココちゃん |
|---|---:|---:|
| 車輪円周 | 0.825 m | 0.6283 m |
| トレッド幅 | 0.40 m | 0.435 m |
| encoder pulse/回転 | 475 | 950 |
| odom publish | 20 Hz | 50 Hz |
| 車体footprint | 約1.20 x 0.70 m | 0.90 x 0.66 m |
| センサー | 複数LiDAR/カメラ構成 | VLP-16 + Hokuyo + IMU |
| LaserScan | `scan3`等 | `/scan3`、`/hokuyo_scan` |
| TF prefix | `sirius3` | `robot`（設定可能） |
| controller | MPPI | MPPI（ココちゃんのfootprint・速度制限を使用） |

ZED、SAM3、RTAB-MAP、semantic costmap、STVLはココちゃんの有効な起動経路に含まない。

## 安全上の未完了

現在はcostmapの独立ObstacleLayerで `/scan3` と `/hokuyo_scan` を使うが、Nav2 Collision Monitorのような独立した最終停止段は起動していない。実機で速度指令経路を確認後、必要なら次の一方向へ追加する。

```text
velocity_smoother -> collision monitor -> twist_mux -> Roboteq
```

同一の `/cmd_vel` へ複数ノードが直接publishする構成にはしない。

## Overlay確認

```bash
source ~/robotbase_ws/install/setup.bash
ros2 pkg prefix robotbase_bringup
ros2 pkg prefix robotbase_sim
ros2 pkg prefix nav2_controller
ros2 pkg prefix roboteq_ros2_driver
```

別ワークスペースを後からsourceすると同名上流packageが切り替わるため、ココちゃん用コマンドは必ず `koko_src` から起動する。
