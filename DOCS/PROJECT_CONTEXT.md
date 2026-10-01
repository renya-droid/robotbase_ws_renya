# Project Context

将来の開発者やAIが、ソース全体を再探索せず作業を始めるための短いコンテキスト。

## 一文で

`robotbase_ws` は、VLP-16とIMUを搭載するRoboteq駆動の差動二輪ロボット「ココちゃん」用ROS 2 Jazzyワークスペース。

## 現在地（2026-08-19）

- 実機手動走行、`keyop2`: 成功
- 旧Nav2: RVizにパスは出たが実機が動かなかった。当時の直接原因はログ不足で未確定
- 新Nav2: 専用最小設定へ置換し`use_realtime_priority: false`、実機自律移動に成功
- 実機Nav2: SLAMしながらの走行も確認済み。障害物余裕・滑らかさ・速度校正は調整中
- Gazebo SLAM: 動作確認済み
- Gazebo Nav2: `params/sim/`、分離launch、`robot/*` TFで `(4, 0)` 到達、`SUCCEEDED`
- ウェイポイント: `robotbase_waypoint`でSIRIUS互換YAMLの記録・連続追従に対応
- 実機ログ/rosbag: このチェックアウトにはなし

## 正本

最初に見る場所:

1. `README.md`
2. `src/robotbase_bringup/` — URDF、RViz、全ロボット固有params/launch
3. `src/robotbase_sim/` — Gazeboだけ
4. `bash/bash_alias2.sh` — ランチャーのボタン定義
5. `DOCS/NAV2_NO_MOTION.md` — 実機切り分け

ルート `params/` の旧Nav2/SLAM/EKF設定は削除済み。SIRIUS由来の `sirius_description` はRVizから呼ばない。

## ロボット固有値

- 車輪直径/円周: 0.20 / 0.6283 m
- トレッド: 0.435 m
- encoder: 950 pulse/回転
- footprint: 前0.425、後0.475、左右0.33 m（全長0.90 x 全幅0.66 m）
- 生odom: `/odom`
- EKF odom: `/odom/filtered`
- PointCloud2: `/velodyne_points`
- Nav2 costmap用LaserScan: `/scan3`（VLP-16）+ `/hokuyo_scan`（足元Hokuyo）
- AMCL/SLAM用LaserScan: `/scan3`（複数リング合成）
- IMU: `/imu`
- TF: `map -> robot/odom -> robot/base_footprint`

TFの `robot` は `robot.env` の `ROBOTBASE_TF_PREFIX` で変更可能。表示名「ココちゃん」とは独立。

## 起動コマンド

シミュレーション:

以下は自動的に`ROS_DOMAIN_ID=58`を使用する。実機コマンドは57を使用する。

```bash
koko_sim
koko_rviz_sim
koko_slamtoolbox_sim  # 地図作成のみ
koko_nav2_sim_map     # 既存地図でNav2
koko_nav2_sim_slam    # 地図なし、SLAM + Nav2
koko_waypoint_follow_sim
```

実機:

```bash
koko_roboteq
koko_velodyne
koko_imu
koko_sf_real
koko_twist_mux
koko_rviz_real
koko_slamtoolbox_real # 地図生成時
koko_nav2_real        # 自律移動時
koko_nav2_real_slam   # 地図を生成・更新しながら自律移動
koko_foxglove         # 共通Foxgloveサーバー、ws://PC_IP:8766
koko_waypoint_follow  # YAMLを選択して連続自律移動
```

GazeboとRVizは別プロセス。`koko_nav2_sim_slam` と `koko_nav2_real_slam` はSLAMとNav2を同時起動する。シミュレーションの `twist_mux` は `koko_sim` に含まれるが、実機では `koko_twist_mux` を別途起動する。

## 速度指令

```text
/cmd_vel_nav
  -> velocity_smoother
  -> /cmd_vel_smoothed
  -> twist_mux
  -> /cmd_vel
  -> roboteq_ros2_driver
```

手動入力 `/cmd_vel_teleop` はNav2より高優先度。idleは1、`/stop` lockは255。lockが残っていてもNav2は動かない。

## Nav2無走行で最初に確認

```bash
ros2 lifecycle get /controller_server
ros2 topic hz /cmd_vel_nav
ros2 topic hz /cmd_vel_smoothed
ros2 topic hz /cmd_vel
ros2 topic hz /odom/filtered
ros2 topic hz /scan3
ros2 run tf2_ros tf2_echo robot/odom robot/base_footprint
```

最初に途切れる境界が原因箇所。新設定ではRT優先度を無効化済みなので、次はtwist_mux、`/stop`、odom/scan3/TF更新を優先して見る。

## 含めないもの

ココちゃんの有効なUI、alias、Nav2/SLAM paramsには次を含めない。

- ZED、SAM3、RTAB-MAP
- LLM dynamic goal、status monitor、BLE gateway
- semantic costmap、STVL
- 外部連携タブ（Foxgloveは依頼によりユーティリティへ個別追加）

上流または移植元ソースが `src/sirius/` に残っていても、新しいbringupから参照しない。

`src/sirius/` は `COLCON_IGNORE` によりビルド対象外。実機PCへのclone、依存導入、udev、VLP-16 NIC、段階試験は `DOCS/REAL_PC_MIGRATION.md` を正本とする。
