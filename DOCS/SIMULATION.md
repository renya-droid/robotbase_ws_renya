# Robotbase Gazeboシミュレータ

## ワールド選択（renya/camera-nav）

ランチャー2の `koko_sim` または通常のsimプリセットを押すと、Gazebo起動前にワールド選択画面が開きます。
端末の `koko_sim` では番号を選択します。従来の部屋固定プリセットはそのままです。

| 選択肢 | 広さ・用途 |
| --- | --- |
| 標準試験場 | 約12 × 12 m。従来の障害物付き試験場 |
| 屋内施設 | 40 × 30 m。幅4 mの十字廊下、8部屋、幅2.4 mの出入口、机 |
| 部屋 | 3.60 × 2.70 m。既存の部屋モデル |

屋内施設はGazeboの基本形状だけで構成した施設型試験場です。外部モデルのダウンロードは不要です。
設定カタログは `src/robotbase_sim/worlds/catalog.json` です。
[施設の配置図](images/indoor_facility.svg)で廊下・部屋・初期位置を確認できます。

`自律移動（sim）` では、選択ワールドに対応する地図が地図一覧の先頭に出ます。Enterでその地図を選択できます。
その場合、Nav2のAMCL初期位置もGazeboのスポーン位置に合わせます。
保存済みの別地図を選ぶ機能も残しています。別地図では必要に応じRVizの2D Pose Estimateで位置を合わせてください。

`人位置Layer・ワールド選択（sim）` プリセットではワールドを選ぶだけで、対応地図・初期位置で人位置Layer付きNav2を起動します。
従来の部屋固定の `人位置Layer付き自律移動（sim）` は変更していません。
ワールド切り替え時はシミュレーション一式を終了してから起動し直してください。

```bash
# 一覧（Gazeboは起動しない）
bash ~/robotbase_ws_renya/bash/startup_bash/start_sim.sh --list-worlds

# 選択画面を出さず、屋内施設を起動
koko_sim --world-id indoor_facility

# 画面なしの起動にも対応
koko_sim --world-id indoor_facility gui:=false
```

`world:=/path/to/world.sdf spawn_x:=... spawn_y:=... spawn_yaw:=...` も従来どおり使用できます。
外部ワールド指定時は対応地図を自動推測しないため、Nav2側でそのワールドの地図を選択してください。
`--world-id` と `world:=` は併用しません。

最後の選択はこのWSの `.runtime/sim_world_<ROS_DOMAIN_ID>.json` に保存します。
これは最後の選択情報であり、Gazeboが実行中であることを示すものではありません。
シミュレーションのDomainごとに分離し、Git管理対象外にしています。

屋内施設のワールドと地図は同じ壁・家具配置から生成しています。再生成する場合:

```bash
python3 src/robotbase_sim/scripts/generate_indoor_facility.py
colcon build --packages-select robotbase_sim --symlink-install
```

## 目的と確認済み範囲

`src/robotbase_sim/` は、実機がなくても次を確認するための最小構成です。

- 差動二輪の手動走行
- Velodyne VLP-16相当の3D点群と、複数リングから変換した2D `/scan3`
- IMUデータ
- slam_toolboxによる2D地図生成
- 同梱地図を使ったNav2の経路生成、障害物回避、速度指令、ゴール到達

2026-08-19にヘッドレス実行で確認した結果は次のとおりです。

| 確認項目 | 結果 |
|---|---|
| `/odom` | 約50 Hz |
| `/velodyne_points` / `/scan3` | 点群配信あり / 約10 Hz |
| `/imu` | 約100 Hz |
| slam_toolbox | Lifecycle `active`、`map -> base` TFと `/map` を確認 |
| Nav2 | 管理ノードすべて `active`。AMCLも`active` |
| `2D Pose Estimate` | `/initialpose`のAMCL購読と、指定値への`/amcl_pose`更新を確認 |
| Local Costmap | 120 x 120セル、約1.67 Hz。障害物・inflationの非ゼロセルを確認 |
| DWB時点の障害物迂回試験 | `(0, 0)`から`(4, 0)`へ到達、`SUCCEEDED` |
| MPPI短距離試験 | `(0, 0)`から`(1, 0)`へ移動し、`NavigateToPose: SUCCEEDED`を確認 |
| MPPI障害物迂回試験 | `(0, 0) → (4, 0)`、`(-4, 0) → (4, 0)`、`(4, 0) → (-4, 0)`がすべて`SUCCEEDED` |
| MPPI前進優先試験 | `PreferForwardCritic: 40`で長距離往路の負速度0件、後方ゴールで最大`-0.027 m/s` |
| MPPI障害物離隔 | 長距離迂回中、車体footprintから主要障害物まで最小約0.29 m（シミュレーション形状から算出） |
| DWB時点の参考記録 | odom位置 `(3.841, -0.164)`へ到着（許容半径0.25 m内） |
| 停止 | 完了後の `/cmd_vel` がゼロ |

GazeboのDiffDriveはplugin既定で有効になる。起動後は`/cmd_vel`からGazebo Transport `/robot/cmd_vel`への到達と、車輪速度・`/odom`変化までを確認する。

## 初回ビルド

```bash
cd ~/robotbase_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -riy
colcon build --symlink-install --packages-select robotbase_sim
source install/setup.bash
```

このワークスペースのNav2やslam_toolboxをまだビルドしていない場合、それらは読み込まれているunderlayのものを使います。本番に近い組み合わせを固定したい場合は、ルートREADMEの全体ビルドを実行してください。

必要な主な実行パッケージは `ros_gz_sim`、`ros_gz_bridge`、`twist_mux`、`velodyne_laserscan`、`slam_toolbox`、`nav2_bringup`、`rviz2` です。依存関係の正本は `src/robotbase_sim/package.xml` です。

## 起動方法

### センサーと手動走行だけ

```bash
koko_sim
```

別ターミナルから次のように動かせます。

```bash
ros2 topic pub --rate 10 /cmd_vel_teleop geometry_msgs/msg/Twist \
  '{linear: {x: 0.25}, angular: {z: 0.3}}'
```

停止は発行側で `Ctrl-C` です。`idle_twist_publisher` が自動的にゼロ指令へ戻します。

`koko_keyop2_sim` は `w/x` の直進速度を0.20 m/s、`a/d` の角速度を0.10 rad/sずつ変更して保持します。純前進を試すときは `s` で両方をゼロにしてから `w` を押します。ウェイポイント記録中は`p`で現在地を10秒停止地点として保存します。

### SLAMで地図生成

```bash
koko_sim
koko_rviz_sim
koko_slamtoolbox_sim
```

RVizの地図を見ながら `/cmd_vel_teleop` で走行します。保存例:

```bash
koko_map_save_sim
```

地図名を入力すると、`~/robotbase_ws/maps_waypoints/maps/` に `.yaml` と `.pgm` が生成されます。保存先ディレクトリはスクリプトが自動作成します。

### 既存地図でNav2自律移動

```bash
# 地図一覧から同梱test_arenaまたは保存済み地図を選択
koko_sim
koko_rviz_sim
koko_nav2_sim_map

# SLAMで保存した任意地図
ros2 launch robotbase_sim navigation.launch.py \
  map:=$HOME/robotbase_ws/maps_waypoints/maps/koko-sim.yaml
```

RVizの「Nav2 Goal」でゴールを指定します。同梱地図はGazeboワールドと一致し、開始位置は `(0, 0, 0)` です。CLIで確認済みゴールを再現する場合:

既存地図モードはAMCLを起動します。起動直後または自己位置がずれた場合は、RVizの `2D Pose Estimate` で地図上の現在位置と向きを指定してください。矢印を放した時点で `/initialpose` がAMCLへ渡り、`map -> robot/odom` が更新されます。

`koko_nav2_sim_map` は、同梱 `test_arena` と `maps_waypoints/maps/` 内の `.yaml` / `.yml` を番号付き一覧で表示します。地図一覧だけを端末で確認する場合:

```bash
bash ~/robotbase_ws/bash/startup_bash/nav2_bringup_sim.sh --list
```

RVizのDisplaysでは `Global Costmap`、`Local Costmap`、`Global Plan`、`Local Plan`、`Local Footprint` を個別に表示・非表示できます。右側の `Navigation 2` はNav2標準パネルです。

```bash
ros2 action send_goal /navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: 4.0, y: 0.0}, orientation: {w: 1.0}}}}'
```

### 地図なしでSLAMしながらNav2自律移動

```bash
koko_sim
koko_rviz_sim
koko_nav2_sim_slam
```

`koko_nav2_sim_slam` はSLAM ToolboxとNav2を同時起動します。map serverや固定 `map -> odom` は起動せず、SLAM Toolboxが `/map` と `map -> robot/odom` を生成します。初期スキャンで地図が生成されてからRVizの「Nav2 Goal」を指定します。

### GUIなし

CIやリモート端末では次を使用します。

```bash
koko_sim_env
ros2 launch robotbase_sim sim.launch.py gui:=false
ros2 launch robotbase_sim mapping.launch.py
# または
ros2 launch robotbase_sim navigation.launch.py
# または
ros2 launch robotbase_sim navigation_slam.launch.py
```

`bash/bash_alias2.sh` を読み込んでいれば次の短縮名も使えます。

| alias | 起動内容 |
|---|---|
| `koko_sim` | Gazebo、ロボット、センサー、twist_mux |
| `koko_rviz_sim` | RVizのみ |
| `koko_slamtoolbox_sim` | slam_toolboxのみ |
| `koko_nav2_sim_map` | 同梱地図サーバー + AMCL + Nav2 |
| `koko_nav2_sim_slam` | slam_toolbox + Nav2（map serverなし） |
| `koko_keyop2_sim` | Domain 58へ手動速度指令 |
| `koko_map_save_sim` | Domain 58の`/map`を保存 |

シミュレーションでは `koko_sim` が `twist_mux` を起動するため、`koko_twist_mux` は不要です。実機では `koko_twist_mux` を別途起動します。

## モデルの前提

| 項目 | 値 |
|---|---:|
| 駆動 | 差動二輪 + 前後キャスター |
| 車輪直径 | 0.20 m |
| トレッド幅 | 0.435 m |
| 車輪joint軸 | `base_footprint`基準のY軸 `(0, 1, 0)` |
| 車体外形 | 約0.90 x 0.66 m |
| VLP16取付位置 | `base_footprint`から `(0, 0, 0.72)` m |
| IMU取付位置 | `base_footprint`から `(0, 0, 0.28)` m |
| VLP16 | 水平720点、垂直16 ring、上下±15度、10 Hz |
| 2D scan | ring 1〜6、8を距離制限付きで合成して `/scan3` へ変換 |
| IMU | 100 Hz、簡易Gaussian noise付き |

センサーの実取付位置が判明したら、SDFとURDFの両方を同じ値へ変更します。

- 物理・センサー: `src/robotbase_sim/models/robotbase.sdf`
- RViz/TF用モデル: `src/robotbase_sim/urdf/robotbase.urdf`

## TF設計

### 地図生成モード

```text
map                         slam_toolboxが推定
└─ robot/odom               Gazeboの開始原点
   └─ robot/base_footprint  Gazebo DiffDriveの動的TF
      ├─ robot/base_link
      ├─ robot/lidar_link
      └─ robot/imu_link
```

### Nav2モード（既定: AMCL）

```text
map                         AMCLが推定
└─ robot/odom
   └─ robot/base_footprint
      ├─ robot/base_link
      ├─ robot/lidar_link
      └─ robot/imu_link
```

既存地図モードの既定は、実機と同じAMCLです。VLP16の単一ringから得る2D scanを使うため、長距離走行では推定がずれる可能性があります。自己位置を再設定する場合はRVizの `2D Pose Estimate` を使います。

経路計画・controllerだけを再現性優先で試し、自己位置推定を評価しない場合に限り、固定identity TFへ切り替えられます。このモードではAMCLが起動しないため `2D Pose Estimate` は効きません。

```bash
ros2 launch robotbase_sim navigation.launch.py localization:=static
```

既定接頭辞は `robot` です。`robot.env` の `ROBOTBASE_TF_PREFIX` を変えると、alias経由のGazebo、URDF、SLAM、Nav2、RVizへ一括反映されます。

各 `koko_sim` 起動は `GZ_PARTITION` にプロセス固有の接尾辞を加えます。またGazebo内部のセンサー・odom・速度トピックも `/robot/...` に分離してからROS側の標準トピックへbridgeします。ROS graph自体も実機Domain 57とシミュレーションDomain 58へ分けるため、実機や別PCの`/clock`、TF、`/scan3`は混入しません。

`koko_sim`は起動中ロックを保持し、設定中のシミュレーションDomainに既存`/clock`または残存RViz/Nav2/SLAMがある場合は起動を拒否する。Gazeboだけを再起動して古いRVizへ0秒からの時刻を送ると、`TF_OLD_DATA`、`Detected jump back in time`、RVizの異常終了を引き起こすためである。再起動時はランチャー上部の赤い「シミュレーション一式を終了」を押し、完了表示後に`koko_sim`、RViz、SLAM/Nav2の順に起動する。端末だけの場合は`bash ~/robotbase_ws/bash/startup_bash/stop_simulation.sh`で同じ終了処理を実行できる。

Domain 58は既定値であり、ランチャー上部の数値欄と「Domainを保存」で変更・永続化できる。保存先は`robot.env`で、全`koko_*_sim`コマンドが同じ値を読む。Domain変更は古いGazeboを終了する代わりにはならないため、現在のDomainにプロセスが残っている場合は先に赤い終了ボタンを使う。

シミュレーションでは0.1 m/s付近の指令で停止状態に留まる事例があったため、Keyop2の直進刻みを0.20 m/sとし、Nav2 MPPIへ`VelocityDeadbandCritic: 0.12 m/s`を設定する。velocity smootherは0.10 m/s未満を0へ丸める。Gazebo DiffDrive自体は起動時に有効になるため、`/model/robot/enable`の外部publishには依存しない。

## トピック経路

センサー:

```text
Gazebo VLP16 -> /velodyne_points -> velodyne_laserscan -> /scan3
Gazebo IMU   -> /imu
Gazebo DiffDrive -> /odom + odom TF
```

Nav2速度指令:

```text
controller_server
  -> /cmd_vel_nav
  -> velocity_smoother
  -> /cmd_vel_smoothed
  -> twist_mux
  -> /cmd_vel
  -> Gazebo DiffDrive
```

優先度はteleop 100、direct 90、navigation 10、idle 1、`/stop` lock 255です。`controller_server.use_realtime_priority` は権限に依存しないよう無効です。

## 最短の診断

```bash
ros2 topic hz /odom
ros2 topic hz /scan3
ros2 topic hz /imu
ros2 run tf2_ros tf2_echo robot/odom robot/base_footprint
ros2 run tf2_ros tf2_echo robot/base_footprint robot/lidar_link
```

Nav2ゴール中:

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 topic hz /cmd_vel_nav
ros2 topic hz /cmd_vel_smoothed
ros2 topic hz /cmd_vel
```

最初に止まった境界を調べます。Nav2モードで `map -> robot/odom` がない場合は `map_to_odom_ground_truth`、地図生成モードで同じTFがない場合は `/slam_toolbox` のLifecycleと `/scan3` を確認します。

## ファイル一覧

| パス | 役割 |
|---|---|
| `launch/sim.launch.py` | Gazebo、bridge、TF、VLP16変換、twist_mux |
| `launch/mapping.launch.py` | slam_toolboxのみ |
| `launch/navigation.launch.py` | 地図 + AMCL（または試験用固定TF）+ Nav2のみ |
| `launch/navigation_slam.launch.py` | slam_toolbox + Nav2（地図なし） |
| `models/robotbase.sdf` | 物理モデル、VLP16、IMU、DiffDrive plugin |
| `urdf/robotbase.urdf` | robot_state_publisher用の固定TFと表示モデル |
| `worlds/test_arena.sdf` | 外周壁と複数障害物を持つ試験場 |
| `maps/test_arena.yaml` | Nav2用の同梱地図 |
| `params/sim/nav2.yaml` | シミュレーション用Nav2設定 |
| `params/sim/slam_toolbox.yaml` | 2D SLAM設定 |
| `params/sim/twist_mux.yaml` | 手動/Nav2/停止の速度優先順位 |
| `params/sim/velodyne_laserscan.yaml` | VLP-16相当点群から複数リング合成 `/scan3` への変換 |
| `params/sim/idle_twist.yaml` | 停止時のゼロ速度発行 |
| `rviz/robotbase.rviz` | SLAM/Nav2共通RViz設定 |

## 現在の制約

- IMUは配信確認用で、現在のオドメトリには融合していません。
- VLP16の単一ringから作る2D scanのため、AMCLの長距離精度は実機設定と合わせて調整が必要です。
- VLP16はGPU lidarによる近似で、実機固有のpacket timingやdriver遅延は再現しません。
- 接触、スリップ、エンコーダ誤差は簡略化されています。
- GUI環境によってEGL警告が出ても、ヘッドレス実行とROSトピックが動作する場合があります。
