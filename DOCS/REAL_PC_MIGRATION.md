# 実機PCへの移行・初回試験

## 結論

コア機能のソフトウェアは、実機PCへ移してセットアップを開始できる状態。ただし、新構成による実機走行はまだ未確認なので、現時点の判定は「実験準備可能」であり「自律走行保証済み」ではない。

確認済み:

- GazeboでSLAM、既存地図Nav2、SLAM同時Nav2が動作
- Nav2の速度経路は `/cmd_vel_nav -> /cmd_vel_smoothed -> twist_mux -> /cmd_vel`
- `robotbase_keyop` はキー入力から `/cmd_vel_teleop` を出力
- `robotbase_bringup`、`robotbase_keyop`、`robotbase_waypoint`、`robotbase_sim` と主要依存パッケージが開発PCでビルド可能
- 実機用paramsは `params/real/`、シミュレーション用は `params/sim/`
- SIRIUS移植元の `src/sirius/` は `COLCON_IGNORE` によりビルド対象外

実機PCで未確認:

- Roboteqへの速度指令と新driver設定による左右車輪の向き・速度
- `/odom` と `/odom/filtered` の値、符号、周期
- IMUの実デバイス名、取付方向、角速度符号
- VLP-16のIP疎通、packet受信、`/scan3`、sensor TF
- 新しい実機Nav2 paramsでのゴール走行

## 0. Git転送前の必須確認

Gitは未コミットファイルを転送しない。開発PCで次を確認する。

```bash
cd ~/robotbase_ws_renya
git status --short
git diff --check
```

今回の変更には新規の `params/`、`src/robotbase_keyop/`、`src/robotbase_waypoint/`、`rviz/`、`maps_waypoints/` と、多数のlaunch/doc変更が含まれる。内容を確認してからcommit・pushする。

```bash
git add -A
git status --short
git commit -m "Prepare Koko robot bringup, simulation, and real-PC config"
git push origin master
```

実機PCでは必ず同じコミットを確認する。

```bash
git rev-parse --short HEAD
```

## 1. 対象環境

開発・確認環境:

- Ubuntu 24.04
- ROS 2 Jazzy
- RMW: `rmw_fastrtps_cpp`
- ワークスペース: `~/robotbase_ws_renya`

現在のBash補助スクリプトは `~/robotbase_ws_renya` を使用します。このPCへの適用結果は [環境構築状況](SETUP_RENYA.md) を参照してください。

```bash
cd ~
git clone git@github.com:YakiFrog/robotbase_ws.git robotbase_ws_renya
cd ~/robotbase_ws_renya
git checkout master
```

SSH鍵を実機PCへ設定していない場合は、GitHub認証を先に行う。

## 2. ROSと依存関係

ROS 2 Jazzy Desktopと開発ツールを導入した後に実行する。

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep \
  libqt5serialport5-dev \
  ros-jazzy-foxglove-bridge \
  xterm terminator

sudo rosdep init 2>/dev/null || true
rosdep update

cd ~/robotbase_ws_renya
source ~/robotbase_ws_renya/bash/activate_koko_env.sh
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

`ros-jazzy-foxglove-bridge` を含む上記依存を導入した後、`rosdep check --from-paths src --ignore-src` が成功することを確認する。このPCでは2026-10-01にFoxglove本体の導入済みと依存検査の成功を確認しました。

## 3. ビルド

実機とシミュレーションに必要なパッケージと、そのワークスペース内依存だけをビルドする。

```bash
cd ~/robotbase_ws_renya
source ~/robotbase_ws_renya/bash/activate_koko_env.sh
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --executor sequential \
  --packages-up-to robotbase_bringup robotbase_keyop robotbase_waypoint robotbase_sim \
  --allow-overriding nav2_costmap_2d
source install/setup.bash
```

確認:

```bash
for package in \
  robotbase_bringup robotbase_keyop robotbase_waypoint robotbase_sim \
  roboteq_ros2_driver witmotion_ros velodyne_driver foxglove_bridge; do
  ros2 pkg prefix "$package"
done
```

ワークスペースでビルドしたパッケージは `~/robotbase_ws_renya/install/` 配下を指すこと。システム導入の依存（例: `foxglove_bridge`）は `/opt/ros/jazzy/` 配下で構いません。

## 4. Bashとランチャー

`~/.bashrc` の末尾へ追加する。

```bash
source ~/robotbase_ws_renya/bash/bash_alias1.sh
source ~/robotbase_ws_renya/bash/bash_alias2.sh
```

反映と確認:

```bash
source ~/.bashrc
koko_env
echo "$ROS_DOMAIN_ID"           # 54
echo "$ROBOTBASE_TF_PREFIX"     # robot
echo "$ROBOTBASE_PARAMS_DIR"    # ~/robotbase_ws_renya/params
```

デスクトップランチャー:

```bash
~/robotbase_ws_renya/bash/install_launcher_shortcut.sh
```

機体名、ROS Domain、Gazebo partition、TF接頭辞は `robot.env` が正本。

## 5. Roboteq

実機params: `params/real/roboteq.yaml`

既定値:

- device: `/dev/roboteq`
- USB VID/PID: `20d2:5740`
- baud: `230400`
- wheel circumference: `0.6283 m`（直径200 mm）
- track width: `0.435 m`
- encoder pulse: `950`
- encoder sign: right `-1`、left `+1`

udev設定:

```bash
sudo install -m 0644 \
  ~/robotbase_ws_renya/src/roboteq_ros2_jazzy_driver/udev_rule/99-roboteq-serial.rules \
  /etc/udev/rules.d/99-roboteq-serial.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

対象機器のUSB VID/PIDが上記と一致することを確認してからruleを適用し、USBを挿し直して確認する。

```bash
ls -l /dev/roboteq
udevadm info -q property -n /dev/roboteq | sort
```

車体を浮かせた状態で最初の低速試験を行う。前進指令で両輪の実車前進方向が一致しなければ、配線と `encoder_sign_r/l` を記録してから調整する。

## 6. WitMotion IMU

実機params: `params/real/imu.yaml`

driverの `port` は `/dev/`を付けない名前を受け取る。現在値 `wt905` は `/dev/wt905` というudev symlinkが存在する前提。

開発PCにはIMU用udev ruleがなく、USB VID/PID/serialも未確認。実機PCで接続後に調べる。

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
udevadm info -a -n /dev/ttyUSB0 | less
```

一時試験では `params/real/imu.yaml` の `port` を実際の名前（例: `ttyUSB0`）へ変更できる。安定運用では実機固有のVID/PIDまたはserialを使って `/dev/wt905` symlinkを作る。ユーザーを `dialout` groupへ追加した場合は、ログアウト・ログインが必要。

```bash
sudo usermod -aG dialout "$USER"
```

## 7. Velodyne VLP-16

実機params: `params/real/velodyne.yaml`

既定値:

- sensor IP: `192.168.1.201`
- UDP port: `2368`
- model: `VLP16`
- rpm: `600`

実機PCのVLP-16接続NICへ、センサーと同一subnetの未使用アドレスを設定する。例は `192.168.1.100/24`。既存ネットワークと競合する場合は別の値を使う。

```bash
ip -brief address
ping -c 3 192.168.1.201
```

packetが来ない場合は、センサーの送信先IP、NIC、firewall、UDP 2368を確認する。

## 8. TFの実機確認

現在のURDFは次を仮定している。

- `robot/base_footprint -> robot/lidar_link`: `(x, y, z) = (0, 0, 0.72 m)`、回転なし
- `robot/base_footprint -> robot/imu_link`: `(0, 0, 0.28 m)`、回転なし
- 車輪半径: `0.10 m`
- 左右車輪中心間: `0.435 m`

センサーの実取付位置・向きが違う場合は、実験前に `src/robotbase_bringup/urdf/robotbase.urdf` を修正する。特にIMUのZ軸角速度符号とVLP-16の前方向を確認する。

## 9. 段階的な実機試験

最初からNav2を起動せず、各コマンドを別ターミナルで順に起動する。

### 9.1 driverとTF

```bash
koko_roboteq
koko_velodyne
koko_imu
koko_sf_real
koko_twist_mux
koko_rviz_real
```

確認:

```bash
ros2 topic hz /odom
ros2 topic hz /odom/filtered
ros2 topic hz /imu
ros2 topic hz /velodyne_points
ros2 topic hz /scan3
ros2 run tf2_ros tf2_echo robot/odom robot/base_footprint
ros2 run tf2_ros tf2_echo robot/base_footprint robot/lidar_link
ros2 run tf2_ros tf2_echo robot/base_footprint robot/imu_link
```

### 9.2 手動操作

車体を浮かせるか、即座に電源を切れる状態で行う。

```bash
koko_keyop2
```

`s`で速度ゼロ、`w/x`で直進、`a/d`で旋回、`q/e`で非常停止/解除。ウェイポイント記録中は`p`で現在地を10秒停止地点として保存する。前進時に残っている角速度を消すには、先に`s`を押してから`w`を押す。

### 9.3 実環境の地図

Gitに含まれる `koko-sim` はGazebo用で、実験場所の地図ではない。実環境では次を使う。

```bash
koko_slamtoolbox_real
koko_map_save
```

地図は `maps_waypoints/maps/<name>.yaml` と `.pgm` に保存される。

### 9.4 Nav2

保存済み地図とAMCLを使う場合:

```bash
koko_nav2_real
```

地図を選び、RVizで初期姿勢を合わせてから近距離ゴールを送る。ゴール中は別ターミナルで確認する。

地図を生成・更新しながらNav2を使う場合:

```bash
koko_nav2_real_slam
```

このモードではSLAM Toolboxが `/map` と `map -> robot/odom` を供給するため、map serverとAMCLは起動しない。`/map` が配信されてから近距離ゴールを送り、必要な地図は別ターミナルの `koko_map_save` で保存する。どちらのNav2モードでも、実機基本driver、EKF、`koko_twist_mux`、RVizは別途必要。

```bash
ros2 lifecycle get /controller_server
ros2 topic hz /cmd_vel_nav
ros2 topic hz /cmd_vel_smoothed
ros2 topic hz /cmd_vel
ros2 topic hz /odom/filtered
ros2 topic hz /scan3
```

速度が最初に途切れる境界とNav2ログを保存する。旧実機でパスだけ出て動かなかった最有力候補と詳細は `DOCS/NAV2_NO_MOTION.md` を参照。

## 10. SIRIUS由来でランチャーから除外した機能

次の旧aliasはSIRIUS由来の未移植コードを参照していたため、`bash/bash_alias2.sh` とランチャーから除外済み。

- `koko_scn`
- `koko_mv_goal`
- `koko_get_pos_dis`
- `koko_get_pos_ent`

必要なら後で `robotbase_*` パッケージへ切り出し、`robot/*` TFと `~/robotbase_ws_renya` の保存先へ変更する。SIRIUSソースを再度有効化して回避しない。

## 11. 次の担当者・AIへ渡す情報

最初に次を読ませる。

1. `README.md`
2. `DOCS/PROJECT_CONTEXT.md`
3. この `DOCS/REAL_PC_MIGRATION.md`
4. 問題がNav2無走行なら `DOCS/NAV2_NO_MOTION.md`

実機試験ごとに残す情報:

- `git rev-parse --short HEAD`
- `uname -a`、`lsb_release -a`、`printenv ROS_DISTRO RMW_IMPLEMENTATION`
- `ls -l /dev/roboteq /dev/wt905`
- `ip -brief address`
- 起動した `koko_*` コマンド一覧
- 各トピックの周波数とTF結果
- driver/Nav2端末ログ
- 使用地図名と初期姿勢
- 車輪を浮かせた試験か、接地試験か
