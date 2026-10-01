# 設定の正本

## 正本一覧

現在使うロボット固有パラメータは、プロジェクト直下の `params/` に集約した。上流パッケージの `src/navigation2/`、`src/slam_toolbox/`、`src/velodyne/` にある既定ファイルは編集しない。

| 対象 | 正本 |
|---|---|
| 機体表示名・ROS/Gazebo分離・TF接頭辞 | `robot.env` |
| URDF | `src/robotbase_bringup/urdf/robotbase.urdf` |
| RViz | `src/robotbase_bringup/rviz/robotbase.rviz` |
| Nav2（実機） | `params/real/nav2.yaml` |
| Nav2（シミュレーション） | `params/sim/nav2.yaml` |
| SLAM Toolbox（実機 / シミュ） | `params/real/slam_toolbox.yaml` / `params/sim/slam_toolbox.yaml` |
| twist_mux（実機 / シミュ） | `params/real/twist_mux.yaml` / `params/sim/twist_mux.yaml` |
| EKF・Roboteq・IMU | `params/real/ekf.yaml`、`roboteq.yaml`、`imu.yaml` |
| Velodyne VLP-16 | `params/real/velodyne.yaml`、`VLP16db.yaml` |
| シミュレーション点群変換・停止速度 | `params/sim/velodyne_laserscan.yaml`、`idle_twist.yaml` |
| キーボード手動操作（実機） | `params/common/keyop.yaml` |
| キーボード手動操作（シミュレーション） | `params/sim/keyop.yaml` |
| Foxglove Bridge（共通） | `params/common/foxglove.yaml`、待受先は`robot.env` |
| Gazeboモデル | `src/robotbase_sim/models/robotbase.sdf` |
| 実機Nav2地図選択 | `bash/startup_bash/nav2_bringup_real.sh` |

RVizの配布用テンプレート `src/robotbase_bringup/rviz/robotbase.rviz` にはNav2標準のNavigation 2パネル、Global/Local Costmap、Global/Local Plan、Local Footprint表示を登録している。通常起動ではTF接頭辞ごとの `rviz/robotbase_<prefix>.rviz` を使い、RVizの通常のSave Configを次回起動へ引き継ぐ。

## 機体識別とTF

`robot.env` の既定値:

```bash
ROBOTBASE_DISPLAY_NAME="ココちゃん"
ROBOTBASE_ID="koko"
ROBOTBASE_ROS_DOMAIN_ID="57"
ROBOTBASE_SIM_ROS_DOMAIN_ID="58"
ROBOTBASE_GZ_PARTITION="koko"
ROBOTBASE_TF_PREFIX="robot"
ROBOTBASE_FOXGLOVE_ADDRESS="0.0.0.0"
ROBOTBASE_FOXGLOVE_PORT="8766"
```

alias経由では `ROBOTBASE_PARAMS_DIR=~/robotbase_ws/params` も設定される。直接launchした場合も同じ場所を既定値として使う。

表示名とTF接頭辞は独立している。TFは既定で次の構造になる。

```text
map
└─ robot/odom
   └─ robot/base_footprint
      ├─ robot/base_link
      ├─ robot/lidar_link
      └─ robot/imu_link
```

接頭辞を変える場合は `ROBOTBASE_TF_PREFIX` だけを変更し、全プロセスを再起動する。alias経由ではGazebo、URDF、各driver、EKF、SLAM、Nav2、RVizへ同じ値が渡る。

## Nav2

実機は `params/real/nav2.yaml`、シミュレータは `params/sim/nav2.yaml` を使う。両方を別々に調整でき、現在の主要差分は次のとおり。

| 項目 | シミュレーション | 実機 |
|---|---|---|
| `use_sim_time` | `true` | `false` |
| localization | AMCL（既定）、固定TF（試験用オプション） | AMCL |
| odometry | `/odom` | `/odom/filtered` |
| LaserScan | `/scan3` | AMCL/SLAM: `/scan3`、costmap: `/scan3` + `/hokuyo_scan` |

起動するNav2ノードはcontroller、planner、smoother、behavior、BT navigator、velocity smoother、Waypoint Follower、map server、AMCL。Route、Docking、Loopback Simulator、Collision Monitorは現在の用途から外した。Waypoint Followerは`params/{real,sim}/nav2.yaml`の同名セクションを使い、`/follow_waypoints`を提供する。シミュレーションでも既存地図モードの既定はAMCLなので、RVizの `2D Pose Estimate` が `/initialpose` を通して有効になる。経路制御だけを再現性優先で試す場合は `localization:=static` を明示できる。

設定の要点:

| 項目 | 値 |
|---|---:|
| controller | MPPI / DiffDrive |
| controller frequency | 10 Hz |
| prediction horizon | 2.8秒（`28 x 0.10`秒） |
| `use_realtime_priority` | `false` |
| 前進基準速度（上限） | 0.60 m/s |
| 最大後退速度 | -0.30 m/s |
| 最大角速度 | 0.90 rad/s |
| 実機の前後速度不感帯 | 0.10 m/s（MPPI評価・odom判定・velocity smoother） |
| footprint | 前0.425、後0.475、左右0.33 m（全長0.90 x 全幅0.66 m） |
| local costmap | 6 x 6 m、0.05 m/cell |
| obstacle source | `/scan3` + `/hokuyo_scan`（global/local共通、独立layer） |
| scan obstacle height | 0.0〜2.0 m（source単位で明示） |
| 後退 | `vx_min: -0.30`で脱出用に許可、`PreferForwardCritic`で通常走行は前進優先 |
| 衝突評価 | `CostCritic.consider_footprint: true`で長方形footprint全体を評価 |

ZED、SAM3、semantic layer、STVLは含まない。VLP-16由来の`/scan3`とEthernet Hokuyoの`/hokuyo_scan`をglobal/local costmapの独立layerで併用する。単一リング `/scan` はNav2とSLAM Toolboxの入力に使わない。

Nav2 JazzyのObstacleLayerは、plugin全体とは別に観測sourceごとの `max_obstacle_height` を持つ。global/localのVelodyne・Hokuyo全sourceで `2.0` mを明示する。

MPPIの予測時間は2.8秒とする。基準速度0.6 m/sでの予測移動量は1.68 mで、半径3 mのlocal costmap内に車体外形まで収まる。旧6秒・0.9 m/s設定は5.4 m先まで予測しようとしてcostmap外へ達し、空き領域で低速周回する原因になっていた。

MPPIには固定の巡航速度パラメータがないため、`vx_max`とvelocity smootherの前進上限を`0.60 m/s`にして、これを運用上の基準速度とする。`PathFollowCritic`を`cost_weight: 12.0`、`offset_from_furthest: 11`とし、0.05 m間隔の経路で約0.55 m先への進行を評価する。障害物衝突判定、footprint、inflation設定は変更しない。

実機は速度の周期的な脈動を抑えるため、探索分散を`vx_std: 0.25`、滑らかさとのトレードオフを`gamma: 0.050`、前進加速度上限をMPPI・velocity smootherとも`0.60 m/s^2`とする。さらに`visualize: false`と`regenerate_noises: false`で、候補軌道描画の負荷と毎周期のnoise再生成による計算揺れを除く。シミュレーションでは可視化・調整を優先するため、これらは従来値のままとする。

`PreferForwardCritic.cost_weight: 40.0`で、通常走行では後退より向き直って前進する軌道を強く優先する。`vx_min: -0.30`は維持しているため、前進軌道が成立しない場合やRecoveryのbackupでは後退できる。シミュレーションの後方ゴール試験で最大後退を約`-0.027 m/s`まで抑えた。

障害物は`CostCritic.cost_weight: 4.0`、`consider_footprint: true`、`collision_cost: 1000000.0`で評価する。旧weight 10はinflation領域へ入るコストが強すぎ、経路より空き領域の周回を選びやすかった。重みを下げても、長方形footprintが接触する軌道には100万コストが付くため、衝突禁止判定は維持される。

実機は前後速度の絶対値が`0.10 m/s`未満では動かない前提とする。実機用MPPIの`VelocityDeadbandCritic: 0.10 m/s`で微小速度の軌道へコストを付け、controllerのodom閾値とvelocity smootherのdeadbandも0.10 m/sにする。シミュレーションでも0.1 m/s付近で停止状態に留まる事例があったため、MPPI Criticを0.12 m/s、odom閾値とsmootherを0.10 m/s、Keyop2の直進刻みを0.20 m/sとする。角速度の不感帯は未計測なので、実機・シミュレーションとも0のままにしている。

## SLAM Toolbox

実機は `params/real/slam_toolbox.yaml`、シミュレータは `params/sim/slam_toolbox.yaml` を使う。入力は `/scan3`、出力TFは `map -> <prefix>/odom`。

```bash
koko_slamtoolbox_sim   # use_sim_time=true
koko_slamtoolbox_real  # use_sim_time=false
```

## 速度経路

```text
controller_server /cmd_vel_nav
  -> velocity_smoother /cmd_vel_smoothed
  -> twist_mux /cmd_vel
  -> roboteq_ros2_driver
```

twist_mux優先度:

| 入力 | priority |
|---|---:|
| `/stop` lock | 255 |
| `/cmd_vel_teleop` | 100 |
| `/cmd_vel_direct` | 90 |
| `/cmd_vel_smoothed` | 10 |
| `/cmd_vel_idle` | 1 |

Nav2とtwist_muxは別コマンドである。実機Nav2試験では `koko_twist_mux` の起動を確認する。

### Roboteq odometryの平滑化

Roboteq driverはposeをencoder受信時に積算し、`/odom`のtwistをsteady clock基準のencoder窓から推定する。推定窓は100 msを目標とし、75--150 msの範囲で確定するため、50 Hzのpublish周期の微小な揺れや、serial packetがまとめて届いたときの影響を受けにくい。32 bit encoderのwraparoundを処理し、最大回転数を超える不正なjumpは除外する。encoderを300 ms以上受信しない場合は速度を0へ戻す。

実機設定の`odom_stream_interval_ms: 10`と`odom_publish_hz: 50.0`はそのまま使用する。前者はencoder取得間隔、後者はROS topicの配信周期であり、100 msはtwist推定に使う観測窓である。

### 実機の指令速度校正

Roboteqの`speed_scale`は[実機用Roboteq設定](../params/real/roboteq.yaml)にあり、現在の作業値は`0.70`である。この値は`/cmd_vel`から左右モータ指令を作る際だけに乗り、encoderから`/odom`を計算する側には乗らない。実距離とodom距離が一致しているのに定常走行中のodom速度だけが指令より低い場合は、次式を目安に調整する。

`right_speed_scale`と`left_speed_scale`は、共通の`speed_scale`、低速底上げ、角加速度制限の計算後に各モータへ適用する。直進時に再現する左右差だけを補正する用途で、通常は片側を`1.0`に固定し、速い側を少し下げる。

```text
new_speed_scale = current_speed_scale * command_speed / steady_odom_speed
```

例として`0.50 m/s`指令に対して定常odomが`0.47 m/s`なら、`0.70 * 0.50 / 0.47 = 0.745`が計算値になる。一度に大きく変えず、まず`0.01--0.02`刻みで試す。driverはパラメータを1秒ごとに再取得するため、一時試験はビルドも再起動も不要である。

```bash
ros2 param set /roboteq_ros2_driver speed_scale 0.72
```

次回起動にも残す場合は`params/real/roboteq.yaml`の`speed_scale`を同じ値へ変更する。YAMLだけの変更なのでビルドは不要だが、既に起動中のdriverへYAML変更だけを反映するには再起動が必要である。なお、床上の実距離とodom距離自体が一致しない場合は`speed_scale`ではなく、`wheel_circumference`、`pulse`、`gear_ratio`を先に校正する。

## 削除した旧設定

次はSIRIUS由来で、現在の構成では不要なため削除した。

- `params/nav2_params*.yaml` と各bak
- `params/mapper_params_online_async*.yaml`
- `params/ekf_fusion.yaml`、`imu_filter.yaml`、`keepout_params.yaml`
- ZED/SAM3/RTAB-MAP起動スクリプト
- semantic/STVL、Hokuyo、docking、loopbackのパラメータ
- 旧MPPI走行モード切替スクリプト（現在は標準設定がMPPI）

`src/robotbase_bringup/config/` の複製も廃止し、`params/` だけを正本にした。実機とシミュレーションの差分確認例:

```bash
diff -u params/real/nav2.yaml params/sim/nav2.yaml
diff -u params/real/slam_toolbox.yaml params/sim/slam_toolbox.yaml
```

## 変更後チェック

```bash
source ~/robotbase_ws/install/setup.bash
ros2 pkg prefix robotbase_bringup
ros2 pkg prefix robotbase_sim
ros2 launch robotbase_bringup nav2.launch.py --show-args
ros2 launch robotbase_bringup slam.launch.py --show-args
```

実機寸法を変えた場合はURDF、SDF、Roboteqの車輪値、Nav2 footprintを同時に更新する。センサー取付位置を変えた場合はURDFとSDFを同時に更新する。
