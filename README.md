# robotbase_ws_renya

SIRIUS用の [`sirius_jazzy_ws`](../sirius_jazzy_ws) を基に、新しい差動二輪ロボット向けに調整している ROS 2 Jazzy ワークスペースです。車輪径、トレッド幅、エンコーダ、車体寸法、センサー構成がSIRIUSと異なります。

新機体の仮称は「ココちゃん」です。表示名、ROS Domain、Gazebo識別子は [`robot.env`](robot.env) で変更できます。この開発PC上のSIRIUSと競合しないよう、Bashコマンドは `koko_*`、ROSは実機Domain 54・シミュレーションDomain 67、Gazeboはpartition `koko`へ分離しています。

Foxglove Bridgeは実機では `koko_foxglove`、シミュレーションでは `koko_foxglove_sim` から起動できます。Siriusの既定ポート8765と競合しないよう、ココちゃんは8766を使います。

## このPCへの切り替え（2026-10-01）

`robotbase_ws_renya` を使用します。59パッケージのビルド、ランチャーテスト4件、デスクトップ／アプリメニューのショートカットと `.bashrc` の切り替えは完了しました。ランチャーの子端末は、このワークスペースのaliasを明示的に読み込みます。旧 `robotbase_ws` とSiriusの検索パスは `koko_env` で除去します。

今回の実機接続・走行とGazebo起動は未確認です。接続設定と確認結果は [環境構築状況](DOCS/SETUP_RENYA.md) を参照してください。

## 移植元での確認履歴（2026-08-19）

以下は移植元の履歴です。今回のPCでの実機・Gazebo動作確認を意味しません。

- 実機の手動操作: 動作確認済み
- `keyop2`: 動作確認済み
- Nav2による実機走行: 動作確認済み。SLAMで地図を更新しながらの自律移動も確認済み
- 実機Nav2の残作業: 障害物余裕、MPPIの滑らかさ、速度校正を継続調整中
- Gazeboシミュレータ: 構築・動作確認済み
- シミュレータでの2D地図生成: 動作確認済み
- シミュレータでのNav2自律移動: 分離launch/シミュレーション専用params/`robot/*` TFで動作確認済み（障害物迂回ゴール成功）
- ウェイポイント: SIRIUS互換YAMLの記録・追従とNav2標準`/follow_waypoints`サーバーを追加
- 原因: 実機トピックを記録していないため未確定。ただし、コード上の優先度は次のとおり

1. 旧設定の `controller_server.use_realtime_priority: true` に対して、実機PCのRT優先度権限が未設定
2. `twist_mux` の起動漏れ、または `/cmd_vel_nav` から `/cmd_vel` までの途中停止
3. `/odom/filtered`、`/scan3`、`/hokuyo_scan`、TF、ローカルコストマップのいずれかが未更新
4. `keyop2` または `/stop` が `twist_mux` のNav2入力より高い優先度を保持

最有力だった1に対し、新しい実機設定では `use_realtime_priority: false` に変更し、その後の実機自律移動は成功しています。ただし当時のログがないため、最初に動かなかった直接原因がRT権限だったか、twist_mux等の起動状態だったかは確定していません。

詳細と実機での判定手順は [Nav2無走行の切り分け](DOCS/NAV2_NO_MOTION.md) を参照してください。

## 実機なしのGazebo実験

`robotbase_sim` パッケージに、差動二輪・Velodyne VLP-16・IMUだけを持つ簡易 Gazebo Sim 環境を追加しています。TFは既定で `robot/*` を使います。

```bash
cd ~/robotbase_ws_renya
source install/setup.bash

# 端末1: Gazeboのみ
koko_sim
# 端末2: RVizのみ
koko_rviz_sim
# 端末3: 用途に応じて1つ
koko_slamtoolbox_sim
koko_nav2_sim_map
koko_nav2_sim_slam

# 必要な場合は別端末
koko_keyop2_sim
koko_map_save_sim
koko_waypoint_follow_sim
koko_waypoint_save_sim
koko_waypoint_record_sim
```

`koko_nav2_sim_map` は同梱・保存済み地図を端末の一覧から選択し、AMCLを起動します。RVizの `2D Pose Estimate` で初期姿勢を指定できます。`koko_nav2_sim_slam` は地図なしでSLAM ToolboxとNav2を同時に使います。GazeboとRVizはどちらのモードでも別起動です。`twist_mux` は `koko_sim` に含まれるため、シミュレーションで別起動する必要はありません。

実機ROS graphはDomain 54、シミュレーションは既定Domain 67に分離しています。ランチャーの「シミュレーション ROS_DOMAIN_ID」で0〜232を選んで保存でき、`robot.env`へ永続化されます。`koko_*_sim`コマンドは保存値を自動的に使うため、実機や別PCの`/clock`がRVizへ混入しません。Gazeboを再起動するときは、ランチャー上部の赤い「シミュレーション一式を終了」ボタンで旧Gazebo・RViz・Nav2・SLAMを終了してください。`koko_sim`は二重起動や残存ノードを検出すると、安全のため起動を拒否します。

## 速度指令の経路

```text
Nav2 controller_server
  └─ /cmd_vel_nav
      └─ velocity_smoother
          └─ /cmd_vel_smoothed
              └─ twist_mux
                  └─ /cmd_vel
                      └─ roboteq_ros2_driver
```

この主経路のトピック名はコード上は整合しています。ただし `twist_mux` は `nav2_real` から自動起動されません。別ターミナルまたはランチャーでの起動が必須です。

`collision_monitor` は未接続の設定を残さず、現在の最小Nav2 launchから外しています。`/scan3` はcostmap回避に使われますが、独立した最終停止レイヤーは未実装です。

## 実機の主な設定

| 項目 | 現在値 |
|---|---:|
| 駆動方式 | 差動二輪 (`DiffDrive`) |
| 車輪直径 / 円周 | 200 mm / 0.6283 m |
| トレッド幅 | 0.435 m |
| エンコーダ | 950 pulse/車輪1回転 |
| Roboteq odom配信 | 50 Hz（速度は約100 msのencoder窓で推定） |
| Nav2フットプリント | 前後0.30 m、左右0.29 m（全長0.60 x 全幅0.58 m） |
| Nav2前進基準（上限） | 0.60 m/s |
| Nav2最大角速度 | 0.65 rad/s |
| 実機の前後速度不感帯 | 0.10 m/s（MPPI・odom判定・速度平滑化へ反映） |
| Nav2 controller | MPPI (`DiffDrive`、2.8秒予測、footprint全体を衝突評価) |
| 後退方針 | 後退可能、MPPI `PreferForwardCritic`で前進を優先 |
| Nav2 costmap入力LaserScan | `/scan3`、`/hokuyo_scan` |
| 制御用オドメトリ | `/odom/filtered`（車輪前進速度＋IMUヨー角速度のEKF融合） |
| 標準回復動作 | Spinなし（コストマップ消去・待機・後退）。Spinサーバーの登録は残る |

値の出典とSIRIUSとの差分は [構成資料](DOCS/ARCHITECTURE.md) にまとめています。

## 実機起動の基本順序

`.bashrc` から次を読み込んでいる前提です。

```bash
source ~/robotbase_ws_renya/bash/bash_alias1.sh
source ~/robotbase_ws_renya/bash/bash_alias2.sh
```

各コマンドは別ターミナルで起動します。

```bash
koko_roboteq       # Roboteq + ココちゃんURDF
koko_velodyne      # VLP-16。/velodyne_points、/scan、複数リング合成/scan3
koko_hokuyo        # Ethernet接続の足元Hokuyo。/hokuyo_scan（global/local costmap用）
koko_imu           # IMU
koko_sf_real       # /odom + /imu -> /odom/filtered、odom TF
koko_twist_mux     # Nav2/keyop2の速度指令を /cmd_vel に統合
koko_nav2_real     # 地図を選択してNav2のみ起動
koko_nav2_real_slam # 地図を生成・更新しながらNav2を起動
koko_rviz_real     # RVizのみ
koko_keyop2        # 必要な場合のみ
koko_waypoint_follow # YAMLを選択して連続自律移動
koko_waypoint_save   # 現在位置を1点保存
koko_waypoint_record # 移動距離ごとに連続保存
koko_foxglove      # 必要な場合のみ。Foxglove WebSocketサーバー
```

`koko_nav2_real_slam` はmap serverとAMCLを起動せず、SLAM Toolboxが `/map` と `map -> robot/odom` を供給します。地図を残す場合は走行後に `koko_map_save` を実行します。

地図とwaypointを同時に保存する場合は、SLAM起動後に別ターミナルで `koko_slam_waypoint_record`（シミュレーションでは `koko_slam_waypoint_record_sim`）を実行します。記録方法に `enter` を選ぶと、このターミナルでEnterを押すたびに現在位置をwaypointへ追記します。`distance` を選ぶと移動距離ごとに自動記録します。Ctrl+Cで記録を終えると、同じ名前で現在の地図を保存します。

ランチャーの「実機基本」プリセットはRoboteq、VLP-16、Hokuyo、IMU、EKF、twist_muxを起動します。「SLAMしながら自律移動（実機）」プリセットは、それらに `koko_nav2_real_slam` とRVizを加えてまとめて起動します。

## 最短の実機診断

Nav2ゴール送信中に、別ターミナルで上から順に確認します。

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /velocity_smoother
ros2 node list | grep twist_mux

ros2 topic hz /cmd_vel_nav
ros2 topic hz /cmd_vel_smoothed
ros2 topic hz /cmd_vel

ros2 topic hz /odom/filtered
ros2 topic hz /scan3
ros2 topic hz /hokuyo_scan
ros2 run tf2_ros tf2_echo robot/odom robot/base_footprint
```

最初に速度が途切れた箇所が故障境界です。`/cmd_vel_nav` が出ない場合はNav2端末で次の文字列を探してください。

```text
Cannot set as real-time thread
No valid trajectories
Timed out waiting for transform
Failed to make progress
```

## ディレクトリ案内

| パス | 役割 |
|---|---|
| `params/real/` | 実機用Nav2・SLAM・twist_mux・Roboteq・IMU・EKF・Velodyne設定 |
| `params/sim/` | シミュレーション用Nav2・SLAM・twist_mux・点群変換設定 |
| `params/common/` | 実機・シミュレーション共通のキーボード操作・Foxglove設定 |
| `src/robotbase_bringup/` | ココちゃん専用URDF、RViz、実機起動launch |
| `src/robotbase_sim/` | Gazebo SimとVLP-16/IMUの模擬センサー |
| `src/robotbase_keyop/` | 実機・シミュレーション共通のキーボード手動操作 |
| `src/robotbase_waypoint/` | SIRIUS互換YAMLのウェイポイント記録・Nav2追従 |
| `rviz/` | Save Configを次回起動へ引き継ぐTF接頭辞別RViz設定 |
| `src/sirius/` | 移植元コード。現在の主要launch/paramsの正本ではない |
| `src/roboteq_ros2_jazzy_driver/` | モータ指令とホイールオドメトリ |
| `src/navigation2/` | ワークスペース内でビルドするNav2本体 |
| `src/slam_toolbox/` | 2D SLAM |
| `src/velodyne/` | Velodyneドライバ |
| `src/witmotion_ros/` | IMUドライバ |
| `bash/` | エイリアスと起動補助スクリプト |
| `other_programs/sirius_launcher/` | エイリアスをGUIボタン化するランチャー |
| `DOCS/` | 本プロジェクト固有の詳細資料と調査記録 |

## ドキュメント

- [DOCS索引](DOCS/README.md)
- [短いプロジェクトコンテキスト](DOCS/PROJECT_CONTEXT.md) — 人・AIとも最初に読む
- [実機PCへの移行・初回試験](DOCS/REAL_PC_MIGRATION.md)
- [構成、データフロー、SIRIUSとの差分](DOCS/ARCHITECTURE.md)
- [Nav2無走行の原因候補と切り分け](DOCS/NAV2_NO_MOTION.md)
- [設定の正本と既知の課題](DOCS/CONFIGURATION.md)
- [Gazeboシミュレータの構成と使い方](DOCS/SIMULATION.md)
- [ココちゃんBash・ランチャー・通信分離](DOCS/LAUNCHER.md)
- [Foxglove Bridgeの接続と設定](DOCS/FOXGLOVE.md)
- [SLAM Toolbox設定とSIRIUSとの差分](DOCS/SLAM_TOOLBOX.md)
- [ウェイポイントの記録・追従](DOCS/WAYPOINTS.md)

## ビルド

```bash
cd ~/robotbase_ws_renya
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -riy
colcon build --symlink-install --executor sequential --allow-overriding nav2_costmap_2d
source install/setup.bash
```

このチェックアウトでは `build/`、`install/`、`log/` はGit管理対象外です。設定変更後は必ず再ビルドし、起動ターミナルで `install/setup.bash` を読み直してください。
