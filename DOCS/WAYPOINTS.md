# ウェイポイント記録・追従

## 概要

SIRIUSの`move_goal.py`と位置記録ツールを参考に、ココちゃん専用の`robotbase_waypoint`パッケージを追加した。実機・シミュレーションで同じYAMLを使い、複数地点をNav2の`NavigateToPose`へ順番に送る。

SIRIUSから引き継いだもの:

- `format_version: '1.0'`のウェイポイントYAML
- 開始番号、ループ、通常の中間点切替距離
- 地点ごとの`threshold`、`stop`、`wait_time`
- 現在位置の1点保存と、一定移動距離ごとの連続記録

ココちゃん向けに変更したもの:

- 固定の`sirius3/base_footprint`を廃止し、`robot.env`のTF接頭辞を使用
- SIRIUSの固定パスを廃止し、`robotbase_ws/maps_waypoints/waypoints/`へ集約
- LLM用`/nav_control`、地図切替shell、SIRIUS固有トピックを含めない
- 中間点で旧Nav2 goalのキャンセル完了を待ってから次を送る
- 停止時は`/cmd_vel_direct`へゼロ速度を送り安全に停止を維持する
- 失敗時は`stop_on_failure: false`（既定）でスキップして次のウェイポイントへ自動遷移する
- YAMLの値とファイル存在を起動時に検証し、同一goalの周期的な再送を行わない

`change_map`と`rotate`は未対応。地図は先に`koko_nav2_real`または`koko_nav2_sim_map`で選択する。

## 起動順

既存地図で使う場合、先に通常どおりセンサー・自己位置推定・twist_mux・Nav2を起動し、RVizで初期姿勢を設定する。その後、別端末でウェイポイント追従を開始する。

シミュレーション:

```bash
koko_sim
koko_rviz_sim
koko_nav2_sim_map
# RVizで2D Pose Estimateを設定
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
koko_nav2_real
# RVizで2D Pose Estimateを設定
koko_waypoint_follow
```

追従コマンドはYAML、開始番号、切替距離、ループ有無を順に質問する。選択するYAMLと、Nav2で選択した地図の座標系が一致している必要がある。

MPPIは後退より前進を強く優先する。最初の地点は、現在姿勢から見て前方に置く。後方から開始すると、到達まで大きく旋回する場合がある。

## 地点の記録

### SLAM地図とウェイポイントを同時に作る

SIRIUSの`get_position_distance.py`相当として、SLAM中の現在姿勢を一定距離ごとに記録し、終了時に地図とウェイポイントを同じ名前で保存できる。

先にSLAM Toolbox（またはSLAMとNav2の同時構成）とRVizを起動し、手動走行できる状態にする。

```bash
# 実機
koko_nav2_real_slam
koko_rviz_real
koko_keyop2
koko_slam_waypoint_record

# シミュレーション
koko_sim
koko_nav2_sim_slam
koko_rviz_sim
koko_keyop2_sim
koko_slam_waypoint_record_sim
```

記録名と距離間隔を入力して走行する。`Ctrl+C`を押すとウェイポイント記録を終了し、現在の`/map`も保存する。例えば名前を`1F-route`にすると次の3ファイルが作られる。

```text
maps_waypoints/maps/1F-route.yaml
maps_waypoints/maps/1F-route.pgm
maps_waypoints/waypoints/1F-route.yaml
```

GUIランチャーでは「地図＋ウェイポイント記録（実機）」または「地図＋ウェイポイント記録（シミュレーション）」プリセットを使える。

`map -> <prefix>/base_footprint`が得られない間は記録されない。SLAMとセンサーを先に正常起動する必要がある。

現在姿勢を1点追加する:

```bash
koko_waypoint_save       # 実機
koko_waypoint_save_sim   # シミュレーション
```

指定距離を移動するたびに記録し、`Ctrl+C`まで続ける:

```bash
koko_waypoint_record       # 実機
koko_waypoint_record_sim   # シミュレーション
```

距離記録中はキーオペ端末を選択したまま`p`キーを押せる。キーオペが
`/waypoint_record/pause`を通知し、記録ノードがその時点の姿勢へ`stop: true`と
`wait_time: 10.0`を設定する。追従時はその地点で10秒停止し、その後に次の
ウェイポイントへ自動的に進む。記録端末を選択して直接`p`を押す操作にも対応する。

同名ファイルがあれば末尾へ追記し、番号は既存の最終番号から続ける。保存中は`map -> <prefix>/base_footprint`が必要なため、AMCLまたはSLAM Toolboxを先に起動する。

## YAML形式

```yaml
format_version: '1.0'
waypoints:
- number: 1
  x: 1.0
  y: 0.0
  angle_radians: 0.0
  threshold: 0.5
- number: 2
  x: 2.0
  y: 1.0
  angle_radians: 1.571
  stop: true
  wait_time: 3.0
- number: 3
  x: 2.0
  y: 2.0
  angle_radians: 1.571
```

| キー | 必須 | 意味 |
|---|---|---|
| `number` | 省略可 | 表示上の地点番号。省略時はリスト順 |
| `x`, `y` | 必須 | `map`座標系の位置[m] |
| `angle_radians` | 必須 | 到着姿勢のyaw[rad] |
| `threshold` | 省略可 | この中間点から次へ切り替える距離[m] |
| `stop` | 省略可 | 地点で停止する。既定停止時間は5秒 |
| `wait_time` | 省略可 | 停止時間[秒]。`stop`より優先 |

通常の中間点は距離閾値で次の`NavigateToPose` goalを送り、現在のgoalをNav2内で更新する。この切替では明示的なキャンセルやゼロ速度指令を送らない。最終点はNav2が姿勢を含めて`SUCCEEDED`を返すまで待つ。`stop`または`wait_time`を持つ中間点は従来どおり停止し、`precise_threshold`（既定0.35 m）を使用する。

サンプルは`maps_waypoints/waypoints/koko-sim-example.yaml`にある。

## Nav2標準Waypoint Follower

`robotbase_bringup/launch/nav2.launch.py`はNav2標準の`waypoint_follower`もLifecycle管理下で起動する。実機・シミュレーションの設定はそれぞれ`params/real/nav2.yaml`と`params/sim/nav2.yaml`内の`waypoint_follower`に置く。RViz Navigation 2パネルなど、`/follow_waypoints` actionを使うクライアント向けである。

`koko_waypoint_follow*`はYAMLの距離閾値やループに対応する独自クライアントで、標準サーバーの`/follow_waypoints`ではなく`/navigate_to_pose`を使う。同時に別クライアントからナビゲーションgoalを送らない。

## 停止と異常終了

待機地点または追従失敗では、次の順で停止する。

```text
ゼロTwist -> /cmd_vel_direct -> twist_mux -> /cmd_vel
                                       ↓
                             /stop=trueで入力をロック
```

Gazeboや一部モータdriverは最後に受けた速度を保持するため、`/stop`だけでmux出力を無通信にすると停止しない可能性がある。このためゼロ指令を先に通す。追従端末を`Ctrl+C`で閉じると、現在goalをキャンセルしてゼロ指令を送り、スクリプトの終了処理が`/stop=false`を送る。

## 切り分け

```bash
ros2 action list | grep -E 'navigate_to_pose|follow_waypoints'
ros2 lifecycle get /waypoint_follower
ros2 run tf2_ros tf2_echo map robot/base_footprint
ros2 topic hz /cmd_vel_nav
ros2 topic hz /cmd_vel_smoothed
ros2 topic hz /cmd_vel
ros2 topic echo /stop
```

- `Waiting for /navigate_to_pose`: Nav2未起動、またはDomain違い
- TF unavailable: AMCL/SLAM未起動、初期姿勢未設定、TF接頭辞違い
- goal rejected/aborted: Nav2端末のplanner/controller/costmapエラーを確認
- goalは進むが車体が動かない: twist_mux、`/stop`、`/cmd_vel`以降を確認

## ビルドとテスト

```bash
cd ~/robotbase_ws
colcon build --symlink-install --packages-select robotbase_waypoint robotbase_bringup
source install/setup.bash
colcon test --packages-select robotbase_waypoint
colcon test-result --verbose
```
